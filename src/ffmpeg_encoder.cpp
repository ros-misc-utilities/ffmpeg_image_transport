// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ffmpeg_image_transport/ffmpeg_encoder.hpp"

#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ffmpeg_image_transport/safe_param.hpp"

namespace ffmpeg_image_transport
{
FFMPEGEncoder::FFMPEGEncoder() : logger_(rclcpp::get_logger("FFMPEGEncoder")) {}

FFMPEGEncoder::~FFMPEGEncoder()
{
  Lock lock(mutex_);
  closeCodec();
}

void FFMPEGEncoder::reset()
{
  Lock lock(mutex_);
  closeCodec();
}

void FFMPEGEncoder::closeCodec()
{
  if (codecContext_) {
    avcodec_close(codecContext_);
    codecContext_ = NULL;
  }
  if (frame_) {
    av_free(frame_);
    frame_ = 0;
  }
  if (packet_) {
    av_packet_free(&packet_);  // also unreferences the packet
    packet_ = nullptr;
  }
}

void FFMPEGEncoder::setParameters(rclcpp::Node * node)
{
  Lock lock(mutex_);
  const std::string ns = "ffmpeg_image_transport.";
  codecName_ = get_safe_param<std::string>(node, ns + "encoding", "libx264");
  profile_ = get_safe_param<std::string>(node, ns + "profile", "main");
  preset_ = get_safe_param<std::string>(node, ns + "preset", "slow");
  qmax_ = get_safe_param<int>(node, ns + "qmax", 10);
  bitRate_ = get_safe_param<int64_t>(node, ns + "bit_rate", 8242880);
  GOPSize_ = get_safe_param<int64_t>(node, ns + "gop_size", 15);
  RCLCPP_INFO_STREAM(
    logger_, "enc: " << codecName_ << " prof: " << profile_ << " preset: " << preset_);
  RCLCPP_INFO_STREAM(
    logger_, "qmax: " << qmax_ << " bitrate: " << bitRate_ << " gop: " << GOPSize_);
}

bool FFMPEGEncoder::initialize(int width, int height, Callback callback)
{
  Lock lock(mutex_);
  callback_ = callback;
  return (openCodec(width, height));
}

bool FFMPEGEncoder::openCodec(int width, int height)
{
  codecContext_ = NULL;
  try {
    if (codecName_.empty()) {
      throw(std::runtime_error("no codec set!"));
    }
    if ((width % 32) != 0) {
      RCLCPP_WARN(logger_, "horiz res must be multiple of 32!");
    }
    if (codecName_ == "h264_nvmpi" && ((width % 64) != 0)) {
      RCLCPP_WARN(logger_, "horiz res must be multiple of 64!");
      throw(std::runtime_error("h264_nvmpi must have horiz rez mult of 64"));
    }
    // find codec
    const AVCodec * codec = avcodec_find_encoder_by_name(codecName_.c_str());
    if (!codec) {
      throw(std::runtime_error("cannot find codec: " + codecName_));
    }
    // allocate codec context
    codecContext_ = avcodec_alloc_context3(codec);
    if (!codecContext_) {
      throw(std::runtime_error("cannot allocate codec context!"));
    }
    codecContext_->bit_rate = bitRate_;
    codecContext_->qmax = qmax_;  // 0: highest, 63: worst quality bound
    codecContext_->width = width;
    codecContext_->height = height;
    codecContext_->time_base = timeBase_;
    codecContext_->framerate = frameRate_;

    // gop size is number of frames between keyframes
    // small gop -> higher bandwidth, lower cpu consumption
    codecContext_->gop_size = GOPSize_;
    // number of bidirectional frames (per group?).
    // NVenc can only handle zero!
    codecContext_->max_b_frames = 0;

    // encoded pixel format. Must be supported by encoder
    // check with e.g.: ffmpeg -h encoder=h264_nvenc -pix_fmts

    codecContext_->pix_fmt = pixFormat_;

    if (
      av_opt_set(codecContext_->priv_data, "profile", profile_.c_str(), AV_OPT_SEARCH_CHILDREN) !=
      0) {
      RCLCPP_ERROR_STREAM(logger_, "cannot set profile: " << profile_);
    }

    if (
      av_opt_set(codecContext_->priv_data, "preset", preset_.c_str(), AV_OPT_SEARCH_CHILDREN) !=
      0) {
      RCLCPP_ERROR_STREAM(logger_, "cannot set preset: " << preset_);
    }
    RCLCPP_DEBUG(
      logger_,
      "codec: %10s, profile: %10s, preset: %10s,"
      " bit_rate: %10ld qmax: %2d",
      codecName_.c_str(), profile_.c_str(), preset_.c_str(), bitRate_, qmax_);
    /* other optimization options for nvenc
         if (av_opt_set_int(codecContext_->priv_data, "surfaces",
         0, AV_OPT_SEARCH_CHILDREN) != 0) {
         RCLCPP_ERROR_STREAM(logger_, "cannot set surfaces!");
         }
      */
    if (avcodec_open2(codecContext_, codec, NULL) < 0) {
      throw(std::runtime_error("cannot open codec!"));
    }
    RCLCPP_DEBUG_STREAM(logger_, "opened codec: " << codecName_);
    frame_ = av_frame_alloc();
    if (!frame_) {
      throw(std::runtime_error("cannot alloc frame!"));
    }
    frame_->width = width;
    frame_->height = height;
    frame_->format = codecContext_->pix_fmt;
    // allocate image for frame
    if (
      av_image_alloc(
        frame_->data, frame_->linesize, width, height, static_cast<AVPixelFormat>(frame_->format),
        64) < 0) {
      throw(std::runtime_error("cannot alloc image!"));
    }
    // Initialize packet
    packet_ = av_packet_alloc();
    packet_->data = NULL;
    packet_->size = 0;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
    if (codecContext_) {
      avcodec_close(codecContext_);
      codecContext_ = NULL;
    }
    if (frame_) {
      av_free(frame_);
      frame_ = 0;
    }
    return (false);
  }
  RCLCPP_DEBUG_STREAM(
    logger_, "intialized codec " << codecName_ << " for image: " << width << "x" << height);
  return (true);
}

void FFMPEGEncoder::encodeImage(const Image & msg)
{
  rclcpp::Time t0;
  if (measurePerformance_) {
    t0 = rclcpp::Clock().now();
  }
  cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  encodeImage(img, msg.header, t0);
  if (measurePerformance_) {
    const auto t1 = rclcpp::Clock().now();
    tdiffDebayer_.update((t1 - t0).seconds());
  }
}

void strided_copy(
  uint8_t * dest, const int stride_dest, const uint8_t * src, const int stride_src, const int n,
  const int length)
{
  if ((stride_dest == stride_src) && (stride_src == length)) {
    memcpy(dest, src, n * length);
  } else {
    for (int ii = 0; ii < n; ii++) {
      memcpy(dest + stride_dest * ii, src + stride_src * ii, length);
    }
  }
}

void FFMPEGEncoder::encodeImage(const cv::Mat & img, const Header & header, const rclcpp::Time & t0)
{
  Lock lock(mutex_);
  rclcpp::Time t1, t2, t3;
  if (measurePerformance_) {
    frameCnt_++;
    t1 = rclcpp::Clock().now();
    totalInBytes_ += img.cols * img.rows;  // raw size!
  }

  const int width = img.cols;
  const int height = img.rows;
  const AVPixelFormat targetFmt = codecContext_->pix_fmt;
  if (targetFmt == AV_PIX_FMT_BGR24) {
    const uint8_t * p = img.data;
    strided_copy(frame_->data[0], frame_->linesize[0], p, width * 3, height, width * 3);
  } else if (targetFmt == AV_PIX_FMT_YUV420P) {
    cv::Mat yuv;
    cv::cvtColor(img, yuv, cv::COLOR_BGR2YUV_I420);
    const uint8_t * p = yuv.data;
    // Y
    strided_copy(frame_->data[0], frame_->linesize[0], p, width, height, width);
    // U
    strided_copy(
      frame_->data[1], frame_->linesize[1], p + width * height, width / 2, height / 2, width / 2);
    // V
    strided_copy(
      frame_->data[2], frame_->linesize[2], p + width * height + width / 2 * height / 2, width / 2,
      height / 2, width / 2);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "cannot convert format bgr8 -> " << (int)codecContext_->pix_fmt);
    return;
  }
  if (measurePerformance_) {
    t2 = rclcpp::Clock().now();
    tdiffFrameCopy_.update((t2 - t1).seconds());
  }

  frame_->pts = pts_++;  //
  ptsToStamp_.insert(PTSMap::value_type(frame_->pts, header.stamp));

  int ret = avcodec_send_frame(codecContext_, frame_);
  if (measurePerformance_) {
    t3 = rclcpp::Clock().now();
    tdiffSendFrame_.update((t3 - t2).seconds());
  }
  // now drain all packets
  while (ret == 0) {
    ret = drainPacket(header, width, height);
  }
  if (measurePerformance_) {
    const rclcpp::Time t4 = rclcpp::Clock().now();
    tdiffTotal_.update((t4 - t0).seconds());
  }
}

int FFMPEGEncoder::drainPacket(const Header & header, int width, int height)
{
  rclcpp::Time t0, t1, t2;
  if (measurePerformance_) {
    t0 = rclcpp::Clock().now();
  }
  int ret = avcodec_receive_packet(codecContext_, packet_);
  if (measurePerformance_) {
    t1 = rclcpp::Clock().now();
    tdiffReceivePacket_.update((t1 - t0).seconds());
  }
  const AVPacket & pk = *packet_;
  if (ret == 0 && pk.size > 0) {
    FFMPEGPacket * packet = new FFMPEGPacket;
    FFMPEGPacketConstPtr pptr(packet);
    packet->data.resize(pk.size);
    packet->width = width;
    packet->height = height;
    packet->pts = pk.pts;
    packet->flags = pk.flags;
    memcpy(&(packet->data[0]), pk.data, pk.size);
    if (measurePerformance_) {
      t2 = rclcpp::Clock().now();
      totalOutBytes_ += pk.size;
      tdiffCopyOut_.update((t2 - t1).seconds());
    }
    packet->header = header;
    auto it = ptsToStamp_.find(pk.pts);
    if (it != ptsToStamp_.end()) {
      packet->header.stamp = it->second;
      packet->encoding = codecName_;
      callback_(pptr);  // deliver packet callback
      if (measurePerformance_) {
        const auto t3 = rclcpp::Clock().now();
        tdiffPublish_.update((t3 - t2).seconds());
      }
      ptsToStamp_.erase(it);
    } else {
      RCLCPP_ERROR_STREAM(logger_, "pts " << pk.pts << " has no time stamp!");
    }
    av_packet_unref(packet_);  // free packet allocated by encoder
  }
  return (ret);
}

void FFMPEGEncoder::printTimers(const std::string & prefix) const
{
  Lock lock(mutex_);
  RCLCPP_INFO_STREAM(
    logger_, prefix << " pktsz: " << totalOutBytes_ / frameCnt_ << " compr: "
                    << totalInBytes_ / (double)totalOutBytes_ << " debay: " << tdiffDebayer_
                    << " fmcp: " << tdiffFrameCopy_ << " send: " << tdiffSendFrame_
                    << " recv: " << tdiffReceivePacket_ << " cout: " << tdiffCopyOut_
                    << " publ: " << tdiffPublish_ << " tot: " << tdiffTotal_);
}
void FFMPEGEncoder::resetTimers()
{
  Lock lock(mutex_);
  tdiffDebayer_.reset();
  tdiffFrameCopy_.reset();
  tdiffSendFrame_.reset();
  tdiffReceivePacket_.reset();
  tdiffCopyOut_.reset();
  tdiffPublish_.reset();
  tdiffTotal_.reset();
  frameCnt_ = 0;
  totalOutBytes_ = 0;
  totalInBytes_ = 0;
}
}  // namespace ffmpeg_image_transport
