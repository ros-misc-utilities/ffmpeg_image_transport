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
#include <string>

#include "ffmpeg_image_transport/safe_param.hpp"
#include "ffmpeg_image_transport/utils.hpp"

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

static void free_frame(AVFrame ** frame)
{
  if (*frame) {
    av_free(*frame);
    *frame = nullptr;
  }
}

void FFMPEGEncoder::closeCodec()
{
  if (codecContext_) {
    avcodec_close(codecContext_);
    codecContext_ = nullptr;
  }
  free_frame(&frame_);
  free_frame(&hw_frame_);
  free_frame(&wrapperFrame_);

  if (packet_) {
    av_packet_free(&packet_);  // also unreferences the packet
    packet_ = nullptr;
  }
  if (hwDeviceContext_) {
    av_buffer_unref(&hwDeviceContext_);
  }
  if (swsContext_) {
    sws_freeContext(swsContext_);
    swsContext_ = NULL;
  }
}

AVPixelFormat FFMPEGEncoder::pixelFormat(const std::string & f) const
{
  if (f.empty()) {
    return (AV_PIX_FMT_NONE);
  }
  const auto fmt = av_get_pix_fmt(f.c_str());
  if (fmt == AV_PIX_FMT_NONE) {
    RCLCPP_ERROR_STREAM(logger_, "ignoring unknown pixel format: " << f);
  }
  return (fmt);
}

void FFMPEGEncoder::setParameters(rclcpp::Node * node)
{
  Lock lock(mutex_);
  const std::string ns = "ffmpeg_image_transport.";
  codecName_ = get_safe_param<std::string>(node, ns + "encoding", "libx264");
  profile_ = get_safe_param<std::string>(node, ns + "profile", "");
  preset_ = get_safe_param<std::string>(node, ns + "preset", "");
  tune_ = get_safe_param<std::string>(node, ns + "tune", "");
  delay_ = get_safe_param<std::string>(node, ns + "delay", "");
  qmax_ = get_safe_param<int>(node, ns + "qmax", 10);
  bitRate_ = get_safe_param<int64_t>(node, ns + "bit_rate", 8242880);
  GOPSize_ = get_safe_param<int64_t>(node, ns + "gop_size", 15);
  pixFormat_ = pixelFormat(get_safe_param<std::string>(node, ns + "pixel_format", ""));
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

void FFMPEGEncoder::openVAAPIDevice(const AVCodec * codec, int width, int height)
{
  int err = 0;
  err = av_hwdevice_ctx_create(&hwDeviceContext_, AV_HWDEVICE_TYPE_VAAPI, NULL, NULL, 0);
  utils::check_for_err("cannot create hw device context", err);
  AVBufferRef * hw_frames_ref = av_hwframe_ctx_alloc(hwDeviceContext_);
  if (!hw_frames_ref) {
    throw std::runtime_error("cannot allocate hw device!");
  }

  AVHWFramesContext * frames_ctx = reinterpret_cast<AVHWFramesContext *>(hw_frames_ref->data);
  frames_ctx->format = utils::find_hw_config(&usesHardwareFrames_, AV_HWDEVICE_TYPE_VAAPI, codec);

  if (usesHardwareFrames_) {
    const auto fmts = utils::get_hwframe_transfer_formats(hw_frames_ref);
    frames_ctx->sw_format = utils::get_preferred_pixel_format("h264_vaapi", fmts);
    if (pixFormat_ != AV_PIX_FMT_NONE) {
      RCLCPP_INFO_STREAM(
        logger_, "user overriding software pix fmt " << utils::pix(frames_ctx->sw_format));
      RCLCPP_INFO_STREAM(logger_, "with " << utils::pix(pixFormat_));
      frames_ctx->sw_format = pixFormat_;  // override default at your own risk!
    } else {
      RCLCPP_INFO_STREAM(
        logger_, "using software pixel format: " << utils::pix(frames_ctx->sw_format));
    }
    if (frames_ctx->sw_format == AV_PIX_FMT_NONE) {
      av_buffer_unref(&hw_frames_ref);
      throw(std::runtime_error("cannot find valid sw pixel format!"));
    }
  }

  frames_ctx->width = width;
  frames_ctx->height = height;
  frames_ctx->initial_pool_size = 20;
  if ((err = av_hwframe_ctx_init(hw_frames_ref)) < 0) {
    av_buffer_unref(&hw_frames_ref);
    utils::throw_err("failed to initialize VAAPI frame context", err);
  }
  codecContext_->hw_frames_ctx = av_buffer_ref(hw_frames_ref);

  av_buffer_unref(&hw_frames_ref);

  if (codecContext_->hw_frames_ctx == nullptr) {
    throw(std::runtime_error("vaapi: cannot create buffer ref!"));
  }
}

bool FFMPEGEncoder::openCodec(int width, int height)
{
  try {
    doOpenCodec(width, height);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
    closeCodec();
    return (false);
  }
  RCLCPP_DEBUG_STREAM(
    logger_, "intialized codec " << codecName_ << " for image: " << width << "x" << height);
  return (true);
}

void FFMPEGEncoder::doOpenCodec(int width, int height)
{
  int err = 0;
  codecContext_ = nullptr;
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
    throw(std::runtime_error("cannot find encoder: " + codecName_));
  }

  auto pixFmts = utils::get_encoder_formats(codec);
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
  codecContext_->gop_size = GOPSize_;
  codecContext_->max_b_frames = 0;  // nvenc can only handle zero!

  if (codecName_.find("vaapi") != std::string::npos) {
    openVAAPIDevice(codec, width, height);
  }

  if (usesHardwareFrames_) {
    AVHWFramesContext * frames_ctx =
      reinterpret_cast<AVHWFramesContext *>(codecContext_->hw_frames_ctx->data);
    codecContext_->sw_pix_fmt = frames_ctx->sw_format;
    codecContext_->pix_fmt = frames_ctx->format;
  } else {
    codecContext_->pix_fmt = utils::get_preferred_pixel_format(codecName_, pixFmts);
    codecContext_->sw_pix_fmt = codecContext_->pix_fmt;
  }

  setAVOption("profile", profile_);
  setAVOption("preset", preset_);
  setAVOption("tune", tune_);
  setAVOption("delay", delay_);
  RCLCPP_DEBUG(
    logger_,
    "codec: %10s, profile: %10s, preset: %10s,"
    " bit_rate: %10ld qmax: %2d",
    codecName_.c_str(), profile_.c_str(), preset_.c_str(), bitRate_, qmax_);

  err = avcodec_open2(codecContext_, codec, NULL);
  utils::check_for_err("cannot open codec", err);

  RCLCPP_INFO_STREAM(logger_, "opened codec: " << codecName_);
  frame_ = av_frame_alloc();
  if (!frame_) {
    throw(std::runtime_error("cannot alloc software frame!"));
  }
  if (usesHardwareFrames_) {
    hw_frame_ = av_frame_alloc();
    if (!hw_frame_) {
      throw(std::runtime_error("cannot alloc hardware frame!"));
    }
  }
  frame_->width = width;
  frame_->height = height;
  frame_->format = codecContext_->sw_pix_fmt;
  // allocate image for frame
  err = av_image_alloc(
    frame_->data, frame_->linesize, width, height, static_cast<AVPixelFormat>(frame_->format), 64);
  utils::check_for_err("cannot alloc image", err);

  if (usesHardwareFrames_) {
    err = av_hwframe_get_buffer(codecContext_->hw_frames_ctx, hw_frame_, 0);
    utils::check_for_err("cannot get hw frame buffer", err);
    if (!hw_frame_->hw_frames_ctx) {
      throw(std::runtime_error("no hardware frame context, out of mem?"));
    }
  }

  // Initialize packet
  packet_ = av_packet_alloc();
  packet_->data = NULL;
  packet_->size = 0;

  // create (src) frame that wraps the received uncompressed image
  wrapperFrame_ = av_frame_alloc();
  wrapperFrame_->width = width;
  wrapperFrame_->height = height;
  wrapperFrame_->format = AV_PIX_FMT_BGR24;

  // initialize format conversion library
  if (!swsContext_) {
    swsContext_ = sws_getContext(
      width, height, AV_PIX_FMT_BGR24,                            // src
      width, height, static_cast<AVPixelFormat>(frame_->format),  // dest
      SWS_FAST_BILINEAR | SWS_ACCURATE_RND, NULL, NULL, NULL);
    if (!swsContext_) {
      throw(std::runtime_error("cannot allocate sws context"));
    }
  }
}

void FFMPEGEncoder::setAVOption(const std::string & field, const std::string & value)
{
  if (!value.empty()) {
    const int err =
      av_opt_set(codecContext_->priv_data, field.c_str(), value.c_str(), AV_OPT_SEARCH_CHILDREN);
    if (err != 0) {
      RCLCPP_ERROR_STREAM(
        logger_, "cannot set option " << field << " to value " << value << ": " << utils::err(err));
    }
  }
}

void FFMPEGEncoder::encodeImage(const Image & msg)
{
  rclcpp::Time t0;
  if (measurePerformance_) {
    t0 = rclcpp::Clock().now();
  }
  // TODO(Bernd): forcing the encoding to be BGR8 is wasteful when
  // the encoder supports monochrome.

  cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  encodeImage(img, msg.header, t0);
  if (measurePerformance_) {
    const auto t1 = rclcpp::Clock().now();
    tdiffDebayer_.update((t1 - t0).seconds());
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
  // bend the memory pointers in colorFrame to the right locations
  av_image_fill_arrays(
    wrapperFrame_->data, wrapperFrame_->linesize, &(img.data[0]),
    static_cast<AVPixelFormat>(wrapperFrame_->format), wrapperFrame_->width, wrapperFrame_->height,
    1 /* alignment, could be better*/);
  sws_scale(
    swsContext_, wrapperFrame_->data, wrapperFrame_->linesize, 0,  // src
    codecContext_->height, frame_->data, frame_->linesize);        // dest

  if (measurePerformance_) {
    t2 = rclcpp::Clock().now();
    tdiffFrameCopy_.update((t2 - t1).seconds());
  }

  frame_->pts = pts_++;  //
  ptsToStamp_.insert(PTSMap::value_type(frame_->pts, header.stamp));

  int ret;
  if (usesHardwareFrames_) {
    ret = av_hwframe_transfer_data(hw_frame_, frame_, 0);  // from software -> hardware frame
    utils::check_for_err("error while copying frame to hw", ret);
    hw_frame_->pts = frame_->pts;
  }

  ret = avcodec_send_frame(codecContext_, usesHardwareFrames_ ? hw_frame_ : frame_);

  if (measurePerformance_) {
    t3 = rclcpp::Clock().now();
    tdiffSendFrame_.update((t3 - t2).seconds());
  }
  // now drain all packets
  while (ret == 0) {
    ret = drainPacket(header, img.cols, img.rows);
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
