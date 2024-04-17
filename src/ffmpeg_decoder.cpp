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

#include "ffmpeg_image_transport/ffmpeg_decoder.hpp"

#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <unordered_map>

namespace ffmpeg_image_transport
{
// default mappings
static const std::unordered_map<std::string, std::string> defaultMap{
  {{"h264_nvenc", "h264"},
   {"libx264", "h264"},
   {"hevc_nvenc", "hevc_cuvid"},
   {"h264_nvmpi", "h264"},
   {"h264_vaapi", "h264"}}};

FFMPEGDecoder::FFMPEGDecoder() : logger_(rclcpp::get_logger("FFMPEGDecoder")) {}

FFMPEGDecoder::~FFMPEGDecoder() { reset(); }

void FFMPEGDecoder::reset()
{
  if (codecContext_) {
    avcodec_close(codecContext_);
    av_free(codecContext_);
    codecContext_ = NULL;
  }
  if (swsContext_) {
    sws_freeContext(swsContext_);
    swsContext_ = NULL;
  }
  if (hwDeviceContext_) {
    av_buffer_unref(&hwDeviceContext_);
  }
  av_free(decodedFrame_);
  decodedFrame_ = NULL;
  av_free(cpuFrame_);
  cpuFrame_ = NULL;
  av_free(colorFrame_);
  colorFrame_ = NULL;
}

bool FFMPEGDecoder::initialize(
  const FFMPEGPacketConstPtr & msg, Callback callback, const std::string & dec)
{
  std::string decoder = dec;
  if (decoder.empty()) {
    RCLCPP_INFO_STREAM(logger_, "no decoder for encoding: " << msg->encoding);
    return (false);
  }
  callback_ = callback;
  encoding_ = msg->encoding;
  return (initDecoder(msg->width, msg->height, encoding_, decoder));
}

static enum AVHWDeviceType get_hw_type(const std::string & name, rclcpp::Logger logger)
{
  enum AVHWDeviceType type = av_hwdevice_find_type_by_name(name.c_str());
  if (type == AV_HWDEVICE_TYPE_NONE) {
    RCLCPP_INFO_STREAM(logger, "hw accel device is not supported: " << name);
    RCLCPP_INFO_STREAM(logger, "available devices:");
    while ((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE)
      RCLCPP_INFO_STREAM(logger, av_hwdevice_get_type_name(type));
    return (type);
  }
  return (type);
}

static AVBufferRef * hw_decoder_init(
  AVBufferRef ** hwDeviceContext, const enum AVHWDeviceType hwType, rclcpp::Logger logger)
{
  int rc = av_hwdevice_ctx_create(hwDeviceContext, hwType, NULL, NULL, 0);
  if (rc < 0) {
    RCLCPP_ERROR_STREAM(logger, "failed to create context for HW device: " << hwType);
    return (NULL);
  }
  return (av_buffer_ref(*hwDeviceContext));
}

static std::unordered_map<AVCodecContext *, AVPixelFormat> pix_format_map;

static enum AVPixelFormat get_hw_format(AVCodecContext * ctx, const enum AVPixelFormat * pix_fmts)
{
  enum AVPixelFormat pf = pix_format_map[ctx];
  const enum AVPixelFormat * p;
  for (p = pix_fmts; *p != -1; p++) {
    if (*p == pf) {
      return *p;
    }
  }
  std::cerr << "Failed to get HW surface format." << std::endl;
  return AV_PIX_FMT_NONE;
}

static enum AVPixelFormat find_pix_format(
  const std::string & codecName, enum AVHWDeviceType hwDevType, const AVCodec * codec,
  const std::string & hwAcc, rclcpp::Logger logger)
{
  for (int i = 0;; i++) {
    const AVCodecHWConfig * config = avcodec_get_hw_config(codec, i);
    if (!config) {
      RCLCPP_WARN_STREAM(
        logger, "decoder " << codecName << " does not support hw accel: " << hwAcc);
      return (AV_PIX_FMT_NONE);
    }
    if (
      config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
      config->device_type == hwDevType) {
      return (config->pix_fmt);
    }
  }
  return (AV_PIX_FMT_NONE);
}

bool FFMPEGDecoder::initDecoder(
  int width, int height, const std::string & encoding, const std::string & decoder)
{
  try {
    const AVCodec * codec = NULL;
    codec = avcodec_find_decoder_by_name(decoder.c_str());
    if (!codec) {
      RCLCPP_ERROR_STREAM(logger_, "cannot find decoder " << decoder);
      throw(std::runtime_error("cannot find decoder " + decoder));
    }
    codecContext_ = avcodec_alloc_context3(codec);
    if (!codecContext_) {
      RCLCPP_ERROR_STREAM(logger_, "alloc context failed for " + decoder);
      codec = NULL;
      throw(std::runtime_error("alloc context failed!"));
    }
    av_opt_set_int(codecContext_, "refcounted_frames", 1, 0);
    const std::string hwAcc("cuda");
    enum AVHWDeviceType hwDevType = get_hw_type(hwAcc, logger_);
    // default
    hwPixFormat_ = AV_PIX_FMT_NONE;

    if (hwDevType != AV_HWDEVICE_TYPE_NONE) {
      codecContext_->hw_device_ctx = hw_decoder_init(&hwDeviceContext_, hwDevType, logger_);
      if (codecContext_->hw_device_ctx != NULL) {
        hwPixFormat_ = find_pix_format(encoding, hwDevType, codec, hwAcc, logger_);
        // must put in global hash for the callback function
        pix_format_map[codecContext_] = hwPixFormat_;
        codecContext_->get_format = get_hw_format;
      } else {  // hardware couldn't be initialized.
        hwDevType = AV_HWDEVICE_TYPE_NONE;
      }
    }
    codecContext_->width = width;
    codecContext_->height = height;
    codecContext_->pkt_timebase = timeBase_;

    if (avcodec_open2(codecContext_, codec, NULL) < 0) {
      RCLCPP_ERROR_STREAM(logger_, "open context failed for " + decoder);
      av_free(codecContext_);
      codecContext_ = NULL;
      codec = NULL;
      throw(std::runtime_error("open context failed!"));
    }
    decodedFrame_ = av_frame_alloc();
    cpuFrame_ = (hwPixFormat_ == AV_PIX_FMT_NONE) ? NULL : av_frame_alloc();
    colorFrame_ = av_frame_alloc();
    colorFrame_->width = width;
    colorFrame_->height = height;
    colorFrame_->format = AV_PIX_FMT_BGR24;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
    reset();
    return (false);
  }
  RCLCPP_INFO_STREAM(logger_, "decoding with " << decoder);
  return (true);
}

bool FFMPEGDecoder::decodePacket(const FFMPEGPacketConstPtr & msg)
{
  rclcpp::Time t0;
  if (measurePerformance_) {
    t0 = rclcpp::Clock().now();
  }
  if (msg->encoding != encoding_) {
    RCLCPP_ERROR_STREAM(
      logger_, "no on-the fly encoding change from " << encoding_ << " to " << msg->encoding);
    return (false);
  }
  AVCodecContext * ctx = codecContext_;
  AVPacket * packet = av_packet_alloc();
  av_new_packet(packet, msg->data.size());  // will add some padding!
  memcpy(packet->data, &msg->data[0], msg->data.size());
  packet->pts = msg->pts;
  packet->dts = packet->pts;
  ptsToStamp_[packet->pts] = msg->header.stamp;
  int ret = avcodec_send_packet(ctx, packet);
  if (ret != 0) {
    RCLCPP_WARN_STREAM(logger_, "send_packet failed for pts: " << msg->pts);
    av_packet_unref(packet);
    return (false);
  }
  ret = avcodec_receive_frame(ctx, decodedFrame_);
  const bool isAcc = (ret == 0) && (decodedFrame_->format == hwPixFormat_);
  if (isAcc) {
    ret = av_hwframe_transfer_data(cpuFrame_, decodedFrame_, 0);
    if (ret < 0) {
      RCLCPP_WARN_STREAM(logger_, "failed to transfer data from GPU->CPU");
      av_packet_unref(packet);
      return (false);
    }
  }
  AVFrame * frame = isAcc ? cpuFrame_ : decodedFrame_;

  if (ret == 0 && frame->width != 0) {
    // convert image to something palatable
    if (!swsContext_) {
      swsContext_ = sws_getContext(
        ctx->width, ctx->height, (AVPixelFormat)frame->format,        // src
        ctx->width, ctx->height, (AVPixelFormat)colorFrame_->format,  // dest
        SWS_FAST_BILINEAR | SWS_ACCURATE_RND, NULL, NULL, NULL);
      if (!swsContext_) {
        RCLCPP_ERROR(logger_, "cannot allocate sws context!!!!");
        return (false);
      }
    }
    // prepare the decoded message
    ImagePtr image(new Image());
    image->height = frame->height;
    image->width = frame->width;
    image->step = image->width * 3;  // 3 bytes per pixel
    image->encoding = sensor_msgs::image_encodings::BGR8;
    image->data.resize(image->step * image->height);

    // bend the memory pointers in colorFrame to the right locations
    av_image_fill_arrays(
      colorFrame_->data, colorFrame_->linesize, &(image->data[0]),
      (AVPixelFormat)colorFrame_->format, colorFrame_->width, colorFrame_->height, 1);
    sws_scale(
      swsContext_, frame->data, frame->linesize, 0,            // src
      ctx->height, colorFrame_->data, colorFrame_->linesize);  // dest
    auto it = ptsToStamp_.find(decodedFrame_->pts);
    if (it == ptsToStamp_.end()) {
      RCLCPP_ERROR_STREAM(logger_, "cannot find pts that matches " << decodedFrame_->pts);
    } else {
      image->header = msg->header;
      image->header.stamp = it->second;
      ptsToStamp_.erase(it);
      callback_(image, decodedFrame_->key_frame == 1);  // deliver callback
    }
  }
  av_packet_unref(packet);
  av_packet_free(&packet);
  if (measurePerformance_) {
    const auto t1 = rclcpp::Clock().now();
    double dt = (t1 - t0).seconds();
    tdiffTotal_.update(dt);
  }
  return (true);
}

void FFMPEGDecoder::resetTimers() { tdiffTotal_.reset(); }

void FFMPEGDecoder::printTimers(const std::string & prefix) const
{
  RCLCPP_INFO_STREAM(logger_, prefix << " total decode: " << tdiffTotal_);
}

const std::unordered_map<std::string, std::string> & FFMPEGDecoder::getDefaultEncoderToDecoderMap()
{
  return (defaultMap);
}
}  // namespace ffmpeg_image_transport
