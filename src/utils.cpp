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

#include "ffmpeg_image_transport/utils.hpp"

#include <algorithm>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/imgutils.h>
}

namespace ffmpeg_image_transport
{
namespace utils
{
std::string pix(AVPixelFormat const & f)
{
  char buf[64];
  buf[63] = 0;
  av_get_pix_fmt_string(buf, sizeof(buf) - 1, f);
  return (std::string(buf));
}

// solution from https://github.com/joncampbell123/composite-video-simulator/issues/5
std::string err(int errnum)
{
  char str[AV_ERROR_MAX_STRING_SIZE];
  return (av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum));
}

void throw_err(const std::string & msg, int errnum)
{
  throw(std::runtime_error(msg + ": " + err(errnum)));
}

void check_for_err(const std::string & msg, int errnum)
{
  if (errnum < 0) {
    throw_err(msg, errnum);
  }
}

enum AVPixelFormat find_hw_config(
  bool * usesHWFrames, enum AVHWDeviceType hwDevType, const AVCodec * codec)
{
  *usesHWFrames = false;
  for (int i = 0;; i++) {
    const AVCodecHWConfig * config = avcodec_get_hw_config(codec, i);
    if (!config) {
      return (AV_PIX_FMT_NONE);
    }
    if (
      ((config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) ||
       (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX)) &&
      config->device_type == hwDevType) {
      *usesHWFrames = (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX);
      return (config->pix_fmt);
    }
  }
  return (AV_PIX_FMT_NONE);
}

static bool has_format(const std::vector<AVPixelFormat> & fmts, const AVPixelFormat & f)
{
  return (std::find(fmts.begin(), fmts.end(), f) != fmts.end());
}

enum AVPixelFormat get_preferred_pixel_format(
  const std::string & encoder, const std::vector<AVPixelFormat> & fmts)
{
  // the only format that worked for vaapi was NV12
  if (encoder.find("vaapi") != std::string::npos) {
    return (has_format(fmts, AV_PIX_FMT_NV12) ? AV_PIX_FMT_NV12 : AV_PIX_FMT_NONE);
  }
  if (has_format(fmts, AV_PIX_FMT_BGR24)) {
    return (AV_PIX_FMT_BGR24);  // fastest, needs no copy
  }
  if (has_format(fmts, AV_PIX_FMT_YUV420P)) {
    return (AV_PIX_FMT_YUV420P);  // needs no interleaving
  }
  if (has_format(fmts, AV_PIX_FMT_NV12)) {
    return (AV_PIX_FMT_NV12);  // needs interleaving :()
  }
  return (AV_PIX_FMT_NONE);
}

std::vector<enum AVPixelFormat> get_encoder_formats(const AVCodec * c)
{
  std::vector<enum AVPixelFormat> formats;
  if (c && c->pix_fmts) {
    for (const auto * p = c->pix_fmts; *p != AV_PIX_FMT_NONE; ++p) {
      formats.push_back(*p);
    }
  }
  return (formats);
}

std::vector<enum AVPixelFormat> get_hwframe_transfer_formats(AVBufferRef * hwframe_ctx)
{
  std::vector<enum AVPixelFormat> formats;
  AVPixelFormat * fmts{nullptr};
  int ret =
    av_hwframe_transfer_get_formats(hwframe_ctx, AV_HWFRAME_TRANSFER_DIRECTION_FROM, &fmts, 0);
  if (ret >= 0) {
    for (const auto * f = fmts; *f != AV_PIX_FMT_NONE; f++) {
      formats.push_back(*f);
    }
  }
  return (formats);
}

}  // namespace utils
}  // namespace ffmpeg_image_transport
