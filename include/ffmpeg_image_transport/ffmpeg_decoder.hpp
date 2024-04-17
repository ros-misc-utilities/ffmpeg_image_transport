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

#ifndef FFMPEG_IMAGE_TRANSPORT__FFMPEG_DECODER_HPP_
#define FFMPEG_IMAGE_TRANSPORT__FFMPEG_DECODER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "ffmpeg_image_transport/tdiff.hpp"
#include "ffmpeg_image_transport/types.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}

namespace ffmpeg_image_transport
{
class FFMPEGDecoder
{
public:
  using Callback = std::function<void(const ImageConstPtr & img, bool isKeyFrame)>;
  using PTSMap = std::unordered_map<int64_t, rclcpp::Time>;

  FFMPEGDecoder();
  ~FFMPEGDecoder();
  bool isInitialized() const { return (codecContext_ != NULL); }
  // Initialize decoder upon first packet received,
  // providing callback to be called when frame is complete.
  // You must still call decodePacket(msg) afterward!
  bool initialize(
    const FFMPEGPacketConstPtr & msg, Callback callback,
    const std::string & encoding = std::string());

  // clears all state, but leaves config intact
  void reset();
  // decode packet (may result in frame callback!)
  bool decodePacket(const FFMPEGPacketConstPtr & msg);
  void setMeasurePerformance(bool p) { measurePerformance_ = p; }
  void printTimers(const std::string & prefix) const;
  void resetTimers();
  void setLogger(rclcpp::Logger logger) { logger_ = logger; }
  static const std::unordered_map<std::string, std::string> & getDefaultEncoderToDecoderMap();

private:
  rclcpp::Logger logger_;
  bool initDecoder(int w, int h, const std::string & encoding, const std::string & decoder);
  // --------------- variables
  Callback callback_;
  PTSMap ptsToStamp_;  // mapping of header

  // --- performance analysis
  bool measurePerformance_{false};
  TDiff tdiffTotal_;
  // --- libav stuff
  AVRational timeBase_{1, 100};
  std::string encoding_;
  AVCodecContext * codecContext_{NULL};
  AVFrame * decodedFrame_{NULL};
  AVFrame * cpuFrame_{NULL};
  AVFrame * colorFrame_{NULL};
  SwsContext * swsContext_{NULL};
  enum AVPixelFormat hwPixFormat_;
  AVPacket packet_;
  AVBufferRef * hwDeviceContext_{NULL};
};
}  // namespace ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_DECODER_HPP_
