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

#ifndef FFMPEG_IMAGE_TRANSPORT__UTILS_HPP_
#define FFMPEG_IMAGE_TRANSPORT__UTILS_HPP_

#include <opencv2/core.hpp>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
}

namespace ffmpeg_image_transport
{
namespace utils
{
/**!
* convert av pixel format to clear text string
*/
std::string pix(const AVPixelFormat & f);
/**!
* convert av error number to string
*/
std::string err(int errnum);
/**!
* throws runtime_error() with decoded av error string
*/
void throw_err(const std::string & msg, int errnum);
/**!
* checks for error and throws runtime_error() with av error string
*/
void check_for_err(const std::string & msg, int errnum);
/**!
* finds hardware configuration, in particular the target pixel format
* and whether the encoder uses hardware frame upload
*/
enum AVPixelFormat find_hw_config(
  bool * usesHWFrames, enum AVHWDeviceType hwDevType, const AVCodec * codec);
std::vector<enum AVPixelFormat> get_hwframe_transfer_formats(AVBufferRef * hwframe_ctx);

/**!
* finds formats that the encoder supports. Note that for VAAPI, this will just
* return AV_PIX_FMT_VAAPI since it uses hardware frames.
*/
std::vector<enum AVPixelFormat> get_encoder_formats(const AVCodec * avctx);
/**!
* picks from a vector of formats the "best" pixel format for a given encoder
*/
enum AVPixelFormat get_preferred_pixel_format(
  const std::string & encoder, const std::vector<AVPixelFormat> & fmts);

}  // namespace utils
}  // namespace ffmpeg_image_transport
#endif  // FFMPEG_IMAGE_TRANSPORT__UTILS_HPP_
