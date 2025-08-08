// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <ffmpeg_encoder_decoder/utils.hpp>

namespace ffmpeg_image_transport
{
namespace utils
{
std::vector<std::string> splitAVOptions(const std::string & opt)
{
  return (ffmpeg_encoder_decoder::utils::split_by_char(opt, ','));
}
std::vector<std::string> splitAVOption(const std::string & opt)
{
  return (ffmpeg_encoder_decoder::utils::split_by_char(opt, ':'));
}
}  // namespace utils
}  // namespace ffmpeg_image_transport
