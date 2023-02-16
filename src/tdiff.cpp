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

#include "ffmpeg_image_transport/tdiff.hpp"

#include <iomanip>

namespace ffmpeg_image_transport
{
std::ostream & operator<<(std::ostream & os, const TDiff & td)
{
  os << std::fixed << std::setprecision(4)
     << td.duration_ * (td.cnt_ > 0 ? 1.0 / static_cast<double>(td.cnt_) : 0);
  return os;
}

}  // namespace ffmpeg_image_transport
