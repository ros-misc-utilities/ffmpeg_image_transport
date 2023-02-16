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

#ifndef FFMPEG_IMAGE_TRANSPORT__SAFE_PARAM_HPP_
#define FFMPEG_IMAGE_TRANSPORT__SAFE_PARAM_HPP_
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace ffmpeg_image_transport
{
template <class T>
T get_safe_param(rclcpp::Node * n, const std::string name, T def)
{
  if (n->has_parameter(name)) {
    T p(def);
    n->get_parameter(name, p);
    return (p);
  }
  return (n->declare_parameter<T>(name, def));
}
}  // namespace ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__SAFE_PARAM_HPP_
