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

#ifndef FFMPEG_IMAGE_TRANSPORT__PARAMETER_DEFINITION_HPP_
#define FFMPEG_IMAGE_TRANSPORT__PARAMETER_DEFINITION_HPP_

#include <rclcpp/rclcpp.hpp>

namespace ffmpeg_image_transport
{
struct ParameterDefinition
{
  using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
  using ParameterValue = rclcpp::ParameterValue;

  rclcpp::ParameterValue declare(rclcpp::Node * node, const std::string & paramBase) const;
  // ---- variables
  ParameterValue defaultValue;
  ParameterDescriptor descriptor;
};
}  // namespace ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__PARAMETER_DEFINITION_HPP_
