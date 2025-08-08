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

#include <ffmpeg_image_transport/parameter_definition.hpp>

namespace ffmpeg_image_transport
{
rclcpp::ParameterValue ParameterDefinition::declare(
  rclcpp::Node * node, const std::string & paramBase) const
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string paramName = paramBase + descriptor.name;
  rclcpp::ParameterValue v;
  try {
    v = node->declare_parameter(paramName, defaultValue, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(node->get_logger(), "%s was previously declared", descriptor.name.c_str());
    v = node->get_parameter(paramName).get_parameter_value();
  }
  return (v);
}
}  // namespace ffmpeg_image_transport
