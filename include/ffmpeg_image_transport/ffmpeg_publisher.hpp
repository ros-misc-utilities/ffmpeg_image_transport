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

#ifndef FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
#define FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_

#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <image_transport/simple_publisher_plugin.hpp>
#include <memory>
#include <sensor_msgs/msg/image.hpp>

namespace ffmpeg_image_transport
{
using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using FFMPEGPublisherPlugin = image_transport::SimplePublisherPlugin<FFMPEGPacket>;
using Image = sensor_msgs::msg::Image;
using FFMPEGPacketConstPtr = FFMPEGPacket::ConstSharedPtr;

class FFMPEGPublisher : public FFMPEGPublisherPlugin
{
public:
  using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
  using ParameterValue = rclcpp::ParameterValue;
  struct ParameterDefinition
  {
    ParameterValue defaultValue;
    ParameterDescriptor descriptor;
  };

  FFMPEGPublisher();
  ~FFMPEGPublisher() override;
  std::string getTransportName() const override { return "ffmpeg"; }

protected:
#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos) override;
#else
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions opt) override;
#endif
  void publish(const Image & message, const PublishFn & publish_fn) const override;

private:
  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz);

  rmw_qos_profile_t initialize(
    rclcpp::Node * node, const std::string & base_name, rmw_qos_profile_t custom_qos);
  void declareParameter(
    rclcpp::Node * node, const std::string & base_name, const ParameterDefinition & definition);
  // variables ---------
  rclcpp::Logger logger_;
  const PublishFn * publishFunction_{NULL};
  ffmpeg_encoder_decoder::Encoder encoder_;
  uint32_t frameCounter_{0};
  // ---------- configurable parameters
  int performanceInterval_{175};  // num frames between perf printouts
  bool measurePerformance_{false};
};
}  // namespace ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
