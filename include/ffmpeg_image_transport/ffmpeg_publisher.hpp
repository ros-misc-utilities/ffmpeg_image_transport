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

#include <image_transport/simple_publisher_plugin.hpp>
#include <memory>

#include "ffmpeg_image_transport/ffmpeg_encoder.hpp"
#include "ffmpeg_image_transport/types.hpp"

namespace ffmpeg_image_transport
{
using FFMPEGPublisherPlugin = image_transport::SimplePublisherPlugin<FFMPEGPacket>;

class FFMPEGPublisher : public FFMPEGPublisherPlugin
{
public:
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
  void packetReady(const FFMPEGPacketConstPtr & pkt);
  rmw_qos_profile_t initialize(rclcpp::Node * node, rmw_qos_profile_t custom_qos);
  // variables ---------
  rclcpp::Logger logger_;
  const PublishFn * publishFunction_{NULL};
  FFMPEGEncoder encoder_;
  uint32_t frameCounter_{0};
  // ---------- configurable parameters
  int performanceInterval_{175};  // num frames between perf printouts
  bool measurePerformance_{false};
};
}  // namespace ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
