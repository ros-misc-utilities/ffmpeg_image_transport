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
#include <ffmpeg_image_transport/parameter_definition.hpp>
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
#ifdef IMAGE_TRANSPORT_USE_PUBLISHER_T
  using PublisherTFn = PublisherT;
#else
  using PublisherTFn = PublishFn;
#endif
#ifdef IMAGE_TRANSPORT_USE_QOS
  using QoSType = rclcpp::QoS;
#else
  using QoSType = rmw_qos_profile_t;
#endif
#ifdef IMAGE_TRANSPORT_USE_NODEINTERFACE
  using NodeType = image_transport::RequiredInterfaces;
#else
  using NodeType = rclcpp::Node *;
#endif

  FFMPEGPublisher();
  ~FFMPEGPublisher() override;
  std::string getTransportName() const override { return "ffmpeg"; }

protected:
#ifdef IMAGE_TRANSPORT_NEEDS_PUBLISHEROPTIONS
  void advertiseImpl(
    NodeType node, const std::string & base_topic, QoSType custom_qos,
    rclcpp::PublisherOptions opt) override;
#else
  void advertiseImpl(NodeType node, const std::string & base_topic, QoSType custom_qos) override;
#endif
  void publish(const Image & message, const PublisherTFn & publisher) const override;
  void shutdown() override;

private:
  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz);

  QoSType initialize(NodeType node, const std::string & base_name, QoSType custom_qos);
  void declareParameter(NodeType node, const ParameterDefinition & definition);
  void handleAVOptions(const std::string & opt);
  // variables ---------
  rclcpp::Logger logger_;
  const PublisherTFn * publishFunction_{NULL};
  ffmpeg_encoder_decoder::Encoder encoder_;
  uint32_t frameCounter_{0};
  std::string paramNamespace_;
  // ---------- configurable parameters
  int performanceInterval_{175};  // num frames between perf printouts
  bool measurePerformance_{false};
};
}  // namespace ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
