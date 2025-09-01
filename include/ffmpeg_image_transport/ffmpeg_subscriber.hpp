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

#ifndef FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_
#define FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_

#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_image_transport/parameter_definition.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <string>

namespace ffmpeg_image_transport
{
using FFMPEGSubscriberPlugin = image_transport::SimpleSubscriberPlugin<FFMPEGPacket>;
class FFMPEGSubscriber : public FFMPEGSubscriberPlugin
{
public:
  using Image = sensor_msgs::msg::Image;
  using ImageConstPtr = Image::ConstSharedPtr;
  using FFMPEGPacket = ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
  using FFMPEGPacketConstPtr = FFMPEGPacket::ConstSharedPtr;
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

  FFMPEGSubscriber();
  ~FFMPEGSubscriber() override;

  std::string getTransportName() const override { return "ffmpeg"; }

protected:
  void internalCallback(const FFMPEGPacketConstPtr & msg, const Callback & user_cb) override;

  void subscribeImpl(
    NodeType node, const std::string & base_topic, const Callback & callback, QoSType custom_qos,
    rclcpp::SubscriptionOptions) override;

  void shutdown() override;
  rclcpp::Logger logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_param_interface_;

private:
  void frameReady(const ImageConstPtr & img, bool /*isKeyFrame*/) const;
  void initialize(NodeType node, const std::string & base_topic);
  std::string getDecodersFromMap(const std::string & encoding);
  void declareParameter(NodeType node, const ParameterDefinition & definition);
  void handleAVOptions(const std::string & opt);
  // -------------- variables
  NodeType node_;
  ffmpeg_encoder_decoder::Decoder decoder_;
  std::string decoderType_;
  const Callback * userCallback_{nullptr};
  std::string paramNamespace_;
};
}  // namespace ffmpeg_image_transport
#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_
