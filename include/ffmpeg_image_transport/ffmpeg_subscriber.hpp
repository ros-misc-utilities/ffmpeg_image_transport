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

#include <image_transport/simple_subscriber_plugin.hpp>
#include <string>

#include "ffmpeg_image_transport/ffmpeg_decoder.hpp"
#include "ffmpeg_image_transport/types.hpp"

namespace ffmpeg_image_transport
{
using FFMPEGSubscriberPlugin = image_transport::SimpleSubscriberPlugin<FFMPEGPacket>;

class FFMPEGSubscriber : public FFMPEGSubscriberPlugin
{
public:
  FFMPEGSubscriber();
  ~FFMPEGSubscriber();

  std::string getTransportName() const override { return "ffmpeg"; }

protected:
  void internalCallback(const FFMPEGPacketConstPtr & msg, const Callback & user_cb) override;

#ifdef IMAGE_TRANSPORT_API_V1
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos) override;
#else
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions) override;
#endif

private:
  void frameReady(const ImageConstPtr & img, bool /*isKeyFrame*/) const;
  void initialize(rclcpp::Node * node);
  // -------------- variables
  rclcpp::Logger logger_;
  rclcpp::Node * node_;
  FFMPEGDecoder decoder_;
  std::string decoderType_;
  const Callback * userCallback_;
};
}  // namespace ffmpeg_image_transport
#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_
