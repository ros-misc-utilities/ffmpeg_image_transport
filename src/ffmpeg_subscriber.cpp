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

#include <ffmpeg_encoder_decoder/safe_param.hpp>
#include <ffmpeg_image_transport/ffmpeg_publisher.hpp>
#include <ffmpeg_image_transport/ffmpeg_subscriber.hpp>
#include <functional>
#include <unordered_map>

using namespace std::placeholders;

namespace ffmpeg_image_transport
{
FFMPEGSubscriber::FFMPEGSubscriber() : logger_(rclcpp::get_logger("FFMPEGSubscriber")) {}

FFMPEGSubscriber::~FFMPEGSubscriber() {}

void FFMPEGSubscriber::frameReady(const ImageConstPtr & img, bool) const { (*userCallback_)(img); }

#ifdef IMAGE_TRANSPORT_API_V1
void FFMPEGSubscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  initialize(node, base_topic);
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos);
}
#else
void FFMPEGSubscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions opt)
{
  initialize(node, base_topic);
#ifdef IMAGE_TRANSPORT_API_V2
  (void)opt;  // to suppress compiler warning
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos);
#else
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos, opt);
#endif
}
#endif

void FFMPEGSubscriber::initialize(rclcpp::Node * node, const std::string & base_topic)
{
  node_ = node;
  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  param_namespace_ = param_base_name + "." + getTransportName() + ".map.";
  // create parameters from default map
  for (const auto & kv : ffmpeg_encoder_decoder::Decoder::getDefaultEncoderToDecoderMap()) {
    const std::string key = param_namespace_ + kv.first;
    rclcpp::ParameterValue v;
    try {
      rcl_interfaces::msg::ParameterDescriptor pd;
      pd.set__name(kv.first)
        .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
        .set__description("ffmpeg decoder map entry")
        .set__read_only(false);
      v = node->declare_parameter(key, rclcpp::ParameterValue(kv.second), pd);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG_STREAM(logger_, "was previously declared: " << kv.first);
      v = node->get_parameter(key).get_parameter_value();
    }
    if (v.get<std::string>() != kv.second) {
      RCLCPP_INFO_STREAM(
        logger_, "overriding default decoder " << kv.second << " for " << kv.first << " with "
                                               << v.get<std::string>());
    }
  }
  const bool mp = ffmpeg_encoder_decoder::get_safe_param<bool>(
    node_, param_namespace_ + "measure_performance", false);
  decoder_.setMeasurePerformance(mp);
}

void FFMPEGSubscriber::internalCallback(const FFMPEGPacketConstPtr & msg, const Callback & user_cb)
{
  if (!decoder_.isInitialized()) {
    if (msg->flags == 0) {
      return;  // wait for key frame!
    }
    if (msg->encoding.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "no encoding provided!");
      return;
    }
    userCallback_ = &user_cb;
    const std::string decoder = ffmpeg_encoder_decoder::get_safe_param<std::string>(
      node_, param_namespace_ + msg->encoding, "");
    if (decoder.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "no valid decoder found for encoding: " << msg->encoding);
      return;
    }
    if (!decoder_.initialize(
          msg->encoding, std::bind(&FFMPEGSubscriber::frameReady, this, _1, _2), decoder)) {
      RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder!");
      return;
    }
  }
  decoder_.decodePacket(
    msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id,
    msg->header.stamp);
}
}  // namespace ffmpeg_image_transport
