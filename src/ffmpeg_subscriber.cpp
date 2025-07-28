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
#include <ffmpeg_encoder_decoder/utils.hpp>
#include <ffmpeg_image_transport/ffmpeg_publisher.hpp>
#include <ffmpeg_image_transport/ffmpeg_subscriber.hpp>
#include <functional>
#include <unordered_map>

using namespace std::placeholders;

namespace ffmpeg_image_transport
{
using PValue = ParameterDefinition::ParameterValue;
using PDescriptor = ParameterDefinition::ParameterDescriptor;

static const ParameterDefinition params[] = {
  {PValue(""), PDescriptor()
                 .set__name("av_options")
                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                 .set__description("comma-separated list of AV options: delay:0")
                 .set__read_only(false)},
  {PValue(false), PDescriptor()
                    .set__name("measure_performance")
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                    .set__description("enable performance timing")
                    .set__read_only(false)}};

FFMPEGSubscriber::FFMPEGSubscriber() : logger_(rclcpp::get_logger("FFMPEGSubscriber")) {}

FFMPEGSubscriber::~FFMPEGSubscriber() {}

void FFMPEGSubscriber::shutdown()
{
  if (decoder_.isInitialized()) {
    RCLCPP_INFO_STREAM(logger_, "flushing decoder.");
    decoder_.flush();
  }
}

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
  uint ns_len = node_->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  param_namespace_ = param_base_name + "." + getTransportName() + ".";

  for (const auto & p : params) {
    declareParameter(node, param_base_name, p);
  }
}

void FFMPEGSubscriber::declareParameter(
  rclcpp::Node * node, const std::string & base_name, const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const auto v = definition.declare(node, base_name, getTransportName());
  const auto & n = definition.descriptor.name;
  if (n == "av_options") {
    RCLCPP_INFO_STREAM(logger_, "declaring av options: " << v.get<std::string>());
    handleAVOptions(v.get<std::string>());
  } else if (n == "measure_performance") {
    decoder_.setMeasurePerformance(v.get<bool>());
  } else {
    RCLCPP_ERROR_STREAM(logger_, "unknown parameter: " << n);
  }
}

void FFMPEGSubscriber::declareEncodingToDecodersMap(const std::string & encoding)
{
  // create parameters from default map
  const std::string key = param_namespace_ + "map." + encoding;
  rclcpp::ParameterValue v;
  try {
    rcl_interfaces::msg::ParameterDescriptor pd;
    pd.set__name(encoding)
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      .set__description("decoders for encoding: " + encoding)
      .set__read_only(false);
    const auto val = node_->declare_parameter(key, rclcpp::ParameterValue(""), pd);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG_STREAM(logger_, "was previously declared: " << encoding);
  }
}

void FFMPEGSubscriber::handleAVOptions(const std::string & opt)
{
  const auto split = ffmpeg_encoder_decoder::utils::split_by_char(opt, ',');
  for (const auto & sl : split) {
    const auto kv = ffmpeg_encoder_decoder::utils::split_by_char(sl, ':');
    if (kv.size() != 2) {
      RCLCPP_WARN_STREAM(logger_, "skipping bad AV option: " << sl);
    } else {
      decoder_.addAVOption(kv[0], kv[1]);
      RCLCPP_INFO_STREAM(logger_, "setting AV option " << kv[0] << " to " << kv[1]);
    }
  }
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
    declareEncodingToDecodersMap(msg->encoding);
    const std::string decoder_names = ffmpeg_encoder_decoder::get_safe_param<std::string>(
      node_, param_namespace_ + "map." + msg->encoding, "");
    const std::vector<std::string> decoders =
      ffmpeg_encoder_decoder::utils::split_by_char(decoder_names, ',');
    if (decoders.empty()) {
      RCLCPP_WARN_STREAM(logger_, "no decoders configured for encoding " << msg->encoding);
      // No alternatives where found, let the decoder figure out which is best to use.
      // Downside: will not catch problems that happen during decode, just when opening
      // the codec.
      if (!decoder_.initialize(
            msg->encoding, std::bind(&FFMPEGSubscriber::frameReady, this, _1, _2), decoders)) {
        RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder!");
        return;
      }
    } else {
      RCLCPP_INFO_STREAM(logger_, "trying configured decoders in order: " << decoder_names);
      for (const auto & dec : decoders) {
        try {
          if (!decoder_.initialize(
                msg->encoding, std::bind(&FFMPEGSubscriber::frameReady, this, _1, _2), {dec})) {
            RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder: " << dec);
            continue;
          }
        } catch (std::runtime_error & e) {
          continue;
        }
        // sometimes the failure is only detected when the decode is happening
        if (!decoder_.decodePacket(
              msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id,
              msg->header.stamp)) {
          RCLCPP_ERROR_STREAM(logger_, "decoder cannot decode packet: " << dec);
          decoder_.reset();
          continue;
        }
        RCLCPP_INFO_STREAM(logger_, "successfully opened decoder " << dec);
        break;
      }
    }
  } else {  // the decoder is already initialized
    decoder_.decodePacket(
      msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id,
      msg->header.stamp);
  }
}
}  // namespace ffmpeg_image_transport
