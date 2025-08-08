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
#include <ffmpeg_image_transport/utils.hpp>
#include <functional>
#include <unordered_map>

using namespace std::placeholders;

namespace ffmpeg_image_transport
{
using PValue = ParameterDefinition::ParameterValue;
using PDescriptor = ParameterDefinition::ParameterDescriptor;

static const ParameterDefinition params[] = {
  {PValue(""), PDescriptor()
                 .set__name("decoder_av_options")
                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                 .set__description("comma-separated list of AV options: delay:0")
                 .set__read_only(false)},
  {PValue(false), PDescriptor()
                    .set__name("decoder_measure_performance")
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                    .set__description("enable performance timing")
                    .set__read_only(false)}};

FFMPEGSubscriber::FFMPEGSubscriber() : logger_(rclcpp::get_logger("FFMPEGSubscriber")) {}

FFMPEGSubscriber::~FFMPEGSubscriber() { decoder_.reset(); }

void FFMPEGSubscriber::shutdown()
{
  if (decoder_.isInitialized()) {
    RCLCPP_INFO_STREAM(logger_, "flushing decoder.");
    decoder_.flush();  // may cause additional frameReady() calls!
    decoder_.reset();
  }
  SimpleSubscriberPlugin::shutdown();
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
  QoSType custom_qos, rclcpp::SubscriptionOptions opt)
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
void FFMPEGSubscriber::initialize(rclcpp::Node * node, const std::string & base_topic_o)
{
  node_ = node;
#ifdef IMAGE_TRANSPORT_RESOLVES_BASE_TOPIC
  const std::string base_topic = base_topic_o;
#else
  const std::string base_topic =
    node_->get_node_topics_interface()->resolve_topic_name(base_topic_o);
#endif
  uint ns_len = node_->get_effective_namespace().length();
  // if a namespace is given (ns_len > 1), then strip one more
  // character to avoid a leading "/" that will then become a "."
  uint ns_prefix_len = ns_len > 1 ? ns_len + 1 : ns_len;
  std::string param_base_name = base_topic.substr(ns_prefix_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  paramNamespace_ = param_base_name + "." + getTransportName() + ".";

  for (const auto & p : params) {
    declareParameter(node, p);
  }
}

void FFMPEGSubscriber::declareParameter(rclcpp::Node * node, const ParameterDefinition & definition)
{
  const auto v = definition.declare(node, paramNamespace_);
  const auto & n = definition.descriptor.name;
  if (n == "decoder_av_options") {
    handleAVOptions(v.get<std::string>());
  } else if (n == "decoder_measure_performance") {
    decoder_.setMeasurePerformance(v.get<bool>());
  } else {
    RCLCPP_ERROR_STREAM(logger_, "unknown parameter: " << n);
  }
}

std::string FFMPEGSubscriber::getDecodersFromMap(const std::string & encoding)
{
  const auto x = ffmpeg_encoder_decoder::utils::split_encoding(encoding);
  std::string decoders;
  // successively create parameters that are more and more generic,
  // i.e. hevc.yuv
  for (int i = static_cast<int>(x.size()); i > 0; --i) {
    std::string p_name;
    for (int j = 0; j < i; j++) {
      p_name += "." + x[j];
    }
    ParameterDefinition pdef{
      PValue(""), PDescriptor()
                    .set__name("decoders" + p_name)
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                    .set__description("decoders for encoding: " + p_name)
                    .set__read_only(false)};
    decoders = pdef.declare(node_, paramNamespace_).get<std::string>();
    if (!decoders.empty()) {
      break;
    }
  }
  return (decoders);
}

void FFMPEGSubscriber::handleAVOptions(const std::string & opt)
{
  const auto split = utils::splitAVOptions(opt);
  for (const auto & sl : split) {
    const auto kv = utils::splitAVOption(sl);
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
  if (decoder_.isInitialized()) {
    // the decoder is already initialized
    decoder_.decodePacket(
      msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id,
      msg->header.stamp);
    return;
  }
  // need to initialize the decoder
  if (msg->flags == 0) {
    return;  // wait for key frame!
  }
  if (msg->encoding.empty()) {
    RCLCPP_ERROR_STREAM(logger_, "no encoding provided!");
    return;
  }
  userCallback_ = &user_cb;
  const auto codec = ffmpeg_encoder_decoder::utils::split_encoding(msg->encoding)[0];
  std::string decoder_names = getDecodersFromMap(msg->encoding);
  decoder_names = ffmpeg_encoder_decoder::utils::filter_decoders(codec, decoder_names);
  if (decoder_names.empty()) {
    decoder_names = ffmpeg_encoder_decoder::utils::find_decoders(codec);
    RCLCPP_WARN_STREAM(
      logger_, "no decoders configured for encoding " << msg->encoding
                                                      << " defaulting to: " << decoder_names);
  }
  if (decoder_names.empty()) {
    RCLCPP_ERROR_STREAM(
      logger_, "cannot find valid decoder for codec: " << codec << " enc: " << msg->encoding);
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "trying decoders in order: " << decoder_names);

  for (const auto & dec : ffmpeg_encoder_decoder::utils::split_decoders(decoder_names)) {
    try {
      if (!decoder_.initialize(
            msg->encoding, std::bind(&FFMPEGSubscriber::frameReady, this, _1, _2), dec)) {
        RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder: " << dec);
        continue;
      }
    } catch (std::runtime_error & e) {
      continue;
    }
    // sometimes the failure is only detected when the decoding is happening.
    // hopefully this is on the first packet.
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
}  // namespace ffmpeg_image_transport
