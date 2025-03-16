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

using namespace std::placeholders;

namespace ffmpeg_image_transport
{
using ParameterDefinition = FFMPEGPublisher::ParameterDefinition;
using ParameterValue = FFMPEGPublisher::ParameterValue;
using ParameterDescriptor = FFMPEGPublisher::ParameterDescriptor;

static const ParameterDefinition params[] = {
  {ParameterValue("libx264"),
   ParameterDescriptor()
     .set__name("encoding")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
     .set__description("ffmpeg encoder to use, see ffmpeg h264 supported encoders")
     .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("preset")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                         .set__description("ffmpeg encoder preset")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("tune")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                         .set__description("ffmpeg encoder tune")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("delay")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                         .set__description("ffmpeg encoder delay")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("crf")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                         .set__description("constant rate factor")
                         .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("pixel_format")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                         .set__description("pixel format to use for encoding")
                         .set__read_only(false)},
  {ParameterValue(static_cast<int>(10)),
   ParameterDescriptor()
     .set__name("qmax")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("max video quantizer scale, see ffmpeg docs")
     .set__read_only(false)
     .set__integer_range(
       {rcl_interfaces::msg::IntegerRange().set__from_value(-1).set__to_value(1024).set__step(1)})},
  {ParameterValue(static_cast<int64_t>(8242880)),
   ParameterDescriptor()
     .set__name("bit_rate")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("target bit rate, see ffmpeg docs")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
  {ParameterValue(static_cast<int>(10)),
   ParameterDescriptor()
     .set__name("gop_size")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("gop size (distance between keyframes)")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
  {ParameterValue(false), ParameterDescriptor()
                            .set__name("measure_performance")
                            .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                            .set__description("enable performance timing")
                            .set__read_only(false)},
  {ParameterValue(static_cast<int>(175)),
   ParameterDescriptor()
     .set__name("performance_interval")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("after how many frames to print perf info")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
};

FFMPEGPublisher::FFMPEGPublisher() : logger_(rclcpp::get_logger("FFMPEGPublisher")) {}

FFMPEGPublisher::~FFMPEGPublisher() {}

// This code was lifted from compressed_image_transport

void FFMPEGPublisher::declareParameter(
  rclcpp::Node * node, const std::string & base_name, const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string transport_name = getTransportName();
  const std::string param_name =
    base_name + "." + transport_name + "." + definition.descriptor.name;
  rclcpp::ParameterValue v;
  try {
    v = node->declare_parameter(param_name, definition.defaultValue, definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    v = node->get_parameter(param_name).get_parameter_value();
  }
  const auto & n = definition.descriptor.name;
  if (n == "encoding") {
    encoder_.setEncoder(v.get<std::string>());
    RCLCPP_INFO_STREAM(logger_, "using encoder: " << v.get<std::string>());
  } else if (n == "preset") {
    encoder_.setPreset(v.get<std::string>());
  } else if (n == "tune") {
    encoder_.setTune(v.get<std::string>());
  } else if (n == "delay") {
    encoder_.setDelay(v.get<std::string>());
  } else if (n == "crf") {
    encoder_.setCRF(v.get<std::string>());
  } else if (n == "pixel_format") {
    encoder_.setPixelFormat(v.get<std::string>());
  } else if (n == "qmax") {
    encoder_.setQMax(v.get<int>());
  } else if (n == "bit_rate") {
    encoder_.setBitRate(v.get<int>());
  } else if (n == "gop_size") {
    encoder_.setGOPSize(v.get<int>());
  } else if (n == "measure_performance") {
    measurePerformance_ = v.get<bool>();
    encoder_.setMeasurePerformance(v.get<bool>());
  } else if (n == "performance_interval") {
    performanceInterval_ = v.get<int>();
  } else {
    RCLCPP_ERROR_STREAM(logger_, "unknown parameter: " << n);
  }
}

void FFMPEGPublisher::packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
{
  auto msg = std::make_shared<FFMPEGPacket>();
  msg->header.frame_id = frame_id;
  msg->header.stamp = stamp;
  msg->encoding = codec;
  msg->width = width;
  msg->height = height;
  msg->pts = pts;
  msg->flags = flags;
  msg->data.assign(data, data + sz);

  (*publishFunction_)(*msg);
}

#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
void FFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos)
{
  auto qos = initialize(node, base_topic, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos);
}
#else
void FFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions opt)
{
  auto qos = initialize(node, base_topic, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos, opt);
}
#endif

rmw_qos_profile_t FFMPEGPublisher::initialize(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos)
{
  // namespace handling code lifted from compressed_image_transport
  const uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  for (const auto & p : params) {
    declareParameter(node, param_base_name, p);
  }
  // bump queue size to 2 * distance between keyframes
  custom_qos.depth = std::max(static_cast<int>(custom_qos.depth), 2 * encoder_.getGOPSize());
  return (custom_qos);
}

void FFMPEGPublisher::publish(const Image & msg, const PublishFn & publish_fn) const
{
  FFMPEGPublisher * me = const_cast<FFMPEGPublisher *>(this);
  if (!me->encoder_.isInitialized()) {
    me->publishFunction_ = &publish_fn;
    if (!me->encoder_.initialize(
          msg.width, msg.height,
          std::bind(&FFMPEGPublisher::packetReady, me, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
      RCLCPP_ERROR_STREAM(logger_, "cannot initialize encoder!");
      return;
    }
  }
  // may trigger packetReady() callback(s) from encoder!
  me->encoder_.encodeImage(msg);

  if (measurePerformance_) {
    if (static_cast<int>(++me->frameCounter_) > performanceInterval_) {
      me->encoder_.printTimers(logger_.get_name());
      me->encoder_.resetTimers();
      me->frameCounter_ = 0;
    }
  }
}

}  // namespace ffmpeg_image_transport
