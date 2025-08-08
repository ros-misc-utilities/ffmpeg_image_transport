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

#include <ffmpeg_encoder_decoder/utils.hpp>
#include <ffmpeg_image_transport/ffmpeg_publisher.hpp>
#include <ffmpeg_image_transport/utils.hpp>

using namespace std::placeholders;

namespace ffmpeg_image_transport
{
using ParameterValue = ParameterDefinition::ParameterValue;
using ParameterDescriptor = ParameterDefinition::ParameterDescriptor;

static const ParameterDefinition params[] = {
  {ParameterValue("libx264"),
   ParameterDescriptor()
     .set__name("encoder")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
     .set__description("ffmpeg encoder to use, see ffmpeg supported encoders")
     .set__read_only(false)},
  {ParameterValue(""),
   ParameterDescriptor()
     .set__name("encoder_av_options")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
     .set__description("comma-separated list of AV options: profile:main,preset:ll")
     .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
                         .set__name("pixel_format")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                         .set__description("pixel format to use for encoding")
                         .set__read_only(false)},
  {ParameterValue(static_cast<int>(-1)),
   ParameterDescriptor()
     .set__name("qmax")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("max video quantizer scale, see ffmpeg docs")
     .set__read_only(false)
     .set__integer_range(
       {rcl_interfaces::msg::IntegerRange().set__from_value(-1).set__to_value(1024).set__step(1)})},
  {ParameterValue(static_cast<int64_t>(-1)),
   ParameterDescriptor()
     .set__name("bit_rate")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("target bit rate, see ffmpeg docs")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(-1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
  {ParameterValue(static_cast<int>(-1)),
   ParameterDescriptor()
     .set__name("gop_size")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("gop size (distance between keyframes)")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(-1)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
  {ParameterValue(static_cast<int>(0)),
   ParameterDescriptor()
     .set__name("max_b_frames")
     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
     .set__description("max number of b frames")
     .set__read_only(false)
     .set__integer_range({rcl_interfaces::msg::IntegerRange()
                            .set__from_value(0)
                            .set__to_value(std::numeric_limits<int>::max())
                            .set__step(1)})},
  {ParameterValue(false), ParameterDescriptor()
                            .set__name("encoder_measure_performance")
                            .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                            .set__description("enable performance timing")
                            .set__read_only(false)},
};

FFMPEGPublisher::FFMPEGPublisher() : logger_(rclcpp::get_logger("FFMPEGPublisher")) {}

FFMPEGPublisher::~FFMPEGPublisher() {}

void FFMPEGPublisher::shutdown()
{
  if (encoder_.isInitialized()) {
    RCLCPP_INFO_STREAM(logger_, "flushing encoder.");
    encoder_.flush();
  }
}

// This code was lifted from compressed_image_transport

void FFMPEGPublisher::declareParameter(rclcpp::Node * node, const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const auto v = definition.declare(node, paramNamespace_);
  const auto & n = definition.descriptor.name;
  if (n == "encoder") {
    encoder_.setEncoder(v.get<std::string>());
    RCLCPP_INFO_STREAM(logger_, "using libav encoder: " << v.get<std::string>());
  } else if (n == "encoder_av_options") {
    handleAVOptions(v.get<std::string>());
  } else if (n == "pixel_format") {
    encoder_.setAVSourcePixelFormat(v.get<std::string>());
  } else if (n == "qmax") {
    encoder_.setQMax(v.get<int>());
  } else if (n == "max_b_frames") {
    encoder_.setMaxBFrames(v.get<int>());
  } else if (n == "bit_rate") {
    encoder_.setBitRate(v.get<int>());
  } else if (n == "gop_size") {
    encoder_.setGOPSize(v.get<int>());
  } else if (n == "encoder_measure_performance") {
    measurePerformance_ = v.get<bool>();
    encoder_.setMeasurePerformance(v.get<bool>());
  } else {
    RCLCPP_ERROR_STREAM(logger_, "unknown parameter: " << n);
  }
}

void FFMPEGPublisher::handleAVOptions(const std::string & opt)
{
  const auto split = utils::splitAVOptions(opt);
  for (const auto & sl : split) {
    const auto kv = utils::splitAVOption(sl);
    if (kv.size() != 2) {
      RCLCPP_WARN_STREAM(logger_, "skipping bad AV option: " << sl);
    } else {
      encoder_.addAVOption(kv[0], kv[1]);
      RCLCPP_INFO_STREAM(logger_, "setting AV option " << kv[0] << " to " << kv[1]);
    }
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
#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
  (*publishFunction_)(*msg);
#else
  (*publishFunction_)->publish(*msg);
#endif
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
  rclcpp::Node * node, const std::string & base_topic, QoSType custom_qos,
  rclcpp::PublisherOptions opt)
{
  auto qos = initialize(node, base_topic, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos, opt);
}
#endif

FFMPEGPublisher::QoSType FFMPEGPublisher::initialize(
  rclcpp::Node * node, const std::string & base_topic, QoSType custom_qos)
{
  // namespace handling code lifted from compressed_image_transport
  uint ns_len = node->get_effective_namespace().length();
  // if a namespace is given (ns_len > 1), then strip one more
  // character to avoid a leading "/" that will then become a "."
  uint ns_prefix_len = ns_len > 1 ? ns_len + 1 : ns_len;
  std::string param_base_name = base_topic.substr(ns_prefix_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  paramNamespace_ = param_base_name + "." + getTransportName() + ".";

  for (const auto & p : params) {
    declareParameter(node, p);
  }
  // bump queue size to 2 * distance between keyframes
#ifdef IMAGE_TRANSPORT_USE_QOS
  custom_qos.keep_last(
    std::max(static_cast<int>(custom_qos.get_rmw_qos_profile().depth), 2 * encoder_.getGOPSize()));
#else
  custom_qos.depth = std::max(static_cast<int>(custom_qos.depth), 2 * encoder_.getGOPSize());
#endif
  return (custom_qos);
}

void FFMPEGPublisher::publish(const Image & msg, const PublisherTFn & publisher) const
{
  FFMPEGPublisher * me = const_cast<FFMPEGPublisher *>(this);
  me->publishFunction_ = &publisher;
  if (!me->encoder_.isInitialized()) {
    if (!me->encoder_.initialize(
          msg.width, msg.height,
          std::bind(&FFMPEGPublisher::packetReady, me, _1, _2, _3, _4, _5, _6, _7, _8, _9),
          msg.encoding)) {
      RCLCPP_ERROR_STREAM(logger_, "cannot initialize encoder");
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
