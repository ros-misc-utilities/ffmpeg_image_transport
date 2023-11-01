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

#include "ffmpeg_image_transport/ffmpeg_publisher.hpp"

#include "ffmpeg_image_transport/safe_param.hpp"

using namespace std::placeholders;

namespace ffmpeg_image_transport
{
FFMPEGPublisher::FFMPEGPublisher() : logger_(rclcpp::get_logger("FFMPEGPublisher")) {}

FFMPEGPublisher::~FFMPEGPublisher() {}

void FFMPEGPublisher::packetReady(const FFMPEGPacketConstPtr & pkt) { (*publishFunction_)(*pkt); }

#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
void FFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos)
{
  auto qos = initialize(node, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos);
}
#else
void FFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions opt)
{
  auto qos = initialize(node, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos, opt);
}
#endif

rmw_qos_profile_t FFMPEGPublisher::initialize(rclcpp::Node * node, rmw_qos_profile_t custom_qos)
{
  encoder_.setParameters(node);
  const std::string ns = "ffmpeg_image_transport.";
  measurePerformance_ = get_safe_param<bool>(node, ns + "measure_performance", false);
  encoder_.setMeasurePerformance(measurePerformance_);
  performanceInterval_ = get_safe_param<int>(node, ns + "performance_interval", 175);

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
          msg.width, msg.height, std::bind(&FFMPEGPublisher::packetReady, me, _1))) {
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
