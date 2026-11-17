// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef FFMPEG_IMAGE_TRANSPORT__IMAGE_TRANSPORT_COMPAT_HPP_
#define FFMPEG_IMAGE_TRANSPORT__IMAGE_TRANSPORT_COMPAT_HPP_

// Compatibility layer for the different versions of the image_transport interface.
//
// Rather than relying on preprocessor defines (which break when a header is included by
// a package that does not set them), the types and signatures are deduced from the
// installed image_transport base classes via templates.

#include <algorithm>
#include <image_transport/publisher_plugin.hpp>
#include <image_transport/subscriber_plugin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>
#include <utility>

namespace ffmpeg_image_transport
{
namespace compat
{
// Gives access to the (protected) advertiseImpl() of a publisher plugin base class
template <class Plugin>
struct AdvertiseProbe : Plugin
{
  using AdvertiseFn = decltype(&AdvertiseProbe::advertiseImpl);
};

// Gives access to the (protected) subscribeImpl() of a subscriber plugin base class
template <class Plugin>
struct SubscribeProbe : Plugin
{
  using SubscribeFn = decltype(&SubscribeProbe::subscribeImpl);
};

// Selects PublisherT (newer image_transport) or else PublishFn (older image_transport)
template <class Plugin>
struct PublisherTProbe : Plugin
{
  template <class T = PublisherTProbe>
  static typename T::PublisherT pick(int);
  template <class T = PublisherTProbe>
  static typename T::PublishFn pick(...);
  using PublisherTFn = decltype(pick(0));
};

// Extracts the node and QoS types from the signature of advertiseImpl()
template <class Fn>
struct AdvertiseSignature;
template <class C, class N, class Q, class... Rest>
struct AdvertiseSignature<void (C::*)(N, const std::string &, Q, Rest...)>
{
  using Node = N;
  using QoS = Q;
};

using Signature = AdvertiseSignature<AdvertiseProbe<image_transport::PublisherPlugin>::AdvertiseFn>;

// rclcpp::Node * for older image_transport, image_transport::RequiredInterfaces for newer
using NodeType = std::decay_t<Signature::Node>;
// rmw_qos_profile_t for older image_transport, rclcpp::QoS for newer
using QoSType = std::decay_t<Signature::QoS>;

// Type of the publisher argument of SimplePublisherPlugin::publish()
template <class Plugin>
using PublisherTFn = typename PublisherTProbe<Plugin>::PublisherTFn;

// Base class that overrides advertiseImpl() with whichever signature the installed
// image_transport has (with or without PublisherOptions). Derived classes must implement
// initialize(), which is called before the actual advertisement.
template <class Base, class Fn = typename AdvertiseProbe<Base>::AdvertiseFn>
class AdvertiseAdapter;

template <class Base, class C, class N, class Q, class... Rest>
class AdvertiseAdapter<Base, void (C::*)(N, const std::string &, Q, Rest...)> : public Base
{
protected:
  void advertiseImpl(N node, const std::string & base_topic, Q custom_qos, Rest... rest) override
  {
    auto qos = initialize(node, base_topic, custom_qos);
    Base::advertiseImpl(node, base_topic, qos, rest...);
  }
  virtual std::decay_t<Q> initialize(N node, const std::string & base_topic, Q custom_qos) = 0;
};

// Base class that overrides subscribeImpl() with whichever signature the installed
// image_transport has (with or without SubscriptionOptions). Derived classes must implement
// initialize(), which is called before the actual subscription.
template <class Base, class Fn = typename SubscribeProbe<Base>::SubscribeFn>
class SubscribeAdapter;

template <class Base, class C, class N, class CB, class Q, class... Rest>
class SubscribeAdapter<Base, void (C::*)(N, const std::string &, const CB &, Q, Rest...)>
: public Base
{
protected:
  void subscribeImpl(
    N node, const std::string & base_topic, const CB & callback, Q custom_qos,
    Rest... rest) override
  {
    initialize(node, base_topic);
    Base::subscribeImpl(node, base_topic, callback, custom_qos, rest...);
  }
  virtual void initialize(N node, const std::string & base_topic) = 0;
};

// Both rclcpp::Node * and image_transport::RequiredInterfaces expose the node interfaces,
// but one is a pointer and the other is not.
template <class N>
N & nodeRef(N * node)
{
  return (*node);
}
template <class N>
N & nodeRef(N & node)
{
  return (node);
}

template <class N>
size_t getNamespaceLength(const N & node)
{
  if constexpr (std::is_pointer_v<N>) {
    return (node->get_effective_namespace().length());
  } else {
    return (std::string(node.get_node_base_interface()->get_namespace()).length());
  }
}

// Convert a node into whatever the installed image_transport wants to see
template <class N = NodeType>
N toNodeType(rclcpp::Node & node)
{
  if constexpr (std::is_pointer_v<N>) {
    return (&node);
  } else {
    return (N(node));
  }
}

// Convert a QoS profile into whatever the installed image_transport wants to see
template <class Q = QoSType>
Q toQoSType(const rmw_qos_profile_t & profile)
{
  if constexpr (std::is_same_v<Q, rmw_qos_profile_t>) {
    return (profile);
  } else {
    return (Q(rclcpp::QoSInitialization::from_rmw(profile), profile));
  }
}

// Make sure the queue depth is at least the one provided
template <class Q>
void setMinimumQueueDepth(Q & qos, int depth)
{
  if constexpr (std::is_same_v<Q, rmw_qos_profile_t>) {
    qos.depth = std::max(static_cast<int>(qos.depth), depth);
  } else {
    qos.keep_last(std::max(static_cast<int>(qos.get_rmw_qos_profile().depth), depth));
  }
}

// Publisher is either a shared pointer to an rclcpp::Publisher or a publish function
template <class P, class M>
void publishMessage(const P & publisher, const M & msg)
{
  if constexpr (std::is_invocable_v<const P &, const M &>) {
    publisher(msg);
  } else {
    publisher->publish(msg);
  }
}

}  // namespace compat
}  // namespace ffmpeg_image_transport
#endif  // FFMPEG_IMAGE_TRANSPORT__IMAGE_TRANSPORT_COMPAT_HPP_
