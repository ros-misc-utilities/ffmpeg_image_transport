// -*-c++-*--------------------------------------------------------------------
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

#include <gtest/gtest.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using ImageTransport = image_transport::ImageTransport;
using Image = sensor_msgs::msg::Image;

template <class T>
static void setNodeParameter(
  rclcpp::Node * node, const std::string & name, const std::string & base, const T & v)
{
  const std::string p_name = base + name;
  const rclcpp::Parameter p(p_name, v);
  try {
    node->declare_parameter<T>(p_name, v);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    // ignore exception
  }
  node->set_parameter(p);
}

class TestPublisher : public rclcpp::Node
{
public:
  explicit TestPublisher(const std::string & ns, const rclcpp::NodeOptions & opt)
  : Node("test_publisher", ns, opt)
  {
  }
  virtual ~TestPublisher() {}
  void setNumberOfImages(size_t n) { num_images_ = n; }
  void setParameterBase(const std::string & base) { parameter_base_ = base; }
  template <class T>
  void setParameter(const std::string & name, const T & v)
  {
    setNodeParameter<T>(this, name, parameter_base_, v);
  }

  void setBitRate(int r) { setParameter<int>("bit_rate", r); }

  // the publisher must be created *after* the parameters have been declared
  // since it only reads the parameters when the plugin is loaded
  void initialize()
  {
    image_transport_ = std::make_shared<ImageTransport>(std::shared_ptr<Node>(this, [](auto *) {}));
    pub_ =
      std::make_shared<image_transport::Publisher>(image_transport_->advertise("camera/image", 1));
  }
  bool publish_next()
  {
    if (counter_ >= num_images_) {
      return (false);
    }
    auto img_msg = makeImageMessage(counter_);
    pub_->publish(img_msg);
    counter_++;
    return (true);
  }
  void shutDown()
  {
    pub_.reset();
    image_transport_.reset();
  }

private:
  Image::ConstSharedPtr makeImageMessage(int n)
  {
    auto img = std::make_shared<Image>();
    img->encoding = "mono8";
    img->header.frame_id = "frame_id";
    img->header.stamp = rclcpp::Time(static_cast<int64_t>(n), RCL_ROS_TIME);
    img->height = 480;
    img->width = 640;
    img->is_bigendian = false;
    img->step = (sensor_msgs::image_encodings::bitDepth(img->encoding) / 8) * img->width *
                sensor_msgs::image_encodings::numChannels(img->encoding);
    img->data.resize(img->step * img->height, static_cast<uint8_t>(n));  // sets data!
    return (img);
  }

  // ----- variables --
  size_t counter_{0};
  size_t num_images_{10};
  std::shared_ptr<ImageTransport> image_transport_;
  std::shared_ptr<image_transport::Publisher> pub_;
  std::string parameter_base_;
};

#ifdef IMAGE_TRANSPORT_USE_QOS
static rclcpp::QoS convert_profile(const rmw_qos_profile_t & p)
{
  return (rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(p), p));
}
#else
static const rmw_qos_profile_t & convert_profile(const rmw_qos_profile_t & p) { return (p); }
#endif

class TestSubscriber : public rclcpp::Node
{
public:
  explicit TestSubscriber(const std::string & ns, const rclcpp::NodeOptions & opt)
  : Node("test_subscriber", ns, opt)
  {
  }
  const auto & getImageCounter() const { return (image_counter_); }
  void setParameterBase(const std::string & base) { parameter_base_ = base; }
  template <class T>
  void setParameter(const std::string & name, const T & v)
  {
    setNodeParameter<T>(this, name, parameter_base_, v);
  }
  void initialize()
  {
    image_transport::TransportHints hints(this);
    sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
      this, "camera/image", std::bind(&TestSubscriber::imageCallback, this, std::placeholders::_1),
      "ffmpeg", convert_profile(rmw_qos_profile_default)));
  }
  void shutDown()
  {
    sub_.reset();
    image_transport_.reset();
  }

private:
  void imageCallback(const Image::ConstSharedPtr & img)
  {
    const int num_channels = sensor_msgs::image_encodings::numChannels(img->encoding);
    bool data_equal{true};
    int last_err = 0;
    for (size_t row = 0; row < img->height; row++) {
      for (size_t col = 0; col < img->width; col++) {
        for (int channel = 0; channel < num_channels; channel++) {
          const uint8_t * p = &(img->data[row * img->step + col * num_channels + channel]);
          int err = static_cast<int>(*p) - static_cast<int>(image_counter_);
          if (err && (err != last_err || col == 0 || col == img->width - 1)) {
            std::cout << "mismatch image # " << rclcpp::Time(img->header.stamp).nanoseconds()
                      << " at line " << row << " col: " << col << " chan: " << channel
                      << ", orig: " << image_counter_ << " now: " << static_cast<int>(*p)
                      << std::endl;
            data_equal = false;
            last_err = err;
          }
        }
      }
    }
    image_counter_++;
    EXPECT_TRUE(data_equal);
  }
  std::shared_ptr<ImageTransport> image_transport_;
  std::shared_ptr<image_transport::Subscriber> sub_;
  std::string parameter_base_;
  size_t image_counter_{0};
};

void lossless_compression_test(
  const std::string & ns, const std::string & encoder, const std::string & av_options,
  const std::string & decoders)
{
  const size_t num_images = 10;
  const std::string parameter_base = "camera.image.ffmpeg.";
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions pub_options;
  auto pub_node = std::make_shared<TestPublisher>(ns, pub_options);
  // pub_node->setBitRate(1000000);
  pub_node->setParameterBase(parameter_base);
  pub_node->setParameter<std::string>("encoder", encoder);
  pub_node->setParameter<std::string>("encoder_av_options", av_options);
  pub_node->setParameter<int>("gop_size", 1);                   // to force immediate publish
  pub_node->setParameter<std::string>("pixel_format", "gray");  // needed for lossless!
  pub_node->setNumberOfImages(num_images);
  pub_node->initialize();  // only after params have been set!
  exec.add_node(pub_node);
  rclcpp::NodeOptions sub_options;
  auto sub_node = std::make_shared<TestSubscriber>(ns, sub_options);
  // must use hevc, for some reason hevc_cuvid will not deliver the last frame
  sub_node->setParameterBase(parameter_base);
  sub_node->setParameter<std::string>("decoders.hevc_gray_bgr8_mono8", decoders);
  sub_node->initialize();  // only after params have been set!
  exec.add_node(sub_node);
  while (pub_node->publish_next() && rclcpp::ok()) {
    exec.spin_some();
  }
  for (int i = 0; i < 10 && rclcpp::ok(); i++) {
    exec.spin_some();
  }
  pub_node->shutDown();
  sub_node->shutDown();
  EXPECT_EQ(sub_node->getImageCounter(), num_images);
}

TEST(ffmpeg_image_transport, test_with_namespace)
{
  rclcpp::init(0, nullptr);
  lossless_compression_test(
    "my_ns", "libx265", "x265-params:lossless=1,tune:zerolatency", "hevc,hevc_cuvid");
}

TEST(ffmpeg_image_transport, test_without_namespace)
{
  lossless_compression_test(
    "/", "libx265", "x265-params:lossless=1,tune:zerolatency", "hevc,hevc_cuvid");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
