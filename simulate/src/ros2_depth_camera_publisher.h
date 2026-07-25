#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class Ros2DepthCameraPublisher
{
public:
  Ros2DepthCameraPublisher(
      const std::string& node_name,
      const std::string& topic,
      int print_every)
  : print_every_(std::max(0, print_every))
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::InitOptions options;
      options.shutdown_on_signal = false;
      rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
    }

    rclcpp::NodeOptions node_options;
    node_options.enable_rosout(false);
    node_options.start_parameter_services(false);
    node_options.start_parameter_event_publisher(false);
    node_ = std::make_shared<rclcpp::Node>(node_name, node_options);
    publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS());
    topic_ = topic;
  }

  void publish(const float* depth_meters, int width, int height, double sim_time)
  {
    if (!depth_meters || width <= 0 || height <= 0) {
      return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp.sec = static_cast<int32_t>(sim_time);
    msg.header.stamp.nanosec =
        static_cast<uint32_t>((sim_time - static_cast<double>(msg.header.stamp.sec)) * 1.0e9);
    msg.header.frame_id = "front_depth";
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.encoding = "32FC1";
    msg.is_bigendian = 0;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(
        static_cast<size_t>(width) * sizeof(float));
    msg.data.resize(static_cast<size_t>(msg.step) * static_cast<size_t>(height));

    std::memcpy(
        msg.data.data(),
        depth_meters,
        static_cast<size_t>(width) * static_cast<size_t>(height) * sizeof(float));
    publisher_->publish(msg);

    if (print_every_ > 0 && frame_count_++ % print_every_ == 0) {
      print_stats(depth_meters, width, height);
    }
  }

private:
  void print_stats(const float* depth_meters, int width, int height)
  {
    const int count = width * height;
    float min_value = std::numeric_limits<float>::infinity();
    float max_value = 0.0f;
    double sum = 0.0;
    int finite_count = 0;
    for (int i = 0; i < count; ++i) {
      const float value = depth_meters[i];
      if (!std::isfinite(value)) {
        continue;
      }
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
      sum += value;
      ++finite_count;
    }
    const float center = depth_meters[(height / 2) * width + width / 2];
    std::cout
        << "[ros2_depth_camera_publisher] topic=" << topic_
        << " size=" << width << "x" << height
        << " min=" << min_value
        << " max=" << max_value
        << " mean=" << (finite_count > 0 ? sum / finite_count : 0.0)
        << " center=" << center
        << " finite=" << finite_count << "/" << count
        << std::endl;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::string topic_;
  int print_every_ = 0;
  int frame_count_ = 0;
};
