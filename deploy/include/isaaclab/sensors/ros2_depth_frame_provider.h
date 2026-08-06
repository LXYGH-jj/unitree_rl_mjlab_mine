// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

#include "isaaclab/sensors/depth_frame_provider.h"

namespace isaaclab
{
namespace sensors
{

template <typename T>
inline T ros2_depth_yaml_as(YAML::Node params, const std::string& key, const T& default_value)
{
    return params && params[key] && !params[key].IsNull() ? params[key].as<T>() : default_value;
}

class Ros2DepthFrameProvider final : public DepthFrameProvider
{
public:
    explicit Ros2DepthFrameProvider(YAML::Node params)
    {
        topic_ = ros2_depth_yaml_as<std::string>(params, "source", "/camera/depth/image_rect_raw");
        timeout_ms_ = ros2_depth_yaml_as<int>(params, "ros2_timeout_ms", 100);
        node_name_ = ros2_depth_yaml_as<std::string>(params, "ros2_node_name", "go2_depth_frame_provider");

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
        node_ = std::make_shared<rclcpp::Node>(node_name_, node_options);
        subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            topic_,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_msg_ = msg;
            });
    }

    bool get_latest_frame(DepthFrame& frame) override
    {
        frame = DepthFrame{};
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
        sensor_msgs::msg::Image::ConstSharedPtr msg;

        while (std::chrono::steady_clock::now() <= deadline) {
            rclcpp::spin_some(node_);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                msg = latest_msg_;
            }
            if (msg) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (!msg) {
            return false;
        }

        return image_to_depth_frame(*msg, frame);
    }

private:
    static bool image_to_depth_frame(const sensor_msgs::msg::Image& msg, DepthFrame& frame)
    {
        frame = DepthFrame{};
        frame.width = static_cast<int>(msg.width);
        frame.height = static_cast<int>(msg.height);
        frame.timestamp = static_cast<double>(msg.header.stamp.sec) +
                          static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

        if (frame.width <= 0 || frame.height <= 0) {
            return false;
        }

        const size_t pixel_count = static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
        frame.data.assign(pixel_count, 0.0f);

        if (msg.encoding == "16UC1") {
            return copy_16uc1(msg, frame, pixel_count);
        }
        if (msg.encoding == "32FC1") {
            return copy_32fc1(msg, frame, pixel_count);
        }

        std::cerr << "[ros2_depth_provider] unsupported image encoding: " << msg.encoding << std::endl;
        frame = DepthFrame{};
        return false;
    }

    static bool copy_16uc1(const sensor_msgs::msg::Image& msg, DepthFrame& frame, size_t pixel_count)
    {
        const size_t bytes_per_pixel = sizeof(uint16_t);
        if (msg.step < static_cast<size_t>(frame.width) * bytes_per_pixel ||
            msg.data.size() < msg.step * static_cast<size_t>(frame.height)) {
            frame = DepthFrame{};
            return false;
        }

        for (int row = 0; row < frame.height; ++row) {
            const uint8_t* row_ptr = msg.data.data() + static_cast<size_t>(row) * msg.step;
            for (int col = 0; col < frame.width; ++col) {
                uint16_t value = 0;
                std::memcpy(&value, row_ptr + static_cast<size_t>(col) * bytes_per_pixel, bytes_per_pixel);
                frame.data[static_cast<size_t>(row * frame.width + col)] = static_cast<float>(value);
            }
        }

        frame.valid = frame.data.size() == pixel_count;
        return frame.valid;
    }

    static bool copy_32fc1(const sensor_msgs::msg::Image& msg, DepthFrame& frame, size_t pixel_count)
    {
        const size_t bytes_per_pixel = sizeof(float);
        if (msg.step < static_cast<size_t>(frame.width) * bytes_per_pixel ||
            msg.data.size() < msg.step * static_cast<size_t>(frame.height)) {
            frame = DepthFrame{};
            return false;
        }

        for (int row = 0; row < frame.height; ++row) {
            const uint8_t* row_ptr = msg.data.data() + static_cast<size_t>(row) * msg.step;
            for (int col = 0; col < frame.width; ++col) {
                float value = 0.0f;
                std::memcpy(&value, row_ptr + static_cast<size_t>(col) * bytes_per_pixel, bytes_per_pixel);
                frame.data[static_cast<size_t>(row * frame.width + col)] = value;
            }
        }

        frame.valid = frame.data.size() == pixel_count;
        return frame.valid;
    }

    std::string topic_;
    std::string node_name_;
    int timeout_ms_ = 100;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::mutex mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_msg_;
};

inline bool get_ros2_depth_frame_from_provider(YAML::Node params, DepthFrame& frame)
{
    const std::string topic = ros2_depth_yaml_as<std::string>(
        params,
        "source",
        "/camera/depth/image_rect_raw");
    const std::string node_name = ros2_depth_yaml_as<std::string>(
        params,
        "ros2_node_name",
        "go2_depth_frame_provider");
    const int timeout_ms = ros2_depth_yaml_as<int>(params, "ros2_timeout_ms", 100);
    const std::string signature = topic + "|" + node_name + "|" + std::to_string(timeout_ms);

    static std::string active_signature;
    static Ros2DepthFrameProvider* provider = nullptr;
    if (!provider || active_signature != signature) {
        // Keep the ROS2 node alive until process exit. Some DDS stacks can throw
        // during node/subscription destruction after a short-lived debug run.
        provider = new Ros2DepthFrameProvider(params);
        active_signature = signature;
    }

    return provider->get_latest_frame(frame);
}

}
}
