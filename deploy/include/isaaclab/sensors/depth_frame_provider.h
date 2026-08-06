// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace isaaclab
{
namespace sensors
{

struct DepthFrame
{
    std::vector<float> data;
    int width = 0;
    int height = 0;
    bool valid = false;
    double timestamp = 0.0;
};

class DepthFrameProvider
{
public:
    virtual ~DepthFrameProvider() = default;
    virtual bool get_latest_frame(DepthFrame& frame) = 0;
};

class NullDepthFrameProvider final : public DepthFrameProvider
{
public:
    bool get_latest_frame(DepthFrame& frame) override
    {
        frame = DepthFrame{};
        return false;
    }
};

class DebugRampDepthFrameProvider final : public DepthFrameProvider
{
public:
    explicit DebugRampDepthFrameProvider(YAML::Node params)
    {
        width_ = yaml_as<int>(params, "input_width", yaml_as<int>(params, "output_width", 87));
        height_ = yaml_as<int>(params, "input_height", yaml_as<int>(params, "output_height", 58));
        near_ = yaml_as<float>(params, "debug_near_m", 0.25f);
        far_ = yaml_as<float>(params, "debug_far_m", 2.0f);
    }

    bool get_latest_frame(DepthFrame& frame) override
    {
        frame.width = width_;
        frame.height = height_;
        frame.valid = width_ > 0 && height_ > 0;
        frame.data.assign(static_cast<size_t>(width_ * height_), far_);
        if (!frame.valid) {
            return false;
        }

        for (int row = 0; row < height_; ++row) {
            const float t = height_ > 1 ? static_cast<float>(row) / static_cast<float>(height_ - 1) : 0.0f;
            const float value = near_ + (far_ - near_) * t;
            for (int col = 0; col < width_; ++col) {
                frame.data[static_cast<size_t>(row * width_ + col)] = value;
            }
        }
        return true;
    }

private:
    template <typename T>
    static T yaml_as(YAML::Node params, const std::string& key, const T& default_value)
    {
        return params && params[key] && !params[key].IsNull() ? params[key].as<T>() : default_value;
    }

    int width_ = 87;
    int height_ = 58;
    float near_ = 0.25f;
    float far_ = 2.0f;
};

class FileDepthFrameProvider final : public DepthFrameProvider
{
public:
    explicit FileDepthFrameProvider(YAML::Node params)
    {
        source_ = yaml_as<std::string>(params, "source", "");
        width_ = yaml_as<int>(params, "input_width", yaml_as<int>(params, "output_width", 87));
        height_ = yaml_as<int>(params, "input_height", yaml_as<int>(params, "output_height", 58));
    }

    bool get_latest_frame(DepthFrame& frame) override
    {
        frame = DepthFrame{};
        frame.width = width_;
        frame.height = height_;
        if (source_.empty() || width_ <= 0 || height_ <= 0) {
            return false;
        }

        std::ifstream stream(source_);
        if (!stream) {
            return false;
        }

        frame.data.reserve(static_cast<size_t>(width_ * height_));
        float value = 0.0f;
        while (stream >> value) {
            frame.data.push_back(value);
        }

        const size_t expected = static_cast<size_t>(width_ * height_);
        if (frame.data.size() != expected) {
            frame.data.resize(expected, 0.0f);
        }
        frame.valid = true;
        return true;
    }

private:
    template <typename T>
    static T yaml_as(YAML::Node params, const std::string& key, const T& default_value)
    {
        return params && params[key] && !params[key].IsNull() ? params[key].as<T>() : default_value;
    }

    std::string source_;
    int width_ = 87;
    int height_ = 58;
};

#ifdef ISAACLAB_ENABLE_ROS2_DEPTH_PROVIDER
bool get_ros2_depth_frame_from_provider(YAML::Node params, DepthFrame& frame);
#endif

#ifdef ISAACLAB_ENABLE_IPC_DEPTH_PROVIDER
bool get_ipc_depth_frame_from_provider(YAML::Node params, DepthFrame& frame);
#endif

inline bool get_depth_frame_from_provider(YAML::Node params, DepthFrame& frame)
{
    std::string provider = "null";
    if (params && params["provider"] && !params["provider"].IsNull()) {
        provider = params["provider"].as<std::string>();
    }

    if (provider == "debug_ramp") {
        DebugRampDepthFrameProvider debug_provider(params);
        return debug_provider.get_latest_frame(frame);
    }
    if (provider == "file") {
        FileDepthFrameProvider file_provider(params);
        return file_provider.get_latest_frame(frame);
    }
    if (provider == "ipc") {
#ifdef ISAACLAB_ENABLE_IPC_DEPTH_PROVIDER
        return get_ipc_depth_frame_from_provider(params, frame);
#else
        static bool warned_ipc_provider_disabled = false;
        if (!warned_ipc_provider_disabled) {
            std::cerr
                << "[depth_frame_provider] provider=ipc requested, but "
                << "ISAACLAB_ENABLE_IPC_DEPTH_PROVIDER is not enabled."
                << std::endl;
            warned_ipc_provider_disabled = true;
        }
        frame = DepthFrame{};
        return false;
#endif
    }
    if (provider == "ros2") {
#ifdef ISAACLAB_ENABLE_ROS2_DEPTH_PROVIDER
        return get_ros2_depth_frame_from_provider(params, frame);
#else
        static bool warned_ros2_provider_disabled = false;
        if (!warned_ros2_provider_disabled) {
            std::cerr
                << "[depth_frame_provider] provider=ros2 requested, but "
                << "ISAACLAB_ENABLE_ROS2_DEPTH_PROVIDER is not enabled."
                << std::endl;
            warned_ros2_provider_disabled = true;
        }
        frame = DepthFrame{};
        return false;
#endif
    }

    NullDepthFrameProvider null_provider;
    return null_provider.get_latest_frame(frame);
}

}
}

#ifdef ISAACLAB_ENABLE_ROS2_DEPTH_PROVIDER
#include "isaaclab/sensors/ros2_depth_frame_provider.h"
#endif

#ifdef ISAACLAB_ENABLE_IPC_DEPTH_PROVIDER
#include "isaaclab/sensors/ipc_depth_frame_provider.h"
#endif
