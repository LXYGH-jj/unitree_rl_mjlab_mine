// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "isaaclab/sensors/depth_frame_provider.h"

namespace isaaclab
{
namespace sensors
{

// The bridge process writes a small binary header followed by a contiguous
// float32 payload into a tmpfs-backed file, typically under /dev/shm.
struct IpcDepthFrameHeader
{
    char magic[8] = {'G', 'O', '2', 'D', 'I', 'P', 'C', '\0'};
    std::uint32_t version = 1;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t payload_type = 1;  // 1 = float32
    std::uint32_t reserved = 0;
    double timestamp = 0.0;
    std::uint64_t payload_bytes = 0;
};

inline std::string ipc_depth_yaml_as_string(YAML::Node params, const std::string& key, const std::string& default_value)
{
    return params && params[key] && !params[key].IsNull() ? params[key].as<std::string>() : default_value;
}

inline std::uint32_t ipc_depth_yaml_as_u32(YAML::Node params, const std::string& key, std::uint32_t default_value)
{
    return params && params[key] && !params[key].IsNull() ? params[key].as<std::uint32_t>() : default_value;
}

class IpcDepthFrameProvider final : public DepthFrameProvider
{
public:
    explicit IpcDepthFrameProvider(YAML::Node params)
    {
        source_ = ipc_depth_yaml_as_string(params, "source", "/dev/shm/go2_depth_latest.bin");
        width_hint_ = ipc_depth_yaml_as_u32(params, "input_width", 0);
        height_hint_ = ipc_depth_yaml_as_u32(params, "input_height", 0);
    }

    bool get_latest_frame(DepthFrame& frame) override
    {
        frame = DepthFrame{};

        std::ifstream stream(source_, std::ios::binary);
        if (!stream) {
            return false;
        }

        IpcDepthFrameHeader header;
        stream.read(reinterpret_cast<char*>(&header), sizeof(header));
        if (!stream) {
            return false;
        }

        if (std::memcmp(header.magic, "GO2DIPC", 7) != 0 || header.version != 1) {
            std::cerr << "[ipc_depth_provider] invalid header in " << source_ << std::endl;
            return false;
        }

        if (header.width == 0 || header.height == 0) {
            header.width = width_hint_;
            header.height = height_hint_;
        }

        if (header.width == 0 || header.height == 0) {
            return false;
        }

        const std::uint64_t expected_bytes =
            static_cast<std::uint64_t>(header.width) * static_cast<std::uint64_t>(header.height) * sizeof(float);
        if (header.payload_type != 1 || header.payload_bytes != expected_bytes) {
            std::cerr << "[ipc_depth_provider] payload size mismatch in " << source_ << std::endl;
            return false;
        }

        frame.width = static_cast<int>(header.width);
        frame.height = static_cast<int>(header.height);
        frame.timestamp = header.timestamp;
        frame.data.resize(static_cast<size_t>(header.width) * static_cast<size_t>(header.height));

        stream.read(reinterpret_cast<char*>(frame.data.data()), static_cast<std::streamsize>(expected_bytes));
        if (!stream) {
            return false;
        }

        frame.valid = true;
        return true;
    }

private:
    std::string source_;
    std::uint32_t width_hint_ = 0;
    std::uint32_t height_hint_ = 0;
};

inline bool get_ipc_depth_frame_from_provider(YAML::Node params, DepthFrame& frame)
{
    IpcDepthFrameProvider provider(params);
    return provider.get_latest_frame(frame);
}

}
}
