// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "isaaclab/sensors/ipc_depth_frame_provider.h"
#include "isaaclab/sensors/depth_preprocess.h"

namespace
{

struct BridgeConfig
{
    std::string node_name = "go2_depth_bridge";
    std::string topic = "/camera/depth/image_rect_raw";
    std::string output_path = "/dev/shm/go2_depth_latest.bin";
    int period_ms = 33;
    int print_every = 30;
};

BridgeConfig parse_args(int argc, char** argv)
{
    BridgeConfig cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto parse_next = [&](std::string& dst) {
            if (i + 1 < argc) {
                dst = argv[++i];
            }
        };
        const auto parse_next_int = [&](int& dst) {
            if (i + 1 < argc) {
                dst = std::max(1, std::stoi(argv[++i]));
            }
        };

        if (arg == "--node-name") {
            parse_next(cfg.node_name);
        } else if (arg == "--topic") {
            parse_next(cfg.topic);
        } else if (arg == "--output") {
            parse_next(cfg.output_path);
        } else if (arg == "--period-ms") {
            parse_next_int(cfg.period_ms);
        } else if (arg == "--print-every") {
            parse_next_int(cfg.print_every);
        } else if (arg.rfind("--node-name=", 0) == 0) {
            cfg.node_name = arg.substr(std::string("--node-name=").size());
        } else if (arg.rfind("--topic=", 0) == 0) {
            cfg.topic = arg.substr(std::string("--topic=").size());
        } else if (arg.rfind("--output=", 0) == 0) {
            cfg.output_path = arg.substr(std::string("--output=").size());
        } else if (arg.rfind("--period-ms=", 0) == 0) {
            cfg.period_ms = std::max(1, std::stoi(arg.substr(std::string("--period-ms=").size())));
        } else if (arg.rfind("--print-every=", 0) == 0) {
            cfg.print_every = std::max(1, std::stoi(arg.substr(std::string("--print-every=").size())));
        }
    }
    return cfg;
}

std::vector<float> convert_image_to_raw_depth(const sensor_msgs::msg::Image& msg)
{
    const size_t pixel_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    std::vector<float> depth;
    depth.reserve(pixel_count);

    if (msg.encoding == "16UC1") {
        const size_t expected_bytes = pixel_count * sizeof(uint16_t);
        if (msg.step < static_cast<size_t>(msg.width) * sizeof(uint16_t) ||
            msg.data.size() < msg.step * static_cast<size_t>(msg.height)) {
            return {};
        }
        for (uint32_t row = 0; row < msg.height; ++row) {
            const uint8_t* row_ptr = msg.data.data() + static_cast<size_t>(row) * msg.step;
            for (uint32_t col = 0; col < msg.width; ++col) {
                uint16_t value = 0;
                std::memcpy(&value, row_ptr + static_cast<size_t>(col) * sizeof(uint16_t), sizeof(uint16_t));
                depth.push_back(static_cast<float>(value));
            }
        }
        if (depth.size() * sizeof(float) < expected_bytes) {
            // No-op: the depth vector is intentionally stored as float32.
        }
        return depth;
    }

    if (msg.encoding == "32FC1") {
        if (msg.step < static_cast<size_t>(msg.width) * sizeof(float) ||
            msg.data.size() < msg.step * static_cast<size_t>(msg.height)) {
            return {};
        }
        depth.resize(pixel_count);
        for (uint32_t row = 0; row < msg.height; ++row) {
            const uint8_t* row_ptr = msg.data.data() + static_cast<size_t>(row) * msg.step;
            for (uint32_t col = 0; col < msg.width; ++col) {
                float value = 0.0f;
                std::memcpy(&value, row_ptr + static_cast<size_t>(col) * sizeof(float), sizeof(float));
                depth[static_cast<size_t>(row) * static_cast<size_t>(msg.width) + static_cast<size_t>(col)] = value;
            }
        }
        return depth;
    }

    return {};
}

class Go2DepthBridgeNode final : public rclcpp::Node
{
public:
    explicit Go2DepthBridgeNode(const BridgeConfig& cfg)
    : Node(cfg.node_name, rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false)),
      cfg_(cfg)
    {
        std::filesystem::create_directories(std::filesystem::path(cfg_.output_path).parent_path());
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            cfg_.topic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_msg_ = std::move(msg);
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(cfg_.period_ms),
            [this]() { this->flush_latest_frame(); });

        RCLCPP_INFO(this->get_logger(), "bridge topic=%s output=%s period_ms=%d",
            cfg_.topic.c_str(), cfg_.output_path.c_str(), cfg_.period_ms);
    }

private:
    static void print_stats(const std::vector<float>& depth)
    {
        const auto stats = isaaclab::sensors::compute_depth_stats(depth);
        std::cout
            << "[go2_depth_bridge] raw finite=" << stats.finite_count
            << " invalid=" << stats.invalid_count
            << " min=" << stats.min
            << " max=" << stats.max
            << " mean=" << stats.mean
            << std::endl;
    }

    bool write_frame_file(const sensor_msgs::msg::Image& msg, const std::vector<float>& depth)
    {
        if (msg.width == 0 || msg.height == 0 || depth.empty()) {
            return false;
        }

        const auto output_path = std::filesystem::path(cfg_.output_path);
        const auto tmp_path = output_path.string() + ".tmp";

        isaaclab::sensors::IpcDepthFrameHeader header;
        std::memcpy(header.magic, "GO2DIPC", 7);
        header.version = 1;
        header.width = msg.width;
        header.height = msg.height;
        header.payload_type = 1;
        header.timestamp = static_cast<double>(msg.header.stamp.sec) +
                           static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        header.payload_bytes = static_cast<std::uint64_t>(depth.size() * sizeof(float));

        std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
        if (!out) {
            return false;
        }

        out.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
        out.write(reinterpret_cast<const char*>(depth.data()), static_cast<std::streamsize>(header.payload_bytes));
        out.flush();
        if (!out) {
            return false;
        }
        out.close();

        std::error_code ec;
        std::filesystem::rename(tmp_path, output_path, ec);
        if (ec) {
            std::filesystem::remove(output_path, ec);
            ec.clear();
            std::filesystem::rename(tmp_path, output_path, ec);
        }
        if (ec) {
            std::cerr << "[go2_depth_bridge] rename failed: " << ec.message() << std::endl;
            return false;
        }
        return true;
    }

    void flush_latest_frame()
    {
        sensor_msgs::msg::Image::ConstSharedPtr msg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            msg = latest_msg_;
            if (!msg || msg == last_written_msg_) {
                return;
            }
        }

        const auto depth = convert_image_to_raw_depth(*msg);
        if (depth.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Unsupported or malformed depth frame.");
            return;
        }

        if (!write_frame_file(*msg, depth)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to write IPC depth frame.");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_written_msg_ = msg;
            if (cfg_.print_every > 0 && (++frame_count_ % cfg_.print_every) == 0) {
                print_stats(depth);
            }
        }
    }

    BridgeConfig cfg_;
    std::mutex mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_msg_;
    sensor_msgs::msg::Image::ConstSharedPtr last_written_msg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::uint64_t frame_count_ = 0;
};

} // namespace

int main(int argc, char** argv)
{
    try {
        const auto cfg = parse_args(argc, argv);

        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
        }

        auto node = std::make_shared<Go2DepthBridgeNode>(cfg);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[go2_depth_bridge] error: " << e.what() << std::endl;
        return 1;
    }
}
