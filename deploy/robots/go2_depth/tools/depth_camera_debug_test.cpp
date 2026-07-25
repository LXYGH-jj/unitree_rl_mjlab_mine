// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "isaaclab/sensors/depth_frame_provider.h"
#include "isaaclab/sensors/depth_preprocess.h"

namespace
{

YAML::Node load_depth_params(const std::filesystem::path& yaml_path)
{
    const YAML::Node root = YAML::LoadFile(yaml_path.string());
    const YAML::Node depth = root["observations"]["depth"];
    if (!depth || depth.IsNull() || !depth.IsMap()) {
        throw std::runtime_error("Missing observations.depth in deploy yaml.");
    }

    for (const auto& item : depth) {
        const std::string observation_name = item.first.as<std::string>();
        if (observation_name == "depth_camera_observation") {
            const YAML::Node params = item.second["params"];
            if (!params || params.IsNull()) {
                throw std::runtime_error("depth_camera_observation has no params.");
            }
            return params;
        }
    }

    throw std::runtime_error("observations.depth does not contain depth_camera_observation.");
}

void print_stats(const std::string& name, const std::vector<float>& data)
{
    const auto stats = isaaclab::sensors::compute_depth_stats(data);
    std::cout
        << name
        << " finite=" << stats.finite_count
        << " invalid=" << stats.invalid_count
        << " min=" << stats.min
        << " max=" << stats.max
        << " mean=" << stats.mean
        << std::endl;
}

}

int main(int argc, char** argv)
{
    try {
        std::filesystem::path yaml_path = "config/policy/depth_student/v0/params/deploy.yaml";
        if (argc > 1) {
            yaml_path = argv[1];
        }

        std::cout << "[depth_debug] loading yaml: " << yaml_path << std::endl;
        const YAML::Node params = load_depth_params(yaml_path);
        const auto preprocess_cfg = isaaclab::sensors::depth_preprocess_config_from_yaml(params);
        const auto debug_cfg = isaaclab::sensors::depth_debug_config_from_yaml(params);

        isaaclab::sensors::DepthFrame frame;
        const bool has_frame = isaaclab::sensors::get_depth_frame_from_provider(params, frame);
        std::vector<float> processed;
        if (!has_frame || !frame.valid) {
            std::cout << "[depth_debug] provider returned no valid frame; using fallback." << std::endl;
            processed = isaaclab::sensors::fallback_depth_output(preprocess_cfg);
        } else {
            std::cout
                << "[depth_debug] provider frame: "
                << frame.width << "x" << frame.height
                << " values=" << frame.data.size()
                << std::endl;
            processed = isaaclab::sensors::preprocess_depth_frame(frame, preprocess_cfg);
            print_stats("[depth_debug] raw", frame.data);
        }

        print_stats("[depth_debug] processed", processed);
        isaaclab::sensors::maybe_debug_depth(frame, processed, preprocess_cfg, debug_cfg);

        std::cout
            << "[depth_debug] output shape: "
            << preprocess_cfg.output_height << "x" << preprocess_cfg.output_width
            << " values=" << processed.size()
            << std::endl;
        if (debug_cfg.enabled) {
            std::cout << "[depth_debug] debug dir: " << debug_cfg.dir << std::endl;
        } else {
            std::cout << "[depth_debug] debug output is disabled in yaml." << std::endl;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[depth_debug] error: " << e.what() << std::endl;
        return 1;
    }
}

