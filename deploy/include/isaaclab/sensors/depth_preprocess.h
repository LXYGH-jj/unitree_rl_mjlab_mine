// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "isaaclab/sensors/depth_frame_provider.h"

namespace isaaclab
{
namespace sensors
{

struct DepthPreprocessConfig
{
    int output_height = 58;
    int output_width = 87;
    float near_clip = 0.0f;
    float far_clip = 2.0f;
    bool invert = true;
    bool centered = false;
    bool zero_is_invalid = true;
    std::string invalid_fill = "far";
    float max_valid_depth = std::numeric_limits<float>::infinity();
    std::string over_max_fill = "far";
    std::string fallback = "zero";
    std::string input_unit = "m";
    std::string resize_mode = "bilinear";
    std::string preprocess_mode = "deploy";
};

struct DepthDebugConfig
{
    bool enabled = false;
    std::string dir = "/tmp/go2_depth_camera_debug";
    int save_every = 30;
    bool save_raw = true;
    bool save_processed = true;
    bool print_stats = true;
};

struct DepthPolicyInputNoiseConfig
{
    bool enabled = false;
    bool require_sim_source = true;
    bool source_is_sim = false;
    unsigned int seed = 0;
    float uniform = 0.0f;
    float gaussian_std = 0.0f;
    float bias = 0.0f;
    float dropout_prob = 0.0f;
    float dropout_value = 0.0f;
};

inline DepthPreprocessConfig depth_preprocess_config_from_yaml(YAML::Node params)
{
    DepthPreprocessConfig cfg;
    if (!params) {
        return cfg;
    }
    if (params["output_height"] && !params["output_height"].IsNull()) cfg.output_height = params["output_height"].as<int>();
    if (params["output_width"] && !params["output_width"].IsNull()) cfg.output_width = params["output_width"].as<int>();
    if (params["near_clip"] && !params["near_clip"].IsNull()) cfg.near_clip = params["near_clip"].as<float>();
    if (params["far_clip"] && !params["far_clip"].IsNull()) cfg.far_clip = params["far_clip"].as<float>();
    if (params["invert"] && !params["invert"].IsNull()) cfg.invert = params["invert"].as<bool>();
    if (params["centered"] && !params["centered"].IsNull()) cfg.centered = params["centered"].as<bool>();
    if (params["zero_is_invalid"] && !params["zero_is_invalid"].IsNull()) cfg.zero_is_invalid = params["zero_is_invalid"].as<bool>();
    if (params["invalid_fill"] && !params["invalid_fill"].IsNull()) cfg.invalid_fill = params["invalid_fill"].as<std::string>();
    if (params["max_valid_depth"] && !params["max_valid_depth"].IsNull()) cfg.max_valid_depth = params["max_valid_depth"].as<float>();
    if (params["over_max_fill"] && !params["over_max_fill"].IsNull()) cfg.over_max_fill = params["over_max_fill"].as<std::string>();
    if (params["fallback"] && !params["fallback"].IsNull()) cfg.fallback = params["fallback"].as<std::string>();
    if (params["input_unit"] && !params["input_unit"].IsNull()) cfg.input_unit = params["input_unit"].as<std::string>();
    if (params["resize_mode"] && !params["resize_mode"].IsNull()) cfg.resize_mode = params["resize_mode"].as<std::string>();
    if (params["preprocess_mode"] && !params["preprocess_mode"].IsNull()) cfg.preprocess_mode = params["preprocess_mode"].as<std::string>();
    return cfg;
}

inline DepthDebugConfig depth_debug_config_from_yaml(YAML::Node params)
{
    DepthDebugConfig cfg;
    if (!params || !params["debug"] || params["debug"].IsNull()) {
        return cfg;
    }
    const auto debug = params["debug"];
    if (debug["enabled"] && !debug["enabled"].IsNull()) cfg.enabled = debug["enabled"].as<bool>();
    if (debug["dir"] && !debug["dir"].IsNull()) cfg.dir = debug["dir"].as<std::string>();
    if (debug["save_every"] && !debug["save_every"].IsNull()) cfg.save_every = std::max(1, debug["save_every"].as<int>());
    if (debug["save_raw"] && !debug["save_raw"].IsNull()) cfg.save_raw = debug["save_raw"].as<bool>();
    if (debug["save_processed"] && !debug["save_processed"].IsNull()) cfg.save_processed = debug["save_processed"].as<bool>();
    if (debug["print_stats"] && !debug["print_stats"].IsNull()) cfg.print_stats = debug["print_stats"].as<bool>();
    return cfg;
}

inline DepthPolicyInputNoiseConfig depth_policy_input_noise_config_from_yaml(YAML::Node params)
{
    DepthPolicyInputNoiseConfig cfg;
    if (!params || !params["policy_input_noise"] || params["policy_input_noise"].IsNull()) {
        return cfg;
    }

    const auto noise = params["policy_input_noise"];
    if (noise["enabled"] && !noise["enabled"].IsNull()) cfg.enabled = noise["enabled"].as<bool>();
    if (noise["require_sim_source"] && !noise["require_sim_source"].IsNull()) {
        cfg.require_sim_source = noise["require_sim_source"].as<bool>();
    }
    if (noise["seed"] && !noise["seed"].IsNull()) cfg.seed = noise["seed"].as<unsigned int>();

    if (params["source"] && !params["source"].IsNull()) {
        const auto source = params["source"].as<std::string>();
        cfg.source_is_sim = source.find("/sim/") != std::string::npos ||
            source.find("sim") != std::string::npos;
    }

    const auto depth = noise["depth"] && !noise["depth"].IsNull() ? noise["depth"] : noise;
    if (depth["uniform"] && !depth["uniform"].IsNull()) cfg.uniform = std::max(0.0f, depth["uniform"].as<float>());
    if (depth["gaussian_std"] && !depth["gaussian_std"].IsNull()) {
        cfg.gaussian_std = std::max(0.0f, depth["gaussian_std"].as<float>());
    }
    if (depth["bias"] && !depth["bias"].IsNull()) cfg.bias = depth["bias"].as<float>();
    if (depth["dropout_prob"] && !depth["dropout_prob"].IsNull()) {
        cfg.dropout_prob = std::clamp(depth["dropout_prob"].as<float>(), 0.0f, 1.0f);
    }
    if (depth["dropout_value"] && !depth["dropout_value"].IsNull()) {
        cfg.dropout_value = depth["dropout_value"].as<float>();
    }
    return cfg;
}

inline void apply_depth_policy_input_noise(
    std::vector<float>& depth,
    const DepthPolicyInputNoiseConfig& noise_cfg,
    const DepthPreprocessConfig& preprocess_cfg)
{
    if (!noise_cfg.enabled || (noise_cfg.require_sim_source && !noise_cfg.source_is_sim)) {
        return;
    }

    static std::mt19937 rng(noise_cfg.seed);
    std::uniform_real_distribution<float> unit_dist(0.0f, 1.0f);
    std::uniform_real_distribution<float> uniform_dist(-noise_cfg.uniform, noise_cfg.uniform);
    std::normal_distribution<float> normal_dist(0.0f, noise_cfg.gaussian_std);

    const float min_value = preprocess_cfg.centered ? -0.5f : 0.0f;
    const float max_value = preprocess_cfg.centered ? 0.5f : 1.0f;
    const float dropout_value = std::clamp(noise_cfg.dropout_value, min_value, max_value);

    for (auto& value : depth) {
        if (!std::isfinite(value)) {
            value = 0.0f;
        }
        value += noise_cfg.bias;
        if (noise_cfg.uniform > 0.0f) {
            value += uniform_dist(rng);
        }
        if (noise_cfg.gaussian_std > 0.0f) {
            value += normal_dist(rng);
        }
        if (noise_cfg.dropout_prob > 0.0f && unit_dist(rng) < noise_cfg.dropout_prob) {
            value = dropout_value;
        }
        value = std::clamp(value, min_value, max_value);
    }
}

inline float depth_invalid_fill_value(const DepthPreprocessConfig& cfg)
{
    if (cfg.invalid_fill == "near") {
        return cfg.near_clip;
    }
    if (cfg.invalid_fill == "zero") {
        return 0.0f;
    }
    return cfg.far_clip;
}

inline float depth_fill_value(const DepthPreprocessConfig& cfg, const std::string& fill)
{
    if (fill == "near") {
        return cfg.near_clip;
    }
    if (fill == "zero") {
        return 0.0f;
    }
    return cfg.far_clip;
}

inline std::vector<float> resize_depth_nearest(
    const std::vector<float>& src,
    int src_height,
    int src_width,
    int dst_height,
    int dst_width)
{
    std::vector<float> dst(static_cast<size_t>(dst_height * dst_width), 0.0f);
    if (src.empty() || src_height <= 0 || src_width <= 0 || dst_height <= 0 || dst_width <= 0) {
        return dst;
    }

    for (int y = 0; y < dst_height; ++y) {
        const int src_y = std::clamp(
            static_cast<int>(std::round((static_cast<float>(y) + 0.5f) * src_height / dst_height - 0.5f)),
            0,
            src_height - 1
        );
        for (int x = 0; x < dst_width; ++x) {
            const int src_x = std::clamp(
                static_cast<int>(std::round((static_cast<float>(x) + 0.5f) * src_width / dst_width - 0.5f)),
                0,
                src_width - 1
            );
            dst[static_cast<size_t>(y * dst_width + x)] = src[static_cast<size_t>(src_y * src_width + src_x)];
        }
    }
    return dst;
}

inline std::vector<float> resize_depth_bilinear(
    const std::vector<float>& src,
    int src_height,
    int src_width,
    int dst_height,
    int dst_width)
{
    std::vector<float> dst(static_cast<size_t>(dst_height * dst_width), 0.0f);
    if (src.empty() || src_height <= 0 || src_width <= 0 || dst_height <= 0 || dst_width <= 0) {
        return dst;
    }

    for (int y = 0; y < dst_height; ++y) {
        const float src_y = (static_cast<float>(y) + 0.5f) * src_height / dst_height - 0.5f;
        const int y0 = std::clamp(static_cast<int>(std::floor(src_y)), 0, src_height - 1);
        const int y1 = std::clamp(y0 + 1, 0, src_height - 1);
        const float wy = std::clamp(src_y - static_cast<float>(y0), 0.0f, 1.0f);

        for (int x = 0; x < dst_width; ++x) {
            const float src_x = (static_cast<float>(x) + 0.5f) * src_width / dst_width - 0.5f;
            const int x0 = std::clamp(static_cast<int>(std::floor(src_x)), 0, src_width - 1);
            const int x1 = std::clamp(x0 + 1, 0, src_width - 1);
            const float wx = std::clamp(src_x - static_cast<float>(x0), 0.0f, 1.0f);

            const float v00 = src[static_cast<size_t>(y0 * src_width + x0)];
            const float v01 = src[static_cast<size_t>(y0 * src_width + x1)];
            const float v10 = src[static_cast<size_t>(y1 * src_width + x0)];
            const float v11 = src[static_cast<size_t>(y1 * src_width + x1)];
            const float top = v00 * (1.0f - wx) + v01 * wx;
            const float bottom = v10 * (1.0f - wx) + v11 * wx;
            dst[static_cast<size_t>(y * dst_width + x)] = top * (1.0f - wy) + bottom * wy;
        }
    }
    return dst;
}

inline std::vector<float> preprocess_depth_frame(const DepthFrame& frame, const DepthPreprocessConfig& cfg)
{
    std::vector<float> depth = frame.data;
    const size_t expected = static_cast<size_t>(std::max(0, frame.width) * std::max(0, frame.height));
    if (depth.size() != expected) {
        depth.resize(expected, depth_invalid_fill_value(cfg));
    }

    const float unit_scale = cfg.input_unit == "mm" ? 0.001f : 1.0f;
    if (cfg.preprocess_mode == "astro_play") {
        for (auto& value : depth) {
            value *= unit_scale;
            if (std::isnan(value) || value == std::numeric_limits<float>::infinity()) {
                value = cfg.far_clip;
            } else if (value == -std::numeric_limits<float>::infinity()) {
                value = cfg.near_clip;
            }
            value = std::clamp(value, cfg.near_clip, cfg.far_clip);
        }
    } else {
        const float invalid_value = depth_invalid_fill_value(cfg);
        for (auto& value : depth) {
            value *= unit_scale;
            const bool invalid_zero = cfg.zero_is_invalid && value <= 0.0f;
            if (!std::isfinite(value) || invalid_zero) {
                value = invalid_value;
            }
            if (std::isfinite(cfg.max_valid_depth) && value > cfg.max_valid_depth) {
                value = depth_fill_value(cfg, cfg.over_max_fill);
            }
            value = std::clamp(value, cfg.near_clip, cfg.far_clip);
        }
    }

    if (frame.height != cfg.output_height || frame.width != cfg.output_width) {
        if (cfg.resize_mode == "nearest") {
            depth = resize_depth_nearest(depth, frame.height, frame.width, cfg.output_height, cfg.output_width);
        } else {
            depth = resize_depth_bilinear(depth, frame.height, frame.width, cfg.output_height, cfg.output_width);
        }
    }

    const float denom = std::max(cfg.far_clip - cfg.near_clip, 1.0e-6f);
    for (auto& value : depth) {
        if (cfg.invert) {
            value = cfg.far_clip - value;
        }
        value = value / denom;
        if (cfg.centered) {
            value -= 0.5f;
        }
        if (!std::isfinite(value)) {
            value = 0.0f;
        }
    }

    depth.resize(static_cast<size_t>(cfg.output_height * cfg.output_width), 0.0f);
    return depth;
}

inline std::vector<float> fallback_depth_output(const DepthPreprocessConfig& cfg)
{
    const size_t count = static_cast<size_t>(cfg.output_height * cfg.output_width);
    if (cfg.fallback == "near") {
        const float near_value = cfg.invert ? (cfg.far_clip - cfg.near_clip) / std::max(cfg.far_clip - cfg.near_clip, 1.0e-6f) : 0.0f;
        return std::vector<float>(count, cfg.centered ? near_value - 0.5f : near_value);
    }
    return std::vector<float>(count, 0.0f);
}

struct DepthStats
{
    float min = 0.0f;
    float max = 0.0f;
    float mean = 0.0f;
    size_t finite_count = 0;
    size_t invalid_count = 0;
};

inline DepthStats compute_depth_stats(const std::vector<float>& data)
{
    DepthStats stats;
    if (data.empty()) {
        return stats;
    }

    stats.min = std::numeric_limits<float>::infinity();
    stats.max = -std::numeric_limits<float>::infinity();
    double sum = 0.0;
    for (float value : data) {
        if (!std::isfinite(value)) {
            ++stats.invalid_count;
            continue;
        }
        stats.min = std::min(stats.min, value);
        stats.max = std::max(stats.max, value);
        sum += value;
        ++stats.finite_count;
    }

    if (stats.finite_count == 0) {
        stats.min = 0.0f;
        stats.max = 0.0f;
        stats.mean = 0.0f;
    } else {
        stats.mean = static_cast<float>(sum / static_cast<double>(stats.finite_count));
    }
    return stats;
}

inline void save_depth_txt(
    const std::filesystem::path& path,
    const std::vector<float>& data,
    int height,
    int width)
{
    std::ofstream stream(path);
    if (!stream) {
        return;
    }
    stream << std::fixed << std::setprecision(6);
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            if (col > 0) {
                stream << ' ';
            }
            stream << data[static_cast<size_t>(row * width + col)];
        }
        stream << '\n';
    }
}

inline void save_depth_pgm(
    const std::filesystem::path& path,
    const std::vector<float>& data,
    int height,
    int width,
    float min_value,
    float max_value)
{
    std::ofstream stream(path);
    if (!stream) {
        return;
    }

    stream << "P2\n" << width << ' ' << height << "\n255\n";
    const float denom = std::max(max_value - min_value, 1.0e-6f);
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            float value = data[static_cast<size_t>(row * width + col)];
            if (!std::isfinite(value)) {
                value = min_value;
            }
            const int pixel = static_cast<int>(std::round(std::clamp((value - min_value) / denom, 0.0f, 1.0f) * 255.0f));
            stream << pixel << (col + 1 == width ? '\n' : ' ');
        }
    }
}

inline void maybe_debug_depth(
    const DepthFrame& raw_frame,
    const std::vector<float>& processed,
    const DepthPreprocessConfig& preprocess_cfg,
    const DepthDebugConfig& debug_cfg)
{
    if (!debug_cfg.enabled) {
        return;
    }

    static size_t frame_index = 0;
    ++frame_index;
    if (frame_index % static_cast<size_t>(std::max(1, debug_cfg.save_every)) != 0) {
        return;
    }

    const auto raw_stats = compute_depth_stats(raw_frame.data);
    const auto processed_stats = compute_depth_stats(processed);
    if (debug_cfg.print_stats) {
        std::cout
            << "[depth] raw " << raw_frame.width << "x" << raw_frame.height
            << " finite=" << raw_stats.finite_count
            << " invalid=" << raw_stats.invalid_count
            << " min=" << raw_stats.min
            << " max=" << raw_stats.max
            << " mean=" << raw_stats.mean
            << " | proc " << preprocess_cfg.output_width << "x" << preprocess_cfg.output_height
            << " min=" << processed_stats.min
            << " max=" << processed_stats.max
            << " mean=" << processed_stats.mean
            << std::endl;
    }

    std::filesystem::create_directories(debug_cfg.dir);
    const std::string index = std::to_string(frame_index);
    if (debug_cfg.save_raw && raw_frame.valid && !raw_frame.data.empty()) {
        save_depth_txt(std::filesystem::path(debug_cfg.dir) / ("raw_depth_" + index + ".txt"), raw_frame.data, raw_frame.height, raw_frame.width);
        save_depth_pgm(std::filesystem::path(debug_cfg.dir) / ("raw_depth_" + index + ".pgm"), raw_frame.data, raw_frame.height, raw_frame.width, raw_stats.min, raw_stats.max);
    }
    if (debug_cfg.save_processed && !processed.empty()) {
        save_depth_txt(std::filesystem::path(debug_cfg.dir) / ("processed_depth_" + index + ".txt"), processed, preprocess_cfg.output_height, preprocess_cfg.output_width);
        save_depth_pgm(std::filesystem::path(debug_cfg.dir) / ("processed_depth_" + index + ".pgm"), processed, preprocess_cfg.output_height, preprocess_cfg.output_width, 0.0f, 1.0f);
    }
}

}
}
