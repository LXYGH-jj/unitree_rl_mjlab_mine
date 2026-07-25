// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <spdlog/spdlog.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace param
{
inline std::string VERSION = "1.0.0.1-ros2";
inline std::filesystem::path bin_path;
inline std::filesystem::path proj_dir;
inline std::filesystem::path config_dir;
inline YAML::Node config;

inline std::filesystem::path get_bin_path()
{
    std::vector<char> path(1024);
    ssize_t len = readlink("/proc/self/exe", path.data(), path.size());
    if (len != -1)
    {
        path[len] = '\0';
        return std::filesystem::path(path.data());
    }
    throw std::runtime_error("Failed to get executable path.");
}

inline std::filesystem::path source_dir()
{
    return std::filesystem::path(__FILE__).parent_path().parent_path();
}

inline std::filesystem::path default_config_file()
{
    const auto src_cfg = source_dir() / "config" / "config.yaml";
    if (std::filesystem::exists(src_cfg))
    {
        return src_cfg;
    }
    return bin_path.parent_path().parent_path().parent_path()
        / "share" / "go2_rl_mjlab_ros2" / "config" / "config.yaml";
}

inline void load_config_file(const std::filesystem::path& config_file)
{
    if (!std::filesystem::exists(config_file))
    {
        throw std::runtime_error("Config file not found: " + config_file.string());
    }
    config = YAML::LoadFile(config_file.string());
    config_dir = config_file.parent_path();
    proj_dir = source_dir().parent_path();
}

inline std::filesystem::path parser_policy_dir(std::filesystem::path policy_dir)
{
    if (policy_dir.is_relative())
    {
        policy_dir = config_dir / policy_dir;
    }
    policy_dir = std::filesystem::weakly_canonical(policy_dir);

    if (!std::filesystem::exists(policy_dir / "exported"))
    {
        std::vector<std::filesystem::path> dir_list;
        for (const auto& entry : std::filesystem::directory_iterator(policy_dir))
        {
            if (entry.is_directory())
            {
                dir_list.push_back(entry.path());
            }
        }
        std::sort(dir_list.begin(), dir_list.end());
        for (auto it = dir_list.rbegin(); it != dir_list.rend(); ++it)
        {
            if (std::filesystem::exists(*it / "exported"))
            {
                policy_dir = *it;
                break;
            }
        }
    }
    spdlog::info("Policy directory: {}", policy_dir.string());
    return policy_dir;
}

inline std::filesystem::path helper(int argc, char** argv)
{
    bin_path = get_bin_path();
    std::filesystem::path config_file = default_config_file();
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc)
        {
            config_file = argv[++i];
        }
        else if (arg == "--version" || arg == "-v")
        {
            std::cout << "Version: " << VERSION << std::endl;
            std::exit(0);
        }
        else if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: go2_ros2_ctrl [--config CONFIG]" << std::endl;
            std::exit(0);
        }
    }
    load_config_file(config_file);
    return config_file;
}

} // namespace param
