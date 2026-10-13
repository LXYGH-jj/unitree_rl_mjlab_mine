// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaaclab
{

class Algorithms
{
public:
    virtual ~Algorithms() = default;
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;
    virtual void reset() {}

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }
    
    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        action.resize(output_shape[1]);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // make sure all input names are in obs
        for (const auto& name : input_names) {
            if (obs.find(name) == obs.end()) {
                throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
            }
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(std::size_t i = 0; i < input_names.size(); ++i)
        {
            const std::string name_str(input_names[i]);
            auto& input_data = obs.at(name_str);
            if (input_data.size() != static_cast<size_t>(input_sizes[i])) {
                throw std::runtime_error(
                    "Observation '" + name_str + "' size mismatch. Expected " +
                    std::to_string(input_sizes[i]) + ", got " +
                    std::to_string(input_data.size()) + "."
                );
            }
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_sizes[i], input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(), output_names.data(), 1);

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), floatarr, output_shape[1] * sizeof(float));
        return action;
    }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};

class DepthStudentOrtRunner : public Algorithms
{
public:
    DepthStudentOrtRunner(std::string model_path)
    {
        // Depth-student ONNX has explicit recurrent state:
        // obs + depth + hidden_in -> actions + hidden_out.
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "depth_student_onnx_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        load_input_metadata();
        load_output_metadata();
        validate_required_io();

        hidden.resize(input_sizes.at("hidden_in"), 0.0f);
        action.resize(output_sizes.at("actions"), 0.0f);
        if (output_sizes.find("yaw") != output_sizes.end())
        {
            depth_yaw.resize(output_sizes.at("yaw"), 0.0f);
        }
    }

    void reset() override
    {
        std::fill(hidden.begin(), hidden.end(), 0.0f);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) override
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        std::vector<Ort::Value> input_tensors;
        input_tensors.reserve(input_names.size());

        // Keep temporary input buffers alive until session->Run() returns.
        std::vector<std::vector<float>> owned_inputs;
        owned_inputs.reserve(input_names.size());

        for (const auto& name : input_names)
        {
            const auto input_size = input_sizes.at(name);
            if (name == "hidden_in")
            {
                input_tensors.push_back(
                    Ort::Value::CreateTensor<float>(
                        memory_info,
                        hidden.data(),
                        hidden.size(),
                        input_shapes.at(name).data(),
                        input_shapes.at(name).size()
                    )
                );
                continue;
            }

            if (obs.find(name) == obs.end())
            {
                throw std::runtime_error(
                    "DepthStudentOrtRunner input '" + name + "' not found in observations."
                );
            }
            if (obs.at(name).size() != input_size)
            {
                throw std::runtime_error(
                    "DepthStudentOrtRunner observation '" + name + "' size mismatch. Expected " +
                    std::to_string(input_size) + ", got " +
                    std::to_string(obs.at(name).size()) + "."
                );
            }

            owned_inputs.push_back(obs.at(name));
            auto& input_data = owned_inputs.back();
            input_tensors.push_back(
                Ort::Value::CreateTensor<float>(
                    memory_info,
                    input_data.data(),
                    input_data.size(),
                    input_shapes.at(name).data(),
                    input_shapes.at(name).size()
                )
            );
        }

        auto output_tensors = session->Run(
            Ort::RunOptions{nullptr},
            input_name_ptrs.data(),
            input_tensors.data(),
            input_tensors.size(),
            output_name_ptrs.data(),
            output_name_ptrs.size()
        );

        for (size_t i = 0; i < output_names.size(); ++i)
        {
            const auto& name = output_names[i];
            auto* output_data = output_tensors[i].GetTensorMutableData<float>();
            if (name == "actions")
            {
                std::lock_guard<std::mutex> lock(act_mtx_);
                std::memcpy(action.data(), output_data, action.size() * sizeof(float));
            }
            else if (name == "hidden_out")
            {
                std::memcpy(hidden.data(), output_data, hidden.size() * sizeof(float));
            }
            else if (name == "yaw" && !depth_yaw.empty())
            {
                std::memcpy(depth_yaw.data(), output_data, depth_yaw.size() * sizeof(float));
            }
        }

        maybe_print_depth_yaw();

        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }

private:
    static size_t tensor_size(const std::vector<int64_t>& shape)
    {
        size_t size = 1;
        for (const auto dim : shape)
        {
            if (dim <= 0)
            {
                throw std::runtime_error("DepthStudentOrtRunner only supports static ONNX tensor shapes.");
            }
            size *= static_cast<size_t>(dim);
        }
        return size;
    }

    void load_input_metadata()
    {
        const auto input_count = session->GetInputCount();
        input_names.reserve(input_count);
        input_name_ptrs.reserve(input_count);
        for (size_t i = 0; i < input_count; ++i)
        {
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.emplace_back(input_name.get());

            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            auto shape = input_type.GetTensorTypeAndShapeInfo().GetShape();
            input_shapes[input_names.back()] = shape;
            input_sizes[input_names.back()] = tensor_size(shape);
        }
        for (const auto& name : input_names)
        {
            input_name_ptrs.push_back(name.c_str());
        }
    }

    void load_output_metadata()
    {
        const auto output_count = session->GetOutputCount();
        output_names.reserve(output_count);
        output_name_ptrs.reserve(output_count);
        for (size_t i = 0; i < output_count; ++i)
        {
            auto output_name = session->GetOutputNameAllocated(i, allocator);
            output_names.emplace_back(output_name.get());

            Ort::TypeInfo output_type = session->GetOutputTypeInfo(i);
            auto shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
            output_shapes[output_names.back()] = shape;
            output_sizes[output_names.back()] = tensor_size(shape);
        }
        for (const auto& name : output_names)
        {
            output_name_ptrs.push_back(name.c_str());
        }
    }

    void validate_required_io() const
    {
        for (const auto& name : {"obs", "depth", "hidden_in"})
        {
            if (input_sizes.find(name) == input_sizes.end())
            {
                throw std::runtime_error("DepthStudentOrtRunner missing ONNX input '" + std::string(name) + "'.");
            }
        }
        for (const auto& name : {"actions", "hidden_out"})
        {
            if (output_sizes.find(name) == output_sizes.end())
            {
                throw std::runtime_error("DepthStudentOrtRunner missing ONNX output '" + std::string(name) + "'.");
            }
        }
        if (input_sizes.at("hidden_in") != output_sizes.at("hidden_out"))
        {
            throw std::runtime_error("DepthStudentOrtRunner hidden_in and hidden_out sizes do not match.");
        }
    }

    void maybe_print_depth_yaw()
    {
        if (depth_yaw.size() < 2)
        {
            return;
        }
        const char* every_env = std::getenv("GO2_DEPTH_PRINT_YAW_EVERY");
        if (every_env == nullptr)
        {
            return;
        }
        const long every_n_steps = std::max(1L, std::strtol(every_env, nullptr, 10));
        depth_yaw_print_step += 1;
        if (depth_yaw_print_step % every_n_steps != 0)
        {
            return;
        }
        std::cout
            << "[yaw_debug] depth_delta_yaw=" << depth_yaw[0]
            << " depth_delta_next_yaw=" << depth_yaw[1]
            << std::endl;
    }

    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<const char*> input_name_ptrs;
    std::vector<const char*> output_name_ptrs;

    std::unordered_map<std::string, std::vector<int64_t>> input_shapes;
    std::unordered_map<std::string, std::vector<int64_t>> output_shapes;
    std::unordered_map<std::string, size_t> input_sizes;
    std::unordered_map<std::string, size_t> output_sizes;

    std::vector<float> hidden;
    std::vector<float> depth_yaw;
    long depth_yaw_print_step = 0;
};
};
