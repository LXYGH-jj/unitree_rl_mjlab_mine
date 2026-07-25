// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <array>

#include <eigen3/Eigen/Dense>
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

namespace isaaclab
{

class MotionLoader;

struct ArticulationData
{
    Eigen::Vector3f GRAVITY_VEC_W = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f FORWARD_VEC_B = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    std::vector<float> joint_stiffness; // sdk order
    std::vector<float> joint_damping; // sdk order

    // Joint positions of all joints.
    Eigen::VectorXf joint_pos;
    
    // Default joint positions of all joints.
    Eigen::VectorXf default_joint_pos;

    // Joint velocities of all joints.
    Eigen::VectorXf joint_vel;

    // Root angular velocity in base world frame.
    Eigen::Vector3f root_ang_vel_b;

    // Root linear velocity in base frame (estimated from IMU acceleration).
    Eigen::Vector3f root_lin_vel_b = Eigen::Vector3f::Zero();

    // Root position in world frame when provided by simulator/SDK high state.
    Eigen::Vector3f root_pos_w = Eigen::Vector3f::Zero();
    bool root_pos_w_valid = false;

    // Projection of the gravity direction on base frame.
    Eigen::Vector3f projected_gravity_b;

    Eigen::Quaternionf root_quat_w;

    std::vector<float> joint_ids_map;

    // Optional foot force readings. Robots without this SDK field leave these at zero.
    std::array<float, 4> foot_force = {0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 4> foot_force_est = {0.0f, 0.0f, 0.0f, 0.0f};
    bool foot_force_available = false;

    unitree::common::UnitreeJoystick* joystick = nullptr;
};

class Articulation
{
public:
    Articulation(){}

    virtual void update(){};

    ArticulationData data;
};

};
