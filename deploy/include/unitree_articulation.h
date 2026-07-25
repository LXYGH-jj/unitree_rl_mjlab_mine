// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <type_traits>
#include <utility>
#include "isaaclab/assets/articulation/articulation.h"

namespace unitree
{

namespace detail
{
template <typename...>
using void_t = void;

template <typename Msg, typename = void>
struct has_foot_force_fields : std::false_type {};

template <typename Msg>
struct has_foot_force_fields<
    Msg,
    void_t<
        decltype(std::declval<Msg&>().foot_force()),
        decltype(std::declval<Msg&>().foot_force_est())
    >
> : std::true_type {};
}

template <typename LowStatePtr, typename SportModeStatePtr = std::nullptr_t>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_)
    : lowstate(std::move(lowstate_))
    {
        data.joystick = &lowstate->joystick;
        last_update_time_ = std::chrono::steady_clock::now();
    }

    BaseArticulation(LowStatePtr lowstate_, SportModeStatePtr sport_mode_state_)
    : lowstate(std::move(lowstate_)), sport_mode_state(std::move(sport_mode_state_))
    {
        data.joystick = &lowstate->joystick;
        last_update_time_ = std::chrono::steady_clock::now();
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        // base_angular_velocity
        for(int i(0); i<3; i++) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyroscope()[i];
        }
        // project_gravity_body
        data.root_quat_w = Eigen::Quaternionf(
            lowstate->msg_.imu_state().quaternion()[0],
            lowstate->msg_.imu_state().quaternion()[1],
            lowstate->msg_.imu_state().quaternion()[2],
            lowstate->msg_.imu_state().quaternion()[3]
        );
        data.root_quat_w.normalize();
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        bool used_sdk_linear_velocity = false;
        data.root_pos_w_valid = false;
        if constexpr (!std::is_same_v<SportModeStatePtr, std::nullptr_t>) {
            if (sport_mode_state && !sport_mode_state->isTimeout()) {
                std::lock_guard<std::mutex> sport_lock(sport_mode_state->mutex_);
                data.root_pos_w = sport_mode_state->position();
                data.root_pos_w_valid = true;
                // SDK sport mode velocity is published in world frame.
                const Eigen::Vector3f lin_vel_w = sport_mode_state->velocity();
                lin_vel_w_ = lin_vel_w;
                data.root_lin_vel_b = data.root_quat_w.conjugate() * lin_vel_w;
                used_sdk_linear_velocity = true;
            }
            // static int print_count = 0;
            // if (++print_count % 50 == 0) {
            //     std::cout
            //         << "[deploy_sport] valid=" << data.root_pos_w_valid
            //         << " pos="
            //         << data.root_pos_w.x() << ", "
            //         << data.root_pos_w.y() << ", "
            //         << data.root_pos_w.z()
            //         << std::endl;
            // }
        }

        if (!used_sdk_linear_velocity) {
            // Fallback: simple IMU integration with leaky damping.
            const auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - last_update_time_).count();
            last_update_time_ = now;
            dt = std::clamp(dt, 1.0e-4f, 5.0e-2f);

            const Eigen::Vector3f acc_b(
                lowstate->msg_.imu_state().accelerometer()[0],
                lowstate->msg_.imu_state().accelerometer()[1],
                lowstate->msg_.imu_state().accelerometer()[2]
            );
            const Eigen::Vector3f acc_w = data.root_quat_w * acc_b + gravity_w_;
            lin_vel_w_ += acc_w * dt;
            const float leak = std::exp(-linear_velocity_leak_rate_ * dt);
            lin_vel_w_ *= leak;
            data.root_lin_vel_b = data.root_quat_w.conjugate() * lin_vel_w_;
        }

        // joint positions and velocities
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].q();
            data.joint_vel[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].dq();
        }

        using LowStateMsg = std::remove_reference_t<decltype(lowstate->msg_)>;
        if constexpr (detail::has_foot_force_fields<LowStateMsg>::value) {
            const auto & foot_force = lowstate->msg_.foot_force();
            const auto & foot_force_est = lowstate->msg_.foot_force_est();
            for (size_t i = 0; i < data.foot_force.size(); ++i) {
                data.foot_force[i] = static_cast<float>(foot_force[i]);
                data.foot_force_est[i] = static_cast<float>(foot_force_est[i]);
            }
            data.foot_force_available = true;
        } else {
            data.foot_force_available = false;
        }
    }

    LowStatePtr lowstate;
    SportModeStatePtr sport_mode_state{};
private:
    Eigen::Vector3f lin_vel_w_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f gravity_w_ = Eigen::Vector3f(0.0f, 0.0f, -9.81f);
    float linear_velocity_leak_rate_ = 1.5f;
    std::chrono::steady_clock::time_point last_update_time_;
};

}
