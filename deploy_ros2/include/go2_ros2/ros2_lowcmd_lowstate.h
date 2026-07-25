#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include "go2_ros2/motor_crc.h"

namespace go2_ros2
{

class Ros2LowState
{
public:
    using MsgType = unitree_go::msg::LowState;
    using SharedPtr = std::shared_ptr<Ros2LowState>;

    void on_message(const MsgType::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            msg_ = *msg;
            last_update_time_ = std::chrono::steady_clock::now();
            connected_ = true;
        }
        cv_.notify_all();
    }

    void update()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (std::all_of(msg_.wireless_remote.begin(), msg_.wireless_remote.end(), [](uint8_t i) { return i == 0; }))
        {
            const auto elapsed = std::chrono::steady_clock::now() - last_joystick_time_;
            if (elapsed > std::chrono::milliseconds(joystick_timeout_ms_))
            {
                is_joystick_timeout_ = true;
            }
        }
        else
        {
            last_joystick_time_ = std::chrono::steady_clock::now();
            is_joystick_timeout_ = false;
        }

        unitree::common::REMOTE_DATA_RX key{};
        std::memcpy(key.buff, msg_.wireless_remote.data(), 40);
        joystick.extract(key);
    }

    bool isTimeout() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_)
        {
            return true;
        }
        const auto elapsed = std::chrono::steady_clock::now() - last_update_time_;
        return elapsed > std::chrono::milliseconds(timeout_ms_);
    }

    void set_timeout_ms(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

    void wait_for_connection()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return connected_; });
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool isJoystickTimeout() const { return is_joystick_timeout_; }

    MsgType msg_{};
    mutable std::mutex mutex_;
    unitree::common::UnitreeJoystick joystick;

private:
    uint32_t timeout_ms_{1000};
    uint32_t joystick_timeout_ms_{3000};
    bool connected_{false};
    bool is_joystick_timeout_{false};
    std::chrono::steady_clock::time_point last_update_time_{};
    std::chrono::steady_clock::time_point last_joystick_time_{};
    std::condition_variable cv_;
};

class Ros2LowCmd
{
public:
    using MsgType = unitree_go::msg::LowCmd;
    using Publisher = rclcpp::Publisher<MsgType>;

    explicit Ros2LowCmd(Publisher::SharedPtr publisher)
    : publisher_(std::move(publisher))
    {
        init_message();
    }

    void publish()
    {
        fill_crc(msg_);
        publisher_->publish(msg_);
    }

    void unlockAndPublish() { publish(); }

    MsgType msg_{};

private:
    void init_message()
    {
        msg_.head[0] = 0xFE;
        msg_.head[1] = 0xEF;
        msg_.level_flag = 0xFF;
        msg_.gpio = 0;
        for (auto& motor : msg_.motor_cmd)
        {
            motor.mode = 0x0A;
            motor.q = POS_STOP_F;
            motor.dq = VEL_STOP_F;
            motor.kp = 0.0f;
            motor.kd = 0.0f;
            motor.tau = 0.0f;
            motor.reserve[0] = 0;
            motor.reserve[1] = 0;
            motor.reserve[2] = 0;
        }
    }

    Publisher::SharedPtr publisher_;
};

} // namespace go2_ros2
