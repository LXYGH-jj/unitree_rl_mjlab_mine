#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "keyboard_joystick.h"
#include "unitree_joystick_dsl.hpp"
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
#include "z1_sdk_controller.h"
#include <cmath>
#include <vector>
#endif

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        if(keyboard) {
            keyboard->update();
            if(keyboard_joystick) keyboard_joystick->update(lowstate->joystick);
        }
        update_z1_diagnostic_toggle();
#else
        if(keyboard) {
            keyboard->update();
            KeyboardJoystick::update_joystick(lowstate->joystick, keyboard->key());
        }
#endif
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static bool z1_sdk_enabled()
    {
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        return z1_sdk_controller && z1_sdk_controller->enabled();
#else
        return false;
#endif
    }

    static int lowcmd_joint_count_for_whole_body()
    {
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        return z1_sdk_enabled() ? 12 : static_cast<int>(lowcmd->msg_.motor_cmd().size());
#else
        return static_cast<int>(lowcmd->msg_.motor_cmd().size());
#endif
    }

    static void stop_z1_diagnostic(const std::string& reason = "")
    {
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        if (!z1_diagnostic_active_) {
            return;
        }
        z1_diagnostic_active_ = false;
        z1_diagnostic_base_q_.clear();
        if (!reason.empty()) {
            spdlog::info("Z1 diagnostic motion disabled: {}", reason);
        } else {
            spdlog::info("Z1 diagnostic motion disabled.");
        }
#else
        (void)reason;
#endif
    }

    static void run_z1_diagnostic_if_active(const std::string& state_name)
    {
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        if (!z1_sdk_enabled()) {
            return;
        }

        const auto cfg = param::config["z1_diagnostic"];
        if (!cfg || !cfg["enabled"] || !cfg["enabled"].as<bool>() || !z1_diagnostic_active_) {
            return;
        }

        const auto current_q = z1_sdk_controller->current_q(std::vector<float>(6, 0.0f));
        if (z1_diagnostic_base_q_.size() != current_q.size()) {
            z1_diagnostic_base_q_ = current_q;
            z1_diagnostic_start_time_sec_ =
                static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
        }

        auto command_q = z1_diagnostic_base_q_;
        const auto amplitude_q = cfg["amplitude_q"].as<std::vector<float>>(std::vector<float>{0.15f, 0, 0, 0, 0, 0});
        const float period_sec = cfg["period_sec"].as<float>(4.0f);
        const double now_sec = static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
        const double elapsed = now_sec - z1_diagnostic_start_time_sec_;
        const double phase = (period_sec > 1.0e-3f)
            ? (2.0 * M_PI * elapsed / static_cast<double>(period_sec))
            : 0.0;
        const float wave = static_cast<float>(std::sin(phase));

        for (size_t i = 0; i < command_q.size() && i < amplitude_q.size(); ++i) {
            command_q[i] += amplitude_q[i] * wave;
        }
        z1_sdk_controller->update(command_q);

        const uint64_t now_ms = unitree::common::GetCurrentTimeMillisecond();
        if (now_ms - z1_diagnostic_last_print_ms_ > 1000) {
            z1_diagnostic_last_print_ms_ = now_ms;
            spdlog::info("Z1 diagnostic motion running in {}. q0={:.3f} cmd0={:.3f}",
                state_name,
                current_q.empty() ? 0.0f : current_q[0],
                command_q.empty() ? 0.0f : command_q[0]);
        }
#else
        (void)state_name;
#endif
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
    static std::shared_ptr<KeyboardJoystick> keyboard_joystick;
    static std::shared_ptr<Z1SdkController> z1_sdk_controller;
#endif

private:
    static void update_z1_diagnostic_toggle()
    {
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        const auto cfg = param::config["z1_diagnostic"];
        if (!cfg || !cfg["enabled"] || !cfg["enabled"].as<bool>() || !keyboard) {
            z1_diagnostic_toggle_pressed_ = false;
            return;
        }

        const std::string toggle_key = cfg["toggle_key"].as<std::string>("u");
        const bool pressed = keyboard->key() == toggle_key;
        if (pressed && !z1_diagnostic_toggle_pressed_) {
            if (z1_diagnostic_active_) {
                stop_z1_diagnostic("manual toggle");
            } else if (z1_sdk_enabled()) {
                z1_diagnostic_base_q_ = z1_sdk_controller->current_q(std::vector<float>(6, 0.0f));
                z1_diagnostic_start_time_sec_ =
                    static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
                z1_diagnostic_active_ = true;
                z1_diagnostic_last_print_ms_ = 0;
                spdlog::info("Z1 diagnostic motion enabled. Toggle key: '{}'", toggle_key);
            } else {
                spdlog::warn("Z1 diagnostic motion requested, but direct Z1 SDK bridge is not active.");
            }
        }
        z1_diagnostic_toggle_pressed_ = pressed;
#endif
    }

#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
    inline static bool z1_diagnostic_active_ = false;
    inline static bool z1_diagnostic_toggle_pressed_ = false;
    inline static double z1_diagnostic_start_time_sec_ = 0.0;
    inline static uint64_t z1_diagnostic_last_print_ms_ = 0;
    inline static std::vector<float> z1_diagnostic_base_q_;
#endif
};
