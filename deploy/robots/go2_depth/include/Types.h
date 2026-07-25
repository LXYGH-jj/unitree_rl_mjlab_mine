#pragma once

#include <memory>

#include "unitree/dds_wrapper/robots/go2/go2.h"

using LowCmd_t = unitree::robot::go2::publisher::LowCmd;
using LowState_t = unitree::robot::go2::subscription::LowState;
using SportModeState_t = unitree::robot::go2::subscription::SportModeState;

extern std::shared_ptr<SportModeState_t> g_go2_depth_sport_state;
