#pragma once

#include <array>
#include <cstdint>

#include "unitree_go/msg/low_cmd.hpp"

namespace go2_ros2
{

constexpr float POS_STOP_F = 2.146E9f;
constexpr float VEL_STOP_F = 16000.0f;

struct RawBmsCmd
{
    uint8_t off;
    std::array<uint8_t, 3> reserve;
};

struct RawMotorCmd
{
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;
    std::array<uint32_t, 3> reserve;
};

struct RawLowCmd
{
    std::array<uint8_t, 2> head;
    uint8_t levelFlag;
    uint8_t frameReserve;
    std::array<uint32_t, 2> SN;
    std::array<uint32_t, 2> version;
    uint16_t bandWidth;
    std::array<RawMotorCmd, 20> motorCmd;
    RawBmsCmd bms;
    std::array<uint8_t, 40> wirelessRemote;
    std::array<uint8_t, 12> led;
    std::array<uint8_t, 2> fan;
    uint8_t gpio;
    uint32_t reserve;
    uint32_t crc;
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len);
void fill_crc(unitree_go::msg::LowCmd& msg);

} // namespace go2_ros2
