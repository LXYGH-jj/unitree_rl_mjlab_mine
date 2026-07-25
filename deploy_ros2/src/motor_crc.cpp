#include "go2_ros2/motor_crc.h"

#include <cstring>

namespace go2_ros2
{

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t crc = 0xFFFFFFFF;
    constexpr uint32_t polynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (crc & 0x80000000)
            {
                crc <<= 1;
                crc ^= polynomial;
            }
            else
            {
                crc <<= 1;
            }
            if (data & xbit)
            {
                crc ^= polynomial;
            }
            xbit >>= 1;
        }
    }
    return crc;
}

void fill_crc(unitree_go::msg::LowCmd& msg)
{
    RawLowCmd raw{};
    std::memcpy(raw.head.data(), msg.head.data(), 2);
    raw.levelFlag = msg.level_flag;
    raw.frameReserve = msg.frame_reserve;
    std::memcpy(raw.SN.data(), msg.sn.data(), 8);
    std::memcpy(raw.version.data(), msg.version.data(), 8);
    raw.bandWidth = msg.bandwidth;

    for (int i = 0; i < 20; ++i)
    {
        raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
        raw.motorCmd[i].q = msg.motor_cmd[i].q;
        raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
        raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
        raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
        raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;
        std::memcpy(raw.motorCmd[i].reserve.data(), msg.motor_cmd[i].reserve.data(), 12);
    }

    raw.bms.off = msg.bms_cmd.off;
    std::memcpy(raw.bms.reserve.data(), msg.bms_cmd.reserve.data(), 3);
    std::memcpy(raw.wirelessRemote.data(), msg.wireless_remote.data(), 40);
    std::memcpy(raw.led.data(), msg.led.data(), 12);
    std::memcpy(raw.fan.data(), msg.fan.data(), 2);
    raw.gpio = msg.gpio;
    raw.reserve = msg.reserve;

    raw.crc = crc32_core(reinterpret_cast<uint32_t*>(&raw), (sizeof(RawLowCmd) >> 2) - 1);
    msg.crc = raw.crc;
}

} // namespace go2_ros2
