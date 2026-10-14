#pragma once
// M2.4d2: isolated localhost-only test protocol; NOT the Unitree Z1 wire protocol.
// No vendor SDK, physical command sink, hardware permission or realtime controller.
#include "z1_transport_session.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace b2z1::m24d2 {
constexpr std::uint32_t kMockMagic = 0x5A315258u; // "Z1RX", test ONLY
constexpr std::uint16_t kMockVersion = 1;
constexpr std::size_t kMockWireBytes = 16 + 6 * 28 + 4; // 188 bytes
constexpr std::uint16_t kMockConnected = 1;
constexpr std::uint16_t kMockJointCtrl = 2;

std::uint32_t crc32(const std::uint8_t* bytes, std::size_t n);

// A hardwired in-memory command sink: cannot send commands over sockets or SDK.
class RecordingSink final : public b2z1::m24c3::CommandSink {
public:
    void emit(const b2z1::z1_jointctrl::JointCommand& cmd) override { commands_.push_back(cmd); }
    const std::vector<b2z1::z1_jointctrl::JointCommand>& commands() const noexcept {
        return commands_;
    }
private:
    std::vector<b2z1::z1_jointctrl::JointCommand> commands_;
};

// The ONLY allowed receiving endpoint is 127.0.0.1. Peer port is explicitly
// verified. The test packet has a MOCK sequence/CRC that the REAL SDK lacks.
// All receiver-validated feedback is routed to M2.4d1 and then M2.4c3's MOCK sink.
class LoopbackLab final {
public:
    LoopbackLab(b2z1::z1_jointctrl::Config safety,
                b2z1::m24d1::Limits rx_limits,
                std::uint16_t expected_peer_port);
    ~LoopbackLab();
    LoopbackLab(const LoopbackLab&) = delete;
    LoopbackLab& operator=(const LoopbackLab&) = delete;

    std::uint16_t listenerPort() const noexcept { return listener_port_; }
    void receiveOne(int timeout_ms); // success or latches permanent fault, always fail closed
    // FAKE approvals for offline mathematical tests; never from physical hardware.
    void authorizeForMockTest(b2z1::z1_jointctrl::Time now);
    void setMockGoal(const b2z1::z1_jointctrl::Joint6& goal,
                     b2z1::z1_jointctrl::Time now);
    bool mockTick(b2z1::z1_jointctrl::Time now);
    const std::vector<b2z1::z1_jointctrl::JointCommand>& mockCommands() const noexcept {
        return sink_.commands();
    }
    bool faulted() const noexcept { return faulted_ || session_.faulted(); }
    const std::string& localFault() const noexcept { return fault_reason_; }

private:
    [[noreturn]] void abortRx(const std::string& reason);
    RecordingSink sink_;
    b2z1::m24d1::TransportSession session_;
    int fd_ = -1;
    std::uint16_t expected_peer_port_ = 0;
    std::uint16_t listener_port_ = 0;
    std::uint64_t last_sequence_ = 0;
    std::uint64_t event_index_ = 0;
    bool faulted_ = false;
    std::string fault_reason_;
};
} // namespace b2z1::m24d2
