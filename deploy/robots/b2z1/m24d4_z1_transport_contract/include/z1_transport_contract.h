#pragma once
// M2.4d4 is a MOCK-ONLY integration contract. Never a hardware SDK adapter.
#include "z1_transport_session.h"
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace b2z1::m24d4 {
using b2z1::m24d1::Snapshot;
using b2z1::m24d1::RxEvidence;
using b2z1::z1_jointctrl::JointCommand;
using b2z1::z1_jointctrl::Time;

struct ReceivedFrame { Snapshot snapshot{}; RxEvidence evidence{}; };
enum class TransportKind { SyntheticMock, VendorDisabled };

// Abstraction only. An untrusted implementation cannot certify physical RX.
// No implementation in this module has physical network, SDK or motor IO.
class IZ1Transport {
public:
    virtual ~IZ1Transport() = default;
    virtual TransportKind kind() const noexcept = 0;
    virtual std::optional<ReceivedFrame> popFrame() = 0;
    virtual void emit(const JointCommand& command) = 0;
    virtual void disarm() noexcept = 0; // logical only, not an E-stop
};

// Intentional permanent refusal; NOT a wrapper that silently opens the SDK.
class VendorDisabledTransport final : public IZ1Transport {
public:
    TransportKind kind() const noexcept override { return TransportKind::VendorDisabled; }
    std::optional<ReceivedFrame> popFrame() override { return std::nullopt; }
    void emit(const JointCommand&) override {
        throw std::runtime_error("M2.4d4 vendor IO LOCKED: no audited RX/physical safety approval");
    }
    void disarm() noexcept override {}
    static constexpr bool kRealSdkRxAvailable = false;
    static constexpr bool kPhysicalOutputEnabled = false;
};

// Synthetic frames are generated in memory; the source assertions in RxEvidence
// are TEST FIXTURES and must NEVER be considered authentic hardware evidence.
class MockZ1Transport final : public IZ1Transport {
public:
    TransportKind kind() const noexcept override { return TransportKind::SyntheticMock; }
    void injectSynthetic(Snapshot decoded, Time when);
    std::optional<ReceivedFrame> popFrame() override;
    void emit(const JointCommand& command) override;
    void disarm() noexcept override;
    std::vector<JointCommand> recordedCommands() const;
    std::size_t pendingFrames() const;
    bool softwareDisarmed() const;
    void failNextEmit();
private:
    mutable std::mutex mutex_;
    std::deque<ReceivedFrame> queue_;
    std::vector<JointCommand> commands_;
    std::uint64_t next_index_ = 1;
    bool disarmed_ = false;
    bool fail_next_emit_ = false;
    static constexpr std::size_t kMaxFrames = 32;
};

// Advisory local host lock. Only cooperating processes honor this lock.
// DOES NOT establish exclusivity against the official z1_ctrl or other clients.
class CooperativeOwnerLease final {
public:
    explicit CooperativeOwnerLease(const std::string& lock_file);
    ~CooperativeOwnerLease();
    CooperativeOwnerLease(const CooperativeOwnerLease&) = delete;
    CooperativeOwnerLease& operator=(const CooperativeOwnerLease&) = delete;
    bool held() const noexcept { return fd_ >= 0; }
private:
    int fd_ = -1;
};

// Mock-only runtime intentionally rejects a VendorDisabledTransport.
// Leases, feedback ingestion, gate checks and sink outputs use existing M2.4d1/c3.
class MockOnlyRuntime final {
public:
    MockOnlyRuntime(IZ1Transport& transport,
                    b2z1::z1_jointctrl::Config safety,
                    b2z1::m24d1::Limits rx,
                    const std::string& cooperative_lock_path);
    MockOnlyRuntime(const MockOnlyRuntime&) = delete;
    MockOnlyRuntime& operator=(const MockOnlyRuntime&) = delete;
    ~MockOnlyRuntime();
    bool pumpOne(Time now); // receive one synthetic frame, if queued
    void authorizeForOfflineTest(Time now); // TEST ONLY, not physical approvals
    void setIkGoal(const b2z1::z1_jointctrl::Joint6& goal, Time now);
    bool tick(Time now);
    void disarm() noexcept;
    bool authorized() const;
    bool faulted() const;
    std::size_t sentCount() const;
    std::string faultReason() const;
private:
    class TransportSink final : public b2z1::m24c3::CommandSink {
    public:
        explicit TransportSink(IZ1Transport& t) : transport_(t) {}
        void emit(const JointCommand& c) override { transport_.emit(c); }
    private:
        IZ1Transport& transport_;
    };
    // Destruction order ensures gate references remain valid.
    CooperativeOwnerLease owner_;
    IZ1Transport& transport_;
    TransportSink sink_;
    b2z1::m24d1::TransportSession gate_;
};
} // namespace b2z1::m24d4
