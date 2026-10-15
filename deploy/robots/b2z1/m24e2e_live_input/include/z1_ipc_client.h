#pragma once
// M2.4e2e: Unix SOCK_SEQPACKET V1 client. NO vendor SDK/DDS/motor code.
// GET timestamp is response time, NOT proven SDK RX time.
#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>

namespace m24e2e {
using Clock = std::chrono::steady_clock;
using Vec6 = std::array<double, 6>;
enum class Mode { Mock, SdkUnqualified };
struct ArmState {
    Mode mode{Mode::Mock};
    Vec6 q{}, dq{};
    std::uint64_t response_ns{}, last_target_seq{};
    std::int64_t target_age_ms{};
    bool tracking{};
    Clock::time_point ipc_received_at{}; // IPC response received by THIS client
};

// A single thread must own a Session. No concurrent calls.
class Session final {
public:
    Session() = default;
    ~Session();
    Session(const Session&)=delete;
    Session& operator=(const Session&)=delete;
    void connectTo(const std::string& socket_path, Mode expected, int timeout_ms=40);
    ArmState get(std::uint64_t request_id);
    void target(std::uint64_t seq, const Vec6& q, bool enable);
    void close() noexcept;
private:
    std::string transact(const std::string& data);
    int fd_{-1};
    Mode mode_{Mode::Mock};
};

// One transport thread; main FSM reads cache only (nonblocking).
// NO auto reconnect after failures; explicit restart required.
// SDK-target sending deliberately not implemented until bench stop semantics qualified.
class ReadOnlyPoller final {
public:
    ReadOnlyPoller()=default;
    ~ReadOnlyPoller();
    ReadOnlyPoller(const ReadOnlyPoller&)=delete;
    ReadOnlyPoller& operator=(const ReadOnlyPoller&)=delete;
    void start(const std::string& path, Mode expected, int timeout_ms=40);
    void stop() noexcept;
    bool latest(ArmState& out, std::string& error) const;
private:
    void loop(std::string path, Mode expected, int timeout_ms) noexcept;
    std::thread thread_{};
    std::atomic<bool> stop_{false};
    mutable std::mutex mu_{};
    ArmState latest_{};
    bool available_{false};
    std::string error_{"NOT_STARTED"};
};
}
