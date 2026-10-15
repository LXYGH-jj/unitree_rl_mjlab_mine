#pragma once
// External SDK service bridge; start/exchange/stop can affect PHYSICAL Z1 motors.
#include "z1_lowcmd_plan.h"
#include <memory>
namespace m24e2b {
class SdkBridge final {
public:
    explicit SdkBridge(bool has_gripper);
    ~SdkBridge();
    SdkBridge(const SdkBridge&)=delete;
    SdkBridge& operator=(const SdkBridge&)=delete;
    // PHYSICAL IO: don't call before reviewed bench procedure; SDK sendRecv is TX+RX.
    void start();
    // Captured at startup using SDK getter, NOT proven new UDP receive time.
    Feedback initialFeedback() const;
    // Must be called by single thread at SDK cadence, not concurrently with vendor thread.
    Feedback exchange(const Command& command);
    bool started() const noexcept;
    // Best-effort LOWCMD -> JOINTCTRL -> PASSIVE handoff, no return-to-home motion.
    // Requires a mechanically supported arm. False means PASSIVE was NOT confirmed.
    // Never substitutes for a physical E-stop, especially if SDK UDP is lost.
    bool stop() noexcept;
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}
