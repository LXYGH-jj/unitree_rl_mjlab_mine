#pragma once
// External SDK service bridge; NO executable in this milestone invokes it.
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
    // Must be called by single thread at SDK cadence, not concurrently with vendor thread.
    Feedback exchange(const Command& command);
    bool started() const noexcept;
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}
