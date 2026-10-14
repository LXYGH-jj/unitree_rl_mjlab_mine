#pragma once

#include "z1_jointctrl_safety.h"

#include <memory>

#include <unitree_arm_sdk/control/unitreeArm.h>

namespace b2z1::z1_jointctrl {

// Draft SDK boundary, deliberately NOT activated by the M2.4b2 offline build.
// Vendor JOINTCTRL owns the motor servo; the user's program supplies q, dq.
// SDK's sendRecvThread is the ONLY candidate sender/receiver when JOINTCTRL is used.
class VendorSdkCandidate final {
public:
    explicit VendorSdkCandidate(bool has_gripper);
    ~VendorSdkCandidate();

    VendorSdkCandidate(const VendorSdkCandidate&) = delete;
    VendorSdkCandidate& operator=(const VendorSdkCandidate&) = delete;

    // M2.4b2 implementation refuses real sockets/control by design.
    void connectReadOnly();
    void enterJointCtrl();
    Feedback copyFeedbackUnverified() const;
    void sendJointCtrlCommand(const JointCommand& command);

private:
    bool has_gripper_ = false;
    std::unique_ptr<UNITREE_ARM::unitreeArm> arm_;
};

} // namespace b2z1::z1_jointctrl
