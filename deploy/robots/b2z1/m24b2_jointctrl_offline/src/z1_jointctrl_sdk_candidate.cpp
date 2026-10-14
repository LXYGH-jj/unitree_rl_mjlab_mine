#include "z1_jointctrl_sdk_candidate.h"

#include <stdexcept>
#include <type_traits>
#include <utility>

namespace b2z1::z1_jointctrl {
namespace {
// Verify that the vendor SDK API required by the intended JOINTCTRL design
// exists in this repository. The official highcmd_development.cpp uses:
// arm.sendRecvThread->start(); arm.startTrack(JOINTCTRL);
// arm.setArmCmd(q,qd); with ONLY the vendor background communication thread.
using Arm = UNITREE_ARM::unitreeArm;
static_assert(std::is_same_v<
    decltype(std::declval<Arm&>().startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL)),
    void>, "SDK startTrack(JOINTCTRL) contract changed");
static_assert(std::is_same_v<
    decltype(std::declval<Arm&>().setArmCmd(std::declval<Vec6>(),
                                           std::declval<Vec6>(),
                                           std::declval<Vec6>())),
    void>, "SDK setArmCmd(q, dq, tau) contract changed");
static_assert(std::is_same_v<
    decltype(std::declval<Arm&>().sendRecv()),
    void>, "SDK sendRecv() contract changed");
} // namespace

VendorSdkCandidate::VendorSdkCandidate(bool has_gripper) : has_gripper_(has_gripper) {}
VendorSdkCandidate::~VendorSdkCandidate() = default;

void VendorSdkCandidate::connectReadOnly() {
    // Merely constructing unitreeArm or starting its communication thread may
    // open a control channel; don't mislabel this as a passive-only probe.
    throw std::runtime_error(
        "M2.4b2 offline-only: vendor SDK connection disabled; no robot I/O");
}

void VendorSdkCandidate::enterJointCtrl() {
    throw std::runtime_error(
        "M2.4b2 offline-only: JOINTCTRL transition blocked until bench review");
}

Feedback VendorSdkCandidate::copyFeedbackUnverified() const {
    // No method is provided that fabricates 'confirmed_new_packet=true'.
    throw std::runtime_error(
        "M2.4b2 offline-only: live packet freshness not verified");
}

void VendorSdkCandidate::sendJointCtrlCommand(const JointCommand&) {
    throw std::runtime_error(
        "M2.4b2 offline-only: physical motion command blocked");
}

} // namespace b2z1::z1_jointctrl
