// Compile-only SDK API contract: no constructor, no sockets, no JOINTCTRL.
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <type_traits>
#include <utility>
using Arm=UNITREE_ARM::unitreeArm;
static_assert(std::is_same_v<decltype(std::declval<Arm&>().startTrack(
    UNITREE_ARM::ArmFSMState::JOINTCTRL)),void>, "vendor startTrack changed");
static_assert(std::is_same_v<decltype(std::declval<Arm&>().setArmCmd(
    std::declval<Vec6>(),std::declval<Vec6>())),void>, "vendor setArmCmd changed");
static_assert(std::is_same_v<decltype(std::declval<Arm&>().sendRecvThread->start()),void>,
              "vendor sendRecvThread::start changed");
// IMPORTANT: No main; compile with -fsyntax-only. No method is invoked.
