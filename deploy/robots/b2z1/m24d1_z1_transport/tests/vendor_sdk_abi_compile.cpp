#include "z1_sdk_getter_snapshot.h"
#include <type_traits>
#include <utility>
using Arm=UNITREE_ARM::unitreeArm;
static_assert(std::is_same_v<decltype(std::declval<Arm&>().startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL)),void>);
static_assert(std::is_same_v<decltype(std::declval<Arm&>().setArmCmd(std::declval<Vec6>(),std::declval<Vec6>())),void>);
static_assert(std::is_pointer_v<decltype(std::declval<Arm&>().sendRecvThread)>);
static_assert(!b2z1::m24d1::kRealVendorRxProofAvailable);
static_assert(!b2z1::m24d1::kRealVendorCommandOutputEnabled);
