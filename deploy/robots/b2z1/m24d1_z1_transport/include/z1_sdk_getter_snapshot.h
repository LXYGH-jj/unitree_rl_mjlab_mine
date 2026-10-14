#pragma once
#include "z1_rx_validation.h"
#include <unitree_arm_sdk/control/unitreeArm.h>
namespace b2z1::m24d1 {
// ABI-bound copy only. NEVER creates SDK, opens network, or calls sendRecv.
// May only be called under a separately demonstrated SDK data synchronization
// contract. Getter polling NEVER provides receiver origin or freshness proof.
Snapshot copySdkGetterSnapshotUnverified(const UNITREE_ARM::unitreeArm& arm,Time sampled_at);
// M2.4d1 does not possess the vendor library's private recv callback. No
// physical SDK command sink is implemented or linked by this module.
constexpr bool kRealVendorRxProofAvailable = false;
constexpr bool kRealVendorCommandOutputEnabled = false;
} // namespace b2z1::m24d1
