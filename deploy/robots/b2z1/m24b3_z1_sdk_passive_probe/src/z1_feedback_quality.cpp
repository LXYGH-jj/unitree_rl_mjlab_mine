#include "z1_feedback_quality.h"
#include <cmath>

namespace b2z1::m24b3 {
Quality inspect(const Snapshot& s) {
    Quality o{};
    o.values_valid = s.sampled_at != Clock::time_point{};
    o.motor_health_ok = true;
    for (int i=0; i<6; ++i) {
        o.values_valid &= std::isfinite(s.q[i]) && std::isfinite(s.dq[i]) &&
                          std::isfinite(s.tau[i]);
        // Report every non-zero raw vendor fault (including 0x40); no silent ignore.
        o.motor_health_ok &= s.motor_error[i] == 0 && s.motor_connection[i] == 0;
    }
    o.sdk_reports_connected = !s.sdk_disconnected;
    o.controller_reports_passive = s.sdk_state_passive;
    // Getters can be stale even when values are finite and the SDK health flag
    // is good. This tool does not establish an independent new-datagram proof.
    o.eligible_for_policy_feedback = false;
    if (!o.values_valid) o.diagnostic = "invalid/NaN/Inf/incomplete sample";
    else if (!o.motor_health_ok) o.diagnostic = "motor error or connection code present";
    else if (!o.sdk_reports_connected) o.diagnostic = "SDK reports disconnection";
    else if (!o.controller_reports_passive) o.diagnostic = "controller is not PASSIVE";
    else o.diagnostic = "locally observed; independent UDP freshness UNVERIFIED";
    return o;
}
} // namespace b2z1::m24b3
