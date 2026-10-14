#include "z1_feedback_quality.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace b2z1::m24b3;
int main() {
    Snapshot s{};
    s.sampled_at = Clock::now();
    s.sdk_disconnected=false;
    s.sdk_state_passive=true;
    Quality q = inspect(s);
    assert(q.values_valid && q.motor_health_ok && q.sdk_reports_connected);
    assert(q.controller_reports_passive && !q.eligible_for_policy_feedback);
    s.independently_confirmed_new_packet=true; // unsupported user assertion MUST NOT unlock it
    assert(!inspect(s).eligible_for_policy_feedback);
    s.q[2]=std::numeric_limits<double>::quiet_NaN();
    assert(!inspect(s).values_valid);
    s.q[2]=0;
    s.motor_error[3]=0x04;
    assert(!inspect(s).motor_health_ok);
    s.motor_error[3]=0;
    s.motor_connection[1]=2;
    assert(!inspect(s).motor_health_ok);
    s.motor_connection[1]=0;
    s.sdk_disconnected=true;
    assert(!inspect(s).sdk_reports_connected);
    s.sdk_disconnected=false;
    s.sdk_state_passive=false;
    assert(!inspect(s).controller_reports_passive);
    s.sdk_state_passive=true;
    s.sampled_at={};
    assert(!inspect(s).values_valid);
    std::cout << "M2.4b3 SDK feedback quality: PASS (offline mock; NO robot IO)\n";
}
