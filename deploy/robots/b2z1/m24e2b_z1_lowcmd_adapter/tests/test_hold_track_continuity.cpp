// Standalone unit test. No Unitree SDK, sockets, or motor I/O.
#include "z1_lowcmd_plan.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace m24e2b;

static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
static bool same(const Vec6& a, const Vec6& b, double tol=1e-12) {
    for(int i=0;i<6;i++) if(std::abs(a[i]-b[i])>tol) return false;
    return true;
}
static Feedback feedback(Vec6 joints, Time at) {
    Feedback f{};
    f.q=joints;
    f.read_at=at;
    f.sdk_connected=true;
    f.lowcmd_reported=true;
    f.temperature.fill(30);
    return f;
}
static Limits benchLimits() {
    Limits l{};
    l.lo={-2.618,0.,-2.8798,-1.5184,-1.3439,-2.7925};
    l.hi={2.618,2.9671,0.,1.5184,1.3439,2.7925};
    l.velocity_max.fill(0.10);
    l.acceleration_max.fill(0.20);
    l.temperature_max.fill(60);
    l.target_ttl=std::chrono::milliseconds(150);
    return l;
}

int main() {
    try {
        constexpr double dt=0.0025;
        const auto delta=std::chrono::microseconds(2500);
        auto now=Clock::now();
        const Vec6 startup{-0.001,0.01,-0.001,-0.08,-0.064,0.};
        Planner planner(benchLimits());
        planner.reset(feedback(startup,now),now);
        require(same(planner.commandPosition(),startup),"reset must anchor measured startup");

        // Reproduce gravity settling in HOLD: SDK-measured J3 moves -0.0068
        // but our low-level command MUST remain at the original reference.
        Vec6 measured=startup;
        measured[2]-=0.006814;
        for(int i=0;i<100;i++) {
            now+=delta;
            const auto hold=planner.tick(feedback(measured,now),now,dt);
            require(!hold.track,"HOLD unexpectedly armed");
            require(same(hold.q,startup),"HOLD drifted with measured joints");
            require(same(hold.dq,Vec6{}),"HOLD velocity must be zero");
        }

        // ARM with EXACT commanded HOLD reference, not the measured joints.
        Target activate{1,now,startup,true};
        planner.acceptTarget(activate,now);
        now+=delta;
        auto first=planner.tick(feedback(measured,now),now,dt);
        require(first.track,"ARM did not start TRACK");
        require(same(first.q,startup),"HOLD->TRACK position command jumped");
        require(same(first.dq,Vec6{}),"ARM with unchanged target must have zero command velocity");

        // Move J5 by 0.005 rad on a bounded trajectory; refresh watchdog.
        Vec6 goal=startup;
        goal[4]+=0.005;
        std::uint64_t seq=2;
        double max_speed=0.0;
        for(int i=0;i<280;i++){
            if(i%10==0){
                planner.acceptTarget(Target{seq++,now,goal,true},now);
            }
            now+=delta;
            const auto cmd=planner.tick(feedback(measured,now),now,dt);
            require(cmd.track,"TRACK watchdog expired during regression");
            require(std::abs(cmd.q[4]-goal[4])<=0.005001,"J5 target overshoot");
            for(int j=0;j<6;j++)
                max_speed=std::max(max_speed,std::abs(cmd.dq[j]));
            require(max_speed<=0.100000001,"planner velocity exceeded limit");
            measured[4]=cmd.q[4]; // mock follows J5, J3 keeps gravity offset
        }
        require(std::abs(planner.commandPosition()[4]-goal[4])<1e-8,
                "J5 planned target never reached");

        // DISARM while measured position differs from command. Keep last
        // position command (no sudden q jump), command zero velocity.
        const Vec6 before=planner.commandPosition();
        measured[4]-=0.002;
        planner.acceptTarget(Target{seq++,now,goal,false},now);
        now+=delta;
        const auto disarmed=planner.tick(feedback(measured,now),now,dt);
        require(!disarmed.track,"DISARM must enter HOLD");
        require(same(disarmed.q,before),"TRACK->HOLD position command jumped");
        require(same(disarmed.dq,Vec6{}),"DISARM must send zero target speed");
        for(int i=0;i<30;i++){
            now+=delta;
            auto stay=planner.tick(feedback(measured,now),now,dt);
            require(same(stay.q,before),"post-DISARM HOLD drifted");
        }

        // Strict J3 boundary conflict: slightly positive initial feedback is
        // admissible for observation/HOLD but cannot be an active TARGET.
        now=Clock::now();
        Vec6 outside=startup;
        outside[2]=+0.000643092; // captured Z1 HOLD CSV initial qcmd
        Planner outside_planner(benchLimits());
        outside_planner.reset(feedback(outside,now),now);
        now+=delta;
        auto hold_out=outside_planner.tick(feedback(outside,now),now,dt);
        require(same(hold_out.q,outside),"HOLD improperly auto-clamped out-of-limit initial pose");
        bool rejected=false;
        try {
            outside_planner.acceptTarget(Target{1,now,outside,true},now);
        } catch (const std::exception&) { rejected=true; }
        require(rejected,"out-of-limit J3 target was not rejected");
        std::cout<<"OFFLINE PASS: HOLD stays fixed across feedback drift; "
                    "HOLD->TRACK has no q jump; TRACK->HOLD preserves q; "
                    "J5 target converges; strict J3 target boundary enforced.\n";
        std::cout<<"NO physical SDK or motors were used.\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"OFFLINE FAIL: "<<e.what()<<'\n';
        return 1;
    }
}
