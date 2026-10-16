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
        double previous_q5=startup[4];
        for(int i=0;i<280;i++){
            if(i%10==0){
                planner.acceptTarget(Target{seq++,now,goal,true},now);
            }
            now+=delta;
            const auto cmd=planner.tick(feedback(measured,now),now,dt);
            require(cmd.track,"TRACK watchdog expired during regression");
            require(cmd.q[4]>=previous_q5-1e-11,
                    "Fixed J5 goal produced backwards planned movement");
            require(cmd.q[4]<=goal[4]+1e-11,
                    "Fixed J5 goal was overshot");
            previous_q5=cmd.q[4];
            for(int j=0;j<6;j++)
                max_speed=std::max(max_speed,std::abs(cmd.dq[j]));
            require(max_speed<=0.100000001,"planner velocity exceeded limit");
            measured[4]=cmd.q[4]; // mock follows J5, J3 keeps gravity offset
        }
        require(std::abs(planner.commandPosition()[4]-goal[4])<1e-8,
                "J5 planned target never reached");
        // Previously velocity-only target tracking repeatedly crossed goal.
        // Holding a fixed target for another 0.25 s must never move qcmd.
        for(int i=0;i<100;i++){
            if(i%10==0)planner.acceptTarget(Target{seq++,now,goal,true},now);
            now+=delta;
            const auto settled=planner.tick(feedback(measured,now),now,dt);
            require(std::abs(settled.q[4]-goal[4])<1e-10,
                    "Settled J5 command oscillates without a new target");
            require(std::abs(settled.dq[4])<1e-10,
                    "Settled J5 commanded velocity not zero");
        }

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

        // Reproduce BOTH near-zero offsets from the real 1016 SDK log.
        // J2 starts -0.00001180 below lower bound 0, J3 starts
        // +0.000365795 above upper bound 0. Neither may move on ARM/J5 jog.
        now=Clock::now();
        Vec6 near_zero=startup;
        near_zero[1]=-0.00001180;
        near_zero[2]=+0.000365795;
        Planner start_planner(benchLimits());
        start_planner.reset(feedback(near_zero,now),now);
        now+=delta;
        const auto start_hold=start_planner.tick(feedback(near_zero,now),now,dt);
        require(same(start_hold.q,near_zero),"startup HOLD moved J3");

        // Repeating exactly that J3 reference is the ONLY allowed
        // above-limit command. Other joints may still be controlled.
        require(start_planner.targetAllowed(1,near_zero[1]),
                "startup J2 frozen anchor should be admitted");
        require(start_planner.targetAllowed(2,near_zero[2]),
                "startup J3 frozen anchor should be admitted");
        require(!start_planner.targetAllowed(1,near_zero[1]-0.00001),
                "J2 outward change must not be admissible");
        require(!start_planner.targetAllowed(2,near_zero[2]+0.00001),
                "J3 outward change must not be admissible");
        require(start_planner.targetAllowed(1,0.),
                "J2 inward goal to 0 must remain permitted");
        require(start_planner.targetAllowed(2,0.),
                "J3 inward goal to 0 must remain permitted");
        Vec6 bench_j5=near_zero;
        bench_j5[4]+=0.005;
        start_planner.acceptTarget(Target{1,now,bench_j5,true},now);
        auto near_feedback=near_zero;
        for(int i=0;i<300;i++){
            now+=delta;
            if(i%20==0 && i>0)
                start_planner.acceptTarget(Target{static_cast<std::uint64_t>(i/20+1),
                                                  now,bench_j5,true},now);
            const auto cmd=start_planner.tick(feedback(near_feedback,now),now,dt);
            require(cmd.track,"near-zero J3/J5 TRACK not enabled");
            require(std::abs(cmd.q[1]-near_zero[1])<1e-10,
                    "J5 jog caused commanded J2 movement");
            require(std::abs(cmd.q[2]-near_zero[2])<1e-10,
                    "J5 jog caused commanded J3 movement");
            require(cmd.q[4]<=bench_j5[4]+1e-10,
                    "J5 exceeded its position target");
            near_feedback[4]=cmd.q[4];
        }
        require(std::abs(start_planner.commandPosition()[4]-bench_j5[4])<1e-8,
                "J5 did not reach goal with near-zero J3 startup");

        // Both +/-0.002 rad are beyond the 0.0015 rad startup exception.
        now=Clock::now();
        Vec6 far_out=startup;
        far_out[1]=-0.002;
        far_out[2]=+0.002;
        Planner far_planner(benchLimits());
        far_planner.reset(feedback(far_out,now),now);
        require(!far_planner.targetAllowed(1,far_out[1]),
                "J2 <-0.0015 startup should not be eligible for ARM");
        require(!far_planner.targetAllowed(2,far_out[2]),
                "J3 >+0.0015 startup should not be eligible for ARM");
        bool rejected=false;
        try {
            far_planner.acceptTarget(Target{1,now,far_out,true},now);
        } catch(const std::exception&) {rejected=true;}
        require(rejected,"J3 far outside 0 upper bound was accepted");

        // Even with an eligible startup, moving J3 farther positive must
        // remain forbidden; it cannot create a new positive travel range.
        now=Clock::now();
        Planner outward_planner(benchLimits());
        outward_planner.reset(feedback(near_zero,now),now);
        Vec6 moved_outward=near_zero;
        moved_outward[1]-=0.00005;
        moved_outward[2]+=0.00005;
        rejected=false;
        try {
            outward_planner.acceptTarget(Target{1,now,moved_outward,true},now);
        } catch(const std::exception&) {rejected=true;}
        require(rejected,"J2/J3 outward request wrongly accepted");

        // Explicitly reject each outward request independently.
        for (int j : {1,2}) {
            now=Clock::now();
            Planner outward(benchLimits());
            outward.reset(feedback(near_zero,now),now);
            Vec6 bad=near_zero;
            bad[j]+=(j==1?-0.00005:+0.00005);
            rejected=false;
            try {outward.acceptTarget(Target{1,now,bad,true},now);}
            catch(const std::exception&) {rejected=true;}
            require(rejected,"one-sided J2 or J3 outward change was accepted");
        }
        std::cout<<"OFFLINE PASS: HOLD stays fixed across feedback drift; "
                    "HOLD->TRACK has no q jump; TRACK->HOLD preserves q; "
                    "J5 target converges; near-zero J2 and J3 frozen while J5 moves; "
                    "outward commands rejected.\n";
        std::cout<<"NO physical SDK or motors were used.\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"OFFLINE FAIL: "<<e.what()<<'\n';
        return 1;
    }
}
