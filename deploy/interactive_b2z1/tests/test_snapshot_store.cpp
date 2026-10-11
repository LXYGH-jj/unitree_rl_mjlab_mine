#include "interactive_snapshot_store.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace interactive_b2z1;

namespace {
void require(bool cond, const char* detail) {
    if (!cond) throw std::runtime_error(detail);
}
template <typename F> void reject(F&& fun, const char* detail) {
    bool rejected = false;
    try { fun(); } catch (const std::exception&) { rejected = true; }
    require(rejected, detail);
}
} // namespace

int main() {
    try {
        SnapshotStore store;
        auto now = MonoClock::now();
        Joint12 bq{}, bdq{};
        Arm6 aq{}, adq{};
        for (std::size_t i=0;i<12;++i) { bq[i]=float(i); bdq[i]=0.2f*float(i); }
        for (std::size_t i=0;i<6;++i) { aq[i]=float(12+i); adq[i]=0.3f*float(i); }
        const std::array<float,4> quat{1.f,0.f,0.f,0.f};
        const Command3 gyro{0.f,0.f,0.f};
        const Command3 cmd{0.2f,0.f,0.f};
        const Command3 goal{0.4f,0.f,0.35f};

        reject([&]{ adaptFeedback(store.copy(cmd,goal),now); }, "empty streams must reject");
        const auto b2_t=now-std::chrono::milliseconds(10);
        const auto z1_t=now-std::chrono::milliseconds(8);
        store.updateB2(bq,bdq,quat,gyro,b2_t);
        reject([&]{ adaptFeedback(store.copy(cmd,goal),now); }, "Z1 missing must reject");
        store.updateZ1(aq,adq,z1_t);
        const auto snap=store.copy(cmd,goal);
        const auto adapted=adaptFeedback(snap,now);
        require(adapted.robot.joint_q[0] == 3.f, "B2 motor mapping wrong");
        require(adapted.robot.joint_q[12] == 12.f, "Z1 index wrong");
        require(std::abs(adapted.robot.ee_local_xyz[0] - .2f)<1e-6f, "EE frame wrong");
        reject([&]{store.updateB2(bq,bdq,quat,gyro,b2_t);}, "duplicate B2 timestamp");
        reject([&]{store.updateZ1(aq,adq,z1_t-std::chrono::milliseconds(1));}, "older Z1 timestamp");
        reject([&]{store.updateZ1(aq,adq,MonoTime{});}, "invalid time");
        reject([&]{adaptFeedback(store.copy(cmd,goal),now+std::chrono::milliseconds(120));},
               "stale streams must reject");
        store.reset();
        reject([&]{adaptFeedback(store.copy(cmd,goal),now);},"reset must invalidate");
        std::cout << "M1.4c SNAPSHOT STORE: PASS\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "M1.4c SNAPSHOT STORE: FAIL: " << e.what() << '\n';
        return 1;
    }
}
