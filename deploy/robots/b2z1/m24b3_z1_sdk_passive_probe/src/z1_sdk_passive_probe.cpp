// M2.4b3 gated candidate: independent SDK feedback DIAGNOSTICS, not motor control.
// WARNING: unitreeArm::sendRecv() sends an SDK UDP command while receiving state.
// It is NOT a physically passive monitor. Only run after electrical inspection.
#include "z1_feedback_quality.h"
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace b2z1::m24b3;

namespace {
constexpr std::size_t kNumJoints = 6;

void showUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " --help | --bench-sdk-passive-observation\n"
              << "IMPORTANT: sendRecv() TRANSMITS UDP; this is NOT receive-only.\n"
              << "Real-hardware execution also requires B2Z1_Z1_PASSIVE_PROBE_ACK=PHYSICAL_INSPECTION_COMPLETE\n"
              << "and an independent electrical inspection and attended E-stop.\n";
}

Snapshot sampleFromSdk(UNITREE_ARM::unitreeArm& arm) {
    auto* state=arm.lowstate;
    auto* comp=arm._ctrlComp;
    if (!state || !comp || !comp->udp || state->q.size()<kNumJoints ||
        state->dq.size()<kNumJoints || state->tau.size()<kNumJoints ||
        state->temperature.size()<kNumJoints ||
        state->errorstate.size()<kNumJoints ||
        state->isMotorConnected.size()<kNumJoints)
        throw std::runtime_error("SDK returned incomplete Z1 state");
    Snapshot s{};
    // Exactly one thread owns sendRecv and reads lowstate. No vendor background
    // sendRecvThread is started, and no joint/motor control mode is selected.
    for (std::size_t j=0; j<kNumJoints; ++j) {
        s.q[j]=state->q[j];
        s.dq[j]=state->dq[j];
        s.tau[j]=state->tau[j];
        s.motor_error[j]=state->errorstate[j];
        s.motor_connection[j]=state->isMotorConnected[j];
        s.temperature[j]=state->temperature[j];
    }
    s.sdk_disconnected=comp->udp->isDisConnect;
    s.sdk_state_passive = comp->recvState.state == UNITREE_ARM::ArmFSMState::PASSIVE;
    s.sampled_at=Clock::now();
    // Don't turn a successful sendRecv() call into a fabricated packet proof.
    s.independently_confirmed_new_packet=false;
    return s;
}
}

int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1],"--help")==0) {
        showUsage(argv[0]);
        return 0;
    }
    if (argc!=2 || std::strcmp(argv[1],"--bench-sdk-passive-observation")!=0) {
        showUsage(argv[0]);
        return 2;
    }
    const char* ack=std::getenv("B2Z1_Z1_PASSIVE_PROBE_ACK");
    if (!ack || std::strcmp(ack,"PHYSICAL_INSPECTION_COMPLETE")!=0) {
        std::cerr << "BLOCKED: No verified bench electrical inspection acknowledgement.\n";
        return 3;
    }
    std::cerr << "WARNING: SDK sendRecv() can TRANSMIT command packets.\n"
              << "Do not run this monitor concurrently with any Z1 client.\n";
    try {
        // Vendor SDK constructor is after all runtime authorization checks.
        UNITREE_ARM::unitreeArm arm(false);
        constexpr int n=20;
        for (int i=0; i<n; ++i) {
            // This method both sends AND receives UDP. We never switch modes.
            arm.sendRecv();
            const Snapshot s=sampleFromSdk(arm);
            const Quality quality=inspect(s);
            std::cout << "sample=" << i << " sdk_connected=" << quality.sdk_reports_connected
                      << " passive=" << quality.controller_reports_passive
                      << " motor_ok=" << quality.motor_health_ok
                      << " values_valid=" << quality.values_valid
                      << " packet_proven=" << quality.eligible_for_policy_feedback
                      << " q=[" << std::fixed << std::setprecision(4);
            for(int j=0;j<6;++j) std::cout << (j?",":"") << s.q[j];
            std::cout << "] dq=[";
            for(int j=0;j<6;++j) std::cout << (j?",":"") << s.dq[j];
            std::cout << "] tau=[";
            for(int j=0;j<6;++j) std::cout << (j?",":"") << s.tau[j];
            std::cout << "] temperature=[";
            for(int j=0;j<6;++j) std::cout << (j?",":"") << s.temperature[j];
            std::cout << "] err=[";
            for(int j=0;j<6;++j) std::cout << (j?",":"") << int(s.motor_error[j]);
            std::cout << "] conn_code=[";
            for(int j=0;j<6;++j) std::cout << (j?",":"") << int(s.motor_connection[j]);
            std::cout << "] note=\"" << quality.diagnostic << "\"\n";
            if (!quality.values_valid || !quality.motor_health_ok ||
                !quality.sdk_reports_connected || !quality.controller_reports_passive) {
                std::cerr << "ABORT: invalid state / not PASSIVE / health flag.\n";
                return 4;
            }
            // Short, finite bench probe only; not a perpetual command publisher.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cerr << "DONE. No JOINTCTRL or LOWCMD transition requested. "
                  << "New packet freshness remains UNVERIFIED.\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr << "SDK diagnostic failed: " << e.what() << "\n";
        return 5;
    }
}
