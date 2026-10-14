#include "z1_jointctrl_sdk_candidate.h"
#include <iostream>
#include <stdexcept>

int main() {
    using namespace b2z1::z1_jointctrl;
    VendorSdkCandidate candidate(false);
    int blocks = 0;
    try { candidate.connectReadOnly(); } catch(const std::runtime_error&) {++blocks;}
    try { candidate.enterJointCtrl(); } catch(const std::runtime_error&) {++blocks;}
    try { candidate.copyFeedbackUnverified(); } catch(const std::runtime_error&) {++blocks;}
    try { candidate.sendJointCtrlCommand(JointCommand{}); } catch(const std::runtime_error&) {++blocks;}
    if (blocks != 4) throw std::runtime_error("hardware must remain disabled");
    std::cout << "M2.4b2 official SDK candidate: PASS (four I/O entry points blocked)\n";
}
