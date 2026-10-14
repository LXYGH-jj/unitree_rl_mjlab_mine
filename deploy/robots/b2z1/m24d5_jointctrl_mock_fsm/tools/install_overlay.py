#!/usr/bin/env python3
"""M2.4d5 opt-in simulation FSM overlay. Refuses version skew/dirty target files."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

MODULE='m24d5_jointctrl_mock_fsm'
TARGETS=[
 'deploy/robots/b2z1/CMakeLists.txt',
 'deploy/robots/b2z1/main.cpp',
 'deploy/robots/b2z1/include/State_InteractiveArmIK.h',
 'deploy/robots/b2z1/src/State_InteractiveArmIK.cpp',
]
BASE_CFG='deploy/robots/b2z1/config/config_interactive_jointctrl_shadow_m24c2_sim.yaml'
NEW_CFG='deploy/robots/b2z1/config/config_interactive_jointctrl_mock_m24d5_sim.yaml'

def replace_once(data, old, new, name):
    if data.count(old)!=1: raise RuntimeError(f'{name}: expected one exact anchor, found {data.count(old)}: {old[:78]!r}')
    return data.replace(old,new,1)

def git(repo,*args):
    return subprocess.run(['git','-C',str(repo),*args],capture_output=True,text=True)

def content(repo):
    for f in TARGETS+[BASE_CFG]:
        if not (repo/f).is_file():raise RuntimeError('missing required file: '+f)
    if git(repo,'merge-base','--is-ancestor','8f94aff','HEAD').returncode!=0:
        raise RuntimeError('HEAD is not a descendant of verified M2.4d4 baseline 8f94aff')
    for f in TARGETS+[BASE_CFG]:
        if git(repo,'diff','--quiet','HEAD','--',f).returncode !=0:
            raise RuntimeError('tracked target differs from HEAD; preserve developer edits first: '+f)
    if (repo/'deploy/robots/b2z1'/MODULE).exists() or (repo/NEW_CFG).exists():
        raise RuntimeError('M2.4d5 destination exists; refusing overwrite')
    data={p:(repo/p).read_text() for p in TARGETS+[BASE_CFG]}
    if 'M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED' not in data['deploy/robots/b2z1/main.cpp']:
        raise RuntimeError('baseline pre-DDS real JOINTCTRL lock was not found')
    main='deploy/robots/b2z1/main.cpp'
    data[main]=replace_once(data[main], '#include "State_JointCtrlShadow.h"',
        '#include "State_JointCtrlShadow.h"\n#include "State_JointCtrlMockSim.h" // M2.4d5 opt-in MuJoCo ONLY',main)
    marker='    unitree::robot::ChannelFactory::Instance()->Init(0, network);'
    gate='''    // M2.4d5: simulation-only FSM config MUST be refused BEFORE any B2 DDS.
    const auto mock_sim = param::config["jointctrl_mock_sim"];
    if (mock_sim) {
        const auto legacy = param::config["z1_sdk"];
        if (!loopback_network ||
            !mock_sim["enabled"] || !mock_sim["enabled"].as<bool>() ||
            !mock_sim["sim_only"] || !mock_sim["sim_only"].as<bool>() ||
            !mock_sim["no_vendor_sdk_io"] || !mock_sim["no_vendor_sdk_io"].as<bool>() ||
            (legacy && legacy["enabled"] && legacy["enabled"].as<bool>())) {
            spdlog::critical("M2.4d5 refuses non-loopback or non-mock Z1 simulation configuration before DDS.");
            return 2;
        }
        const char* sim_ack = std::getenv("B2Z1_SIM_ONLY_ACK");
        if (!sim_ack || std::string(sim_ack) != "MUJOCO_LOOPBACK_NO_HARDWARE") {
            spdlog::critical("M2.4d5 requires explicit simulation-only ACK before DDS.");
            return 2;
        }
    }
'''
    data[main]=replace_once(data[main],marker,gate+marker,main)
    data[main]=replace_once(data[main], '#include "FSM/CtrlFSM.h"',
        '#include "FSM/CtrlFSM.h"\n#include <cstdlib>\n#include <string>',main)
    cm='deploy/robots/b2z1/CMakeLists.txt'
    data[cm]=replace_once(data[cm], '  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/include\n',
        '  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/include\n'
        '  ${PROJECT_SOURCE_DIR}/m24d4_z1_transport_contract/include\n'
        '  ${PROJECT_SOURCE_DIR}/m24d5_jointctrl_mock_fsm/include\n',cm)
    data[cm]=replace_once(data[cm],
        '  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/src/z1_transport_session.cpp\n',
        '  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/src/z1_transport_session.cpp\n'
        '  ${PROJECT_SOURCE_DIR}/m24d4_z1_transport_contract/src/z1_transport_contract.cpp\n'
        '  ${PROJECT_SOURCE_DIR}/m24d5_jointctrl_mock_fsm/src/m24d5_sim_gate.cpp\n',cm)
    h='deploy/robots/b2z1/include/State_InteractiveArmIK.h'
    data[h]=replace_once(data[h], '#include "interactive_pipeline.h"',
        '#include "interactive_pipeline.h"\n#include "m24d5_sim_gate.h"',h)
    data[h]=replace_once(data[h], 'class State_InteractiveArmIK final : public FSMState {',
        'class State_InteractiveArmIK : public FSMState {',h)
    data[h]=replace_once(data[h], '    bool has_walked_{false};',
        '    bool has_walked_{false};\n'
        '    bool mock_sim_mode_{false};\n'
        '    std::unique_ptr<b2z1::m24d5::SimGate> mock_sim_gate_{};',h)
    cpp='deploy/robots/b2z1/src/State_InteractiveArmIK.cpp'
    data[cpp]=replace_once(data[cpp], '#include <limits>', '#include <limits>\n#include <unistd.h> // mock-only cooperative lease filename',cpp)
    data[cpp]=replace_once(data[cpp],
        '    : FSMState(state, std::move(state_string)) {',
        '''    : FSMState(state, std::move(state_string)) {
    mock_sim_mode_ = (getStateString() == "JointCtrlMockSim");
    if (mock_sim_mode_) {
        const auto mc = param::config["jointctrl_mock_sim"];
        if (!mc || !mc["enabled"] || !mc["enabled"].as<bool>() ||
            !mc["sim_only"] || !mc["sim_only"].as<bool>() ||
            !mc["no_vendor_sdk_io"] || !mc["no_vendor_sdk_io"].as<bool>())
            throw std::runtime_error("M2.4d5 requires explicit mock-only simulation config");
        const auto real = param::config["z1_jointctrl"];
        if (real && real["enabled"] && real["enabled"].as<bool>())
            throw std::runtime_error("M2.4d5 refuses real Z1 JOINTCTRL");
    }''',cpp)
    data[cpp]=replace_once(data[cpp],
        '    clear_motion();\n}\n\nvoid State_InteractiveArmIK::enter()',
        '''    clear_motion();
    if(mock_sim_gate_) mock_sim_gate_->disarm();
}

void State_InteractiveArmIK::enter()''',cpp)
    data[cpp]=replace_once(data[cpp], '    capture_arm_goal(s);\n    commanded_ = s.leg_q;',
        '''    capture_arm_goal(s);
    if (mock_sim_mode_) {
        const auto qlo=arm_model_.getJointQMin(), qhi=arm_model_.getJointQMax();
        if(qlo.size()!=6 || qhi.size()!=6) throw std::runtime_error("M2.4d5 missing simulator joint limits");
        b2z1::m24d5::Joint6 lo{},hi{};
        for(int i=0;i<6;++i) {
            lo[i]=qlo[i]+.015; hi[i]=qhi[i]-.015;
            if(!(std::isfinite(lo[i]) && std::isfinite(hi[i]) && lo[i] < hi[i]))
                throw std::runtime_error("M2.4d5 invalid simulated joint limits");
        }
        const auto lease=std::string("/tmp/b2z1_m24d5_mock_")+
            std::to_string(static_cast<unsigned long>(::getuid()))+".lock";
        mock_sim_gate_=std::make_unique<b2z1::m24d5::SimGate>(lo,hi,lease);
    }
    commanded_ = s.leg_q;''',cpp)
    old='''    if(!arm_motion_locked_ && !fault_) {
        for(int i=0;i<6;++i)
            arm_cmd_[i]+=std::clamp(arm_goal_q_[i]-arm_cmd_[i],-max_arm_step_rad_,max_arm_step_rad_);
    }'''
    new='''    if (mock_sim_mode_) {
        if (!mock_sim_gate_ || arm_motion_locked_ || fault_)
            throw std::runtime_error("M2.4d5 mock gate unavailable or arm safety locked");
        b2z1::m24d5::Joint6 q{},dq{},goal{};
        for(int i=0;i<6;++i) {
            q[i]=sample.arm_q[i]; dq[i]=sample.arm_dq[i]; goal[i]=arm_goal_q_[i];
        }
        const auto next=mock_sim_gate_->step(sample.tick_ms,q,dq,goal,Clock::now());
        if(next) for(int i=0;i<6;++i) arm_cmd_[i]=(*next)[i];
        if ((motor_steps_ % 200u)==0u)
            spdlog::info("[M24d5] SIM mock gate sent={} measured_q0={:.4f} gated_target_q0={:.4f} (NO REAL Z1 IO)",
                mock_sim_gate_->commandsEmitted(),q[0],arm_cmd_[0]);
        // Below, the existing 20-slot MuJoCo LowCmd publisher transmits this
        // vetted MOCK q to the simulator. No Z1 vendor network call exists.
    } else if(!arm_motion_locked_ && !fault_) {
        for(int i=0;i<6;++i)
            arm_cmd_[i]+=std::clamp(arm_goal_q_[i]-arm_cmd_[i],-max_arm_step_rad_,max_arm_step_rad_);
    }'''
    data[cpp]=replace_once(data[cpp],old,new,cpp)
    data[cpp]=replace_once(data[cpp],
        '        if(fault_)hold_on_policy_fault();\n        else motor_step(state,now);',
        '        if(fault_) {\n            if(mock_sim_mode_) {publish_pending_=false;return;} // no publish after mock gate fault\n            hold_on_policy_fault(); // existing legacy simulator mode unchanged\n        } else motor_step(state,now);',cpp)
    data[cpp]=replace_once(data[cpp],
        'void State_InteractiveArmIK::exit() {\n    clear_motion();',
        'void State_InteractiveArmIK::exit() {\n    if(mock_sim_gate_) {mock_sim_gate_->disarm();mock_sim_gate_.reset();}\n    clear_motion();',cpp)
    cfg=data[BASE_CFG]
    cfg=replace_once(cfg,
        '    JointCtrlShadow:\n      id: 7\n      type: JointCtrlShadow',
        '    JointCtrlMockSim:\n      id: 7\n      type: JointCtrlMockSim',NEW_CFG)
    cfg=replace_once(cfg,'      JointCtrlShadow: RT + B.on_pressed',
        '      JointCtrlMockSim: RT + B.on_pressed',NEW_CFG)
    cfg=replace_once(cfg,
        '  JointCtrlShadow:\n    transitions:\n      FixStand: LT + up.on_pressed\n      Passive: LT + B.on_pressed\n      SlowLieDown: LT + down.on_pressed',
        '  JointCtrlMockSim:\n    transitions:\n      FixStand: LT + up.on_pressed\n      Passive: LT + B.on_pressed\n      SlowLieDown: LT + down.on_pressed',NEW_CFG)
    cfg+='''\n# M2.4d5: controls only the simulator's 20-slot LowCmd through MOCK safety.
# NEVER operate this config against a real robot or bridge to real hardware.
jointctrl_mock_sim:
  enabled: true
  sim_only: true
  no_vendor_sdk_io: true
'''
    data[NEW_CFG]=cfg
    return data

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('repo')
    gp=ap.add_mutually_exclusive_group(required=True)
    gp.add_argument('--check',action='store_true')
    gp.add_argument('--apply',action='store_true')
    args=ap.parse_args()
    repo=Path(args.repo).resolve()
    data=content(repo)
    print('M2.4d5 BASE: verified descendant of 8f94aff, targeted sources clean')
    print('ADD:',MODULE,'and',NEW_CFG)
    print('MODIFY ONLY:',', '.join(TARGETS))
    print('HARDWARE: existing pre-DDS Z1 JOINTCTRL lock preserved; mock requires lo+ACK')
    if args.check:
        print('DRY RUN: no changes');return
    package=Path(__file__).resolve().parent.parent
    destination=repo/'deploy/robots/b2z1'/MODULE
    # source is copied once; hard guard against accidental production patches
    shutil.copytree(package,destination,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    try:
        for path,text in data.items():
            target=repo/path
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(text)
    except Exception:
        # Roll back only our modifications; preserve originally clean HEAD blobs.
        for path in TARGETS:
            blob=git(repo,'show','HEAD:'+path)
            if blob.returncode==0:(repo/path).write_text(blob.stdout)
        (repo/NEW_CFG).unlink(missing_ok=True)
        shutil.rmtree(destination)
        raise
    print('M2.4d5 INSTALLED: new opt-in FSM alias JointCtrlMockSim; no real SDK path enabled')

if __name__=='__main__':
    try:main()
    except Exception as ex:print('M2.4d5 REFUSED:',ex,file=sys.stderr);sys.exit(2)
