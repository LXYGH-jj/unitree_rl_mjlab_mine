#!/usr/bin/env python3
"""E2g: native real FSM candidate; snapshot-on-DDS-callback; keyboard-only; dual hardware gates."""
import argparse, pathlib, shutil, subprocess, sys, yaml
ROOT=pathlib.Path(__file__).resolve().parents[1]
SUB=pathlib.Path('deploy/robots/b2z1')
SRCMAIN=SUB/'main.cpp'; CMAKE=SUB/'CMakeLists.txt'; FSM=pathlib.Path('deploy/include/FSM/FSMState.h')
DEST=SUB/'m24e2g_real_control'
CFG=SUB/'config/config_m24e2g_real_candidate.yaml'
BASE=SUB/'config/config_interactive_arm_ik_sim.yaml'

def replace_once(s,a,b,path):
    if b in s:return s
    if s.count(a)!=1:raise ValueError(f'{path}: expected exactly one anchor {a!r}; got {s.count(a)}')
    return s.replace(a,b,1)

def patch(files):
    main=files[SRCMAIN];cmake=files[CMAKE];fsm=files[FSM]
    main=replace_once(main,'#include "State_InteractiveArmRealPreview.h" // M2.4e2f preview ONLY',
       '#include "State_InteractiveArmRealPreview.h" // M2.4e2f preview ONLY\n#include "State_InteractiveArmReal.h" // M2.4e2g real candidate\n#include "StampedB2LowState.h"',SRCMAIN)
    main=replace_once(main,'    FSMState::lowstate = std::make_shared<LowState_t>();',
       '''    // E2g timestamped subscriber: the stamp is written on ACTUAL DDS receipt.
    // Old simulation/B2 defaults remain intact unless interactive_real.enabled.
    if (param::config["interactive_real"] &&
        param::config["interactive_real"]["enabled"] &&
        param::config["interactive_real"]["enabled"].as<bool>()) {
        FSMState::lowstate = std::make_shared<StampedB2LowState>();
    } else {
        FSMState::lowstate = std::make_shared<LowState_t>();
    }''',SRCMAIN)
    main=replace_once(main,'    unitree::robot::ChannelFactory::Instance()->Init(0, network);',
       '''    // E2g fail closed BEFORE DDS, stand commands and legacy SDK ownership.
    const auto real_cfg = param::config["interactive_real"];
    if (real_cfg && real_cfg["enabled"] && real_cfg["enabled"].as<bool>()) {
        const auto legacy=param::config["z1_sdk"];
        if (legacy && legacy["enabled"] && legacy["enabled"].as<bool>()) {
            spdlog::critical("E2g refuses old Z1 direct SDK bridge"); return 2;
        }
        if (!real_cfg["keyboard_only"] || !real_cfg["keyboard_only"].as<bool>()) {
            spdlog::critical("E2g keyboard_only=true required"); return 2;
        }
        if (real_cfg["enable_z1_sdk_targets"] && real_cfg["enable_z1_sdk_targets"].as<bool>()) {
            spdlog::critical("E2g Z1 real target output NOT BENCH QUALIFIED");return 2;
        }
        const bool b2out = real_cfg["enable_b2_output"] && real_cfg["enable_b2_output"].as<bool>();
        const char* ack=std::getenv("B2Z1_B2_ACTUATION_ACK");
        if (!loopback_network && (!b2out || !ack ||
             std::string(ack)!="B2_REAL_BENCH_CONFIRMED")) {
            spdlog::critical("E2g real network blocked: B2 actuation gate/bench ACK missing");return 2;
        }
        if (loopback_network && b2out) {
            spdlog::critical("E2g forbids B2 real-output gate on loopback");return 2;
        }
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, network);''',SRCMAIN)
    cmake=replace_once(cmake,'  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/include',
        '  ${PROJECT_SOURCE_DIR}/m24e2g_real_control/include\n  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/include',CMAKE)
    cmake=replace_once(cmake,'  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/src/State_InteractiveArmRealPreview.cpp',
        '  ${PROJECT_SOURCE_DIR}/m24e2g_real_control/src/State_InteractiveArmReal.cpp\n  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/src/State_InteractiveArmRealPreview.cpp',CMAKE)
    fsm=replace_once(fsm,'        lowstate->update();\n#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK',
        '''        lowstate->update();
        if(keyboard) keyboard->update();
        // E2g keyboard-only development: do not merge physical wireless controller
        // data into FSM transitions. Build a fresh remote image from keyboard only.
        const auto real=param::config["interactive_real"];
        if (real && real["enabled"] && real["enabled"].as<bool>() &&
            real["keyboard_only"] && real["keyboard_only"].as<bool>()) {
            unitree::common::REMOTE_DATA_RX key{};
            const std::string k=keyboard?keyboard->key():std::string{};
            if(k=="f") key.RF_RX.btn.components.up=1;
            if(k=="g") key.RF_RX.btn.components.X=1;
            if(k=="p") key.RF_RX.btn.components.B=1;
            if(k=="l") key.RF_RX.btn.components.down=1;
            lowstate->joystick.extract(key);
        } else {
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK''',FSM)
    fsm=replace_once(fsm,'        update_z1_diagnostic_toggle();\n#else',
       '        update_z1_diagnostic_toggle();\n#else',FSM) if False else fsm
    fsm=replace_once(fsm,'        KeyboardJoystick::update_joystick(lowstate->joystick, keyboard->key());\n        }\n#endif',
        '        KeyboardJoystick::update_joystick(lowstate->joystick, keyboard->key());\n        }\n#endif\n        }',FSM)
    fsm=replace_once(fsm,'        if(keyboard) {\n            keyboard->update();',
        '        if(keyboard) {',FSM)
    # Update was moved to the start of pre_run; do not call it twice in the no-SDK branch.
    fsm=replace_once(fsm,'            keyboard->update();\n            KeyboardJoystick::update_joystick',
        '            KeyboardJoystick::update_joystick',FSM)
    # Existing BUILD has SDK macro, so disable the SDK keyboard overlay in only-keyboard mode.
    fsm=replace_once(fsm,'            if(keyboard_joystick) keyboard_joystick->update(lowstate->joystick);',
        '''            if(keyboard_joystick) keyboard_joystick->update(lowstate->joystick);''',FSM) if False else fsm
    # On non-macro case the same branch will bypass joystick due to else above.
    fsm=replace_once(fsm,'    static int lowcmd_joint_count_for_whole_body()\n    {',
        '''    static int lowcmd_joint_count_for_whole_body()
    {
        const auto real=param::config["interactive_real"];
        if(real && real["enabled"] && real["enabled"].as<bool>())
            return 12; // REAL B2 only; Z1 must never share B2 LowCmd slots 12..17.''',FSM)
    return {SRCMAIN:main,CMAKE:cmake,FSM:fsm}

def config(src):
    c=yaml.safe_load(src)
    needed=['Passive','FixStand','SlowLieDown']
    c['FSM']['_']={name:{'id':idx} for idx,name in [(1,'Passive'),(2,'FixStand'),(4,'SlowLieDown')]}
    c['FSM']['_']['InteractiveArmReal']={'id':6,'type':'InteractiveArmReal'}
    c['FSM']['Passive']['transitions']={'FixStand':'up.pressed'}
    c['FSM']['FixStand']['transitions']={'Passive':'B.on_pressed','SlowLieDown':'down.on_pressed','InteractiveArmReal':'X.on_pressed'}
    c['FSM']['InteractiveArmReal']={'transitions':{'FixStand':'up.pressed','Passive':'B.on_pressed'}}
    # C++ native standing/lying interpolator requires qs[0] and qs[1] SAME 12-DOF length.
    for state in ('Passive','FixStand','SlowLieDown'):
        d=c['FSM'][state]
        for k in ('mode','kd','kp'):
            if k in d:d[k]=d[k][:12]
        if 'qs' in d:d['qs']=[q[:12] for q in d['qs']]
        for k in ('target_indices','target_q','target_duration'):
            d.pop(k,None)
    c['z1_sdk']['enabled']=False
    c['z1_diagnostic']['enabled']=False
    c.pop('arm_ik_stand',None);c.pop('interactive_arm_ik',None)
    c.pop('interactive',None)
    c.pop('sim_auto_start',None)
    c.pop('jointctrl_mock_sim',None);c.pop('jointctrl_shadow',None)
    c['interactive_real']={
        'enabled':True,'keyboard_only':True,
        'z1_backend':'sdk','socket_path':'/tmp/b2z1_z1_worker.sock',
        'policy_path':'../../interactive_b2z1/build/b2z1_interactive_policy.onnx',
        'enable_b2_output':False,'enable_z1_sdk_targets':False,
        'enable_walking':False,'ee_obs_source':'ik_goal',
        'mount_xyz_base':[.03,0.,.08],'tcp_offset_sdk_x':.0882,
    }
    return '# E2g REAL candidate: no nonloopback startup unless separately gated. NOT READY FOR UNSUPERVISED HARDWARE.\n'+yaml.safe_dump(c,sort_keys=False)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('repo',type=pathlib.Path)
    x=parser.add_mutually_exclusive_group(required=True);x.add_argument('--check',action='store_true');x.add_argument('--apply',action='store_true')
    a=parser.parse_args();repo=a.repo.resolve()
    if subprocess.run(['git','-C',str(repo),'merge-base','--is-ancestor','f2a3567','HEAD']).returncode:
        raise RuntimeError('repo does not descend from f2a3567')
    for dep in ('m24e2f_real_fsm_preview','m24e2e_live_input'):
        if not (repo/SUB/dep).is_dir():raise RuntimeError('missing '+dep)
    if (repo/DEST).exists() or (repo/CFG).exists():raise RuntimeError('destination exists; no overwrites')
    olds={p:(repo/p).read_text() for p in (SRCMAIN,CMAKE,FSM)}
    changes=patch(olds)
    cfg=config((repo/BASE).read_text())
    assert yaml.safe_load(cfg)['FSM']['FixStand']['qs'][1] and len(yaml.safe_load(cfg)['FSM']['FixStand']['qs'][1])==12
    print('E2g: ADD isolated real FSM + callback-stamped B2 subscriber; patch main/CMake/FSMState, new config.')
    print('Real B2 output OFF; Z1 SDK targets HARD LOCKED. Existing preview/MuJoCo configs unchanged.')
    if a.check:print('DRY RUN unchanged');return
    shutil.copytree(ROOT,repo/DEST,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    backups=repo/DEST/'tools/backups';backups.mkdir(exist_ok=True)
    for p,s in olds.items():(backups/(p.name+'.before_e2g')).write_text(s)
    try:
        for p,s in changes.items():(repo/p).write_text(s)
        (repo/CFG).write_text(cfg)
    except BaseException:
        for p,s in olds.items():(repo/p).write_text(s)
        if (repo/CFG).exists():(repo/CFG).unlink()
        raise
    print('INSTALLED',repo/DEST)
if __name__=='__main__':
    try:main()
    except Exception as e:print('INSTALL ERROR:',e,file=sys.stderr);sys.exit(2)
