#!/usr/bin/env python3
"""Check anchor transformations and fail-closed install without touching real repo."""
from pathlib import Path
import importlib.util
import tempfile
import subprocess
import sys
MOD=Path(__file__).resolve().parents[1]/'tools/install_overlay.py'
spec=importlib.util.spec_from_file_location('ov',MOD)
ov=importlib.util.module_from_spec(spec);spec.loader.exec_module(ov)
main='''#include "State_InteractiveArmIK.h"\nint main(){\n    unitree::robot::ChannelFactory::Instance()->Init(0, network);\n}\n'''
new_main=ov.build_main(main)
assert new_main.index('const auto jointctrl_hardware') < new_main.index('ChannelFactory::Instance()->Init')
assert 'State_JointCtrlShadow.h' in new_main
cmake='''include_directories(\n  ${PROJECT_SOURCE_DIR}/../../interactive_b2z1/sim_closed_loop\n)\ntarget_sources(foo PRIVATE\n  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/src/interactive_snapshot_store.cpp\n)\n'''
new_cmake=ov.build_cmake(cmake)
assert '/m24c_shadow_integration/include' in new_cmake
assert '/m24c_shadow_integration/src/m24c_shadow_bridge.cpp' in new_cmake
base='''FSM:\n  _:\n    Passive:\n      id: 1\n  Passive:\n    transitions:\n      FixStand: LT + up.on_pressed\n  FixStand:\n    transitions:\n      InteractiveArmIK: X.on_pressed\nz1_sdk:\n  enabled: false\n'''
new_yaml=ov.build_yaml(base)
assert new_yaml.count('JointCtrlShadow:')==3
assert 'z1_jointctrl:\n  enabled: false' in new_yaml
try:
    import yaml
    y=yaml.safe_load(new_yaml)
    assert y['FSM']['_']['JointCtrlShadow']['id']==7
    assert y['z1_jointctrl']['enabled'] is False
except ImportError:
    pass
assert 'z1_jointctrl_controller' not in new_main
print('M2.4c2 installer anchors / before-DDS interlock / YAML fixture: PASS')
