import importlib.util
import pathlib
root=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('installer',root/'tools/install_overlay.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
main='''#include "State_JointCtrlShadow.h"
int main() {
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
}
'''
cmake='''include_directories(
  include
)
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC ABC)
'''
a,b=m.transform(main,cmake)
assert 'State_InteractiveArmRealPreview.h' in a
assert a.index('M2.4e2f preview refuses')<a.index('ChannelFactory::Instance()->Init')
assert b.index('m24e2f_real_fsm_preview/include')<b.index('target_compile_definitions')
assert m.transform(a,b)==(a,b)
try:
    m.transform(main.replace('State_JointCtrlShadow.h','missing.h'),cmake)
    raise AssertionError('missing anchor should fail')
except RuntimeError:pass
sample='''FSM:
  _:
    Passive: {id: 1}
    FixStand: {id: 2}
    Interactive: {id: 3}
  FixStand:
    transitions: {Passive: "LT + B.on_pressed"}
z1_sdk:
  enabled: true
z1_diagnostic:
  enabled: true
interactive:
  policy_path: "a.onnx"
'''
content=m.conf_transform(sample)
assert 'enabled: false' in content and 'backend: mock' in content
assert 'RT + Y.on_pressed' in content
print('E2f installer/test config guards PASS')
