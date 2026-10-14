from pathlib import Path
import importlib.util
import subprocess
import sys
import tempfile

base=Path(__file__).resolve().parents[1]
p=base/'tools/install_overlay.py'
spec=importlib.util.spec_from_file_location('m24c3_installer',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
cmake='''include_directories(\n  ${PROJECT_SOURCE_DIR}/../../interactive_b2z1/m24c_shadow_integration/include\n)\ntarget_sources(${PROJECT_NAME}_lib PRIVATE\n  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/m24c_shadow_integration/src/m24c_shadow_bridge.cpp\n)\n'''
assert 'm24c3_jointctrl_runtime/include' in m.proposed(cmake)
assert 'm24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp' in m.proposed(cmake)
try: m.proposed(m.proposed(cmake))
except RuntimeError: pass
else: raise AssertionError('repeat install must fail')
try: m.proposed('invalid CMake anchor')
except RuntimeError: pass
else: raise AssertionError('anchor drift must fail')
print('M2.4c3 installer marker/duplicate guards: PASS')
