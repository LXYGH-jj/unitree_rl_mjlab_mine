from pathlib import Path
root=Path(__file__).resolve().parents[1]
src=(root/'src/State_InteractiveArmRealPreview.cpp').read_text()
for key in ['makeLiveInput(', 'pipeline_->step(', 'inverseKinematics(', 'ReadOnlyPoller','z1_.latest(',
            'sim_only', 'backend', 'B2 lowstate tick stalled','preview refuses non-loopback']:
    if key=='preview refuses non-loopback':
        assert key in (root/'tools/install_overlay.py').read_text()
    else: assert key in (src+(root/'include/State_InteractiveArmRealPreview.h').read_text()),key
for banned in ['setArmCmd(', 'z1_.target(', 'unlockAndPublish(', 'sendRecv(', 'motor_cmd()[']:
    assert banned not in src, banned
assert 'post_run(){' in src and 'FSMState::post_run()' not in src
print('E2f preview-only output guards PASS')
