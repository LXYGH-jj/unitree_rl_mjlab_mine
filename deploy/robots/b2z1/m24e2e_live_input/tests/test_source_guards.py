import pathlib,sys
root=pathlib.Path(__file__).resolve().parents[1]
client=(root/'src/z1_ipc_client.cpp').read_text()
assert 'SDK TARGET disabled pending' in client
assert 'std::this_thread::sleep_until' in client
assert 'V1 HELLO_ACK SDK_MODE_UNQUALIFIED' in client
assert 'IPC response timestamp invalid/stale' in client
assert 'SO_RCVTIMEO' in client
repo=pathlib.Path(sys.argv[1]) if len(sys.argv)>1 and sys.argv[1] else None
if repo:
    read=(repo/'deploy/robots/b2z1/src/State_InteractiveArmIK.cpp').read_text()
    assert 'msg.motor_state()[12+i]' in read, 'Existing sim-specific input path unexpectedly changed'
    old=(repo/'deploy/interactive_b2z1/src/interactive_feedback_adapter.cpp').read_text()
    assert 'max_interstream_skew' in old and 'b2MotorToPolicy' in old
    assert (repo/'deploy/robots/b2z1/m24e2d_sdk_worker_bridge/src/z1_worker_dual.cpp').exists()
print('M2.4e2e source guards PASS')
