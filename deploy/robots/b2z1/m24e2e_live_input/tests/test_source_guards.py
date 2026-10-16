import pathlib,sys
root=pathlib.Path(__file__).resolve().parents[1]
client=(root/'src/z1_ipc_client.cpp').read_text()
assert 'SDK TARGET disabled pending' in client
assert 'mock_j5_control && expected!=Mode::Mock' in client, 'SDK mock-command gate missing'
assert 's.target(seq++,q,true)' in client, 'single-thread mock refresh missing'
assert 's.target(seq++,q,false)' in client, 'mock disarm missing'
assert 'Mock J5 control unavailable/not connected' in client
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
    real=(repo/'deploy/robots/b2z1/m24e2g_real_control/src/State_InteractiveArmReal.cpp').read_text()
    assert 'z1_.start(socket_path_,mode_,40,mock_j5_jog_)' in real
    assert 'mode_!=m24e2e::Mode::Mock || send_b2_' in real
    assert 'z1_.setMockJ5Delta(new_delta)' in real
print('M2.4e2e source guards PASS')
