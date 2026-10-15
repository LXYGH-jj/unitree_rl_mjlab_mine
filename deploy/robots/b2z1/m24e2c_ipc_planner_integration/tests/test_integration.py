"""M2.4e2c: test only the synthetic worker; no Z1 SDK is loaded."""
import os
import socket
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

EXE = os.environ["M24E2C_MOCK_EXE"]


class IntegratedWorkerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="m24e2c_")
        self.path = str(Path(self.tmp.name) / "worker.sock")
        self.proc = subprocess.Popen([EXE, "--socket", self.path],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for _ in range(100):
            if os.path.exists(self.path):
                break
            if self.proc.poll() is not None:
                self.fail("worker died: " + self.proc.stderr.read().decode())
            time.sleep(.01)
        self.assertTrue(os.path.exists(self.path))
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.sock.settimeout(1.)
        self.sock.connect(self.path)

    def tearDown(self):
        self.sock.close()
        self.proc.terminate()
        try:
            self.proc.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.communicate(timeout=2)
        self.tmp.cleanup()

    def ask(self, req):
        self.sock.send(req.encode())
        return self.sock.recv(512).decode()

    def target(self, seq, q1=1.50):
        return self.ask(f"V1 TARGET {seq} {time.monotonic_ns()} 1 0 {q1} -.63 -.84 0 1.57")

    def test_hello_and_get_compatible_with_e2a(self):
        self.assertIn("MOCK_ONLY", self.ask("V1 HELLO"))
        s = self.ask("V1 GET 1").split()
        self.assertEqual(s[:3], ["V1", "STATE", "MOCK"])
        self.assertEqual(s[7], "HOLD")
        self.assertEqual(len(s), 20)
        self.assertAlmostEqual(float(s[9]), 1.48)

    def test_goal_planner_motion_watchdog(self):
        self.assertEqual(self.target(1), "V1 ACK 1 ACCEPTED")
        time.sleep(.055)
        s = self.ask("V1 GET 2").split()
        self.assertEqual(s[7], "TRACK")
        self.assertGreater(float(s[9]), 1.48)
        self.assertLess(float(s[9]), 1.50)
        self.assertLessEqual(abs(float(s[15])), .4 + 1e-4)
        time.sleep(.19)
        s = self.ask("V1 GET 3").split()
        self.assertEqual(s[7], "HOLD")
        self.assertGreaterEqual(int(s[6]), 150)
        self.assertIn("OK", self.ask("V1 DIAG"))

    def test_bad_target_latches_and_ignores_new_goals(self):
        self.assertIn("ACK", self.target(1))
        self.assertIn("NONMONOTONIC_SEQUENCE", self.target(1))
        self.assertIn("FAULT", self.ask("V1 DIAG"))
        self.assertIn("FAULT_LATCHED", self.target(2))
        self.assertEqual(self.ask("V1 GET 3").split()[7], "HOLD")

    def test_out_of_range_latches(self):
        self.assertIn("TARGET_OUT_OF_LIMIT", self.target(1,999))
        self.assertIn("FAULT", self.ask("V1 DIAG"))

    def test_sdk_mode_is_not_available(self):
        p = subprocess.run([EXE, "--backend", "sdk"],
                           capture_output=True, text=True, timeout=2)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("No SDK load", p.stderr)


if __name__ == "__main__":
    unittest.main()
