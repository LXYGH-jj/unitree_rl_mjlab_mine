"""No hardware; launch the mock worker executable only."""
import os
import socket
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

EXE = os.environ["M24E2A_MOCK_EXE"]


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="m24e2a_")
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

    def test_hello_and_valid_state(self):
        self.assertIn("MOCK_ONLY", self.ask("V1 HELLO"))
        state = self.ask("V1 GET 1").split()
        self.assertEqual(state[:3], ["V1", "STATE", "MOCK"])
        self.assertEqual(state[7], "HOLD")
        self.assertEqual(len(state), 20)
        self.assertAlmostEqual(float(state[9]), 1.48)

    def test_movement_and_watchdog(self):
        stamp = time.monotonic_ns()
        self.assertEqual(self.ask(f"V1 TARGET 1 {stamp} 1 0 1.50 -.63 -.84 0 1.57"),
                         "V1 ACK 1 ACCEPTED")
        time.sleep(.05)
        s = self.ask("V1 GET 2").split()
        self.assertEqual(s[7], "TRACK")
        self.assertGreater(float(s[9]), 1.48)
        time.sleep(.20)
        s = self.ask("V1 GET 3").split()
        self.assertEqual(s[7], "HOLD")
        self.assertGreaterEqual(int(s[6]), 150)

    def test_replay_and_invalid_target(self):
        stamp = time.monotonic_ns()
        req=f"V1 TARGET 2 {stamp} 1 0 1.50 -.63 -.84 0 1.57"
        self.assertIn("ACK",self.ask(req))
        self.assertIn("NONMONOTONIC_SEQUENCE",self.ask(req))
        self.assertIn("TARGET_OUT_OF_LIMIT",self.ask(f"V1 TARGET 3 {time.monotonic_ns()} 1 999 1.5 -.63 -.84 0 1.57"))
        self.assertEqual(self.ask("V1 GET 4").split()[7], "HOLD")

    def test_bad_messages(self):
        self.assertIn("BAD_VERSION",self.ask("V2 GET 1"))
        self.assertIn("INVALID_TARGET_VALUES",self.ask("V1 TARGET 1 10 1 0"))
        self.assertIn("UNKNOWN_VERB",self.ask("V1 UNSUPPORTED"))
        self.assertIn("MALFORMED_PACKET", self.ask("V1 HELLO EXTRA"))
        self.assertIn("MOCK_ONLY", self.ask("V1 HELLO"))


if __name__ == "__main__":
    unittest.main()
