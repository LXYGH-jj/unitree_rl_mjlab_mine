import os
import subprocess
import unittest

BIN = os.environ['M24E2D_MOCK_EXE']

class CliGates(unittest.TestCase):
    def run_cmd(self,*args):
        return subprocess.run([BIN,*args],capture_output=True,text=True,timeout=3)
    def test_sdk_locked_by_default(self):
        result=self.run_cmd('--backend','sdk')
        self.assertNotEqual(result.returncode,0)
        self.assertIn('No SDK load',result.stderr)
    def test_sdk_locked_despite_ack(self):
        result=self.run_cmd('--backend','sdk','--bench-ack','Z1_BENCH_ATTENDED_LOWCMDSDK_RISK_ACCEPTED')
        self.assertNotEqual(result.returncode,0)
        self.assertIn('No SDK load',result.stderr)
    def test_help_does_not_load_sdk(self):
        result=self.run_cmd('--help')
        self.assertEqual(result.returncode,0)
        self.assertIn('--backend mock',result.stdout)
    def test_malformed_request_rejected(self):
        result=self.run_cmd('--socket','relative.sock')
        self.assertNotEqual(result.returncode,0)
        self.assertIn('Socket path must',result.stderr)

if __name__=='__main__': unittest.main()
