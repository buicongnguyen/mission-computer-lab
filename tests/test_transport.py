"""Exercise real pipes, including peers that leave a response unfinished."""
from pathlib import Path
import subprocess
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from supervisor_client import exchange


class TransportTests(unittest.TestCase):
    def run_peer(self,reply,timeout=0.5):
        # The peer reports readiness first, so interpreter startup never counts against the deadline.
        script=('import sys,time;sys.stderr.write("ready\\n");sys.stderr.flush();sys.stdin.readline();'
                'sys.stdout.write('+repr(reply)+');sys.stdout.flush();time.sleep(3)')
        p=subprocess.Popen([sys.executable,'-c',script],stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            self.assertEqual(p.stderr.readline(),'ready\n')
            return exchange(p,[7],timeout)
        finally:
            p.kill();p.wait()
            p.stdin.close();p.stdout.close();p.stderr.close()
    def test_partial_line_has_deadline(self):
        with self.assertRaisesRegex(RuntimeError,'timeout after a partial line'):self.run_peer('7 ACTIVE healthy')
    def test_silent_peer_times_out(self):
        with self.assertRaisesRegex(RuntimeError,'timeout$'):self.run_peer('')
    def test_malformed_line_does_not_wait_for_stderr_eof(self):
        with self.assertRaisesRegex(RuntimeError,'Malformed'):self.run_peer('bad\n')
    def test_invalid_sequences_modes_and_velocities(self):
        for reply in ['8 ACTIVE healthy 0 0 0\n','7 UNKNOWN reason 0 0 0\n',
                      '7 ACTIVE healthy nan 0 0\n','7 ACTIVE healthy 2 2 2\n']:
            with self.subTest(reply=reply),self.assertRaisesRegex(RuntimeError,'Malformed'):self.run_peer(reply)
    def test_extra_response_data_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'extra response data'):self.run_peer('7 ACTIVE healthy 0 0 0\n7 ACTIVE healthy 0 0 0\n')
    def test_valid_reply(self):
        self.assertEqual(self.run_peer('7 ACTIVE healthy 1 0 0\n'),('ACTIVE','healthy',[1.,0.,0.]))

if __name__=='__main__':unittest.main()
