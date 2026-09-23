"""Process lifetime regressions without starting an autopilot."""
from pathlib import Path
import signal
import unittest
from unittest.mock import Mock,patch
from run_sitl import Processes

class ProcessCleanupTests(unittest.TestCase):
    def test_exited_leader_still_cleans_owned_group(self):
        processes=Processes(Path('.'),{});child=Mock(pid=12345)
        child.poll.return_value=0
        with patch('run_sitl.os.killpg') as kill:
            processes.stop(child)
        self.assertEqual(kill.call_args_list[0].args,(12345,signal.SIGINT))
        self.assertEqual(kill.call_args_list[-1].args,(12345,signal.SIGKILL))
    def test_cleanup_continues_after_one_failure(self):
        processes=Processes(Path('.'),{});processes.children=[('first',Mock()),('second',Mock())]
        stream=Mock();processes.handles=[stream]
        with patch.object(processes,'stop',side_effect=[RuntimeError('failed'),None]) as stop:
            with self.assertRaisesRegex(RuntimeError,'second'):processes.close()
            self.assertEqual(stop.call_count,2)
        stream.close.assert_called_once()

if __name__=='__main__':unittest.main()
