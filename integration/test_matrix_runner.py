"""A diagnostic full run must retain failure status when it continues to later scenarios."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import run_sitl as R


class MatrixRunnerTests(unittest.TestCase):
    def test_keep_going_preserves_failed_case_and_nonzero_exit(self):
        for keep, expected in ((False, 1), (True, 2)):
            with self.subTest(keep_going=keep), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp) / 'run'
                argv = ['run_sitl', '--workspace', tmp, '--output', str(out), '--all'] + (
                    ['--keep-going'] if keep else []
                )
                rows = [
                    dict(scenario='nominal', passed=False, checks={'flight': False}),
                    dict(scenario='camera_dropout', passed=True, checks={'flight': True}),
                ]
                with (
                    patch.object(R.sys, 'argv', argv),
                    patch.object(R.signal, 'signal'),
                    patch.object(R, 'SCENARIOS', ('nominal', 'camera_dropout')),
                    patch.object(R, 'capture', return_value={'fixture': 'unchanged'}),
                    patch.object(R, 'run_scenario', side_effect=rows) as fly,
                ):
                    self.assertEqual(R.main(), 1)
                self.assertEqual(fly.call_count, expected)
                results = json.loads((out / 'results.json').read_text())
                self.assertEqual(len(results), expected)
                self.assertIs(results[0]['passed'], False)
                self.assertIs(json.loads((out / 'provenance.json').read_text())['inputs_unchanged'], True)


if __name__ == '__main__':
    unittest.main()
