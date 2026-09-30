"""Repository-level contracts: what flight provenance covers."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import provenance  # noqa: E402


class Provenance(unittest.TestCase):
    def test_flights_hash_the_code_they_run_and_nothing_else(self):
        files = {p.relative_to(ROOT).as_posix() for p in provenance.runtime_python()}
        runtime = {
            p.relative_to(ROOT).as_posix() for p in ROOT.glob('integration/*.py') if not p.name.startswith('test_')
        }
        self.assertLessEqual(runtime, files)  # The runner and every node it launches.
        for module in ('guardian', 'world', 'security', 'supervisor_client', 'evidence_contracts', 'provenance'):
            self.assertIn(f'tools/{module}.py', files)  # Imported by them, directly or indirectly.
        for outside in ('tools/publish_sitl.py', 'tools/check_docs.py', 'tools/guardian_sim.py', 'tools/run_demo.py'):
            self.assertNotIn(outside, files)  # Publishing and offline tools can change after a run.
        self.assertFalse({f for f in files if Path(f).name.startswith('test_')})


if __name__ == '__main__':
    unittest.main()
