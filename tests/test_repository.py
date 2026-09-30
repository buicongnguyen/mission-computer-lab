"""Repository-level contracts: what flight provenance covers, and the test counts the documents state."""

import ast
from pathlib import Path
import re
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


def test_methods(folder):
    """Test methods of the unittest classes in a folder's test_*.py files, as unittest discovery counts them."""
    total = 0
    for path in (ROOT / folder).glob('test_*.py'):
        for node in ast.parse(path.read_text(encoding='utf-8')).body:
            if isinstance(node, ast.ClassDef):
                total += sum(isinstance(f, ast.FunctionDef) and f.name.startswith('test') for f in node.body)
    return total


class StatedCounts(unittest.TestCase):
    def test_the_readme_and_results_page_state_the_real_suite_sizes(self):
        counts = {
            'Python unit and process tests': test_methods('tests'),
            'ROS boundary and runner tests': test_methods('integration'),
            'browser page tests': len(list((ROOT / 'tests').glob('test_*.mjs'))),
        }
        for doc in ('README.md', 'docs/results.md'):
            text = (ROOT / doc).read_text(encoding='utf-8')
            for phrase, count in counts.items():
                with self.subTest(doc=doc, suite=phrase):
                    stated = {int(n) for n in re.findall(rf'(\d+) {phrase}', text)}
                    self.assertEqual(stated, {count})  # Stated at least once, and never a stale number.


if __name__ == '__main__':
    unittest.main()
