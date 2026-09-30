"""Copy verified evidence into the repository's offline replay sample."""

from pathlib import Path
import shutil
from check_artifacts import check

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'artifacts/latest'
check(source)
destination = ROOT / 'artifacts/sample'
destination.mkdir(parents=True, exist_ok=True)
for name in ('report.json', 'report-data.js', 'metrics.csv', 'execution.md', 'synthetic_detector.onnx'):
    shutil.copyfile(source / name, destination / name)
shutil.copyfile(source / 'execution.md', ROOT / 'docs/execution.md')
print('Published verified offline replay into artifacts/sample')
