#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-.venv/bin/python}"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
"$PYTHON" -m unittest discover -s tests -p 'test_*.py' -v
"$PYTHON" tools/run_demo.py --all --output artifacts/latest
"$PYTHON" tools/check_artifacts.py artifacts/latest
