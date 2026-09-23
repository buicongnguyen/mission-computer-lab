#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
for tool in python3 g++ cmake; do
  command -v "$tool" >/dev/null || { echo "Missing $tool; install python3-venv build-essential cmake"; exit 1; }
done
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
