# WSL runbook and extension guide

This guide reproduces the executed pipeline. The baseline uses Ubuntu 22.04 in WSL2, C++17 and Python 3.10. It does not require ROS, Gazebo, Docker, a GPU, a Qualcomm SDK, or real flight hardware. The separate PX4/Gazebo/ROS integration has also been implemented and executed; use section 9 to reach its dedicated runbook.

## 1. Open the correct environment

From **Windows PowerShell**:

```powershell
wsl --list --verbose
wsl -d Ubuntu-22.04
```

Then, inside **Ubuntu Bash**:

```bash
cd /mnt/c/src/mission-computer-lab   # your checkout
pwd
uname -a
python3 --version
g++ --version
cmake --version
```

Local validation used a checkout on the Windows mount (`/mnt/c/...`); replace the path with your clone location. An existing WSL2 Ubuntu 22.04 installation is sufficient. Microsoft documents WSL GUI support separately, but this baseline needs only a Windows browser for its HTML. [WSL GUI reference](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)

If an Ubuntu prerequisite is missing, install the limited set below. These commands change the Linux distribution and may request your Linux password; the tested machine already had the required tools.

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-venv python3-pip git
```

For larger ROS/PX4 builds, use the Linux filesystem such as `~/work/mission-computer-lab` to avoid slow Windows-mounted metadata operations. If copying this project, exclude `.venv` and `build`; virtual environments and CMake caches contain absolute paths and should be recreated at the destination.

## 2. Install the project environment and compile

From the repository root:

```bash
bash scripts/bootstrap_wsl.sh
```

The script creates `.venv`, installs the four directly pinned Python dependencies and compiles two C++ binaries. It does not install ROS, add system repositories or change `.bashrc`. The first installation may take several minutes on `/mnt/c`.

For the exact recorded dependency set, install the full lock after environment creation:

```bash
.venv/bin/python -m pip install -r requirements.lock.txt
```

`requirements.txt` records direct choices; `requirements.lock.txt` records all installed Python package versions from the successful WSL environment. Python 3.10/Linux is the tested combination. Other Python versions and architectures may not have compatible wheels for these pins.

## 3. Run all validation and generate evidence

```bash
bash scripts/run_all.sh
```

Expected stages:

1. CMake rebuilds the C++17 core and executable.
2. CTest runs the supervisor contract suite.
3. Python runs 23 perception, planning, localization, security, evidence and process-contract tests.
4. The harness runs eight scenarios.
5. The artifact checker verifies the complete scenario matrix and six artifact-verification cases.

The command must exit with code 0. A scenario PASS means its explicit checks passed in the modeled world; it does not mean a real aircraft would be safe. See [architecture](architecture.md) for every shortcut.

## 4. Understand the generated files

| File under `artifacts/latest/` | Purpose |
|---|---|
| `report.json` | Environment and hashes, boot labels, security cases, scenario checks, events and sampled telemetry |
| `report-data.js` | The same evidence wrapped for a local-file browser replay without a server |
| `metrics.csv` | One row per scenario with measured timing and localization data |
| `execution.md` | Readable evidence summary |
| `synthetic_detector.onnx` | The actual generated model loaded by ONNX Runtime |

The report includes the model and supervisor-binary SHA-256 digests. Keep those alongside evidence when changing code. Timing is measured per run; it is not expected to match the sample exactly. The source seed is fixed at 17 by default. Motion and sensor-noise traces are reproducible if the runtime stays within the illustrative inference budget.

## 5. Publish the verified sample and open the replay

```bash
.venv/bin/python tools/publish_sample.py
```

This first rechecks the full matrix, then copies the evidence into the shareable `artifacts/sample/` and the summary into `docs/execution.md`. A single-scenario run cannot replace the sample because the checker requires all eight scenarios.

Open `web/index.html` in a Windows browser. It loads the committed sample directly from local files. Select a scenario, press Play, drag the time slider, and compare the trajectory, estimate, camera output and state events.

From **Windows PowerShell**, one way to open it is:

```powershell
Start-Process '<your checkout>\web\index.html'
```

Alternatively, run a loopback-only server in **Ubuntu**, keep its terminal open, and open the URL in Windows:

```bash
.venv/bin/python -m http.server 8765 --bind 127.0.0.1
```

Then visit [the local replay](http://localhost:8765/web/index.html). Stop the server with Ctrl+C when finished. The file-based replay is the simpler option if WSL localhost forwarding is unavailable.

## 6. Reproduce one failure

```bash
.venv/bin/python tools/run_demo.py --scenario gps_dropout --output artifacts/gps-study
.venv/bin/python tools/run_demo.py --scenario camera_dropout --output artifacts/camera-study
.venv/bin/python tools/run_demo.py --scenario companion_crash --output artifacts/crash-study
```

Faults begin at simulated second 5. Camera dropout ends at 5.8; GPS/link/IMU losses persist. Inference overload is a reported 120 ms duration during 5–5.8 s. Low battery becomes 15% at 5 s. Companion crash kills the supervisor process at 5 s; the independent stub watchdog reacts after its command-age threshold.

The scenario runner exits nonzero on failed checks. Inspect events without a GUI:

```bash
.venv/bin/python -c 'import json; r=json.load(open("artifacts/gps-study/report.json")); print(r["results"][0]["events"])'
```

The full replay uses `artifacts/sample/`; a study output does not change it. To update the shared replay, rerun the entire suite and publish the sample again.

## 7. Inspect and test individual layers

```bash
ctest --test-dir build --output-on-failure
.venv/bin/python -m unittest discover -s tests -p 'test_*.py' -v
.venv/bin/python -c 'import onnxruntime as o; print(o.get_available_providers())'
```

The available-provider list may include options other than CPU; the detector explicitly selects CPU and records the session's active provider. Do not use the available list as evidence that a model ran on an accelerator.

For an additional C++ memory/undefined-behavior check:

```bash
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DENABLE_SANITIZERS=ON
cmake --build build-sanitize --parallel 2
ctest --test-dir build-sanitize --output-on-failure
```

## 8. Rebuild the Markdown-to-HTML guides

The rendered HTML files are included. Regeneration needs Node.js 20 or newer and the locked marked and Mermaid dependencies:

```bash
npm ci
npm run docs
node tests/test_diagrams.mjs
```

The document renderer converts every `docs/*.md`, preserves fenced code and tables, and rewrites local `.md` links to `.html`. It is for trusted local documentation; it is not a service for rendering arbitrary untrusted Markdown. A normal clone uses `npm ci`; `MARKED_MODULE` can point the renderer at an alternative local `marked` build.

After rendering, parse every diagram and verify the bundled script:

```bash
node tests/test_diagrams.mjs
```

## 9. Real ROS 2 / PX4 / Gazebo integration — implemented

The actual firmware integration now has its own [detailed WSL SITL guide](sitl-guide.md), [flight evidence report](sitl-execution.md) and [offline replay](../web/sitl.html). It runs ROS 2 Humble, PX4 v1.16.0, matching typed messages, XRCE Agent v2.4.3 and Gazebo Harmonic. The normal mission, camera recovery, companion crash and GPS fix-loss cases have been executed.

```bash
# On a fresh Ubuntu 22.04 WSL environment (skip if already built):
sudo bash scripts/install_sitl_system.sh
bash scripts/build_sitl.sh

# Run with a new output directory:
bash scripts/run_sitl.sh --all --output artifacts/my-sitl-run
~/work/mission-computer-lab/venv/bin/python tools/publish_sitl.py --input artifacts/my-sitl-run
```

The integration uses `/mission/camera/image`, `/mission/lidar/scan`, `/mission/perception` and `/mission/decision`, plus the firmware's `/fmu/in` and `/fmu/out` interfaces. The SITL guide documents the exact types, QoS, coordinate conversion, freshness handling, startup policy and measured acceptance criteria. No physical flight hardware is needed.

## 10. Optional Qualcomm deployment — requires a supported target

Obtain the exact SoC/board/BSP matrix, vendor image instructions and runtime license/access first. Record OS, kernel, firmware, accelerator runtime and model versions. Copy the validated application and model only through the board's supported deployment workflow.

Validate CPU output first, then use the supported QNN/QAIRT or vendor runtime path. Record operator support and fallback, warm/cold p50/p95/p99 latency, memory, throughput, thermal behavior and power. Run the same recorded inputs through CPU and accelerator paths and compare outputs. A generic ONNX file alone does not guarantee NPU compatibility. [QNN provider constraints](https://onnxruntime.ai/docs/execution-providers/QNN-ExecutionProvider.html)

Board secure boot, provisioning and fuse lifecycle need the vendor's exact documentation and the organization's key custody/recovery design. The repo's software-only manifest checks are preparation for that discussion; there are no hardware provisioning commands here.

## Troubleshooting

| Symptom | Likely cause and useful action |
|---|---|
| PowerShell says `Access is denied` for WSL inside an agent sandbox | WSL service access may require the host's approval mechanism; this is distinct from missing Ubuntu. Running WSL from your own terminal can confirm installation. |
| `python3 -m venv` fails | Install Ubuntu's `python3-venv`; then recreate only the project virtual environment. |
| Installation is slow on `/mnt/c` | Windows filesystem metadata overhead; wait for pip or use a fresh Linux-filesystem clone. |
| ORT or NumPy import error | Use `.venv/bin/python`, Python 3.10, and the pinned packages; do not mix Windows and Linux virtual environments. |
| No `mission_supervisor` executable | Run CMake build; the harness default expects `build/mission_supervisor`. |
| C++ sample rejected | Check field count, strictly increasing timestamps/sequence and finite/range constraints. |
| Scenario fails a timing-related policy under heavy load | Inspect actual CPU inference durations; repeat after reducing load and record both results rather than hiding the failure. |
| Replay still shows old results | Run the full suite, publish the sample, and reload the page. |
| Local replay is blank | Ensure `artifacts/sample/report-data.js` exists alongside the repository structure; do not move only the HTML file. |
| Expected GitHub status is absent | The workflow runs only after an authorized remote push; local success is not proof of hosted CI success. |

The [execution report](execution.md) records what actually ran during initial creation. The [review record](review-report.md) lists the defects found in each review pass and the regression tests that now cover them.
