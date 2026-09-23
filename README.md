# Mission Computer Lab

A reproducible **PX4 + Gazebo + ROS 2 mission-computer simulation**, built around the split between a Qualcomm-class mission computer and an NXP-class PX4 flight controller. A C++17 supervisor gates mission intent on sensor freshness; ROS 2 carries typed payload and control messages over XRCE-DDS; real PX4 firmware flies a Gazebo x500 through takeoff, an A*-planned inspection route and landing. Faults are injected on purpose, and every outcome is checked from independently recorded telemetry.

**Live replay: [buicongnguyen.github.io/mission-computer-lab](https://buicongnguyen.github.io/mission-computer-lab/)**: recorded PX4 flights, fault injections and acceptance checks, viewable in the browser without installing anything.

Everything runs on an x86-64 Ubuntu 22.04 host (WSL2). Qualcomm hardware, NPU execution and hardware secure boot are outside what was executed; [Honest boundaries](#honest-boundaries) lists exactly what was and was not run.

## At a glance

- **4/4 real PX4 SITL flights pass**: nominal mission, camera dropout and recovery, companion-computer crash (PX4 failsafe takes over), and GPS fix loss (HOLD, then LAND). Each requires an observed, ordered climb → land → disarm cycle, not just accepted commands.
- **8/8 fast policy scenarios** in an accelerated harness, including IMU loss, link loss, inference overrun and low battery.
- **Signed model boot gate**: the model is signed at release; at boot the stored file is verified (Ed25519, version floor, device binding) and only the verified bytes are loaded. Tampered or unsigned models stop the perception node.
- **Layered tests**: C++ contract suite, 28 Python tests, 14 ROS boundary tests, dashboard state tests, Mermaid/HTML checks, and CI that also runs the suite under `python -O`.
- **Four recorded review passes**: each defect is paired with its fix and regression test in [the review record](docs/review-report.md).

## Ten-minute tour

1. **See it fly**: the [live flight replay](https://buicongnguyen.github.io/mission-computer-lab/web/sitl.html) (or [`web/sitl.html`](web/sitl.html) in a clone) replays recorded PX4 telemetry, mode transitions and acceptance checks for all four flights. The [fast policy replay](https://buicongnguyen.github.io/mission-computer-lab/web/index.html) covers all eight harness scenarios.
2. **Understand the design**: [architecture and contracts](docs/architecture.md): data path, supervisor protocol, state machine, threat model.
3. **Read the core**: [`src/supervisor.cpp`](src/supervisor.cpp) (health policy, ~80 lines) and [`integration/mission_node.py`](integration/mission_node.py) (ROS ↔ PX4 adapter: freshness, frames, arming rules, handoff to PX4).
4. **Check the evidence**: [flight results and exact upstream revisions](docs/sitl-execution.md) and [the review record](docs/review-report.md).

## How the project maps to mission-computer work

| Area | What is implemented and tested here | What still requires real hardware |
|---|---|---|
| Autonomy: perception, fusion, planning, navigation | Procedural camera → ONNX Runtime inference; planar LiDAR → inflated occupancy → A*; PX4 EKF2 in SITL; educational Kalman filter in the fast harness | Trained detector, VIO/SLAM, dynamic replanning |
| ROS 2 middleware and PX4 integration | Typed interfaces (`ros2/mission_interfaces`), best-effort sensor QoS, versioned PX4 topics over XRCE-DDS, offboard velocity control, command ACKs, MAVLink GCS heartbeat | NXP flight-controller hardware link and transport tuning |
| Multi-sensor integration | Camera, LiDAR, GNSS and IMU freshness contracts; ENU/NED conversion; replay-safe capture timestamps; fix-validity checks | Calibration, extrinsics, hardware timestamping |
| Reliability and failure ownership | HOLD/LAND supervisor with dwell times and intermittent-fault escalation; permanent handoff to PX4 on failsafe, disarm, estimator loss or operator mode change; companion-crash failsafe | Hardware-in-the-loop and flight qualification |
| AI/ML deployment | CPU `ExecutionProvider` selected and recorded; latency budget enforced by the supervisor | QNN/NPU conversion, quantization, profiling, power/thermal |
| Platform security | Signed artifact manifest (hash, version floor, device identity) verified from disk at boot; six tamper/rollback/identity cases; threat model | Boot ROM, fuses, TEE, protected key provisioning, persistent anti-rollback |
| Telemetry, logging, diagnostics | Independent observer process, JSONL logs, ROS bags, PX4 logs, run-time provenance hashes | Fleet telemetry and on-target diagnostics |
| DevOps and testing | CMake/CTest, unittest suites, GitHub Actions, docs drift check, evidence schema gates | Target image builds (Yocto/BSP) |
| Operator dashboards | Two offline JavaScript replays driven by recorded results | Live ground-station UI |

## What actually runs

```text
Signed model verification → virtual camera → ONNX CPU → synthetic object bbox
Procedural ROS LaserScan → inflated occupancy → A* route
Gazebo flight sensors + dynamics → PX4 EKF2 → ROS telemetry
          ↓
C++17 mission supervisor ↔ Python via actual process IPC
          ↓
ROS offboard velocity + mode/arm/land commands → PX4 → Gazebo x500
Independent observer + ROS bag + firmware logs → acceptance checks → replay
```

The ONNX graph is hand-authored brightness segmentation. Perception freshness gates mission health; detections do not steer or pursue objects. Procedural LiDAR contributes the initial static path. PX4 uses EKF2 in SITL; the separate fast harness uses an educational six-state filter. Neither demonstrates GPS-denied SLAM. The MAVLink process supplies a local GCS heartbeat; offboard control uses ROS 2/DDS.

## Run it

**Fast suite** (Ubuntu 22.04 / Python 3.10, a few minutes, no ROS needed):

```bash
bash scripts/bootstrap_wsl.sh
.venv/bin/python -m pip install -r requirements.lock.txt
bash scripts/run_all.sh
.venv/bin/python tools/publish_sample.py
```

Then open `web/index.html`. The committed samples make both replays usable without installing anything; for a loopback server run `python3 -m http.server 8765 --bind 127.0.0.1` and open `http://localhost:8765/web/sitl.html`.

**Real PX4 flights** (after the one-time install and build in [the SITL guide](docs/sitl-guide.md)):

```bash
bash scripts/run_sitl.sh --all --output ~/work/mission-computer-lab/retests/my-run
~/work/mission-computer-lab/venv/bin/python tools/publish_sitl.py --input ~/work/mission-computer-lab/retests/my-run
```

Run one instance at a time, with no flight hardware attached. [The complete reproduction guide](docs/complete-reproduction-guide.md) covers fresh setup, every test layer, evidence inspection and the debugging history.

## Evidence

| Layer | Result | Where |
|---|---|---|
| PX4 / Gazebo / ROS 2 flights | 4/4 scenarios, all named checks pass, inputs unchanged during the run | [SITL report](docs/sitl-execution.md), [raw evidence](artifacts/sitl-sample/report.json) |
| Fast policy scenarios | 8/8 | [Execution report](docs/execution.md) |
| Signed-artifact policy cases | 6/6 behave as specified | Same report |
| C++ supervisor contracts | 1 CTest program, including degraded-link escalation | [`tests/test_supervisor.cpp`](tests/test_supervisor.cpp) |
| Python unit and process tests | 28 | [`tests/`](tests/) |
| ROS boundary and runner tests | 14 | [`integration/test_*.py`](integration/) |

| Fast scenario | Intended evidence |
|---|---|
| Nominal | Inspect route and complete mission |
| Camera dropout | Hold, wait for stable data, resume and complete |
| GNSS loss | Show estimate degradation, hold and latched landing |
| Link loss | Detect stale heartbeat and escalate |
| Inference overrun | React to injected reported latency, then recover |
| Low battery | Immediate landing policy |
| IMU loss | Stop mission intent and land after sustained fault |
| Companion crash | Kill the C++ subprocess; exercise independent autopilot-stub watchdog |

Timings are WSL wall-clock observations of a tiny graph and local IPC, not real-time guarantees or Qualcomm performance measurements.

## Honest boundaries

**Executed:** PX4 SITL, Gazebo physics and flight sensors, EKF2, ROS 2 typed payload messages, DDS, MAVLink heartbeat, procedural camera/LiDAR, CPU ONNX, A*, C++ supervision and process kill, Ed25519 artifact verification, fault checks and recorded replays.

**Not executed:** Qualcomm BSP/boot ROM/fuses/TEE, NPU/GPU inference, rendered Gazebo payload camera/LiDAR, trained detection, VIO, real OTA installation or physical drone tests. Battery input in the SITL adapter is fixed at a simulated 95%; low-battery policy is tested in the fast harness. The signing key is ephemeral per run and its public key is not in protected storage.

No credentials or production signing keys are included. This repository does not establish safety, security or aerospace certification.

## Repository map

| Path | Purpose |
|---|---|
| `src/`, `include/` | C++ supervisor, validation and process protocol |
| `tools/` | Sensors, perception, estimator/planner, security, evidence harness and publishers |
| `integration/`, `ros2/` | ROS nodes, typed interfaces, MAVLink heartbeat and real flight orchestrator |
| `simulation/` | Gazebo inspection world with x500 and obstacles |
| `tests/` | Policy, model, planning, localization, security, transport and evidence checks |
| `scripts/` | WSL bootstrap, SITL install/build and one-command verification |
| `web/` | Replay dashboards driven by recorded results |
| `artifacts/sample/`, `artifacts/sitl-sample/` | Compact reference evidence for both replays |
| `docs/` | Architecture, runbooks, domain notes, execution reports and review record (Markdown with rendered HTML) |

Python dependency versions are pinned in `requirements.lock.txt` (fast suite) and `requirements-sitl.lock.txt` (flight stack).
