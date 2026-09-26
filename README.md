# Mission Computer Lab

A reproducible **PX4 + Gazebo + ROS 2 mission-computer simulation**, built around the split between a Qualcomm-class mission computer and an NXP-class PX4 flight controller. A C++17 supervisor gates mission intent on sensor freshness; ROS 2 carries typed payload and control messages over XRCE-DDS; real PX4 firmware flies a Gazebo x500 through takeoff, an A*-planned inspection route and landing. Faults are injected on purpose, and every outcome is checked from independently recorded telemetry. A fleet scenario flies three drones from a moving carrier vehicle: a ground station sequences launches and landings, each drone flies its own altitude layer, and all three land back on their pads while the carrier drives. **Guardian drones** extend the station's sensors, warn early, keep clear of intruders and survive jamming and GNSS spoofing, under an explicit split of authority between each drone, the station and a command center; they are non-kinetic by design.

**Reference replay (earlier passing revision; [latest validation](docs/validation.md)): [buicongnguyen.github.io/mission-computer-lab](https://buicongnguyen.github.io/mission-computer-lab/)**: recorded PX4 flights, fault injections, the fleet on its moving carrier, the guardian evaluation, Gazebo videos of selected demonstrations and an interactive 3D replay, viewable in the browser without installing anything.

Everything runs on an x86-64 Ubuntu 22.04 host (WSL2). Qualcomm hardware, NPU execution and hardware secure boot are outside what was executed; [Honest boundaries](#honest-boundaries) lists exactly what was and was not run.

## At a glance

- **Latest PX4 validation: 12/13 scenarios pass** on the fixed source. Unresolved: guardian_fast: dispersed_before_impact. [Current results and runtime hashes](docs/validation.md) distinguish this retest from earlier reference recordings.
- **Fleet from a moving carrier**: three PX4 instances launch in turn, inspect at 3, 4 and 5 m, and land back one at a time. The latest run passed 35/35 checks, including independent PX4 landed/contact confirmation. Worst touchdown error 6.6 cm; closest airborne approach 2.11 m.
- **Guardian drones, evaluated**: four designs × eight scenarios × 40 seeds. The latest PX4 guardian retest passed 7/8 scenarios and 323/324 checks. The review fixed eight control/publication defects and addressed flight-discovered descent, logging and map-sharing issues. The fast-inbound timing limitation remains documented in [validation](docs/validation.md); [fixes and reproduction](docs/review-fixes.md) and [seeded evaluation](docs/guardian-results.md) explain the evidence.
- **See it fly**: every published reference flight has an interactive three.js 3D replay; Gazebo video is included for the four single-drone flights, fleet and configured guardian demonstration; `--gui` opens the live Gazebo window for demonstrations.
- **8/8 fast policy scenarios** in an accelerated harness, including IMU loss, link loss, inference overrun and low battery.
- **Signed model boot gate**: the model is signed at release; at boot the stored file is verified (Ed25519, version floor, device binding) and only the verified bytes are loaded. Tampered or unsigned models stop the perception node.
- **Layered tests**: C++ contract suite, 78 Python tests (including multi-seed closed-loop runs and the guardian decision logic), 59 ROS boundary tests (including the fleet station, deck landing and the guardian station, vehicle, environment and center nodes), four replay state tests, Mermaid/HTML checks, and CI that also runs the suite under `python -O`.
- **Nine recorded review passes**: each defect is paired with its fix and regression test in [the review record](docs/review-report.md).

## Ten-minute tour

1. **See it fly**: the [live flight replay](https://buicongnguyen.github.io/mission-computer-lab/web/sitl.html) (or [`web/sitl.html`](web/sitl.html) in a clone) replays recorded PX4 telemetry, mode transitions and acceptance checks for all four single-drone flights, as a 2D map, a 3D scene or the Gazebo video. The [fleet page](https://buicongnguyen.github.io/mission-computer-lab/web/fleet.html) shows the three drones and the moving carrier, with an overview video and the carrier's deck camera. The [guardian page](https://buicongnguyen.github.io/mission-computer-lab/web/guardian.html) compares the four designs, replays any simulated run and shows the eight PX4 guardian flights. The [fast policy replay](https://buicongnguyen.github.io/mission-computer-lab/web/index.html) covers all eight harness scenarios.
2. **Understand the design**: [architecture and contracts](docs/architecture.md): data path, supervisor protocol, state machine, threat model; and [guardian drones: design and evaluation](docs/guardian.md).
3. **Read the core**: [`src/supervisor.cpp`](src/supervisor.cpp) (health policy, ~80 lines), [`integration/mission_node.py`](integration/mission_node.py) (ROS ↔ PX4 adapter: freshness, frames, arming rules, handoff to PX4), for the fleet [`integration/fleet_station_node.py`](integration/fleet_station_node.py) and [`integration/fleet_mission_node.py`](integration/fleet_mission_node.py), and for the guardians [`tools/guardian.py`](tools/guardian.py), the decision logic both simulators run.
4. **Check the evidence**: [flight results and exact upstream revisions](docs/sitl-execution.md) and [the review record](docs/review-report.md).

## How the project maps to mission-computer work

| Area | What is implemented and tested here | What still requires real hardware |
|---|---|---|
| Autonomy: perception, fusion, planning, navigation | Procedural camera → ONNX Runtime inference; planar LiDAR → occupancy map grown from every scan → A* that replans when a newly seen obstacle closes the route; PX4 EKF2 in SITL; educational Kalman filter in the fast harness | Trained detector, VIO/SLAM, moving-obstacle avoidance |
| ROS 2 middleware and PX4 integration | Typed interfaces (`ros2/mission_interfaces`), best-effort sensor QoS, versioned PX4 topics over XRCE-DDS, offboard velocity control, command ACKs, MAVLink GCS heartbeat | NXP flight-controller hardware link and transport tuning |
| Multi-sensor integration | Camera, LiDAR, GNSS and IMU freshness contracts; ENU/NED conversion; replay-safe capture timestamps; fix-validity checks | Calibration, extrinsics, hardware timestamping |
| Reliability and failure ownership | HOLD/LAND supervisor with dwell times and intermittent-fault escalation; permanent handoff to PX4 on failsafe, disarm, estimator loss or operator mode change; companion-crash failsafe | Hardware-in-the-loop and flight qualification |
| AI/ML deployment | CPU `ExecutionProvider` selected and recorded; latency budget enforced by the supervisor | QNN/NPU conversion, quantization, profiling, power/thermal |
| Platform security | Signed artifact manifest (hash, version floor, device identity) verified from disk at boot; six tamper/rollback/identity cases; threat model | Boot ROM, fuses, TEE, protected key provisioning, persistent anti-rollback |
| Multi-vehicle operations | Three PX4 instances with per-vehicle namespaces and system IDs; a station on a moving carrier that grants launch and landing clearances; altitude layering; lead-compensated landing on a moving deck with go-around | Radio links and their latency or loss, relative positioning on the deck (RTK or vision), deck motion, wind, airspace procedures |
| Threat awareness and protection | Multi-sensor track fusion with camera classification; posture and keep-clear decisions split between drone, station and center by time budget; lost-link procedures; a GNSS spoofing cross-check; seeded evaluation of four designs and a real-PX4 guardian flight | Real sensors (radar, EO/IR, RF) and their performance, authenticated links, rules of engagement, and anything that responds to a threat |
| Telemetry, logging, diagnostics | Independent observer per vehicle, JSONL logs, ROS bags, PX4 logs, a station log, run-time provenance hashes | Fleet telemetry at scale and on-target diagnostics |
| DevOps and testing | CMake/CTest, unittest suites, GitHub Actions, docs drift check, evidence schema gates | Target image builds (Yocto/BSP) |
| Operator dashboards | Four offline replays driven by recorded results (2D, 3D and recorded Gazebo video), with light and dark themes | Live ground-station UI |

## What actually runs

```text
Signed model verification → virtual camera → ONNX CPU → synthetic object bbox
Procedural ROS LaserScan → occupancy from every scan → A* route, replanned in flight
Gazebo flight sensors + dynamics → PX4 EKF2 → ROS telemetry
          ↓
C++17 mission supervisor ↔ Python via actual process IPC
          ↓
ROS offboard velocity + mode/arm/land commands → PX4 → Gazebo x500
Independent observer + ROS bag + firmware logs → acceptance checks → replay

Fleet: station on a moving carrier → launch/land clearances → three copies of the stack above
Guardians: simulated intruder, sensors and jammed links → station fusion and posture → orders; onboard reflexes; center decisions
Gazebo overview and deck cameras → H.264 video in simulation time → replay pages
```

The ONNX graph is hand-authored brightness segmentation. Perception freshness gates mission health; detections do not steer or pursue objects. Procedural LiDAR builds the occupancy map; scans taken in flight trigger a replan if they reveal an obstacle the first scan could not see. PX4 uses EKF2 in SITL; the separate fast harness uses an educational six-state filter. Neither demonstrates GPS-denied SLAM. The MAVLink process supplies a local GCS heartbeat; offboard control uses ROS 2/DDS.

## Run it

**Fast suite** (Ubuntu 22.04 / Python 3.10, a few minutes, no ROS needed):

```bash
bash scripts/bootstrap_wsl.sh
.venv/bin/python -m pip install -r requirements.lock.txt
bash scripts/run_all.sh
.venv/bin/python tools/publish_sample.py
```

Then open `web/index.html`. The committed samples make every replay usable without installing anything; for a loopback server run `python3 -m http.server 8765 --bind 127.0.0.1` and open `http://localhost:8765/web/sitl.html`.

**Real PX4 flights** (after the one-time install and build in [the SITL guide](docs/sitl-guide.md)):

```bash
bash scripts/run_sitl.sh --all --video --output ~/work/mission-computer-lab/retests/my-run
~/work/mission-computer-lab/venv/bin/python tools/publish_sitl.py --input ~/work/mission-computer-lab/retests/my-run
```

`--all` runs the four single-drone scenarios, then the fleet, then the guardians. `--video` records the four single-drone flights, fleet and guardian scenarios configured with `video: True`; `--gui` also opens the live Gazebo window (WSLg). For one scenario use `--scenario guardian_jamming` (or `nominal`, `camera_dropout`, `companion_crash`, `gps_loss`, `fleet_carrier`, or another of the eight `guardian_*` scenarios listed in [the SITL guide](docs/sitl-guide.md)).

**Guardian evaluation** (standard library only, about half a minute on eight cores):

```bash
python3 tools/guardian_sim.py --seeds 40 --workers 8 --output artifacts/guardian
```

Run one instance at a time, with no flight hardware attached. [The complete reproduction guide](docs/complete-reproduction-guide.md) covers fresh setup, every test layer, evidence inspection and the debugging history.

## Evidence

| Layer | Result | Where |
|---|---|---|
| PX4 / Gazebo / ROS 2 flights | Latest retest 12/13; every failed check retained; runtime inputs unchanged | [Current validation](docs/validation.md), [reference replay report](docs/sitl-execution.md) |
| Fast policy scenarios | 8/8 | [Execution report](docs/execution.md) |
| Guardian designs | 4 designs × 8 threat scenarios × 40 seeds; every run keeps both invariants (no keep-clear move closes on a tracked threat, no action outside its authority) | [Guardian results](docs/guardian-results.md), [design and evaluation](docs/guardian.md) |
| Signed-artifact policy cases | 6/6 behave as specified | Same report |
| C++ supervisor contracts | 1 CTest program, including degraded-link escalation | [`tests/test_supervisor.cpp`](tests/test_supervisor.cpp) |
| Python unit and process tests | 78 | [`tests/`](tests/) |
| ROS boundary and runner tests | 59 | [`integration/test_*.py`](integration/) |

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

**Executed:** PX4 SITL (one vehicle, and three at once for the fleet and the guardians), Gazebo physics and flight sensors, a driven carrier vehicle, Gazebo-rendered recording cameras, EKF2, ROS 2 typed payload messages, DDS, MAVLink heartbeat, procedural camera/LiDAR, CPU ONNX, A*, C++ supervision and process kill, Ed25519 artifact verification, fault checks and recorded replays.

**Not executed:** Qualcomm BSP/boot ROM/fuses/TEE, NPU/GPU inference, rendered Gazebo payload camera/LiDAR (the rendered cameras only film the flights), inter-vehicle radio links, relative positioning for deck landing, deck motion or wind, re-launching from a moving deck (stock PX4 GNSS checks read the deck's motion as drift; see [the SITL guide](docs/sitl-guide.md)), real threat sensors (the guardians' threats, sensors, jamming and GNSS spoofing are simulated, with illustrative parameters), any response to a threat (the guardians are non-kinetic by design), trained detection, VIO, real OTA installation or physical drone tests. Battery input in the SITL adapter is fixed at a simulated 95%; low-battery policy is tested in the fast harness. The signing key is ephemeral per run and its public key is not in protected storage.

No credentials or production signing keys are included. This repository does not establish safety, security or aerospace certification.

## Repository map

| Path | Purpose |
|---|---|
| `src/`, `include/` | C++ supervisor, validation and process protocol |
| `tools/` | Sensors, perception, estimator/planner, security, evidence harness and publishers; `guardian.py` and `guardian_sim.py` for the guardian decision logic and its evaluation |
| `integration/`, `ros2/` | ROS nodes (including the fleet and guardian station, vehicle, environment and center nodes), typed interfaces, MAVLink heartbeat and real flight orchestrator |
| `simulation/` | Gazebo worlds: single-drone inspection, the fleet with a moving carrier, and the guardian world, which the runner fills with each scenario's threats; each has recording cameras |
| `tests/` | Policy, model, planning, localization, security, transport and evidence checks |
| `scripts/` | WSL bootstrap, SITL install/build and one-command verification |
| `web/` | Replay dashboards (2D, 3D, video) driven by recorded results; three.js vendored in `web/vendor/` |
| `artifacts/sample/`, `artifacts/sitl-sample/`, `artifacts/guardian/` | Compact reference evidence for the replays and the guardian evaluation |
| `docs/` | Architecture, runbooks, domain notes, references, execution reports and review record (Markdown with rendered HTML) |

Python dependency versions are pinned in `requirements.lock.txt` (fast suite) and `requirements-sitl.lock.txt` (flight stack).

## References

The core open-source stack, pinned to the versions the evidence was recorded with. [The full reference list](docs/references.md) adds the tooling libraries, the upstream guides followed, and the Qualcomm and NXP target-platform documentation.

| Project | Version | Source | Documentation |
|---|---|---|---|
| PX4 Autopilot | v1.16.0 | [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot/tree/v1.16.0) | [docs.px4.io/v1.16](https://docs.px4.io/v1.16/en/) |
| px4_msgs | release/1.16 | [PX4/px4_msgs](https://github.com/PX4/px4_msgs/tree/release/1.16) | [uXRCE-DDS bridge](https://docs.px4.io/v1.16/en/middleware/uxrce_dds) |
| Micro XRCE-DDS Agent | v2.4.3 | [eProsima/Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.3) | [micro-xrce-dds.docs.eprosima.com](https://micro-xrce-dds.docs.eprosima.com/) |
| ROS 2 | Humble | [ros2/ros2](https://github.com/ros2/ros2) | [docs.ros.org/en/humble](https://docs.ros.org/en/humble/) |
| Gazebo | Harmonic | [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim), [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) | [gazebosim.org](https://gazebosim.org/docs/harmonic/getstarted/) |
| ONNX Runtime | 1.20.1 | [microsoft/onnxruntime](https://github.com/microsoft/onnxruntime) | [onnxruntime.ai](https://onnxruntime.ai/) |
| MAVLink | pymavlink 2.4.49 | [mavlink/mavlink](https://github.com/mavlink/mavlink) | [mavlink.io](https://mavlink.io/en/) |
| three.js | 0.147.0 | [mrdoob/three.js](https://github.com/mrdoob/three.js/tree/r147) | [threejs.org](https://threejs.org/docs/) |
