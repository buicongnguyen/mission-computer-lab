# PX4 / ROS 2 / Gazebo execution evidence

Actual PX4/Gazebo/ROS; procedural camera/lidar; CPU synthetic ONNX; no Qualcomm hardware.

Generated UTC: 2026-09-30T20:26:54.928928+00:00

| Scenario | Result | Wall duration (s) | Max estimated altitude (m) | Min estimated obstacle clearance (m) |
|---|---|---:|---:|---:|
| nominal | PASS | 83.86 | 2.950 | 1.188 |
| camera_dropout | PASS | 85.33 | 3.005 | 1.149 |
| companion_crash | PASS | 50.57 | 2.744 | 3.479 |
| gps_loss | PASS | 52.90 | 2.745 | 3.699 |

All four scenarios require actual armed offboard state, observed climb, land mode, landed state and final disarm. Normal and camera-recovery runs also require a reached goal, COMPLETE and accepted land command. GPS loss requires a post-injection stale-GNSS landing decision. Companion crash requires a subsequent PX4 failsafe.

Clearance uses valid recorded PX4 position estimates against known cylinders. Invalid estimates after injected GPS loss are excluded; clearance is not established for that degraded interval. This is not Gazebo ground-truth collision verification. Wall duration includes startup and process cleanup.

## Fleet from a moving carrier

Three PX4 instances launch in sequence from pads on a carrier vehicle, fly separate inspection legs at 3, 4.5 and 6 m, and land back on the carrier while it drives. A ground-station node grants one launch at a time and one landing at a time, lowest layer first; each vehicle keeps its own C++ supervisor and PX4 failsafes.

| Vehicle | Goal | Altitude layer (m) | Touchdown pad error (m) | Carrier speed at touchdown (m/s) | Min obstacle clearance (m) |
|---|---|---:|---:|---:|---:|
| px4_0 | (9, 9) | 3 | 0.078 | 0.27 | 1.15 |
| px4_1 | (10, 3) | 4.5 | 0.027 | 0.27 | 1.05 |
| px4_2 | (-1, 10) | 6 | 0.068 | 0.27 | 2.17 |

Minimum separation between airborne vehicles: 1.48 m. Carrier travel: 12.0 m. Recording two cameras slows this simulation below real time; the adapters judge freshness on simulation time, as PX4 does.

## Guardians against threats

Three PX4 instances hold watch posts around the carrier while each scenario adds its own threats, a jammer, a GNSS spoofer or a dead link to the center; the station and each guardian run the same decision code as the fast simulator. Separations, drift and landings are measured on Gazebo truth. See [the guardian design](guardian.md).

| Scenario | What happens | Checks | Warning (simulated s) | Closest guardian to a threat (m) | Recovery decided by |
|---|---|---:|---:|---:|---|
| `guardian_intruder` | One intruder, links intact | 42 / 42 | 12.1 | 4.77 | center |
| `guardian_fast` | A fast object diving on the carrier | 41 / 41 | 1.9 | 3.84 | center |
| `guardian_swarm` | Three intruders from two sectors | 42 / 42 | 12.2 | 4.39 | center |
| `guardian_birds` | Circling birds and sensor clutter | 39 / 39 | — | — | center |
| `guardian_jamming` | An intruder while one guardian is jammed | 43 / 43 | 12.1 | 5.43 | center |
| `guardian_spoofing` | A GNSS drag-off on every receiver | 41 / 41 | — | — | center |
| `guardian_center_loss` | An intruder with no link to the center | 41 / 41 | 12.3 | 5.61 | station (delegated) |
| `guardian_combined` | Two intruders, a jammed guardian, birds and no center | 43 / 43 | 12.1 | 1.93 | station (delegated) |

## Exact upstream revisions

- PX4 v1.16.0: `6ea3539157ca358c70a515878b77077af7d4611d`
- px4_msgs release/1.16: `392e831c1f659429ca83902e66820d7094591410`
- XRCE Agent v2.4.3: `73622810d984349b80bbac0ef55fc0b694d62222`

## Evidence and limitations

- Interactive replays of the [single-drone flights](../web/sitl.html), the [fleet](../web/fleet.html) and the [guardians](../web/guardian.html), and the [machine-readable report](../artifacts/sitl-sample/report.json).
- Per-scenario reference folders preserve final parameters, firmware log, mission/perception logs and acceptance result.
- Runtime source/binary hashes and installed versions were captured before flight; inputs were checked again after the matrix. Per-file SHA-256 hashes identify the exact tested inputs, including changes not yet committed when tested.
- Original run folders retain ROS bags and independent observer JSONL; Linux runtime folders retain PX4 ULogs.
- Camera and LiDAR are procedural ROS publishers. PX4 flight sensors and vehicle dynamics come from Gazebo.
- Battery supervision uses a fixed simulated 95% input in this adapter; low-battery testing remains in the accelerated suite.
- The model is a hand-authored brightness segmentation graph on CPU. No trained drone classifier, NPU profiling, VIO or Qualcomm boot-chain validation is claimed.
- DDS discovery uses domain 42 and the normal network transports. This is a simulation setup, not a secured operational deployment.

Reproduce with [the WSL SITL guide](sitl-guide.md).
