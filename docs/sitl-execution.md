# PX4 / ROS 2 / Gazebo execution evidence

**Historical passing reference:** this replay predates the latest safety fixes. The current retest passes 12/13 scenarios; see [current validation and remaining failures](validation.md). The publisher refused to replace this sample with a failed matrix.

Actual PX4/Gazebo/ROS; procedural camera/lidar; CPU synthetic ONNX; no Qualcomm hardware.

Generated UTC: 2026-09-26T10:01:59.246224+00:00

| Scenario | Result | Wall duration (s) | Max estimated altitude (m) | Min estimated obstacle clearance (m) |
|---|---|---:|---:|---:|
| nominal | PASS | 82.94 | 3.051 | 1.204 |
| camera_dropout | PASS | 84.20 | 2.983 | 1.186 |
| companion_crash | PASS | 49.80 | 2.731 | 3.538 |
| gps_loss | PASS | 54.21 | 2.683 | 3.619 |

All four scenarios require actual armed offboard state, observed climb, land mode, landed state and final disarm. Normal and camera-recovery runs also require a reached goal, COMPLETE and accepted land command. GPS loss requires a post-injection stale-GNSS landing decision. Companion crash requires a subsequent PX4 failsafe.

Clearance uses valid recorded PX4 position estimates against known cylinders. Invalid estimates after injected GPS loss are excluded; clearance is not established for that degraded interval. This is not Gazebo ground-truth collision verification. Wall duration includes startup and process cleanup.

## Fleet from a moving carrier

Three PX4 instances launch in sequence from pads on a carrier vehicle, fly separate inspection legs at 3, 4 and 5 m, and land back on the carrier while it drives. A ground-station node grants one launch and one landing at a time; each vehicle keeps its own C++ supervisor and PX4 failsafes.

| Vehicle | Goal | Altitude layer (m) | Touchdown pad error (m) | Carrier speed at touchdown (m/s) | Min obstacle clearance (m) |
|---|---|---:|---:|---:|---:|
| px4_0 | (9, 9) | 3 | 0.052 | 0.27 | 1.19 |
| px4_1 | (10, 3) | 4 | 0.044 | 0.27 | 1.02 |
| px4_2 | (-1, 10) | 5 | 0.048 | 0.27 | 2.19 |

Minimum separation between airborne vehicles: 2.00 m. Carrier travel: 12.0 m. Recording two cameras slows this simulation below real time; the adapters judge freshness on simulation time, as PX4 does.

## Guardians against threats

Three PX4 instances hold watch posts around the carrier while each scenario adds its own threats, a jammer, a GNSS spoofer or a dead link to the center; the station and each guardian run the same decision code as the fast simulator. Separations, drift and landings are measured on Gazebo truth. See [the guardian design](guardian.md).

| Scenario | What happens | Checks | Warning (simulated s) | Closest guardian to a threat (m) | Recovery decided by |
|---|---|---:|---:|---:|---|
| `guardian_intruder` | One intruder, links intact | 38 / 38 | 12.3 | 5.96 | center |
| `guardian_fast` | A fast object diving on the carrier | 37 / 37 | 1.8 | 5.01 | center |
| `guardian_swarm` | Three intruders from two sectors | 38 / 38 | 12.6 | 4.98 | center |
| `guardian_birds` | Circling birds and sensor clutter | 35 / 35 | — | — | center |
| `guardian_jamming` | An intruder while one guardian is jammed | 39 / 39 | 12.6 | 4.17 | center |
| `guardian_spoofing` | A GNSS drag-off on every receiver | 37 / 37 | — | — | center |
| `guardian_center_loss` | An intruder with no link to the center | 37 / 37 | 12.6 | 5.93 | station (delegated) |
| `guardian_combined` | Two intruders, a jammed guardian, birds and no center | 39 / 39 | 12.2 | 2.63 | station (delegated) |

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
