# PX4 / ROS 2 / Gazebo execution evidence

Actual PX4/Gazebo/ROS; procedural camera/lidar; CPU synthetic ONNX; no Qualcomm hardware.

Generated UTC: 2026-09-24T18:22:42.343244+00:00

| Scenario | Result | Wall duration (s) | Max estimated altitude (m) | Min estimated obstacle clearance (m) |
|---|---|---:|---:|---:|
| nominal | PASS | 131.64 | 3.016 | 1.171 |
| camera_dropout | PASS | 127.66 | 3.048 | 1.158 |
| companion_crash | PASS | 66.79 | 2.768 | 3.977 |
| gps_loss | PASS | 59.10 | 2.742 | 3.728 |

All four scenarios require actual armed offboard state, observed climb, land mode, landed state and final disarm. Normal and camera-recovery runs also require a reached goal, COMPLETE and accepted land command. GPS loss requires a post-injection stale-GNSS landing decision. Companion crash requires a subsequent PX4 failsafe.

Clearance uses valid recorded PX4 position estimates against known cylinders. Invalid estimates after injected GPS loss are excluded; clearance is not established for that degraded interval. This is not Gazebo ground-truth collision verification. Wall duration includes startup and process cleanup.

## Fleet from a moving carrier

Three PX4 instances launch in sequence from pads on a carrier vehicle, fly separate inspection legs at 3, 4 and 5 m, and land back on the carrier while it drives. A ground-station node grants one launch and one landing at a time; each vehicle keeps its own C++ supervisor and PX4 failsafes.

| Vehicle | Goal | Altitude layer (m) | Touchdown pad error (m) | Carrier speed at touchdown (m/s) | Min obstacle clearance (m) |
|---|---|---:|---:|---:|---:|
| px4_0 | (9, 9) | 3 | 0.030 | 0.27 | 1.23 |
| px4_1 | (10, 3) | 4 | 0.034 | 0.27 | 1.36 |
| px4_2 | (-1, 10) | 5 | 0.037 | 0.27 | 2.16 |

Minimum separation between airborne vehicles: 1.71 m. Carrier travel: 11.7 m. Recording two cameras slows this simulation below real time; the adapters judge freshness on simulation time, as PX4 does.

## Exact upstream revisions

- PX4 v1.16.0: `6ea3539157ca358c70a515878b77077af7d4611d`
- px4_msgs release/1.16: `392e831c1f659429ca83902e66820d7094591410`
- XRCE Agent v2.4.3: `73622810d984349b80bbac0ef55fc0b694d62222`

## Evidence and limitations

- [Interactive replay](../web/sitl.html) and [machine-readable report](../artifacts/sitl-sample/report.json).
- Per-scenario reference folders preserve final parameters, firmware log, mission/perception logs and acceptance result.
- Runtime source/binary hashes and installed versions were captured before flight; inputs were checked again after the matrix. Per-file SHA-256 hashes identify the exact tested inputs, including changes not yet committed when tested.
- Original run folders retain ROS bags and independent observer JSONL; Linux runtime folders retain PX4 ULogs.
- Camera and LiDAR are procedural ROS publishers. PX4 flight sensors and vehicle dynamics come from Gazebo.
- Battery supervision uses a fixed simulated 95% input in this adapter; low-battery testing remains in the accelerated suite.
- The model is a hand-authored brightness segmentation graph on CPU. No trained drone classifier, NPU profiling, VIO or Qualcomm boot-chain validation is claimed.
- DDS discovery uses domain 42 and the normal network transports. This is a simulation setup, not a secured operational deployment.

Reproduce with [the WSL SITL guide](sitl-guide.md).
