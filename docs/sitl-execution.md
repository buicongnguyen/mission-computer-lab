# PX4 / ROS 2 / Gazebo execution evidence

Actual PX4/Gazebo/ROS; procedural camera/lidar; CPU synthetic ONNX; no Qualcomm hardware.

Generated UTC: 2026-09-23T17:05:39.355922+00:00

| Scenario | Result | Wall duration (s) | Max estimated altitude (m) | Min estimated obstacle clearance (m) |
|---|---|---:|---:|---:|
| nominal | PASS | 49.75 | 3.027 | 1.183 |
| camera_dropout | PASS | 50.93 | 2.988 | 1.228 |
| companion_crash | PASS | 30.12 | 2.952 | 3.345 |
| gps_loss | PASS | 33.35 | 2.835 | 3.130 |

All four scenarios require actual armed offboard state, observed climb, land mode, landed state and final disarm. Normal and camera-recovery runs also require a reached goal, COMPLETE and accepted land command. GPS loss requires a post-injection stale-GNSS landing decision. Companion crash requires a subsequent PX4 failsafe.

Clearance uses valid recorded PX4 position estimates against known cylinders. Invalid estimates after injected GPS loss are excluded; clearance is not established for that degraded interval. This is not Gazebo ground-truth collision verification. Wall duration includes startup and process cleanup.

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
