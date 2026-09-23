# Execution evidence

Accelerated kinematic simulation; synthetic sensors; CPU ONNX; no Qualcomm hardware/PX4/ROS 2

Generated UTC: 2026-09-23T16:56:28Z

| Scenario | Result | End | ONNX p95 (ms) | Supervisor IPC p95 (ms) | Localization RMSE (m) |
|---|---|---|---:|---:|---:|
| nominal | PASS | COMPLETE | 0.032 | 0.074 | 0.043 |
| camera_dropout | PASS | COMPLETE | 0.031 | 0.074 | 0.044 |
| gps_dropout | PASS | LAND | 0.030 | 0.069 | 0.222 |
| link_dropout | PASS | LAND | 0.028 | 0.068 | 0.044 |
| inference_overrun | PASS | COMPLETE | 0.028 | 0.071 | 0.044 |
| low_battery | PASS | LAND | 0.034 | 0.082 | 0.043 |
| imu_dropout | PASS | LAND | 0.033 | 0.075 | 0.074 |
| companion_crash | PASS | LAND | 0.027 | 0.070 | 0.044 |

Timing is measured wall-clock time in WSL, not a real-time guarantee or Qualcomm benchmark. The injected 120 ms inference fault is a synthetic reported latency; it does not sleep or emulate an NPU.

Security policy checks: 6/6.

Raw environment, model/binary hashes, per-scenario checks, events and replay frames: report.json.
