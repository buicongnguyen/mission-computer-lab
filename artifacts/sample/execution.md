# Execution evidence

Accelerated kinematic simulation; synthetic sensors; CPU ONNX; no Qualcomm hardware/PX4/ROS 2

Generated UTC: 2026-09-24T00:45:55Z

| Scenario | Result | End | ONNX p95 (ms) | Supervisor IPC p95 (ms) | Localization RMSE (m) |
|---|---|---|---:|---:|---:|
| nominal | PASS | COMPLETE | 0.057 | 0.134 | 0.043 |
| camera_dropout | PASS | COMPLETE | 0.055 | 0.137 | 0.044 |
| gps_dropout | PASS | LAND | 0.049 | 0.125 | 0.196 |
| link_dropout | PASS | LAND | 0.059 | 0.139 | 0.043 |
| inference_overrun | PASS | COMPLETE | 0.056 | 0.143 | 0.044 |
| low_battery | PASS | LAND | 0.072 | 0.141 | 0.042 |
| imu_dropout | PASS | LAND | 0.097 | 0.245 | 0.073 |
| companion_crash | PASS | LAND | 0.067 | 0.162 | 0.044 |

Timing is measured wall-clock time in WSL, not a real-time guarantee or Qualcomm benchmark. The injected 120 ms inference fault is a synthetic reported latency; it does not sleep or emulate an NPU.

Security policy checks: 6/6.

Raw environment, model/binary hashes, per-scenario checks, events and replay frames: report.json.
