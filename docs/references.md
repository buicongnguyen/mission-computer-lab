# References

Every project this lab builds on, with its source repository, its official website or documentation, the exact version pinned here, and what it does in the lab. Links were checked on 24 September 2026. Pinned revisions are enforced by [`scripts/build_sitl.sh`](https://github.com/buicongnguyen/mission-computer-lab/blob/main/scripts/build_sitl.sh), `requirements*.lock.txt` and `package-lock.json`; the flight evidence records them again at run time.

## This project

| | Link |
|---|---|
| Source code | [github.com/buicongnguyen/mission-computer-lab](https://github.com/buicongnguyen/mission-computer-lab) |
| Live replay site | [buicongnguyen.github.io/mission-computer-lab](https://buicongnguyen.github.io/mission-computer-lab/) |
| PX4 flight replay | [web/sitl.html](https://buicongnguyen.github.io/mission-computer-lab/web/sitl.html) |
| Fast policy replay | [web/index.html](https://buicongnguyen.github.io/mission-computer-lab/web/index.html) |

## Flight stack (executed)

| Project | Version used | Source | Website and documentation | Role in this lab |
|---|---|---|---|---|
| PX4 Autopilot | v1.16.0 (`6ea3539`) | [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot/tree/v1.16.0) | [px4.io](https://px4.io/), [PX4 Guide v1.16](https://docs.px4.io/v1.16/en/) | Flight firmware in SITL: EKF2, offboard mode, land mode and failsafes |
| px4_msgs | release/1.16 (`392e831`) | [PX4/px4_msgs](https://github.com/PX4/px4_msgs/tree/release/1.16) | [uXRCE-DDS bridge](https://docs.px4.io/v1.16/en/middleware/uxrce_dds) | ROS 2 message definitions that match the firmware's uORB topics |
| PX4 Gazebo models | Bundled with PX4 v1.16 | [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) | [Gazebo simulation](https://docs.px4.io/v1.16/en/sim_gazebo_gz/) | x500 quadcopter model flown in the inspection world |
| Micro XRCE-DDS Agent | v2.4.3 (`7362281`) | [eProsima/Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.3) | [Micro XRCE-DDS docs](https://micro-xrce-dds.docs.eprosima.com/) | Companion-side agent bridging PX4's client to the DDS/ROS 2 network |
| ROS 2 Humble | Ubuntu 22.04 packages | [ros2/ros2](https://github.com/ros2/ros2) | [docs.ros.org/en/humble](https://docs.ros.org/en/humble/) | Nodes, typed interfaces and QoS for payload, perception and control |
| rosbag2 | Humble | [ros2/rosbag2](https://github.com/ros2/rosbag2) | [docs.ros.org/en/humble](https://docs.ros.org/en/humble/) | Recording the decision, perception and LiDAR topics as evidence |
| Gazebo Harmonic | `gz-harmonic` 1.0.0 | [gazebosim/gz-harmonic](https://github.com/gazebosim/gz-harmonic), [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) | [gazebosim.org](https://gazebosim.org/docs/harmonic/getstarted/) | Vehicle physics and the flight sensors PX4 consumes |
| MAVLink and pymavlink | pymavlink 2.4.49 | [mavlink/mavlink](https://github.com/mavlink/mavlink), [ArduPilot/pymavlink](https://github.com/ArduPilot/pymavlink) | [mavlink.io](https://mavlink.io/en/) | Local ground-station heartbeat |

## Perception, security and tooling

| Project | Version used | Source | Website and documentation | Role in this lab |
|---|---|---|---|---|
| ONNX Runtime | 1.20.1 | [microsoft/onnxruntime](https://github.com/microsoft/onnxruntime) | [onnxruntime.ai](https://onnxruntime.ai/) | CPU inference of the detector, loaded from verified bytes |
| ONNX | 1.17.0 | [onnx/onnx](https://github.com/onnx/onnx) | [onnx.ai](https://onnx.ai/) | Authoring and checking the hand-built segmentation graph |
| NumPy | 1.26.4 | [numpy/numpy](https://github.com/numpy/numpy) | [numpy.org](https://numpy.org/) | Sensor frames, Kalman filter and metrics |
| pyca/cryptography | 44.0.3 | [pyca/cryptography](https://github.com/pyca/cryptography) | [cryptography.io](https://cryptography.io/) | Ed25519 signing and verification of the model manifest |
| Mermaid | 11.17.2 | [mermaid-js/mermaid](https://github.com/mermaid-js/mermaid) | [mermaid.js.org](https://mermaid.js.org/) | Offline diagrams in the guides |
| marked | 17.0.5 | [markedjs/marked](https://github.com/markedjs/marked) | [marked.js.org](https://marked.js.org/) | Rendering the Markdown guides to HTML |
| jsdom | 26.1.0 | [jsdom/jsdom](https://github.com/jsdom/jsdom) | — | Inert DOM for diagram syntax tests |

## Upstream guides used for the integration

- [PX4 ROS 2 user guide (v1.16)](https://docs.px4.io/v1.16/en/ros2/user_guide): client/agent architecture, message alignment and companion setup.
- [PX4 ROS 2 offboard control example (v1.16)](https://docs.px4.io/v1.16/en/ros2/offboard_control): the setpoint and proof-of-life pattern the mission adapter follows.
- [PX4 offboard mode (v1.16)](https://docs.px4.io/v1.16/en/flight_modes/offboard): entry requirements and loss behavior.
- [PX4 uXRCE-DDS bridge (v1.16)](https://docs.px4.io/v1.16/en/middleware/uxrce_dds): versioned topics such as `vehicle_status_v1`.
- [PX4 Gazebo simulation (v1.16)](https://docs.px4.io/v1.16/en/sim_gazebo_gz/): standalone Gazebo mode and model/world selection.
- [ROS 2 quality of service](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html): best-effort sensor subscriptions and endpoint compatibility.

## Target platforms (referenced, not executed)

| Platform | Links | Relevance |
|---|---|---|
| Qualcomm IQ9 series | [Product brief](https://docs.qualcomm.com/bundle/publicresource/87-83840-1_REV_A_Qualcomm_IQ9_Series_Product_Brief.pdf), [secure boot architecture](https://www.qualcomm.com/developer/blog/2024/12/secure-boot-as-part-of-platform-security-architecture-modern-system-on-chip) | Mission-computer target family and its hardware trust chain |
| Qualcomm AI acceleration | [ONNX Runtime QNN execution provider](https://onnxruntime.ai/docs/execution-providers/QNN-ExecutionProvider.html), [Qualcomm AI Hub docs](https://workbench.aihub.qualcomm.com/docs/index.html), [quic/ai-hub-models](https://github.com/quic/ai-hub-models) | Path from the CPU provider used here to NPU execution |
| NXP MR-VMU-RT1176 | [NXP product page](https://www.nxp.com/design/design-center/development-boards-and-designs/VEHICLE-MANAGEMENT-UNIT), [NXP VMU documentation](https://nxp.gitbook.io/vmu-rt1176), [PX4 board guide](https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176) | NXP-based PX4 flight-controller class that the SITL firmware stands in for |

The [domain notes](domain-notes.md) discuss these platforms in context, and the [architecture guide](architecture.md) maps each simulated component to its production counterpart.
