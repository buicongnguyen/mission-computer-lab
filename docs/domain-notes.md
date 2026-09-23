# Domain notes: mission computer and flight-controller boundaries

Research date: **22 September 2026**. These notes explain the engineering context behind the project: what a Qualcomm-based mission computer paired with an NXP-based PX4 flight controller has to do, and which of those responsibilities this repository executes, models or leaves for real hardware. Public vendor documentation supports the technical interpretation. Work areas and priorities are engineering judgments.

## Responsibilities and project coverage

A mission-computer platform role is systems integration and platform ownership: connect an autonomy computer to a separate flight controller, make perception and navigation workloads dependable, and carry secure platform bring-up through to manufacturing readiness. The difficult part is explaining and containing failures across those boundaries, not only implementing an object detector.

| Work area | Typical engineering work | Evidence a senior engineer would produce | Coverage in this project |
|---|---|---|---|
| Platform bring-up | Select and integrate a BSP, inspect boot logs, enable buses and peripherals, debug kernel/device-tree problems | Reproducible image build, board/version matrix, bring-up report, peripheral tests | Migration plan only; no target hardware |
| Boot and manufacturing trust | Work with security and factory teams on authorized image chains, device identity, provisioning, lifecycle transitions and controlled debug | Threat model, provisioning design, failure/recovery procedures, traceability | Release-signed model verified from disk at boot in user space; hardware chain modeled |
| Sensor integration | Define timestamps and coordinate frames; calibrate cameras, LiDAR and IMU; handle skew, drops and degraded quality | Interface contracts, calibration artifacts, recorded data, freshness tests | Synthetic camera/LiDAR, PX4 IMU/GNSS, explicit freshness and frame contracts |
| AI deployment | Export and validate a model; select supported operators; quantify accuracy, memory and latency; investigate fallback | Model card, conversion log, device profiling trace, numerical comparison | Real ONNX Runtime CPU execution of a hand-authored graph |
| Autonomy | Combine estimates with goals and environmental constraints; implement planning and health gates | State machine, requirements, replayable failures, field-test criteria | PX4 EKF2 in SITL, educational filter in the fast harness, A*, C++ mission gate |
| Flight-controller integration | Separate mission intent from stabilization; manage transport, units, frames, acknowledgments and loss behavior | Versioned message contract, integration tests, autopilot failsafe evidence | Actual PX4/Gazebo/ROS 2 offboard flight, crash failsafe and operator-takeover handoff |
| Software assurance | Make changes reviewable; isolate failures; measure regressions; support cross-functional triage | CI, traceability, fault matrix, logs, release evidence | Local test layers, fault matrix, provenance hashes, CI workflow, recorded review passes |

## Two computers, two responsibilities

The **mission computer** consumes higher-level sensor data, runs perception/localization/planning, and sends bounded mission intent. The **flight controller** owns fast stabilization and must retain appropriate failure behavior if the mission computer crashes. The project demonstrates this with actual PX4 offboard-loss handling after a mission-process kill; its fast harness also has an independent stub watchdog.

PX4's ROS 2 integration exposes uORB topics through a client/agent bridge. With uXRCE-DDS, the client runs on PX4 and the agent normally runs on the companion. Message definitions and firmware versions must agree. For the documented v1.16 route, PX4 recommends Humble on Ubuntu 22.04. This is a reproducibility baseline, not a claim that v1.16 is the newest release. [PX4 ROS 2 guide](https://docs.px4.io/v1.16/en/ros2/user_guide)

NXP flight controllers are not interchangeable. NXP documents the MR-VMU-RT1176 as a specific vehicle-management platform, and PX4 has a corresponding board guide. A real integration starts from the exact part number, carrier revision, firmware branch and enabled transport. [NXP platform](https://www.nxp.com/design/design-center/development-boards-and-designs/VEHICLE-MANAGEMENT-UNIT), [PX4 board guide](https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176)

## RB5, IQ9 and what WSL can represent

RB5 is a useful architectural reference, but the architecture here is **IQ9-oriented at the interface level** while the simulator stays hardware-independent. The vendor's IQ9 product brief identifies an IQ-9075 configuration and Linux/Ubuntu options; the supported software stack depends on the exact product and BSP. An RB5 binary, driver, security flow or NPU package should not be assumed to transfer unchanged. [Qualcomm IQ9 product brief](https://docs.qualcomm.com/bundle/publicresource/87-83840-1_REV_A_Qualcomm_IQ9_Series_Product_Brief.pdf)

Qualcomm's June 2026 Linux update describes a unified IoT Linux offering with versioned platform components. Release qualification therefore depends on whether a program uses Qualcomm Linux, an Ubuntu vendor image or a custom Yocto BSP. Installing Ubuntu x86-64 in WSL does not reproduce any of those board images. [Qualcomm Linux 2.0 announcement](https://www.qualcomm.com/developer/blog/2026/06/qualcomm-linux-2-now-available)

| Can execute in this WSL lab | Can model or reason about | Requires an actual supported target or service |
|---|---|---|
| C++ logic, Python tooling, ONNX CPU, tests, artifact signatures | Boot stages, watchdog policy, data contracts, lifecycle design | Qualcomm boot ROM and fuses |
| Synthetic sensors and estimator/planner behavior | NPU latency budgets and overload policies | Qualcomm accelerator execution and power/thermal measurements |
| Local process IPC, ROS 2/DDS, MAVLink heartbeat and recorded replay | Secured operational deployment topology | Real sensor calibration, drivers and radio behavior |
| Fault injection in a simplified world | Recovery requirements | Airworthiness/safety evidence and flight validation |

## AI/NPU: the deployment boundary

ONNX is a model representation; choosing that format does not make execution accelerated. This project explicitly selects `CPUExecutionProvider` and records it in the evidence. The QNN execution provider documentation describes Qualcomm acceleration through the QNN SDK and lists platform/backend constraints. Its Windows/Android instructions do not carry over directly to an Ubuntu IQ9 board; the board vendor's supported Linux deployment path and package matrix come first. [ONNX Runtime QNN provider](https://onnxruntime.ai/docs/execution-providers/QNN-ExecutionProvider.html)

A credible accelerator experiment would compare the same inputs and model on CPU and supported Qualcomm hardware, verify output differences, inspect per-operator placement, then record warm/cold latency, memory, throughput, power and temperature. Qualcomm AI Hub offers device-backed compilation/profiling workflows, subject to target availability. No cloud profiling was run here. [Qualcomm AI Hub](https://workbench.aihub.qualcomm.com/docs/index.html)

The generated detector is intentionally a **hand-authored brightness-segmentation graph**, not trained ML. It validates tensor preprocessing, model loading, invocation, output interpretation, hashes and timing. It has no credible real-drone detection accuracy. A next-stage model needs documented dataset permission, class definition, train/validation/test separation, small-object recall, false alarms, latency and lighting/weather coverage. A visual drone class cannot establish hostile intent.

## Reliability, middleware and security details

ROS 2 QoS affects whether endpoints connect and how sensor data is delivered; its sensor-data profile favors best effort and small queues. Each topic's policy should be chosen deliberately, then tested for requested/offered compatibility and stale-data behavior. The integration uses real DDS with best-effort sensor subscriptions and typed application messages. The C++ core retains its local pipe boundary. [ROS 2 QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

PX4 offboard control needs a continuing proof-of-life stream and appropriate loss configuration. Its documented threshold is greater than 2 Hz, with a stream established before entry and loss action governed by parameters. The integration publishes at 20 Hz and tests the application freshness policy separately from PX4's actual offboard-loss response. [PX4 offboard behavior](https://docs.px4.io/v1.16/en/flight_modes/offboard)

MAVLink 2 signing authenticates messages; signing is not encryption. Confidentiality, authorization and replay behavior are separate design requirements. Neither MAVLink signing nor transport encryption is implemented in this prototype. [MAVLink signing](https://mavlink.io/en/guide/message_signing.html)

Qualcomm describes a hardware-anchored boot authentication chain. An Ed25519 verifier running inside an already-running Linux process does not reproduce that chain. The signed model manifest here is a narrow artifact-integrity exercise: sign at release, verify the stored file at boot, load only verified bytes. Production needs protected anchors, secure lifecycle rules, persistent rollback state and controlled recovery. No fuse addresses or programming steps are inferred from generic public material. [Qualcomm secure boot architecture](https://www.qualcomm.com/developer/blog/2024/12/secure-boot-as-part-of-platform-security-architecture-modern-system-on-chip)

Security and aerospace process standards such as ISO 27001 and DO-326A are organizational assurance frameworks. This repository does not establish compliance or certification with them.

## Scope choices

The project deliberately favors one complete, inspectable path with meaningful failures over breadth. A full stack with a trained detector, GPS-denied SLAM and NPU execution would be more realistic, but it could not be validated end to end in the available time, and a polished dashboard should never imply that unexecuted parts were completed. The chosen evidence is failure behavior and honest boundaries: GPS loss, camera recovery, intermittent-fault escalation, a dead companion process, operator takeover and rejected artifact tampering, each with a concrete path from stub to production technology in [the architecture guide](architecture.md). Yocto work would begin with layers, recipes, BSP configuration and reproducible builds. [Yocto quick build](https://docs.yoctoproject.org/brief-yoctoprojectqs/index.html)
