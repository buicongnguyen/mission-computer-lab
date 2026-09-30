# Results

What has been run, what it showed, and where the evidence is. Every PX4 figure below comes from one flight matrix, recorded on 30 September 2026 on the committed source and published as it ran; the [replays](../web/sitl.html) and videos are that same run.

## PX4 flights: 13 of 13 pass

Real PX4 v1.16 firmware in Gazebo Harmonic with ROS 2 Humble, on Ubuntu 22.04 in WSL2. Each flight is judged by named checks computed from independently recorded logs, and for the guardians from Gazebo truth ([how](sitl-guide.md)). 456/456 named checks passed; none failed.

| Scenario | What it shows | Checks | Key measurement |
|---|---|---:|---|
| `nominal` | Take-off, an A*-planned inspection route at 3 m, landing | 22/22 | Highest estimate 2.95 m; closest estimated approach to an obstacle 1.19 m |
| `camera_dropout` | Camera frames stop for 0.8 s in flight | 25/25 | HOLD 0.80 s (wall clock) after the runner signalled the dropout, ACTIVE 1.7 s later, mission completed |
| `companion_crash` | The mission computer's processes are killed in flight | 20/20 | PX4 failsafe 0.99 s after the kill; landed and disarmed |
| `gps_loss` | The GNSS fix is removed in flight | 21/21 | LAND 4.2 s after the fix was lost; landed and disarmed |
| `fleet_carrier` | Three drones launch from, and land back on, a moving carrier | 36/36 | Touchdowns 7.8, 2.7 and 6.8 cm from the pads with the carrier at 0.27 m/s; closest airborne approach 1.48 m |
| `guardian_intruder` | One intruder, links intact | 42/42 | RED 12.1 s before arrival; the carrier relocated 10.0 m clear; closest guardian 4.77 m |
| `guardian_fast` | A fast object diving on the carrier | 41/41 | RED 1.9 s before impact; the nearest guardian 5.9 m from the impact point at impact (post 3.6 m) |
| `guardian_swarm` | Three intruders from two sectors | 42/42 | Three distinct tracks confirmed; closest guardian 4.39 m |
| `guardian_birds` | Circling birds and sensor clutter | 39/39 | No RED; every guardian held its post |
| `guardian_jamming` | An intruder while one guardian is jammed | 43/43 | The jammed guardian kept clear on its own, 5.44 m from the intruder at the closest |
| `guardian_spoofing` | A GNSS drag-off on every receiver | 41/41 | Flagged 10 s after the drag began; drift at most 1.41 m |
| `guardian_center_loss` | An intruder with no link to the center | 41/41 | Recovery under delegation, 20 s after the request |
| `guardian_combined` | Two intruders, a jammed guardian, birds and no center | 43/43 | Closest guardian 1.93 m; recovery under delegation |

Two things make these numbers trustworthy. First, a required check that cannot be evaluated fails; it is never left out, and the code that decides PASS is itself tested ([`integration/acceptance.py`](https://github.com/buicongnguyen/mission-computer-lab/blob/main/integration/acceptance.py), [`tests/test_acceptance.py`](https://github.com/buicongnguyen/mission-computer-lab/blob/main/tests/test_acceptance.py)). Second, the runner hashes the flight code, everything it imports and the built binaries before the first flight and again after the last (40 files, unchanged), and the publisher refuses a run with any failed check or changed input. The replays, videos and this page were then generated from that run.

## Guardian designs in the fast simulator

Four designs (station only, onboard only, networked, hybrid) against eight threat scenarios, 40 seeds each: 1280 runs, plus a layout experiment. Both safety invariants held in every run: every keep-clear or dispersal move was safe against the tracks it was decided on, and no layer acted outside its authority. The hybrid design kept every guardian clear in every run of five of the six threat scenarios; against a 250 m/s object diving on the station, the overwatch above it escaped in 10% of runs, and 90% once offset 250 m. Details: [guardian results](guardian-results.md) and [the design](guardian.md).

## Fast policy harness

Eight fault scenarios (nominal, camera dropout, GNSS loss, link loss, inference overrun, low battery, IMU loss, companion crash) pass on a kinematic simulator with an educational Kalman filter, and six signed-artifact cases behave as specified. [Execution report](execution.md).

## Tests

1 C++ contract program (CTest), 102 Python unit and process tests, 68 ROS boundary and runner tests, and 6 browser page tests. CI runs all of them on every push, the ROS tests inside the official ROS 2 Humble container; it also runs the Python tests under `python -O`, lints, and fails if the committed HTML is stale. The PX4/Gazebo flights run locally, not in CI.

## Where to look next

- [Engineering findings](findings.md): what building and flying this taught, in eighteen short entries.
- [Review record](review-report.md): every finding of ten review passes, with its fix and test.
- [Validation history](validation.md): each earlier flight matrix, including the failed ones.
- [SITL guide](sitl-guide.md): how to run the flights, their topics and every acceptance check.
