# Logic and code review record

Review dates: 23 September 2026 (passes 1–3), 24 September 2026 (passes 4 and 5), 25 September 2026 (pass 6) and 26 September 2026 (passes 7–8) and 26–27 September 2026 (pass 9). Scope: the WSL mission-computer lab, including C++ policy contracts, Python adapters and transport, flight orchestration, evidence publication, replays and guides. Nine sequential passes were performed over the implementation; later passes also reviewed earlier fixes. This is an engineering review, not a flight qualification or a claim that all possible defects have been eliminated.

The current source contains the fixes below. The measured reference artifacts identify runtime inputs by SHA-256, including source changes that were uncommitted when tested.

## Pass 1 — Mission logic and control boundaries

Followed the flow from source timestamps through mission health, C++ evaluation, arming, velocity publication and landing. Read the C++ state/range rules and its tests, then examined how the ROS adapter used them.

| Finding | Why it mattered | Fix and regression evidence |
|---|---|---|
| High: future capture stamps were clamped to the current time | A future-stamped message could repeatedly appear fresh | Reject future and replayed payload stamps; preserve a fixed capture time on a monotonic clock. Test covers future, repeated and backward-wall-clock cases. |
| High: mode/arm requests did not require ACTIVE | The application could ask to arm while its own supervisor was in INIT or HOLD | Require ACTIVE, PX4 preflight readiness and two seconds of pre-streaming. Test checks INIT, HOLD, failed preflight and the healthy case. |
| High: post-landing core evaluation continued | The original camera reference produced a geofence transition during PX4-owned touchdown after COMPLETE | Latch terminal handoff and stop subsequent core evaluation and offboard commands. Test exercises COMPLETE and LAND, including later ticks. |
| High: failsafe handoff could disappear with a cleared flag | A later status could allow application control to resume after PX4 had taken over | Latch the handoff after an in-flight failsafe, disarm or invalid estimator. Regression checks flag clearing cannot resume the mission. |
| High: response timeout covered only the first readable byte | `readline()` could wait forever on an incomplete line; reading stderr to EOF could also hang while the child lived | Shared bounded pipe reader uses a monotonic deadline through the newline, a 1024-byte limit, sequence/mode/finite-velocity checks, and no blocking stderr drain. Real subprocess tests include a deliberately unfinished line and malformed response. |
| Medium: sampled logs could omit the only terminal decision | Permanent handoff would make an omitted COMPLETE impossible to recover from a later sample | Always record and observe LAND/COMPLETE regardless of normal one-in-five decimation. Test uses an off-cadence sequence. |
| Medium: fast harness could reach its step limit in LAND while airborne | Terminal mode alone did not prove the simulated landing finished | Require reaching COMPLETE or the modeled ground threshold before the step limit; all eight fast scenarios exercise the additional acceptance check. |

Additional payload guards reject malformed LiDAR geometry, NaN/below-minimum ranges and invalid inference durations. They are input sanity checks, not calibration or perception-accuracy validation. Positive infinite LiDAR returns remain valid no-hit values.

The C++ state policy itself did not need to change. Its landing latch and hard-limit priority remain intact. The ownership fix belongs at the ROS boundary: after handoff, the application no longer evaluates mission policy during PX4's descent.

## Pass 2 — Orchestration and evidence reliability

Traced startup, parameter application, process lifetime, fault injection, log reading, acceptance evaluation and publication. Checked what an incomplete experiment could accidentally label successful.

| Finding | Why it mattered | Fix and verification |
|---|---|---|
| High: a process group was cleaned only while its leader lived | Descendant processes could survive an exited launcher and interfere with a later run | Signal the owned group even after leader exit; finish cleanup for all children even if one fails. Two mocked process-lifetime regressions check both branches. |
| Medium: inherited `PX4_PARAM_*` values were silently applied | Unrelated shell settings could alter the configured experiment | Remove inherited parameter overrides and apply only this runner's explicit policy. Recorded parameter logs remain available for inspection. |
| Medium: duration and fault-delay limits used the adjustable wall clock | A host clock adjustment could move experiment deadlines | Use monotonic time for duration, startup and injection delay. Keep wall timestamps for correlating recorded events. |
| High: any malformed JSONL line was ignored | Corrupted observations could disappear without making a run fail | Ignore only an unfinished last line while actively reading a live file. Reject malformed complete lines and malformed final logs. |
| High: disconnected state flags could stand in for a complete flight | Old ground reports and separate armed/land states were insufficient to prove the actual order | Require armed offboard, valid climb above 2 m, armed land mode, touchdown and final disarm in order, plus a valid vertical estimate near ground. A negative fixture deliberately combines stale ground state with airborne observations. |
| Medium: goal and recovery acceptance were too broad | Goal checking omitted altitude, and a HOLD unrelated to the camera could satisfy a recovery check | Require the 3-D goal within 0.5 m; camera HOLD must have `vision_stale`, old camera age and subsequent ACTIVE. |
| High: duplicate scenarios, empty check maps and `assert`-based gates | Incomplete evidence could pass; `python -O` disabled the original checks | Require exactly one of each scenario, explicit Boolean results, exact security-case identities/outcomes and finite increasing traces. Use explicit exceptions. Test duplicate, empty, numeric-truthy and NaN cases. |
| Medium: publication described the environment at publication time | Delayed publication could attribute an old run to changed code | Capture versions, upstream revisions, runtime source and built-binary hashes before flight and compare after the matrix. Publish saved provenance. Validate all required source files and per-case/aggregate agreement before copying. |
| Medium: a reused output root could overwrite its summary | Individual old cases could coexist with a replacement summary | Require a fresh empty root for every SITL invocation. |

The strengthened acceptance gate intentionally refuses old pre-review flight summaries without run-time provenance. Preserve those as historical evidence and rerun the experiment to produce a current reference.

## Pass 3 — Review the fixes, docs and diagram integration

Re-read the modified boundaries and validation code, checked negative cases, and compared the runbooks against executable behavior. This pass found and closed additional gaps in the preceding changes:

| Finding | Resolution |
|---|---|
| A nonempty checks dictionary could still omit a required check | Both reference publishers now require the expected named checks for each scenario. A regression removes the timeout check and expects rejection. |
| Merely finding a bag database did not establish recording | Inspect read-only SQLite message/topic joins and require messages on decision, perception and LiDAR topics. Tests reject empty and partially populated bags. |
| Existing documentation renderer would display Mermaid as code | Convert Mermaid fences to diagram containers, bundle the renderer locally, initialize strict mode, and retain readable source with a failure fallback. |
| Initial Mermaid selection had known dependency advisories | Upgrade to pinned Mermaid 11.17.2; the final npm audit reported zero known advisories. The lockfile records the full dependency resolution. |
| Mermaid label parsing required a DOM even in a syntax-only test | Supply an inert jsdom document for parser tests; it does not load URLs or perform browser layout rendering. |
| The first Linux retest rejected expected horizontal-estimator degradation after injected GPS loss | Record horizontal and vertical validity separately. Require validity before injection, finite positions throughout, and valid vertical estimation at touchdown. A negative regression still rejects invalidity before injection. Clearance excludes invalid-position intervals. Preserve the failed matrix and rerun all four cases. |
| Link checks skipped anchor targets and used Python assertions | Validate local fragment IDs, duplicate IDs and file targets using explicit exceptions. |
| Runbooks still described the previous landing behavior, test counts and publication-time hashes | Update both Markdown and HTML with monotonic freshness, permanent handoff, current commands, run-time provenance and review links. Preserve the old landing issue as history. |
| First full retest stopped with OS `No data available` during a live `/mnt/c` recording | Preserve that failed attempt, add traceback logging for future runner exceptions and move the final raw recording to WSL's Linux filesystem. The initial error lacked a traceback, so its exact failing syscall is unconfirmed. No acceptance checks were weakened. |

The final dependency audit is a point-in-time advisory check, not a security guarantee. Diagram sources are trusted repository documentation; do not reuse the Markdown renderer as an unrestricted HTML publishing service.

<a id="pass-4"></a>
## Pass 4 — Escalation, boot trust and operator authority

Review date: 24 September 2026. Every source file was re-read, and each hypothesis was tested by execution before code changed: a degraded-link frame sequence was fed to the real supervisor binary, the boot gate was called on arbitrary bytes, and the recorded PX4 state sequences of all four flights were checked against each proposed adapter rule.

| Finding | Why it mattered | Fix and regression evidence |
|---|---|---|
| High: intermittent faults never escalated | The fault timer reset on the first healthy sample. With link packets every 0.6 s, a 60 s probe of the real binary alternated between 11-tick HOLDs and single-tick ACTIVE commands: no LAND, and the vehicle crept forward on a failing link | A fault episode now ends only after 2 s of continuous health; any unhealthy sample 2 s or more into an episode latches LAND. A C++ degraded-link test aborts against the previous implementation; a second test shows a recovered single dropout still clears after 2 s of health. Recorded HOLD episodes (camera about 1.1 s; GPS 2.05 s to LAND) were checked first, so the flight outcomes are unchanged |
| High: the boot gate could not fail | `boot_gate` generated a key, signed the bytes it was given and verified them in the same call, so arbitrary bytes returned `VERIFIED IN USER SPACE`. The perception node also created a missing model itself | Release and boot are now separate. `provision` writes a detached manifest and signature; `boot_gate` reads the stored file once, verifies it against a separately supplied public key and returns the exact bytes ONNX Runtime loads, leaving no window between checking and loading. The SITL runner acts as release authority and never writes the private key; the node no longer creates models and exits when verification fails. Four tests cover verified loading, a tampered file, a different trust anchor and a missing signature. A live ROS run logged `boot_rejected: payload_tampered` and exited with code 3 |
| Medium: the adapter would override an operator | Handoff covered PX4 failsafe, disarm and estimator loss, but not a pilot or GCS leaving OFFBOARD without a failsafe. Within 2 s the adapter would have switched the vehicle back to OFFBOARD | Once armed OFFBOARD has been observed, any other navigation state while armed is a permanent `external_mode_change` handoff. Being armed before OFFBOARD during normal entry still requests OFFBOARD once. No recorded flight leaves OFFBOARD before the existing handoff |
| Low: pre-flight estimator loss kept stale proof-of-life | The pre-stream start time survived an estimator dropout, so arming could be requested right after recovery without a fresh two-second pre-stream, and the log line repeated at 20 Hz | Restart the pre-stream on invalidity and log once per episode; regression test |
| Low: evidence tooling details | SQLite handles closed only through garbage collection; a weaker `landed_at_end` check was silently overwritten by the ordered-flight version; the published obstacle table was a hand-maintained copy | `contextlib.closing`, dead check removed, obstacles published from `world.OBSTACLES`. A new test checks that the Gazebo world's cylinders and x500 spawn frame match the planner |
| Process: CI gaps | CI did not run the `python -O` suite that the evidence gates rely on, and nothing detected stale committed HTML | CI now runs the Python suite under `-O` and fails if `npm run docs` changes committed HTML |

| Pass 4 verification | Observed result |
|---|---|
| C++ Release build and CTest | 1/1 passed; the new degraded-link test aborts against the previous `supervisor.cpp` |
| Lightweight Python tests | 28/28 passed; also passed under `python -O` |
| ROS boundary and runner tests | 14/14 passed |
| Fast scenario matrix / signed-artifact cases | 8/8 passed / 6/6 behaved as specified |
| PX4/Gazebo/ROS matrix on the final code | 4/4 passed with every named check; `inputs_unchanged: true`; runner exit code 0 |
| Published evidence | Regenerated from that run; scanned for local paths and personal data |

The flight matrix was rerun after the last source change, so the published per-file provenance hashes match the shared source files exactly. The recorded `source_commit` is the base revision in the development history, which this repository does not include; use the per-file SHA-256 hashes to match evidence to source.

<a id="pass-5"></a>
## Pass 5 — Independent review, planner safety and the published site

Review date: 24 September 2026. Three independent reviewers covered the C++ and Python core, the ROS/PX4 orchestration, and the browser and documentation tooling. Every finding below was reproduced before it was fixed.

| Finding | Why it mattered | Fix and regression evidence |
|---|---|---|
| High: LAND stopped descending at an estimated zero altitude | After GNSS loss the dead-reckoned altitude drifts low; 8 of 20 seeds left the fast-harness vehicle hovering 0.1–0.4 m above ground until timeout. The published PASS relied on seed 17 | LAND now descends at 0.3–0.7 m/s until touchdown is detected. A C++ test covers estimates at and below zero; a multi-seed test runs every landing scenario on 8 seeds |
| Medium: the single-scan map let A* plan through a hidden cylinder | From the origin, the (6,6) cylinder is mostly occluded by (4,3), so six cells inside real obstacles were free. Goals such as (10,6) planned straight through it (clearance −1.1 m); the shipped goal was safe only by tie-breaking | Both harnesses now add every scan to the map, placed where the vehicle was at capture, and replan when the remaining route closes; with no route left the SITL mission lands. All 28 goals whose first plan was unsafe now complete with at least 1.0 m clearance. The shipped route needs no replan, so recorded flights are unchanged |
| Medium: the camera fault was timed from process start | The dropout fired 18 s after the payload node started, so slower startup could have tested it on the ground, and the check accepted any HOLD | The runner now signals the dropout 3 s after the vehicle is observed above 2 m, and the HOLD must follow the injection. Pass 5 flight: airborne at 7.2 s, injected at 10.2 s, HOLD at 10.5 s, recovered at 11.6 s |
| Medium: SIGTERM or SIGHUP orphaned the whole simulation | Children run in their own sessions, so a closed terminal left PX4, Gazebo and the agent running and blocked every later run | The runner converts both signals into a normal exit so its cleanup always runs |
| Medium: the site deployed even when CI failed, and new pages escaped the stale-HTML check | `git diff` ignores untracked files, and the Pages workflow ran independently of tests | Pages now deploys only commits whose CI passed and checks links against the assembled site. CI fails on any untracked or modified generated HTML |
| Medium: the flight replay showed PX4 DESCEND as a raw `12` | The headline GPS-loss failsafe was unreadable; preflight-only changes produced duplicate rows and the first event read `-0.00 s` | Full PX4 v1.16 state names, preflight shown, duplicates dropped, times clamped. Replay tests now load the browser data file, drive real animation frames and assert named states |
| Low: several evidence and interface gaps | Overflowing numbers such as `1e999` passed the non-finite guard; the publisher could leave a mixed sample; the artifact checker trusted the harness's own pass flags; transport tests accepted any error; perception from an unverified model counted as fresh; nominal flights did not reject unexpected faults; ground altitude drift before arming could trip the geofence | Strict JSON parsing everywhere; atomic sample swap with a nearest-rank percentile; cross-checks of terminal mode, events, clearance, error and boot stages; specific transport errors; the mission node accepts only the verified model hash; nominal and camera flights require no unexpected HOLD, LAND or failsafe; altitude is re-zeroed until arming |
| Low: replay and accessibility details | Invalid PX4 estimates were drawn as real motion; canvases were blurry on high-DPI screens and unreadable on phones; the 11–12 m map rows were cut off; the focus ring and control borders were below 3:1 contrast; every diagram was captioned "Pipeline diagram" | Invalid estimates are grey and dashed; canvases follow the displayed size and pixel ratio; the map scale fits the grid; focus and borders meet 3:1 in both themes; captions follow the diagram type |

Also added: light and dark themes with a remembered toggle and theme-aware diagrams, a references page linking every upstream project's source and documentation, and links between the site and the repository.

| Pass 5 verification | Observed result |
|---|---|
| C++ Release build and CTest | 1/1 passed, including the LAND descent and degraded-link cases |
| Lightweight Python tests | 35/35 passed, including multi-seed and hidden-obstacle closed-loop runs; also under `python -O` |
| ROS boundary and runner tests | 19/19 passed |
| Fast scenario matrix / signed-artifact cases | 8/8 passed / 6/6 behaved as specified |
| PX4/Gazebo/ROS matrix on the final code | 4/4 passed with every named check, including the new in-flight camera and no-unexpected-fault checks; `inputs_unchanged: true`; all 35 recorded input hashes match the published files |
| Web checks | Diagram parsing, both replay suites, the theme test and link checks pass; light and dark pages were inspected in headless Edge |

Not changed, and recorded as remaining work: the ROS contract tests still need a ROS container job before CI can run them, DDS traffic is not isolated or authenticated, and the SITL battery input remains fixed.

<a id="pass-6"></a>
## Pass 6 — Watching the flights, and a fleet from a moving carrier

Review date: 25 September 2026. This pass added three ways to see the simulation (Gazebo video recorded during each flight, a live Gazebo window for demonstrations, and an interactive 3D replay) and a new scenario: three drones launching from and landing back on a moving carrier vehicle. Building the fleet exposed defects that a single drone at the origin could not. Most first appeared in failed or incorrect runs (the first recording run and fleet runs 1 to 3); the station's clock, the payload's first-pose handling and the recording timeout were found by reviewing the new code.

| Finding | Why it mattered | Fix and regression evidence |
|---|---|---|
| High: the adapters judged freshness on the wall clock while PX4 ran on simulation time | Recording video slows the simulation (real-time factor 0.64 with one camera, 0.41 with two). On the first fleet run, healthy telemetry aged past its wall-clock deadlines on the ground, and the supervisors escalated to LAND before take-off | Gazebo's `/clock` is bridged into ROS and every adapter runs with `use_sim_time`, matching the supervisor contract's single simulation clock; hardware still uses the monotonic clock. Evidence: every scenario in the published matrix passed while recording, at well below real time |
| High: every command was addressed to system 1 | In the fleet, the second vehicle ignored 142 arm and mode requests and never took off, because PX4 accepts only commands addressed to its own `MAV_SYS_ID` | Each adapter takes `--system-id` (PX4 instance + 1). Contract test `test_commands_are_addressed_to_this_vehicle` |
| Medium: the carrier restarted after its final stop | Landed vehicles keep reporting their last phase, `descend`, which satisfied the start condition again, so the carrier drove off with the fleet on board | A `parked` latch makes the stop final. Contract test drives the station through recovery and asserts exactly one start and one stop |
| Medium: the station timed launch spacing on the wall clock | At reduced real-time factor the 4 s gap shrank in simulated time | The station runs on simulation time. Contract test checks both the airborne condition and the gap with an injected clock |
| Medium: only PX4 instance 0 received a ground-station heartbeat | Instances 1 and 2 failed preflight with "No connection to the ground control station" | The heartbeat process sends to each instance's MAVLink port |
| Medium: `--video` silently produced no file | Gazebo's recorder encodes to a temporary file in the server's working directory and renames it on stop; the rename cannot cross from Linux storage to the Windows mount | Gazebo runs in the output directory; results list a video only when the file exists, and the fleet page test requires each published video |
| Medium, recorded and not changed: PX4 treats a moving deck as GNSS drift | After touchdown each drone's estimate stayed where it landed while the carrier carried it away (up to 4 m of error), and PX4 reported "GPS Horizontal Pos Drift too high". EKF2 runs its GNSS drift and speed checks only on the ground and at rest, and a drone on a deck at a steady 0.27 m/s is at rest to its IMU, so EKF2 skips the GNSS samples. The failing preflight check would also block re-arming while the carrier moves (observed as a preflight failure; re-arming was not attempted) | Firmware settings were left at their defaults, because loosening a GNSS quality gate is an aircraft-level decision. The acceptance checks already use only touchdown and airborne samples. The fleet replay now draws landed drones on their pads, with a regression assertion, and the SITL guide explains the behavior and the design options |
| Low: fleet-only integration faults | A camera named `deck` on a link with a `deck` visual crashed Gazebo; a station log field named `kind` collided with the logger's own argument; payload scans before the first pose were placed at the world origin, wrong for vehicles spawned elsewhere; the wall-clock timeout did not allow for slower recording runs | Unique sensor names; the field is `grant`; no scan until a pose arrives, with each vehicle's spawn offset applied; the timeout scales by 2.5 with `--video`. Contract tests cover landing order (one at a time, lowest holding layer first), lead-compensated pad tracking and the descent go-around |

Added in this pass: `simulation/worlds/fleet.sdf` (carrier with three pads, a deck camera and an overview camera), `integration/fleet_mission_node.py` and `integration/fleet_station_node.py`, per-vehicle namespaces throughout the adapters, the [fleet page](../web/fleet.html) with 3D and video views, the 3D and video views on the [flight replay](../web/sitl.html), and a fleet page state test in CI.

| Pass 6 verification | Observed result |
|---|---|
| C++ Release build and CTest | 1/1 passed |
| Lightweight Python tests | 35/35 passed; also under `python -O` |
| ROS boundary and runner tests | 25/25 passed, including five new fleet tests: launch spacing, carrier parking, landing order, pad lead and go-around |
| Fast scenario matrix / signed-artifact cases | 8/8 passed / 6/6 behaved as specified |
| PX4/Gazebo/ROS matrix with video on the final code | 5/5 passed: the four single-drone flights with every named check, then the fleet with 32/32 checks; `inputs_unchanged: true`; all 37 recorded source hashes match the published files; about 11 minutes of wall time at below real-time speed |
| Fleet measurements | Touchdown 3.0, 3.4 and 3.7 cm from the pads with the carrier at 0.27 m/s; closest approach between airborne drones 1.71 m; minimum estimated obstacle clearance 1.23 m; carrier travel 11.7 m with one start and one stop |
| Recorded video | Six recordings (four single-drone, fleet overview and deck camera), 23 MB in total at 600 kbps; frames from every recording were extracted and inspected |
| Web checks | Six Mermaid diagrams parse; the three replay state suites, the theme test and link checks pass on the repository and on the assembled Pages layout; the fleet page, its 3D view and the architecture diagram were inspected in headless Edge |

Not changed, and recorded as remaining work: the ROS contract tests still need a ROS container job before CI can run them, the moving-deck GNSS behavior above needs an aircraft-level design before any re-launch from a moving carrier, and the station has no lost-link or authentication model.

<a id="pass-7"></a>
## Pass 7 — Guardian drones: evaluation and a real-PX4 flight

Review date: 26 September 2026. The request was to let drones act as guardians against an attacking drone or a missile from far away, cooperating with the station, the command center and their own autonomy. [The guardian design](guardian.md) evaluates that idea, improves it twice (non-kinetic protection with authority split by time budget), and records the evaluation. This pass added `tools/guardian.py` (the decision logic), `tools/guardian_sim.py` (four designs against eight threats over 40 seeds), the guardian PX4 scenario and its nodes, and [the guardian page](../web/guardian.html). Every defect below was found in a failing run or test and traced before it was fixed.

| Finding | Why it mattered | Fix and regression evidence |
|---|---|---|
| High: a fixed-gain filter with a two-point velocity start | An 18 m/s intruder read as 10–60 m/s and its heading flipped, so RED came seconds before arrival instead of a minute | Constant-velocity Kalman filter weighted by each sensor's noise. `test_confirms_and_estimates_a_straight_target` requires the velocity within 4 m/s |
| High: friendly association by nearest reported position | A threat passing close to a guardian was absorbed as that guardian and dropped from tracking, and the mismatch raised a false spoofing alarm | Each friendly claims only its nearest return; the rest are tracked. Covered by the spoofing and combined scenarios over 40 seeds |
| High: keep-clear chattering | A guardian that moved far enough to clear the prediction was told to hold, stopped halfway and was back in conflict; in the swarm and combined scenarios guardians came within 20–40 m of intruders | An episode latch keeps the move until the conflict has passed and re-plans instead of holding. `test_episode_is_kept_and_replanned_but_never_dropped_to_hold` |
| Medium: straight-line prediction against a weaving intruder | At long range a 20° weave misses the station by hundreds of metres, so RED waited for the last few hundred metres | Inbound also means heading within 25° of the station. `test_weaving_drone_heading_roughly_inbound_counts` |
| Medium: false RED alerts from clutter and birds | Clutter lined up into fast tracks; a bird's noisy speed estimate hovering around the slow-track threshold kept the persistence timer running because the heading hysteresis also bridged the bird gate | A young track may only take plausibly reachable detections; RED needs support beyond confirmation; fast tracks need a steady speed estimate; the bird gate is judged on every update. `test_clutter_does_not_confirm_implausible_tracks`, `test_fast_track_is_danger_at_once_only_with_a_steady_speed`, `test_bird_whose_speed_estimate_hovers_at_the_threshold_never_raises_red`; 0 false alerts in 160 bird runs |
| Medium: lost-link return at ground level | A jammed guardian flew home at the station's height, through the intruders' altitude band | The rally point keeps cruise altitude |
| Medium: the station and center nodes crashed on their first report | A log field named `kind` collided with the logger's own argument, the same pitfall as the fleet station; found on the first PX4 guardian run | Fields renamed; `integration/test_guardian_nodes.py` now builds the real station, center and environment nodes and drives them without Gazebo, so this class of fault fails in seconds |
| Low: the authority check misjudged relayed recovery | Station orders relaying the center's `recover` were checked as station decisions | Orders carry the deciding authority; `test_recovery_waits_for_the_center_and_carries_its_authority` |
| Low: warning time and page data | The PX4 warning was measured in wall time (the simulation ran below real time); the replay payload was 3.5 MB; the evaluation's artifacts were ignored by git and not copied to Pages | Simulated seconds; a columnar replay format of 0.8 MB; `artifacts/guardian/` is tracked and published |

Recorded, not changed: in the combined scenario one hybrid run in 40 came within 70 m of an intruder (safe radius 75 m), when the overwatch above the station was caught between two intruders converging on it; the layout experiment shows that offsetting the overwatch removes that case. The fast inbound object cannot be escaped by a drone hovering over its impact point with under three seconds of warning; that is a limit of physics and layout, not of the decision logic.

| Pass 7 verification | Observed result |
|---|---|
| C++ Release build and CTest | 1/1 passed |
| Lightweight Python tests | 56/56 passed, including 21 for the guardian logic and its simulator; also under `python -O` |
| ROS boundary and runner tests | 32/32 passed, including seven that build the real guardian station, center and environment nodes |
| Fast scenario matrix / signed-artifact cases | 8/8 passed / 6/6 behaved as specified |
| Guardian evaluation | 4 designs × 8 scenarios × 40 seeds, plus 240 layout runs, on Linux with Python 3.10 in about 30 s. Every run keeps both invariants; 0 false alerts in 160 bird runs; the report's hashes match `tools/guardian.py` and `tools/guardian_sim.py` |
| PX4/Gazebo/ROS matrix with video on the final code | 6/6 passed; `inputs_unchanged: true`; all 46 recorded source hashes match the published files, including every guardian file |
| Guardian flight | 38/38 checks; 15.9 simulated seconds from RED to arrival; closest guardian 4.54 m from the intruder; carrier 8.0 m clear after relocating. `px4_0` lost its link at 63.1 s, held and then kept clear on its own, and regained its link at 77.1 s after its move took it out of the jamming zone; recovery followed the center's decision |
| Fleet flight | 32/32 checks; touchdown 7.0, 0.7 and 2.4 cm from the pads; closest approach 1.71 m |
| Recorded video | Eight recordings (four single-drone, fleet overview and deck, guardian overview and close-up); frames extracted and inspected |
| Web checks | Seven Mermaid diagrams parse; four replay state suites, the theme test and link checks pass on the repository and on the assembled Pages layout; the guardian page, its 3D view and the design document were inspected in headless Edge |

<a id="pass-8"></a>

## Pass 8 — Eight guardian flights, and three independent reviews

Review date: 26 September 2026. The request was to implement the guardian scenarios on PX4 and to review the logic and the code. The single PX4 guardian flight of pass 7 became eight: one per threat the fast simulator evaluates. `integration/guardian_layout.py` holds the scenario table; the runner writes each flight's Gazebo world from a template and judges it on Gazebo truth. Three reviewers then read the work independently: the decision module and fast simulator, the PX4 integration, and the published pages and publisher. They replayed their doubts in the simulator before reporting them, 35 findings in all. Every finding below was reproduced, then fixed with a test or a check, or recorded as a limit. Flying the scenarios found seven more; they are listed separately. [The guardian design](guardian.md#review) summarises what changed in the design.

| Decision module and fast simulator | Why it mattered | Fix and regression evidence |
|---|---|---|
| High: the integrity check compared stale or masked reports with any return near them, and an alarm never cleared | 30 of 280 non-spoofing hybrid runs switched a healthy guardian to station fixes of a bird or another guardian; navigation errors reached 2 km | The station keeps its own track of each guardian by continuity, gated by sensor accuracy, reported velocity, barometric height and sensor coverage; old reports are not compared; alarms clear after six consistent observations; fixes correct only horizontal position. `test_the_station_never_mistakes_a_bird_for_its_own_guardian`, `test_integrity_ignores_old_reports_and_clears_after_consistent_observations`; the report now counts false alarms and a healthy guardian's worst navigation error: 0 and 0 m in 1280 runs |
| Medium: the switch to station fixes was sent once | One lost message left a flagged guardian on GNSS, dragged 603 m | Every fix carries the switch; every order names the navigation source |
| Medium: `predicted_miss` sampled 24 points in 60 s | A 250 m/s object "missed" by 250 m a guardian it would hit | Exact closest approach per leg. `test_predicted_miss_is_exact_for_a_fast_object` |
| Medium: assessment extrapolated a relocating station's velocity forever | RED dropped about 2.5 s after relocation began, with the threat still inbound | Threats are judged against the station's stop point. `test_a_relocating_station_is_judged_where_it_will_stop` |
| Medium: the report no longer matched the decision code | The published numbers came from older code | Regenerated; the hash test guards it |
| Low–Medium: the invariants passed by construction; kept orders and paths were never checked; "X% pass" was the worst cell | The claim could not fail, and was worded as more than it measured | Moves must open the range along the whole straight path; kept orders are re-checked each cycle; the results page states the exact share and also measures the property on truth. `test_never_closes_on_any_threat_anywhere_along_the_move`, `test_a_kept_move_is_replanned_once_the_rest_of_it_would_close_on_a_track` |
| Low–Medium: late measurements were compared with the current state | A 0.4 s-late detection moved a fast track by 32 m | Compared with where the track was. `test_a_late_measurement_is_compared_with_where_the_track_was_then` |
| Low: class ties went to "bird" | A drone labelled 2:2 could never raise RED | Ties go to the more protective class. `test_a_tied_class_vote_goes_to_the_class_that_needs_more_protection` |
| Low: the lost-link rally point was the station's live position; delegation read the simulator's own flag | Knowledge the guardian and station could not have | The rally is the station's last-heard position; delegation follows the center's silence. Hybrid safety in the combined scenario, measured with the honest rally, is still 40/40 |
| Low: false REDs were forced to 0 whenever a real threat existed; the page credited camera labels for zero false REDs from birds | Spoofing and combined false alarms were hidden; the claim was wrong | Counted in every scenario (spoofing: 1–2 per design without the cross-check, 0 for hybrid; combined: 1 in 40 hybrid runs); the claim is corrected |
| Low: the station pruned tracks only when a detection arrived | Tracks up to 13 s old fed orders | Pruned every step |
| Low: dispersal edge cases | A guardian at the hazard centre was sent to the centre; the hazard followed the moving station | Dispersal points are checked like any move and centred on the predicted impact point. `test_dispersal_is_guarded_like_any_other_move` |
| Low: all sensors and links drew from one random stream; a test never reached its retry branch | Differences between designs were partly noise | One stream per sensor and per link message; `test_center_retries_a_decision_until_the_link_returns` |

| PX4 integration | Why it mattered | Fix and regression evidence |
|---|---|---|
| High: birds held the posture at AMBER; one bird flew faster than the slow-speed gate and circled where the carrier relocates | The quiet watch could never end; an unlabelled bird raised RED | Bird tracks do not hold AMBER; the PX4 birds were laid out and scaled like the fast simulator's. `test_birds_a_camera_has_labelled_leave_the_posture_green`, `test_posture_ignores_birds` |
| Medium–High: dispersal skipped every guard | The fast-object dispersal target was nearer the object and beside a cylinder | Guarded and centred on the impact point; `dispersed_before_impact` now also requires the guardian to be truly further from the impact point at impact. `test_a_fast_object_disperses_the_guardian_near_its_impact_point_safely` |
| Medium: a stale answer from the center was accepted | A recovery decided before a new event would recall guardians after it | Numbered requests; only the open one's answer counts, at GREEN. `test_a_decision_to_an_earlier_clear_is_ignored_after_a_new_event` |
| Medium: the station planned from stale guardian state | Orders computed for a jammed guardian's old position | Orders only to guardians heard within a second, a silent one told to hold, wider separation around it; guardians re-check station moves against their own tracks. `test_a_guardian_the_station_has_not_heard_from_is_told_to_hold_and_given_room`, `test_a_guardian_does_not_fly_a_station_move_that_closes_on_what_it_sees` |
| Medium: the spoofer's comment had the drag direction reversed, the drag headed for a cylinder, and it launched a process every 0.5 s | See the first flight finding below | Correct comment; south-west drag; an in-process client called only when the offset changes, with failures logged |
| Medium: a guardian flew on a frozen correction if fixes stopped | Silent drift after the alarm had latched | Lands in place after a 3 s fix timeout. `test_a_guardian_on_station_fixes_lands_in_place_when_they_stop` |
| Medium–Low: no reflex outside the watch; the lost-link return flew a straight line and never landed; the return fell back to a straight line from a blocked cell | Could cross a cylinder or another post | A planned route to the rally, holding there for clearance (`test_a_lost_link_return_follows_a_planned_route_to_the_last_known_carrier`); station moves respect the reserved cells. Recorded, not changed: the reflex runs only on watch, because launch and recovery happen at GREEN |
| Low–Medium: relocation always drove to the same stop | In the swarm the carrier drove toward the second intruder (predicted miss 3.1–3.7 m against 3 m) | The stop is chosen from the tracked threats. `test_relocation_moves_further_when_a_threat_comes_from_the_east` |
| Low: several checks were weaker than their names | `threat_confirmed` passed on clutter; one track could match two threats; `dispersed_before_impact` checked only an order; warning was measured to a fleeing carrier | Matched to truth, one-to-one, truth distance at impact, arrival at the threat's aim point, and `never_closed_on_threat` covers dispersal and the whole path. Recorded, not changed: `fleet_min_separation` counts airborne pairs only |

| Published pages and publisher | Why it mattered | Fix and regression evidence |
|---|---|---|
| Medium: eight flights in the shared report data | Several megabytes on the flight and fleet pages, which never use them | The guardian flights go to their own `guardian-data.js`, slimmed to what the page reads: 1.9 MB for eight flights. The full `result.json` stays with each flight |
| Medium: a flight in which nothing flew could pass | Birds' odometry missing would leave every check green | The publisher requires truth for every vehicle and threat from its start; a new `threats_flew` check |
| Medium: misleading event text | "threats start" with no threats; recovery listed twice; a second relocation read like the first; unknown events read "undefined link restored" | Text built from what each flight has; the station's acceptance shown as such; explicit cases with a safe fallback |
| Medium: deploy order | Web changes without the new artifacts would show no flights | Committed together |
| Low: video buttons, tiles, colours, 3D display, stale wording, empty state | Wrong notes or a stale video; a carrier-miss tile that looks like a failure on the fast flight; birds drawn as hostile and the jamming zone in the intruders' red; threats frozen outside their flight | Per-camera buttons; a dispersal tile; birds not hostile and a violet zone; threats shown only while they fly, facing their motion; wording updated; controls disabled with no data. Recorded, not changed: the event log runs on wall time while the tiles use simulated time |
| Low: the page test could pass vacuously | It tolerated empty threat traces and never reset its replay stub | Stronger assertions: every threat drawn, only birds non-hostile, per-camera buttons, no FAIL, no "undefined", no jamming text without a jammer, both recovery paths |

| Found while flying | What it did | Fix and regression evidence |
|---|---|---|
| The spoofer ran `gz service` twice a second | The load starved the simulation; the supervisors saw stale IMU data and landed two guardians | In-process gz-transport client (above) |
| A supervisor exited on an invalid sample with no reason logged | One guardian's adapter stopped mid-flight | The adapter logs the supervisor's own reason and never sends a sample that does not advance in time |
| A route passed a metre under another guardian holding its post | 0.99 m between guardians in the fast-object flight | Routes and moves keep out of the cells around the other posts. `test_each_guardian_routes_around_the_other_posts_but_can_reach_its_own` |
| Keep-clear from unlabelled birds | Guardians left their posts, out of camera range, so the birds stayed unlabelled and held AMBER until noise raised RED | A slow track no camera has classed as a drone or fast object gets only a collision margin, within the reflex horizon; a keep-clear, like RED, waits for support beyond confirmation. `guardians_held_their_posts` replaces a check that no keep-clear ever happens |
| The path rule made distant birds veto every move | An overwatch at its ceiling could not move while an intruder passed beneath | Only relevant tracks count. `test_relevant_tracks_leave_out_birds_and_tracks_that_stay_far_away` |
| A guardian cornered at the arena edge under its ceiling | 1.41 m from an intruder crossing above (safe radius 1.5 m) | Keep-clear may also descend, above a floor. `test_a_guardian_at_its_ceiling_descends_away_from_a_threat_crossing_above` |
| The station's friendly track slid onto a bird below or beside a guardian | A healthy guardian was flagged | The height and coverage gates above |

| Pass 8 verification | Observed result |
|---|---|
| C++ Release build and CTest | 1/1 passed |
| Lightweight Python tests | 70/70 passed, including 35 for the guardian logic and its simulator; also under `python -O`; on Windows Python 3.14 and Linux Python 3.10 |
| ROS boundary and runner tests | 46/46 passed, including 21 that build the real guardian station, center, environment and vehicle logic |
| Guardian evaluation | 4 designs × 8 scenarios × 40 seeds plus 240 layout runs on Linux (Python 3.10) in about a minute. Both invariants held in 1280 of 1280 runs; no healthy guardian was flagged as spoofed; hybrid kept every guardian clear in every run of five of the six threat scenarios (fast inbound: 10%, 90% with the offset overwatch); the report's hashes match the code |
| PX4/Gazebo/ROS matrix with video on the final code | 13/13 passed in about 33 minutes; `inputs_unchanged: true` |
| Guardian flights | 300/300 checks over eight flights; RED 12.2–12.6 simulated seconds before arrival for slow intruders and 1.8 s before a fast object's impact; closest guardian to a threat 2.63 m (combined) against a 1.5 m safe radius; spoofing flagged in 11 s with at most 1.19 m of drift; delegated recovery with no center link |
| Smoke flights before the matrix | Found the separation, starvation, supervisor, bird and cornering faults listed above; each scenario was re-flown after its fix |
| Web checks | Seven Mermaid diagrams parse; the four replay state suites (the guardian suite now drives all eight flights) and the theme test pass; the published guardian data is 1.8 MB, and the shared report data shrank from 1.4 MB to 0.7 MB |

<a id="pass-9"></a>

## Pass 9 — Phase boundaries, full-path clearance and trustworthy publication

Eight defects were reproduced against published commit `dac9f1b` even though its existing tests passed. The fixes and runnable reproduction steps are in [the review-fixes guide](review-fixes.md). Review proceeded through control-flow inspection, focused failure regressions and recorded simulation execution; these were sequential checks in this task, not claims of independent reviewers.

| Finding | Fix | Regression evidence |
|---|---|---|
| R1: expired station fixes and lost uplink were checked only on watch | Emergency decisions precede every armed phase; handoff stops the current tick before further offboard publication | Every airborne phase, transit/approach link loss, return-phase reflex and immediate-handoff tests |
| R2: a safe endpoint could have an unsafe path through another guardian | Whole-segment distance against friendly positions and uncertainty circles | Crossing/tangent/clear paths and stale-position regions |
| R3: no A* route became a direct corridor command | Failed return plan requests LAND and hands off | A complete obstacle barrier produces no direct transit target |
| R4: carrier pose and clearances never expired | One-second capture-time contracts; reject replay/future data; mask clearances for stale reports; stop descent on expiry or revocation | Separate stream failures, replay rejection and fresh clearance revocation |
| R5: disarm could count as landing | Fresh independent PX4 landed/contact evidence after airborne state, with source time consistent with current pose; independent runner checks | Missing/stale/replayed/delayed/preflight contact, disarm-only and valid landing cases |
| R6: incomplete entities could survive publication | Exact unique vehicle set and exact configured threat set | Full publisher rejection preserves the previous sample; identity/trace cases |
| R7: promotion failure removed the reference path | Restore old directory; recover interrupted promotion; retain installed sample on cleanup failure | Fault-injected promotion, retry, interruption and cleanup tests |
| R8: cached moves bypassed current free space | Revalidate retained and received avoidance moves | Newly invalid cached path and station target, plus still-valid cached path |

The first fleet smoke run exposed another touchdown weakness: at an estimated altitude of about 0.38 m, clamping the descent target to 0.3 m produced too little downward command and left one vehicle armed on the deck. The run was interrupted through its owned runner after recording the stall; cleanup completed and its logs were preserved under `retests/fixes-20260926-smoke`. The target now continues downward to the supervisor's nonnegative target floor. A regression covers that estimate, and the second smoke run passed all 35 fleet checks. Landing revocation and delayed landed-packet capture were also checked during implementation.

The first full matrix then passed all four single-drone flights and the fleet, but failed `guardian_intruder` when rejecting a blocked station target with no local tracks produced an infinite clearance metric. Strict JSON logging rejected the order and the mission process exited. `keep_clear_order` now omits that unavailable metric; both cached-target and station-target regressions require strict JSON serialization with no tracks. The failed matrix remains in `retests/fixes-20260926-final`; no evidence from it is published as a complete passing run.

The next matrix, preserved in `retests/fixes-20260926-final2`, passed the repaired intruder flight but failed the fast scenario's dispersal check despite safe recovery and landing. The station's continuous geometry admitted a route that the drone's conservative occupancy grid rejected. Guardians now report their blocked cells, and station orders must satisfy both the station geometry and that grid. A regression inserts a reported blocked cell into the previous route and requires a clear replacement. The onboard acceptance gate remains active.

A tested `--keep-going` runner option now records all scenarios while preserving failed checks and a nonzero exit. The remaining fast-inbound timing limitation is recorded in validation; sharing obstacle maps did not eliminate it.

The portable suite has 78 tests and the ROS/runner suite has 59. The 40-seed guardian evaluation was regenerated with matching decision-source hashes; eight fast policy scenarios and six security cases passed. Current flight results and failed checks are recorded in [validation](validation.md); [SITL execution](sitl-execution.md) describes the earlier passing reference replay.

## Diagrams and how to read them

| Document | Diagram | What it explains |
|---|---|---|
| [Architecture](architecture.md) | Fast-harness data-flow chart | Sensor, estimator, planner, supervisor and evidence paths; bounding boxes are observations rather than steering commands |
| [Guardian design](guardian.md) | Authority data-flow chart | Center, station and guardians: what each layer sends and decides |
| [Architecture](architecture.md) | Fleet data-flow chart | Station, carrier and three per-vehicle stacks; what flows to and from the station |
| [Architecture](architecture.md) | C++ state graph | INIT, ACTIVE, HOLD, LAND and COMPLETE; the ROS handoff boundary is explained alongside it |
| [SITL runbook](sitl-guide.md) | Mermaid sequence diagram | Process startup, typed data, supervision, terminal handoff, crash fallback and independent flight observations |
| [Complete reproduction guide](complete-reproduction-guide.md#pipeline) | Dependency graph | WSL, compiler, Python, ROS, PX4, Gazebo, interfaces, tests and generated documentation |
| [Complete reproduction guide](complete-reproduction-guide.md#pipeline) | Flight-integration data-flow chart | Actual PX4 flight sensing/control versus procedural payload sensors and recorded evidence |
| [Review fixes](review-fixes.md) | Sequence diagram, emergency flowchart and validation graph | Stream freshness and landing sequencing, priority of emergency decisions, and the path from regression tests to Pages deployment |

Markdown viewers with Mermaid support render the fenced source. Included HTML uses `docs/assets/mermaid-11.17.2.min.js` and `diagrams.js` locally, so viewing the cloned guides needs no CDN or npm installation. Expanding “Mermaid source” reveals the exact definition. Only regenerating HTML and running syntax checks requires `npm ci`.

The integration follows the official [Mermaid usage documentation](https://mermaid.js.org/config/usage.html): initialization is explicit, strict security mode is enabled, and rendering uses `mermaid.run`. No link callbacks are enabled in diagrams.

## Reproduce the reviewed checks

From Ubuntu WSL, after following the prepared-environment steps in the complete guide:

```bash
cd "$DRONE_REPO"   # your checkout
bash scripts/run_all.sh
bash scripts/test_integration.sh
.venv/bin/python -O -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: one CTest executable passes; 78 lightweight Python tests pass; eight fast scenarios pass; six security experiments behave as specified; 59 integration tests pass. Running the Python cases with `-O` also verifies evidence gates do not rely on removable `assert` statements. These commands do not start a physical aircraft.

For the actual flight matrix, use a fresh Linux output directory and retain it for later inspection:

```bash
export SITL_WORKSPACE="$HOME/work/mission-computer-lab"
REVIEW_RUN="$SITL_WORKSPACE/retests/review-$(date +%Y%m%d-%H%M%S)"
bash scripts/run_sitl.sh --all --video --output "$REVIEW_RUN"
echo "Flight runner exit code: $?"
```

Expect thirteen PASS lines (four single-drone scenarios, the fleet, then the eight guardian flights) and exit code zero. Do not publish if the runner failed. `results.json` contains detailed named checks; `provenance.json` must have `inputs_unchanged: true`. If a run fails, inspect its scenario `result.json`, `runner-error.log` if present, application logs and independent `observer.jsonl` before making a new attempt. Process cleanup still runs on a caught failure. Keep failed runs for comparison.

After a successful full matrix:

```bash
.venv/bin/python tools/publish_sample.py
"$SITL_WORKSPACE/venv/bin/python" tools/publish_sitl.py --input "$REVIEW_RUN"
```

In a shell with Node.js 20 or newer, from the repository root:

```bash
npm ci
npm run docs
node tests/test_diagrams.mjs
node tests/test_dashboard.mjs
node tests/test_sitl_dashboard.mjs
node tests/test_fleet_dashboard.mjs
node tests/test_guardian_dashboard.mjs
node tests/test_theme.mjs
npm audit
```

Back in Ubuntu WSL:

```bash
.venv/bin/python tools/check_docs.py
git diff --check
```

Expect every Mermaid definition to parse, offline bundle checks to pass, all four replay state suites and the theme test to pass, and all local HTML links and anchors to resolve. The audit result can change as new advisories are published.

## Recorded verification and remaining boundaries

| Passes 1–3 verification (23 September) | Observed result |
|---|---|
| C++ Release build and CTest | Build succeeded; 1/1 test executable passed |
| Lightweight Python tests | 23/23 passed; also passed under `python -O` |
| ROS boundary and runner cleanup tests | 12/12 passed |
| Fast scenario matrix | 8/8 passed |
| Signed-artifact policy cases | 6/6 behaved as expected |
| Final PX4/Gazebo/ROS matrix | 4/4 passed with ordered flight, validity and bag checks |
| Runtime provenance | `inputs_unchanged: true`; final runner exit code 0 |
| Mermaid syntax and offline bundle | 5/5 diagram definitions parsed; all requested types present |
| Replays | Both state suites passed; rendering/layout not claimed |
| npm dependency audit | 0 reported advisories after the pinned upgrade |

The final local results and measured flight values are linked in [the validation record](validation.md), [fast execution report](execution.md) and [PX4 execution report](sitl-execution.md). The intermediate pass-3 Linux run passed three flights but failed the overly broad GPS validity check; its result was preserved unchanged, as were the first failed attempt and the final pass-3 run, locally with their bags and logs. Compact current evidence, from the pass 4 run, is published under `artifacts/sample/` and `artifacts/sitl-sample/`.

The review does not turn the procedural camera into a trained detector or the WSL host into a Qualcomm board. The executed boundary remains CPU ONNX, procedural payload sensors, A* with scan-driven replanning of static obstacles, simulated dynamics and actual PX4 firmware. Battery input in the flight adapter is still fixed, timing synchronization is single-host, and no VIO, NPU benchmark, secure boot fuse operation, dynamic replanning or flight certification is claimed.

Land commands are one-shot; the adapter stops offboard proof-of-life after handoff and relies on the configured PX4 fallback if needed. The suite does not currently inject a dropped land-command packet. Passes 1–3 did not complete browser layout inspection because the embedded browser's local-file policy blocked access. Pass 9 verifies diagram parsing, static packaging/links and replay state separately; it makes no new pixel-level verification claim. Hosted CI and Pages now run on GitHub; full ROS/PX4 verification remains local in WSL.
