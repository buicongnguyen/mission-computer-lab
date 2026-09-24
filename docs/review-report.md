# Logic and code review record

Review dates: 23 September 2026 (passes 1–3), 24 September 2026 (passes 4 and 5) and 25 September 2026 (pass 6). Scope: the WSL mission-computer lab, including C++ policy contracts, Python adapters and transport, flight orchestration, evidence publication, replays and guides. Six sequential passes were performed over the same implementation; later passes also reviewed earlier fixes. This is a local engineering review, not a flight qualification or a claim that all possible defects have been eliminated.

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

## Diagrams and how to read them

| Document | Diagram | What it explains |
|---|---|---|
| [Architecture](architecture.md) | Fast-harness data-flow chart | Sensor, estimator, planner, supervisor and evidence paths; bounding boxes are observations rather than steering commands |
| [Architecture](architecture.md) | Fleet data-flow chart | Station, carrier and three per-vehicle stacks; what flows to and from the station |
| [Architecture](architecture.md) | C++ state graph | INIT, ACTIVE, HOLD, LAND and COMPLETE; the ROS handoff boundary is explained alongside it |
| [SITL runbook](sitl-guide.md) | Mermaid sequence diagram | Process startup, typed data, supervision, terminal handoff, crash fallback and independent flight observations |
| [Complete reproduction guide](complete-reproduction-guide.md#pipeline) | Dependency graph | WSL, compiler, Python, ROS, PX4, Gazebo, interfaces, tests and generated documentation |
| [Complete reproduction guide](complete-reproduction-guide.md#pipeline) | Flight-integration data-flow chart | Actual PX4 flight sensing/control versus procedural payload sensors and recorded evidence |

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

Expected: one CTest executable passes; 35 lightweight Python tests pass; eight fast scenarios pass; six security experiments behave as specified; 25 integration tests pass. Running the Python cases with `-O` also verifies evidence gates do not rely on removable `assert` statements. These commands do not start a physical aircraft.

For the actual flight matrix, use a fresh Linux output directory and retain it for later inspection:

```bash
export SITL_WORKSPACE="$HOME/work/mission-computer-lab"
REVIEW_RUN="$SITL_WORKSPACE/retests/review-$(date +%Y%m%d-%H%M%S)"
bash scripts/run_sitl.sh --all --video --output "$REVIEW_RUN"
echo "Flight runner exit code: $?"
```

Expect five PASS lines (four single-drone scenarios, then the fleet) and exit code zero. Do not publish if the runner failed. `results.json` contains detailed named checks; `provenance.json` must have `inputs_unchanged: true`. If a run fails, inspect its scenario `result.json`, `runner-error.log` if present, application logs and independent `observer.jsonl` before making a new attempt. Process cleanup still runs on a caught failure. Keep failed runs for comparison.

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
node tests/test_theme.mjs
npm audit
```

Back in Ubuntu WSL:

```bash
.venv/bin/python tools/check_docs.py
git diff --check
```

Expect every Mermaid definition to parse, offline bundle checks to pass, all three replay state suites and the theme test to pass, and all local HTML links and anchors to resolve. The audit result can change as new advisories are published.

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

Land commands are one-shot; the adapter stops offboard proof-of-life after handoff and relies on the configured PX4 fallback if needed. The suite does not currently inject a dropped land-command packet. Browser layout inspection was not completed because the embedded browser's local-file security policy blocked access. Diagram parsing, static packaging/links and replay state tests are verified separately; they are not pixel-level visual verification. GitHub-hosted CI has not run because this work remains local.
