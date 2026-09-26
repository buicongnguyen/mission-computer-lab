# Validation record

## Ninth review pass retest — 27 September 2026

**Current result: 12/13 PX4/Gazebo scenarios passed on the fixed runtime inputs.** All thirteen ran with `--all --keep-going --video`; failed checks remain failures and cause a nonzero exit. The fleet passed 35/35 checks; guardians passed 323/324. Every fleet/guardian vehicle now needs independent PX4 landed/contact evidence as well as disarm and pad proximity.

The publisher correctly refuses this matrix. The replay sample remains the earlier passing recording from the previous revision; notices on each flight page identify that distinction. It is not proof that the current source passes all flight checks.

| Scenario | Result | Passed checks | Failed checks |
|---|---|---|---|
| `nominal` | PASS | 22/22 | — |
| `camera_dropout` | PASS | 25/25 | — |
| `companion_crash` | PASS | 20/20 | — |
| `gps_loss` | PASS | 21/21 | — |
| `fleet_carrier` | PASS | 35/35 | — |
| `guardian_intruder` | PASS | 41/41 | — |
| `guardian_fast` | FAIL | 39/40 | `dispersed_before_impact` |
| `guardian_swarm` | PASS | 41/41 | — |
| `guardian_birds` | PASS | 38/38 | — |
| `guardian_jamming` | PASS | 42/42 | — |
| `guardian_spoofing` | PASS | 40/40 | — |
| `guardian_center_loss` | PASS | 40/40 | — |
| `guardian_combined` | PASS | 42/42 | — |

All 50 runtime input hashes match the current source/binary; `inputs_unchanged` is true. The recorded `source_commit` is the base commit before these tested changes were committed; per-file hashes identify the tested implementation. [Compact results and hashes](review-validation.json) preserve each check, including failures. Raw records remain at `/home/n/work/mission-computer-lab/retests/fixes-20260927-complete`.

The 78 portable Python tests passed normally and under `python -O`; all 59 ROS/runner tests passed; CTest passed 1/1; the accelerated matrix passed 8/8 scenarios and 6/6 security cases. The 40-seed guardian evaluation was regenerated with matching decision-source hashes. Browser state, Mermaid and local-link checks are performed before committing the rendered pages; no new pixel-level browser verification is claimed.

Earlier failed runs are preserved: `retests/fixes-20260926-smoke` exposed insufficient deck descent; `smoke2` passed all 35 fleet checks after that fix. `fixes-20260926-final` exposed a nonfinite logging metric; `fixes-20260926-final2` and `fixes-20260926-fast-smoke` exposed the effect of stricter obstacle validation on fast-inbound timing. The latter completed safe recovery/landing but increased distance from the impact point by only 0.38 m before arrival, below the 0.50 m requirement. Sharing the vehicle map fixes planning disagreement, but does not establish reliable timing performance for this tightly constrained scenario. No acceptance threshold was lowered.

See [the fix-by-fix guide](review-fixes.md) and [review pass 9](review-report.md#pass-9). The dated sections below record earlier revisions and samples.

## Eighth review pass retest — 26 September 2026

**Result: all thirteen scenarios passed on the final code while recording video, and every recorded input hash matches the published source.** The matrix flies the four single-drone scenarios, the fleet, and eight guardian flights, one per threat the fast simulator evaluates; the published samples and videos come from this run, which took about 33 minutes. Guardian flights: 300/300 checks, judged on Gazebo truth; RED 12.2–12.6 simulated seconds before a slow intruder reached the carrier's parking spot and 1.8 s before a fast object's impact; the closest guardian stayed 2.63 m from a threat (combined scenario; safe radius 1.5 m); the jammed guardian kept clear on its own; the station caught a GNSS drag-off on all three receivers in 11 s and held them within 1.19 m of their posts; with no center link, recovery ran under delegation. The guardian evaluation was regenerated on Linux (Python 3.10): both invariants held in all 1280 runs and no healthy guardian was flagged as spoofed. Current checks: one CTest executable, 70 lightweight Python tests (also under `python -O`), 46 ROS/runner regression tests, eight fast scenarios and six signed-artifact cases, plus four replay state tests, theme, diagram and link checks. Three independent reviews (decision logic and simulator, PX4 integration, published pages) and the flights themselves produced the findings in [pass 8 of the review record](review-report.md#pass-8).

## Seventh review pass retest — 26 September 2026

**Result: all six scenarios passed on the final code while recording video, and every recorded input hash matches the published source.** The matrix adds three guardians against an intruder to the four single-drone flights and the fleet; the published samples and videos come from this run. Guardian flight: 38/38 checks; 15.9 simulated seconds of warning; the closest guardian stayed 4.54 m from the intruder and the relocated carrier 8.0 m; the jammed guardian kept clear on its own and regained its link after leaving the jamming zone; recovery followed the center's decision. The guardian evaluation ran 4 designs × 8 threat scenarios × 40 seeds plus the layout experiment on Linux (Python 3.10); its report records the hash of the decision code it ran. Current checks: one CTest executable, 56 lightweight Python tests (also under `python -O`), 32 ROS/runner regression tests, eight fast scenarios and six signed-artifact cases, plus four replay state tests, theme, diagram and link checks. The guardian page, its 3D view and both guardian videos were inspected in headless Edge and from extracted video frames. Findings, fixes and verification are in [pass 7 of the review record](review-report.md#pass-7).

## Sixth review pass retest — 25 September 2026

**Result: all five scenarios passed on the final code while recording video, and every recorded input hash matches the published source.** The four single-drone flights and the three-drone fleet ran as one matrix with `--video`, at below real time; its samples were later replaced by the pass 7 run. Fleet: 32/32 checks; the three drones touched down 3.0, 3.4 and 3.7 cm from their pads while the carrier drove at 0.27 m/s; the closest approach between airborne drones was 1.71 m; the carrier travelled 11.7 m and stopped once. Current checks: one CTest executable, 35 lightweight Python tests (also under `python -O`), 25 ROS/runner regression tests, eight fast scenarios and six signed-artifact cases, plus three replay state tests, theme, diagram and link checks. The fleet page, both videos and the 3D views were inspected in headless Edge and from extracted video frames. Findings, fixes and verification are in [pass 6 of the review record](review-report.md#pass-6).

## Fifth review pass retest — 24 September 2026

**Result: all four flight scenarios passed on the final code, and every recorded input hash matches the published source.** Its samples were later replaced by the pass 6 run. Current checks: one CTest executable, 35 lightweight Python tests (also under `python -O`), 19 ROS/runner regression tests, eight fast scenarios and six signed-artifact cases, plus replay, theme, diagram and link checks. Findings, fixes and verification are in [pass 5 of the review record](review-report.md#pass-5).

## Fourth review pass retest — 24 September 2026

**Result: all four flight scenarios passed on the final code, and the run-time input hashes remained unchanged.** Its samples were later replaced by the pass 5 run. Checks at that time: one CTest executable, 28 lightweight Python tests (also under `python -O`), 14 ROS/runner regression tests, eight fast scenarios and six signed-artifact cases. Findings, fixes and verification are in [pass 4 of the review record](review-report.md#pass-4).

## Three-pass review retest — 23 September 2026

**Result: all four reviewed flight scenarios passed, and the run-time input hashes remained unchanged.** That matrix was the published sample until pass 4. Both replay state tests, all five diagram syntax checks and static checks across 16 local HTML files passed.

See [the detailed review record](review-report.md) for findings, fixes and repeatable commands. Checks at that time: 23 lightweight Python tests, 12 ROS/runner regression tests, one CTest executable, eight fast scenarios and six signed-artifact cases. The review also adds five Mermaid syntax/offline-bundle checks and explicit HTML fragment validation. The final four-case flight matrix uses the stricter ordered-flight and recorded-topic acceptance gates; measurements appear in [the current flight report](sitl-execution.md).

The first pass-3 retest stopped on an OS `No data available` error while recording live evidence on the Windows mount. Its results remain failed. The final retry recorded to WSL's Linux filesystem. The precise original failing syscall was not captured; future runner exceptions now retain a traceback. Failed results were preserved; the GPS-specific validity criterion was then corrected as described below.

The first Linux matrix passed nominal, camera and companion-crash scenarios. GPS-loss landed and disarmed but failed a new check that incorrectly required horizontal validity after intentionally removing GPS. The checker now distinguishes horizontal and vertical validity, still rejects invalidity before injection, and requires valid vertical position at touchdown. The original failed results were preserved, and the full matrix was rerun.

Mermaid is pinned at 11.17.2 with a committed lockfile and a local browser bundle. The dependency audit reported zero known advisories at review time. Parser/static checks do not establish visual layout, which remains unverified in the embedded browser.

## Initial implementation record — 22 September 2026

Executed locally on 22 September 2026 in the existing **Ubuntu 22.04.5 WSL2** distribution. This records the checks performed during initial creation; it does not imply hosted GitHub CI or hardware validation.

| Check | Observed result |
|---|---|
| C++17 Release build, GCC 11.4 | Passed; core compiled with Wall/Wextra/Wpedantic/Werror |
| CTest supervisor contract suite | 1/1 test executable passed; multiple contract assertions |
| Python unit/integration suite | 14/14 passed |
| Full fault matrix | 8/8 scenarios passed |
| Signed-artifact experiment matrix | 6/6 cases behaved as expected |
| Generated evidence schema/completeness check | Passed |
| C++ Debug build with address/undefined-behavior sanitizers | Built and CTest passed |
| ONNX execution provider | CPUExecutionProvider explicitly selected and recorded |
| PX4 v1.16.0 + ROS Humble + Gazebo Harmonic | Built and executed in WSL; four flight scenarios passed |
| ROS adapter regression checks | 5/5 passed: invalid GPS, replay freshness, topic versions/QoS and fixed-array JSON |
| Qualcomm hardware / accelerator / hardware boot chain | Not executed |
| GitHub-hosted workflow | Supplied, not run remotely |
| Visual HTML review in the embedded browser | Blocked by the browser's local-file URL security policy; not completed |
| Dashboard state test with a minimal DOM | Passed for eight selections, timeline, embedded image data and play/pause; does not verify rendering |
| PX4 replay state test with a minimal DOM | Passed all four scenarios, acceptance checks, transitions, final disarm and play/pause; does not verify rendering |
| Static HTML checks | Passed local links, viewport metadata and dashboard element IDs |

The actual flight integration also found and corrected DDS discovery settings, startup parameter overrides, status-rate/freshness mismatch, invalid GPS packets incorrectly appearing fresh, and numpy scalar serialization in the observer. The [SITL evidence](sitl-execution.md) records the final passing runs.

The initial validation found and corrected two input/state edge cases: negative sequence numbers are rejected at the process boundary, and a fault during startup cannot shorten the full one-second preflight health dwell. Regression checks cover both.

The build produced millisecond-scale Windows-mounted-filesystem clock-skew warnings. Compilation and both CTest runs completed successfully. A Linux-filesystem checkout is preferable for larger builds and avoids relying on Windows mount timestamp behavior.

Exact reproduction commands and dependency pins are in [the runbook](wsl-guide.md). Per-scenario measurements and environment/model/binary hashes are in [the execution report](execution.md) and `artifacts/sample/report.json`. No test-coverage percentage, NPU speedup, flight-safety claim or model-accuracy claim is inferred from these checks.
