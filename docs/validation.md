# Validation record

## Fourth review pass retest — 24 September 2026

**Result: all four flight scenarios passed on the final code, and the run-time input hashes remained unchanged.** The published samples come from this run. Current checks: one CTest executable, 28 lightweight Python tests (also under `python -O`), 14 ROS/runner regression tests, eight fast scenarios and six signed-artifact cases. Findings, fixes and verification are in [pass 4 of the review record](review-report.md#pass-4).

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
