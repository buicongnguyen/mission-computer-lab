# Engineering findings

What building and flying this lab exposed, layer by layer: the mission computer's own policy, its boundary with PX4, several vehicles at once, estimation and planning, the guardians' decision rules, and the evidence itself. Each finding was reproduced before it was fixed and is now guarded by a test or a flight check. [The review record](review-report.md) lists every finding, pass by pass.

## The supervisor and its authority

### 1. Intermittent faults never escalated

- **Seen:** a 60 s probe of the real supervisor binary, with link packets every 0.6 s, alternated between 11-tick HOLDs and single-tick ACTIVE commands for the whole minute. It never landed, and the vehicle crept forward on a failing link.
- **Cause:** the fault timer reset on the first healthy sample.
- **Fix:** a fault episode ends only after 2 s of continuous health; any unhealthy sample 2 s or more into an episode latches LAND.
- **Guard:** a C++ degraded-link test that fails against the previous implementation, and a second test showing that a single recovered dropout still clears.

### 2. The boot gate could not fail

- **Seen:** arbitrary bytes came back `VERIFIED IN USER SPACE`.
- **Cause:** the gate generated a key, signed the bytes and verified them in one call.
- **Fix:** release and boot are separate. Release writes a detached manifest and signature. At boot the stored file is read once and verified against a separately supplied public key (hash, version floor, device identity), and exactly those bytes are loaded into ONNX Runtime, so nothing can change between the check and the load.
- **Guard:** tests for a verified load, a tampered file, a different trust anchor and a missing signature. In a live ROS run a tampered model stopped the perception node (`boot_rejected: payload_tampered`, exit code 3).

### 3. The adapter would have overridden a pilot

- **Seen:** the handoff to PX4 covered failsafe, disarm and estimator loss, but not an operator leaving OFFBOARD. Within 2 s the adapter would have switched the vehicle back.
- **Fix:** once armed OFFBOARD has been seen, any other navigation state while armed is a permanent `external_mode_change` handoff.
- **Guard:** adapter regression tests. Every recorded flight was checked against the new rule first; no outcome changed.

## Time, identity and several vehicles

### 4. Freshness was judged on the wrong clock

- **Seen:** on the first fleet run, all three supervisors escalated to LAND on the ground, before take-off.
- **Cause:** the adapters judged sensor freshness on the wall clock while PX4 ran on simulation time. Recording video slows the simulation (real-time factor 0.64 with one camera, 0.41 with two), so healthy telemetry looked stale.
- **Fix:** Gazebo's `/clock` is bridged into ROS and every adapter runs with `use_sim_time`, matching the supervisor's single simulation clock. Hardware keeps the monotonic clock.
- **Guard:** every published flight is recorded on video, well below real time.

### 5. Every command went to system 1

- **Seen:** in the fleet, the second vehicle ignored 142 arm and mode requests and never took off.
- **Cause:** PX4 accepts only commands addressed to its own `MAV_SYS_ID`.
- **Fix:** each adapter takes its vehicle's system ID.
- **Guard:** `test_commands_are_addressed_to_this_vehicle`.

### 6. PX4 reads a moving deck as GNSS drift (recorded, not changed)

- **Seen:** after touchdown on the driving carrier, each drone's estimate stayed where it had landed while the deck carried it up to 4 m away, and PX4 reported "GPS Horizontal Pos Drift too high".
- **Cause:** EKF2 runs its GNSS drift checks on the ground at rest, and a drone on a deck moving at a steady 0.27 m/s is at rest to its IMU.
- **Decision:** the firmware gates keep their defaults. Loosening a GNSS quality gate is an aircraft-level decision, and the same check would block re-arming on a moving deck. The flight checks use airborne and touchdown samples, the replay draws landed drones on their pads, and [the SITL guide](sitl-guide.md) sets out the design options.

### 7. A descent through a lower layer

- **Seen:** in a fleet flight two drones passed 0.19 m apart, and their traces jump as if they touched.
- **Cause:** the station cleared the lowest of the vehicles already holding over their pads. px4_1 (4 m layer) arrived first and was cleared, while px4_0 (3 m) was still flying home along the carrier, over px4_1's pad, and px4_1 descended through 3 m into its path. Earlier runs had passed only because the lowest layer happened to arrive first. For the guardians the order was not even by altitude: their state carried the deciding layer ("station", "onboard") under the key the station reads as the altitude layer.
- **Fix:** a landing is cleared only for the lowest vehicle still armed and airborne, once it holds over its pad; higher vehicles wait. The guardian's deciding layer has its own key, and the station drops a state without an altitude layer. Waiting over its pad, a higher drone then sat exactly one layer above the lane a lower one flew in along, and the next flight measured 0.97 m against the 1 m minimum, so the fleet's layers are now 1.5 m apart (3, 4.5 and 6 m), and the guardians' too after a spoofing flight measured 1.04 m. A cleared vehicle whose touchdown is never confirmed is released after 10 s, so it strands nobody above it.
- **Guard:** `test_one_landing_at_a_time_lowest_airborne_layer_first`, `test_a_vehicle_down_away_from_the_carrier_holds_no_landing_up`, `test_a_cleared_landing_without_a_touchdown_stops_holding_the_others` and `test_a_guardian_state_keeps_its_altitude_layer_for_the_landing_order`, and the fleet page test checks the published landing order.

## Estimation and planning

### 8. LAND stopped descending at an estimated zero altitude

- **Seen:** after GNSS loss the dead-reckoned altitude drifted low, and 8 of 20 seeds left the fast-harness vehicle hovering 0.1–0.4 m above the ground until timeout. The published pass rested on one seed.
- **Fix:** LAND descends at 0.3–0.7 m/s until touchdown is detected, not until the estimate reads zero.
- **Guard:** a C++ test for estimates at and below zero, and a multi-seed test over every landing scenario.

### 9. A single-scan map planned through a hidden obstacle

- **Seen:** from the start point one cylinder hides most of another. Six cells inside real obstacles were marked free, and goals such as (10, 6) planned straight through them (clearance −1.1 m). The shipped goal was safe only because of tie-breaking.
- **Fix:** every scan is added to the map where the vehicle was when it was taken, and the route is replanned when it closes. With no route left the mission lands.
- **Guard:** all 28 goals whose first plan was unsafe now complete with at least 1.0 m of clearance.

### 10. A simulated LiDAR that believed a spoofed GNSS

- **Seen:** in every spoofing flight one guardian logged 14 to 16 stale-vision holds, and twice latched LAND. Earlier flights passed only because the LAND came after the descent had begun; one smoke flight latched it while waiting over the pad and failed.
- **Cause:** the procedural LiDAR was computed from the PX4 estimate, which the GNSS drag-off carries metres from where the guardian really holds its post on station fixes. For that guardian the estimate grazed a cylinder, ranges fell below the minimum, and the adapter rejected the scans.
- **Fix:** a real LiDAR measures the real surroundings whatever the GNSS says, so in the guardian flights the scan is computed from the airframe's true pose, which Gazebo already provides.
- **Guard:** `test_the_payload_lidar_measures_the_true_surroundings`; the next spoofing flight logged no stale vision at all.

## Guardian decision rules

### 11. A fixed-gain filter misread an intruder's speed

- **Seen:** an 18 m/s intruder read as anything from 10 to 60 m/s, with a flipping heading, so the alert came seconds before arrival instead of a minute.
- **Fix:** a constant-velocity Kalman filter that weights each sensor by its noise.
- **Guard:** `test_confirms_and_estimates_a_straight_target` bounds the velocity error.

### 12. Two safety rules that cancelled each other out

- **Seen:** on PX4, a guardian at its ceiling could not move while an intruder passed beneath it. The rule "never close on any track" counted distant birds, and they vetoed every escape direction. Another guardian, cornered at the arena edge, came within 1.41 m of an intruder crossing above it (safe radius 1.5 m).
- **Fix:** a move is judged only against tracks that could matter: not birds, and not tracks predicted to stay far away. Keep-clear may also descend, down to a floor.
- **Guard:** `test_relevant_tracks_leave_out_birds_and_tracks_that_stay_far_away` and `test_a_guardian_at_its_ceiling_descends_away_from_a_threat_crossing_above`.

### 13. Three rules kept a guardian in the impact area

- **Seen:** in the fast-object PX4 flight, the guardian nearest the impact point gained only 0.38 m of distance from it before impact, short of the 0.5 m the check requires.
- **Cause:** three rules worked against leaving. Every move had to open the range to each track's current position, and the object dived in from the north-west, so most ways out of the impact area were moves toward where it was at that instant. Keeping clear came before dispersing, and the best keep-clear move, which maximises the miss from the object's path, ran along the edge of the area. And the impact estimate of a young fast track moved by metres between updates: a dispersal re-planned every cycle flipped direction, and the guardian, refusing one target, fell back to a keep-clear move toward the impact point.
- **Fix:** a steady fast object is judged on its predicted path: a move may not shorten its predicted miss. Inside the area, a dispersal that is itself a keep-clear move comes first. A dispersal under way is kept while it still leads out, and the order carries the area, so a guardian that cannot fly the station's move re-plans its own way out.
- **Guard:** a unit test for each rule, built on the flight's own geometry, and the flight check `never_closed_on_threat`, which re-judges every logged move against the fast paths it was given. On the next flight the guardian was 5.0 m from the impact point at impact, 1.4 m further out than its post, and the full matrix then passed.

### 14. An unlabelled bird held the guardians out

- **Seen:** in two combined flights the guardians were never recalled, and the flight timed out.
- **Cause:** after the intruders passed, the guardians were held where keep-clear had left them, too far for their cameras to label a circling bird. Re-acquired as a new, unclassified track every lap of its circle, the bird brought the posture back to AMBER within seconds of each GREEN. The station asked the center, and started its delegation timer, only at GREEN, and any AMBER reset both.
- **Fix:** the question to the center starts when RED clears, and only a new RED resets it. Recall orders still wait for a GREEN moment, so an unidentified object in view keeps the guardians out only while it is actually there. The fast simulator follows the same rule.
- **Guard:** `test_an_amber_flicker_after_the_event_does_not_hold_recovery_back`; the next combined flight recovered under delegation.

## The evidence itself

### 15. Checks that were weaker than their names

- **Seen:** `threat_confirmed` passed on clutter; one track could match two threats; `dispersed_before_impact` only checked that an order was sent; the "never closes on a threat" invariant passed by construction; a disarm could count as a landing.
- **Fix:** threats are matched to Gazebo truth one to one; dispersal is judged by the guardian's true distance from the impact point at impact; every move is re-judged along its whole path; a landing needs fresh, independent PX4 landed and ground-contact evidence after the vehicle was airborne.
- **Guard:** the checks live in [`integration/acceptance.py`](https://github.com/buicongnguyen/mission-computer-lab/blob/main/integration/acceptance.py) and are tested with synthetic logs, each flaw failing the check that names it. Re-judging the published guardian flights from their published logs reproduces every guardian-stage verdict and figure; the fleet-stage checks need the observer logs, which are not published.

### 16. One bad value could end a flight

- **Seen:** a full flight matrix failed because an order carried an infinite clearance value. Strict JSON logging rejected it, and that vehicle's adapter exited mid-flight. A later review found that one malformed message on several topics would raise inside a ROS callback and stop the node.
- **Fix:** the unavailable value is omitted. Every node checks what it receives (types, finite numbers, vector lengths) and drops a malformed message with a `dropped_message` record instead of stopping.
- **Guard:** `test_malformed_messages_are_dropped_and_logged_never_fatal` sends each node malformed input. The flight check `messages_well_formed` fails any flight in which a message was dropped, so the robustness cannot hide a fault.

### 17. A wall-clock window failed good landings

- **Seen:** the jamming flight, filmed at below real time, failed two landings that PX4 itself had confirmed.
- **Cause:** the check timed the disarm from PX4's last status record, which arrives about 2.8 s after the disarm (its preflight checks fail once the offboard stream stops), and allowed 2 s of wall-clock time. At 0.4 of real time that is under a second of simulated time.
- **Fix:** the check times from the arming transition itself, and allows 4 s for PX4's landed-and-contact report and for the station's touchdown.
- **Guard:** a regression test built from the flight's own timeline; the stale and early reports the earlier test covers are still rejected, and every landing of the earlier flight matrices still passes.

### 18. A landed guardian lost its position fixes

- **Seen:** in the spoofing flight the station recorded one guardian's touchdown 500 s after it had disarmed on its pad.
- **Cause:** a guardian whose GNSS is being dragged flies on the station's datalink fixes. The station stopped updating its fix of a guardian once it disarmed and kept sending the last airborne one, so the landed guardian's corrected position drifted with the ongoing drag-off, and its pad error stayed over the touchdown limit.
- **Fix:** fixes stay current on the deck, and the guardian keeps recording its own track after the handoff, so each fix is matched against its estimate at that moment; the integrity cross-check itself still runs only in flight.
- **Guard:** `test_a_landed_guardian_on_station_fixes_keeps_getting_current_ones` and `test_the_track_goes_on_after_the_handoff`.
