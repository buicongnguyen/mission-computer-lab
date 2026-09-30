"""Acceptance checks for every PX4 flight, as pure functions of its recorded logs.

run_sitl.py reads a flight's logs and calls these; tools/publish_sitl.py requires the check names they produce;
tests/test_acceptance.py drives them with synthetic records, so the code that decides PASS is itself tested
without a simulator. Every required check is always present: one that could not be evaluated is False, never
missing. Nothing here imports ROS.
"""

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'integration'))
from evidence_contracts import flight_cycle_checks, fleet_landing_confirmed  # noqa: E402
from guardian import authorised, safe_move  # noqa: E402
import guardian_layout as GL  # noqa: E402
from world import GOAL as INSPECTION_XY, OBSTACLES  # noqa: E402

DECK = 0.6  # m: carrier deck height in the fleet and guardian worlds.
GOAL = (*INSPECTION_XY, 3.0)  # The single drone's inspection goal, at its default 3 m cruise altitude.

# Every single-drone flight: observed telemetry, an ordered climb -> land -> disarm cycle and recorded payloads.
SINGLE_CHECKS = (
    'telemetry_received',
    'offboard_entered',
    'takeoff_observed',
    'disarmed_at_end',
    'estimated_obstacle_clearance',
    'altitude_bounded',
    'land_mode_observed',
    'image_messages',
    'perception_messages',
    'lidar_messages',
    'arming_ack_accepted',
    'no_runner_error',
    'ordered_flight_cycle',
    'landed_at_end',
    'final_pose_near_ground',
    'finite_positions',
    'estimator_valid_before_fault',
    'rosbag_recorded',
)
# What each single-drone scenario must additionally show.
SINGLE_SCENARIO_CHECKS = {
    'nominal': ('mission_complete', 'goal_reached', 'landing_ack_accepted', 'no_unexpected_faults'),
    'camera_dropout': (
        'mission_complete',
        'goal_reached',
        'landing_ack_accepted',
        'no_unexpected_faults',
        'fault_injected',
        'camera_hold_observed',
        'camera_recovered',
    ),
    'companion_crash': ('fault_injected', 'px4_failsafe_after_crash'),
    'gps_loss': ('fault_injected', 'gps_fault_handled', 'gps_fix_lost'),
}
FLEET_VEHICLE_CHECKS = (
    'offboard_entered',
    'takeoff_observed',
    'goal_reached',
    'returned_to_carrier',
    'landed_on_pad',
    'carrier_moving_at_touchdown',
    'disarmed_at_end',
    'no_failsafe',
    'obstacle_clearance',
    'landed_confirmed',
)
FLEET_CHECKS = (
    'fleet_min_separation',
    'landings_sequenced',
    'carrier_moved',
    'rosbag_recorded',
    'no_runner_error',
    'messages_well_formed',
)


def required_single(name):
    """Named checks a single-drone result must carry."""
    return set(SINGLE_CHECKS) | set(SINGLE_SCENARIO_CHECKS[name])


def required_fleet(names):
    """Named checks the fleet result must carry, for these vehicle namespaces."""
    return {f'{ns}_{c}' for ns in names for c in FLEET_VEHICLE_CHECKS} | set(FLEET_CHECKS)


def never_by_omission(checks, required):
    """A required check that could not be evaluated fails; it is never silently absent."""
    for missing in set(required) - set(checks):
        checks[missing] = False
    return checks


def single_flight(name, records, mission_records, armed_seen, injected_at, error, bag_recorded):
    """Checks for one single-drone flight, and the facts its result reports.

    records: the independent observer's log; mission_records: the adapter's log; armed_seen: whether the
    runner saw the vehicle armed; injected_at: wall time the fault was injected, if any."""
    statuses = [r for r in records if r['kind'] == 'status']
    poses = [r for r in records if r['kind'] == 'position']
    decisions = [r for r in records if r['kind'] == 'decision']
    acks = [r for r in records if r['kind'] == 'ack']
    counts = [r for r in records if r['kind'] == 'counts']
    max_alt = max([-r['ned'][2] for r in poses], default=0.0)
    transitions = [r for r in mission_records if r['kind'] == 'transition']
    min_clearance = min(
        (
            math.hypot(p['ned'][1] - x, p['ned'][0] - y) - radius
            for p in poses
            if p['valid']
            for x, y, radius in OBSTACLES
        ),
        default=0.0,
    )
    checks = {
        'telemetry_received': len(poses) > 30,
        'offboard_entered': any(s['nav_state'] == 14 and s['arming_state'] == 2 for s in statuses),
        'takeoff_observed': max_alt > 2.0,
        'disarmed_at_end': bool(statuses and statuses[-1]['arming_state'] == 1 and armed_seen),
        'estimated_obstacle_clearance': min_clearance > 0.5,
        'altitude_bounded': max_alt < 4.0,
        'land_mode_observed': any(s['nav_state'] == 18 and s['arming_state'] == 2 for s in statuses),
        'image_messages': bool(counts and counts[-1].get('image', 0) > 20),
        'perception_messages': bool(counts and counts[-1].get('perception', 0) > 20),
        'lidar_messages': bool(counts and counts[-1].get('scan', 0) > 20),
        'arming_ack_accepted': any(a['command'] == 400 and a['result'] == 0 for a in acks),
        'no_runner_error': error is None,
    }
    checks.update(flight_cycle_checks(records, injected_at if name == 'gps_loss' else None))
    checks['rosbag_recorded'] = bag_recorded
    if name in ('nominal', 'camera_dropout'):
        checks['mission_complete'] = any(r.get('mode') == 'COMPLETE' for r in transitions)
        checks['goal_reached'] = any(
            r.get('mode') == 'COMPLETE' and math.dist(r['position'], GOAL) < 0.5 for r in decisions
        )
        checks['landing_ack_accepted'] = any(a['command'] == 21 and a['result'] == 0 for a in acks)
        # Only the injected fault may interrupt these flights: no other HOLD or LAND and no PX4 failsafe.
        allowed = {'nominal': set(), 'camera_dropout': {'vision_stale'}}[name]
        checks['no_unexpected_faults'] = not any(s['failsafe'] for s in statuses) and all(
            t['mode'] not in ('HOLD', 'LAND') or (t['mode'] == 'HOLD' and t['reason'] in allowed) for t in transitions
        )
    if name == 'camera_dropout':
        checks['fault_injected'] = injected_at is not None
        # The HOLD must follow the in-flight injection, not a startup gap before takeoff.
        holds = [
            r
            for r in decisions
            if r['mode'] == 'HOLD'
            and r['reason'] == 'vision_stale'
            and r['camera_age'] > 0.3
            and injected_at is not None
            and r['wall_time'] >= injected_at
        ]
        checks['camera_hold_observed'] = bool(holds)
        checks['camera_recovered'] = bool(
            holds and any(r['mode'] == 'ACTIVE' and r['wall_time'] > holds[0]['wall_time'] for r in decisions)
        )
    if name == 'companion_crash':
        checks['fault_injected'] = injected_at is not None
        checks['px4_failsafe_after_crash'] = (
            any(r['failsafe'] and r['wall_time'] > injected_at for r in statuses) if injected_at else False
        )
    if name == 'gps_loss':
        checks['fault_injected'] = injected_at is not None
        checks['gps_fault_handled'] = (
            any(
                r['mode'] == 'LAND' and r['reason'] == 'gnss_stale' and r['wall_time'] >= injected_at
                for r in transitions
            )
            if injected_at
            else False
        )
        checks['gps_fix_lost'] = (
            any(r['kind'] == 'gps' and r['fix_type'] < 3 and r['wall_time'] >= injected_at for r in records)
            if injected_at
            else False
        )
    never_by_omission(checks, required_single(name))
    facts = {
        'statuses': statuses,
        'poses': poses,
        'decisions': decisions,
        'acks': acks,
        'counts': counts,
        'max_alt': max_alt,
        'transitions': transitions,
        'min_clearance': min_clearance,
    }
    return checks, facts


def world_trace(records, spawn):
    """Observer positions (PX4 local NED from the spawn point) as world ENU samples."""
    return [
        {
            'wall_time': p['wall_time'],
            'e': p['ned'][1] + spawn[0],
            'n': p['ned'][0] + spawn[1],
            'u': -p['ned'][2] + spawn[2],
            'valid': p['valid'],
        }
        for p in records
        if p['kind'] == 'position'
    ]


def fleet_vehicle(v, spawn, records, mission_records, touchdown, carrier, truth=None):
    """Checks for one fleet or guardian vehicle (named '<ns>_<check>'), and its result record.

    v: the vehicle's layout (ns, pad, goal, altitude); touchdown: the station's touchdown record, if any;
    carrier: the station's carrier samples; truth: Gazebo truth samples for a guardian vehicle, whose
    clearance and landing are judged on truth because a spoofed receiver misleads the estimate."""
    ns = v['ns']
    guardian = truth is not None
    statuses = [r for r in records if r['kind'] == 'status']
    trace = world_trace(records, spawn)
    phases = [r for r in mission_records if r['kind'] == 'phase']
    names = [p['phase'] for p in phases]
    inspect = next((p for p in phases if p['phase'] == 'inspect'), None)
    clearance = min(
        (
            math.hypot(t['e'] - x, t['n'] - y) - r
            for t in (truth if guardian else trace)
            if (guardian or t['valid']) and t['u'] > DECK + 0.3
            for x, y, r in OBSTACLES
        ),
        default=0.0,
    )
    checks = {}
    checks[f'{ns}_offboard_entered'] = any(s['nav_state'] == 14 and s['arming_state'] == 2 for s in statuses)
    checks[f'{ns}_takeoff_observed'] = max((t['u'] for t in trace), default=0.0) > 2.0
    if guardian:
        watch = next((p for p in phases if p['phase'] == 'watch'), None)
        checks[f'{ns}_on_watch'] = bool(watch and math.dist(watch['position'][:2], v['goal']) < 0.6)
    else:
        checks[f'{ns}_goal_reached'] = bool(inspect and math.dist(inspect['position'][:2], v['goal']) < 0.6)
    checks[f'{ns}_returned_to_carrier'] = 'rendezvous' in names and 'descend' in names
    checks[f'{ns}_landed_on_pad'] = bool(
        touchdown
        and touchdown['pad_error'] is not None
        and touchdown['pad_error'] < 0.4
        and abs(touchdown['position'][2] - DECK) < 0.35
    )
    if guardian and touchdown and truth and carrier:  # Where the airframe came to rest, against its pad.
        rest = min(truth, key=lambda t: abs(t['wall_time'] - touchdown['wall_time']))
        c = min(carrier, key=lambda x: abs(x['wall_time'] - touchdown['wall_time']))
        pad = (c['e'] + v['pad'] * math.cos(c['yaw']), c['n'] + v['pad'] * math.sin(c['yaw']))
        touchdown = {**touchdown, 'true_pad_error': math.hypot(rest['e'] - pad[0], rest['n'] - pad[1])}
        checks[f'{ns}_landed_on_pad'] = checks[f'{ns}_landed_on_pad'] and touchdown['true_pad_error'] < 0.4
    if not guardian:
        checks[f'{ns}_carrier_moving_at_touchdown'] = bool(
            touchdown and touchdown['carrier'] and touchdown['carrier']['speed'] > 0.15
        )
    checks[f'{ns}_disarmed_at_end'] = bool(statuses and statuses[-1]['arming_state'] == 1)
    checks[f'{ns}_landed_confirmed'] = fleet_landing_confirmed(records, touchdown)
    checks[f'{ns}_no_failsafe'] = not any(s['failsafe'] for s in statuses)
    checks[f'{ns}_obstacle_clearance'] = clearance > 0.5
    vehicle = {
        **v,
        'goal': list(v['goal']),
        'spawn': spawn,
        'trace': trace,
        'statuses': statuses,
        'phases': phases,
        'plan': next((r for r in mission_records if r['kind'] == 'plan'), None),
        'replans': [r for r in mission_records if r['kind'] == 'replan'],
        'transitions': [r for r in mission_records if r['kind'] == 'transition'],
        'touchdown': touchdown,
        'min_clearance_m': clearance,
        'guardian': [r for r in mission_records if r['kind'] == 'guardian'],
        **({'truth': truth[::2]} if guardian else {}),
    }
    return checks, vehicle


def resample(trace, times):
    """World positions at the given wall times (nearest earlier sample), or None before the first sample."""
    out, i = [], 0
    for t in times:
        while i + 1 < len(trace) and trace[i + 1]['wall_time'] <= t:
            i += 1
        out.append(trace[i] if trace and trace[0]['wall_time'] <= t else None)
    return out


def min_separation(series, every=1):
    """Closest distance between two airborne vehicles, time-aligned on the first vehicle's samples."""
    times = [t['wall_time'] for t in series[0]][::every] if series and series[0] else []
    tracks = [resample(t, times) for t in series]
    separations = []
    for k in range(len(times)):
        airborne = [tr[k] for tr in tracks if tr[k] and tr[k]['u'] > DECK + 0.5]
        separations += [
            math.dist((a['e'], a['n'], a['u']), (b['e'], b['n'], b['u']))
            for j, a in enumerate(airborne)
            for b in airborne[j + 1 :]
        ]
    return min(separations) if separations else None


def fleet_flight(vehicle_checks, vehicles, station, separation, *, guardian, error, bag_recorded, dropped):
    """The per-vehicle checks followed by the fleet's: separation, sequenced landings, the carrier's drive, the
    recording, the runner, and malformed messages (dropped: the flight's dropped_message records; a node drops
    a malformed message instead of dying, but a drop in a flight is still a fault). A guardian scenario's
    completeness is enforced by guardian_flight, which adds the rest of its checks."""
    checks = dict(vehicle_checks)
    carrier = [r for r in station if r['kind'] == 'carrier']
    grants = [r for r in station if r['kind'] == 'clearance']
    touchdowns = {r['drone']: r for r in station if r['kind'] == 'touchdown'}
    lands = [g for g in grants if g['grant'] == 'land']
    checks['fleet_min_separation'] = separation is not None and separation > 1.0
    checks['landings_sequenced'] = len(lands) == len(vehicles) and all(
        lands[k - 1]['drone'] in touchdowns and lands[k]['wall_time'] >= touchdowns[lands[k - 1]['drone']]['wall_time']
        for k in range(1, len(lands))
    )
    if not guardian:
        checks['carrier_moved'] = bool(carrier) and carrier[-1]['e'] - carrier[0]['e'] > 3.0
    checks['rosbag_recorded'] = bag_recorded
    checks['no_runner_error'] = error is None
    checks['messages_well_formed'] = not dropped
    if not guardian:
        never_by_omission(checks, required_fleet([v['ns'] for v in vehicles]))
    return checks


def guardian_truth(world):
    """Truth from the environment's log: every guardian and threat at the same instant, 5 times a second."""
    records = [r for r in world if r['kind'] == 'truth']
    guardians = {
        g['ns']: [
            {'wall_time': r['wall_time'], 't': r['t'], 'e': p[0], 'n': p[1], 'u': p[2]}
            for r in records
            for p in [r['guardians'].get(g['ns'])]
            if p
        ]
        for g in GL.GUARDIANS
    }
    return records, guardians


def guardian_flight(name, world, center, station, vehicles, carrier, checks):
    """Acceptance for one guardian scenario, from the environment's truth, the station, the center and the
    vehicles. Adds its checks to `checks` (which already holds the fleet ones) and returns the extra fields
    of the result."""
    scenario = GL.SCENARIOS[name]
    expect = scenario['expect']
    cfg = GL.CFG
    records, _ = guardian_truth(world)
    start = next((r for r in world if r['kind'] == 'scenario_start'), None)
    posture = [r for r in station if r['kind'] == 'posture']
    orders = [r for r in station if r['kind'] == 'order']
    red = next((r for r in posture if r['state'] == 'RED'), None)
    relocate = next((r for r in station if r['kind'] == 'relocate'), None)
    recovery = next((r for r in station if r['kind'] == 'recovery'), None)
    decisions = [r for r in station if r['kind'] == 'center_decision']
    hostile = [m for m, spec in scenario['threats'].items() if spec['kind'] != 'bird']
    dispersal = None
    checks['scenario_started'] = start is not None
    # Separation and arrival, from truth sampled at the same instant for every body. Arrival is judged at each
    # threat's aim point, where the carrier was parked: once the carrier drives off, its own closest approach
    # would move the goalposts.
    separations = {g['ns']: math.inf for g in GL.GUARDIANS}
    arrival = None
    closest = (math.inf, None)
    station_miss = math.inf
    for r in records:
        if not r['threats']:
            continue
        c = next((x for x in carrier if x['wall_time'] >= r['wall_time']), carrier[-1] if carrier else None)
        for m in hostile:
            q = r['threats'].get(m)
            if not q or q[2] < 0.0:
                continue  # Below ground: a diving object after its impact.
            for ns, p in r['guardians'].items():
                if p and p[2] > DECK + 0.5:
                    separations[ns] = min(separations[ns], math.dist(p, q))
            if c:
                station_miss = min(station_miss, math.hypot(q[0] - c['e'], q[1] - c['n']))
            aim = scenario['threats'][m]['aim']
            d = math.hypot(q[0] - aim[0], q[1] - aim[1])
            if d < closest[0]:
                closest = (d, r['t'])
            if arrival is None and d < cfg.protect_radius:
                arrival = r['t']
    arrival = arrival if arrival is not None else closest[1]
    warning = None if red is None or arrival is None else arrival - red['t']

    def matches(record):
        """Hostile threats within 2 m of a confirmed track when it was confirmed, on truth."""
        near = min(records, key=lambda x: abs(x['t'] - record['t'])) if records else None
        return [
            (math.dist(near['threats'][m], record['p']), m)
            for m in hostile
            if near and near['threats'].get(m) and math.dist(near['threats'][m], record['p']) < 2.0
        ]

    confirmed = [r for r in station if r['kind'] == 'confirmed']
    if hostile:
        checks['guardians_kept_clear'] = all(v >= cfg.safe_radius for v in separations.values())
    if expect['red']:
        checks['threat_confirmed'] = any(matches(r) for r in confirmed)  # A real threat, not clutter or a bird.
        checks['red_before_arrival'] = warning is not None and warning > 0
    else:
        checks['no_red_alert'] = red is None
        # Birds, clutter or a drag-off may cause a brief keep-clear from a noisy track, which is no alarm; what
        # must hold is the watch: on truth, no guardian strayed more than one keep-clear step from its post.
        end = next((r['t'] for r in station if r['kind'] in ('recovery', 'watch_complete')), None)
        watch = [r for r in records if start and r['t'] >= start['t'] and (end is None or r['t'] <= end)]
        strayed = max(
            (
                math.dist(r['guardians'][g['ns']][:2], g['post'])
                for r in watch
                for g in GL.GUARDIANS
                if r['guardians'].get(g['ns'])
            ),
            default=math.inf,
        )
        checks['guardians_held_their_posts'] = strayed <= cfg.keep_clear_step + 0.5
    if expect.get('relocate'):
        checks['carrier_relocated_clear'] = bool(relocate) and station_miss >= cfg.protect_radius
    if expect.get('station_order'):
        jammed = expect.get('jammed')
        checks['station_ordered_keep_clear'] = any(
            d['action'] == 'keep_clear' and d['layer'] == 'station'
            for v in vehicles
            if v['ns'] != jammed
            for d in v['guardian']
        )
    if expect.get('jammed'):
        ns = expect['jammed']
        alone = [
            d
            for v in vehicles
            if v['ns'] == ns
            for d in v['guardian']
            if d['layer'] == 'onboard' and (d['link_age'] or 0) > cfg.lost_link
        ]
        checks['jammed_guardian_acted_alone'] = any(d['action'] == 'keep_clear' for d in alone) and any(
            r['kind'] == 'dropped' and r['ns'] == ns for r in world
        )
    if expect.get('tracks'):
        # Distinct hostile threats the station confirmed a track on, one track per threat, nearest pairs first.
        pairs = sorted((d, r['track'], m) for r in confirmed for d, m in matches(r))
        used = set()
        matched = set()
        for _, track, m in pairs:
            if track not in used and m not in matched:
                used.add(track)
                matched.add(m)
        checks['distinct_tracks_confirmed'] = len(matched) >= expect['tracks']
    if expect.get('disperse'):
        # Ordered away before impact, and on truth further from the impact point at impact than its post is.
        ns = expect['disperse']
        impact = next((r for r in world if r['kind'] == 'impact'), None)
        ordered = [o for o in orders if o['ns'] == ns and o['action'] in ('disperse', 'keep_clear')]
        post = next(g['post'] for g in GL.GUARDIANS if g['ns'] == ns)
        at = min(records, key=lambda x: abs(x['t'] - impact['t'])) if impact and records else None
        where = at and at['guardians'].get(ns)
        aim = scenario['threats'][impact['model']]['aim'] if impact else None
        checks['dispersed_before_impact'] = bool(
            impact
            and ordered
            and ordered[0]['t'] < impact['t']
            and where
            and math.dist(where[:2], aim[:2]) > math.dist(post, aim[:2]) + 0.5
        )
        dispersal = {
            'ns': ns,
            'post_m': math.dist(post, aim[:2]) if aim else None,
            'impact_m': math.dist(where[:2], aim[:2]) if where and aim else None,
        }
    if expect.get('spoof'):
        spoof = next((r for r in world if r['kind'] == 'spoof_start'), None)
        alarms = [r for r in station if r['kind'] == 'integrity']
        checks['spoofing_detected'] = bool(spoof and alarms and alarms[0]['t'] - spoof['t'] < 20.0)
        switched = {v['ns'] for v in vehicles for d in v['guardian'] if d['action'] == 'navigate_by_station'}
        checks['navigated_by_station_fixes'] = switched == {g['ns'] for g in GL.GUARDIANS}
        drift = {}
        for g in GL.GUARDIANS:  # True distance from the post while on watch, which the drag-off pulls on.
            post = g['post']
            watching = [
                r for r in records if spoof and r['t'] >= spoof['t'] and (recovery is None or r['t'] < recovery['t'])
            ]
            drift[g['ns']] = max(
                (
                    math.hypot(r['guardians'][g['ns']][0] - post[0], r['guardians'][g['ns']][1] - post[1])
                    for r in watching
                    if r['guardians'].get(g['ns'])
                ),
                default=None,
            )
        checks['drift_bounded'] = all(d is not None and d < 3.0 for d in drift.values())
    else:
        drift = None
    by = expect['recovery']
    if by == 'center':
        checks['recovery_by_center'] = bool(
            recovery and recovery['by'] == 'center' and decisions and recovery['wall_time'] >= decisions[0]['wall_time']
        )
    else:
        request = next(
            (
                r
                for r in station
                if r['kind'] == 'center_report' and r['report'] in ('clear_after_red', 'watch_complete')
            ),
            None,
        )
        checks['recovery_under_delegation'] = bool(
            recovery
            and recovery['by'] == by
            and not decisions
            and request
            and recovery['t'] - request['t'] >= cfg.center_timeout - 0.5
        )
    # Every keep-clear or dispersal decision, by station or vehicle, was a safe move against what it logged: it
    # opened the range along its whole path to every relevant slow track, and shortened no steady fast object's
    # predicted miss. Both tests are exact for straight lines.
    closing = []
    for r in orders + [d for v in vehicles for d in v['guardian'] if d['layer'] == 'onboard']:
        if r['action'] in ('keep_clear', 'disperse') and r.get('target') and r.get('avoid') is not None:
            if not safe_move(r['own'], r['target'], r['avoid'], r.get('fast') or [], cfg):
                closing.append(r)
    checks['never_closed_on_threat'] = not closing
    # An order relaying a center decision (recovery) carries the center's authority, not the station's.
    decided = [(d['action'], d['layer']) for v in vehicles for d in v['guardian']] + [
        (r['action'], r.get('authority') or 'station') for r in orders
    ]
    checks['authority_respected'] = all(authorised(a, layer) for a, layer in decided)
    if scenario['threats']:
        # Every scripted threat really flew: truth within 2 s of its start, and at least a metre of travel.
        flew = True
        for m, spec in scenario['threats'].items():
            begin = (start['t'] if start else math.inf) + spec.get('delay', 0.0)
            path = [r['threats'][m] for r in records if r['threats'].get(m) and r['t'] >= begin]
            first = next((r['t'] for r in records if r['threats'].get(m) and r['t'] >= begin), math.inf)
            flew &= bool(path) and first - begin <= 2.0 and sum(math.dist(a, b) for a, b in zip(path, path[1:])) >= 1.0
        checks['threats_flew'] = flew
    never_by_omission(checks, GL.expected_checks(name))
    threats = {
        m: {
            'kind': spec['kind'],
            'trace': [
                {
                    'wall_time': r['wall_time'],
                    't': r['t'],
                    'e': r['threats'][m][0],
                    'n': r['threats'][m][1],
                    'u': r['threats'][m][2],
                }
                for r in records
                if r['threats'].get(m)
            ],
        }
        for m, spec in scenario['threats'].items()
    }
    return {
        'title': scenario['title'],
        'expect': expect,
        'threats': threats,
        'scenario_start': start,
        'warning_s': warning,
        'station_miss_m': None if math.isinf(station_miss) else station_miss,
        'guardian_separation_m': {k: (None if math.isinf(v) else v) for k, v in separations.items()},
        'posture': posture,
        'orders': orders,
        'relocation': [r for r in station if r['kind'] in ('relocate', 'relocated')],
        'center': [r for r in center if r['kind'] in ('decision', 'unreachable')],
        'recovery': recovery,
        'integrity': [r for r in station if r['kind'] in ('integrity', 'integrity_clear')],
        'drift_m': drift,
        'dispersal': dispersal,
        'jamming': [
            r
            for r in world
            if r['kind']
            in ('scenario_start', 'jammer_off', 'link', 'spoof_start', 'spoof_error', 'impact', 'threat_stop')
        ],
        'layout': {
            'guardians': GL.GUARDIANS,
            'jammer': scenario.get('jammer'),
            'spoof': scenario.get('spoof'),
            'relocate': GL.RELOCATE,
            'safe_radius': cfg.safe_radius,
            'clear_radius': cfg.clear_radius,
            'protect_radius': cfg.protect_radius,
            'center_down': bool(scenario.get('center_down')),
            'watch': scenario.get('watch'),
        },
    }
