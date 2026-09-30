"""PASS/FAIL acceptance of PX4 flights (integration/acceptance.py), driven with synthetic logs and replayed on the
published evidence: the code that decides a flight passed is tested without a simulator."""

import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'integration'))
import acceptance as A  # noqa: E402
import guardian_layout as GL  # noqa: E402
from evidence_contracts import read_records  # noqa: E402

T0 = 1000.0  # Wall time at which a synthetic log starts.
SINGLE = ('nominal', 'camera_dropout', 'companion_crash', 'gps_loss')


def status(t, nav, arm, failsafe=False):
    return {'kind': 'status', 'wall_time': T0 + t, 'nav_state': nav, 'arming_state': arm, 'failsafe': failsafe}


def single_log(name):
    """Observer and adapter logs of a clean flight of this scenario: OFFBOARD, climb to 3 m, the scenario's fault
    and its handling, LAND, touchdown, disarm. Returns (records, mission_records, injected_at)."""
    ups = [0.5 * k for k in range(6)] + [3.0] * 28 + [2.5 - 0.5 * k for k in range(6)]
    records = [status(0.0, 14, 2), status(34.5, 18, 2), status(41.0, 18, 1)]
    records += [
        {'kind': 'position', 'wall_time': T0 + 1 + k, 'ned': [0.0, 0.0, -up], 'valid': True} for k, up in enumerate(ups)
    ]
    records += [
        {'kind': 'land', 'wall_time': T0 + 40.5, 'landed': True},
        {'kind': 'ack', 'wall_time': T0 + 0.5, 'command': 400, 'result': 0},
        {'kind': 'ack', 'wall_time': T0 + 34.0, 'command': 21, 'result': 0},
        {'kind': 'counts', 'wall_time': T0 + 41.0, 'image': 200, 'perception': 200, 'scan': 200},
        {
            'kind': 'decision',
            'wall_time': T0 + 30.0,
            'mode': 'COMPLETE',
            'reason': 'goal',
            'position': list(A.GOAL),
            'camera_age': 0.1,
        },
    ]
    mission = [
        {'kind': 'transition', 'wall_time': T0 + 2.0, 'mode': 'ACTIVE', 'reason': 'armed'},
        {'kind': 'transition', 'wall_time': T0 + 30.0, 'mode': 'COMPLETE', 'reason': 'goal'},
    ]
    injected = None if name == 'nominal' else T0 + 15.0
    if name == 'camera_dropout':
        records += [
            {
                'kind': 'decision',
                'wall_time': T0 + 16.0,
                'mode': 'HOLD',
                'reason': 'vision_stale',
                'position': [4.0, 4.0, 3.0],
                'camera_age': 0.6,
            },
            {
                'kind': 'decision',
                'wall_time': T0 + 18.0,
                'mode': 'ACTIVE',
                'reason': 'ok',
                'position': [4.0, 4.0, 3.0],
                'camera_age': 0.1,
            },
        ]
        mission.append({'kind': 'transition', 'wall_time': T0 + 16.0, 'mode': 'HOLD', 'reason': 'vision_stale'})
    if name == 'companion_crash':
        records.append(status(16.0, 14, 2, failsafe=True))
    if name == 'gps_loss':
        records.append({'kind': 'gps', 'wall_time': T0 + 16.0, 'fix_type': 0})
        mission.append({'kind': 'transition', 'wall_time': T0 + 16.0, 'mode': 'LAND', 'reason': 'gnss_stale'})
    return sorted(records, key=lambda r: r['wall_time']), sorted(mission, key=lambda r: r['wall_time']), injected


def judge_single(name, records=None, mission=None, injected=None, armed_seen=True, error=None, bag=True):
    clean = single_log(name)
    records, mission = records if records is not None else clean[0], mission if mission is not None else clean[1]
    checks, _ = A.single_flight(name, records, mission, armed_seen, injected or clean[2], error, bag)
    return checks


def failing(checks):
    return {k for k, v in checks.items() if v is not True}


class SingleFlight(unittest.TestCase):
    def test_a_clean_flight_of_each_scenario_passes_with_exactly_its_named_checks(self):
        for name in SINGLE:
            with self.subTest(name):
                checks = judge_single(name)
                self.assertEqual(set(checks), A.required_single(name))
                self.assertEqual(failing(checks), set())

    def test_each_flaw_fails_the_check_that_names_it(self):
        records, mission, _ = single_log('nominal')
        hold = {'kind': 'transition', 'wall_time': T0 + 20.0, 'mode': 'HOLD', 'reason': 'obstacle'}
        high = {'kind': 'position', 'wall_time': T0 + 20.5, 'ned': [0.0, 0.0, -4.2], 'valid': True}
        for flaw, checks, expected in (
            ('never seen armed', judge_single('nominal', armed_seen=False), {'disarmed_at_end'}),
            ('runner error', judge_single('nominal', error='scenario_timeout'), {'no_runner_error'}),
            ('no bag', judge_single('nominal', bag=False), {'rosbag_recorded'}),
            (
                'landing refused',
                judge_single('nominal', [r for r in records if r.get('command') != 21]),
                {'landing_ack_accepted'},
            ),
            (
                'unplanned hold',
                judge_single('nominal', mission=sorted(mission + [hold], key=lambda r: r['wall_time'])),
                {'no_unexpected_faults'},
            ),
            (
                'failsafe',
                judge_single('nominal', records + [status(41.5, 18, 1, failsafe=True)]),
                {'no_unexpected_faults'},
            ),
            (
                'above the ceiling',
                judge_single('nominal', sorted(records + [high], key=lambda r: r['wall_time'])),
                {'altitude_bounded'},
            ),
        ):
            with self.subTest(flaw):
                self.assertEqual(failing(checks), expected)

    def test_a_fault_response_counts_only_after_the_injection(self):
        # A camera HOLD from a startup gap, or a GNSS fix lost before the injection, is not the fault response.
        records = single_log('camera_dropout')[0]
        early = [
            dict(r, wall_time=T0 + 5.0) if r['kind'] == 'decision' and r['mode'] != 'COMPLETE' else r for r in records
        ]
        self.assertEqual(failing(judge_single('camera_dropout', early)), {'camera_hold_observed', 'camera_recovered'})
        records = single_log('gps_loss')[0]
        early = [dict(r, wall_time=T0 + 5.0) if r['kind'] == 'gps' else r for r in records]
        self.assertEqual(failing(judge_single('gps_loss', early)), {'gps_fix_lost'})
        self.assertEqual(
            failing(
                judge_single('companion_crash', [r for r in single_log('companion_crash')[0] if not r.get('failsafe')])
            ),
            {'px4_failsafe_after_crash'},
        )

    def test_a_flight_with_no_evidence_fails_every_check_it_cannot_show_and_omits_none(self):
        for name in SINGLE:
            with self.subTest(name):
                checks, _ = A.single_flight(name, [], [], False, None, None, False)
                self.assertEqual(set(checks), A.required_single(name))
                self.assertEqual(
                    failing(checks), set(checks) - {'altitude_bounded', 'no_runner_error', 'no_unexpected_faults'}
                )


class Fleet(unittest.TestCase):
    def fleet(self, station=(), separation=2.0, dropped=()):
        vehicles = [{'ns': ns} for ns in ('a', 'b')]
        return A.fleet_flight(
            {}, vehicles, list(station), separation, guardian=True, error=None, bag_recorded=True, dropped=list(dropped)
        )

    def test_landings_are_cleared_one_at_a_time(self):
        grant = lambda ns, t: {'kind': 'clearance', 'grant': 'land', 'drone': ns, 'wall_time': T0 + t}
        down = [
            {'kind': 'touchdown', 'drone': 'a', 'wall_time': T0 + 4},
            {'kind': 'touchdown', 'drone': 'b', 'wall_time': T0 + 8},
        ]
        self.assertTrue(self.fleet([grant('a', 1), grant('b', 5)] + down)['landings_sequenced'])
        self.assertFalse(
            self.fleet([grant('a', 1), grant('b', 3)] + down)['landings_sequenced']
        )  # b before a was down.
        self.assertFalse(self.fleet([grant('a', 1)] + down)['landings_sequenced'])  # b was never cleared.

    def test_a_dropped_message_or_a_close_pass_fails_the_flight(self):
        self.assertTrue(self.fleet()['messages_well_formed'])
        self.assertFalse(
            self.fleet(dropped=[{'kind': 'dropped_message', 'topic': 'clearance'}])['messages_well_formed']
        )
        self.assertFalse(self.fleet(separation=0.9)['fleet_min_separation'])
        self.assertFalse(self.fleet(separation=None)['fleet_min_separation'])  # Never two airborne at once: not shown.

    def test_separation_is_time_aligned_and_counts_only_airborne_pairs(self):
        a = [{'wall_time': T0 + k, 'e': 0.0, 'n': 0.0, 'u': 3.0} for k in range(10)]
        b = [{'wall_time': T0 + k + 0.5, 'e': 10.0 - k, 'n': 0.0, 'u': 3.0} for k in range(10)]
        self.assertEqual(A.min_separation([a, b]), 2.0)  # At T0 + 9, b's latest sample is from T0 + 8.5.
        self.assertEqual(A.min_separation([a, b], every=2), 3.0)
        self.assertIsNone(A.min_separation([a, [dict(s, u=A.DECK) for s in b]]))  # b never left the deck.

    def test_a_guardian_landing_and_clearance_are_judged_on_truth(self):
        # The station's estimate says on the pad; where the airframe really came to rest decides.
        v = {'ns': 'px4_1', 'pad': 0.0, 'goal': (0, 12), 'altitude': 5.0}
        touchdown = {'wall_time': T0 + 50, 'pad_error': 0.1, 'position': [0.0, -1.5, 0.62], 'carrier': None}
        carrier = [{'wall_time': T0 + 50, 'e': 0.0, 'n': -1.5, 'yaw': 0.0}]
        rest = {'wall_time': T0 + 50, 't': 50.0, 'e': 0.05, 'n': -1.5, 'u': A.DECK}
        own, vehicle = A.fleet_vehicle(v, [0.0, -1.5, A.DECK], [], [], touchdown, carrier, [rest])
        self.assertTrue(own['px4_1_landed_on_pad'])
        self.assertAlmostEqual(vehicle['touchdown']['true_pad_error'], 0.05)
        own, _ = A.fleet_vehicle(v, [0.0, -1.5, A.DECK], [], [], touchdown, carrier, [dict(rest, e=1.0)])
        self.assertFalse(own['px4_1_landed_on_pad'])
        through = {'wall_time': T0 + 20, 't': 20.0, 'e': 4.0, 'n': 3.0, 'u': 3.0}  # Inside the cylinder at (4, 3).
        own, _ = A.fleet_vehicle(v, [0.0, -1.5, A.DECK], [], [], touchdown, carrier, [through, rest])
        self.assertFalse(own['px4_1_obstacle_clearance'])

    def test_every_multi_vehicle_scenario_produces_exactly_the_checks_the_publisher_requires(self):
        for name in ('fleet_carrier', *GL.SCENARIOS):
            with self.subTest(name):
                guardian = name != 'fleet_carrier'
                layout = (
                    [{**g, 'goal': g['post']} for g in GL.GUARDIANS]
                    if guardian
                    else [{'ns': f'px4_{i}', 'pad': 0.0, 'goal': (9, 9), 'altitude': 3.0} for i in range(3)]
                )
                checks, vehicles = {}, []
                for v in layout:
                    own, vehicle = A.fleet_vehicle(v, [0.0, 0.0, A.DECK], [], [], None, [], [] if guardian else None)
                    checks.update(own)
                    vehicles.append(vehicle)
                checks = A.fleet_flight(
                    checks, vehicles, [], None, guardian=guardian, error=None, bag_recorded=False, dropped=[]
                )
                if guardian:
                    A.guardian_flight(name, [], [], [], vehicles, [], checks)
                required = set(GL.expected_checks(name)) if guardian else A.required_fleet([v['ns'] for v in layout])
                self.assertEqual(set(checks), required)
                self.assertFalse(all(checks.values()))  # A flight with no evidence never passes.


def truth(t, guardians=None, threats=None):
    return {'kind': 'truth', 't': t, 'wall_time': T0 + t, 'guardians': guardians or {}, 'threats': threats or {}}


def on_post(**moved):
    """Every guardian at its post, except those given as ns=(e, n)."""
    return {g['ns']: [*moved.get(g['ns'], g['post']), g['altitude']] for g in GL.GUARDIANS}


def guardian(name, world=(), station=(), center=(), decisions=None, carrier=()):
    vehicles = [{'ns': g['ns'], 'guardian': (decisions or {}).get(g['ns'], [])} for g in GL.GUARDIANS]
    checks = {}
    extra = A.guardian_flight(name, list(world), list(center), list(station), vehicles, list(carrier), checks)
    return checks, extra


class GuardianFlight(unittest.TestCase):
    def test_dispersal_is_judged_on_truth_at_impact(self):
        start = {'kind': 'scenario_start', 't': 0.0}
        impact = {'kind': 'impact', 't': 8.0, 'model': 'fast'}
        order = {
            'kind': 'order',
            't': 5.0,
            'ns': 'px4_0',
            'action': 'disperse',
            'own': [1, 2, 4],
            'target': [4, 6, 4],
            'avoid': [],
            'fast': [],
        }
        away = [start, truth(0.0, on_post()), truth(8.0, on_post(px4_0=(4.0, 6.0))), impact]
        checks, extra = guardian('guardian_fast', away, [order])
        self.assertTrue(checks['dispersed_before_impact'])
        self.assertAlmostEqual(extra['dispersal']['post_m'], math.dist((1, 2), (0, -1.5)))
        self.assertAlmostEqual(extra['dispersal']['impact_m'], math.dist((4, 6), (0, -1.5)))
        stayed = [start, truth(0.0, on_post()), truth(8.0, on_post()), impact]
        self.assertFalse(
            guardian('guardian_fast', stayed, [order])[0]['dispersed_before_impact']
        )  # Ordered, never moved.
        self.assertFalse(
            guardian('guardian_fast', away, [dict(order, t=9.0)])[0]['dispersed_before_impact']
        )  # Too late.

    def test_a_move_that_closes_on_a_slow_threat_or_shortens_a_fast_objects_miss_fails(self):
        # Slow intruder 5 m north; fast object on a line 2 m east of the guardian, passing in 4 s.
        own = [0.0, 0.0, 4.0]
        fast = [[[2.0, 20.0, 4.0], [0.0, -5.0, 0.0]]]
        move = lambda target, **k: {
            'kind': 'order',
            't': 1.0,
            'ns': 'px4_1',
            'action': 'keep_clear',
            'own': own,
            'target': target,
            **k,
        }
        for target, logged, safe in (
            ([0.0, -3.0, 4.0], {'avoid': [[0.0, 5.0, 4.0]]}, True),
            ([0.0, 3.0, 4.0], {'avoid': [[0.0, 5.0, 4.0]]}, False),  # Toward the intruder.
            ([-3.0, 0.0, 4.0], {'avoid': [], 'fast': fast}, True),
            ([3.0, 0.0, 4.0], {'avoid': [], 'fast': fast}, False),  # Into the fast object's path: 1 m miss, not 2.
            ([3.0, 0.0, 4.0], {'fast': fast}, True),  # A move logged without what it avoided is not judged.
        ):
            with self.subTest(target=target, logged=sorted(logged)):
                self.assertIs(
                    guardian('guardian_intruder', station=[move(target, **logged)])[0]['never_closed_on_threat'], safe
                )
                onboard = {'px4_1': [dict(move(target, **logged), layer='onboard')]}
                self.assertIs(guardian('guardian_intruder', decisions=onboard)[0]['never_closed_on_threat'], safe)

    def test_delegated_recovery_waits_for_the_center_timeout_and_center_recovery_follows_its_decision(self):
        request = {'kind': 'center_report', 't': 50.0, 'report': 'clear_after_red'}
        recovered = lambda t, by='station (delegated)': {'kind': 'recovery', 't': t, 'by': by, 'wall_time': T0 + t}
        judge = lambda station: guardian('guardian_center_loss', station=station)[0]['recovery_under_delegation']
        self.assertTrue(judge([request, recovered(50.0 + GL.CFG.center_timeout)]))
        self.assertFalse(judge([request, recovered(60.0)]))  # Did not wait for the center.
        self.assertFalse(judge([request, {'kind': 'center_decision', 'wall_time': T0 + 55}, recovered(71.0)]))
        self.assertFalse(judge([recovered(71.0)]))  # No request to the center at all.
        decision = {'kind': 'center_decision', 'wall_time': T0 + 55}
        judge = lambda station: guardian('guardian_intruder', station=station)[0]['recovery_by_center']
        self.assertTrue(judge([decision, recovered(56.0, 'center')]))
        self.assertFalse(judge([decision, recovered(54.0, 'center')]))  # Before the center decided.

    def test_distinct_tracks_are_matched_one_to_one(self):
        confirmed = lambda track, p: {'kind': 'confirmed', 't': 10.0, 'track': track, 'p': p}

        def judge(threats, tracks):
            return guardian('guardian_swarm', [truth(10.0, on_post(), threats)], tracks)[0]['distinct_tracks_confirmed']

        apart = {'intruder': [0.0, 10.0, 4.0], 'intruder_2': [5.0, 10.0, 4.0], 'intruder_3': [10.0, 10.0, 7.0]}
        twice = [confirmed(1, [0.2, 10.0, 4.0]), confirmed(2, [0.0, 10.3, 4.0]), confirmed(3, [10.0, 10.0, 7.0])]
        self.assertFalse(judge(apart, twice))  # Two tracks on one threat count once.
        self.assertTrue(judge(apart, twice + [confirmed(4, [5.0, 10.0, 4.0])]))
        close = dict(apart, intruder_2=[1.0, 10.0, 4.0])
        between = [confirmed(1, [0.5, 10.0, 4.0]), confirmed(3, [10.0, 10.0, 7.0])]
        self.assertFalse(judge(close, between))  # One track between two threats counts once.
        self.assertTrue(judge(close, between + [confirmed(2, [1.2, 10.0, 4.0])]))

    def test_every_scripted_threat_must_really_fly(self):
        start = {'kind': 'scenario_start', 't': 0.0}
        flying = [truth(0.2 * k, on_post(), {'intruder': [-3.4, 23.0 - 0.24 * k, 4.0]}) for k in range(1, 11)]
        judge = lambda world: guardian('guardian_intruder', [start] + world)[0]['threats_flew']
        self.assertTrue(judge(flying))
        self.assertFalse(judge([dict(r, t=r['t'] + 3.0) for r in flying]))  # First seen 3 s after its start.
        self.assertFalse(judge([dict(r, threats={'intruder': [-3.4, 23.0, 4.0]}) for r in flying]))  # Never moved.
        self.assertFalse(judge([]))

    def test_a_scenario_with_no_evidence_fails_and_omits_none_of_its_checks(self):
        for name in GL.SCENARIOS:
            with self.subTest(name):
                checks, _ = guardian(name)
                self.assertLessEqual(set(GL.expected_checks(name)), set(checks))
                self.assertFalse(checks['scenario_started'])


class PublishedEvidence(unittest.TestCase):
    def test_published_guardian_verdicts_re_derive_from_their_logs(self):
        """Re-judging each published guardian flight from its published logs reproduces its verdicts and figures."""
        fleet_stage = {f'{g["ns"]}_{c}' for g in GL.GUARDIANS for c in GL.VEHICLE_CHECKS} | set(A.FLEET_CHECKS)
        for name in GL.SCENARIOS:
            with self.subTest(name):
                folder = ROOT / 'artifacts/sitl-sample' / name
                result = json.loads((folder / 'result.json').read_text())
                checks = {k: v for k, v in result['checks'].items() if k in fleet_stage}
                logs = [read_records(folder / f) for f in ('world.jsonl', 'center.jsonl', 'station.jsonl')]
                extra = A.guardian_flight(name, *logs, result['vehicles'], result['carrier'], checks)
                own = lambda c: {k: v for k, v in c.items() if k not in fleet_stage}
                self.assertEqual(own(checks), own(result['checks']))
                for key in ('warning_s', 'station_miss_m', 'guardian_separation_m', 'drift_m', 'dispersal'):
                    self.assertEqual(extra[key], result[key], key)


if __name__ == '__main__':
    unittest.main()
