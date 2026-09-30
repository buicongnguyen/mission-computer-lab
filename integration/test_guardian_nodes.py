"""Guardian SITL nodes run as real ROS nodes without Gazebo or PX4: logging, fusion, posture, orders, relay,
navigation integrity and the scenario scripts."""

import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace, MethodType
import unittest
from unittest.mock import Mock
import rclpy
from std_msgs.msg import String
from center_node import CenterNode
from guardian import Track, Tracker, authorised, safe_move
from guardian_layout import (
    CFG,
    EAST_HIGH,
    GUARDIANS,
    RELOCATE,
    SCENARIOS,
    NORTH,
    FAST,
    expected_checks,
    free_space,
    impact_time,
    reserved_for,
    threat_position,
    world_sdf,
)
from guardian_mission_node import GuardianMission
from fleet_mission_node import FleetMission
from fleet_contracts import FreshInput
from mission_node import Mission
from guardian_station_node import GuardianStation
from guardian_world_node import World


def message(data):
    m = String()
    m.data = json.dumps(data)
    return m


def intruder_at(t):
    return threat_position(NORTH, t)[0]


class Fresh(dict):
    """Receipt times of guardian states in the tests: always just heard."""

    def get(self, key, default=None):
        return math.inf


class GuardianNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.log.close()
            node.destroy_node()
        self.folder.cleanup()

    def path(self, name):
        return str(Path(self.folder.name) / name)

    def records(self, name):
        return [json.loads(l) for l in Path(self.path(name)).read_text().splitlines()]

    def station(self, scenario='guardian_intruder'):
        s = GuardianStation(
            SimpleNamespace(
                drones=','.join(g['ns'] for g in GUARDIANS),
                log=self.path('station.jsonl'),
                speed=0.0,
                max_east=14.0,
                scenario=scenario,
            )
        )
        self.nodes.append(s)
        s.carrier = {'e': 0.0, 'n': -1.5, 'yaw': 0.0, 'speed': 0.0}
        s.heard = Fresh()
        for g in GUARDIANS:
            s.states[g['ns']] = {
                'phase': 'watch',
                'position': [float(g['post'][0]), float(g['post'][1]), g['altitude']],
                'armed': True,
                'layer': g['altitude'],
                'pad_error': None,
            }
            s.flown.add(g['ns'])
            s.clear[g['ns']]['launch'] = True
        return s

    def fly(self, s, start, until, source='px4_1'):
        t = start
        while t <= until:
            p = intruder_at(t)
            if math.dist(p, s.states[source]['position']) < 8.0 or math.dist(p[:2], [0.0, -1.5]) < 6.0:
                s.on_detections(
                    message({'source': source, 'detections': [{'p': p, 'label': 'drone', 'sigma': 0.15, 't': t + 1.0}]})
                )
            s.protect(t + 1.0)
            t = round(t + 0.2, 6)

    def test_station_raises_red_relocates_and_orders_keep_clear_without_closing(self):
        s = self.station()
        self.fly(s, 0.0, 16.0)
        self.assertEqual(s.posture.state, 'RED')
        self.assertTrue(s.relocating)
        self.assertEqual(s.carrier_speed(20.0), RELOCATE['speed'])
        records = self.records('station.jsonl')
        kinds = {r['kind'] for r in records}
        self.assertLessEqual({'confirmed', 'posture', 'crew_alert', 'relocate', 'center_report', 'order'}, kinds)
        keep = [r for r in records if r['kind'] == 'order' and r['action'] == 'keep_clear']
        self.assertTrue(keep)
        for r in keep:
            self.assertTrue(safe_move(r['own'], r['target'], r['avoid'], r['fast'], CFG))  # Along the whole move.
        self.assertEqual(
            next(r for r in records if r['kind'] == 'relocate')['target_x'], 8.0
        )  # NORTH only: the nearest safe stop.

    def settle(self, s, now, limit=120.0):
        while s.posture.state != 'GREEN' and now < limit:
            now += 0.2
            s.protect(now)
        return now

    def test_recovery_waits_for_the_center_and_carries_its_authority(self):
        s = self.station()
        self.fly(s, 0.0, 40.0)
        now = self.settle(s, 41.0)
        s.protect(now + 5.0)
        self.assertTrue(s.held)
        self.assertIsNone(s.recovery)
        self.assertTrue(
            any(
                r['kind'] == 'center_report' and r['report'] == 'clear_after_red' for r in self.records('station.jsonl')
            )
        )
        self.assertTrue(all(o['action'] == 'hold' for o in s.orders.values()))
        s.on_decision(message({'decision': 'recover', 'request': s.request - 1}))
        s.protect(now + 5.5)
        self.assertIsNone(s.recovery)  # An answer to an older question does not count.
        s.on_decision(message({'decision': 'recover', 'request': s.request}))
        s.protect(now + 6.0)
        self.assertEqual((s.recovery, s.recovery_by), ('recover', 'center'))
        for o in s.orders.values():
            self.assertEqual((o['action'], o['authority']), ('recover', 'center'))

    def test_a_decision_to_an_earlier_clear_is_ignored_after_a_new_event(self):
        s = self.station()
        self.fly(s, 0.0, 40.0)
        now = self.settle(s, 41.0)
        s.protect(now + 1.0)
        first = s.request
        s.posture.update(now + 2.0, ['danger'])
        s.protect(now + 2.0)  # A new track goes RED before the answer arrives.
        self.assertIsNone(s.request)
        s.on_decision(message({'decision': 'recover', 'request': first}))
        self.assertIsNone(s.recovery)
        now = self.settle(s, now + 3.0)
        s.protect(now + 1.0)
        self.assertGreater(s.request, first)

    def test_station_lands_guardians_under_delegation_only_after_the_center_timeout(self):
        s = self.station()
        self.fly(s, 0.0, 40.0)
        self.settle(s, 41.0)
        cleared = s.clear_since
        self.assertIsNotNone(cleared)
        s.protect(cleared + CFG.center_timeout - 1.0)
        self.assertIsNone(s.recovery)
        s.protect(cleared + CFG.center_timeout + 0.5)
        self.assertEqual((s.recovery, s.recovery_by), ('recover', 'station (delegated)'))
        self.assertTrue(authorised('recover', 'station (delegated)'))

    def test_a_quiet_watch_ends_by_asking_the_center(self):
        s = self.station('guardian_birds')
        watch = SCENARIOS['guardian_birds']['watch']
        for k in range(int((watch + 6.0) / 0.5)):
            s.protect(10.0 + k * 0.5)
        self.assertIsNotNone(s.watch_request)
        self.assertTrue(
            any(r['kind'] == 'center_report' and r['report'] == 'watch_complete' for r in self.records('station.jsonl'))
        )
        s.on_decision(message({'decision': 'recover', 'request': s.request}))
        s.protect(10.0 + watch + 7.0)
        self.assertEqual((s.recovery, s.recovery_by), ('recover', 'center'))

    def test_birds_a_camera_has_labelled_leave_the_posture_green(self):
        s = self.station('guardian_birds')
        t = 10.0
        for k in range(40):  # A bird circling near px4_1, labelled by its camera.
            a = 0.3 * k * 0.2
            p = [-3.0 + 1.5 * math.cos(a), 8.0 + 1.5 * math.sin(a), 3.5]
            s.on_detections(
                message({'source': 'px4_1', 'detections': [{'p': p, 'label': 'bird', 'sigma': 0.15, 't': t}]})
            )
            s.protect(t)
            t = round(t + 0.2, 6)
        self.assertTrue(s.tracker.confirmed())
        self.assertEqual(s.posture.state, 'GREEN')

    def test_a_guardian_the_station_has_not_heard_from_is_told_to_hold_and_given_room(self):
        s = self.station()
        s.heard = {g['ns']: 0.0 for g in GUARDIANS}
        s.heard['px4_0'] = -10.0
        s.protect(0.5)
        self.assertEqual(s.orders['px4_0'], {'action': 'hold', 'target': None, 'reason': 'state_stale'})
        self.assertNotEqual(s.orders['px4_1'].get('reason'), 'state_stale')

    def test_relocation_moves_further_when_a_threat_comes_from_the_east(self):
        s = self.station()
        north = Track(1, 0.0, list(NORTH['start']), 's')
        north.v = [0.15, -1.19, 0.0]
        north.labels = {'drone': 5}
        self.assertEqual(s.relocation_stop(0.0, [north]), 8.0)
        east = Track(2, 0.0, list(EAST_HIGH['start']), 's')
        east.v = [-1.06, -0.56, 0.0]
        east.labels = {'drone': 5}
        self.assertEqual(s.relocation_stop(0.0, [north, east]), 11.0)

    def test_a_fast_object_disperses_the_guardian_near_its_impact_point_safely(self):
        s = self.station('guardian_fast')
        t = 0.0
        for k in range(14):
            p, _ = threat_position(FAST, k * 0.2)
            s.on_detections(
                message({'source': 'station', 'detections': [{'p': p, 'label': None, 'sigma': 0.1, 't': t}]})
            )
            s.protect(t)
            t = round(t + 0.2, 6)
        order = s.orders['px4_0']
        self.assertEqual(order['action'], 'disperse')
        record = [r for r in self.records('station.jsonl') if r['kind'] == 'order' and r['action'] == 'disperse'][-1]
        self.assertTrue(record['fast'])
        self.assertTrue(safe_move(record['own'], record['target'], record['avoid'], record['fast'], CFG))
        self.assertTrue(free_space(record['own'], record['target'], reserved=reserved_for('px4_0')))
        post = next(
            g['post'] for g in GUARDIANS if g['ns'] == 'px4_0'
        )  # Away from where it comes down, never toward it.
        self.assertGreater(math.dist(order['target'][:2], FAST['aim'][:2]), math.dist(post, FAST['aim'][:2]))
        # A conservative cell reported by the vehicle can block a route clear in the station's geometry.
        own = s.states['px4_0']['position']
        old_target = order['target']
        cell = tuple(round((a + b) / 2) for a, b in zip(own[:2], old_target[:2]))
        self.assertFalse(free_space(own, old_target, blocked={cell}))
        s.states['px4_0']['blocked_cells'] = [list(cell)]
        s.protect(t)
        updated = s.orders['px4_0']['target']
        self.assertNotEqual(updated, old_target)
        self.assertTrue(free_space(own, updated, blocked={cell}))

    def test_navigation_cross_check_flags_a_dragged_guardian_and_sends_it_fixes(self):
        s = self.station('guardian_spoofing')
        ns = 'px4_1'
        for k in range(10):  # The reported (GNSS) position drifts east of the station's own fix.
            s.states[ns]['position'] = [0.3 * k, 12.0, 5.0]
            s.on_fixes(message({ns: [0.0, 12.0, 5.0, float(k)]}))
        self.assertIn(ns, s.navigating)
        self.assertEqual(s.integrity[ns].state, 'spoofed')
        self.assertTrue(any(r['kind'] == 'integrity' and r['ns'] == ns for r in self.records('station.jsonl')))
        for o in (g['ns'] for g in GUARDIANS if g['ns'] != ns):
            self.assertNotIn(o, s.navigating)
        s.uplink = Mock()
        s.publish_clearance(10.0)
        sent = json.loads(s.uplink.publish.call_args.args[0].data)
        self.assertEqual((sent['nav_mode'][ns], sent['nav_mode']['px4_0']), ('station', 'gnss'))
        self.assertIn(ns, sent['nav'])
        for k in range(CFG.integrity_clear):  # The drag-off ends: GNSS agrees with the fixes again.
            s.states[ns]['position'] = [0.0, 12.0, 5.0]
            s.on_fixes(message({ns: [0.0, 12.0, 5.0, 20.0 + k]}))
        self.assertNotIn(ns, s.navigating)
        self.assertTrue(any(r['kind'] == 'integrity_clear' for r in self.records('station.jsonl')))

    def test_a_landed_guardian_on_station_fixes_keeps_getting_current_ones(self):
        # Landed and disarmed while still dragged: the cross-check stops, but the fixes stay current, so the
        # guardian's corrected position, and the pad error the station's touchdown depends on, follow the truth.
        s = self.station('guardian_spoofing')
        ns = 'px4_1'
        for k in range(10):
            s.states[ns]['position'] = [0.3 * k, 12.0, 5.0]
            s.on_fixes(message({ns: [0.0, 12.0, 5.0, float(k)]}))
        self.assertIn(ns, s.navigating)
        s.states[ns].update(armed=False, phase='descend', position=[5.0, -1.5, 0.6])
        s.on_fixes(message({ns: [0.0, -1.5, 0.6, 15.0]}))
        self.assertEqual(s.fix[ns], [0.0, -1.5, 0.6, 15.0])
        self.assertEqual(s.integrity[ns].state, 'spoofed')  # No cross-check on the deck.
        s.uplink = Mock()
        s.publish_clearance(15.0)
        self.assertEqual(json.loads(s.uplink.publish.call_args.args[0].data)['nav'][ns], [0.0, -1.5, 0.6, 15.0])

    def test_malformed_messages_are_dropped_and_logged_never_fatal(self):
        def raw(text):
            m = String()
            m.data = text
            return m

        s = self.station()
        tracks = len(s.tracker.tracks)
        states = json.dumps(s.states, sort_keys=True)
        bad_detections = (
            'not json',
            '[]',
            '{"detections": 5}',
            '{"source": "px4_1", "detections": [{"p": [1, 2], "t": 1}]}',
            '{"source": "px4_1", "detections": [{"p": [1, 2, "x"], "t": 1}]}',
            '{"source": ["px4_1"], "detections": []}',
            '{"source": "px4_1", "detections": [{"p": [1, 2, 3], "t": 1, "sigma": 0}]}',  # A zero sigma divides.
        )
        for text in bad_detections:
            self.assertFalse(s.on_detections(raw(text)))
        self.assertFalse(s.on_fixes(raw('[1, 2]')))
        self.assertFalse(s.on_fixes(raw('{"px4_1": [0, 12, 5]}')))
        self.assertFalse(s.on_decision(raw('{}')))
        for text in (
            '{"telemetry_fresh": true, "t": 1, "armed": true, "phase": "watch", "position": [0, 12]}',
            '{"telemetry_fresh": true, "t": 1, "armed": "yes", "phase": "watch", "position": [0, 12, 5]}',
            '{"telemetry_fresh": true, "t": 1, "armed": true, "phase": "watch", "position": [0, 12, 5], "layer": 6, '
            '"px4_stamp": "late"}',
        ):
            self.assertFalse(s.on_state('px4_1', raw(text)))
        self.assertEqual(len(s.tracker.tracks), tracks)
        self.assertEqual(json.dumps(s.states, sort_keys=True), states)
        dropped = [r for r in self.records('station.jsonl') if r['kind'] == 'dropped_message']
        self.assertEqual(len(dropped), len(bad_detections) + 6)
        self.assertTrue(all(r['error'] and r['topic'] for r in dropped))
        s.protect(1.0)
        s.tick()  # Still running, with the state it had.
        c = CenterNode(SimpleNamespace(log=self.path('center.jsonl'), unreachable=False))
        self.nodes.append(c)
        # A non-finite request would be queued, then stop the node when its answer is logged.
        for text in ('[1]', '{"state": "RED"}', 'nope', '{"kind": "clear_after_red", "request": NaN}'):
            self.assertFalse(c.report(raw(text)))
        self.assertEqual(c.center.inbox, [])
        c.tick()
        self.assertEqual([r['kind'] for r in self.records('center.jsonl')], ['dropped_message'] * 4)
        w = self.world()
        self.assertFalse(w.state('px4_0', raw('[]')))
        self.assertNotIn('px4_0', w.phase)

    def test_center_logs_reports_and_answers_after_its_delays(self):
        c = CenterNode(SimpleNamespace(log=self.path('center.jsonl'), unreachable=False))
        self.nodes.append(c)
        c.report(message({'kind': 'clear_after_red'}))
        c.report(message({'kind': 'watch_complete'}))
        self.assertEqual([r['report'] for r in self.records('center.jsonl')], ['clear_after_red', 'watch_complete'])
        now = c.now()
        self.assertEqual(c.center.step(now), [])
        self.assertEqual(sorted(d for _, d, _ in c.center.step(now + 10.0)), ['recover', 'recover'])

    def test_an_unreachable_center_hears_nothing_and_decides_nothing(self):
        c = CenterNode(SimpleNamespace(log=self.path('center.jsonl'), unreachable=True))
        self.nodes.append(c)
        c.report(message({'kind': 'clear_after_red'}))
        self.assertEqual([r['kind'] for r in self.records('center.jsonl')], ['unreachable'])
        self.assertEqual(c.center.step(c.now() + 100.0), [])

    def world(self, scenario='guardian_jamming'):
        w = World(
            SimpleNamespace(
                log=self.path('world.jsonl'), seed=1, scenario=scenario, world='guardian', origin=[47.397971, 8.546164]
            )
        )
        self.nodes.append(w)
        return w

    def test_world_drops_links_only_inside_the_jamming_zone(self):
        w = self.world()
        w.uplink = {g['ns']: Mock() for g in GUARDIANS}
        for i, p in enumerate(([1.0, 2.0, 4.0], [0.0, 12.0, 5.0], [12.0, 8.0, 3.0])):
            w.truth[f'x500_{i}'] = {'p': p, 'v': [0.0, 0.0, 0.0]}
        w.jamming = True
        w.station_uplink(message({'orders': {}}))
        self.assertEqual(
            {ns: m.publish.called for ns, m in w.uplink.items()}, {'px4_0': False, 'px4_1': True, 'px4_2': True}
        )
        self.assertEqual(w.dropped['px4_0'], 1)
        w.jamming = False
        w.station_uplink(message({'orders': {}}))
        self.assertTrue(w.uplink['px4_0'].publish.called)
        w.started = w.now() - 1.0
        w.truth['intruder'] = {'p': [0.0, 6.0, 4.0], 'v': [0.0, 0.0, 0.0]}
        self.assertTrue(
            any(
                w.detect({'range': 8.0, 'pd': 0.9, 'sigma': 0.15}, [0.0, 12.0, 5.0], 'intruder', NORTH)
                for _ in range(5)
            )
        )
        self.assertIsNone(w.detect({'range': 8.0, 'pd': 1.0, 'sigma': 0.15}, [0.0, 30.0, 5.0], 'intruder', NORTH))

    def test_station_sensor_loses_low_flyers_beyond_its_short_range(self):
        w = self.world()
        w.started = w.now() - 1.0
        sensor = {'range': 20.0, 'low_altitude': 6.0, 'low_range': 6.0, 'pd': 1.0, 'sigma': 0.1}
        w.truth['intruder'] = {'p': [0.0, 10.0, 4.0], 'v': [0.0, 0.0, 0.0]}
        self.assertIsNone(w.detect(sensor, [0.0, -1.5, 0.3], 'intruder', NORTH))  # 11.5 m away and low.
        w.truth['intruder'] = {'p': [0.0, 10.0, 7.0], 'v': [0.0, 0.0, 0.0]}
        self.assertIsNotNone(w.detect(sensor, [0.0, -1.5, 0.3], 'intruder', NORTH))  # High flyers are seen to 20 m.

    def test_scripts_follow_their_lines_and_the_fast_object_ends_below_ground(self):
        self.assertEqual(threat_position(NORTH, 0.0)[0], list(NORTH['start']))
        end, done = threat_position(FAST, impact_time(FAST) + 10.0)
        self.assertTrue(done)
        self.assertLess(end[2], 0.0)
        at_impact, _ = threat_position(FAST, impact_time(FAST))
        self.assertLess(math.dist(at_impact, FAST['aim']), 1e-6)
        for name in SCENARIOS:
            world = world_sdf(name)
            for model in SCENARIOS[name]['threats']:
                self.assertIn(f'<model name="{model}">', world)
            self.assertEqual(len(expected_checks(name)), len(set(expected_checks(name))))

    def guardian(self, order, link_age, now=100.0):
        g = SimpleNamespace(
            ns='px4_0',
            post=[1.0, 2.0, 4.0],
            altitude=4.0,
            onboard=Tracker(CFG),
            order=order,
            last_uplink=now - link_age,
            carrier=(0.0, -1.5, 0.0, 0.0, 0.0),
            blocked=set(reserved_for('px4_0')),
            reflex=None,
            action='watch',
            layer='station',
            hold_at=None,
            guard_target=None,
            log=Mock(),
            start_return=Mock(return_value=[1.0, -1.0, 4.0]),
            nav_offset=None,
            nav_fix=None,
            nav_fix_at=None,
            rally=[0.0, -1.5, 4.0],
            rally_route=None,
            send_command=Mock(),
            handed_off=False,
            land_requested=False,
            clearance={},
            posture=None,
            track=[],
            now=lambda: now,
            fleet_now=lambda: now,
            clearance_input=FreshInput(),
        )
        for name in ('guard', 'log_decision', 'navigation', 'station_fix', 'on_clearance'):
            setattr(g, name, MethodType(getattr(GuardianMission, name), g))
        g.handoff_land = MethodType(Mission.handoff_land, g)
        g.accept_clearance = MethodType(FleetMission.accept_clearance, g)
        return g

    def feed_track(self, g, now):
        for k in range(12):  # An intruder closing on the post from the north at 1.2 m/s.
            t = now - 2.4 + k * 0.2
            g.onboard.update(t, [([0.6, 2.0 + 1.2 * (now + 3.0 - t), 4.0], 'px4_0', 'drone', 0.15)])

    def test_jammed_guardian_keeps_clear_on_its_own_without_closing(self):
        g = self.guardian({'action': 'watch', 'target': [1.0, 2.0, 4.0]}, link_age=5.0)
        self.feed_track(g, 100.0)
        target = g.guard(100.0, [1.0, 2.0, 4.0])
        self.assertEqual((g.action, g.layer), ('keep_clear', 'onboard'))
        logged = g.log.write.call_args.kwargs
        self.assertTrue(logged['avoid'])
        self.assertTrue(safe_move([1.0, 2.0, 4.0], target, logged['avoid'], logged['fast'], CFG))

    def test_guardian_follows_station_orders_and_center_recovery(self):
        g = self.guardian({'action': 'keep_clear', 'target': [4.0, 0.5, 4.0], 'until': 200.0}, link_age=0.1)
        self.assertEqual(g.guard(100.0, [1.0, 2.0, 4.0]), [4.0, 0.5, 4.0])
        self.assertEqual(g.layer, 'station')
        g = self.guardian({'action': 'recover', 'target': None, 'authority': 'center'}, link_age=0.1)
        self.assertEqual(g.guard(100.0, [1.0, 2.0, 4.0]), [1.0, -1.0, 4.0])
        g.start_return.assert_called_once()
        self.assertEqual(g.log.write.call_args.kwargs['layer'], 'center')
        g = self.guardian({'action': 'watch', 'target': [1.0, 2.0, 4.0]}, link_age=CFG.lost_link + 1)
        g.guard(100.0, [1.2, 2.0, 4.0])
        self.assertEqual((g.action, g.layer), ('lost_link_hold', 'onboard'))

    def test_each_guardian_routes_around_the_other_posts_but_can_reach_its_own(self):
        from world import astar

        for g in GUARDIANS:
            me = SimpleNamespace(ns=g['ns'], post=[float(g['post'][0]), float(g['post'][1]), g['altitude']])
            reserved = GuardianMission.reserved_cells(me)
            post = tuple(g['post'])
            self.assertNotIn(post, reserved)
            for other in GUARDIANS:
                if other is not g:
                    self.assertIn(tuple(other['post']), reserved)
            route = astar((round(g['pad']), -2), post, reserved)  # From its pad on the carrier's deck.
            self.assertTrue(route)
            self.assertFalse(set(route) & reserved)

    def test_a_guardian_state_keeps_its_altitude_layer_for_the_landing_order(self):
        # The state once carried the deciding layer ('station', 'onboard') under 'layer', the key in which the
        # fleet state gives the altitude layer the station lands vehicles by, so guardians landed in no set order.
        g = self.guardian({'action': 'watch', 'target': [1.0, 2.0, 4.0]}, link_age=0.1)
        g.action, g.layer, g.origin, g.pose = 'keep_clear', 'onboard', None, None
        extra = GuardianMission.extra_state(g)
        self.assertNotIn('layer', extra)
        self.assertEqual((extra['action'], extra['decided_by']), ('keep_clear', 'onboard'))
        s = self.station()
        before = dict(s.states['px4_1'])
        s.on_state('px4_1', message({**before, 'telemetry_fresh': True, 'layer': None}))  # Dropped, not fatal.
        self.assertEqual(s.states['px4_1'], before)
        dropped = [r['topic'] for r in self.records('station.jsonl') if r['kind'] == 'dropped_message']
        self.assertEqual(dropped, ['fleet state'])

    def test_a_dispersal_order_carries_its_impact_area_and_a_malformed_one_is_dropped(self):
        g = self.guardian({'action': 'watch', 'target': [1.0, 2.0, 4.0]}, link_age=0.1)
        order = {
            'action': 'disperse',
            'target': [0.8, 4.1, 4.0],
            'reason': 'impact_area',
            'hazard': [[0.8, -3.1, 0.0], 6.0],
        }
        uplink = lambda o: message({'t': 100.0, 'clearance': {}, 'orders': {'px4_0': o}})
        for hazard in ([[0.8, -3.1], 6.0], [[0.8, -3.1, 0.0], 'far'], [[0.8, -3.1, 0.0]]):
            self.assertFalse(g.on_clearance(uplink(dict(order, hazard=hazard))))
        self.assertFalse(g.on_clearance(uplink(dict(order, target=[0.8, 4.1]))))  # Flown later, on the timer.
        nav = message({'t': 100.0, 'clearance': {}, 'orders': {'px4_0': order}, 'nav': {'px4_0': [0.0, 1.0, 4.0]}})
        self.assertFalse(g.on_clearance(nav))  # A station fix is x, y, z and its time.
        self.assertEqual(g.order['action'], 'watch')
        self.assertEqual([c.args[0] for c in g.log.write.call_args_list], ['dropped_message'] * 5)
        g.on_clearance(uplink(order))  # Its timestamp was not used up by the dropped messages.
        self.assertEqual(g.order['hazard'], order['hazard'])
        self.assertEqual(g.guard(100.0, [1.0, 2.0, 4.0]), order['target'])  # Nothing seen: the station's move stands.
        self.assertEqual((g.action, g.layer), ('disperse', 'station'))

    def test_guardian_navigates_by_station_fixes_once_its_gnss_is_flagged(self):
        g = self.guardian({'action': 'watch', 'target': [1.0, 2.0, 4.0]}, link_age=0.1)
        g.track = [(t, [1.0 + 0.1 * t, 2.0, 4.0]) for t in range(10)]  # The GNSS-based estimate has been dragged east.
        g.station_fix([1.0, 2.0, 4.0, 9.0])
        self.assertAlmostEqual(g.navigation(9.0, [1.9, 2.0, 4.0])[0], 1.0, places=6)
        self.assertEqual(g.log.write.call_args.kwargs['action'], 'navigate_by_station')
        self.assertTrue(authorised('navigate_by_station', 'station'))
        g.on_clearance(
            message({'t': 100.0, 'clearance': {}, 'orders': {}, 'nav_mode': {'px4_0': 'gnss'}})
        )  # The cross-check cleared.
        self.assertIsNone(g.nav_offset)
        self.assertEqual(g.navigation(9.0, [1.9, 2.0, 4.0]), [1.9, 2.0, 4.0])

    def test_a_guardian_on_station_fixes_lands_in_place_when_they_stop(self):
        g = self.guardian({'action': 'watch', 'target': [1.0, 2.0, 4.0]}, link_age=0.1)
        g.nav_offset = [0.5, 0.0]
        g.nav_fix_at = 100.0 - CFG.fix_timeout - 1.0
        self.assertEqual(g.guard(100.0, [1.0, 2.0, 4.0]), [1.0, 2.0, 4.0])
        self.assertTrue(g.handed_off and g.land_requested)
        g.send_command.assert_called_once()
        self.assertTrue(any(c.kwargs.get('action') == 'land_in_place' for c in g.log.write.call_args_list))

    def test_a_lost_link_return_follows_a_planned_route_to_the_last_known_carrier(self):
        from world import OBSTACLES, occupancy

        g = self.guardian({'action': 'watch', 'target': [12.0, 8.0, 3.0]}, link_age=CFG.lost_link_return + 1)
        g.ns = 'px4_2'
        g.post = [12.0, 8.0, 3.0]
        g.altitude = 3.0
        g.rally = [0.0, -1.5, 3.0]
        g.blocked = occupancy(
            [
                {'hit': True, 'x': x + r * math.cos(a / 8 * math.pi), 'y': y + r * math.sin(a / 8 * math.pi)}
                for x, y, r in OBSTACLES
                for a in range(16)
            ],
            blocked=set(reserved_for('px4_2')),
        )
        first = g.guard(100.0, [12.0, 8.0, 3.0])
        self.assertEqual(g.action, 'lost_link_return')
        route = [tuple(round(c) for c in p[:2]) for p in [first] + g.rally_route]
        self.assertFalse(set(route) & g.blocked)  # Never across a cylinder or another post, unlike a straight line.
        self.assertEqual(route[-1], (0, -2))


if __name__ == '__main__':
    unittest.main()
