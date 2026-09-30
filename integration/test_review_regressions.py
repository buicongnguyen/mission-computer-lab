"""Cross-phase and stream-loss regressions for review R1-R5 and R8; no flight processes."""

import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from fleet_contracts import FreshInput
from fleet_mission_node import FleetMission
from fleet_station_node import Station
from guardian_mission_node import GuardianMission
from guardian_layout import CFG, free_space
from guardian import Tracker, Track
from px4_msgs.msg import VehicleStatus


class ReviewContracts(unittest.TestCase):
    def vehicle(self, phase='watch'):
        m = GuardianMission.__new__(GuardianMission)
        fields = dict(
            ns='px4_0',
            phase=phase,
            phase_since=0.0,
            pad=0.0,
            carrier=(0.0, -1.0, 0.0, 0.3, 0.0),
            status=SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED),
            altitude=4.0,
            post=[1.0, 2.0, 4.0],
            log=Mock(),
            onboard=Tracker(CFG),
            last_uplink=100.0,
            nav_offset=None,
            nav_fix_at=None,
            reflex=None,
            rally=None,
            rally_route=None,
            order={'action': 'watch'},
            action=None,
            layer=None,
            hold_at=None,
            guard_target=None,
            targets=[[9.0, 9.0, 4.0]],
            goal=(9, 9),
            waypoint=0,
            blocked=set(),
            clearance={'launch': True, 'land': True},
            handed_off=False,
            land_requested=False,
            send_command=Mock(),
            now=lambda: 100.0,
            fleet_now=lambda: 100.0,
            carrier_input=FreshInput(),
            clearance_input=FreshInput(),
            fleet_hold=None,
        )
        for k, v in fields.items():
            setattr(m, k, v)
        m.carrier_input.accept(100.0, 100.0)
        m.clearance_input.accept(100.0, 100.0)
        return m

    def test_expired_station_fixes_handoff_in_every_airborne_phase(self):
        for phase in ('preflight', 'outbound', 'watch', 'return', 'rendezvous', 'descend'):
            with self.subTest(phase=phase):
                m = self.vehicle(phase)
                m.nav_offset = [1.0, 1.0]
                m.nav_fix_at = 0.0
                m.next_target(100.0, [1.0, 2.0, 4.0])
                self.assertTrue(m.handed_off and m.land_requested)
                m.send_command.assert_called_once()

    def test_lost_uplink_holds_during_transit_and_approach(self):
        for phase in ('outbound', 'return', 'rendezvous', 'descend'):
            with self.subTest(phase=phase):
                m = self.vehicle(phase)
                m.last_uplink = 90.0
                self.assertEqual(m.next_target(100.0, [1.0, 2.0, 4.0])[0], [1.0, 2.0, 4.0])
                self.assertEqual(m.action, 'lost_link_hold')
                self.assertFalse(m.handed_off)

    def test_onboard_reflex_remains_active_during_return(self):
        m = self.vehicle('return')
        tr = Track(1, 100.0, [1.0, 5.0, 4.0], 'onboard', 'drone')
        tr.v = [0.0, -1.0, 0.0]
        tr.n = 10
        tr.confirmed = True
        m.onboard.tracks = [tr]
        m.next_target(100.0, [1.0, 2.0, 4.0])
        self.assertEqual((m.action, m.layer), ('keep_clear', 'onboard'))

    def test_no_route_never_becomes_a_direct_return_leg(self):
        m = self.vehicle('watch')
        m.blocked = {(x, 4) for x in range(-2, 13)}
        self.assertEqual(m.start_return(100.0, [9.0, 9.0, 4.0]), [9.0, 9.0, 4.0])
        self.assertTrue(m.handed_off and m.land_requested)
        self.assertNotEqual(m.phase, 'return')

    def test_stale_pose_or_clearance_blocks_launch_and_aborts_descent(self):
        for stream in ('carrier_input', 'clearance_input'):
            for phase in ('rendezvous', 'descend'):
                with self.subTest(stream=stream, phase=phase):
                    m = self.vehicle(phase)
                    getattr(m, stream).stamp = 90.0
                    self.assertFalse(m.may_request_flight())
                    target, _ = FleetMission.next_target(m, 100.0, [1.0, 2.0, 2.0])
                    self.assertEqual(target, [1.0, 2.0, 4.0])
                    self.assertEqual(m.phase, 'rendezvous')

    def test_freshness_rejects_replay_future_old_and_nonfinite_capture_times(self):
        f = FreshInput()
        self.assertTrue(f.accept(10.0, 10.0))
        for stamp in (10.0, 9.0, 20.0, float('nan'), float('inf'), True, None):
            self.assertFalse(f.accept(stamp, 10.5))
        self.assertTrue(f.fresh(10.5))
        self.assertFalse(f.fresh(12.0))
        self.assertFalse(f.fresh(9.0))
        self.assertFalse(f.accept(10.1, 12.0))
        self.assertTrue(f.accept(12.0, 12.0))

    def test_fresh_revocation_of_landing_clearance_stops_descent(self):
        m = self.vehicle('descend')
        m.clearance['land'] = False
        self.assertEqual(FleetMission.next_target(m, 100.0, [1.0, 2.0, 2.0])[0], [1.0, 2.0, 4.0])
        self.assertEqual(m.phase, 'rendezvous')

    def test_deck_descent_continues_with_an_estimate_below_deck_height(self):
        m = self.vehicle('descend')
        target, _ = FleetMission.next_target(m, 100.0, [0.0, -1.0, 0.38])
        self.assertEqual(target[2], 0.0)
        self.assertLessEqual(target[2] - 0.38, -0.3)

    def test_segment_keeps_clear_of_friends_and_uncertain_regions(self):
        start, end, other = [-1.0, -1.0, 4.0], [3.0, -1.0, 4.0], [1.0, -1.0, 4.0]
        self.assertFalse(free_space(start, end, others=[other]))
        self.assertFalse(free_space(start, end, wide=[(other, 1.5)]))
        self.assertTrue(free_space(start, end, others=[[1.0, -3.0, 4.0]]))  # Tangent at the 2 m margin.
        self.assertTrue(free_space(start, end, others=[[1.0, -4.0, 4.0]]))

    def station(self):
        s = SimpleNamespace(
            states={},
            flown=set(),
            landed=set(),
            airborne_at={},
            airborne_px4={},
            land_reports={},
            log=Mock(),
            carrier={'e': 0.0, 'n': 0.0, 'speed': 0.3},
            clock=10.0,
            carrier_input=FreshInput(),
            state_inputs={'px4_0': FreshInput()},
            clear={'px4_0': {'launch': True, 'land': True}},
        )
        s.now = lambda: s.clock
        s.carrier_input.accept(10.0, 10.0)
        return s

    def report(self, s, armed, z):
        return Station.on_state(
            s,
            'px4_0',
            SimpleNamespace(
                data=json.dumps(
                    dict(
                        t=s.clock,
                        telemetry_fresh=True,
                        px4_stamp=int(s.clock * 1e6),
                        armed=armed,
                        position=[0.0, 0.0, z],
                        phase='descend',
                        pad_error=0.1,
                    )
                )
            ),
        )

    def test_disarm_without_fresh_independent_contact_does_not_release_landing(self):
        for case in ('missing', 'stale', 'replayed', 'delayed_capture', 'preflight_capture', 'no_contact', 'valid'):
            with self.subTest(case=case):
                s = self.station()
                self.report(s, True, 3.0)
                s.clock = 10.5
                land = SimpleNamespace(
                    timestamp=9000000 if case == 'preflight_capture' else 10500000,
                    landed=True,
                    ground_contact=case != 'no_contact',
                )
                if case != 'missing':
                    Station.on_land(s, 'px4_0', land)
                if case in ('stale', 'replayed', 'delayed_capture'):
                    s.clock = 14.0
                    s.carrier_input.accept(14.0, 14.0)
                    if case == 'replayed':
                        Station.on_land(s, 'px4_0', land)
                    if case == 'delayed_capture':
                        land.timestamp += 100000
                        Station.on_land(s, 'px4_0', land)
                else:
                    s.clock = 10.6
                self.report(s, False, 0.8)
                self.assertEqual(bool(s.landed), case == 'valid')

    def test_stale_vehicle_state_masks_clearances_and_replay_cannot_refresh_it(self):
        s = self.station()
        self.report(s, True, 3.0)
        s.clock = 12.0
        s.carrier_input.accept(12.0, 12.0)
        stale = SimpleNamespace(data=json.dumps(dict(t=10.0, telemetry_fresh=True, armed=True, position=[0, 0, 3])))
        self.assertFalse(Station.on_state(s, 'px4_0', stale))
        self.assertEqual(Station.active_clearance(s, 12.0)['px4_0'], {'launch': False, 'land': False})


if __name__ == '__main__':
    unittest.main()
