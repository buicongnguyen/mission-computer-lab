"""Evidence completeness and directory replacement regressions (review R5-R7)."""

from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import publish_sitl as P
from evidence_contracts import fleet_landing_confirmed


class PublicationContracts(unittest.TestCase):
    def result(self, scenario='fleet_carrier'):
        r = dict(
            scenario=scenario,
            vehicles=[dict(ns=ns, trace=[{'wall_time': 0.0}], truth=[{'t': 0.0}]) for ns in P.FLEET_NAMES],
            carrier=[{'wall_time': 0.0}],
            min_separation_m=2.0,
            scenario_start={'t': 0.0},
            threats={'intruder': {'trace': [{'t': 0.0}]}},
        )
        r['checks'] = {k: True for k in P.required_checks(r)}
        return r

    def test_vehicle_ids_are_exact_and_unique(self):
        r = self.result()
        P.validate_vehicle_ids(r)
        for names in (
            ['px4_0', 'px4_0'],
            ['px4_0', 'px4_1'],
            ['px4_0', 'px4_1', 'other'],
            ['px4_0', 'px4_1', 'px4_2', 'px4_2'],
        ):
            for scenario in ('fleet_carrier', 'guardian_intruder'):
                with self.subTest(names=names, scenario=scenario):
                    bad = deepcopy(r)
                    bad['scenario'] = scenario
                    bad['vehicles'] = [dict(ns=n) for n in names]
                    with self.assertRaises(ValueError):
                        P.validate_vehicle_ids(bad)

    def test_guardian_threats_must_match_the_configured_scenario(self):
        r = self.result('guardian_intruder')
        P.validate_guardian(r)
        for threats in (
            {},
            {'other': {'trace': [{'t': 0.0}]}},
            dict(r['threats'], extra={'trace': [{'t': 0.0}]}),
            {'intruder': {'trace': []}},
        ):
            with self.subTest(threats=threats):
                with self.assertRaises(ValueError):
                    P.validate_guardian(dict(r, threats=threats))
        r['vehicles'][0]['truth'] = []
        with self.assertRaises(ValueError):
            P.validate_guardian(r)

    def test_full_publisher_rejects_duplicate_vehicles_before_replacing_existing_sample(self):
        root = Path(__file__).resolve().parents[1]
        names = ['fleet_carrier', 'nominal', 'camera_dropout', 'companion_crash', 'gps_loss', *P.GUARDIANS]
        results = [json.loads((root / 'artifacts/sitl-sample' / n / 'result.json').read_text()) for n in names]
        results[0]['vehicles'] = [results[0]['vehicles'][0]] * 2
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            source = base / 'input'
            (source / 'fleet_carrier').mkdir(parents=True)
            (source / 'results.json').write_text(json.dumps(results))
            (source / 'fleet_carrier/result.json').write_text(json.dumps(results[0]))
            (source / 'provenance.json').write_text('{"inputs_unchanged":true}')
            sample = base / 'artifacts/sitl-sample'
            sample.mkdir(parents=True)
            (sample / 'old').write_text('retained')
            with patch.object(P, 'ROOT', base), patch.object(sys, 'argv', ['publish_sitl', '--input', str(source)]):
                with self.assertRaisesRegex(ValueError, 'configured vehicle'):
                    P.main()
            self.assertEqual((sample / 'old').read_text(), 'retained')

    def test_failed_promotion_restores_old_sample_and_retry_succeeds(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            sample = base / 'sitl-sample'
            stage = base / 'stage'
            sample.mkdir()
            stage.mkdir()
            (sample / 'old').write_text('old')
            (stage / 'new').write_text('new')
            rename = Path.rename

            def fail(path, dest):
                if path == stage:
                    raise PermissionError('promotion failed')
                return rename(path, dest)

            with patch.object(Path, 'rename', fail):
                with self.assertRaises(PermissionError):
                    P.replace_sample(stage, sample)
            self.assertEqual((sample / 'old').read_text(), 'old')
            self.assertTrue((stage / 'new').exists())
            P.replace_sample(stage, sample)
            self.assertTrue((sample / 'new').exists())
            self.assertFalse((sample / 'old').exists())

    def test_interrupted_promotion_is_recovered_and_cleanup_failure_keeps_current_sample(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            sample = base / 'sitl-sample'
            retired = base / '.sitl-sample-retired'
            stage = base / 'stage'
            retired.mkdir()
            (retired / 'old').write_text('old')
            stage.mkdir()
            (stage / 'new').write_text('new')
            P.recover_sample(sample)
            self.assertEqual((sample / 'old').read_text(), 'old')
            with patch.object(P.shutil, 'rmtree', side_effect=PermissionError('cleanup failed')):
                with self.assertRaises(PermissionError):
                    P.replace_sample(stage, sample)
            self.assertTrue((sample / 'new').exists())
            self.assertTrue((retired / 'old').exists())
            P.recover_sample(sample)
            self.assertTrue((sample / 'new').exists())

    def test_fleet_landing_requires_independent_contact_after_airborne_state(self):
        records = [
            dict(kind='status', wall_time=1.0, arming_state=2, nav_state=14),
            dict(kind='position', wall_time=2.0, valid=True, ned=[0.0, 0.0, -3.0]),
            dict(kind='status', wall_time=10.0, arming_state=1, nav_state=14),
        ]
        touchdown = {'wall_time': 10.1}
        self.assertFalse(fleet_landing_confirmed(records, touchdown))
        for t in (0.0, 3.0, 20.0):
            self.assertFalse(
                fleet_landing_confirmed(
                    records + [dict(kind='land', wall_time=t, landed=True, ground_contact=True)], touchdown
                )
            )
        landed = dict(kind='land', wall_time=9.9, landed=True, ground_contact=True)
        self.assertTrue(fleet_landing_confirmed(records + [landed], touchdown))
        self.assertFalse(fleet_landing_confirmed(records + [dict(landed, ground_contact=False)], touchdown))

    def test_a_landing_is_timed_from_the_disarm_not_from_later_status_changes(self):
        # guardian_jamming, 30 September: PX4 disarmed, then logged its preflight checks failing 2.8 s later, once the
        # offboard stream had stopped. Timing from that last status rejected two good landings.
        records = [
            dict(kind='status', wall_time=1.0, arming_state=2, nav_state=14),
            dict(kind='position', wall_time=50.0, valid=True, ned=[0.0, 0.0, -3.0]),
            dict(kind='land', wall_time=59.92, landed=True, ground_contact=True),
            dict(kind='status', wall_time=60.0, arming_state=1, nav_state=14),
            dict(kind='status', wall_time=62.78, arming_state=1, nav_state=14, preflight=False),
        ]
        self.assertTrue(fleet_landing_confirmed(records, {'wall_time': 60.94}))
        # The fleet flight the same day: the station declared touchdown 3.1 s after the disarm, at 0.4 of real time.
        self.assertTrue(fleet_landing_confirmed(records, {'wall_time': 63.1}))
        self.assertFalse(fleet_landing_confirmed(records, {'wall_time': 66.0}))  # Not this landing.
        # A landed report from before the vehicle was last airborne is not evidence of the final landing.
        early = records[:2] + [
            dict(kind='land', wall_time=57.5, landed=True, ground_contact=True),
            dict(kind='position', wall_time=58.5, valid=True, ned=[0.0, 0.0, -2.0]),
        ]
        self.assertFalse(fleet_landing_confirmed(early + records[3:], {'wall_time': 60.94}))


if __name__ == '__main__':
    unittest.main()
