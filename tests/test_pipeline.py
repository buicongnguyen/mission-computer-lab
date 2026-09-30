import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from perception import Detector, camera_frame, create_model
from security import boot_gate, load_public_key, provision, public_key_hex, sign, verify, security_experiments
from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from world import Localizer, astar, lidar, occupancy, GOAL, OBSTACLES
import run_demo

ROOT = Path(__file__).resolve().parents[1]


class SecurityTests(unittest.TestCase):
    def test_signature_tampering_rollback_and_identity(self):
        result = security_experiments(b'model artifact')
        self.assertEqual(len(result), 6)
        self.assertTrue(all(r['passed'] for r in result))

    def test_version_schema_rejects_boolean(self):
        key = Ed25519PrivateKey.generate()
        manifest, sig = sign(b'payload', key, version=True)
        with self.assertRaises(ValueError):
            verify(b'payload', manifest, sig, key.public_key())

    def test_current_version_is_accepted(self):
        key = Ed25519PrivateKey.generate()
        manifest, sig = sign(b'p', key, version=2)
        self.assertTrue(verify(b'p', manifest, sig, key.public_key(), minimum_version=2))


class BootGateTests(unittest.TestCase):
    """The gate must check the stored artifact, not re-sign whatever bytes it is given."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.model = Path(self.tmp.name) / 'model.onnx'
        create_model(self.model)
        self.key = Ed25519PrivateKey.generate()
        provision(self.model, self.key)
        self.anchor = Path(self.tmp.name) / 'trust-anchor.pub'
        self.anchor.write_text(public_key_hex(self.key))

    def tearDown(self):
        self.tmp.cleanup()

    def test_verified_bytes_are_the_loaded_model(self):
        payload, stages = boot_gate(self.model, load_public_key(self.anchor))
        self.assertEqual(payload, self.model.read_bytes())
        self.assertEqual(stages[2]['status'], 'VERIFIED IN USER SPACE')
        self.assertEqual(Detector(payload).infer(camera_frame(0))['bbox'], [27, 26, 36, 34])

    def test_tampered_stored_artifact_is_rejected(self):
        self.model.write_bytes(self.model.read_bytes() + b'x')
        with self.assertRaisesRegex(ValueError, 'payload_tampered'):
            boot_gate(self.model, self.key.public_key())

    def test_other_trust_anchor_is_rejected(self):
        with self.assertRaises(InvalidSignature):
            boot_gate(self.model, Ed25519PrivateKey.generate().public_key())

    def test_unsigned_artifact_is_rejected(self):
        self.model.with_name('model.onnx.sig').unlink()
        with self.assertRaises(FileNotFoundError):
            boot_gate(self.model, self.key.public_key())


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        path = Path(self.tmp.name) / 'model.onnx'
        create_model(path)
        self.detector = Detector(path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_synthetic_object_has_correct_bbox(self):
        result = self.detector.infer(camera_frame(0))
        self.assertEqual(result['bbox'], [27, 26, 36, 34])
        self.assertEqual(result['pixels'], 48)

    def test_background_has_no_detection(self):
        self.assertIsNone(self.detector.infer(camera_frame(0, False))['bbox'])

    def test_cpu_provider_is_explicit(self):
        self.assertEqual(self.detector.session.get_providers(), ['CPUExecutionProvider'])


class PlanningTests(unittest.TestCase):
    def test_lidar_range(self):
        rays = lidar([0, 0, 0], [(4, 0, 1)], rays=4, max_range=10)
        self.assertAlmostEqual(rays[0]['range'], 3)
        self.assertFalse(rays[1]['hit'])

    def test_astar_avoids_blocked_cells(self):
        blocked = occupancy(lidar([0, 0, 0]))
        path = astar((0, 0), GOAL, blocked)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], GOAL)
        self.assertFalse(any(p in blocked for p in path))
        self.assertTrue(all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(path, path[1:])))

    def test_gazebo_world_matches_planner_and_spawn_frame(self):
        # SITL LiDAR, clearance checks and the replay assume these three copies agree.
        world = ET.parse(ROOT / 'simulation/worlds/inspection.sdf').getroot().find('world')
        cylinders = set()
        for model in world.findall('model'):
            if model.get('name').startswith('obstacle_'):
                x, y = map(float, model.find('pose').text.split()[:2])
                cylinders.add((x, y, float(model.find('.//collision/geometry/cylinder/radius').text)))
        self.assertEqual(cylinders, set(OBSTACLES))
        vehicle = next(i for i in world.findall('include') if i.find('name').text == 'x500_0')
        self.assertEqual([float(v) for v in vehicle.find('pose').text.split()[:2]], [0.0, 0.0])

    def test_no_path_is_explicit(self):
        self.assertEqual(astar((0, 0), (2, 2), {(1, 0), (-1, 0), (0, 1), (0, -1)}), [])

    def test_gnss_update_reduces_covariance(self):
        kf = Localizer()
        _, before = kf.step(np.zeros(3), None, 0.05)
        _, after = kf.step(np.zeros(3), np.zeros(3), 0.05)
        self.assertLess(after, before)

    def test_dead_reckoning_uncertainty_grows(self):
        kf = Localizer()
        _, start = kf.step(np.zeros(3), None, 0.05)
        for _ in range(20):
            _, end = kf.step(np.zeros(3), None, 0.05)
        self.assertGreater(end, start)


def route_clearance(path):
    """Smallest distance from the straight segments between cells to a real cylinder surface."""
    return min(
        math.hypot(ax + (bx - ax) * i / 20 - cx, ay + (by - ay) * i / 20 - cy) - r
        for cx, cy, r in OBSTACLES
        for (ax, ay), (bx, by) in zip(path, path[1:])
        for i in range(21)
    )


class HarnessRobustnessTests(unittest.TestCase):
    """Closed-loop harness runs with the real C++ supervisor, beyond the single published seed."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        path = Path(cls.tmp.name) / 'model.onnx'
        create_model(path)
        cls.detector = Detector(path)
        cls.binary = ROOT / 'build/mission_supervisor'

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_hidden_obstacle_forces_a_replan_instead_of_a_collision(self):
        goal = (10, 6)
        # From the origin the (6,6) cylinder is mostly occluded, so the first plan crosses it.
        self.assertLess(route_clearance(astar((0, 0), goal, occupancy(lidar([0, 0, 0])))), 0)
        result = run_demo.run_scenario('nominal', self.detector, 17, self.binary, goal=goal)
        self.assertTrue(result['passed'], result['checks'])
        self.assertGreater(len(result['plans']), 1)
        self.assertGreater(result['summary']['min_obstacle_clearance_m'], 0.9)
        self.assertTrue(any(e['mode'] == 'REPLAN' for e in result['events']))

    def test_published_route_needs_no_replan(self):
        result = run_demo.run_scenario('nominal', self.detector, 17, self.binary)
        self.assertTrue(result['passed'])
        self.assertEqual(len(result['plans']), 1)

    def test_landing_scenarios_pass_on_every_seed(self):
        # LAND once stopped at an estimated zero altitude; after GNSS loss that left 8/20 seeds hovering.
        for name in ('gps_dropout', 'imu_dropout', 'link_dropout', 'low_battery'):
            for seed in range(8):
                with self.subTest(scenario=name, seed=seed):
                    result = run_demo.run_scenario(name, self.detector, seed, self.binary)
                    self.assertTrue(result['passed'], result['checks'])


class ProcessContractTests(unittest.TestCase):
    def test_malformed_frame_emits_no_command(self):
        result = subprocess.run(
            [str(ROOT / 'build/mission_supervisor')], input='invalid frame\n', text=True, capture_output=True, timeout=3
        )
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, '')

    def test_replayed_frame_terminates(self):
        frame = '0 0 0 0 0 0 1 0 0 0 0 0 3 10 0\n'
        result = subprocess.run(
            [str(ROOT / 'build/mission_supervisor')], input=frame * 2, text=True, capture_output=True, timeout=3
        )
        self.assertEqual(result.returncode, 2)
        self.assertEqual(len(result.stdout.splitlines()), 1)

    def test_negative_sequence_emits_no_command(self):
        result = subprocess.run(
            [str(ROOT / 'build/mission_supervisor')],
            input='-1 0 0 0 0 0 1 0 0 0 0 0 3 10 0\n',
            text=True,
            capture_output=True,
            timeout=3,
        )
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, '')


if __name__ == '__main__':
    unittest.main()
