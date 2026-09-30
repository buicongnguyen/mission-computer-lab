#!/usr/bin/env python3
"""SITL-only environment for the guardian scenarios: threats, sensors, jammed links and GNSS spoofing.

Everything here stands in for the physical world, not for software under test. It flies each threat along
its script (Gazebo moves the models), turns Gazebo truth into noisy detections for each guardian and for
the station, adds sensor clutter, gives the station a datalink position fix of each guardian, relays every
station <-> guardian message except while a guardian is inside the jamming zone, and drags every GNSS
receiver off by moving the world's geographic origin. Guardians always receive their own sensor's
detections: jamming cuts the link, not the sensor.
"""

import argparse
import json
import math
import os
import queue
import random
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common import JsonLog, drop_malformed
from guardian_layout import GUARDIANS, SCENARIOS, SENSORS, WATCH_BEFORE_START, impact_time, threat_position


class World(Node):
    def __init__(self, args):
        super().__init__('guardian_world')
        self.log = JsonLog(args.log)
        self.rng = random.Random(args.seed)
        self.name = args.scenario
        self.scenario = SCENARIOS[args.scenario]
        self.threats = self.scenario['threats']
        self.jammer = self.scenario.get('jammer')
        self.spoof = self.scenario.get('spoof')
        self.world = args.world
        self.origin = args.origin
        self.spoof_offset = 0.0
        self.last_spoof = None
        self.spoofer = None
        self.sent_offset = None
        self.truth = {}
        self.phase = {}
        self.watch_since = None
        self.started = None
        self.jamming = False
        self.link = {}
        self.stopped = set()
        self.impacts = set()
        self.carrier = None
        self.next_scan = 0.0
        self.next_fix = 0.0
        self.next_log = 0.0
        self.dropped = {g['ns']: 0 for g in GUARDIANS}
        self.drive = {m: self.create_publisher(Twist, f'/model/{m}/cmd_vel', 10) for m in self.threats}
        for model in [*self.threats, 'carrier', *[f'x500_{i}' for i in range(len(GUARDIANS))]]:
            self.create_subscription(
                Odometry, f'/model/{model}/odometry', lambda m, model=model: self.odometry(model, m), 10
            )
        self.station_detections = self.create_publisher(String, '/station/detections', 10)
        self.fixes = self.create_publisher(String, '/station/fixes', 10)
        self.onboard = {}
        self.uplink = {}
        self.relay_state = {}
        for g in GUARDIANS:
            ns = g['ns']
            self.create_subscription(String, f'/{ns}/guardian/state', lambda m, ns=ns: self.state(ns, m), 10)
            self.onboard[ns] = self.create_publisher(String, f'/{ns}/guardian/detections', 10)
            self.uplink[ns] = self.create_publisher(String, f'/{ns}/guardian/uplink', 10)
            self.relay_state[ns] = self.create_publisher(String, f'/fleet/{ns}/state', 10)
        self.create_subscription(String, '/station/uplink', self.station_uplink, 10)
        self.log.write(
            'scenario',
            name=self.name,
            threats=self.threats,
            jammer=self.jammer,
            spoof=self.spoof,
            center_down=bool(self.scenario.get('center_down')),
        )
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odometry(self, model, m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        self.truth[model] = {'p': [p.x, p.y, p.z], 'v': [v.x, v.y, v.z]}

    def guardian_truth(self, i):
        return (self.truth.get(f'x500_{i}') or {}).get('p')

    def jammed(self, ns):
        if not self.jamming:
            return False
        i = [g['ns'] for g in GUARDIANS].index(ns)
        p = self.guardian_truth(i)
        return p is not None and math.dist(p[:2], self.jammer['p']) < self.jammer['r']

    def relay(self, publisher, message, ns, kind):
        if self.jammed(ns):
            self.dropped[ns] += 1
            if self.dropped[ns] % 25 == 1:
                self.log.write('dropped', ns=ns, message=kind, count=self.dropped[ns], t=self.now())
            return
        publisher.publish(message)

    @drop_malformed('guardian state')
    def state(self, ns, msg):
        self.phase[ns] = json.loads(msg.data).get('phase')
        self.relay(self.relay_state[ns], msg, ns, 'state')

    def station_uplink(self, msg):
        for ns, publisher in self.uplink.items():
            self.relay(publisher, msg, ns, 'uplink')

    def tick(self):
        now = self.now()
        if now <= 0.0:
            return
        # The scenario starts once every guardian has held its watch post for a few seconds.
        if self.started is None:
            watching = len(self.phase) == len(GUARDIANS) and all(p == 'watch' for p in self.phase.values())
            self.watch_since = (self.watch_since or now) if watching else None
            if watching and now - self.watch_since >= WATCH_BEFORE_START:
                self.started = now
                self.jamming = bool(self.jammer)
                self.log.write('scenario_start', t=now)
        if self.started is not None:
            self.fly(now)
            self.jam(now)
            self.spoof_step(now)
        for g in GUARDIANS:  # Record each guardian's link going down and coming back.
            jammed = self.jammed(g['ns'])
            if jammed != self.link.get(g['ns'], False):
                self.link[g['ns']] = jammed
                self.log.write('link', ns=g['ns'], jammed=jammed, t=now)
        if now >= self.next_log:
            self.next_log = now + 0.2
            self.log.write(
                'truth',
                t=now,
                guardians={g['ns']: self.guardian_truth(i) for i, g in enumerate(GUARDIANS)},
                threats={m: self.truth.get(m, {}).get('p') for m in self.threats} if self.started is not None else {},
                jamming=self.jamming,
                spoof_offset=self.spoof_offset,
            )
        if self.started is not None and now >= self.next_scan:
            self.next_scan = now + SENSORS['guardian']['period']
            self.scan(now)
        if now >= self.next_fix:
            self.next_fix = now + SENSORS['locator']['period']
            self.locate(now)

    def fly(self, now):
        """Velocity commands that keep every threat on its script (Gazebo integrates them)."""
        for model, spec in self.threats.items():
            twist = Twist()
            t = now - self.started - spec.get('delay', 0.0)
            have = self.truth.get(model)
            if spec['motion'] == 'dive' and model not in self.impacts and have and t >= impact_time(spec):
                self.impacts.add(model)
                self.log.write('impact', model=model, t=now, p=have['p'])
            if t >= 0.0 and have and model not in self.stopped:
                goal, done = threat_position(spec, t + 0.3)
                if done and math.dist(have['p'], goal) < 0.3:
                    self.stopped.add(model)
                    self.log.write('threat_stop', model=model, t=now, p=have['p'])
                else:
                    v = [(g - p) / 0.3 for g, p in zip(goal, have['p'])]
                    if spec['motion'] == 'dive':  # The fast object's body axis points along its dive.
                        twist.linear.x = math.hypot(*v)
                    else:
                        twist.linear.x, twist.linear.y, twist.linear.z = v
            self.drive[model].publish(twist)

    def jam(self, now):
        if (
            self.jamming
            and self.jammer.get('watch') in self.truth
            and self.truth[self.jammer['watch']]['p'][1] < self.jammer['off_y']
        ):
            self.jamming = False
            self.log.write('jammer_off', t=now)

    def spoof_step(self, now):
        """Drag every receiver off by moving the world's geographic origin, ramped in slowly."""
        if not self.spoof or (self.last_spoof is not None and now - self.last_spoof < 0.5):
            return
        dt = 0.0 if self.last_spoof is None else now - self.last_spoof
        self.last_spoof = now
        rate = self.spoof['rate'] * min(1.0, (now - self.started) / self.spoof['ramp'])
        self.spoof_offset = min(self.spoof['max'], self.spoof_offset + rate * dt)
        if dt == 0.0:
            self.spoofer = Spoofer(self.world)
            self.log.write('spoof_start', t=now, **self.spoof)
        if self.spoofer.failed > self.spoofer.logged:
            self.spoofer.logged = self.spoofer.failed
            self.log.write('spoof_error', t=now, failed=self.spoofer.failed)
        if self.sent_offset is not None and abs(self.spoof_offset - self.sent_offset) < 0.01:
            return  # Saturated: nothing to send.
        # Moving the origin by -d makes every receiver report a position d further along -d, so a guardian
        # holding its post on GNSS is physically dragged by +d: heading_deg is the direction of that drag.
        heading = math.radians(self.spoof['heading_deg'])
        de = self.spoof_offset * math.cos(heading)
        dn = self.spoof_offset * math.sin(heading)
        lat = self.origin[0] - math.degrees(dn / 6378137.0)
        lon = self.origin[1] - math.degrees(de / (6378137.0 * math.cos(math.radians(self.origin[0]))))
        self.spoofer.request(lat, lon)
        self.sent_offset = self.spoof_offset

    def detect(self, sensor, origin, model, spec):
        """A noisy detection of one threat from origin, with a camera label when close enough, or None."""
        have = self.truth.get(model)
        if not have or self.started is None or self.now() - self.started < spec.get('delay', 0.0) or have['p'][2] < 0.0:
            return None
        p = have['p']
        r = math.dist(origin, p)
        limit = sensor['low_range'] if 'low_altitude' in sensor and p[2] < sensor['low_altitude'] else sensor['range']
        if r > limit or self.rng.random() > sensor['pd']:
            return None
        label = None
        if r <= sensor.get('classify_range', -1):
            kinds = ('drone', 'fast', 'bird')
            label = (
                spec['kind']
                if self.rng.random() < sensor['accuracy']
                else self.rng.choice([k for k in kinds if k != spec['kind']])
            )
        return {
            'p': [c + self.rng.gauss(0, sensor['sigma']) for c in p],
            'label': label,
            'sigma': sensor['sigma'],
            't': self.now(),
        }

    def clutter(self, origin, sensor):
        """Poisson false detections scattered through the sensor's volume."""
        lam = self.scenario.get('clutter', 0.0) * sensor['period']
        out = []
        k = 0
        prod = self.rng.random()
        threshold = math.exp(-lam)
        while prod > threshold:
            k += 1
            prod *= self.rng.random()
        for _ in range(k):
            r = sensor['range'] * self.rng.random() ** (1 / 3)
            a = self.rng.uniform(0, 2 * math.pi)
            z = max(0.3, origin[2] + self.rng.uniform(-3, 3))
            out.append(
                {
                    'p': [origin[0] + r * math.cos(a), origin[1] + r * math.sin(a), z],
                    'label': None,
                    'sigma': sensor['sigma'],
                    't': self.now(),
                }
            )
        return out

    def scan(self, now):
        for i, g in enumerate(GUARDIANS):
            pose = self.guardian_truth(i)
            if pose is None:
                continue
            found = [
                d for m, s in self.threats.items() for d in [self.detect(SENSORS['guardian'], pose, m, s)] if d
            ] + self.clutter(pose, SENSORS['guardian'])
            if not found:
                continue
            message = String()
            message.data = json.dumps({'source': g['ns'], 'detections': found})
            self.onboard[g['ns']].publish(message)
            self.relay(self.station_detections, message, g['ns'], 'detections')
        if self.carrier_pose():
            found = [
                d
                for m, s in self.threats.items()
                for d in [self.detect(SENSORS['station'], self.carrier_pose(), m, s)]
                if d
            ]
            if found:
                message = String()
                message.data = json.dumps({'source': 'station', 'detections': found})
                self.station_detections.publish(message)

    def carrier_pose(self):
        return (self.truth.get('carrier') or {}).get('p')

    def locate(self, now):
        """The station's datalink ranging fixes each guardian it can hear (not while jammed)."""
        c = self.carrier_pose()
        if c is None:
            return
        fixes = {}
        for i, g in enumerate(GUARDIANS):
            p = self.guardian_truth(i)
            if p is None or self.jammed(g['ns']) or math.dist(p, c) > SENSORS['locator']['range']:
                continue
            fixes[g['ns']] = [*[x + self.rng.gauss(0, SENSORS['locator']['sigma']) for x in p[:2]], p[2], now]
        if fixes:
            message = String()
            message.data = json.dumps(fixes)
            self.fixes.publish(message)


class Spoofer:
    """Sets the world's spherical coordinates through Gazebo's own service, in process and on a worker thread
    (the latest request wins). The gz command-line tool costs about a second of CPU per call; two calls a
    second starved the simulation until the vehicles' supervisors saw stale IMU data and landed them."""

    def __init__(self, world):
        os.environ.setdefault(
            'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python'
        )  # gz-msgs predates the venv's protobuf.
        from gz.transport13 import Node as GzNode
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.spherical_coordinates_pb2 import SphericalCoordinates

        self.node = GzNode()
        self.types = (SphericalCoordinates, Boolean)
        self.service = f'/world/{world}/set_spherical_coordinates'
        self.pending = queue.Queue(maxsize=1)
        self.failed = 0
        self.logged = 0
        threading.Thread(target=self.run, daemon=True).start()

    def request(self, lat, lon):
        try:
            self.pending.get_nowait()
        except queue.Empty:
            pass
        self.pending.put((lat, lon))

    def run(self):
        request_type, reply_type = self.types
        while True:
            lat, lon = self.pending.get()
            message = request_type()
            message.surface_model = request_type.EARTH_WGS84
            message.latitude_deg = lat
            message.longitude_deg = lon
            message.elevation = 0.0
            ok, reply = self.node.request(self.service, message, request_type, reply_type, 1000)
            if not (ok and reply.data):
                self.failed += 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--log', required=True)
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--scenario', required=True, choices=sorted(SCENARIOS))
    p.add_argument('--world', default='guardian')
    p.add_argument(
        '--origin', type=float, nargs=2, default=[47.397971, 8.546164], help='World origin latitude and longitude'
    )
    args = p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = World(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
