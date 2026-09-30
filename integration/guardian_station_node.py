#!/usr/bin/env python3
"""SITL-only guardian station on the carrier: fuses detections, sets the protection posture, orders the
guardians, relocates the carrier within its delegated authority, checks each guardian's navigation against
its own datalink fix, and asks the center for recovery.

It never commands a vehicle's flight directly: orders go over the (jammable) uplink to each guardian's own
adapter, which keeps its C++ supervisor, its PX4 failsafes and its onboard keep-clear reflex.
"""

import argparse
import json
import math
import sys
import rclpy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from common import drop_malformed, finite_number, finite_vector
from fleet_station_node import Station
from guardian import (
    GREEN,
    INF,
    RED,
    NavIntegrity,
    Posture,
    Tracker,
    assess,
    fast_paths,
    impact_point,
    relevant,
    station_approach,
    station_orders,
    steady_fast,
)
from guardian_layout import CFG, GUARDIANS, ORDER_AGE, RELOCATE, SCENARIOS, free_space, reserved_for


class GuardianStation(Station):
    def __init__(self, args):
        super().__init__(args, name='guardian_station')
        self.hazard = None  # ((centre, radius), time last confirmed) of a fast object's impact area
        self.scenario = SCENARIOS[args.scenario]
        self.tracker = Tracker(CFG)
        self.posture = Posture(CFG)
        self.orders = {}
        self.recovery = None
        self.recovery_by = None
        self.held = False
        self.clear_since = None
        self.pending_since = -1e9
        self.relocating = False
        self.relocated = False
        self.relocate_x = None
        self.watch_since = None
        self.watch_request = None
        self.logged_tracks = set()
        self.last_orders = {}
        self.request = None
        self.requests = 0
        self.heard = {}
        self.integrity = {g['ns']: NavIntegrity(CFG) for g in GUARDIANS}
        self.fix = {}
        self.navigating = set()
        self.posts = {g['ns']: [float(g['post'][0]), float(g['post'][1]), g['altitude']] for g in GUARDIANS}
        self.uplink = self.create_publisher(String, '/station/uplink', 10)
        self.center_report = self.create_publisher(String, '/center/report', 10)
        self.create_subscription(String, '/station/detections', self.on_detections, 10)
        self.create_subscription(String, '/station/fixes', self.on_fixes, 10)
        self.create_subscription(String, '/center/decision', self.on_decision, 10)

    def on_state(self, drone, msg):
        if super().on_state(drone, msg):
            self.heard[drone] = self.now()

    @drop_malformed('/station/detections')
    def on_detections(self, msg):
        data = json.loads(msg.data)
        if not isinstance(data['source'], str):
            raise ValueError('source must be a name')
        for d in data['detections']:
            finite_vector(d['p'])
            finite_number(d['t'])
            finite_number(d.get('sigma', CFG.sigma))
        for d in data['detections']:
            self.tracker.update(d['t'], [(d['p'], data['source'], d.get('label'), d.get('sigma', CFG.sigma))])

    @drop_malformed('/station/fixes')
    def on_fixes(self, msg):
        """Compare each guardian's reported GNSS position with the station's own datalink fix of it."""
        fixes = json.loads(msg.data)
        now = self.now()
        for fix in fixes.values():
            finite_vector(fix, 4)  # x, y, z, time; all checked before any is used.
        for ns, fix in fixes.items():
            state = self.states.get(ns)
            if not state:
                continue
            # A flagged guardian flies on these fixes until it is down, so they stay current on the deck too. Frozen
            # at the last airborne fix, under a drag-off that went on, a landed guardian's corrected position drifted
            # and its pad error stayed over the touchdown limit for eight minutes.
            self.fix[ns] = fix
            if not state.get('armed') or state.get('phase') not in ('watch', 'return', 'rendezvous', 'descend'):
                continue  # The cross-check itself runs only in flight.
            monitor = self.integrity[ns]
            before = monitor.state
            # A guardian flying on station fixes reports the corrected position; compare what its GNSS says.
            reported = state.get('gnss_position') or state['position']
            after = monitor.update(now, reported, fix, age=now - self.heard.get(ns, -INF))
            if after == 'spoofed' and before == 'ok':
                self.navigating.add(ns)
                self.log.write('integrity', ns=ns, residual=monitor.residual, t=now)
                self.send_center(now, 'integrity', ns=ns)
            elif after == 'ok' and before == 'spoofed':
                self.navigating.discard(ns)
                self.log.write('integrity_clear', ns=ns, residual=monitor.residual, t=now)

    @drop_malformed('/center/decision')
    def on_decision(self, msg):
        data = json.loads(msg.data)
        decision = data['decision']
        request = data.get('request')
        self.log.write('center_decision', decision=decision, request=request, t=self.now())
        # Only an answer to the question still open counts: one sent before a new event must not recall guardians.
        if (
            decision in ('recover', 'resume')
            and self.recovery is None
            and request is not None
            and request == self.request
            and self.posture.state == GREEN
        ):
            self.recovery = decision
            self.recovery_by = 'center'
            self.log.write('recovery', by='center', decision=decision, request=request, t=self.now())

    def send_center(self, now, kind, **fields):
        message = String()
        message.data = json.dumps({'kind': kind, **fields})
        self.center_report.publish(message)
        self.log.write('center_report', report=kind, t=now, **fields)

    def station_pose(self):
        c = self.carrier or {'e': 0.0, 'n': 0.0, 'yaw': 0.0, 'speed': 0.0}
        return [c['e'], c['n'], 0.0], [c['speed'] * math.cos(c['yaw']), c['speed'] * math.sin(c['yaw']), 0.0]

    def station_goal(self):
        sp, _ = self.station_pose()
        return [self.relocate_x, sp[1], 0.0] if self.relocating and self.relocate_x is not None else None

    def relocation_stop(self, now, tracks):
        """Where along the road to shelter: the nearest stop that keeps every tracked threat's predicted path at
        least twice the protected radius away (the carrier drives there, then stops), else the one that keeps
        them farthest away. Driving always to the same stop once took the carrier toward a second intruder."""
        sp, _ = self.station_pose()
        threats = [tr for tr in tracks if tr.cls != 'bird']
        scored = []
        for x in sorted(RELOCATE['stops'], key=lambda x: abs(x - sp[0])):
            goal = [x, sp[1], 0.0]
            v = [math.copysign(RELOCATE['speed'], x - sp[0]), 0.0, 0.0]
            misses = [station_approach(tr.predict(now), tr.v, sp, v, goal) for tr in threats]
            scored.append((min((d for t, d in misses if t < CFG.warn_horizon), default=INF), x))
        good = [x for miss, x in scored if miss >= 2 * CFG.protect_radius]
        return good[0] if good else max(scored)[1]

    def tick(self):
        now = self.now()
        if now > 0.0:
            self.protect(now)
        super().tick()

    def protect(self, now):
        self.tracker.update(now, [])  # Drop tracks that have gone quiet.
        tracks = self.tracker.confirmed()
        sp, sv = self.station_pose()
        for tr in tracks:
            if tr.id not in self.logged_tracks:
                self.logged_tracks.add(tr.id)
                self.log.write('confirmed', track=tr.id, p=tr.p, v=tr.v, sources=sorted(tr.sources), cls=tr.cls, t=now)
        levels = [assess(tr, now, sp, sv, CFG, station_goal=self.station_goal()) for tr in tracks]
        before = self.posture.state
        if self.posture.update(now, [l for l, _, _ in levels]):
            state = self.posture.state
            self.log.write('posture', state=state, t=now, tracks=[tr.id for tr in tracks])
            self.send_center(now, 'posture', state=state)
            if state == RED:
                self.log.write('crew_alert', t=now)
                if not self.relocated and not self.relocating:
                    self.relocating = True
                    self.relocate_x = self.relocation_stop(now, tracks)
                    self.log.write('relocate', t=now, carrier=self.carrier, target_x=self.relocate_x)
            if before == RED:
                self.held = True
            if state == GREEN and self.held:
                self.clear_since = now
        if self.relocating:  # Threats seen after the decision can move the stop further along.
            stop = self.relocation_stop(now, tracks)
            if stop > self.relocate_x:
                self.relocate_x = stop
                self.log.write('relocate', t=now, carrier=self.carrier, target_x=stop)
        watching = {d: s for d, s in self.states.items() if s.get('phase') == 'watch'}
        # A planned watch with nothing seen ends by asking the center, as a clear after RED does.
        all_watching = len(watching) == len(self.drones)
        self.watch_since = (self.watch_since or now) if all_watching else self.watch_since
        if (
            self.scenario.get('watch')
            and self.watch_request is None
            and not self.held
            and self.watch_since is not None
            and now - self.watch_since >= self.scenario['watch'] + 5.0
            and self.posture.state == GREEN
        ):
            self.watch_request = now
            self.log.write('watch_complete', t=now)
        asking = self.recovery is None and self.posture.state == GREEN and (self.held or self.watch_request is not None)
        if asking:
            # Each clear is its own question, numbered, and resent until answered or the center times out.
            if self.request is None:
                self.requests += 1
                self.request = self.requests
                self.pending_since = -1e9
            if now - self.pending_since >= 5.0:
                self.send_center(now, 'clear_after_red' if self.held else 'watch_complete', request=self.request)
                self.pending_since = now
            since = self.clear_since if self.held else self.watch_request
            if (
                since is not None and now - since >= CFG.center_timeout
            ):  # The center has not answered: land under delegation.
                self.recovery = 'recover'
                self.recovery_by = 'station (delegated)'
                self.log.write('recovery', by=self.recovery_by, t=now)
        elif self.posture.state != GREEN:
            self.request = None
        # Orders are planned only for guardians heard from recently; a silent (jammed) guardian is told to hold
        # where it is, so that when its link returns it does not fly a move planned from where it used to be.
        fresh = {d: s for d, s in watching.items() if now - self.heard.get(d, -INF) <= ORDER_AGE}
        view = {d: {'p': s['position'], 'post': self.posts[d]} for d, s in fresh.items()}

        def free_for(name):
            others = [s['position'] for d, s in self.states.items() if d != name and s.get('armed') and d in fresh]
            wide = [
                (s['position'], 2.0 + CFG.guardian_speed * min(5.0, now - self.heard.get(d, now)))
                for d, s in self.states.items()
                if d != name and s.get('armed') and d not in fresh
            ]
            # Plan against the vehicle's conservative mapped cells as well as the station's known geometry.
            # Otherwise the station can repeatedly send a geometrically clear route the vehicle must reject.
            blocked = {tuple(cell) for cell in fresh[name].get('blocked_cells', [])}
            return lambda p: (
                free_space(view[name]['p'], p, others, reserved=reserved_for(name), wide=wide)
                and free_space(view[name]['p'], p, blocked=blocked)
            )

        fast = [(tr, t) for tr, (l, t, _) in zip(tracks, levels) if l == 'danger' and steady_fast(tr, CFG)]
        # A fast object's impact area, centred where it comes down: disperse from it. It stays declared for as
        # long as RED would after the last danger, so a young track's flickering assessment cannot drop a
        # dispersing guardian back to holding halfway out.
        if fast:
            self.hazard = ((impact_point(min(fast, key=lambda x: x[1])[0], now, sp), 2 * CFG.protect_radius), now)
        hazard = self.hazard[0] if self.hazard and now - self.hazard[1] < CFG.clear_time else None
        orders = station_orders(
            now,
            self.posture.state,
            tracks,
            view,
            CFG,
            free_for=free_for,
            recovery=self.recovery,
            hazard=hazard,
            held=(self.held or self.watch_request is not None) and self.recovery is None,
            previous=self.orders,
        )
        for d in watching:
            if d not in fresh:
                orders[d] = {'action': 'hold', 'target': None, 'reason': 'state_stale'}
        for name, order in orders.items():
            if order['action'] in ('recover', 'resume'):
                order['authority'] = self.recovery_by
            last = self.last_orders.get(name)
            if last is None or last['action'] != order['action'] or last.get('target') != order.get('target'):
                own = view[name]['p'] if name in view else watching[name]['position']
                self.log.write(
                    'order',
                    ns=name,
                    layer='station',
                    t=now,
                    own=own,
                    threats=[tr.predict(now) for tr in tracks],
                    avoid=relevant(own, tracks, now, CFG),
                    fast=fast_paths(tracks, now, CFG),
                    **{k: v for k, v in order.items() if k != 'tracks'},
                )
                self.last_orders[name] = order
        # Keep recovery orders for guardians already heading home, so a late uplink still carries them.
        self.orders = {
            **{d: o for d, o in self.orders.items() if d not in watching and o.get('action') == 'recover'},
            **orders,
        }

    def carrier_speed(self, now):
        """Relocation on RED is the station's delegated protective action: drive along the road to the chosen stop."""
        if self.relocating and self.carrier and self.carrier['e'] >= self.relocate_x:
            self.relocating = False
            self.relocated = True
            self.log.write('relocated', t=now, carrier=self.carrier)
        return RELOCATE['speed'] if self.relocating else 0.0

    def publish_clearance(self, now):
        # Every guardian is told which navigation to use; a flagged one flies on the station's fixes, and goes
        # back to GNSS when the cross-check clears.
        nav = {ns: self.fix[ns] for ns in self.navigating if ns in self.fix}
        mode = {g['ns']: 'station' if g['ns'] in self.navigating else 'gnss' for g in GUARDIANS}
        message = String()
        message.data = json.dumps(
            {
                'clearance': self.active_clearance(now),
                'orders': self.orders,
                'posture': self.posture.state,
                'nav': nav,
                'nav_mode': mode,
                't': now,
            }
        )
        self.uplink.publish(message)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--drones', required=True)
    p.add_argument('--log', required=True)
    p.add_argument('--scenario', required=True, choices=sorted(SCENARIOS))
    p.add_argument('--speed', type=float, default=0.0)
    p.add_argument('--max-east', type=float, default=14.0)
    args = p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = GuardianStation(args)
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
