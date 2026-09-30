#!/usr/bin/env python3
"""Fast guardian simulator: seeded runs of every threat scenario under four architectures.

Kinematic and illustrative. Sensor ranges, speeds and delays are round numbers chosen to expose design
trade-offs, not the specification of any real system. Threats fly to fixed aim points; nothing here models
a seeker, a countermeasure or an engagement. The decision logic is tools/guardian.py, the same module the
PX4 SITL guardian nodes run.
"""

import argparse
import hashlib
import json
import math
from multiprocessing import Pool
from pathlib import Path
import platform
import random
import sys
from guardian import (
    RED,
    Center,
    Config,
    NavIntegrity,
    Posture,
    Tracker,
    assess,
    authorised,
    closest_approach,
    fast_paths,
    impact_point,
    onboard_decide,
    predicted_miss,
    relevant,
    safe_move,
    segment_miss,
    station_orders,
    steady_fast,
    sub,
)

ROOT = Path(__file__).resolve().parents[1]
DT = 0.1
CFG = Config(
    gate=40.0,
    gate_speed=320.0,
    accel_noise=8.0,
    sigma=10.0,
    confirm_hits=4,
    confirm_window=5.0,
    drop_tentative=3.0,
    drop_confirmed=8.0,
    protect_radius=100.0,
    warn_horizon=150.0,
    slow_persist=8.0,
    slow_speed=12.0,
    inner_radius=400.0,
    clear_radius=150.0,
    safe_radius=75.0,
    order_horizon=60.0,
    reflex_horizon=20.0,
    keep_clear_step=250.0,
    climb_step=40.0,
    max_altitude=120.0,
    guardian_speed=12.0,
    fast_speed=60.0,
    clear_time=20.0,
    lost_link=3.0,
    lost_link_return=60.0,
    center_timeout=90.0,
    integrity_threshold=60.0,
    integrity_persistence=3,
    integrity_clear=6,
    integrity_max_age=2.0,
    fix_timeout=10.0,
)
# A move judged on truth may come this much nearer a real threat than it started (about 2.5 sigma of the
# guardian's own sensor at close range) before it counts as closing: decisions are made on tracks, not truth.
TRUTH_TOLERANCE = 25.0
ORDER_AGE = 1.5  # s: the station plans orders only for guardians it has heard from this recently
# Illustrative sensors. The ground sensor loses low flyers early (terrain and clutter); drones look down.
STATION_SENSOR = {
    'range': 1200.0,
    'low_altitude': 50.0,
    'low_range': 400.0,
    'pd': 0.9,
    'sigma': 6.0,
    'period': 1.0,
    'clutter': 0.2,
}
# Guardians also carry a camera that labels what they detect inside classify_range, right 90% of the time.
DRONE_SENSOR = {
    'range': 800.0,
    'pd': 0.85,
    'sigma': 10.0,
    'period': 0.5,
    'clutter': 0.05,
    'classify_range': 500.0,
    'accuracy': 0.9,
}
LABELS = ('drone', 'fast', 'bird')
POSTS = {
    'g0': (0.0, 900.0, 80.0),
    'g1': (900.0, 0.0, 80.0),
    'g2': (0.0, -900.0, 80.0),
    'g3': (-900.0, 0.0, 80.0),
    'g4': (0.0, 0.0, 60.0),
}
# Layout experiments run the hybrid design with the watch posts moved.
LAYOUTS = {
    'baseline': POSTS,
    'offset_overwatch': {**POSTS, 'g4': (250.0, 0.0, 60.0)},
    'wide_pickets': {
        **{g: (x * 1500 / 900, y * 1500 / 900, z) for g, (x, y, z) in POSTS.items() if g != 'g4'},
        'g4': POSTS['g4'],
    },
}
LAYOUT_NOTES = {
    'baseline': 'Pickets 900 m out; overwatch hovering over the station.',
    'offset_overwatch': 'Overwatch moved 250 m off the station it watches.',
    'wide_pickets': 'Pickets 1500 m out instead of 900 m.',
}
EXPERIMENT_SCENARIOS = ('intruder', 'fast_inbound', 'swarm')
GUARDIAN_SPEED = 12.0
GUARDIAN_CLIMB = 4.0
GUARDIAN_ACCEL = 4.0
LINK_LATENCY = 0.2
LINK_LOSS = 0.02
CENTER_LATENCY = 1.5
STATION_SPEED = 8.0
RELOCATE = 400.0
ARCHITECTURES = {
    'station_only': {'drone_sensing': False, 'network': True, 'reflex': False, 'lost_link': False, 'integrity': False},
    'onboard_only': {'drone_sensing': True, 'network': False, 'reflex': True, 'lost_link': False, 'integrity': False},
    'networked': {'drone_sensing': True, 'network': True, 'reflex': False, 'lost_link': False, 'integrity': False},
    'hybrid': {'drone_sensing': True, 'network': True, 'reflex': True, 'lost_link': True, 'integrity': True},
}
SCENARIOS = ('intruder', 'fast_inbound', 'swarm', 'birds', 'jamming', 'spoofing', 'center_loss', 'combined')
DESCRIPTIONS = {
    'intruder': 'One low drone at 15-20 m/s from a random bearing, flying to the station.',
    'fast_inbound': 'A 250 m/s object from 10 km at 150-250 m, diving on the station for its last 1.5 km.',
    'swarm': 'Five low drones from two sectors, staggered over 30 s.',
    'birds': 'Eight slow birds circling 700-1500 m out, with five times the usual sensor clutter.',
    'jamming': 'An intruder along a picket\'s sector while a jammer cuts that picket\'s link.',
    'spoofing': 'A GNSS drag-off on one picket: its reported position drifts at up to 3 m/s.',
    'center_loss': 'An intruder while the station cannot reach the center at all.',
    'combined': 'Three intruders, a jammed picket, birds and no center link.',
}


class Threat:
    def __init__(self, name, kind, p, speed, aim, start, weave=0.0, period=20.0, dive=False, loiter=None):
        self.name = name
        self.kind = kind
        self.p = list(p)
        self.speed = speed
        self.aim = list(aim)
        self.start = start
        self.weave = weave
        self.period = period
        self.loiter = loiter
        self.v = [0.0, 0.0, 0.0]
        self.passed = False
        self.ended = False
        self.dive = dive
        self.climb = 0.0
        self.phase = random.Random(name).random() * 2 * math.pi

    def active(self, now):
        return now >= self.start and not self.ended

    def step(self, now, dt):
        if not self.active(now):
            return
        if self.loiter:  # A bird circling a point, with its own turn noise.
            c, r, omega = self.loiter
            a = omega * (now - self.start) + self.phase
            target = [c[0] + r * math.cos(a), c[1] + r * math.sin(a), self.p[2]]
            d = sub(target, self.p)
            n = math.hypot(*d[:2]) or 1.0
            self.v = [d[0] / n * self.speed, d[1] / n * self.speed, 0.0]
        else:
            if not self.passed:
                d = sub(self.aim, self.p)[:2]
                dist = math.hypot(*d)
                if dist < 60.0:
                    self.passed = True
                    if self.dive:
                        self.ended = True
                        return  # It reaches its aim point and is gone.
                heading = math.atan2(d[1], d[0]) + math.radians(self.weave) * math.sin(
                    2 * math.pi * (now - self.start) / self.period
                )
                if self.dive and dist < 1500.0:
                    self.climb = -self.p[2] * self.speed / max(dist, 1.0)  # Terminal dive.
                self.v = [self.speed * math.cos(heading), self.speed * math.sin(heading), self.climb]
            if self.p[2] <= 0.0:
                self.v[2] = 0.0
        self.p = [a + b * dt for a, b in zip(self.p, self.v)]
        self.p[2] = max(0.0, self.p[2])


def approach(rng, name, bearing, speed, altitude, distance, start, weave=0.0, aim=(0.0, 0.0, 0.0)):
    p = [aim[0] + distance * math.cos(bearing), aim[1] + distance * math.sin(bearing), altitude]
    return Threat(name, 'drone', p, speed, aim, start, weave=weave, period=rng.uniform(15, 30))


def scenario(name, rng, posts=POSTS):
    """Threats, jammer, spoofer, center availability and duration for one seeded run."""
    s = {'threats': [], 'jammer': None, 'spoof': None, 'center_down': False, 'duration': 300.0, 'clutter_scale': 1.0}
    b = rng.uniform(0, 2 * math.pi)
    if name in ('intruder', 'center_loss'):
        s['threats'].append(
            approach(rng, 't1', b, rng.uniform(15, 20), rng.uniform(30, 50), 3000.0, 20.0, weave=rng.uniform(0, 20))
        )
        s['center_down'] = name == 'center_loss'
        s['duration'] = 380.0
    elif name == 'fast_inbound':
        p = [10000 * math.cos(b), 10000 * math.sin(b), rng.uniform(150, 250)]
        s['threats'].append(Threat('t1', 'fast', p, 250.0, (0.0, 0.0, 0.0), 20.0, dive=True))
        s['duration'] = 240.0
    elif name in ('swarm', 'combined'):
        count = 5 if name == 'swarm' else 3
        b2 = b + rng.uniform(math.pi / 2, math.pi)
        for i in range(count):
            s['threats'].append(
                approach(
                    rng,
                    f't{i + 1}',
                    (b if i % 2 == 0 else b2) + rng.uniform(-0.3, 0.3),
                    rng.uniform(15, 20),
                    rng.uniform(25, 50),
                    2800.0 + rng.uniform(0, 400),
                    20.0 + rng.uniform(0, 30),
                    weave=rng.uniform(0, 15),
                )
            )
        s['duration'] = 400.0
    elif name == 'jamming':
        post = 'g1'
        b = math.atan2(posts[post][1], posts[post][0]) + rng.uniform(-0.25, 0.25)
        s['threats'].append(
            approach(rng, 't1', b, rng.uniform(15, 20), rng.uniform(30, 50), 3000.0, 20.0, weave=rng.uniform(0, 10))
        )
        s['duration'] = 380.0
    elif name == 'spoofing':
        s['duration'] = 260.0
    if name in ('birds', 'combined'):
        for i in range(8):
            r = rng.uniform(700, 1500)
            a = rng.uniform(0, 2 * math.pi)
            c = [r * math.cos(a), r * math.sin(a)]
            s['threats'].append(
                Threat(
                    f'b{i + 1}',
                    'bird',
                    [c[0] + 100, c[1], rng.uniform(20, 60)],
                    rng.uniform(6, 10),
                    c,
                    0.0,
                    loiter=(c, rng.uniform(80, 200), rng.choice((-1, 1)) * rng.uniform(0.03, 0.06)),
                )
            )
        s['clutter_scale'] = 5.0
        if name == 'birds':
            s['duration'] = 240.0
    if name in ('jamming', 'combined'):
        # The jammer sits beside the picket in the threat's sector and switches on as the threat starts.
        attacked = min(
            posts,
            key=lambda g: math.dist(posts[g][:2], [3000 * math.cos(b), 3000 * math.sin(b)]) if g != 'g4' else math.inf,
        )
        jp = posts[attacked]
        s['jammer'] = {
            'p': [jp[0] * 1.15, jp[1] * 1.15],
            'r': 600.0,
            'on': 20.0,
            'off': s['duration'],
            'victim': attacked,
        }
    if name in ('combined',):
        s['center_down'] = True
    if name == 'spoofing':
        s['spoof'] = {'victim': 'g0', 'onset': 40.0, 'rate': 3.0, 'ramp': 30.0, 'heading': rng.uniform(0, 2 * math.pi)}
    return s


class Guardian:
    def __init__(self, name, post):
        self.name = name
        self.post = list(post)
        self.p = list(post)
        self.v = [0.0, 0.0, 0.0]
        self.offset = [0.0, 0.0, 0.0]
        self.action = 'watch'
        self.target = list(post)
        self.layer = 'station'
        self.hold_at = None
        self.order = {'action': 'watch', 'target': list(post)}
        self.last_uplink = 0.0
        self.tracker = Tracker(CFG)
        self.nav_by_station = False
        self.fix = None
        self.nav_since = None
        self.landed = False
        self.next_scan = 0.0
        self.next_report = 0.0
        self.reflex = None
        self.last_checked = None
        self.rally = [0.0, 0.0, 0.0]  # Pre-briefed: the station's position as last heard over the uplink.

    def gnss(self):
        return [a + b for a, b in zip(self.p, self.offset)]

    def nav(self, now):
        """Navigation position: GNSS, or the station's last fix carried forward by the vehicle's own velocity.
        A fix corrects only the horizontal position; height stays barometric, which a drag-off does not move."""
        if self.nav_by_station and self.fix:
            t, p = self.fix
            return [p[0] + self.v[0] * (now - t), p[1] + self.v[1] * (now - t), self.p[2]]
        return self.gnss()

    def use_station_fixes(self, now, on):
        if on and not self.nav_by_station:
            self.nav_since = now
        if not on:
            self.fix = None
        self.nav_by_station = on

    def fly(self, now, dt):
        if self.landed:
            self.v = [0.0, 0.0, 0.0]
            return
        nav = self.nav(now)
        goal = self.target or nav
        desired = [(g - n) * 0.5 for g, n in zip(goal, nav)]
        h = math.hypot(desired[0], desired[1])
        if h > GUARDIAN_SPEED:
            desired[0] *= GUARDIAN_SPEED / h
            desired[1] *= GUARDIAN_SPEED / h
        desired[2] = max(-GUARDIAN_CLIMB, min(GUARDIAN_CLIMB, desired[2]))
        dv = sub(desired, self.v)
        n = math.hypot(*dv)
        lim = GUARDIAN_ACCEL * dt
        if n > lim:
            dv = [c * lim / n for c in dv]
        self.v = [a + b for a, b in zip(self.v, dv)]
        self.p = [a + b * dt for a, b in zip(self.p, self.v)]


def detect(rng, sensor, origin, objects, now, clutter_scale, low_mask=False, kinds=None):
    """Noisy (position, name, label) detections of the objects in range, plus Poisson clutter.

    Only sensors with a camera attach labels, inside their classification range."""
    out = []

    def label(kind, rng_):
        if 'classify_range' not in sensor or rng_ > sensor['classify_range']:
            return None
        if kind in LABELS and rng.random() < sensor['accuracy']:
            return kind
        return rng.choice([l for l in LABELS if l != kind])

    for name, p in objects:
        rng_ = math.dist(origin, p)
        limit = sensor['low_range'] if low_mask and p[2] < sensor['low_altitude'] else sensor['range']
        if rng_ <= limit and rng.random() < sensor['pd']:
            s = sensor['sigma'] + 0.005 * rng_
            out.append(([c + rng.gauss(0, s) for c in p], name, label((kinds or {}).get(name), rng_), s))
    lam = sensor['clutter'] * clutter_scale
    k = 0
    threshold = math.exp(-lam)
    prod = rng.random()
    while prod > threshold:
        k += 1
        prod *= rng.random()
    for _ in range(k):
        r = sensor['range'] * math.sqrt(rng.random())
        a = rng.uniform(0, 2 * math.pi)
        out.append(
            (
                [origin[0] + r * math.cos(a), origin[1] + r * math.sin(a), rng.uniform(0, 200)],
                'clutter',
                label('clutter', r),
                sensor['sigma'],
            )
        )
    return out


def closing(own, target, positions):
    """How much nearer than at the start a straight move own -> target comes to any of these positions."""
    return max((math.dist(own, q) - segment_miss(sub(own, q), sub(target, own), 1.0) for q in positions), default=0.0)


def segment_min(p0, v_rel, dt):
    """Minimum distance over one step for linear relative motion."""
    t, d = closest_approach(p0, v_rel)
    return d if t <= dt else math.dist([a + b * dt for a, b in zip(p0, v_rel)], [0, 0, 0])


def run(scenario_name, architecture, seed, trace=False, layout='baseline'):
    posts = LAYOUTS[layout]
    key = f'{scenario_name}/{seed}'
    rng = random.Random(key)
    flags = ARCHITECTURES[architecture]
    s = scenario(scenario_name, rng, posts)
    # Common random numbers: every sensor and every link draws from its own stream, so the four designs
    # see the same threats, the same station returns and the same lost packets.
    station_rng = random.Random(key + '/station')
    drone_rng = {n: random.Random(f'{key}/{n}') for n in posts}
    decision_time = rng.uniform(8, 20)
    center_down = s['center_down']
    guardians = {n: Guardian(n, p) for n, p in posts.items()}
    threats = s['threats']
    real = [t for t in threats if t.kind != 'bird']
    victim = s['spoof'] and s['spoof']['victim']
    station = {
        'p': [0.0, 0.0, 0.0],
        'v': [0.0, 0.0, 0.0],
        'goal': None,
        'tracker': Tracker(CFG),
        'posture': Posture(CFG),
        'next_scan': 0.0,
        'next_orders': 0.0,
        'reports': {},
        'gnss': {},
        'velocity': {},
        'integrity': {n: NavIntegrity(CFG) for n in guardians},
        'heard': {},
        'friendly': {},
        'rally': {},
        'recovery': None,
        'recovery_by': None,
        'clear_since': None,
        'orders': {},
        'pending_since': -1e9,
        'held': False,
        'request': None,
        'requests': 0,
        'hazard': None,
    }
    center = Center(CENTER_LATENCY, decision_time, reachable=lambda now: not center_down)
    uplink = []
    downlink = []
    center_replies = []
    events = []
    samples = []
    metrics = {
        'min_separation': math.inf,
        'red': [],
        'arrival': {},
        'first_confirm': None,
        'first_confirm_range': None,
        'keep_clear': 0,
        'closing': 0,
        'decision_closing': 0,
        'authority_violations': 0,
        'false_red': 0,
        'station_cpa': math.inf,
        'jam_lost': None,
        'jam_action': None,
        'cpa': {},
        'spoof_detect': None,
        'spoof_drift': 0.0,
        'delegated': False,
        'false_integrity': 0,
        'nav_error': 0.0,
        'landed_in_place': 0,
    }

    def log(now, layer, action, who, **detail):
        if not authorised(action, layer):
            metrics['authority_violations'] += 1
        if trace or action in ('crew_alert', 'recover', 'resume', 'posture'):
            events.append({'t': round(now, 1), 'layer': layer, 'action': action, 'who': who, **detail})

    def expected_position(name, now):
        """Last report, or where the pre-briefed lost-link procedure has taken a guardian that went silent."""
        last = station['reports'].get(name, guardians[name].post)
        age = now - station['heard'].get(name, 0.0)
        if not flags['lost_link'] or age <= CFG.lost_link_return:
            return last
        rally = station['rally'].get(name, [0.0, 0.0, 0.0])
        d = sub(rally, last)[:2]
        n = math.hypot(*d)
        step = min(n, GUARDIAN_SPEED * (age - CFG.lost_link_return))
        return last if n < 1e-6 else [last[0] + d[0] / n * step, last[1] + d[1] / n * step, last[2]]

    def link_ok(now, g, kind):
        if not flags['network']:
            return False
        j = s['jammer']
        if j and j['on'] <= now < j['off'] and math.dist(g.p[:2], j['p']) < j['r']:
            return False
        return random.Random(f'{key}/{g.name}/{kind}/{now:.1f}').random() > LINK_LOSS

    def check_move(now, g, target, tracks, own=None):
        """A move being executed. The invariant: it is a safe_move against the tracks it was decided on (it opens
        the range along its whole path to every relevant slow track, and shortens no steady fast object's
        predicted miss). Measured, not claimed: on truth, whether it brought the guardian more than tracking
        error nearer a real slow threat that a track was on and that truly passes within the clear radius inside
        the planning horizon, or shortened a real fast object's true miss by more than that. Predictions are
        straight lines and weaving threats defeat them, so this can happen; a threat nobody has tracked yet is a
        detection gap and is not counted."""
        own = own or g.nav(now)
        predicted = [tr.predict(now) for tr in tracks]
        if not safe_move(own, target, relevant(own, tracks, now, CFG), fast_paths(tracks, now, CFG), CFG):
            metrics['decision_closing'] += 1
        horizon = max(CFG.order_horizon, CFG.reflex_horizon)
        tracked = [t for t in real if t.active(now) and any(math.dist(q, t.p) < 200.0 for q in predicted)]
        known = [
            t.p
            for t in tracked
            if t.kind != 'fast'
            and (lambda tc, d: tc < horizon and d < CFG.clear_radius)(*closest_approach(sub(t.p, g.p), t.v))
        ]
        shortened = max(
            (
                predicted_miss(g.p, g.p, [(t.p, t.v)], CFG, horizon)
                - predicted_miss(g.p, target, [(t.p, t.v)], CFG, horizon)
                for t in tracked
                if t.kind == 'fast'
            ),
            default=0.0,
        )
        if max(closing(g.p, target, known), shortened) > TRUTH_TOLERANCE:
            metrics['closing'] += 1  # Truth measure, reported as a rate.

    def associate_friendlies(now, seen):
        """The station's own track of each airborne guardian from its sensor returns, kept by continuity rather
        than by the guardian's report, which may be spoofed or stale. Returns the claimed returns and the
        positions fit for the integrity cross-check.

        Continuity first: a track is carried forward by the guardian's reported velocity and takes the
        nearest return within three sigma of the sensor at that range plus what acceleration and a drag-off
        can add since its last update, so a missed return cannot hand the track to a bird. Then every other airborne guardian claims its nearest return near its report, so no
        guardian is ever tracked as a threat; that return starts a new track only if it agrees with a fresh
        report to within three sigma (a guardian on station fixes reports a position the fixes corrected).
        Every association also checks height against the reported (barometric) altitude, which a GNSS
        drag-off does not move: a bird circling below a guardian is never taken for it."""
        tight = []
        claimed = set()
        observed = {}
        done = set()
        sigma = lambda p: STATION_SENSOR['sigma'] + 0.005 * math.dist(p, station['p'])
        covered = lambda p: (
            math.dist(p, station['p'])
            <= (STATION_SENSOR['low_range'] if p[2] < STATION_SENSOR['low_altitude'] else STATION_SENSOR['range'])
        )

        def level(name, p, dt):
            z = station['reports'].get(name, guardians[name].post)[2]
            return abs(p[2] - z) < 3 * sigma(p) + GUARDIAN_CLIMB * (dt + now - station['heard'].get(name, 0.0))

        for name, g in guardians.items():
            f = station['friendly'].get(name)
            if g.landed or (f and now - f['t'] > 3.0):
                station['friendly'].pop(name, None)
                continue
            if f:
                # Carried forward by the guardian's reported velocity (a drag-off biases it by a few m/s at most).
                v = station['velocity'].get(name, [0.0, 0.0, 0.0])
                dt = now - f['t']
                centre = [a + b * dt for a, b in zip(f['p'], v)]
                if not covered(centre):
                    continue  # Out of the sensor's view: no return there can be this guardian.
                gate = 3 * sigma(centre) + GUARDIAN_ACCEL * dt * dt / 2 + 5.0 * dt
                tight += [
                    (math.dist(centre, p), name, i)
                    for i, (p, *_) in enumerate(seen)
                    if math.dist(centre, p) < gate and level(name, p, now - f['t'])
                ]
        for _, name, i in sorted(tight):  # Nearest pairs first, one return per guardian and per return.
            if name in done or i in claimed:
                continue
            done.add(name)
            claimed.add(i)
            f = station['friendly'][name]
            f['p'] = seen[i][0]
            f['t'] = now
            f['hits'] += 1
            if f['hits'] >= 3:
                observed[name] = f['p']
        wide = []
        for name, g in guardians.items():
            if g.landed or name in done:
                continue
            e = expected_position(name, now)
            gate = 250.0 if now - station['heard'].get(name, 0.0) < CFG.lost_link else 400.0
            wide += [(math.dist(e, p), name, i) for i, (p, *_) in enumerate(seen) if math.dist(e, p) < gate]
        for d, name, i in sorted(wide):
            if name in done or i in claimed:
                continue
            done.add(name)
            claimed.add(i)
            if (
                name not in station['friendly']
                and d < 3 * sigma(seen[i][0])
                and level(name, seen[i][0], 0.0)
                and now - station['heard'].get(name, -math.inf) <= CFG.integrity_max_age
            ):
                station['friendly'][name] = {'p': seen[i][0], 't': now, 'hits': 1}
        return claimed, observed

    now = 0.0
    steps = int(s['duration'] / DT)
    for k in range(steps + 1):
        now = round(k * DT, 6)
        # Truth: threats and guardians move; record separations over the step.
        for t in threats:
            t.step(now, DT)
        for g in guardians.values():
            spoof = s['spoof']
            if spoof and g.name == victim and now >= spoof['onset']:
                rate = spoof['rate'] * min(1.0, (now - spoof['onset']) / spoof['ramp'])
                g.offset[0] += rate * math.cos(spoof['heading']) * DT
                g.offset[1] += rate * math.sin(spoof['heading']) * DT
                metrics['spoof_drift'] = max(metrics['spoof_drift'], math.dist(g.p[:2], g.post[:2]))
            g.fly(now, DT)
            if not g.landed and g.name != victim:
                metrics['nav_error'] = max(metrics['nav_error'], math.dist(g.nav(now)[:2], g.p[:2]))
        if station['goal']:
            d = sub(station['goal'], station['p'])
            n = math.hypot(*d[:2])
            station['v'] = [0.0, 0.0, 0.0] if n < 1.0 else [d[0] / n * STATION_SPEED, d[1] / n * STATION_SPEED, 0.0]
            station['p'] = [a + b * DT for a, b in zip(station['p'], station['v'])]
        for t in real:
            if not t.active(now):
                continue
            for g in guardians.values():
                if not g.landed:
                    metrics['min_separation'] = min(
                        metrics['min_separation'], segment_min(sub(g.p, t.p), sub(g.v, t.v), DT)
                    )
            horizontal = math.dist(t.p[:2], station['p'][:2])
            metrics['station_cpa'] = min(metrics['station_cpa'], horizontal)
            if horizontal < metrics['cpa'].get(t.name, (math.inf, 0))[0]:
                metrics['cpa'][t.name] = (horizontal, now)
            if t.name not in metrics['arrival'] and horizontal < CFG.protect_radius:
                metrics['arrival'][t.name] = now
        # Guardian sensing, reports and onboard decisions.
        objects = [(t.name, t.p) for t in threats if t.active(now)]
        kinds = {t.name: t.kind for t in threats}
        for g in guardians.values():
            if flags['drone_sensing'] and now >= g.next_scan and not g.landed:
                g.next_scan = now + DRONE_SENSOR['period']
                seen = detect(drone_rng[g.name], DRONE_SENSOR, g.p, objects, now, s['clutter_scale'], kinds=kinds)
                if flags['reflex']:
                    g.tracker.update(now, [(p, g.name, lab, sg) for p, _, lab, sg in seen])
                if seen and link_ok(now, g, 'detections'):
                    downlink.append((now + LINK_LATENCY, 'detections', g.name, now, seen))
            if flags['network'] and now >= g.next_report:
                g.next_report = now + 0.5
                if link_ok(now, g, 'state'):
                    downlink.append((now + LINK_LATENCY, 'state', g.name, now, (g.nav(now), g.gnss(), list(g.v))))
        for item in [m for m in uplink if m[0] <= now]:
            uplink.remove(item)
            _, kind, name, payload = item
            g = guardians[name]
            g.last_uplink = now
            if kind == 'order':
                g.order = payload
                g.rally = payload['station']
                if flags['integrity']:
                    g.use_station_fixes(now, payload['nav'] == 'station')
            elif kind == 'fix':
                g.use_station_fixes(now, True)
                g.fix = payload  # Every fix carries the switch with it.
        if k % 2 == 0:
            for g in guardians.values():
                if g.landed:
                    continue
                link_age = now - g.last_uplink if flags['lost_link'] else 0.0
                order = g.order if flags['network'] else {'action': 'watch', 'target': g.post}
                if flags['reflex']:
                    fix_age = None if not g.nav_by_station else now - (g.fix[0] if g.fix else g.nav_since)
                    rally = [g.rally[0], g.rally[1], g.post[2]]  # Home at cruise altitude, never low.
                    action, target, layer, detail = onboard_decide(
                        now,
                        g.nav(now),
                        g.tracker.confirmed(),
                        link_age,
                        order,
                        CFG,
                        free=lambda p: math.hypot(p[0], p[1]) < 5000.0,
                        rally=rally,
                        previous=g.reflex,
                        fix_age=fix_age,
                    )
                    g.reflex = detail if (action, layer) == ('keep_clear', 'onboard') else None
                else:
                    action, target, layer, detail = order.get('action', 'watch'), order.get('target'), 'station', {}
                if layer == 'station' and order.get('authority'):
                    layer = order['authority']
                if action in ('hold', 'lost_link_hold'):
                    if g.action != action or g.hold_at is None:
                        g.hold_at = g.nav(now)
                    target = g.hold_at
                elif action == 'recover':
                    target = list(g.rally[:2]) + [20.0]
                if action == 'keep_clear' and layer == 'onboard':
                    if detail is not g.last_checked:
                        metrics['keep_clear'] += 1
                        g.last_checked = detail
                    check_move(now, g, target, g.tracker.confirmed())  # New or kept, every time it is re-decided.
                if (action, layer) != (g.action, g.layer):
                    log(now, layer, action, g.name, **({'miss': round(detail['miss'])} if 'miss' in detail else {}))
                    if (
                        s['jammer']
                        and g.name == s['jammer']['victim']
                        and layer == 'onboard'
                        and metrics['jam_action'] is None
                        and metrics['jam_lost'] is not None
                    ):
                        metrics['jam_action'] = now
                g.action, g.target, g.layer = action, target, layer
                if action == 'land_in_place':
                    g.landed = True
                    metrics['landed_in_place'] += 1
                if action in ('recover', 'lost_link_return') and target and math.dist(g.p[:2], target[:2]) < 30.0:
                    g.landed = True
            j = s['jammer']
            if (
                j
                and metrics['jam_lost'] is None
                and now >= j['on']
                and flags['network']
                and math.dist(guardians[j['victim']].p[:2], j['p']) < j['r']
            ):
                metrics['jam_lost'] = now
        # Station: own sensor, fusion of delivered reports, posture, orders, center.
        batch = []
        for item in sorted([m for m in downlink if m[0] <= now], key=lambda m: m[3]):
            downlink.remove(item)
            _, kind, name, t_meas, payload = item
            if kind == 'state':
                station['reports'][name], station['gnss'][name], station['velocity'][name] = payload
                station['heard'][name] = t_meas
            else:
                batch += [(t_meas, p, lab, sg) for p, _, lab, sg in payload]
        if now >= station['next_scan']:
            station['next_scan'] = now + STATION_SENSOR['period']
            seen = detect(
                station_rng,
                STATION_SENSOR,
                station['p'],
                objects + [(g.name, g.p) for g in guardians.values() if not g.landed],
                now,
                s['clutter_scale'],
                low_mask=True,
            )
            # Each friendly claims only its own return; every other return goes to the threat tracker, so a
            # threat passing close to a guardian is never absorbed as that guardian.
            claimed, observed = associate_friendlies(now, seen)
            batch += [(now, p, lab, sg) for i, (p, _, lab, sg) in enumerate(seen) if i not in claimed]
            for name, p in observed.items():
                if not flags['integrity']:
                    continue
                monitor = station['integrity'][name]
                before = monitor.state
                state = monitor.update(
                    now, station['gnss'].get(name), p, age=now - station['heard'].get(name, -math.inf)
                )
                if state != before:
                    log(
                        now,
                        'station',
                        'navigate_by_station' if state == 'spoofed' else 'navigate_by_gnss',
                        name,
                        residual=round(monitor.residual),
                    )
                    if state == 'spoofed' and name == victim and metrics['spoof_detect'] is None:
                        metrics['spoof_detect'] = now - s['spoof']['onset']
                    if state == 'spoofed' and name != victim:
                        metrics['false_integrity'] += 1
                if state == 'spoofed' and link_ok(now, guardians[name], 'fix'):
                    uplink.append((now + LINK_LATENCY, 'fix', name, (now, p)))
        for t_meas, p, lab, sg in sorted(batch, key=lambda b: b[0]):
            station['tracker'].update(max(t_meas, 0.0), [(p, 'fused', lab, sg)])
        tracks = station['tracker'].update(now, [])  # Prune tracks that have gone quiet, every step.
        for tr in tracks:
            if metrics['first_confirm'] is None and any(math.dist(tr.p, t.p) < 200.0 for t in real if t.active(now)):
                metrics['first_confirm'] = now
                metrics['first_confirm_range'] = math.dist(tr.p[:2], station['p'][:2])
        levels = [assess(tr, now, station['p'], station['v'], CFG, station['goal']) for tr in tracks]
        danger = [(tr, t) for tr, (l, t, _) in zip(tracks, levels) if l == 'danger']
        before = station['posture'].state
        if station['posture'].update(now, [l for l, _, _ in levels]):
            state = station['posture'].state
            log(now, 'station', 'posture', 'station', state=state)
            center.send(now, {'kind': 'posture', 'state': state})
            if state == RED:
                metrics['red'].append(now)
                log(now, 'station', 'crew_alert', 'station')
                # False unless some danger track is a real threat (birds, clutter, a friendly taken for a threat).
                if not any(math.dist(tr.predict(now), t.p) < 300.0 for tr, _ in danger for t in real if t.active(now)):
                    metrics['false_red'] += 1
                if danger:
                    tr = min(danger, key=lambda x: x[1])[0]
                    vx, vy = tr.v[0], tr.v[1]
                    n = math.hypot(vx, vy) or 1.0
                    side = 1.0 if (station['p'][0] - tr.p[0]) * (-vy) + (station['p'][1] - tr.p[1]) * vx >= 0 else -1.0
                    station['goal'] = [
                        station['p'][0] - vy / n * RELOCATE * side,
                        station['p'][1] + vx / n * RELOCATE * side,
                        0.0,
                    ]
                    log(now, 'station', 'relocate', 'station')
            if before == RED:
                station['held'] = True
                station['clear_since'] = now  # RED has cleared; only a new RED resets the question.
        # After a RED event, ask the center once RED has cleared (recall orders still wait for GREEN). Each clear gets its own request number, and only an answer to
        # the current one counts: an answer to an earlier question must not recall guardians after a new
        # event. With no answer by center_timeout, the station recovers on its delegated authority.
        if station['held'] and station['posture'].state != RED and station['recovery'] is None:
            if station['request'] is None:
                station['requests'] += 1
                station['request'] = station['requests']
                station['pending_since'] = -1e9
            if now - station['pending_since'] >= 5.0:  # Resent as often as the PX4 station resends it.
                center.send(now, {'kind': 'clear_after_red', 'request': station['request']})
                station['pending_since'] = now
            if now - station['clear_since'] >= CFG.center_timeout:
                station['recovery'] = 'recover'
                station['recovery_by'] = 'station (delegated)'
                metrics['delegated'] = True
                log(now, 'station (delegated)', 'recover', 'station')
        elif station['posture'].state == RED:
            station['request'] = None
        for arrival, decision, request in center.step(now):
            center_replies.append((arrival, decision, request))
        for item in [c for c in center_replies if c[0] <= now]:
            center_replies.remove(item)
            _, decision, request = item
            log(now, 'center', decision, 'center')
            if (
                decision in ('recover', 'resume')
                and station['recovery'] is None
                and request is not None
                and request == station['request']
                and station['posture'].state != RED
            ):
                station['recovery'] = decision
                station['recovery_by'] = 'center'
        if flags['network'] and now >= station['next_orders']:
            station['next_orders'] = now + 0.5
            # Orders go only to guardians heard from within three report periods, planned from each report
            # carried forward by its velocity: a silent guardian acts on its own authority, and an order
            # planned from an old report can send a guardian back toward a threat from where it really is.
            view = {
                n: {
                    'p': [
                        a + b * (now - station['heard'][n])
                        for a, b in zip(station['reports'][n], station['velocity'][n])
                    ],
                    'post': g.post,
                }
                for n, g in guardians.items()
                if not g.landed and n in station['reports'] and now - station['heard'][n] <= ORDER_AGE
            }
            # A hazard area stays declared for as long as RED would after its last danger, so a young track's
            # flickering assessment cannot drop a dispersing guardian back to holding halfway out.
            # Only a steady fast object declares an impact area, as on the PX4 station; slower threats are
            # handled by keep-clear moves.
            fast = [(tr, t) for tr, t in danger if steady_fast(tr, CFG)]
            if fast:
                station['hazard'] = (
                    (impact_point(min(fast, key=lambda x: x[1])[0], now, station['p']), 2 * CFG.protect_radius),
                    now,
                )
            hazard = station['hazard'][0] if station['hazard'] and now - station['hazard'][1] < CFG.clear_time else None
            orders = station_orders(
                now,
                station['posture'].state,
                tracks,
                view,
                CFG,
                free_for=lambda name: lambda q: math.hypot(q[0], q[1]) < 5000.0,
                recovery=station['recovery'],
                hazard=hazard,
                held=station['held'] and station['recovery'] is None,
                previous=station['orders'],
            )
            for name, order in orders.items():
                if order['action'] in ('keep_clear', 'disperse'):
                    if order['action'] == 'keep_clear' and order is not station['orders'].get(name):
                        metrics['keep_clear'] += 1
                    check_move(now, guardians[name], order['target'], tracks, own=view[name]['p'])
                if order['action'] in ('recover', 'resume'):
                    order['authority'] = station['recovery_by']
                station['rally'][name] = list(station['p'])
                nav = 'station' if station['integrity'][name].state == 'spoofed' else 'gnss'
                if link_ok(now, guardians[name], 'order'):
                    uplink.append(
                        (now + LINK_LATENCY, 'order', name, {**order, 'station': list(station['p']), 'nav': nav})
                    )
            station['orders'] = orders
        if trace and k % 10 == 0:
            samples.append(
                {
                    't': round(now),
                    'posture': station['posture'].state,
                    'station': [round(c) for c in station['p'][:2]],
                    'guardians': {n: [round(c) for c in g.p] + [g.action, int(g.landed)] for n, g in guardians.items()},
                    'reported': {
                        n: [round(c) for c in g.nav(now)[:2]]
                        for n, g in guardians.items()
                        if any(abs(o) > 1 for o in g.offset)
                    },
                    'threats': {t.name: [round(c) for c in t.p] for t in threats if t.active(now)},
                    'tracks': [[round(c) for c in tr.p[:2]] for tr in tracks],
                }
            )
    warning = None
    margin = None
    for t in real:
        # Warning runs to the moment the threat enters the zone or, if it never does, its closest approach.
        arrival = metrics['arrival'].get(t.name, metrics['cpa'].get(t.name, (0, None))[1])
        if arrival is None:
            continue
        red = [r for r in metrics['red'] if t.start <= r <= arrival]
        w = arrival - red[0] if red else 0.0
        warning = w if warning is None else min(warning, w)
    if warning is not None:
        margin = warning - (2 * CENTER_LATENCY + decision_time)
    jam = s['jammer']
    result = {
        'scenario': scenario_name,
        'architecture': architecture,
        'layout': layout,
        'seed': seed,
        'real_threats': len(real),
        'detected': metrics['first_confirm'] is not None if real else None,
        'first_confirm_range_m': None
        if metrics['first_confirm_range'] is None
        else round(metrics['first_confirm_range'], 1),
        'entered_zone': bool(metrics['arrival']),
        'warning_s': None if warning is None else round(warning, 1),
        'center_margin_s': None if margin is None or center_down else round(margin, 1),
        'min_separation_m': None if not real else round(metrics['min_separation'], 1),
        'guardians_safe': None if not real else metrics['min_separation'] >= CFG.safe_radius,
        'station_clear': None if not real else metrics['station_cpa'] >= CFG.protect_radius,
        'false_red': metrics['false_red'],
        'false_integrity': metrics['false_integrity'],
        'healthy_nav_error_m': round(metrics['nav_error'], 1),
        'landed_in_place': metrics['landed_in_place'],
        'jam_fallback_s': None
        if not jam or metrics['jam_lost'] is None or metrics['jam_action'] is None
        else round(metrics['jam_action'] - metrics['jam_lost'], 1),
        'spoof_detect_s': None if metrics['spoof_detect'] is None else round(metrics['spoof_detect'], 1),
        'spoof_drift_m': None if not s['spoof'] else round(metrics['spoof_drift'], 1),
        'keep_clear_decisions': metrics['keep_clear'],
        'never_closed': metrics['decision_closing'] == 0,
        'closed_on_truth': metrics['closing'] > 0,
        'authority_ok': metrics['authority_violations'] == 0,
        'delegated_recovery': metrics['delegated'],
        'center_decision_s': round(decision_time, 1),
    }
    if trace:
        result['trace'] = {
            'samples': samples,
            'events': events,
            'jammer': jam and {'p': [round(c) for c in jam['p']], 'r': jam['r']},
            'posts': {n: list(p) for n, p in posts.items()},
        }
    return result


def summarise(rows):
    def q(values, p):
        values = sorted(v for v in values if v is not None)
        return (
            None if not values else round(values[min(len(values) - 1, max(0, math.ceil(p / 100 * len(values)) - 1))], 1)
        )

    def rate(values):
        values = [v for v in values if v is not None]
        return None if not values else round(sum(bool(v) for v in values) / len(values), 3)

    out = {'runs': len(rows)}
    for key in (
        'warning_s',
        'center_margin_s',
        'min_separation_m',
        'first_confirm_range_m',
        'spoof_detect_s',
        'spoof_drift_m',
        'jam_fallback_s',
        'healthy_nav_error_m',
    ):
        vals = [r[key] for r in rows]
        if any(v is not None for v in vals):
            out[key] = {'p10': q(vals, 10), 'median': q(vals, 50), 'p90': q(vals, 90)}
    for key in ('detected', 'guardians_safe', 'station_clear', 'never_closed', 'authority_ok', 'closed_on_truth'):
        r = rate([row[key] for row in rows])
        if r is not None:
            out[key + '_rate'] = r
    margins = [r['center_margin_s'] for r in rows if r['center_margin_s'] is not None]
    if margins:
        out['center_in_time_rate'] = round(sum(m > 0 for m in margins) / len(margins), 3)
    out['invariants_rate'] = round(sum(r['never_closed'] and r['authority_ok'] for r in rows) / len(rows), 3)
    out['healthy_nav_error_max_m'] = max(r['healthy_nav_error_m'] for r in rows)
    for key in ('false_red', 'false_integrity', 'landed_in_place'):
        out[key + '_total'] = sum(r[key] for r in rows)
    return out


LABELS_MD = {
    'station_only': 'Station only',
    'onboard_only': 'Onboard only',
    'networked': 'Networked',
    'hybrid': 'Hybrid',
}


def markdown(report):
    """Results page generated from the report, so the published numbers cannot drift from the evidence."""
    table = report['summary']
    archs = list(ARCHITECTURES)
    f = lambda v, d=0: '—' if v is None else f'{v:.{d}f}'
    head = '| Scenario | ' + ' | '.join(LABELS_MD[a] for a in archs) + ' |\n|---|' + '---:|' * len(archs)
    lines = [
        '# Guardian simulator results',
        '',
        f"Generated by `tools/guardian_sim.py` from {report['seeds']} seeded runs per scenario and design (Python {report['python']}, {report['platform']}). "
        'The simulator is kinematic and its parameters are illustrative round numbers; [the design and evaluation](guardian.md) explains each design and what the numbers mean.',
        '',
        '## Guardians kept clear of every threat',
        '',
        f"Share of runs in which every airborne guardian stayed outside the {report['config']['safe_radius']:.0f} m safe radius of every real threat.",
        '',
        head,
    ]
    for s in SCENARIOS:
        lines.append(
            f'| {s} | '
            + ' | '.join(
                '—'
                if table[s][a].get('guardians_safe_rate') is None
                else f"{table[s][a]['guardians_safe_rate'] * 100:.0f}%"
                for a in archs
            )
            + ' |'
        )
    lines += [
        '',
        '## Warning before arrival',
        '',
        f"Median seconds from the station's RED alert to the threat entering the {report['config']['protect_radius']:.0f} m protected zone "
        "(or its closest approach, if the station moved out of the way); 10th to 90th percentile in brackets.",
        '',
        head,
    ]
    for s in SCENARIOS:
        cells = []
        for a in archs:
            w = table[s][a].get('warning_s')
            cells.append('—' if not w else f"{f(w['median'])} ({f(w['p10'])}–{f(w['p90'])})")
        lines.append(f'| {s} | ' + ' | '.join(cells) + ' |')
    lines += [
        '',
        '## Other measures',
        '',
        '| Measure | ' + ' | '.join(LABELS_MD[a] for a in archs) + ' |\n|---|' + '---:|' * len(archs),
    ]
    rows = [
        ('False RED alerts, birds scenario (total)', 'birds', lambda r: str(r['false_red_total'])),
        ('False RED alerts, spoofing scenario (total)', 'spoofing', lambda r: str(r['false_red_total'])),
        ('Jammed picket acts alone (median s)', 'jamming', lambda r: f(r.get('jam_fallback_s', {}).get('median'), 1)),
        ('GNSS drag-off detected (median s)', 'spoofing', lambda r: f(r.get('spoof_detect_s', {}).get('median'))),
        ('Spoofed picket drift (median m)', 'spoofing', lambda r: f(r.get('spoof_drift_m', {}).get('median'))),
        (
            'Center could decide in time, low intruder',
            'intruder',
            lambda r: '—' if r.get('center_in_time_rate') is None else f"{r['center_in_time_rate'] * 100:.0f}%",
        ),
    ]
    for label, s, get in rows:
        lines.append(f'| {label} | ' + ' | '.join(get(table[s][a]) for a in archs) + ' |')
    nav = lambda a: max(table[s][a]['healthy_nav_error_max_m'] for s in SCENARIOS)
    lines.append(
        '| Healthy guardian flagged as spoofed, all scenarios (total) | '
        + ' | '.join(str(sum(table[s][a]['false_integrity_total'] for s in SCENARIOS)) for a in archs)
        + ' |'
    )
    lines.append(
        '| Worst navigation error of a healthy guardian, all scenarios (m) | '
        + ' | '.join(f(nav(a)) for a in archs)
        + ' |'
    )
    runs = len(SCENARIOS) * len(archs) * report['seeds']
    held = sum(table[s][a]['invariants_rate'] * table[s][a]['runs'] for s in SCENARIOS for a in archs)
    lines.append(
        '| Runs where a move brought a guardian nearer a real threat, judged on truth | '
        + ' | '.join(
            f"{sum(table[s][a].get('closed_on_truth_rate', 0) * table[s][a]['runs'] for s in SCENARIOS):.0f}"
            for a in archs
        )
        + ' |'
    )
    lines += [
        '',
        f"Both invariants held in {held:.0f} of {runs} runs ({held / runs * 100:.1f}%). First: every keep-clear or dispersal decision, "
        "new or kept, opened the range along its whole path to every relevant slow track it was based on (not a bird, predicted "
        "to pass within twice the clear radius inside twice the planning horizon), and shortened no steady fast object's "
        "predicted miss. Second: no layer took an action outside its authority. Both are properties of the decisions, checked "
        "in every run. The row above measures the same thing on truth instead: a run counts if some move brought a guardian "
        f"more than {TRUTH_TOLERANCE:.0f} m nearer a real slow threat that a track was on and that truly passed within the clear "
        f"radius inside the planning horizon, or shortened a real fast object's true miss by more than {TRUTH_TOLERANCE:.0f} m. "
        "These come from straight-line predictions of weaving threats at long range; whether any guardian lost its safe radius "
        "is what the first table shows.",
        '',
        '## Layout experiment (hybrid design)',
        '',
        '| Layout | Low intruder warning (s) | Fast inbound kept clear | Fast inbound warning (s) | Swarm warning (s) |',
        '|---|---:|---:|---:|---:|',
    ]
    for k, r in report['layouts']['summary'].items():
        lines.append(
            f"| {k.replace('_', ' ')}: {report['layouts']['notes'][k]} | {f(r['intruder']['warning_s']['median'])} | "
            f"{r['fast_inbound']['guardians_safe_rate'] * 100:.0f}% | {f(r['fast_inbound']['warning_s']['median'], 1)} | {f(r['swarm']['warning_s']['median'])} |"
        )
    lines += [
        '',
        '## Reproduce',
        '',
        '```bash',
        f"python3 tools/guardian_sim.py --seeds {report['seeds']} --workers 8 --output artifacts/guardian",
        '```',
        '',
        'The report records the SHA-256 of `tools/guardian.py` and `tools/guardian_sim.py`; a test fails if either changes without regenerating it.',
    ]
    return '\n'.join(lines) + '\n'


# The replay carries seed 0 of the hybrid design and the one baseline that shows what changes in each scenario.
REPLAY_BASELINE = {
    'intruder': 'station_only',
    'fast_inbound': 'station_only',
    'swarm': 'station_only',
    'birds': 'station_only',
    'jamming': 'networked',
    'spoofing': 'networked',
    'center_loss': 'station_only',
    'combined': 'networked',
}


def compact(trace):
    """Columnar replay: flat integer arrays instead of one dictionary per second (a fraction of the size)."""
    samples = trace['samples']
    n = len(samples)
    actions = sorted({g[3] for x in samples for g in x['guardians'].values()})

    def series(key, name):
        idx = [i for i, x in enumerate(samples) if name in x[key]]
        return {
            'first': idx[0],
            'p': [
                c for i in range(idx[0], idx[-1] + 1) for c in samples[i][key].get(name, samples[idx[-1]][key][name])
            ],
        }

    return {
        'n': n,
        'posture': ''.join(x['posture'][0] for x in samples),
        'station': [c for x in samples for c in x['station']],
        'actions': actions,
        'events': trace['events'],
        'jammer': trace['jammer'],
        'posts': trace['posts'],
        'guardians': {
            g: {
                'p': [c for x in samples for c in x['guardians'][g][:3]],
                'a': [actions.index(x['guardians'][g][3]) for x in samples],
                'landed': [x['guardians'][g][4] for x in samples],
            }
            for g in samples[0]['guardians']
        },
        'threats': {t: series('threats', t) for t in sorted({t for x in samples for t in x['threats']})},
        'reported': {g: series('reported', g) for g in sorted({g for x in samples for g in x['reported']})},
        'tracks': [[c for tr in x['tracks'] for c in tr] for x in samples],
    }


def job(args):
    return run(*args)


def evaluate(seeds, workers=1):
    jobs = [(s, a, seed, seed == 0) for s in SCENARIOS for a in ARCHITECTURES for seed in range(seeds)]
    jobs += [
        (s, 'hybrid', seed, False, layout)
        for layout in LAYOUTS
        if layout != 'baseline'
        for s in EXPERIMENT_SCENARIOS
        for seed in range(seeds)
    ]
    if workers > 1:
        with Pool(workers) as pool:
            rows = pool.map(job, jobs, chunksize=4)
    else:
        rows = [job(j) for j in jobs]
    base = [r for r in rows if r['layout'] == 'baseline']
    table = {
        s: {a: summarise([r for r in base if r['scenario'] == s and r['architecture'] == a]) for a in ARCHITECTURES}
        for s in SCENARIOS
    }
    layouts = {
        layout: {
            s: summarise(
                [r for r in rows if r['layout'] == layout and r['scenario'] == s and r['architecture'] == 'hybrid']
            )
            for s in EXPERIMENT_SCENARIOS
        }
        for layout in LAYOUTS
    }
    rows = base + [r for r in rows if r['layout'] != 'baseline']
    traces = {
        s: {
            a: compact(
                next(r['trace'] for r in base if r['scenario'] == s and r['architecture'] == a and r['seed'] == 0)
            )
            for a in (REPLAY_BASELINE[s], 'hybrid')
        }
        for s in SCENARIOS
    }
    return table, layouts, traces, [{k: v for k, v in r.items() if k != 'trace'} for r in rows]


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--seeds', type=int, default=40)
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--output', type=Path, help='Write report.json, report-data.js and runs.jsonl here')
    p.add_argument('--verify', type=Path, help='Recompute and compare with a published report.json')
    args = p.parse_args()
    table, layouts, traces, rows = evaluate(args.seeds, args.workers)
    sources = {
        name: hashlib.sha256((ROOT / 'tools' / name).read_bytes()).hexdigest()
        for name in ('guardian.py', 'guardian_sim.py')
    }
    report = {
        'schema': 1,
        'description': 'Fast guardian simulator: seeded kinematic runs, illustrative parameters.',
        'seeds': args.seeds,
        'dt': DT,
        'scenarios': {s: DESCRIPTIONS[s] for s in SCENARIOS},
        'architectures': ARCHITECTURES,
        'config': CFG.__dict__,
        'sensors': {'station': STATION_SENSOR, 'guardian': DRONE_SENSOR},
        'posts': POSTS,
        'links': {'guardian_latency_s': LINK_LATENCY, 'guardian_loss': LINK_LOSS, 'center_latency_s': CENTER_LATENCY},
        'summary': table,
        'layouts': {'notes': LAYOUT_NOTES, 'posts': LAYOUTS, 'summary': layouts},
        'source_sha256': sources,
        'python': platform.python_version(),
        'platform': platform.system(),
    }
    if args.verify:
        published = json.loads(args.verify.read_text())
        if [published['summary'], published['layouts']['summary']] != json.loads(json.dumps([table, layouts])):
            sys.exit('Recomputed summary differs from ' + str(args.verify))
        print('Recomputed summary matches', args.verify)
        return
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / 'report.json').write_text(json.dumps(report, indent=1) + '\n', encoding='utf-8')
        (args.output / 'runs.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows), encoding='utf-8')
        (args.output / 'report-data.js').write_text(
            'window.GUARDIAN_REPORT=' + json.dumps({**report, 'traces': traces}, separators=(',', ':')) + ';\n',
            encoding='utf-8',
        )
        page = markdown(json.loads(json.dumps(report)))
        (args.output / 'guardian-results.md').write_text(page, encoding='utf-8')
        (ROOT / 'docs/guardian-results.md').write_text(page, encoding='utf-8')
    for s in SCENARIOS:
        for a in ARCHITECTURES:
            r = table[s][a]
            w = r.get('warning_s', {}).get('median')
            m = r.get('min_separation_m', {}).get('median')
            print(
                f"{s:12} {a:12} warn {w!s:>6} s  safe {r.get('guardians_safe_rate')!s:>5}  minsep {m!s:>6} m  "
                f"falseRED {r['false_red_total']:>3}  spoof {r.get('spoof_detect_s', {}).get('median')!s:>5}  "
                f"drift {r.get('spoof_drift_m', {}).get('median')!s:>6}  jam {r.get('jam_fallback_s', {}).get('median')!s:>5}  "
                f"closed {r.get('never_closed_rate')} auth {r.get('authority_ok_rate')}"
            )
    for layout, t in layouts.items():
        for s, r in t.items():
            print(
                f"layout {layout:16} {s:12} warn {r.get('warning_s', {}).get('median')!s:>6} s  safe {r.get('guardians_safe_rate')}  "
                f"minsep {r.get('min_separation_m', {}).get('median')}"
            )


if __name__ == '__main__':
    main()
