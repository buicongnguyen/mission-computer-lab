#!/usr/bin/env python3
"""Launch only task-owned local simulation processes, record evidence, then clean up."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from evidence_contracts import read_records, bag_has_topics
from perception import create_model
from provenance import capture
from security import provision, public_key_hex

SCENARIOS = (
    'nominal',
    'camera_dropout',
    'companion_crash',
    'gps_loss',
    'fleet_carrier',
)  # Guardian scenarios are appended below.
MULTI = ('fleet_carrier',)
# Three vehicles launch from pads on a carrier; each flies its own leg at its own altitude layer. The layers are
# 1.5 m apart: all three come home along the carrier, so one can pass under another holding over its pad, and at
# 1 m apart that pass measured 0.97 m against the 1 m separation minimum.
FLEET = [
    {'ns': 'px4_0', 'pad': -1.1, 'goal': (9, 9), 'altitude': 3.0},
    {'ns': 'px4_1', 'pad': 0.0, 'goal': (10, 3), 'altitude': 4.5},
    {'ns': 'px4_2', 'pad': 1.1, 'goal': (-1, 10), 'altitude': 6.0},
]
CARRIER_START = (0.0, -1.5)
sys.path.insert(0, str(ROOT / 'integration'))
import acceptance as A
from acceptance import DECK
import guardian_layout as GL

# Guardians hold watch posts instead of flying inspection legs; the same launch and landing machinery applies.
GUARDIAN_FLEET = [{'ns': g['ns'], 'pad': g['pad'], 'goal': g['post'], 'altitude': g['altitude']} for g in GL.GUARDIANS]
SCENARIOS += tuple(GL.SCENARIOS)
MULTI += tuple(GL.SCENARIOS)
# Adapter nodes judge freshness on Gazebo's clock, as PX4 does: recording video can slow the simulation
# below real time, and wall-clock freshness would then report healthy links as stale.
SIM_TIME = ['--ros-args', '-p', 'use_sim_time:=true']
CLOCK_BRIDGE = '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
SITL_PARAMETERS = {
    'COM_RC_IN_MODE': '4',
    'COM_RCL_EXCEPT': '4',
    'NAV_DLL_ACT': '2',
    'COM_OF_LOSS_T': '0.5',
    'COM_OBL_RC_ACT': '4',
    'COM_FAIL_ACT_T': '0',
    'COM_DISARM_LAND': '1',
    'SYS_FAILURE_EN': '1',
}


class Processes:
    def __init__(self, output, env):
        self.output = output
        self.env = env
        self.children = []
        self.handles = []

    def launch(self, name, args, cwd=None):
        stream = (self.output / (name + '.log')).open('w')
        self.handles.append(stream)
        p = subprocess.Popen(
            args,
            cwd=cwd,
            env=self.env,
            stdout=stream,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.children.append((name, p))
        return p

    def stop(self, p, sig=signal.SIGINT):
        # The leader can exit before its children. Its owned group still needs cleanup.
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=6)
        except subprocess.TimeoutExpired:
            pass
        finally:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.wait(timeout=3)

    def close(self):
        errors = []
        for name, p in reversed(self.children):
            try:
                self.stop(p)
            except Exception as exc:
                errors.append(f'{name}: {exc}')
        for h in self.handles:
            h.close()
        if errors:
            raise RuntimeError('Cleanup failed: ' + '; '.join(errors))


def record_video(env, path=None, service='/overview/record_video'):
    """Start recording a world camera to `path`, or stop when `path` is None."""
    request = f'start: true, format: "mp4", save_filename: "{path}"' if path else 'stop: true'
    reply = subprocess.run(
        [
            'gz',
            'service',
            '-s',
            service,
            '--reqtype',
            'gz.msgs.VideoRecord',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            request,
        ],
        env=env,
        capture_output=True,
        text=True,
        timeout=15,
    )
    if 'data: true' not in reply.stdout:
        raise RuntimeError(
            'Video recorder did not ' + ('start' if path else 'stop') + ': ' + (reply.stdout + reply.stderr).strip()
        )


def run_scenario(name, workspace, base_output, timeout, video=False, gui=False):
    output = base_output / name
    if output.exists():
        raise RuntimeError(f'Output already exists: {output}. Choose a fresh --output directory.')
    output.mkdir(parents=True)
    run_dir = workspace / 'runs' / f'{name}-{time.time_ns()}'
    run_dir.mkdir(parents=True)
    px4 = workspace / 'PX4-Autopilot'
    build = px4 / 'build/px4_sitl_default'
    env = {k: v for k, v in os.environ.items() if not k.startswith('PX4_PARAM_')}
    env.update(
        {
            'HEADLESS': '1',
            'GZ_IP': '127.0.0.1',
            'GZ_PARTITION': f'mission_{os.getpid()}_{name}',
            'GZ_SIM_RESOURCE_PATH': str(px4 / 'Tools/simulation/gz/models'),
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_MODEL_NAME': 'x500_0',
            'PX4_GZ_WORLD': 'inspection',
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'gz_x500',
            'PX4_PARAM_COM_RC_IN_MODE': '4',
            'PX4_PARAM_COM_RCL_EXCEPT': '4',
            'PX4_PARAM_NAV_DLL_ACT': '2',
            'PX4_PARAM_COM_OF_LOSS_T': '0.5',
            'PX4_PARAM_COM_OBL_RC_ACT': '4',
            'PX4_PARAM_COM_FAIL_ACT_T': '0',
            'PX4_PARAM_COM_DISARM_LAND': '1',
            'PX4_PARAM_SYS_FAILURE_EN': '1',
        }
    )
    processes = Processes(output, env)
    injected_at = None
    last_progress = 0
    error = None
    launched = time.monotonic()
    armed_seen = False
    flying_since = None
    recording = False
    try:
        agent_library_path = str(workspace / 'agent-install/lib') + ':' + env.get('LD_LIBRARY_PATH', '')
        processes.launch(
            'agent',
            [
                'env',
                'LD_LIBRARY_PATH=' + agent_library_path,
                str(workspace / 'agent-install/bin/MicroXRCEAgent'),
                'udp4',
                '-p',
                '8888',
                '-v',
                '3',
            ],
        )
        # Headless EGL rendering serves the overview camera; it does not change the flight physics. The
        # recorder encodes to a temporary file in the server's working directory and renames it on stop,
        # so run the server in the output folder (a rename cannot cross from /mnt/c to the Linux disk).
        processes.launch(
            'gazebo',
            [
                'gz',
                'sim',
                '-r',
                '-s',
                '-v',
                '2',
                '--headless-rendering',
                str(ROOT / 'simulation/worlds/inspection.sdf'),
            ],
            cwd=output,
        )
        processes.launch('px4', [str(build / 'bin/px4'), '-d', str(build / 'etc'), '-w', str(run_dir)])
        processes.launch('gcs', [sys.executable, str(ROOT / 'integration/gcs_heartbeat.py')])
        if gui:
            processes.launch('gui', ['gz', 'sim', '-g'])  # Live 3D view (WSLg); closing it does not fail the run.
        # Airframe startup can overwrite environment parameter overrides. Apply
        # and record the final simulation policy only after rcS has completed.
        startup_deadline = time.monotonic() + 40
        while 'Startup script returned successfully' not in (output / 'px4.log').read_text():
            if time.monotonic() > startup_deadline:
                raise RuntimeError('PX4 startup timeout')
            if any(p.poll() is not None for _, p in processes.children):
                raise RuntimeError('Startup process exited')
            time.sleep(0.2)
        with (output / 'parameters.log').open('w') as parameter_log:
            for key, value in env.items():
                if key.startswith('PX4_PARAM_'):
                    parameter = key.removeprefix('PX4_PARAM_')
                    subprocess.run(
                        [str(build / 'bin/px4-param'), 'set', parameter, value],
                        env=env,
                        stdout=parameter_log,
                        stderr=subprocess.STDOUT,
                        check=True,
                        timeout=5,
                    )
                    subprocess.run(
                        [str(build / 'bin/px4-param'), 'show', parameter],
                        env=env,
                        stdout=parameter_log,
                        stderr=subprocess.STDOUT,
                        check=True,
                        timeout=5,
                    )
        if video:
            record_video(env, output / 'flight.mp4')
            recording = True
        processes.launch(
            'observer',
            [sys.executable, str(ROOT / 'integration/observer_node.py'), '--log', str(output / 'observer.jsonl')],
        )
        processes.launch('clock', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', CLOCK_BRIDGE])
        payload = processes.launch(
            'payload',
            [sys.executable, str(ROOT / 'integration/payload_node.py'), '--camera-drop-for', '0.8', *SIM_TIME],
        )
        # Act as the release authority: sign the stored model, keep the private key in memory and
        # hand the node only a public key stored outside the artifact folder.
        model = output / 'detector.onnx'
        create_model(model)
        release_key = Ed25519PrivateKey.generate()
        provision(model, release_key)
        trust_anchor = run_dir / 'trust-anchor.pub'
        trust_anchor.write_text(public_key_hex(release_key) + '\n')
        del release_key
        processes.launch(
            'perception',
            [
                sys.executable,
                str(ROOT / 'integration/perception_node.py'),
                '--model',
                str(model),
                '--public-key',
                str(trust_anchor),
                '--log',
                str(output / 'perception.jsonl'),
            ],
        )
        mission = processes.launch(
            'mission',
            [
                sys.executable,
                str(ROOT / 'integration/mission_node.py'),
                '--allow-sitl',
                '--log',
                str(output / 'mission.jsonl'),
                '--model-sha256',
                hashlib.sha256(model.read_bytes()).hexdigest(),
                *SIM_TIME,
            ],
        )
        # Capture selected typed application topics; raw firmware evidence is independently logged.
        processes.launch(
            'rosbag',
            [
                'ros2',
                'bag',
                'record',
                '-o',
                str(output / 'rosbag'),
                '/mission/decision',
                '/mission/perception',
                '/mission/lidar/scan',
            ],
        )
        while time.monotonic() - launched < timeout:
            time.sleep(0.25)
            failures = [
                n
                for n, p in processes.children
                if p.poll() is not None
                and n != 'gui'
                and not (n == 'mission' and name == 'companion_crash' and injected_at)
            ]
            if failures:
                raise RuntimeError('Processes exited: ' + ', '.join(failures))
            records = read_records(output / 'observer.jsonl', live=True)
            states = [r for r in records if r['kind'] == 'status']
            poses = [r for r in records if r['kind'] == 'position']
            if states:
                state = states[-1]
                armed_seen |= state['arming_state'] == 2
                if state['arming_state'] == 2 and poses and -poses[-1]['ned'][2] > 2 and flying_since is None:
                    flying_since = time.monotonic()
                if name != 'nominal' and flying_since and time.monotonic() - flying_since > 3 and injected_at is None:
                    injected_at = time.time()
                    if name == 'companion_crash':
                        processes.stop(mission, signal.SIGKILL)
                    elif name == 'camera_dropout':
                        os.kill(payload.pid, signal.SIGUSR1)  # In flight, not at a fixed uptime.
                    else:
                        # v1.16's Gazebo bridge does not acknowledge `failure gps
                        # off`. Its supported SIM_GPS_USED parameter drives fix
                        # validity in the actual GPS sensor publication instead.
                        command = subprocess.run(
                            [str(build / 'bin/px4-param'), 'set', 'SIM_GPS_USED', '0'],
                            env=env,
                            capture_output=True,
                            text=True,
                            timeout=5,
                        )
                        (output / 'gps-injection.log').write_text(command.stdout + command.stderr)
                        if command.returncode:
                            raise RuntimeError('GPS injection command failed')
                    print(name, 'fault injected', flush=True)
                lands = [r for r in records if r['kind'] == 'land']
                landed = bool(lands and lands[-1]['landed'] and lands[-1]['wall_time'] >= state['wall_time'])
                if armed_seen and state['arming_state'] == 1 and landed:
                    break
            if time.monotonic() - last_progress > 15:
                last_progress = time.monotonic()
                print(
                    name,
                    'elapsed',
                    round(time.monotonic() - launched),
                    's',
                    states[-1] if states else 'waiting for DDS telemetry',
                    flush=True,
                )
        else:
            error = 'scenario_timeout'
    except Exception as exc:
        error = str(exc)
        (output / 'runner-error.log').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        if recording:
            # Stop while Gazebo still runs, then give the encoder a moment to finalize the file.
            try:
                record_video(env)
                for _ in range(50):
                    if (output / 'flight.mp4').is_file() and (output / 'flight.mp4').stat().st_size:
                        break
                    time.sleep(0.1)
            except Exception as exc:
                error = (error + '; ' if error else '') + str(exc)
        try:
            processes.close()
        except Exception as exc:
            error = (error + '; ' if error else '') + str(exc)
    records = read_records(output / 'observer.jsonl')
    mission_records = read_records(output / 'mission.jsonl')
    checks, facts = A.single_flight(
        name, records, mission_records, armed_seen, injected_at, error, bag_has_topics(output / 'rosbag')
    )
    result = {
        'scenario': name,
        'passed': all(checks.values()),
        'checks': checks,
        'error': error,
        'duration_wall_s': round(time.monotonic() - launched, 2),
        'max_altitude_m': facts['max_alt'],
        'minimum_estimated_clearance_m': facts['min_clearance'],
        'injected_at': injected_at,
        'statuses': facts['statuses'],
        'acks': facts['acks'],
        'message_counts': facts['counts'][-1] if facts['counts'] else {},
        'trace': facts['poses'],
        'decisions': facts['decisions'],
        'mission_transitions': facts['transitions'],
        'gps_states': [r for r in records if r['kind'] == 'gps'],
        'plan': next((r for r in mission_records if r['kind'] == 'plan'), None),
        'replans': [r for r in mission_records if r['kind'] == 'replan'],
        'video': 'flight.mp4' if (output / 'flight.mp4').is_file() else None,
    }
    (output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    print(name, 'PASS' if result['passed'] else 'FAIL', json.dumps(checks), error or '', flush=True)
    return result


def run_fleet(workspace, base_output, timeout, video=False, gui=False, name='fleet_carrier'):
    """Three PX4 instances on a carrier: the fleet inspection legs, or one of the guardian scenarios."""
    guardian = name in GL.SCENARIOS
    FLEET_ = GUARDIAN_FLEET if guardian else FLEET
    world = 'guardian' if guardian else 'fleet'
    scenario = GL.SCENARIOS.get(name, {})
    output = base_output / name
    if output.exists():
        raise RuntimeError(f'Output already exists: {output}. Choose a fresh --output directory.')
    output.mkdir(parents=True)
    stamp = time.time_ns()
    px4 = workspace / 'PX4-Autopilot'
    build = px4 / 'build/px4_sitl_default'
    env = {k: v for k, v in os.environ.items() if not k.startswith('PX4_PARAM_')}
    env.update(
        {
            'HEADLESS': '1',
            'GZ_IP': '127.0.0.1',
            'GZ_PARTITION': f'mission_{os.getpid()}_{name}',
            'GZ_SIM_RESOURCE_PATH': str(px4 / 'Tools/simulation/gz/models'),
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': world,
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'gz_x500',
            **{'PX4_PARAM_' + k: v for k, v in SITL_PARAMETERS.items()},
        }
    )
    processes = Processes(output, env)
    error = None
    launched = time.monotonic()
    recording = []
    last_progress = 0
    spawns = {v['ns']: [CARRIER_START[0] + v['pad'], CARRIER_START[1], DECK] for v in FLEET_}
    try:
        agent_library_path = str(workspace / 'agent-install/lib') + ':' + env.get('LD_LIBRARY_PATH', '')
        processes.launch(
            'agent',
            [
                'env',
                'LD_LIBRARY_PATH=' + agent_library_path,
                str(workspace / 'agent-install/bin/MicroXRCEAgent'),
                'udp4',
                '-p',
                '8888',
                '-v',
                '3',
            ],
        )
        world_file = ROOT / 'simulation/worlds/fleet.sdf'
        if guardian:  # Each guardian scenario's world carries its own threat models, markers and jamming zone.
            world_file = output / 'world.sdf'
            world_file.write_text(GL.world_sdf(name), encoding='utf-8')
        processes.launch(
            'gazebo', ['gz', 'sim', '-r', '-s', '-v', '2', '--headless-rendering', str(world_file)], cwd=output
        )
        # One PX4 per airframe: instance N attaches to x500_N, uses DDS namespace px4_N and system ID N+1.
        for i, v in enumerate(FLEET_):
            run_dir = workspace / 'runs' / f'{name}-{v["ns"]}-{stamp}'
            run_dir.mkdir(parents=True)
            processes.launch(
                f'px4_{i}',
                [
                    'env',
                    f'PX4_GZ_MODEL_NAME=x500_{i}',
                    f'PX4_UXRCE_DDS_NS={v["ns"]}',
                    str(build / 'bin/px4'),
                    '-i',
                    str(i),
                    '-d',
                    str(build / 'etc'),
                    '-w',
                    str(run_dir),
                ],
            )
            time.sleep(1)
        processes.launch(
            'gcs',
            [
                sys.executable,
                str(ROOT / 'integration/gcs_heartbeat.py'),
                '--ports',
                ','.join(str(18570 + i) for i in range(len(FLEET_))),
            ],
        )
        if gui:
            processes.launch('gui', ['gz', 'sim', '-g'])
        startup_deadline = time.monotonic() + 60
        while not all(
            'Startup script returned successfully' in (output / f'px4_{i}.log').read_text() for i in range(len(FLEET_))
        ):
            if time.monotonic() > startup_deadline:
                raise RuntimeError('PX4 startup timeout')
            if any(p.poll() is not None for n, p in processes.children if n != 'gui'):
                raise RuntimeError('Startup process exited')
            time.sleep(0.2)
        with (output / 'parameters.log').open('w') as parameter_log:
            for i in range(len(FLEET_)):
                for parameter, value in SITL_PARAMETERS.items():
                    for action in (['set', parameter, value], ['show', parameter]):
                        subprocess.run(
                            [str(build / 'bin/px4-param'), '--instance', str(i), *action],
                            env=env,
                            stdout=parameter_log,
                            stderr=subprocess.STDOUT,
                            check=True,
                            timeout=5,
                        )
        # The carrier's odometry and drive command cross between Gazebo and ROS 2 through the bridge.
        # Guardian scenarios also bridge each threat's odometry and drive, and each airframe's true pose.
        intruder_topics = (
            (
                [
                    t
                    for m in scenario.get('threats', {})
                    for t in (
                        f'/model/{m}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        f'/model/{m}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    )
                ]
                + [f'/model/x500_{i}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' for i in range(len(FLEET_))]
            )
            if guardian
            else []
        )
        processes.launch(
            'bridge',
            [
                'ros2',
                'run',
                'ros_gz_bridge',
                'parameter_bridge',
                CLOCK_BRIDGE,
                '/model/carrier/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/carrier/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                *intruder_topics,
            ],
        )
        if video and (not guardian or scenario.get('video')):  # The guardian scenarios record one headline flight.
            cameras = (
                (('flight.mp4', '/overview/record_video'), ('close.mp4', '/close/record_video'))
                if guardian
                else (('flight.mp4', '/overview/record_video'), ('deck.mp4', '/deck/record_video'))
            )
            for file, service in cameras:
                record_video(env, output / file, service)
                recording.append((file, service))
        model = output / 'detector.onnx'
        create_model(model)
        release_key = Ed25519PrivateKey.generate()
        provision(model, release_key)
        trust_anchor = workspace / 'runs' / f'{name}-trust-anchor-{stamp}.pub'
        trust_anchor.write_text(public_key_hex(release_key) + '\n')
        del release_key
        model_sha256 = hashlib.sha256(model.read_bytes()).hexdigest()
        if guardian:
            # The environment (intruder, sensors, jammed link) and the center are stand-ins, not software under test.
            processes.launch(
                'world',
                [
                    sys.executable,
                    str(ROOT / 'integration/guardian_world_node.py'),
                    '--log',
                    str(output / 'world.jsonl'),
                    '--scenario',
                    name,
                    *SIM_TIME,
                ],
            )
            processes.launch(
                'center',
                [
                    sys.executable,
                    str(ROOT / 'integration/center_node.py'),
                    '--log',
                    str(output / 'center.jsonl'),
                    *(['--unreachable'] if scenario.get('center_down') else []),
                    *SIM_TIME,
                ],
            )
        for i, v in enumerate(FLEET_):
            ns = v['ns']
            spawn = [str(c) for c in spawns[ns]]
            processes.launch(
                f'observer_{ns}',
                [
                    sys.executable,
                    str(ROOT / 'integration/observer_node.py'),
                    '--ns',
                    ns,
                    '--log',
                    str(output / f'observer_{ns}.jsonl'),
                ],
            )
            processes.launch(
                f'payload_{ns}',
                [sys.executable, str(ROOT / 'integration/payload_node.py'), '--ns', ns, '--spawn', *spawn, *SIM_TIME],
            )
            processes.launch(
                f'perception_{ns}',
                [
                    sys.executable,
                    str(ROOT / 'integration/perception_node.py'),
                    '--ns',
                    ns,
                    '--model',
                    str(model),
                    '--public-key',
                    str(trust_anchor),
                    '--log',
                    str(output / f'perception_{ns}.jsonl'),
                ],
            )
            processes.launch(
                f'mission_{ns}',
                [
                    sys.executable,
                    str(ROOT / f'integration/{"guardian" if guardian else "fleet"}_mission_node.py'),
                    '--allow-sitl',
                    '--ns',
                    ns,
                    '--spawn',
                    *spawn,
                    '--goal',
                    *[str(g) for g in v['goal']],
                    '--altitude',
                    str(v['altitude']),
                    '--pad',
                    str(v['pad']),
                    '--system-id',
                    str(i + 1),
                    '--model-sha256',
                    model_sha256,
                    '--log',
                    str(output / f'mission_{ns}.jsonl'),
                    *SIM_TIME,
                ],
            )
        processes.launch(
            'station',
            [
                sys.executable,
                str(ROOT / f'integration/{"guardian" if guardian else "fleet"}_station_node.py'),
                '--drones',
                ','.join(v['ns'] for v in FLEET_),
                '--log',
                str(output / 'station.jsonl'),
                *(['--scenario', name] if guardian else []),
                *SIM_TIME,
            ],
        )
        station_topics = ['/station/uplink', '/station/fixes', '/center/decision'] if guardian else ['/fleet/clearance']
        processes.launch(
            'rosbag',
            [
                'ros2',
                'bag',
                'record',
                '-o',
                str(output / 'rosbag'),
                *[f'/{v["ns"]}/mission/decision' for v in FLEET_],
                *station_topics,
                '/model/carrier/odometry',
            ],
        )
        while time.monotonic() - launched < timeout:
            time.sleep(0.5)
            failures = [n for n, p in processes.children if p.poll() is not None and n != 'gui']
            if failures:
                raise RuntimeError('Processes exited: ' + ', '.join(failures))
            station = read_records(output / 'station.jsonl', live=True)
            touchdowns = {r['drone'] for r in station if r['kind'] == 'touchdown'}
            if touchdowns == {v['ns'] for v in FLEET_}:
                time.sleep(3)
                break  # Let the carrier stop and the logs settle.
            if time.monotonic() - last_progress > 15:
                last_progress = time.monotonic()
                carrier = next((r for r in reversed(station) if r['kind'] == 'carrier'), None)
                print(
                    name,
                    'elapsed',
                    round(time.monotonic() - launched),
                    's touchdowns',
                    sorted(touchdowns),
                    'carrier',
                    carrier and round(carrier['e'], 1),
                    flush=True,
                )
        else:
            error = 'scenario_timeout'
    except Exception as exc:
        error = str(exc)
        (output / 'runner-error.log').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        for file, service in recording:
            try:
                record_video(env, None, service)
                for _ in range(50):
                    if (output / file).is_file() and (output / file).stat().st_size:
                        break
                    time.sleep(0.1)
            except Exception as exc:
                error = (error + '; ' if error else '') + str(exc)
        try:
            processes.close()
        except Exception as exc:
            error = (error + '; ' if error else '') + str(exc)
    station = read_records(output / 'station.jsonl')
    carrier = [r for r in station if r['kind'] == 'carrier']
    grants = [r for r in station if r['kind'] == 'clearance']
    touchdowns = {r['drone']: r for r in station if r['kind'] == 'touchdown'}
    world_log = read_records(output / 'world.jsonl') if guardian else []
    truth = A.guardian_truth(world_log)[1] if guardian else {}
    vehicle_checks, vehicles = {}, []
    for v in FLEET_:
        ns = v['ns']
        own, vehicle = A.fleet_vehicle(
            v,
            spawns[ns],
            read_records(output / f'observer_{ns}.jsonl'),
            read_records(output / f'mission_{ns}.jsonl'),
            touchdowns.get(ns),
            carrier,
            truth[ns] if guardian else None,
        )
        vehicle_checks.update(own)
        vehicles.append(vehicle)
    # Separation from the independent observers (or, for guardians, truth), time-aligned at 5 Hz while two are airborne.
    min_separation = A.min_separation(
        [truth[v['ns']] for v in vehicles] if guardian else [v['trace'] for v in vehicles], 1 if guardian else 2
    )
    topics = [f'/{v["ns"]}/mission/decision' for v in FLEET_] + (
        ['/station/uplink'] if guardian else ['/fleet/clearance']
    )
    checks = A.fleet_flight(
        vehicle_checks,
        vehicles,
        station,
        min_separation,
        guardian=guardian,
        error=error,
        bag_recorded=bag_has_topics(output / 'rosbag', topics),
        dropped=[
            r for log in sorted(output.glob('*.jsonl')) for r in read_records(log) if r.get('kind') == 'dropped_message'
        ],
    )
    extra = (
        A.guardian_flight(name, world_log, read_records(output / 'center.jsonl'), station, vehicles, carrier, checks)
        if guardian
        else {}
    )
    result = {
        'scenario': name,
        'passed': all(checks.values()),
        'checks': checks,
        'error': error,
        'duration_wall_s': round(time.monotonic() - launched, 2),
        'vehicles': vehicles,
        'carrier': carrier,
        'clearances': grants,
        'min_separation_m': min_separation,
        'deck_height_m': DECK,
        **extra,
        'videos': {
            k: f
            for k, f in (('overview', 'flight.mp4'), ('deck', 'deck.mp4'), ('close', 'close.mp4'))
            if (output / f).is_file()
        },
    }
    (output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    print(name, 'PASS' if result['passed'] else 'FAIL', json.dumps(checks), error or '', flush=True)
    return result


def main():
    # Children run in their own sessions, so a closed terminal or a CI cancel must still reach the
    # `finally` cleanup; otherwise PX4, Gazebo and the agent survive and block the next run.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda number, frame: sys.exit(128 + number))
    p = argparse.ArgumentParser()
    p.add_argument('--workspace', type=Path, required=True)
    p.add_argument('--scenario', choices=SCENARIOS, default='nominal')
    p.add_argument('--all', action='store_true')
    p.add_argument(
        '--keep-going',
        action='store_true',
        help='Run remaining scenarios after a failed case; failures still exit nonzero',
    )
    p.add_argument('--output', type=Path, default=ROOT / 'artifacts/sitl-latest')
    p.add_argument('--timeout', type=float, default=150.0)
    p.add_argument(
        '--video', action='store_true', help='Record single-drone, fleet and configured guardian demonstration cameras'
    )
    p.add_argument('--gui', action='store_true', help='Open the live Gazebo 3D window (WSLg) while flying')
    args = p.parse_args()
    workspace = args.workspace.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        p.error('--timeout must be finite and positive')
    if any(out.iterdir()):
        p.error('--output must be an empty directory to preserve prior evidence')
    provenance = {'captured_at_unix': time.time(), 'environment': capture(workspace)}
    (out / 'provenance.json').write_text(json.dumps(provenance, indent=2))
    results = []
    # The guard is wall-clock; recording runs the simulation at roughly 0.4-0.6x real time.
    timeout = args.timeout * (2.5 if args.video else 1.0)
    for name in SCENARIOS if args.all else (args.scenario,):
        if name in MULTI:
            results.append(run_fleet(workspace, out, max(timeout, 900.0), video=args.video, gui=args.gui, name=name))
        else:
            results.append(run_scenario(name, workspace, out, timeout, video=args.video, gui=args.gui))
        if not results[-1]['passed'] and not args.keep_going:
            break
    (out / 'results.json').write_text(json.dumps(results, indent=2, allow_nan=False))
    provenance['inputs_unchanged'] = provenance['environment'] == capture(workspace)
    (out / 'provenance.json').write_text(json.dumps(provenance, indent=2))
    return 0 if provenance['inputs_unchanged'] and all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
