#!/usr/bin/env python3
"""Publish compact, measured SITL evidence; leave bags and upstream builds local."""

import argparse
import sys
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import tempfile
from evidence_contracts import require_matrix, read_records, strict_loads
from world import OBSTACLES

ROOT = Path(__file__).resolve().parents[1]
FILES = (
    'result.json',
    'parameters.log',
    'perception.jsonl',
    'mission.jsonl',
    'gps-injection.log',
    'px4.log',
    'flight.mp4',
)
FLEET = 'fleet_carrier'
FLEET_NAMES = ('px4_0', 'px4_1', 'px4_2')
FLEET_FILES = ('result.json', 'parameters.log', 'station.jsonl', 'flight.mp4', 'deck.mp4')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'integration'))
from acceptance import required_fleet, required_single
import guardian_layout as GL

GUARDIANS = tuple(GL.SCENARIOS)
GUARDIAN_FILES = (
    'result.json',
    'parameters.log',
    'station.jsonl',
    'world.jsonl',
    'center.jsonl',
    'world.sdf',
    'flight.mp4',
    'close.mp4',
)


def required_checks(result):
    """Named checks a multi-vehicle result must carry: the fleet's fixed list, or what each guardian scenario expects."""
    if result['scenario'] == FLEET:
        return required_fleet(FLEET_NAMES)
    return set(GL.expected_checks(result['scenario']))


def source_logs(result):
    return ('station.jsonl',) if result['scenario'] == FLEET else ('station.jsonl', 'world.jsonl', 'center.jsonl')


def validate_fleet(result, folder):
    names = validate_vehicle_ids(result)
    if not required_checks(result) <= result['checks'].keys():
        raise ValueError('Missing required checks: ' + result['scenario'])
    for name in ('parameters.log', *source_logs(result), *[f'mission_{ns}.jsonl' for ns in names]):
        if not (folder / name).is_file() or not (folder / name).stat().st_size:
            raise ValueError('Missing source evidence: ' + str(folder / name))
        if name.endswith('.jsonl'):
            read_records(folder / name)
    if any(not v['trace'] for v in result['vehicles']) or not result['carrier']:
        raise ValueError('Empty fleet trace')
    if not isinstance(result['min_separation_m'], (int, float)) or not math.isfinite(result['min_separation_m']):
        raise ValueError('Invalid fleet separation')
    result['replay_start_wall_time'] = min(v['trace'][0]['wall_time'] for v in result['vehicles'])


def validate_vehicle_ids(result):
    expected = set(FLEET_NAMES if result['scenario'] == FLEET else (g['ns'] for g in GL.GUARDIANS))
    names = [v['ns'] for v in result['vehicles']]
    if len(names) != len(expected) or set(names) != expected:
        raise ValueError('Expected exactly one of every configured vehicle: ' + result['scenario'])
    return names


def validate_guardian(result):
    """A guardian flight must carry Gazebo truth for every vehicle and every scripted threat, starting with the
    scenario: without it a flight in which nothing flew could pass every check it names."""
    validate_vehicle_ids(result)
    if set(result['threats']) != set(GL.SCENARIOS[result['scenario']]['threats']):
        raise ValueError('Threat identities differ from scenario: ' + result['scenario'])
    if any(not v.get('truth') for v in result['vehicles']):
        raise ValueError('Missing guardian truth: ' + result['scenario'])
    start = (result.get('scenario_start') or {}).get('t')
    if start is None:
        raise ValueError('Scenario never started: ' + result['scenario'])
    for model, threat in result['threats'].items():
        delay = GL.SCENARIOS[result['scenario']]['threats'][model].get('delay', 0.0)
        if not threat['trace'] or threat['trace'][0]['t'] - (start + delay) > 2.0:
            raise ValueError(f"Threat {model} has no truth from its start: " + result['scenario'])


# The guardian page reads only these vehicle fields; the full result stays in each flight's result.json.
GUARDIAN_VEHICLE_FIELDS = ('ns', 'pad', 'goal', 'altitude', 'truth', 'touchdown', 'guardian', 'min_clearance_m')


def slim(value, digits=3):
    """Round floats (positions to a millimetre) so eight flights stay a reasonable page payload."""
    if isinstance(value, float):
        return round(value, digits)
    if isinstance(value, list):
        return [slim(v, digits) for v in value]
    if isinstance(value, dict):
        return {k: (v if k == 'wall_time' else slim(v, digits)) for k, v in value.items()}
    return value


def slim_guardian(result):
    out = {k: v for k, v in result.items() if k != 'vehicles'}
    out['vehicles'] = [{k: v[k] for k in GUARDIAN_VEHICLE_FIELDS if k in v} for v in result['vehicles']]
    return slim(out)


def nearest_rank(values, p):
    values = sorted(values)
    return values[max(0, math.ceil(p / 100 * len(values)) - 1)] if values else None


def recover_sample(sample):
    """Recover a process interruption between retirement and promotion before touching new evidence."""
    retired = sample.with_name('.sitl-sample-retired')
    if retired.exists() and not sample.exists():
        retired.rename(sample)


def replace_sample(staging, sample):
    """Promote a complete directory, rolling back if promotion raises; retain backups on cleanup errors."""
    recover_sample(sample)
    retired = sample.with_name('.sitl-sample-retired')
    if retired.exists():
        shutil.rmtree(retired)  # A current sample exists after recovery.
    if sample.exists():
        sample.rename(retired)
    try:
        staging.rename(sample)
    except BaseException:
        recover_sample(sample)
        raise
    if retired.exists():
        shutil.rmtree(retired)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, help='Legacy option; environment is read from run-time provenance')
    args = parser.parse_args()
    source = args.input.resolve()
    recover_sample(ROOT / 'artifacts/sitl-sample')
    results = strict_loads((source / 'results.json').read_text())
    expected = {'nominal', 'camera_dropout', 'companion_crash', 'gps_loss', FLEET, *GUARDIANS}
    require_matrix(results, expected)
    provenance = strict_loads((source / 'provenance.json').read_text())
    if provenance.get('inputs_unchanged') is not True:
        raise ValueError('Runtime inputs changed during the experiment; rerun the matrix')
    # Validate and derive everything before the reference sample is touched.
    for result in results:
        if result['scenario'] in (FLEET, *GUARDIANS):
            folder = source / result['scenario']
            if strict_loads((folder / 'result.json').read_text()) != result:
                raise ValueError('Summary differs from per-scenario evidence: ' + result['scenario'])
            validate_fleet(result, folder)
            if result['scenario'] in GUARDIANS:
                validate_guardian(result)
            continue
        required = required_single(result['scenario'])
        if not required <= result['checks'].keys():
            raise ValueError('Missing required checks: ' + result['scenario'])
        folder = source / result['scenario']
        if strict_loads((folder / 'result.json').read_text()) != result:
            raise ValueError('Summary differs from per-scenario evidence: ' + result['scenario'])
        for name in ('parameters.log', 'px4.log', 'perception.jsonl', 'mission.jsonl'):
            if not (folder / name).is_file() or not (folder / name).stat().st_size:
                raise ValueError('Missing source evidence: ' + str(folder / name))
        if not result['trace']:
            raise ValueError('Empty flight trace: ' + result['scenario'])
        for metric in ('duration_wall_s', 'max_altitude_m', 'minimum_estimated_clearance_m'):
            if not isinstance(result[metric], (int, float)) or not math.isfinite(result[metric]):
                raise ValueError(f'Invalid {metric}: ' + result['scenario'])
        read_records(folder / 'mission.jsonl')
        inference = [r['inference_ms'] for r in read_records(folder / 'perception.jsonl') if r['kind'] == 'inference']
        result['onnx_sampled_p95_ms'] = nearest_rank(inference, 95)
        result['onnx_timing_note'] = 'One logged timing per ten frames; CPU synthetic graph; not an NPU benchmark.'
        result['replay_start_wall_time'] = result['trace'][0]['wall_time']
    environment = provenance['environment']
    revisions = environment['upstream_revisions']
    report = {
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'environment': environment,
        'provenance': provenance,
        'boundary': 'Actual PX4/Gazebo/ROS; procedural camera/lidar; CPU synthetic ONNX; no Qualcomm hardware.',
        'obstacles': [list(o) for o in OBSTACLES],
        'results': [r for r in results if r['scenario'] not in (FLEET, *GUARDIANS)],
        'fleet': next(r for r in results if r['scenario'] == FLEET),
        'guardians': {name: slim_guardian(next(r for r in results if r['scenario'] == name)) for name in GUARDIANS},
    }
    serialized = json.dumps(report, indent=2, allow_nan=False)
    # The single-drone and fleet pages load report-data.js; only the guardian page needs the eight flights.
    page = {k: v for k, v in report.items() if k != 'guardians'}
    # Stage a complete sample, then promote with rollback. No earlier run's files survive the replacement.
    sample = ROOT / 'artifacts/sitl-sample'
    staging = Path(tempfile.mkdtemp(prefix='.sitl-sample-', dir=sample.parent))
    try:
        shutil.copy2(source / 'provenance.json', staging / 'provenance.json')
        for result in results:
            destination = staging / result['scenario']
            destination.mkdir()
            base = (
                FLEET_FILES
                if result['scenario'] == FLEET
                else GUARDIAN_FILES
                if result['scenario'] in GUARDIANS
                else None
            )
            files = (*base, *[f'mission_{v["ns"]}.jsonl' for v in result['vehicles']]) if base else FILES
            for file in files:
                if (source / result['scenario'] / file).exists():
                    shutil.copy2(source / result['scenario'] / file, destination / file)
        (staging / 'report.json').write_text(serialized, encoding='utf-8')
        (staging / 'report-data.js').write_text(
            'window.SITL_REPORT=' + json.dumps(page, separators=(',', ':'), allow_nan=False) + ';\n', encoding='utf-8'
        )
        (staging / 'guardian-data.js').write_text(
            'window.SITL_GUARDIANS=' + json.dumps(report['guardians'], separators=(',', ':'), allow_nan=False) + ';\n',
            encoding='utf-8',
        )
        replace_sample(staging, sample)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    lines = [
        '# PX4 / ROS 2 / Gazebo execution evidence',
        '',
        report['boundary'],
        '',
        'Generated UTC: ' + report['generated_utc'],
        '',
        '| Scenario | Result | Wall duration (s) | Max estimated altitude (m) | Min estimated obstacle clearance (m) |',
        '|---|---|---:|---:|---:|',
    ]
    for r in report['results']:
        lines.append(
            f"| {r['scenario']} | PASS | {r['duration_wall_s']:.2f} | {r['max_altitude_m']:.3f} | {r['minimum_estimated_clearance_m']:.3f} |"
        )
    lines += [
        '',
        'All four scenarios require actual armed offboard state, observed climb, land mode, landed state and final disarm. Normal and camera-recovery runs also require a reached goal, COMPLETE and accepted land command. GPS loss requires a post-injection stale-GNSS landing decision. Companion crash requires a subsequent PX4 failsafe.',
        '',
        'Clearance uses valid recorded PX4 position estimates against known cylinders. Invalid estimates after injected GPS loss are excluded; clearance is not established for that degraded interval. This is not Gazebo ground-truth collision verification. Wall duration includes startup and process cleanup.',
        '',
        '## Fleet from a moving carrier',
        '',
        'Three PX4 instances launch in sequence from pads on a carrier vehicle, fly separate inspection legs at 3, 4.5 and 6 m, and land back on the carrier while it drives. A ground-station node grants one launch at a time and one landing at a time, lowest layer first; each vehicle keeps its own C++ supervisor and PX4 failsafes.',
        '',
        '| Vehicle | Goal | Altitude layer (m) | Touchdown pad error (m) | Carrier speed at touchdown (m/s) | Min obstacle clearance (m) |',
        '|---|---|---:|---:|---:|---:|',
    ]
    fleet = report['fleet']
    guardians = report['guardians']
    for v in fleet['vehicles']:
        touchdown = v['touchdown'] or {}
        lines.append(
            f"| {v['ns']} | ({v['goal'][0]}, {v['goal'][1]}) | {v['altitude']:.0f} | {touchdown.get('pad_error', float('nan')):.3f} | "
            f"{(touchdown.get('carrier') or {}).get('speed', float('nan')):.2f} | {v['min_clearance_m']:.2f} |"
        )
    lines += [
        '',
        f"Minimum separation between airborne vehicles: {fleet['min_separation_m']:.2f} m. Carrier travel: "
        f"{fleet['carrier'][-1]['e'] - fleet['carrier'][0]['e']:.1f} m. Recording two cameras slows this simulation below real time; "
        'the adapters judge freshness on simulation time, as PX4 does.',
        '',
        '## Guardians against threats',
        '',
        'Three PX4 instances hold watch posts around the carrier while each scenario adds its own threats, a jammer, a GNSS '
        'spoofer or a dead link to the center; the station and each guardian run the same decision code as the fast simulator. '
        'Separations, drift and landings are measured on Gazebo truth. See [the guardian design](guardian.md).',
        '',
        '| Scenario | What happens | Checks | Warning (simulated s) | Closest guardian to a threat (m) | Recovery decided by |',
        '|---|---|---:|---:|---:|---|',
    ]
    for name, g in guardians.items():
        seps = [v for v in (g.get('guardian_separation_m') or {}).values() if v is not None]
        lines.append(
            f"| `{name}` | {g['title']} | {sum(g['checks'].values())} / {len(g['checks'])} | "
            f"{'—' if g.get('warning_s') is None else format(g['warning_s'], '.1f')} | {format(min(seps), '.2f') if seps else '—'} | "
            f"{(g.get('recovery') or {}).get('by', '—')} |"
        )
    lines += ['', '## Exact upstream revisions', '']
    lines += [f'- {name}: `{revision}`' for name, revision in revisions.items()]
    lines += [
        '',
        '## Evidence and limitations',
        '',
        '- Interactive replays of the [single-drone flights](../web/sitl.html), the [fleet](../web/fleet.html) and the [guardians](../web/guardian.html), and the [machine-readable report](../artifacts/sitl-sample/report.json).',
        '- Per-scenario reference folders preserve final parameters, firmware log, mission/perception logs and acceptance result.',
        '- Runtime source/binary hashes and installed versions were captured before flight; inputs were checked again after the matrix. Per-file SHA-256 hashes identify the exact tested inputs, including changes not yet committed when tested.',
        '- Original run folders retain ROS bags and independent observer JSONL; Linux runtime folders retain PX4 ULogs.',
        '- Camera and LiDAR are procedural ROS publishers. PX4 flight sensors and vehicle dynamics come from Gazebo.',
        '- Battery supervision uses a fixed simulated 95% input in this adapter; low-battery testing remains in the accelerated suite.',
        '- The model is a hand-authored brightness segmentation graph on CPU. No trained drone classifier, NPU profiling, VIO or Qualcomm boot-chain validation is claimed.',
        '- DDS discovery uses domain 42 and the normal network transports. This is a simulation setup, not a secured operational deployment.',
        '',
        'Reproduce with [the WSL SITL guide](sitl-guide.md).',
    ]
    (ROOT / 'docs/sitl-execution.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('Published', sample)


if __name__ == '__main__':
    main()
