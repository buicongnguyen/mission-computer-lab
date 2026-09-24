#!/usr/bin/env python3
"""Publish compact, measured SITL evidence; leave bags and upstream builds local."""
import argparse
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import shutil
import tempfile
from evidence_contracts import require_matrix,read_records,strict_loads
from world import OBSTACLES

ROOT=Path(__file__).resolve().parents[1]
FILES=('result.json','parameters.log','perception.jsonl','mission.jsonl','gps-injection.log','px4.log','flight.mp4')
FLEET='fleet_carrier'
FLEET_FILES=('result.json','parameters.log','station.jsonl','flight.mp4','deck.mp4')
FLEET_VEHICLE_CHECKS=('offboard_entered','takeoff_observed','goal_reached','returned_to_carrier','landed_on_pad',
                      'carrier_moving_at_touchdown','disarmed_at_end','no_failsafe','obstacle_clearance')
FLEET_CHECKS=('fleet_min_separation','landings_sequenced','carrier_moved','rosbag_recorded','no_runner_error')

def validate_fleet(result,folder):
    names=[v['ns'] for v in result['vehicles']]
    required={f'{ns}_{c}' for ns in names for c in FLEET_VEHICLE_CHECKS}|set(FLEET_CHECKS)
    if len(names)<2 or not required<=result['checks'].keys():raise ValueError('Missing required fleet checks')
    for name in ('parameters.log','station.jsonl',*[f'mission_{ns}.jsonl' for ns in names]):
        if not (folder/name).is_file() or not (folder/name).stat().st_size:
            raise ValueError('Missing source evidence: '+str(folder/name))
        if name.endswith('.jsonl'):read_records(folder/name)
    if any(not v['trace'] for v in result['vehicles']) or not result['carrier']:raise ValueError('Empty fleet trace')
    if not isinstance(result['min_separation_m'],(int,float)) or not math.isfinite(result['min_separation_m']):
        raise ValueError('Invalid fleet separation')
    result['replay_start_wall_time']=min(v['trace'][0]['wall_time'] for v in result['vehicles'])

def nearest_rank(values,p):
    values=sorted(values)
    return values[max(0,math.ceil(p/100*len(values))-1)] if values else None

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--workspace',type=Path,help='Legacy option; environment is read from run-time provenance')
    args=parser.parse_args();source=args.input.resolve()
    results=strict_loads((source/'results.json').read_text())
    expected={'nominal','camera_dropout','companion_crash','gps_loss',FLEET}
    require_matrix(results,expected)
    provenance=strict_loads((source/'provenance.json').read_text())
    if provenance.get('inputs_unchanged') is not True:
        raise ValueError('Runtime inputs changed during the experiment; rerun the matrix')
    # Validate and derive everything before the reference sample is touched.
    for result in results:
        if result['scenario']==FLEET:
            folder=source/FLEET
            if strict_loads((folder/'result.json').read_text())!=result:raise ValueError('Summary differs from per-scenario evidence: '+FLEET)
            validate_fleet(result,folder);continue
        required={'telemetry_received','offboard_entered','takeoff_observed','disarmed_at_end',
            'landed_at_end','estimated_obstacle_clearance','altitude_bounded','land_mode_observed',
            'image_messages','perception_messages','lidar_messages','arming_ack_accepted',
            'no_runner_error','ordered_flight_cycle','final_pose_near_ground','finite_positions','estimator_valid_before_fault','rosbag_recorded'}
        if result['scenario'] in ('nominal','camera_dropout'):
            required.update(('mission_complete','goal_reached','landing_ack_accepted','no_unexpected_faults'))
        if result['scenario']=='camera_dropout':required.update(('fault_injected','camera_hold_observed','camera_recovered'))
        if result['scenario']=='companion_crash':required.update(('fault_injected','px4_failsafe_after_crash'))
        if result['scenario']=='gps_loss':required.update(('fault_injected','gps_fault_handled','gps_fix_lost'))
        if not required<=result['checks'].keys():raise ValueError('Missing required checks: '+result['scenario'])
        folder=source/result['scenario']
        if strict_loads((folder/'result.json').read_text())!=result:
            raise ValueError('Summary differs from per-scenario evidence: '+result['scenario'])
        for name in ('parameters.log','px4.log','perception.jsonl','mission.jsonl'):
            if not (folder/name).is_file() or not (folder/name).stat().st_size:
                raise ValueError('Missing source evidence: '+str(folder/name))
        if not result['trace']:raise ValueError('Empty flight trace: '+result['scenario'])
        for metric in ('duration_wall_s','max_altitude_m','minimum_estimated_clearance_m'):
            if not isinstance(result[metric],(int,float)) or not math.isfinite(result[metric]):
                raise ValueError(f'Invalid {metric}: '+result['scenario'])
        read_records(folder/'mission.jsonl')
        inference=[r['inference_ms'] for r in read_records(folder/'perception.jsonl') if r['kind']=='inference']
        result['onnx_sampled_p95_ms']=nearest_rank(inference,95)
        result['onnx_timing_note']='One logged timing per ten frames; CPU synthetic graph; not an NPU benchmark.'
        result['replay_start_wall_time']=result['trace'][0]['wall_time']
    environment=provenance['environment'];revisions=environment['upstream_revisions']
    report={'generated_utc':datetime.now(timezone.utc).isoformat(),'environment':environment,'provenance':provenance,
            'boundary':'Actual PX4/Gazebo/ROS; procedural camera/lidar; CPU synthetic ONNX; no Qualcomm hardware.',
            'obstacles':[list(o) for o in OBSTACLES],'results':[r for r in results if r['scenario']!=FLEET],
            'fleet':next(r for r in results if r['scenario']==FLEET)}
    serialized=json.dumps(report,indent=2,allow_nan=False)
    # Build the new sample beside the old one and swap it in, so a failure never leaves a mixture
    # and files from an earlier run (such as an old gps-injection.log) cannot survive.
    sample=ROOT/'artifacts/sitl-sample'
    staging=Path(tempfile.mkdtemp(prefix='.sitl-sample-',dir=sample.parent))
    try:
        shutil.copy2(source/'provenance.json',staging/'provenance.json')
        for result in results:
            destination=staging/result['scenario'];destination.mkdir()
            files=(*FLEET_FILES,*[f'mission_{v["ns"]}.jsonl' for v in result['vehicles']]) if result['scenario']==FLEET else FILES
            for file in files:
                if (source/result['scenario']/file).exists():shutil.copy2(source/result['scenario']/file,destination/file)
        (staging/'report.json').write_text(serialized,encoding='utf-8')
        (staging/'report-data.js').write_text('window.SITL_REPORT='+json.dumps(report,separators=(',',':'),allow_nan=False)+';\n',encoding='utf-8')
        retired=sample.with_name('.sitl-sample-retired')
        if retired.exists():shutil.rmtree(retired)
        if sample.exists():sample.rename(retired)
        staging.rename(sample)
        if retired.exists():shutil.rmtree(retired)
    finally:
        if staging.exists():shutil.rmtree(staging)
    lines=['# PX4 / ROS 2 / Gazebo execution evidence','',report['boundary'],'',
           'Generated UTC: '+report['generated_utc'],'',
           '| Scenario | Result | Wall duration (s) | Max estimated altitude (m) | Min estimated obstacle clearance (m) |',
           '|---|---|---:|---:|---:|']
    for r in report['results']:
        lines.append(f"| {r['scenario']} | PASS | {r['duration_wall_s']:.2f} | {r['max_altitude_m']:.3f} | {r['minimum_estimated_clearance_m']:.3f} |")
    lines += ['','All four scenarios require actual armed offboard state, observed climb, land mode, landed state and final disarm. Normal and camera-recovery runs also require a reached goal, COMPLETE and accepted land command. GPS loss requires a post-injection stale-GNSS landing decision. Companion crash requires a subsequent PX4 failsafe.','',
              'Clearance uses valid recorded PX4 position estimates against known cylinders. Invalid estimates after injected GPS loss are excluded; clearance is not established for that degraded interval. This is not Gazebo ground-truth collision verification. Wall duration includes startup and process cleanup.','',
              '## Fleet from a moving carrier','',
              'Three PX4 instances launch in sequence from pads on a carrier vehicle, fly separate inspection legs at 3, 4 and 5 m, and land back on the carrier while it drives. A ground-station node grants one launch and one landing at a time; each vehicle keeps its own C++ supervisor and PX4 failsafes.','',
              '| Vehicle | Goal | Altitude layer (m) | Touchdown pad error (m) | Carrier speed at touchdown (m/s) | Min obstacle clearance (m) |',
              '|---|---|---:|---:|---:|---:|']
    fleet=report['fleet']
    for v in fleet['vehicles']:
        touchdown=v['touchdown'] or {}
        lines.append(f"| {v['ns']} | ({v['goal'][0]}, {v['goal'][1]}) | {v['altitude']:.0f} | {touchdown.get('pad_error',float('nan')):.3f} | "
                     f"{(touchdown.get('carrier') or {}).get('speed',float('nan')):.2f} | {v['min_clearance_m']:.2f} |")
    lines += ['',f"Minimum separation between airborne vehicles: {fleet['min_separation_m']:.2f} m. Carrier travel: "
              f"{fleet['carrier'][-1]['e']-fleet['carrier'][0]['e']:.1f} m. Recording two cameras slows this simulation below real time; "
              'the adapters judge freshness on simulation time, as PX4 does.','',
              '## Exact upstream revisions','']
    lines += [f'- {name}: `{revision}`' for name,revision in revisions.items()]
    lines += ['','## Evidence and limitations','',
              '- [Interactive replay](../web/sitl.html) and [machine-readable report](../artifacts/sitl-sample/report.json).',
              '- Per-scenario reference folders preserve final parameters, firmware log, mission/perception logs and acceptance result.',
              '- Runtime source/binary hashes and installed versions were captured before flight; inputs were checked again after the matrix. Per-file SHA-256 hashes identify the exact tested inputs, including changes not yet committed when tested.',
              '- Original run folders retain ROS bags and independent observer JSONL; Linux runtime folders retain PX4 ULogs.',
              '- Camera and LiDAR are procedural ROS publishers. PX4 flight sensors and vehicle dynamics come from Gazebo.',
              '- Battery supervision uses a fixed simulated 95% input in this adapter; low-battery testing remains in the accelerated suite.',
              '- The model is a hand-authored brightness segmentation graph on CPU. No trained drone classifier, NPU profiling, VIO or Qualcomm boot-chain validation is claimed.',
              '- DDS discovery uses domain 42 and the normal network transports. This is a simulation setup, not a secured operational deployment.',
              '', 'Reproduce with [the WSL SITL guide](sitl-guide.md).']
    (ROOT/'docs/sitl-execution.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('Published',sample)

if __name__=='__main__':main()
