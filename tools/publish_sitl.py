#!/usr/bin/env python3
"""Publish compact, measured SITL evidence; leave bags and upstream builds local."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import shutil
from evidence_contracts import require_matrix,read_records
from world import OBSTACLES

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--workspace',type=Path,help='Legacy option; environment is read from run-time provenance')
    args=parser.parse_args();source=args.input.resolve()
    results=json.loads((source/'results.json').read_text())
    expected={'nominal','camera_dropout','companion_crash','gps_loss'}
    require_matrix(results,expected)
    provenance=json.loads((source/'provenance.json').read_text())
    if provenance.get('inputs_unchanged') is not True:
        raise ValueError('Runtime inputs changed during the experiment; rerun the matrix')
    # Validate every source before touching the reference sample.
    for result in results:
        required={'telemetry_received','offboard_entered','takeoff_observed','disarmed_at_end',
            'landed_at_end','estimated_obstacle_clearance','altitude_bounded','land_mode_observed',
            'image_messages','perception_messages','lidar_messages','arming_ack_accepted',
            'no_runner_error','ordered_flight_cycle','final_pose_near_ground','finite_positions','estimator_valid_before_fault','rosbag_recorded'}
        if result['scenario'] in ('nominal','camera_dropout'):
            required.update(('mission_complete','goal_reached','landing_ack_accepted'))
        if result['scenario']=='camera_dropout':required.update(('camera_hold_observed','camera_recovered'))
        if result['scenario']=='companion_crash':required.update(('fault_injected','px4_failsafe_after_crash'))
        if result['scenario']=='gps_loss':required.update(('fault_injected','gps_fault_handled','gps_fix_lost'))
        if not required<=result['checks'].keys():raise ValueError('Missing required checks: '+result['scenario'])
        folder=source/result['scenario']
        if json.loads((folder/'result.json').read_text())!=result:
            raise ValueError('Summary differs from per-scenario evidence: '+result['scenario'])
        for name in ('parameters.log','px4.log','perception.jsonl','mission.jsonl'):
            if not (folder/name).is_file() or not (folder/name).stat().st_size:
                raise ValueError('Missing source evidence: '+str(folder/name))
        read_records(folder/'perception.jsonl');read_records(folder/'mission.jsonl')
    sample=ROOT/'artifacts/sitl-sample';sample.mkdir(exist_ok=True)
    environment=provenance['environment'];revisions=environment['upstream_revisions']
    shutil.copy2(source/'provenance.json',sample/'provenance.json')
    for result in results:
        name=result['scenario'];destination=sample/name;destination.mkdir(exist_ok=True)
        for file in ('result.json','parameters.log','perception.jsonl','mission.jsonl','gps-injection.log','px4.log'):
            if (source/name/file).exists():shutil.copy2(source/name/file,destination/file)
        observations=[json.loads(line) for line in (source/name/'perception.jsonl').read_text().splitlines()]
        inference=[r['inference_ms'] for r in observations if r['kind']=='inference']
        inference.sort()
        result['onnx_sampled_p95_ms']=inference[min(len(inference)-1,int(.95*len(inference)))] if inference else None
        result['onnx_timing_note']='One logged timing per ten frames; CPU synthetic graph; not an NPU benchmark.'
        first=result['trace'][0]['wall_time']
        result['replay_start_wall_time']=first
    report={'generated_utc':datetime.now(timezone.utc).isoformat(),'environment':environment,'provenance':provenance,
            'boundary':'Actual PX4/Gazebo/ROS; procedural camera/lidar; CPU synthetic ONNX; no Qualcomm hardware.',
            'obstacles':[list(o) for o in OBSTACLES],'results':results}
    (sample/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (sample/'report-data.js').write_text('window.SITL_REPORT='+json.dumps(report,separators=(',',':'))+';\n',encoding='utf-8')
    lines=['# PX4 / ROS 2 / Gazebo execution evidence','',report['boundary'],'',
           'Generated UTC: '+report['generated_utc'],'',
           '| Scenario | Result | Wall duration (s) | Max estimated altitude (m) | Min estimated obstacle clearance (m) |',
           '|---|---|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['scenario']} | PASS | {r['duration_wall_s']:.2f} | {r['max_altitude_m']:.3f} | {r['minimum_estimated_clearance_m']:.3f} |")
    lines += ['','All four scenarios require actual armed offboard state, observed climb, land mode, landed state and final disarm. Normal and camera-recovery runs also require a reached goal, COMPLETE and accepted land command. GPS loss requires a post-injection stale-GNSS landing decision. Companion crash requires a subsequent PX4 failsafe.','',
              'Clearance uses valid recorded PX4 position estimates against known cylinders. Invalid estimates after injected GPS loss are excluded; clearance is not established for that degraded interval. This is not Gazebo ground-truth collision verification. Wall duration includes startup and process cleanup.','',
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
