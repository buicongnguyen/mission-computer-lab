"""Accelerated software-in-the-loop model; no PX4, Gazebo, hardware or NPU is used."""
import argparse
import base64
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import struct
import subprocess
import sys
import time
import zlib
import numpy as np
import onnxruntime as ort
from perception import Detector, camera_frame, create_model
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from security import boot_gate, provision, security_experiments
from world import GOAL, OBSTACLES, Localizer, astar, lidar, occupancy
from supervisor_client import exchange

ROOT=Path(__file__).resolve().parents[1]
SCENARIOS=('nominal','camera_dropout','gps_dropout','link_dropout','inference_overrun',
           'low_battery','imu_dropout','companion_crash')
DT=0.05

def png_data(frame):
    rgb=(frame[0].transpose(1,2,0)*255).astype(np.uint8)
    def chunk(kind,data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    raw=b''.join(b'\0'+row.tobytes() for row in rgb)
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',64,48,8,2,0,0,0))
    png+=chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b'')
    return 'data:image/png;base64,'+base64.b64encode(png).decode()

def percentile(values,p):
    return float(np.percentile(values,p)) if values else None

def run_scenario(name, detector, seed, binary):
    rng=np.random.default_rng(seed)
    truth=np.zeros(3); velocity=np.zeros(3); prior_velocity=np.zeros(3)
    localizer=Localizer()
    scan=lidar(truth)
    blocked=occupancy(scan)
    path=astar((0,0),GOAL,blocked)
    if not path: raise RuntimeError('no initial path')
    waypoints=[[0,0,3]]+[[x,y,3] for x,y in path[1:]]+[[*GOAL,0]]
    waypoint=0; stamps={'imu':0.,'gnss':0.,'vision':0.,'link':0.}
    trace=[]; events=[]; rt=[]; inference=[]; errors=[]; modes=[]
    last_mode=None; detection={'bbox':None,'inference_ms':0}; camera=png_data(camera_frame(0))
    process=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE,text=True,bufsize=1)
    crashed=False; last_command_time=0; min_clearance=math.inf; accepted_targets=[]
    samples=0; reason='initializing';terminated=False
    try:
        for step in range(1800):
            now=round(step*DT,6); fault=now>=5
            imu_bad=name=='imu_dropout' and fault
            gnss_bad=name=='gps_dropout' and fault
            camera_bad=name=='camera_dropout' and 5<=now<5.8
            link_bad=name=='link_dropout' and fault
            acceleration=(velocity-prior_velocity)/DT+rng.normal(0,0.025,3)
            if not imu_bad: stamps['imu']=now
            else: acceleration=np.zeros(3)
            gnss=None
            if step%4==0 and not gnss_bad:
                gnss=truth+rng.normal(0,0.04,3); stamps['gnss']=now
            estimate,uncertainty=localizer.step(acceleration,gnss,DT)
            if step%2==0 and not camera_bad:
                frame=camera_frame(now)
                detection=detector.infer(frame)
                inference.append(detection['inference_ms'])
                camera=png_data(frame); stamps['vision']=now
            if step%2==0 and not link_bad: stamps['link']=now
            scan=lidar(truth)  # ideal 2-D sensor; localization/calibration error is not modeled here
            complete=waypoint>=len(waypoints)
            target=waypoints[min(waypoint,len(waypoints)-1)]
            if not complete and np.linalg.norm(estimate-np.array(target))<0.22:
                accepted_targets.append(waypoint); waypoint+=1
                complete=waypoint>=len(waypoints)
                target=waypoints[min(waypoint,len(waypoints)-1)]
            battery=0.15 if name=='low_battery' and fault else 0.95-now*0.001
            reported_latency=120. if name=='inference_overrun' and 5<=now<5.8 else detection['inference_ms']
            if name=='companion_crash' and fault and not crashed:
                process.kill(); process.wait(timeout=2); crashed=True
                events.append({'time':now,'mode':'PROCESS_EXIT','reason':'injected_companion_crash'})
            if crashed:
                # Independent AUTOPILOT STUB watchdog. This is not the C++ supervisor or real PX4.
                if now-last_command_time>0.5+1e-9:
                    mode,reason='LAND','autopilot_stub_watchdog'
                    command=np.array([0.,0.,-min(0.7,max(0.,truth[2]))])
                else:
                    mode,reason='COAST','autopilot_stub_last_command'
                    command=velocity.copy()
            else:
                fields=[step,now,stamps['imu'],stamps['gnss'],stamps['vision'],stamps['link'],
                        battery,*estimate,*target,reported_latency,int(complete)]
                start=time.perf_counter_ns()
                mode,reason,values=exchange(process,fields,3)
                rt.append((time.perf_counter_ns()-start)/1e6)
                command=np.array(values)
                last_command_time=now
            modes.append(mode); samples+=1
            error=float(np.linalg.norm(estimate-truth)); errors.append(error)
            clearance=min(math.hypot(truth[0]-x,truth[1]-y)-r for x,y,r in OBSTACLES)
            min_clearance=min(min_clearance,clearance)
            if mode!=last_mode:
                events.append({'time':now,'mode':mode,'reason':reason}); last_mode=mode
            if step%5==0 or mode=='COMPLETE':
                trace.append({'time':now,'position':truth.tolist(),'estimate':estimate.tolist(),
                              'target':target,'mode':mode,'reason':reason,'battery':battery,
                              'uncertainty':uncertainty,'error':error,'bbox':detection['bbox'],
                              'inference_ms':detection['inference_ms'],
                              'reported_inference_ms':reported_latency,'camera':camera,
                              'gnss_age':round(now-stamps['gnss'],3),
                              'lidar':[[round(p['x'],2),round(p['y'],2)] for p in scan if p['hit']]})
            if mode=='COMPLETE' or (mode=='LAND' and truth[2]<=0.05):
                terminated=True;break
            prior_velocity=velocity.copy(); velocity=command
            truth+=velocity*DT; truth[2]=max(0.,truth[2])
    finally:
        if process.poll() is None:
            process.stdin.close()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        process.stdout.close(); process.stderr.close()
        if not process.stdin.closed: process.stdin.close()
    expected_complete=name in ('nominal','camera_dropout','inference_overrun')
    checks={'terminated_before_timeout':terminated,
            'expected_terminal_mode':modes[-1]==('COMPLETE' if expected_complete else 'LAND'),
            'no_obstacle_collision':min_clearance>0,
            'bounded_estimate_error':max(errors)<2.0}
    if name in ('camera_dropout','inference_overrun'):
        checks['hold_then_recover']='HOLD' in modes and modes[-1]=='COMPLETE'
    if name in ('gps_dropout','link_dropout','imu_dropout'):
        checks['hold_then_land']='HOLD' in modes and modes[-1]=='LAND'
    if name=='companion_crash': checks['independent_watchdog']=reason=='autopilot_stub_watchdog'
    return {'scenario':name,'seed':seed,'passed':all(checks.values()),'checks':checks,
            'summary':{'simulated_seconds':now,'samples':samples,'terminal_mode':modes[-1],
                       'supervisor_roundtrip_p95_ms':percentile(rt,95),
                       'onnx_cpu_p95_ms':percentile(inference,95),
                       'localization_rmse_m':float(np.sqrt(np.mean(np.square(errors)))),
                       'max_localization_error_m':max(errors),'min_obstacle_clearance_m':min_clearance,
                       'waypoints_reached':len(accepted_targets)},
            'events':events,'path':path,'blocked':[list(p) for p in sorted(blocked)],'trace':trace}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--all',action='store_true')
    parser.add_argument('--scenario',choices=SCENARIOS,default='nominal')
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/latest')
    parser.add_argument('--seed',type=int,default=17)
    parser.add_argument('--binary',type=Path,default=ROOT/'build/mission_supervisor')
    args=parser.parse_args()
    if os.name!='posix': parser.error('Run this harness inside WSL/Linux.')
    output=args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    model=output/'synthetic_detector.onnx'; create_model(model)
    # Release step signs the stored file; boot verifies it from disk and loads only verified bytes.
    release_key=Ed25519PrivateKey.generate(); provision(model,release_key)
    payload,boot=boot_gate(model,release_key.public_key()); del release_key
    detector=Detector(payload)
    selected=SCENARIOS if args.all else (args.scenario,)
    results=[]
    for name in selected:
        result=run_scenario(name,detector,args.seed,args.binary.resolve()); results.append(result)
        print(name, 'PASS' if result['passed'] else 'FAIL',result['summary'],flush=True)
    security=security_experiments(payload)
    report={'schema_version':1,'description':'Accelerated kinematic simulation; synthetic sensors; CPU ONNX; no Qualcomm hardware/PX4/ROS 2',
            'environment':{'platform':platform.platform(),'python':platform.python_version(),
                           'onnxruntime':ort.__version__,'providers':detector.session.get_providers(),
                           'model_sha256':hashlib.sha256(payload).hexdigest(),
                           'binary_sha256':hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                           'generated_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())},
            'boot':boot,'security':security,'obstacles':OBSTACLES,'results':results}
    serialized=json.dumps(report,separators=(',',':'),allow_nan=False)
    (output/'report.json').write_text(serialized,encoding='utf-8')
    (output/'report-data.js').write_text('window.MISSION_REPORT = '+serialized+';\n',encoding='utf-8')
    with (output/'metrics.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=['scenario','passed']+list(results[0]['summary']))
        writer.writeheader()
        for r in results: writer.writerow({'scenario':r['scenario'],'passed':r['passed'],**r['summary']})
    lines=['# Execution evidence','',report['description'],'',
           'Generated UTC: '+report['environment']['generated_utc'],'',
           '| Scenario | Result | End | ONNX p95 (ms) | Supervisor IPC p95 (ms) | Localization RMSE (m) |',
           '|---|---|---|---:|---:|---:|']
    for r in results:
        s=r['summary']; lines.append(f"| {r['scenario']} | {'PASS' if r['passed'] else 'FAIL'} | {s['terminal_mode']} | {s['onnx_cpu_p95_ms']:.3f} | {s['supervisor_roundtrip_p95_ms']:.3f} | {s['localization_rmse_m']:.3f} |")
    lines+=['','Timing is measured wall-clock time in WSL, not a real-time guarantee or Qualcomm benchmark. The injected 120 ms inference fault is a synthetic reported latency; it does not sleep or emulate an NPU.','',
            'Security policy checks: '+str(sum(x['passed'] for x in security))+'/'+str(len(security))+'.',
            '', 'Raw environment, model/binary hashes, per-scenario checks, events and replay frames: report.json.']
    (output/'execution.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if not all(r['passed'] for r in results) or not all(r['passed'] for r in security): return 1
    return 0

if __name__=='__main__': sys.exit(main())
