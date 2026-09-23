#!/usr/bin/env python3
"""Launch only task-owned local simulation processes, record evidence, then clean up."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from world import OBSTACLES
from evidence_contracts import read_records,flight_cycle_checks,bag_has_topics
from perception import create_model
from provenance import capture
from security import provision,public_key_hex
SCENARIOS=('nominal','camera_dropout','companion_crash','gps_loss')

class Processes:
    def __init__(self,output,env):self.output=output;self.env=env;self.children=[];self.handles=[]
    def launch(self,name,args,cwd=None):
        stream=(self.output/(name+'.log')).open('w');self.handles.append(stream)
        p=subprocess.Popen(args,cwd=cwd,env=self.env,stdout=stream,stderr=subprocess.STDOUT,
                           stdin=subprocess.DEVNULL,start_new_session=True)
        self.children.append((name,p));return p
    def stop(self,p,sig=signal.SIGINT):
        # The leader can exit before its children. Its owned group still needs cleanup.
        try:os.killpg(p.pid,sig)
        except ProcessLookupError:pass
        try:p.wait(timeout=6)
        except subprocess.TimeoutExpired:pass
        finally:
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            p.wait(timeout=3)
    def close(self):
        errors=[]
        for name,p in reversed(self.children):
            try:self.stop(p)
            except Exception as exc:errors.append(f'{name}: {exc}')
        for h in self.handles:h.close()
        if errors:raise RuntimeError('Cleanup failed: '+'; '.join(errors))

def run_scenario(name,workspace,base_output,timeout):
    output=base_output/name
    if output.exists():raise RuntimeError(f'Output already exists: {output}. Choose a fresh --output directory.')
    output.mkdir(parents=True);run_dir=workspace/'runs'/f'{name}-{time.time_ns()}';run_dir.mkdir(parents=True)
    px4=workspace/'PX4-Autopilot';build=px4/'build/px4_sitl_default'
    env={k:v for k,v in os.environ.items() if not k.startswith('PX4_PARAM_')}
    env.update({'HEADLESS':'1','GZ_IP':'127.0.0.1','GZ_PARTITION':f'mission_{os.getpid()}_{name}',
        'GZ_SIM_RESOURCE_PATH':str(px4/'Tools/simulation/gz/models'),
        'PX4_GZ_STANDALONE':'1','PX4_GZ_MODEL_NAME':'x500_0','PX4_GZ_WORLD':'inspection',
        'PX4_SYS_AUTOSTART':'4001','PX4_SIM_MODEL':'gz_x500','PX4_PARAM_COM_RC_IN_MODE':'4',
        'PX4_PARAM_COM_RCL_EXCEPT':'4','PX4_PARAM_NAV_DLL_ACT':'2','PX4_PARAM_COM_OF_LOSS_T':'0.5',
        'PX4_PARAM_COM_OBL_RC_ACT':'4','PX4_PARAM_COM_FAIL_ACT_T':'0','PX4_PARAM_COM_DISARM_LAND':'1',
        'PX4_PARAM_SYS_FAILURE_EN':'1'})
    processes=Processes(output,env);injected_at=None;last_progress=0;error=None
    launched=time.monotonic();armed_seen=False;flying_since=None
    try:
        agent_library_path=str(workspace/'agent-install/lib')+':'+env.get('LD_LIBRARY_PATH','')
        processes.launch('agent',['env','LD_LIBRARY_PATH='+agent_library_path,
                                  str(workspace/'agent-install/bin/MicroXRCEAgent'),'udp4','-p','8888','-v','3'])
        processes.launch('gazebo',['gz','sim','-r','-s',str(ROOT/'simulation/worlds/inspection.sdf')])
        processes.launch('px4',[str(build/'bin/px4'),'-d',str(build/'etc'),'-w',str(run_dir)])
        processes.launch('gcs',[sys.executable,str(ROOT/'integration/gcs_heartbeat.py')])
        # Airframe startup can overwrite environment parameter overrides. Apply
        # and record the final simulation policy only after rcS has completed.
        startup_deadline=time.monotonic()+40
        while 'Startup script returned successfully' not in (output/'px4.log').read_text():
            if time.monotonic()>startup_deadline:raise RuntimeError('PX4 startup timeout')
            if any(p.poll() is not None for _,p in processes.children):raise RuntimeError('Startup process exited')
            time.sleep(0.2)
        with (output/'parameters.log').open('w') as parameter_log:
            for key,value in env.items():
                if key.startswith('PX4_PARAM_'):
                    parameter=key.removeprefix('PX4_PARAM_')
                    subprocess.run([str(build/'bin/px4-param'),'set',parameter,value],env=env,
                                   stdout=parameter_log,stderr=subprocess.STDOUT,check=True,timeout=5)
                    subprocess.run([str(build/'bin/px4-param'),'show',parameter],env=env,
                                   stdout=parameter_log,stderr=subprocess.STDOUT,check=True,timeout=5)
        processes.launch('observer',[sys.executable,str(ROOT/'integration/observer_node.py'),'--log',str(output/'observer.jsonl')])
        payload=[sys.executable,str(ROOT/'integration/payload_node.py')]
        if name=='camera_dropout':payload+=['--camera-drop-at','18','--camera-drop-for','0.8']
        processes.launch('payload',payload)
        # Act as the release authority: sign the stored model, keep the private key in memory and
        # hand the node only a public key stored outside the artifact folder.
        model=output/'detector.onnx';create_model(model);release_key=Ed25519PrivateKey.generate()
        provision(model,release_key);trust_anchor=run_dir/'trust-anchor.pub'
        trust_anchor.write_text(public_key_hex(release_key)+'\n');del release_key
        processes.launch('perception',[sys.executable,str(ROOT/'integration/perception_node.py'),
                         '--model',str(model),'--public-key',str(trust_anchor),'--log',str(output/'perception.jsonl')])
        mission=processes.launch('mission',[sys.executable,str(ROOT/'integration/mission_node.py'),
                             '--allow-sitl','--log',str(output/'mission.jsonl')])
        # Capture selected typed application topics; raw firmware evidence is independently logged.
        processes.launch('rosbag',['ros2','bag','record','-o',str(output/'rosbag'),
                                  '/mission/decision','/mission/perception','/mission/lidar/scan'])
        while time.monotonic()-launched<timeout:
            time.sleep(0.25)
            failures=[n for n,p in processes.children if p.poll() is not None and not(n=='mission' and name=='companion_crash' and injected_at)]
            if failures:raise RuntimeError('Processes exited: '+', '.join(failures))
            records=read_records(output/'observer.jsonl',live=True)
            states=[r for r in records if r['kind']=='status'];poses=[r for r in records if r['kind']=='position']
            if states:
                state=states[-1];armed_seen|=state['arming_state']==2
                if state['arming_state']==2 and poses and -poses[-1]['ned'][2]>2 and flying_since is None:
                    flying_since=time.monotonic()
                if name in ('companion_crash','gps_loss') and flying_since and time.monotonic()-flying_since>3 and injected_at is None:
                    injected_at=time.time()
                    if name=='companion_crash':processes.stop(mission,signal.SIGKILL)
                    else:
                        # v1.16's Gazebo bridge does not acknowledge `failure gps
                        # off`. Its supported SIM_GPS_USED parameter drives fix
                        # validity in the actual GPS sensor publication instead.
                        command=subprocess.run([str(build/'bin/px4-param'),'set','SIM_GPS_USED','0'],env=env,capture_output=True,text=True,timeout=5)
                        (output/'gps-injection.log').write_text(command.stdout+command.stderr)
                        if command.returncode:raise RuntimeError('GPS injection command failed')
                    print(name,'fault injected',flush=True)
                lands=[r for r in records if r['kind']=='land']
                landed=bool(lands and lands[-1]['landed'] and lands[-1]['wall_time']>=state['wall_time'])
                if armed_seen and state['arming_state']==1 and landed:break
            if time.monotonic()-last_progress>15:
                last_progress=time.monotonic();print(name,'elapsed',round(time.monotonic()-launched),'s',
                    states[-1] if states else 'waiting for DDS telemetry',flush=True)
        else:error='scenario_timeout'
    except Exception as exc:
        error=str(exc)
        (output/'runner-error.log').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        try:processes.close()
        except Exception as exc:error=(error+'; ' if error else '')+str(exc)
    records=read_records(output/'observer.jsonl');mission_records=read_records(output/'mission.jsonl')
    statuses=[r for r in records if r['kind']=='status'];poses=[r for r in records if r['kind']=='position']
    decisions=[r for r in records if r['kind']=='decision'];acks=[r for r in records if r['kind']=='ack']
    counts=[r for r in records if r['kind']=='counts'];max_alt=max([-r['ned'][2] for r in poses],default=0.)
    states=[r['nav_state'] for r in statuses];modes=[r['mode'] for r in decisions]
    transitions=[r for r in mission_records if r['kind']=='transition']
    min_clearance=min((math.hypot(p['ned'][1]-x,p['ned'][0]-y)-radius
                       for p in poses if p['valid'] for x,y,radius in OBSTACLES),default=0.)
    checks={'telemetry_received':len(poses)>30,
            'offboard_entered':any(s['nav_state']==14 and s['arming_state']==2 for s in statuses),
            'takeoff_observed':max_alt>2.,
            'disarmed_at_end':bool(statuses and statuses[-1]['arming_state']==1 and armed_seen),
            'estimated_obstacle_clearance':min_clearance>0.5,
            'altitude_bounded':max_alt<4.,
            'land_mode_observed':any(s['nav_state']==18 and s['arming_state']==2 for s in statuses),
            'image_messages':bool(counts and counts[-1].get('image',0)>20),
            'perception_messages':bool(counts and counts[-1].get('perception',0)>20),
            'lidar_messages':bool(counts and counts[-1].get('scan',0)>20),
            'arming_ack_accepted':any(a['command']==400 and a['result']==0 for a in acks),
            'no_runner_error':error is None}
    checks.update(flight_cycle_checks(records,injected_at if name=='gps_loss' else None))
    checks['rosbag_recorded']=bag_has_topics(output/'rosbag')
    if name in ('nominal','camera_dropout'):
        checks['mission_complete']=any(r.get('mode')=='COMPLETE' for r in transitions)
        checks['goal_reached']=any(r.get('mode')=='COMPLETE' and math.dist(r['position'],[9.,9.,3.])<0.5 for r in decisions)
        checks['landing_ack_accepted']=any(a['command']==21 and a['result']==0 for a in acks)
    if name=='camera_dropout':
        holds=[r for r in decisions if r['mode']=='HOLD' and r['reason']=='vision_stale' and r['camera_age']>0.3]
        checks['camera_hold_observed']=bool(holds)
        checks['camera_recovered']=bool(holds and any(r['mode']=='ACTIVE' and r['wall_time']>holds[0]['wall_time'] for r in decisions))
    if name=='companion_crash':
        checks['fault_injected']=injected_at is not None
        checks['px4_failsafe_after_crash']=any(r['failsafe'] and r['wall_time']>injected_at for r in statuses) if injected_at else False
    if name=='gps_loss':
        checks['fault_injected']=injected_at is not None
        checks['gps_fault_handled']=any(r['mode']=='LAND' and r['reason']=='gnss_stale' and r['wall_time']>=injected_at for r in transitions) if injected_at else False
        checks['gps_fix_lost']=any(r['kind']=='gps' and r['fix_type']<3 and r['wall_time']>=injected_at for r in records) if injected_at else False
    result={'scenario':name,'passed':all(checks.values()),'checks':checks,'error':error,
            'duration_wall_s':round(time.monotonic()-launched,2),'max_altitude_m':max_alt,
            'minimum_estimated_clearance_m':min_clearance,
            'injected_at':injected_at,'statuses':statuses,'acks':acks,'message_counts':counts[-1] if counts else {},
            'trace':poses,'decisions':decisions,'mission_transitions':transitions,
            'gps_states':[r for r in records if r['kind']=='gps'],
            'plan':next((r for r in mission_records if r['kind']=='plan'),None)}
    (output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(name,'PASS' if result['passed'] else 'FAIL',json.dumps(checks),error or '',flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--scenario',choices=SCENARIOS,default='nominal');p.add_argument('--all',action='store_true')
    p.add_argument('--output',type=Path,default=ROOT/'artifacts/sitl-latest');p.add_argument('--timeout',type=float,default=150.)
    args=p.parse_args();workspace=args.workspace.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if not math.isfinite(args.timeout) or args.timeout<=0:p.error('--timeout must be finite and positive')
    if any(out.iterdir()):p.error('--output must be an empty directory to preserve prior evidence')
    provenance={'captured_at_unix':time.time(),'environment':capture(workspace)}
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))
    results=[]
    for name in (SCENARIOS if args.all else (args.scenario,)):
        results.append(run_scenario(name,workspace,out,args.timeout))
        if not results[-1]['passed']:break
    (out/'results.json').write_text(json.dumps(results,indent=2))
    provenance['inputs_unchanged']=provenance['environment']==capture(workspace)
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))
    return 0 if provenance['inputs_unchanged'] and all(r['passed'] for r in results) else 1
if __name__=='__main__':sys.exit(main())
