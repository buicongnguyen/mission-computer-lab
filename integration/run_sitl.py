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

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from world import OBSTACLES
from evidence_contracts import read_records,flight_cycle_checks,bag_has_topics,fleet_landing_confirmed
from perception import create_model
from provenance import capture
from security import provision,public_key_hex
SCENARIOS=('nominal','camera_dropout','companion_crash','gps_loss','fleet_carrier')  # Guardian scenarios are appended below.
MULTI=('fleet_carrier',)
# Three vehicles launch from pads on a carrier; each flies its own leg at its own altitude layer.
FLEET=[{'ns':'px4_0','pad':-1.1,'goal':(9,9),'altitude':3.},
       {'ns':'px4_1','pad':0.,'goal':(10,3),'altitude':4.},
       {'ns':'px4_2','pad':1.1,'goal':(-1,10),'altitude':5.}]
CARRIER_START=(0.,-1.5);DECK=0.6
sys.path.insert(0,str(ROOT/'integration'))
import guardian_layout as GL
from guardian import authorised,safe_move
# Guardians hold watch posts instead of flying inspection legs; the same launch and landing machinery applies.
GUARDIAN_FLEET=[{'ns':g['ns'],'pad':g['pad'],'goal':g['post'],'altitude':g['altitude']} for g in GL.GUARDIANS]
SCENARIOS+=tuple(GL.SCENARIOS);MULTI+=tuple(GL.SCENARIOS)
# Adapter nodes judge freshness on Gazebo's clock, as PX4 does: recording video can slow the simulation
# below real time, and wall-clock freshness would then report healthy links as stale.
SIM_TIME=['--ros-args','-p','use_sim_time:=true']
CLOCK_BRIDGE='/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
SITL_PARAMETERS={'COM_RC_IN_MODE':'4','COM_RCL_EXCEPT':'4','NAV_DLL_ACT':'2','COM_OF_LOSS_T':'0.5',
                 'COM_OBL_RC_ACT':'4','COM_FAIL_ACT_T':'0','COM_DISARM_LAND':'1','SYS_FAILURE_EN':'1'}

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

def record_video(env,path=None,service='/overview/record_video'):
    """Start recording a world camera to `path`, or stop when `path` is None."""
    request=f'start: true, format: "mp4", save_filename: "{path}"' if path else 'stop: true'
    reply=subprocess.run(['gz','service','-s',service,'--reqtype','gz.msgs.VideoRecord',
                          '--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],
                         env=env,capture_output=True,text=True,timeout=15)
    if 'data: true' not in reply.stdout:
        raise RuntimeError('Video recorder did not '+('start' if path else 'stop')+': '+(reply.stdout+reply.stderr).strip())

def run_scenario(name,workspace,base_output,timeout,video=False,gui=False):
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
    launched=time.monotonic();armed_seen=False;flying_since=None;recording=False
    try:
        agent_library_path=str(workspace/'agent-install/lib')+':'+env.get('LD_LIBRARY_PATH','')
        processes.launch('agent',['env','LD_LIBRARY_PATH='+agent_library_path,
                                  str(workspace/'agent-install/bin/MicroXRCEAgent'),'udp4','-p','8888','-v','3'])
        # Headless EGL rendering serves the overview camera; it does not change the flight physics. The
        # recorder encodes to a temporary file in the server's working directory and renames it on stop,
        # so run the server in the output folder (a rename cannot cross from /mnt/c to the Linux disk).
        processes.launch('gazebo',['gz','sim','-r','-s','-v','2','--headless-rendering',
                                   str(ROOT/'simulation/worlds/inspection.sdf')],cwd=output)
        processes.launch('px4',[str(build/'bin/px4'),'-d',str(build/'etc'),'-w',str(run_dir)])
        processes.launch('gcs',[sys.executable,str(ROOT/'integration/gcs_heartbeat.py')])
        if gui:processes.launch('gui',['gz','sim','-g'])  # Live 3D view (WSLg); closing it does not fail the run.
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
        if video:record_video(env,output/'flight.mp4');recording=True
        processes.launch('observer',[sys.executable,str(ROOT/'integration/observer_node.py'),'--log',str(output/'observer.jsonl')])
        processes.launch('clock',['ros2','run','ros_gz_bridge','parameter_bridge',CLOCK_BRIDGE])
        payload=processes.launch('payload',[sys.executable,str(ROOT/'integration/payload_node.py'),'--camera-drop-for','0.8',*SIM_TIME])
        # Act as the release authority: sign the stored model, keep the private key in memory and
        # hand the node only a public key stored outside the artifact folder.
        model=output/'detector.onnx';create_model(model);release_key=Ed25519PrivateKey.generate()
        provision(model,release_key);trust_anchor=run_dir/'trust-anchor.pub'
        trust_anchor.write_text(public_key_hex(release_key)+'\n');del release_key
        processes.launch('perception',[sys.executable,str(ROOT/'integration/perception_node.py'),
                         '--model',str(model),'--public-key',str(trust_anchor),'--log',str(output/'perception.jsonl')])
        mission=processes.launch('mission',[sys.executable,str(ROOT/'integration/mission_node.py'),
                             '--allow-sitl','--log',str(output/'mission.jsonl'),
                             '--model-sha256',hashlib.sha256(model.read_bytes()).hexdigest(),*SIM_TIME])
        # Capture selected typed application topics; raw firmware evidence is independently logged.
        processes.launch('rosbag',['ros2','bag','record','-o',str(output/'rosbag'),
                                  '/mission/decision','/mission/perception','/mission/lidar/scan'])
        while time.monotonic()-launched<timeout:
            time.sleep(0.25)
            failures=[n for n,p in processes.children if p.poll() is not None and n!='gui'
                      and not(n=='mission' and name=='companion_crash' and injected_at)]
            if failures:raise RuntimeError('Processes exited: '+', '.join(failures))
            records=read_records(output/'observer.jsonl',live=True)
            states=[r for r in records if r['kind']=='status'];poses=[r for r in records if r['kind']=='position']
            if states:
                state=states[-1];armed_seen|=state['arming_state']==2
                if state['arming_state']==2 and poses and -poses[-1]['ned'][2]>2 and flying_since is None:
                    flying_since=time.monotonic()
                if name!='nominal' and flying_since and time.monotonic()-flying_since>3 and injected_at is None:
                    injected_at=time.time()
                    if name=='companion_crash':processes.stop(mission,signal.SIGKILL)
                    elif name=='camera_dropout':os.kill(payload.pid,signal.SIGUSR1)  # In flight, not at a fixed uptime.
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
        if recording:
            # Stop while Gazebo still runs, then give the encoder a moment to finalize the file.
            try:
                record_video(env)
                for _ in range(50):
                    if (output/'flight.mp4').is_file() and (output/'flight.mp4').stat().st_size:break
                    time.sleep(0.1)
            except Exception as exc:error=(error+'; ' if error else '')+str(exc)
        try:processes.close()
        except Exception as exc:error=(error+'; ' if error else '')+str(exc)
    records=read_records(output/'observer.jsonl');mission_records=read_records(output/'mission.jsonl')
    statuses=[r for r in records if r['kind']=='status'];poses=[r for r in records if r['kind']=='position']
    decisions=[r for r in records if r['kind']=='decision'];acks=[r for r in records if r['kind']=='ack']
    counts=[r for r in records if r['kind']=='counts'];max_alt=max([-r['ned'][2] for r in poses],default=0.)
    states=[r['nav_state'] for r in statuses]
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
        # Only the injected fault may interrupt these flights: no other HOLD or LAND and no PX4 failsafe.
        allowed={'nominal':set(),'camera_dropout':{'vision_stale'}}[name]
        checks['no_unexpected_faults']=(not any(s['failsafe'] for s in statuses) and
            all(t['mode'] not in ('HOLD','LAND') or (t['mode']=='HOLD' and t['reason'] in allowed) for t in transitions))
    if name=='camera_dropout':
        checks['fault_injected']=injected_at is not None
        # The HOLD must follow the in-flight injection, not a startup gap before takeoff.
        holds=[r for r in decisions if r['mode']=='HOLD' and r['reason']=='vision_stale' and r['camera_age']>0.3
               and injected_at is not None and r['wall_time']>=injected_at]
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
            'plan':next((r for r in mission_records if r['kind']=='plan'),None),
            'replans':[r for r in mission_records if r['kind']=='replan'],
            'video':'flight.mp4' if (output/'flight.mp4').is_file() else None}
    (output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(name,'PASS' if result['passed'] else 'FAIL',json.dumps(checks),error or '',flush=True)
    return result

def resample(trace,times):
    """World positions at the given wall times (nearest earlier sample), or None before the first sample."""
    out,i=[],0
    for t in times:
        while i+1<len(trace) and trace[i+1]['wall_time']<=t:i+=1
        out.append(trace[i] if trace and trace[0]['wall_time']<=t else None)
    return out

def run_fleet(workspace,base_output,timeout,video=False,gui=False,name='fleet_carrier'):
    """Three PX4 instances on a carrier: the fleet inspection legs, or one of the guardian scenarios."""
    guardian=name in GL.SCENARIOS;FLEET_=GUARDIAN_FLEET if guardian else FLEET;world='guardian' if guardian else 'fleet'
    scenario=GL.SCENARIOS.get(name,{})
    output=base_output/name
    if output.exists():raise RuntimeError(f'Output already exists: {output}. Choose a fresh --output directory.')
    output.mkdir(parents=True);stamp=time.time_ns()
    px4=workspace/'PX4-Autopilot';build=px4/'build/px4_sitl_default'
    env={k:v for k,v in os.environ.items() if not k.startswith('PX4_PARAM_')}
    env.update({'HEADLESS':'1','GZ_IP':'127.0.0.1','GZ_PARTITION':f'mission_{os.getpid()}_{name}',
        'GZ_SIM_RESOURCE_PATH':str(px4/'Tools/simulation/gz/models'),'PX4_GZ_STANDALONE':'1','PX4_GZ_WORLD':world,
        'PX4_SYS_AUTOSTART':'4001','PX4_SIM_MODEL':'gz_x500',**{'PX4_PARAM_'+k:v for k,v in SITL_PARAMETERS.items()}})
    processes=Processes(output,env);error=None;launched=time.monotonic();recording=[];last_progress=0
    spawns={v['ns']:[CARRIER_START[0]+v['pad'],CARRIER_START[1],DECK] for v in FLEET_}
    try:
        agent_library_path=str(workspace/'agent-install/lib')+':'+env.get('LD_LIBRARY_PATH','')
        processes.launch('agent',['env','LD_LIBRARY_PATH='+agent_library_path,
                                  str(workspace/'agent-install/bin/MicroXRCEAgent'),'udp4','-p','8888','-v','3'])
        world_file=ROOT/'simulation/worlds/fleet.sdf'
        if guardian:  # Each guardian scenario's world carries its own threat models, markers and jamming zone.
            world_file=output/'world.sdf';world_file.write_text(GL.world_sdf(name),encoding='utf-8')
        processes.launch('gazebo',['gz','sim','-r','-s','-v','2','--headless-rendering',str(world_file)],cwd=output)
        # One PX4 per airframe: instance N attaches to x500_N, uses DDS namespace px4_N and system ID N+1.
        for i,v in enumerate(FLEET_):
            run_dir=workspace/'runs'/f'{name}-{v["ns"]}-{stamp}';run_dir.mkdir(parents=True)
            processes.launch(f'px4_{i}',['env',f'PX4_GZ_MODEL_NAME=x500_{i}',f'PX4_UXRCE_DDS_NS={v["ns"]}',
                                         str(build/'bin/px4'),'-i',str(i),'-d',str(build/'etc'),'-w',str(run_dir)])
            time.sleep(1)
        processes.launch('gcs',[sys.executable,str(ROOT/'integration/gcs_heartbeat.py'),
                                '--ports',','.join(str(18570+i) for i in range(len(FLEET_)))])
        if gui:processes.launch('gui',['gz','sim','-g'])
        startup_deadline=time.monotonic()+60
        while not all('Startup script returned successfully' in (output/f'px4_{i}.log').read_text() for i in range(len(FLEET_))):
            if time.monotonic()>startup_deadline:raise RuntimeError('PX4 startup timeout')
            if any(p.poll() is not None for n,p in processes.children if n!='gui'):raise RuntimeError('Startup process exited')
            time.sleep(0.2)
        with (output/'parameters.log').open('w') as parameter_log:
            for i in range(len(FLEET_)):
                for parameter,value in SITL_PARAMETERS.items():
                    for action in (['set',parameter,value],['show',parameter]):
                        subprocess.run([str(build/'bin/px4-param'),'--instance',str(i),*action],env=env,
                                       stdout=parameter_log,stderr=subprocess.STDOUT,check=True,timeout=5)
        # The carrier's odometry and drive command cross between Gazebo and ROS 2 through the bridge.
        # Guardian scenarios also bridge each threat's odometry and drive, and each airframe's true pose.
        intruder_topics=([t for m in scenario.get('threats',{}) for t in (f'/model/{m}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                                                                          f'/model/{m}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist')]+
                         [f'/model/x500_{i}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' for i in range(len(FLEET_))]) if guardian else []
        processes.launch('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',CLOCK_BRIDGE,
                                   '/model/carrier/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                                   '/model/carrier/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',*intruder_topics])
        if video and (not guardian or scenario.get('video')):  # The guardian scenarios record one headline flight.
            cameras=((('flight.mp4','/overview/record_video'),('close.mp4','/close/record_video')) if guardian else
                     (('flight.mp4','/overview/record_video'),('deck.mp4','/deck/record_video')))
            for file,service in cameras:
                record_video(env,output/file,service);recording.append((file,service))
        model=output/'detector.onnx';create_model(model);release_key=Ed25519PrivateKey.generate()
        provision(model,release_key);trust_anchor=workspace/'runs'/f'{name}-trust-anchor-{stamp}.pub'
        trust_anchor.write_text(public_key_hex(release_key)+'\n');del release_key
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest()
        if guardian:
            # The environment (intruder, sensors, jammed link) and the center are stand-ins, not software under test.
            processes.launch('world',[sys.executable,str(ROOT/'integration/guardian_world_node.py'),'--log',str(output/'world.jsonl'),
                                      '--scenario',name,*SIM_TIME])
            processes.launch('center',[sys.executable,str(ROOT/'integration/center_node.py'),'--log',str(output/'center.jsonl'),
                                       *(['--unreachable'] if scenario.get('center_down') else []),*SIM_TIME])
        for i,v in enumerate(FLEET_):
            ns=v['ns'];spawn=[str(c) for c in spawns[ns]]
            processes.launch(f'observer_{ns}',[sys.executable,str(ROOT/'integration/observer_node.py'),'--ns',ns,
                                               '--log',str(output/f'observer_{ns}.jsonl')])
            processes.launch(f'payload_{ns}',[sys.executable,str(ROOT/'integration/payload_node.py'),'--ns',ns,'--spawn',*spawn,*SIM_TIME])
            processes.launch(f'perception_{ns}',[sys.executable,str(ROOT/'integration/perception_node.py'),'--ns',ns,
                             '--model',str(model),'--public-key',str(trust_anchor),'--log',str(output/f'perception_{ns}.jsonl')])
            processes.launch(f'mission_{ns}',[sys.executable,str(ROOT/f'integration/{"guardian" if guardian else "fleet"}_mission_node.py'),'--allow-sitl',
                             '--ns',ns,'--spawn',*spawn,'--goal',*[str(g) for g in v['goal']],'--altitude',str(v['altitude']),
                             '--pad',str(v['pad']),'--system-id',str(i+1),'--model-sha256',model_sha256,
                             '--log',str(output/f'mission_{ns}.jsonl'),*SIM_TIME])
        processes.launch('station',[sys.executable,str(ROOT/f'integration/{"guardian" if guardian else "fleet"}_station_node.py'),
                                    '--drones',','.join(v['ns'] for v in FLEET_),'--log',str(output/'station.jsonl'),
                                    *(['--scenario',name] if guardian else []),*SIM_TIME])
        station_topics=['/station/uplink','/station/fixes','/center/decision'] if guardian else ['/fleet/clearance']
        processes.launch('rosbag',['ros2','bag','record','-o',str(output/'rosbag'),
                                   *[f'/{v["ns"]}/mission/decision' for v in FLEET_],*station_topics,'/model/carrier/odometry'])
        while time.monotonic()-launched<timeout:
            time.sleep(0.5)
            failures=[n for n,p in processes.children if p.poll() is not None and n!='gui']
            if failures:raise RuntimeError('Processes exited: '+', '.join(failures))
            station=read_records(output/'station.jsonl',live=True)
            touchdowns={r['drone'] for r in station if r['kind']=='touchdown'}
            if touchdowns=={v['ns'] for v in FLEET_}:
                time.sleep(3);break  # Let the carrier stop and the logs settle.
            if time.monotonic()-last_progress>15:
                last_progress=time.monotonic();carrier=next((r for r in reversed(station) if r['kind']=='carrier'),None)
                print(name,'elapsed',round(time.monotonic()-launched),'s touchdowns',sorted(touchdowns),
                      'carrier',carrier and round(carrier['e'],1),flush=True)
        else:error='scenario_timeout'
    except Exception as exc:
        error=str(exc)
        (output/'runner-error.log').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        for file,service in recording:
            try:
                record_video(env,None,service)
                for _ in range(50):
                    if (output/file).is_file() and (output/file).stat().st_size:break
                    time.sleep(0.1)
            except Exception as exc:error=(error+'; ' if error else '')+str(exc)
        try:processes.close()
        except Exception as exc:error=(error+'; ' if error else '')+str(exc)
    station=read_records(output/'station.jsonl')
    carrier=[r for r in station if r['kind']=='carrier'];grants=[r for r in station if r['kind']=='clearance']
    touchdowns={r['drone']:r for r in station if r['kind']=='touchdown'}
    checks={};vehicles=[];truth=guardian_truth(output)[1] if guardian else {}
    for v in FLEET_:
        ns=v['ns'];spawn=spawns[ns]
        records=read_records(output/f'observer_{ns}.jsonl');mission_records=read_records(output/f'mission_{ns}.jsonl')
        statuses=[r for r in records if r['kind']=='status']
        # Observer positions are PX4 local NED from the spawn point; convert to world ENU.
        trace=[{'wall_time':p['wall_time'],'e':p['ned'][1]+spawn[0],'n':p['ned'][0]+spawn[1],'u':-p['ned'][2]+spawn[2],
                'valid':p['valid']} for p in records if p['kind']=='position']
        phases=[r for r in mission_records if r['kind']=='phase'];names=[p['phase'] for p in phases]
        inspect=next((p for p in phases if p['phase']=='inspect'),None);touchdown=touchdowns.get(ns)
        # Guardian scenarios judge clearance and landing on truth: a spoofed receiver misleads the estimate.
        clearance=min((math.hypot(t['e']-x,t['n']-y)-r for t in (truth[ns] if guardian else trace)
                       if (guardian or t['valid']) and t['u']>DECK+0.3 for x,y,r in OBSTACLES),default=0.)
        checks[f'{ns}_offboard_entered']=any(s['nav_state']==14 and s['arming_state']==2 for s in statuses)
        checks[f'{ns}_takeoff_observed']=max((t['u'] for t in trace),default=0.)>2.
        if guardian:
            watch=next((p for p in phases if p['phase']=='watch'),None)
            checks[f'{ns}_on_watch']=bool(watch and math.dist(watch['position'][:2],v['goal'])<0.6)
        else:checks[f'{ns}_goal_reached']=bool(inspect and math.dist(inspect['position'][:2],v['goal'])<0.6)
        checks[f'{ns}_returned_to_carrier']='rendezvous' in names and 'descend' in names
        checks[f'{ns}_landed_on_pad']=bool(touchdown and touchdown['pad_error'] is not None and touchdown['pad_error']<0.4
                                           and abs(touchdown['position'][2]-DECK)<0.35)
        if guardian and touchdown and truth.get(ns) and carrier:  # Where the airframe came to rest, against its pad.
            rest=min(truth[ns],key=lambda t:abs(t['wall_time']-touchdown['wall_time']))
            c=min(carrier,key=lambda x:abs(x['wall_time']-touchdown['wall_time']))
            pad=(c['e']+v['pad']*math.cos(c['yaw']),c['n']+v['pad']*math.sin(c['yaw']))
            touchdown={**touchdown,'true_pad_error':math.hypot(rest['e']-pad[0],rest['n']-pad[1])}
            checks[f'{ns}_landed_on_pad']=checks[f'{ns}_landed_on_pad'] and touchdown['true_pad_error']<0.4
        if not guardian:checks[f'{ns}_carrier_moving_at_touchdown']=bool(touchdown and touchdown['carrier'] and touchdown['carrier']['speed']>0.15)
        checks[f'{ns}_disarmed_at_end']=bool(statuses and statuses[-1]['arming_state']==1)
        checks[f'{ns}_landed_confirmed']=fleet_landing_confirmed(records,touchdown)
        checks[f'{ns}_no_failsafe']=not any(s['failsafe'] for s in statuses)
        checks[f'{ns}_obstacle_clearance']=clearance>0.5
        vehicles.append({**v,'goal':list(v['goal']),'spawn':spawn,'trace':trace,'statuses':statuses,'phases':phases,
                         'plan':next((r for r in mission_records if r['kind']=='plan'),None),
                         'replans':[r for r in mission_records if r['kind']=='replan'],
                         'transitions':[r for r in mission_records if r['kind']=='transition'],
                         'touchdown':touchdown,'min_clearance_m':clearance,
                         'guardian':[r for r in mission_records if r['kind']=='guardian'],
                         **({'truth':truth[ns][::2]} if guardian else {})})
    # Separation from the independent observers (or, for guardians, truth), time-aligned at 5 Hz while two are airborne.
    series=[truth[v['ns']] for v in vehicles] if guardian else [v['trace'] for v in vehicles]
    times=[t['wall_time'] for t in series[0]][::(1 if guardian else 2)] if series and series[0] else []
    tracks=[resample(t,times) for t in series];separations=[]
    for k in range(len(times)):
        airborne=[tr[k] for tr in tracks if tr[k] and tr[k]['u']>DECK+0.5]
        separations+=[math.dist((a['e'],a['n'],a['u']),(b['e'],b['n'],b['u'])) for j,a in enumerate(airborne) for b in airborne[j+1:]]
    min_separation=min(separations) if separations else None
    lands=[g for g in grants if g['grant']=='land']
    checks['fleet_min_separation']=min_separation is not None and min_separation>1.0
    checks['landings_sequenced']=len(lands)==len(FLEET_) and all(
        lands[k-1]['drone'] in touchdowns and lands[k]['wall_time']>=touchdowns[lands[k-1]['drone']]['wall_time'] for k in range(1,len(lands)))
    if not guardian:checks['carrier_moved']=bool(carrier) and carrier[-1]['e']-carrier[0]['e']>3.
    checks['rosbag_recorded']=bag_has_topics(output/'rosbag',[f'/{v["ns"]}/mission/decision' for v in FLEET_]+
                                             (['/station/uplink'] if guardian else ['/fleet/clearance']))
    checks['no_runner_error']=error is None
    # A node drops a malformed message instead of dying; a drop in a flight is still a fault, and fails it.
    dropped=[r for log in sorted(output.glob('*.jsonl')) for r in read_records(log) if r.get('kind')=='dropped_message']
    checks['messages_well_formed']=not dropped
    extra=guardian_checks(name,output,station,vehicles,carrier,checks) if guardian else {}
    result={'scenario':name,'passed':all(checks.values()),'checks':checks,'error':error,
            'duration_wall_s':round(time.monotonic()-launched,2),'vehicles':vehicles,'carrier':carrier,'clearances':grants,
            'min_separation_m':min_separation,'deck_height_m':DECK,**extra,
            'videos':{k:f for k,f in (('overview','flight.mp4'),('deck','deck.mp4'),('close','close.mp4')) if (output/f).is_file()}}
    (output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(name,'PASS' if result['passed'] else 'FAIL',json.dumps(checks),error or '',flush=True)
    return result

def guardian_truth(output):
    """Truth from the environment's log: every guardian and threat at the same instant, 5 times a second."""
    records=[r for r in read_records(output/'world.jsonl') if r['kind']=='truth']
    guardians={g['ns']:[{'wall_time':r['wall_time'],'t':r['t'],'e':p[0],'n':p[1],'u':p[2]} for r in records
                        for p in [r['guardians'].get(g['ns'])] if p] for g in GL.GUARDIANS}
    return records,guardians

def guardian_checks(name,output,station,vehicles,carrier,checks):
    """Acceptance for one guardian scenario, from the environment's truth, the station, the center and the vehicles."""
    scenario=GL.SCENARIOS[name];expect=scenario['expect'];cfg=GL.CFG
    world=read_records(output/'world.jsonl');center=read_records(output/'center.jsonl');records,_=guardian_truth(output)
    start=next((r for r in world if r['kind']=='scenario_start'),None)
    posture=[r for r in station if r['kind']=='posture'];orders=[r for r in station if r['kind']=='order']
    red=next((r for r in posture if r['state']=='RED'),None);relocate=next((r for r in station if r['kind']=='relocate'),None)
    recovery=next((r for r in station if r['kind']=='recovery'),None);decisions=[r for r in station if r['kind']=='center_decision']
    hostile=[m for m,spec in scenario['threats'].items() if spec['kind']!='bird'];dispersal=None
    checks['scenario_started']=start is not None
    # Separation and arrival, from truth sampled at the same instant for every body. Arrival is judged at each
    # threat's aim point, where the carrier was parked: once the carrier drives off, its own closest approach
    # would move the goalposts.
    separations={g['ns']:math.inf for g in GL.GUARDIANS};arrival=None;closest=(math.inf,None);station_miss=math.inf
    for r in records:
        if not r['threats']:continue
        c=next((x for x in carrier if x['wall_time']>=r['wall_time']),carrier[-1] if carrier else None)
        for m in hostile:
            q=r['threats'].get(m)
            if not q or q[2]<0.:continue  # Below ground: a diving object after its impact.
            for ns,p in r['guardians'].items():
                if p and p[2]>DECK+0.5:separations[ns]=min(separations[ns],math.dist(p,q))
            if c:station_miss=min(station_miss,math.hypot(q[0]-c['e'],q[1]-c['n']))
            aim=scenario['threats'][m]['aim'];d=math.hypot(q[0]-aim[0],q[1]-aim[1])
            if d<closest[0]:closest=(d,r['t'])
            if arrival is None and d<cfg.protect_radius:arrival=r['t']
    arrival=arrival if arrival is not None else closest[1]
    warning=None if red is None or arrival is None else arrival-red['t']
    def matches(record):
        """Hostile threats within 2 m of a confirmed track when it was confirmed, on truth."""
        near=min(records,key=lambda x:abs(x['t']-record['t'])) if records else None
        return [(math.dist(near['threats'][m],record['p']),m) for m in hostile
                if near and near['threats'].get(m) and math.dist(near['threats'][m],record['p'])<2.]
    confirmed=[r for r in station if r['kind']=='confirmed']
    if hostile:checks['guardians_kept_clear']=all(v>=cfg.safe_radius for v in separations.values())
    if expect['red']:
        checks['threat_confirmed']=any(matches(r) for r in confirmed)  # A real threat, not clutter or a bird.
        checks['red_before_arrival']=warning is not None and warning>0
    else:
        checks['no_red_alert']=red is None
        # Birds, clutter or a drag-off may cause a brief keep-clear from a noisy track, which is no alarm; what
        # must hold is the watch: on truth, no guardian strayed more than one keep-clear step from its post.
        end=next((r['t'] for r in station if r['kind'] in ('recovery','watch_complete')),None)
        watch=[r for r in records if start and r['t']>=start['t'] and (end is None or r['t']<=end)]
        strayed=max((math.dist(r['guardians'][g['ns']][:2],g['post']) for r in watch for g in GL.GUARDIANS
                     if r['guardians'].get(g['ns'])),default=math.inf)
        checks['guardians_held_their_posts']=strayed<=cfg.keep_clear_step+0.5
    if expect.get('relocate'):checks['carrier_relocated_clear']=bool(relocate) and station_miss>=cfg.protect_radius
    if expect.get('station_order'):
        jammed=expect.get('jammed')
        checks['station_ordered_keep_clear']=any(d['action']=='keep_clear' and d['layer']=='station'
                                                 for v in vehicles if v['ns']!=jammed for d in v['guardian'])
    if expect.get('jammed'):
        ns=expect['jammed']
        alone=[d for v in vehicles if v['ns']==ns for d in v['guardian'] if d['layer']=='onboard' and (d['link_age'] or 0)>cfg.lost_link]
        checks['jammed_guardian_acted_alone']=(any(d['action']=='keep_clear' for d in alone) and
                                              any(r['kind']=='dropped' and r['ns']==ns for r in world))
    if expect.get('tracks'):
        # Distinct hostile threats the station confirmed a track on, one track per threat, nearest pairs first.
        pairs=sorted((d,r['track'],m) for r in confirmed for d,m in matches(r));used=set();matched=set()
        for _,track,m in pairs:
            if track not in used and m not in matched:used.add(track);matched.add(m)
        checks['distinct_tracks_confirmed']=len(matched)>=expect['tracks']
    if expect.get('disperse'):
        # Ordered away before impact, and on truth further from the impact point at impact than its post is.
        ns=expect['disperse'];impact=next((r for r in world if r['kind']=='impact'),None)
        ordered=[o for o in orders if o['ns']==ns and o['action'] in ('disperse','keep_clear')]
        post=next(g['post'] for g in GL.GUARDIANS if g['ns']==ns)
        at=min(records,key=lambda x:abs(x['t']-impact['t'])) if impact and records else None
        where=at and at['guardians'].get(ns);aim=scenario['threats'][impact['model']]['aim'] if impact else None
        checks['dispersed_before_impact']=bool(impact and ordered and ordered[0]['t']<impact['t'] and where and
                                               math.dist(where[:2],aim[:2])>math.dist(post,aim[:2])+0.5)
        dispersal={'ns':ns,'post_m':math.dist(post,aim[:2]) if aim else None,
                   'impact_m':math.dist(where[:2],aim[:2]) if where and aim else None}
    if expect.get('spoof'):
        spoof=next((r for r in world if r['kind']=='spoof_start'),None);alarms=[r for r in station if r['kind']=='integrity']
        checks['spoofing_detected']=bool(spoof and alarms and alarms[0]['t']-spoof['t']<20.)
        switched={v['ns'] for v in vehicles for d in v['guardian'] if d['action']=='navigate_by_station'}
        checks['navigated_by_station_fixes']=switched=={g['ns'] for g in GL.GUARDIANS}
        drift={}
        for g in GL.GUARDIANS:  # True distance from the post while on watch, which the drag-off pulls on.
            post=g['post'];watching=[r for r in records if spoof and r['t']>=spoof['t'] and (recovery is None or r['t']<recovery['t'])]
            drift[g['ns']]=max((math.hypot(r['guardians'][g['ns']][0]-post[0],r['guardians'][g['ns']][1]-post[1])
                                for r in watching if r['guardians'].get(g['ns'])),default=None)
        checks['drift_bounded']=all(d is not None and d<3. for d in drift.values())
    else:drift=None
    by=expect['recovery']
    if by=='center':
        checks['recovery_by_center']=bool(recovery and recovery['by']=='center' and decisions and recovery['wall_time']>=decisions[0]['wall_time'])
    else:
        request=next((r for r in station if r['kind']=='center_report' and r['report'] in ('clear_after_red','watch_complete')),None)
        checks['recovery_under_delegation']=bool(recovery and recovery['by']==by and not decisions and request and
                                                 recovery['t']-request['t']>=cfg.center_timeout-0.5)
    # Every keep-clear or dispersal decision, by station or vehicle, was a safe move against what it logged: it
    # opened the range along its whole path to every relevant slow track, and shortened no steady fast object's
    # predicted miss. Both tests are exact for straight lines.
    closing=[]
    for r in orders+[d for v in vehicles for d in v['guardian'] if d['layer']=='onboard']:
        if r['action'] in ('keep_clear','disperse') and r.get('target') and r.get('avoid') is not None:
            if not safe_move(r['own'],r['target'],r['avoid'],r.get('fast') or [],cfg):closing.append(r)
    checks['never_closed_on_threat']=not closing
    # An order relaying a center decision (recovery) carries the center's authority, not the station's.
    decided=[(d['action'],d['layer']) for v in vehicles for d in v['guardian']]+[(r['action'],r.get('authority') or 'station') for r in orders]
    checks['authority_respected']=all(authorised(a,l) for a,l in decided)
    if scenario['threats']:
        # Every scripted threat really flew: truth within 2 s of its start, and at least a metre of travel.
        flew=True
        for m,spec in scenario['threats'].items():
            begin=(start['t'] if start else math.inf)+spec.get('delay',0.)
            path=[r['threats'][m] for r in records if r['threats'].get(m) and r['t']>=begin]
            first=next((r['t'] for r in records if r['threats'].get(m) and r['t']>=begin),math.inf)
            flew&=bool(path) and first-begin<=2. and sum(math.dist(a,b) for a,b in zip(path,path[1:]))>=1.
        checks['threats_flew']=flew
    for missing in set(GL.expected_checks(name))-set(checks):checks[missing]=False  # Never pass by omission.
    threats={m:{'kind':spec['kind'],'trace':[{'wall_time':r['wall_time'],'t':r['t'],'e':r['threats'][m][0],'n':r['threats'][m][1],
                                             'u':r['threats'][m][2]} for r in records if r['threats'].get(m)]}
             for m,spec in scenario['threats'].items()}
    return {'title':scenario['title'],'expect':expect,'threats':threats,'scenario_start':start,'warning_s':warning,
            'station_miss_m':None if math.isinf(station_miss) else station_miss,
            'guardian_separation_m':{k:(None if math.isinf(v) else v) for k,v in separations.items()},
            'posture':posture,'orders':orders,'relocation':[r for r in station if r['kind'] in ('relocate','relocated')],
            'center':[r for r in center if r['kind'] in ('decision','unreachable')],'recovery':recovery,
            'integrity':[r for r in station if r['kind'] in ('integrity','integrity_clear')],'drift_m':drift,
            'dispersal':dispersal,
            'jamming':[r for r in world if r['kind'] in ('scenario_start','jammer_off','link','spoof_start','spoof_error','impact','threat_stop')],
            'layout':{'guardians':GL.GUARDIANS,'jammer':scenario.get('jammer'),'spoof':scenario.get('spoof'),'relocate':GL.RELOCATE,
                      'safe_radius':cfg.safe_radius,'clear_radius':cfg.clear_radius,'protect_radius':cfg.protect_radius,
                      'center_down':bool(scenario.get('center_down')),'watch':scenario.get('watch')}}

def main():
    # Children run in their own sessions, so a closed terminal or a CI cancel must still reach the
    # `finally` cleanup; otherwise PX4, Gazebo and the agent survive and block the next run.
    for sig in (signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda number,frame:sys.exit(128+number))
    p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--scenario',choices=SCENARIOS,default='nominal');p.add_argument('--all',action='store_true')
    p.add_argument('--keep-going',action='store_true',help='Run remaining scenarios after a failed case; failures still exit nonzero')
    p.add_argument('--output',type=Path,default=ROOT/'artifacts/sitl-latest');p.add_argument('--timeout',type=float,default=150.)
    p.add_argument('--video',action='store_true',help='Record single-drone, fleet and configured guardian demonstration cameras')
    p.add_argument('--gui',action='store_true',help='Open the live Gazebo 3D window (WSLg) while flying')
    args=p.parse_args();workspace=args.workspace.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if not math.isfinite(args.timeout) or args.timeout<=0:p.error('--timeout must be finite and positive')
    if any(out.iterdir()):p.error('--output must be an empty directory to preserve prior evidence')
    provenance={'captured_at_unix':time.time(),'environment':capture(workspace)}
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))
    results=[]
    # The guard is wall-clock; recording runs the simulation at roughly 0.4-0.6x real time.
    timeout=args.timeout*(2.5 if args.video else 1.)
    for name in (SCENARIOS if args.all else (args.scenario,)):
        if name in MULTI:results.append(run_fleet(workspace,out,max(timeout,900.),video=args.video,gui=args.gui,name=name))
        else:results.append(run_scenario(name,workspace,out,timeout,video=args.video,gui=args.gui))
        if not results[-1]['passed'] and not args.keep_going:break
    (out/'results.json').write_text(json.dumps(results,indent=2,allow_nan=False))
    provenance['inputs_unchanged']=provenance['environment']==capture(workspace)
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))
    return 0 if provenance['inputs_unchanged'] and all(r['passed'] for r in results) else 1
if __name__=='__main__':sys.exit(main())
