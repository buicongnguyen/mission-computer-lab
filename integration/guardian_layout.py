"""Layout and parameters of the PX4 guardian scenarios.

The arena is 20 m across, so speeds, ranges and radii are the fast simulator's scaled down; the decision
code (tools/guardian.py) is the same. Three guardians hold watch posts around the carrier; each scenario
adds its own threats, a jammer, a GNSS spoofer or a dead link to the center, and states what the evidence
must show. Threats fly to fixed points; nothing here models a seeker, a countermeasure or an engagement.
"""
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from guardian import Config

# slow_speed sits between the birds (0.3 m/s) and the intruders (1.2 m/s), with room for the filter's speed
# noise at this sensor accuracy (about 0.4 m/s on a circling bird).
CFG=Config(gate=0.8,gate_speed=8.,accel_noise=0.3,sigma=0.15,confirm_hits=4,confirm_window=2.,drop_tentative=1.5,
           drop_confirmed=4.,protect_radius=3.,warn_horizon=30.,slow_persist=1.5,slow_speed=0.8,inner_radius=6.,
           clear_radius=3.,safe_radius=1.5,order_horizon=12.,reflex_horizon=4.,keep_clear_step=3.,climb_step=1.,
           max_altitude=6.,min_altitude=2.5,guardian_speed=1.2,fast_speed=4.,clear_time=5.,lost_link=1.5,lost_link_return=30.,
           center_timeout=20.,integrity_threshold=1.,integrity_persistence=3,integrity_clear=6,integrity_max_age=1.,
           fix_timeout=3.)
ORDER_AGE=1.  # s: the station orders only guardians whose state it heard this recently (states arrive at 5 Hz).
# Each guardian launches from its pad, holds a watch post at its own altitude layer, and lands back on its pad.
GUARDIANS=[{'ns':'px4_0','pad':-1.1,'post':(1,2),'altitude':4.},   # Near the carrier, in the northern corridor.
           {'ns':'px4_1','pad':0.,'post':(0,12),'altitude':5.},    # Northern picket: first contact from the north.
           {'ns':'px4_2','pad':1.1,'post':(12,8),'altitude':3.}]   # Eastern picket.
CARRIER=(0.,-1.5)
# Threat scripts. 'line' flies start -> aim and on past it; 'dive' stops at its aim (impact); 'circle' loiters.
NORTH={'kind':'drone','motion':'line','start':(-3.4,23.,4.),'aim':(0.,-1.5),'speed':1.2,'delay':0.}
NORTH_2={'kind':'drone','motion':'line','start':(-5.4,23.5,3.),'aim':(0.,-1.5),'speed':1.2,'delay':6.}
EAST_HIGH={'kind':'drone','motion':'line','start':(22.,10.,7.),'aim':(0.,-1.5),'speed':1.2,'delay':3.}
FAST={'kind':'fast','motion':'dive','start':(-12.,24.,10.),'aim':(0.,-1.5,0.6),'speed':5.5,'delay':0.}
# Birds circle more than 7.5 m from both carrier stops, slower than slow_speed, each within camera range of a
# guardian for part of its circle (a bird no camera ever labels would hold the posture at AMBER, by design).
BIRDS={'bird_0':{'kind':'bird','motion':'circle','center':(8.5,11.5),'radius':2.,'alt':4.,'speed':0.3,'delay':0.},
       'bird_1':{'kind':'bird','motion':'circle','center':(-3.,8.),'radius':1.5,'alt':3.5,'speed':0.3,'delay':0.},
       'bird_2':{'kind':'bird','motion':'circle','center':(4.,8.),'radius':1.5,'alt':4.5,'speed':0.3,'delay':0.}}
JAMMER={'p':(1.5,3.5),'r':2.5,'off_y':-1.,'watch':'intruder'}  # Around px4_0's post, until that intruder passes off_y.
# An area drag-off. heading_deg is the direction a guardian holding its post on GNSS is physically dragged;
# south-west keeps every guardian more than 2 m from the cylinders for the first 3 m of drag.
SPOOF={'rate':0.25,'ramp':8.,'max':6.,'heading_deg':225.}
SCENARIOS={
    'guardian_intruder':{'title':'One intruder, links intact','threats':{'intruder':NORTH},
        'expect':{'red':True,'relocate':True,'station_order':True,'recovery':'center'}},
    'guardian_fast':{'title':'A fast object diving on the carrier','threats':{'fast':FAST},
        'expect':{'red':True,'disperse':'px4_0','recovery':'center'}},
    'guardian_swarm':{'title':'Three intruders from two sectors','threats':{'intruder':NORTH,'intruder_2':NORTH_2,'intruder_3':EAST_HIGH},
        'expect':{'red':True,'relocate':True,'tracks':3,'recovery':'center'}},
    'guardian_birds':{'title':'Circling birds and sensor clutter','threats':dict(BIRDS),'clutter':0.3,'watch':40.,
        'expect':{'red':False,'recovery':'center'}},
    'guardian_jamming':{'title':'An intruder while one guardian is jammed','threats':{'intruder':NORTH},'jammer':JAMMER,
        'expect':{'red':True,'relocate':True,'station_order':True,'jammed':'px4_0','recovery':'center'},'video':True},
    'guardian_spoofing':{'title':'A GNSS drag-off on every receiver','threats':{},'spoof':SPOOF,'watch':45.,
        'expect':{'red':False,'spoof':True,'recovery':'center'}},
    'guardian_center_loss':{'title':'An intruder with no link to the center','threats':{'intruder':NORTH},'center_down':True,
        'expect':{'red':True,'relocate':True,'recovery':'station (delegated)'}},
    'guardian_combined':{'title':'Two intruders, a jammed guardian, birds and no center','threats':{'intruder':NORTH,'intruder_2':NORTH_2,
        'bird_0':BIRDS['bird_0'],'bird_2':BIRDS['bird_2']},'jammer':JAMMER,'center_down':True,'clutter':0.3,
        'expect':{'red':True,'relocate':True,'jammed':'px4_0','tracks':2,'recovery':'station (delegated)'}},
}
WATCH_BEFORE_START=5.  # s every guardian must hold its post before the scenario's threats start.
# On RED the station drives the carrier east along the road to one of these stops: the nearest that keeps
# every tracked threat's predicted path twice the protected radius away, else the one that keeps it farthest.
RELOCATE={'stops':(5.,8.,11.),'speed':0.8}
# Illustrative sensors. The station's own sensor sees high flyers to 20 m but loses anything below 6 m beyond
# 6 m (terrain and clutter); guardians look in every direction and classify inside 5 m. The station also
# locates its own guardians by their datalink (ranging), which gives an independent position fix unless jammed.
SENSORS={'guardian':{'range':8.,'pd':0.9,'sigma':0.15,'period':0.2,'classify_range':5.,'accuracy':0.9},
         'station':{'range':20.,'low_altitude':6.,'low_range':6.,'pd':0.9,'sigma':0.1,'period':0.2},
         'locator':{'range':25.,'sigma':0.2,'period':0.5}}
CENTER={'latency':0.5,'decision_time':3.}
GRID=(-2,12)  # Planning grid bounds (cells), as in tools/world.py.

VEHICLE_CHECKS=('offboard_entered','takeoff_observed','on_watch','returned_to_carrier','landed_on_pad','disarmed_at_end',
                'no_failsafe','obstacle_clearance','landed_confirmed')

def expected_checks(name):
    """Every named check a guardian scenario must produce, derived from what it expects to show."""
    scenario=SCENARIOS[name];expect=scenario['expect']
    out=[f'{g["ns"]}_{c}' for g in GUARDIANS for c in VEHICLE_CHECKS]
    out+=['fleet_min_separation','landings_sequenced','rosbag_recorded','no_runner_error','messages_well_formed',
          'scenario_started','never_closed_on_threat','authority_respected']
    if scenario['threats']:out.append('threats_flew')
    if any(s['kind']!='bird' for s in scenario['threats'].values()):out.append('guardians_kept_clear')
    out+=['threat_confirmed','red_before_arrival'] if expect['red'] else ['no_red_alert','guardians_held_their_posts']
    for key,check in (('relocate','carrier_relocated_clear'),('station_order','station_ordered_keep_clear'),
                      ('jammed','jammed_guardian_acted_alone'),('tracks','distinct_tracks_confirmed'),('disperse','dispersed_before_impact')):
        if expect.get(key):out.append(check)
    if expect.get('spoof'):out+=['spoofing_detected','navigated_by_station_fixes','drift_bounded']
    out.append('recovery_by_center' if expect['recovery']=='center' else 'recovery_under_delegation')
    return out

def impact_time(spec):
    """Seconds after its start at which a diving threat reaches its aim."""
    s=spec['start'];a=spec['aim'];return math.dist(s,[a[0],a[1],a[2] if len(a)>2 else s[2]])/spec['speed']

def threat_position(spec,t):
    """Scripted position of a threat t seconds after it started, and whether its script has ended."""
    if spec['motion']=='circle':
        a=spec['speed']*t/spec['radius'];c=spec['center']
        return [c[0]+spec['radius']*math.cos(a),c[1]+spec['radius']*math.sin(a),spec['alt']],False
    s=spec['start'];a=spec['aim'];aim3=[a[0],a[1],a[2] if len(a)>2 else s[2]]
    d=[q-p for p,q in zip(s,aim3)];n=math.dist(s,aim3);k=spec['speed']*t/n
    # A dive reaches its aim (impact) at k = 1, then carries on into the ground and is gone.
    if spec['motion']=='dive' and k>=1.25:return [p+c*1.25 for p,c in zip(s,d)],True
    return [p+c*k for p,c in zip(s,d)],False

def start_pose(spec):
    return threat_position(spec,0.)[0]

def reserved_for(ns):
    """Cells a guardian's routes and moves keep out of: the 3 x 3 cells around every other guardian's post.
    Pre-briefed transit once let one guardian pass a metre under another holding its post."""
    own=next(g['post'] for g in GUARDIANS if g['ns']==ns)
    return {(g['post'][0]+dx,g['post'][1]+dy) for g in GUARDIANS if g['post']!=own for dx in (-1,0,1) for dy in (-1,0,1)}

def free_space(start,p,others=(),blocked=None,reserved=(),wide=()):
    """Whether a move from start to p stays in the arena, off the known cylinders (or the blocked cells a
    vehicle has mapped) and the reserved cells, below the ceiling, at least 2 m from other guardians and
    outside the (position, radius) circles in `wide` (guardians last heard some time ago)."""
    from world import OBSTACLES
    if not (GRID[0]+0.3<=p[0]<=GRID[1]-0.3 and GRID[0]+0.3<=p[1]<=GRID[1]-0.3) or math.hypot(p[0],p[1])>18.:return False
    if p[2]>CFG.max_altitude:return False
    def segment_distance(point):
        delta=[b-a for a,b in zip(start[:2],p[:2])];length2=sum(v*v for v in delta)
        t=0. if length2==0. else max(0.,min(1.,sum((q-a)*v for q,a,v in zip(point,start,delta))/length2))
        return math.hypot(*[a+t*v-q for a,v,q in zip(start,delta,point)])
    if any(segment_distance(o)<2. for o in others):return False
    if any(segment_distance(o)<r for o,r in wide):return False
    steps=max(1,int(math.dist(start[:2],p[:2])/0.25))
    for k in range(steps+1):
        q=[a+(b-a)*k/steps for a,b in zip(start[:2],p[:2])];cell=(round(q[0]),round(q[1]))
        if cell in reserved:return False
        if blocked is not None:
            if cell in blocked:return False
        elif any(math.hypot(q[0]-x,q[1]-y)<r+1.0 for x,y,r in OBSTACLES):return False
    return True

# Visual-only threat models, added to the guardian world template for each scenario.
_ROTORS=''.join(f'<visual name="rotor_{i}"><pose>{x} {y} 0.05 0 0 0</pose><geometry><cylinder><radius>0.16</radius><length>0.01</length>'
                f'</cylinder></geometry><material><ambient>0.9 0.3 0.3 1</ambient><diffuse>0.95 0.35 0.35 1</diffuse></material></visual>'
                for i,(x,y) in enumerate(((0.32,0.32),(-0.32,0.32),(0.32,-0.32),(-0.32,-0.32))))
VISUALS={
    'drone':('<visual name="hull"><geometry><box><size>0.34 0.34 0.12</size></box></geometry><material><ambient>0.75 0.08 0.08 1</ambient>'
             '<diffuse>0.85 0.1 0.1 1</diffuse></material></visual>'
             '<visual name="arm_a"><pose>0 0 0.02 0 0 0.785</pose><geometry><box><size>0.9 0.05 0.03</size></box></geometry><material>'
             '<ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>'
             '<visual name="arm_b"><pose>0 0 0.02 0 0 -0.785</pose><geometry><box><size>0.9 0.05 0.03</size></box></geometry><material>'
             '<ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>'+_ROTORS),
    'fast':('<visual name="body"><pose>0 0 0 0 1.5708 0</pose><geometry><cylinder><radius>0.1</radius><length>1.1</length></cylinder></geometry>'
            '<material><ambient>0.35 0.36 0.38 1</ambient><diffuse>0.45 0.46 0.48 1</diffuse></material></visual>'
            '<visual name="tail"><pose>-0.5 0 0 0 0 0</pose><geometry><box><size>0.12 0.5 0.04</size></box></geometry>'
            '<material><ambient>0.7 0.1 0.1 1</ambient><diffuse>0.8 0.12 0.12 1</diffuse></material></visual>'),
    'bird':('<visual name="body"><geometry><sphere><radius>0.12</radius></sphere></geometry><material><ambient>0.35 0.25 0.15 1</ambient>'
            '<diffuse>0.45 0.32 0.2 1</diffuse></material></visual>'
            '<visual name="wings"><geometry><box><size>0.12 0.7 0.02</size></box></geometry><material><ambient>0.3 0.22 0.14 1</ambient>'
            '<diffuse>0.4 0.3 0.18 1</diffuse></material></visual>'),
}

def world_sdf(name):
    """The guardian world for one scenario: the template plus its threat models, post markers and jamming zone."""
    scenario=SCENARIOS[name];template=(ROOT/'simulation/worlds/guardian.sdf').read_text(encoding='utf-8')
    colours=('0.1 0.7 0.6','0.95 0.7 0.2','0.55 0.45 0.95');parts=[]
    def disc(model,x,y,rgb,r=0.6,alpha=1.):
        return (f'<model name="{model}"><static>true</static><pose>{x} {y} 0.005 0 0 0</pose><link name="link"><visual name="visual">'
                f'<transparency>{1-alpha:g}</transparency><geometry><cylinder><radius>{r}</radius><length>0.01</length></cylinder></geometry>'
                f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual></link></model>')
    for i,g in enumerate(GUARDIANS):parts.append(disc(f'post_{i}',*g['post'],colours[i]))
    if scenario.get('jammer'):parts.append(disc('jamming_zone',*scenario['jammer']['p'],'0.85 0.2 0.2',r=scenario['jammer']['r'],alpha=0.35))
    for model,spec in scenario['threats'].items():
        p=start_pose(spec);yaw=0.;pitch=0.
        if spec['motion']=='dive':  # Point the fast object along its dive.
            a=spec['aim'];yaw=math.atan2(a[1]-p[1],a[0]-p[0]);pitch=math.atan2(p[2]-a[2],math.dist(p[:2],a[:2]))
        parts.append(f'<model name="{model}"><pose>{p[0]} {p[1]} {p[2]} 0 {pitch} {yaw}</pose><link name="body"><gravity>false</gravity>'
                     '<inertial><mass>1.5</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.04</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
                     f'{VISUALS[spec["kind"]]}</link>'
                     f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"><topic>/model/{model}/cmd_vel</topic></plugin>'
                     '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"><odom_frame>world</odom_frame>'
                     f'<robot_base_frame>{model}</robot_base_frame><odom_publish_frequency>20</odom_publish_frequency><dimensions>3</dimensions></plugin></model>')
    marker='<!-- SCENARIO MODELS -->'
    if marker not in template:raise RuntimeError('guardian.sdf lacks the scenario marker')
    return template.replace(marker,'\n  '.join(parts))
