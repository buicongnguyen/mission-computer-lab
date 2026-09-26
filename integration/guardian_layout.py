"""Layout and parameters of the PX4 guardian scenario.

The arena is 20 m across, so speeds, ranges and radii are the fast simulator's scaled down; the decision
code (tools/guardian.py) is the same. An intruder flies a straight line from the north to where the carrier
is parked. It passes close to two watch posts: the first guardian to see it reports over the network, the
second is jammed as the intruder arrives and has to keep clear on its own. The third post is far away.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from guardian import Config

CFG=Config(gate=0.8,gate_speed=4.,accel_noise=0.3,sigma=0.15,confirm_hits=4,confirm_window=2.,drop_tentative=1.5,
           drop_confirmed=4.,protect_radius=3.,warn_horizon=30.,slow_persist=1.5,slow_speed=0.5,inner_radius=6.,
           clear_radius=3.,safe_radius=1.5,order_horizon=12.,reflex_horizon=4.,keep_clear_step=3.,climb_step=1.,
           max_altitude=6.,guardian_speed=1.2,fast_speed=5.,clear_time=5.,lost_link=1.5,lost_link_return=30.,
           center_timeout=40.,integrity_threshold=1.,integrity_persistence=3)
# Each guardian launches from its pad, holds a watch post at its own altitude layer, and lands back on its pad.
GUARDIANS=[{'ns':'px4_0','pad':-1.1,'post':(1,2),'altitude':4.},   # In the path, jammed as the intruder arrives.
           {'ns':'px4_1','pad':0.,'post':(0,12),'altitude':5.},    # First contact; reports over the network.
           {'ns':'px4_2','pad':1.1,'post':(12,8),'altitude':3.}]   # Far from the path.
INTRUDER={'start':(-3.4,23.,4.),'aim':(0.,-1.5),'speed':1.2,'stop_y':-25.,'watch_before_start':5.}
JAMMER={'p':(1.5,3.5),'r':2.5,'off_y':-1.}  # Jams inside r from the intruder's start until it passes off_y.
RELOCATE={'x':8.,'speed':0.8}  # On RED the station drives the carrier east along the road to here.
SENSORS={'guardian':{'range':8.,'pd':0.9,'sigma':0.15,'period':0.2,'classify_range':5.,'accuracy':0.9},
         'station':{'range':6.,'pd':0.9,'sigma':0.1,'period':0.2}}
CENTER={'latency':0.5,'decision_time':3.}
GRID=(-2,12)  # Planning grid bounds (cells), as in tools/world.py.

def free_space(start,p,others=(),blocked=None):
    """Whether a keep-clear move from start to p stays in the arena, off the known cylinders (or the blocked
    cells a vehicle has mapped), below the ceiling and at least 2 m from other guardians."""
    import math
    from world import OBSTACLES
    if not (GRID[0]+0.3<=p[0]<=GRID[1]-0.3 and GRID[0]+0.3<=p[1]<=GRID[1]-0.3) or math.hypot(p[0],p[1])>18.:return False
    if p[2]>CFG.max_altitude or any(math.dist(p[:2],o[:2])<2. for o in others):return False
    steps=max(1,int(math.dist(start[:2],p[:2])/0.25))
    for k in range(steps+1):
        q=[a+(b-a)*k/steps for a,b in zip(start[:2],p[:2])]
        if blocked is not None:
            if (round(q[0]),round(q[1])) in blocked:return False
        elif any(math.hypot(q[0]-x,q[1]-y)<r+1.0 for x,y,r in OBSTACLES):return False
    return True
