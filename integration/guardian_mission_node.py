#!/usr/bin/env python3
"""SITL-only guardian vehicle: launch from the carrier, hold a watch post, keep clear of intruders, land back.

The safety chain is the fleet vehicle's (C++ supervisor, freshness gates, PX4 failsafes and handoff). On
watch it runs the shared guardian decision (tools/guardian.py): it follows the station's orders from the
uplink, but keeps clear on its own when its sensor predicts a conflict inside the reflex horizon, and holds
position on its own when the uplink goes quiet. It never steers toward a track.
"""
import argparse
import json
import math
import sys
import rclpy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from fleet_mission_node import FleetMission
from mission_node import Mission
from px4_msgs.msg import VehicleCommand
from guardian import INF,Tracker,onboard_decide,relevant
from guardian_layout import CFG,free_space,reserved_for
from world import astar

class GuardianMission(FleetMission):
    CLEARANCE_TOPIC='/{ns}/guardian/uplink'  # Relayed by the environment, which drops it while jammed.
    STATE_TOPIC='/{ns}/guardian/state'
    def __init__(self,args):
        super().__init__(args)
        self.post=[float(args.goal[0]),float(args.goal[1]),self.altitude]
        self.order={'action':'watch','target':self.post};self.last_uplink=None;self.onboard=Tracker(CFG)
        self.action=None;self.layer=None;self.hold_at=None;self.reflex=None;self.guard_target=None;self.posture=None
        self.nav_fix=None;self.nav_offset=None  # Station fixes, once the station finds this vehicle's GNSS untrustworthy.
        self.nav_fix_at=None;self.rally=None;self.rally_route=None
        self.create_subscription(String,f'/{self.ns}/guardian/detections',self.on_detections,10)
    def reserved_cells(self):
        """Pre-briefed transit keeps clear of the other guardians' watch posts: one guardian's route once passed
        a metre under another holding its post (the altitude layers are only 1 m apart)."""
        return reserved_for(self.ns)
    def on_clearance(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        self.clearance=data['clearance'].get(self.ns,{});self.order=data['orders'].get(self.ns) or self.order
        self.posture=data.get('posture');self.last_uplink=self.now()
        if self.carrier:self.rally=[self.carrier[0],self.carrier[1],self.altitude]  # Pre-briefed: last heard, not live.
        if (data.get('nav_mode') or {}).get(self.ns)=='gnss' and self.nav_offset is not None:
            # The station's cross-check has cleared: back to GNSS.
            self.nav_offset=self.nav_fix=self.nav_fix_at=None
            self.log.write('guardian',action='navigate_by_gnss',layer='station',target=None,own=None,uptime=self.now(),
                           link_age=0.,threats=None)
        fix=(data.get('nav') or {}).get(self.ns)
        if fix and (self.nav_fix is None or fix[3]>self.nav_fix[3]):self.station_fix(fix)
    def station_fix(self,fix):
        """Blend a new station fix into the offset between true and GNSS-based position (horizontal only)."""
        if not self.track:return
        t,estimate=min(self.track,key=lambda item:abs(item[0]-fix[3]))
        offset=[fix[0]-estimate[0],fix[1]-estimate[1]]
        first=self.nav_offset is None
        self.nav_offset=offset if first else [0.7*a+0.3*b for a,b in zip(self.nav_offset,offset)];self.nav_fix=fix
        self.nav_fix_at=self.now()
        if first:self.log.write('guardian',action='navigate_by_station',layer='station',target=None,own=estimate,uptime=self.now(),
                                link_age=0.,threats=None,offset=offset)
    def navigation(self,now,position):
        if self.nav_offset is None:return position
        return [position[0]+self.nav_offset[0],position[1]+self.nav_offset[1],position[2]]
    def world_position(self):
        raw=super().world_position()
        return None if raw is None else self.navigation(self.now(),raw)
    def on_detections(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        for d in data['detections']:self.onboard.update(d['t'],[(d['p'],self.ns,d.get('label'),d.get('sigma',CFG.sigma))])
    def extra_state(self):
        raw=FleetMission.world_position(self)
        return {'action':self.action,'layer':self.layer,'posture':self.posture,'gnss_position':raw,
                'nav':'station' if self.nav_offset is not None else 'gnss',
                'link_age':None if self.last_uplink is None else round(self.now()-self.last_uplink,2)}
    def next_target(self,now,position):
        if self.phase=='outbound':
            target,done=Mission.next_target(self,now,position)
            if done:self.set_phase('watch',now,position=position)
            return target,False
        if self.phase=='watch':return self.guard(now,position),False
        return super().next_target(now,position)
    def guard(self,now,position):
        self.onboard.update(now,[]);tracks=self.onboard.confirmed()
        link_age=INF if self.last_uplink is None else now-self.last_uplink
        fix_age=None if self.nav_offset is None else now-self.nav_fix_at
        action,target,layer,detail=onboard_decide(now,position,tracks,link_age,self.order,CFG,
                                                  free=lambda p:free_space(position,p,blocked=self.blocked),
                                                  rally=self.rally,previous=self.reflex,fix_age=fix_age)
        self.reflex=detail if (action,layer)==('keep_clear','onboard') else None
        if layer=='station' and self.order.get('authority'):layer=self.order['authority']
        if action=='recover':
            self.log_decision(now,position,action,layer,None,tracks,link_age,detail)
            return self.start_return(now,position)
        if action=='land_in_place':
            # Its GNSS is known to be spoofed and the station's fixes have stopped: no position can be trusted.
            self.log_decision(now,position,action,layer,None,tracks,link_age,detail)
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND);self.land_requested=True
            self.handed_off=True;self.log.write('handoff',reason='navigation_unverified')
            return list(position)
        if action=='lost_link_return':
            # Fly a planned route to the rally point (clear of the cylinders and the other posts), then hold
            # there for a landing clearance; if no route exists, hold where it is.
            if self.rally_route is None:
                cell=lambda q:(min(12,max(-2,round(q[0]))),min(12,max(-2,round(q[1]))))
                path=astar(cell(position),cell(target),self.blocked)
                self.rally_route=[[float(x),float(y),self.altitude] for x,y in path[1:]] or [list(position)]
            while len(self.rally_route)>1 and math.dist(position[:2],self.rally_route[0][:2])<0.5:self.rally_route.pop(0)
            target=self.rally_route[0]
        else:self.rally_route=None
        if action in ('hold','lost_link_hold'):
            if self.action!=action or self.hold_at is None:self.hold_at=list(position)
            target=self.hold_at
        elif action in ('watch','resume'):target=self.post
        target=list(target) if target else list(position)
        if (action,layer)!=(self.action,self.layer) or (action in ('keep_clear','disperse') and target!=self.guard_target):
            self.log_decision(now,position,action,layer,target,tracks,link_age,detail)
        self.action,self.layer,self.guard_target=action,layer,target
        return target
    def log_decision(self,now,position,action,layer,target,tracks,link_age,detail):
        self.log.write('guardian',action=action,layer=layer,target=target,own=position,uptime=now,
                       link_age=None if math.isinf(link_age) else round(link_age,2),
                       threats=[tr.predict(now) for tr in tracks] if layer=='onboard' else None,
                       avoid=relevant(position,tracks,now,CFG) if layer=='onboard' else None,
                       **{k:detail[k] for k in ('miss','t_cpa','until') if k in detail})

def main():
    p=argparse.ArgumentParser();p.add_argument('--allow-sitl',action='store_true');p.add_argument('--log',required=True)
    p.add_argument('--model-sha256');p.add_argument('--ns',required=True)
    p.add_argument('--spawn',type=float,nargs=3,required=True,help='World ENU of the vehicle at startup (its PX4 local origin)')
    p.add_argument('--goal',type=int,nargs=2,required=True,help='Watch post (grid cell)');p.add_argument('--altitude',type=float,required=True)
    p.add_argument('--pad',type=float,required=True,help='Pad offset along the carrier axis (m)')
    p.add_argument('--system-id',type=int,required=True,help='MAV_SYS_ID of this vehicle (PX4 instance + 1)')
    args=p.parse_args(remove_ros_args(sys.argv)[1:])
    if not args.allow_sitl:p.error('This adapter is simulation-only. Launch through scripts/run_sitl.sh.')
    rclpy.init();node=GuardianMission(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
