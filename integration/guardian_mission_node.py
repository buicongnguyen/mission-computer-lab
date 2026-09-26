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
from guardian import INF,Tracker,onboard_decide
from guardian_layout import CFG,free_space

class GuardianMission(FleetMission):
    CLEARANCE_TOPIC='/{ns}/guardian/uplink'  # Relayed by the environment, which drops it while jammed.
    STATE_TOPIC='/{ns}/guardian/state'
    def __init__(self,args):
        super().__init__(args)
        self.post=[float(args.goal[0]),float(args.goal[1]),self.altitude]
        self.order={'action':'watch','target':self.post};self.last_uplink=None;self.onboard=Tracker(CFG)
        self.action=None;self.layer=None;self.hold_at=None;self.reflex=None;self.guard_target=None;self.posture=None
        self.create_subscription(String,f'/{self.ns}/guardian/detections',self.on_detections,10)
    def on_clearance(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        self.clearance=data['clearance'].get(self.ns,{});self.order=data['orders'].get(self.ns) or self.order
        self.posture=data.get('posture');self.last_uplink=self.now()
    def on_detections(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        for d in data['detections']:self.onboard.update(d['t'],[(d['p'],self.ns,d.get('label'),d.get('sigma',CFG.sigma))])
    def extra_state(self):
        return {'action':self.action,'layer':self.layer,'posture':self.posture,
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
        rally=[self.carrier[0],self.carrier[1],self.altitude] if self.carrier else None
        action,target,layer,detail=onboard_decide(now,position,tracks,link_age,self.order,CFG,
                                                  free=lambda p:free_space(position,p,blocked=self.blocked),
                                                  rally=rally,previous=self.reflex)
        self.reflex=detail if (action,layer)==('keep_clear','onboard') else None
        if layer=='station' and self.order.get('authority'):layer=self.order['authority']
        if action=='recover':
            self.log_decision(now,position,action,layer,None,tracks,link_age,detail)
            return self.start_return(now,position)
        if action in ('hold','lost_link_hold'):
            if self.action!=action or self.hold_at is None:self.hold_at=list(position)
            target=self.hold_at
        elif action in ('watch','resume'):target=self.post
        target=list(target) if target else list(position)
        if (action,layer)!=(self.action,self.layer) or (action=='keep_clear' and target!=self.guard_target):
            self.log_decision(now,position,action,layer,target,tracks,link_age,detail)
        self.action,self.layer,self.guard_target=action,layer,target
        return target
    def log_decision(self,now,position,action,layer,target,tracks,link_age,detail):
        self.log.write('guardian',action=action,layer=layer,target=target,own=position,uptime=now,
                       link_age=None if math.isinf(link_age) else round(link_age,2),
                       threats=[tr.predict(now) for tr in tracks] if layer=='onboard' else None,
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
