#!/usr/bin/env python3
"""SITL-only fleet vehicle: launch from the carrier, fly an inspection leg, land back on the moving carrier.

Everything safety-related stays in the base adapter: the C++ supervisor, freshness gates, handoff to PX4
and scan-driven replanning. This class only sequences the mission and aims at the moving pad.
"""
import argparse
import json
import math
import time
import sys
import rclpy
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus
from mission_node import Mission
from world import astar

DECK=0.6      # Carrier deck height (m, world).
LEAD=1.0      # s. The supervisor commands (target - position) at gain 1/s, so aiming one second ahead
              # of the pad makes the commanded velocity equal the carrier's velocity when aligned.
ROAD_ROW=-1   # Grid row beside the road, south of every obstacle: returns join the carrier there.
DESCENT=0.6   # m below the current height: limits the descent rate to about 0.6 m/s.
GO_AROUND=0.5 # m of horizontal misalignment during descent that aborts back to the hold altitude.

class FleetMission(Mission):
    CLEARANCE_TOPIC='/fleet/clearance'   # Station -> vehicle; the guardian scenario routes these through a link.
    STATE_TOPIC='/fleet/{ns}/state'      # Vehicle -> station.
    def __init__(self,args):
        super().__init__(args)
        self.pad=args.pad;self.phase='preflight';self.phase_since=0.;self.inspect_until=None
        self.carrier=None;self.clearance={}
        self.create_subscription(Odometry,'/model/carrier/odometry',self.on_carrier,10)
        self.create_subscription(String,self.CLEARANCE_TOPIC.format(ns=self.ns),self.on_clearance,10)
        self.state=self.create_publisher(String,self.STATE_TOPIC.format(ns=self.ns),10)
        self.create_timer(0.2,self.report)  # Keeps reporting after handoff, so the station sees touchdown.
    def on_carrier(self,msg):
        q=msg.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        v=msg.twist.twist.linear  # Body frame; rotate into world ENU.
        self.carrier=(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw,
                      v.x*math.cos(yaw)-v.y*math.sin(yaw),v.x*math.sin(yaw)+v.y*math.cos(yaw))
    def on_clearance(self,msg):
        try:self.clearance=json.loads(msg.data).get(self.ns,{})
        except ValueError:pass
    def pad_position(self):
        e,n,yaw,_,_=self.carrier
        return e+self.pad*math.cos(yaw),n+self.pad*math.sin(yaw)
    def pad_target(self,altitude):
        (pe,pn),(_,_,_,ve,vn)=self.pad_position(),self.carrier
        return [pe+ve*LEAD,pn+vn*LEAD,altitude]
    def set_phase(self,phase,now,**fields):
        self.phase=phase;self.phase_since=now;self.log.write('phase',phase=phase,uptime=now,**fields)
    def may_request_flight(self):return bool(self.clearance.get('launch'))
    def world_position(self):
        if self.origin is None or self.pose is None:return None
        return [self.pose.y-self.origin[0]+self.spawn[0],self.pose.x-self.origin[1]+self.spawn[1],self.origin[2]-self.pose.z+self.spawn[2]]
    def next_target(self,now,position):
        armed=self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED
        if self.phase=='preflight':
            if armed:self.set_phase('outbound',now,position=position)
            return self.targets[0],False
        if self.phase=='outbound':
            target,done=super().next_target(now,position)
            if done:self.set_phase('inspect',now,position=position);self.inspect_until=now+3.
            return target,False
        if self.phase=='inspect':
            goal=[float(self.goal[0]),float(self.goal[1]),self.altitude]
            if now<self.inspect_until or self.carrier is None:return goal,False
            return self.start_return(now,position),False
        if self.phase=='return':
            target,done=super().next_target(now,position)
            if done:self.set_phase('rendezvous',now,position=position)
            return target,False
        if self.carrier is None:return position,False
        error=math.dist(position[:2],self.pad_position())
        if self.phase=='rendezvous':
            # Hold at this vehicle's altitude layer above its pad until the station clears it to land.
            if self.clearance.get('land') and error<0.3 and now-self.phase_since>2:
                self.set_phase('descend',now,position=position,pad_error=error)
            return self.pad_target(self.altitude),False
        if error>GO_AROUND and position[2]>DECK+0.5:
            self.set_phase('rendezvous',now,position=position,pad_error=error,reason='go_around')
            return self.pad_target(self.altitude),False
        # Descend at a limited rate and press gently onto the deck; PX4 detects touchdown and disarms.
        return self.pad_target(max(DECK-0.3,position[2]-DESCENT)),False
    def start_return(self,now,position):
        """Head south to the road corridor, which no obstacle reaches; the carrier drives along it."""
        entry=(min(12,max(-2,round(position[0]))),ROAD_ROW)
        path=astar((round(position[0]),round(position[1])),entry,self.blocked) or [entry]
        self.goal=entry;self.targets=[[float(x),float(y),self.altitude] for x,y in path];self.waypoint=0
        self.set_phase('return',now,path=path)
        return self.targets[0]
    def report(self):
        position=self.world_position()
        if position is None or self.status is None:return
        pad_error=math.dist(position[:2],self.pad_position()) if self.carrier else None
        state={'phase':self.phase,'position':position,'armed':self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED,
               'handed_off':self.handed_off,'layer':self.altitude,'pad_error':pad_error,'wall_time':time.time(),**self.extra_state()}
        message=String();message.data=json.dumps(state);self.state.publish(message)
    def extra_state(self):return {}  # The guardian vehicle adds its protective action.

def main():
    p=argparse.ArgumentParser();p.add_argument('--allow-sitl',action='store_true');p.add_argument('--log',required=True)
    p.add_argument('--model-sha256');p.add_argument('--ns',required=True)
    p.add_argument('--spawn',type=float,nargs=3,required=True,help='World ENU of the vehicle at startup (its PX4 local origin)')
    p.add_argument('--goal',type=int,nargs=2,required=True);p.add_argument('--altitude',type=float,required=True)
    p.add_argument('--pad',type=float,required=True,help='Pad offset along the carrier axis (m)')
    p.add_argument('--system-id',type=int,required=True,help='MAV_SYS_ID of this vehicle (PX4 instance + 1)')
    args=p.parse_args(remove_ros_args(sys.argv)[1:])
    if not args.allow_sitl:p.error('This adapter is simulation-only. Launch through scripts/run_sitl.sh.')
    rclpy.init();node=FleetMission(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
