#!/usr/bin/env python3
"""SITL-only ground station on a moving carrier: fleet launch/landing clearances and carrier driving.

It never commands a vehicle directly. Each vehicle's own adapter and supervisor decide whether flying is
safe; the station only sequences who may launch or land, drives the carrier and records fleet evidence.
"""
import argparse
import itertools
import json
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common import JsonLog,stamp_seconds,px4_topic,SENSOR_QOS
from px4_msgs.msg import VehicleLandDetected
from fleet_contracts import FreshInput,LAND_TIMEOUT

DECK=0.6
AIRBORNE=2.0      # m world altitude: a vehicle counts as airborne above this.
LAUNCH_GAP=4.0    # s between successive launch clearances.

class Station(Node):
    def __init__(self,args,name='fleet_station'):
        super().__init__(name);self.args=args;self.log=JsonLog(args.log)
        self.drones=args.drones.split(',');self.states={};self.carrier=None
        self.carrier_input=FreshInput();self.state_inputs={d:FreshInput() for d in self.drones}
        self.land_reports={};self.airborne_at={};self.airborne_px4={}
        self.clear={d:{'launch':False,'land':False} for d in self.drones}
        self.flown=set();self.landed=set();self.last_launch=-1e9;self.moving=False;self.parked=False
        self.min_separation=math.inf;self.last_log=0.
        self.create_subscription(Odometry,'/model/carrier/odometry',self.on_carrier,10)
        for d in self.drones:self.create_subscription(String,f'/fleet/{d}/state',lambda m,d=d:self.on_state(d,m),10)
        for d in self.drones:
            self.create_subscription(VehicleLandDetected,px4_topic('vehicle_land_detected',VehicleLandDetected,ns=d),
                                     lambda m,d=d:self.on_land(d,m),SENSOR_QOS)
        self.drive=self.create_publisher(Twist,'/model/carrier/cmd_vel',10)
        self.clearance=self.create_publisher(String,'/fleet/clearance',10)
        self.create_timer(0.1,self.tick)
    def on_carrier(self,msg):
        if not self.carrier_input.accept(stamp_seconds(msg.header.stamp),self.now()):return
        q=msg.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        self.carrier={'e':p.x,'n':p.y,'yaw':yaw,'speed':math.hypot(v.x,v.y)}
    def on_state(self,drone,msg):
        try:state=json.loads(msg.data)
        except ValueError:return False
        if not isinstance(state,dict) or state.get('telemetry_fresh') is not True:return False
        if not self.state_inputs[drone].accept(state.get('t'),self.now()):return False
        self.states[drone]=state
        if state['armed'] and state['position'][2]>AIRBORNE and drone not in self.flown:
            self.flown.add(drone);self.airborne_at[drone]=self.now()
            self.airborne_px4[drone]=state.get('px4_stamp',math.inf)
        land=self.land_reports.get(drone)
        confirmed=(land and land['landed'] and land['ground_contact']
                   and 0<=self.now()-land['received']<=LAND_TIMEOUT
                   and land['received']>self.airborne_at.get(drone,math.inf)
                   and land['timestamp']>self.airborne_px4.get(drone,math.inf)
                   and abs(state.get('px4_stamp',-math.inf)-land['timestamp'])<=LAND_TIMEOUT*1e6)
        if (drone in self.flown and not state['armed'] and drone not in self.landed and confirmed
                and abs(state['position'][2]-DECK)<0.35 and self.carrier_input.fresh(self.now())
                and state.get('pad_error') is not None and state['pad_error']<0.4):
            self.landed.add(drone);self.log.write('touchdown',drone=drone,position=state['position'],
                pad_error=state['pad_error'],carrier=self.carrier,land_timestamp=land['timestamp'])
        return True
    def on_land(self,drone,msg):
        previous=self.land_reports.get(drone)
        if msg.timestamp<=0 or (previous and msg.timestamp<=previous['timestamp']):return
        self.land_reports[drone]={'timestamp':msg.timestamp,'received':self.now(),
                                  'landed':bool(msg.landed),'ground_contact':bool(msg.ground_contact)}
    def streams_fresh(self,now):
        return self.carrier_input.fresh(now) and all(s.fresh(now) for s in self.state_inputs.values())
    def active_clearance(self,now):
        return {d:{kind:bool(value and self.carrier_input.fresh(now) and self.state_inputs[d].fresh(now))
                   for kind,value in clear.items()} for d,clear in self.clear.items()}
    def grant(self,drone,kind):
        self.clear[drone][kind]=True;self.log.write('clearance',drone=drone,grant=kind,carrier=self.carrier)
    def now(self):return self.get_clock().now().nanoseconds/1e9  # Simulation time under use_sim_time.
    def tick(self):
        now=self.now()
        if len(self.states)==len(self.drones) and self.streams_fresh(now):
            self.sequence_launches(now);self.grant_landings()
        twist=Twist();twist.linear.x=self.carrier_speed(now) if self.carrier_input.fresh(now) else 0.;self.drive.publish(twist)
        self.publish_clearance(now);self.record(now)
    def sequence_launches(self,now):
        """Launch in order, each after the previous one is airborne and LAUNCH_GAP after its clearance."""
        for i,d in enumerate(self.drones):
            if self.clear[d]['launch']:continue
            if (i==0 or self.drones[i-1] in self.flown) and now-self.last_launch>=LAUNCH_GAP:
                self.grant(d,'launch');self.last_launch=now
            break
    def grant_landings(self):
        """One landing at a time, lowest altitude layer first among the vehicles holding overhead."""
        busy=[d for d in self.drones if self.clear[d]['land'] and d not in self.landed]
        holding=[d for d in self.drones if self.states[d]['phase']=='rendezvous' and d not in self.landed]
        if not busy and holding:self.grant(min(holding,key=lambda d:self.states[d]['layer']),'land')
    def carrier_speed(self,now):
        """Drive once every vehicle is up and the first heads home, so recoveries meet a moving deck."""
        if not self.streams_fresh(now):return 0.
        if len(self.states)==len(self.drones):
            homeward=any(s['phase'] in ('return','rendezvous','descend') for s in self.states.values())
            if not self.moving and not self.parked and homeward and len(self.flown)==len(self.drones):
                self.moving=True;self.log.write('carrier_start',carrier=self.carrier)
        if self.moving and (len(self.landed)==len(self.drones) or (self.carrier and self.carrier['e']>=self.args.max_east)):
            # Parked for good: landed vehicles still report their last phase, which must not restart the drive.
            self.moving=False;self.parked=True;self.log.write('carrier_stop',carrier=self.carrier)
        return self.args.speed if self.moving else 0.
    def publish_clearance(self,now):
        message=String();message.data=json.dumps({'t':now,'clearance':self.active_clearance(now)});self.clearance.publish(message)
    def record(self,now):
        airborne=[s['position'] for d,s in self.states.items() if s['armed'] and s['position'][2]>DECK+0.5]
        for a,b in itertools.combinations(airborne,2):self.min_separation=min(self.min_separation,math.dist(a,b))
        if now-self.last_log>=0.2 and self.carrier:
            self.last_log=now
            self.log.write('carrier',**self.carrier,moving=self.moving,
                           min_separation=None if math.isinf(self.min_separation) else self.min_separation)

def main():
    p=argparse.ArgumentParser();p.add_argument('--drones',required=True);p.add_argument('--log',required=True)
    p.add_argument('--speed',type=float,default=0.3);p.add_argument('--max-east',type=float,default=14.)
    args=p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init();node=Station(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
