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
from common import JsonLog

DECK=0.6
AIRBORNE=2.0      # m world altitude: a vehicle counts as airborne above this.
LAUNCH_GAP=4.0    # s between successive launch clearances.

class Station(Node):
    def __init__(self,args):
        super().__init__('fleet_station');self.args=args;self.log=JsonLog(args.log)
        self.drones=args.drones.split(',');self.states={};self.carrier=None
        self.clear={d:{'launch':False,'land':False} for d in self.drones}
        self.flown=set();self.landed=set();self.last_launch=-1e9;self.moving=False;self.parked=False
        self.min_separation=math.inf;self.last_log=0.
        self.create_subscription(Odometry,'/model/carrier/odometry',self.on_carrier,10)
        for d in self.drones:self.create_subscription(String,f'/fleet/{d}/state',lambda m,d=d:self.on_state(d,m),10)
        self.drive=self.create_publisher(Twist,'/model/carrier/cmd_vel',10)
        self.clearance=self.create_publisher(String,'/fleet/clearance',10)
        self.create_timer(0.1,self.tick)
    def on_carrier(self,msg):
        q=msg.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        self.carrier={'e':p.x,'n':p.y,'yaw':yaw,'speed':math.hypot(v.x,v.y)}
    def on_state(self,drone,msg):
        try:state=json.loads(msg.data)
        except ValueError:return
        self.states[drone]=state
        if state['armed'] and state['position'][2]>AIRBORNE:self.flown.add(drone)
        if drone in self.flown and not state['armed'] and drone not in self.landed:
            self.landed.add(drone);self.log.write('touchdown',drone=drone,position=state['position'],
                                                  pad_error=state['pad_error'],carrier=self.carrier)
    def grant(self,drone,kind):
        self.clear[drone][kind]=True;self.log.write('clearance',drone=drone,grant=kind,carrier=self.carrier)
    def now(self):return self.get_clock().now().nanoseconds/1e9  # Simulation time under use_sim_time.
    def tick(self):
        now=self.now()
        if len(self.states)==len(self.drones):
            # Launch in order, each after the previous one is airborne.
            for i,d in enumerate(self.drones):
                if self.clear[d]['launch']:continue
                if (i==0 or self.drones[i-1] in self.flown) and now-self.last_launch>=LAUNCH_GAP:
                    self.grant(d,'launch');self.last_launch=now
                break
            # Drive once every vehicle is up and the first heads home, so recoveries meet a moving deck.
            homeward=any(s['phase'] in ('return','rendezvous','descend') for s in self.states.values())
            if not self.moving and not self.parked and homeward and len(self.flown)==len(self.drones):
                self.moving=True;self.log.write('carrier_start',carrier=self.carrier)
            # One landing at a time, lowest altitude layer first among the vehicles holding overhead.
            busy=[d for d in self.drones if self.clear[d]['land'] and d not in self.landed]
            holding=[d for d in self.drones if self.states[d]['phase']=='rendezvous' and d not in self.landed]
            if not busy and holding:self.grant(min(holding,key=lambda d:self.states[d]['layer']),'land')
        if self.moving and (len(self.landed)==len(self.drones) or (self.carrier and self.carrier['e']>=self.args.max_east)):
            # Parked for good: landed vehicles still report their last phase, which must not restart the drive.
            self.moving=False;self.parked=True;self.log.write('carrier_stop',carrier=self.carrier)
        twist=Twist();twist.linear.x=self.args.speed if self.moving else 0.;self.drive.publish(twist)
        message=String();message.data=json.dumps(self.clear);self.clearance.publish(message)
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
