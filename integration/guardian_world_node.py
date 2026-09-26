#!/usr/bin/env python3
"""SITL-only environment for the guardian scenario: the intruder, the sensors and the jammed link.

Everything here stands in for the physical world, not for software under test. It flies the intruder along
a scripted line (Gazebo moves the model), turns Gazebo truth into noisy detections for each guardian and for
the station, and relays every station <-> guardian message except while a guardian is inside the jamming
zone. Guardians always receive their own sensor's detections: jamming cuts the link, not the sensor.
"""
import argparse
import json
import math
import random
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from common import SENSOR_QOS,JsonLog,px4_topic
from guardian_layout import GUARDIANS,INTRUDER,JAMMER,SENSORS

class World(Node):
    def __init__(self,args):
        super().__init__('guardian_world');self.log=JsonLog(args.log);self.rng=random.Random(args.seed)
        self.spawn={g['ns']:[args.carrier[0]+g['pad'],args.carrier[1],args.deck] for g in GUARDIANS}
        self.pose={};self.phase={};self.watch_since=None;self.started=None;self.jamming=False;self.jam_done=False
        self.intruder=None;self.carrier=None;self.next_scan=0.;self.dropped={g['ns']:0 for g in GUARDIANS};self.link={}
        self.create_subscription(Odometry,'/model/intruder/odometry',lambda m:self.odometry('intruder',m),10)
        self.create_subscription(Odometry,'/model/carrier/odometry',lambda m:self.odometry('carrier',m),10)
        self.drive=self.create_publisher(Twist,'/model/intruder/cmd_vel',10)
        self.station_detections=self.create_publisher(String,'/station/detections',10)
        self.onboard={};self.uplink={};self.relay_state={}
        for g in GUARDIANS:
            ns=g['ns']
            self.create_subscription(VehicleLocalPosition,px4_topic('vehicle_local_position',VehicleLocalPosition,ns=ns),
                                     lambda m,ns=ns:self.local(ns,m),SENSOR_QOS)
            self.create_subscription(String,f'/{ns}/guardian/state',lambda m,ns=ns:self.state(ns,m),10)
            self.onboard[ns]=self.create_publisher(String,f'/{ns}/guardian/detections',10)
            self.uplink[ns]=self.create_publisher(String,f'/{ns}/guardian/uplink',10)
            self.relay_state[ns]=self.create_publisher(String,f'/fleet/{ns}/state',10)
        self.create_subscription(String,'/station/uplink',self.station_uplink,10)
        self.create_timer(0.05,self.tick)
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def odometry(self,name,m):
        p=m.pose.pose.position;v=m.twist.twist.linear
        setattr(self,name,{'p':[p.x,p.y,p.z],'v':[v.x,v.y,v.z]})
    def local(self,ns,m):
        s=self.spawn[ns];self.pose[ns]=[m.y+s[0],m.x+s[1],-m.z+s[2]]
    def jammed(self,ns):
        return self.jamming and ns in self.pose and math.dist(self.pose[ns][:2],JAMMER['p'])<JAMMER['r']
    def relay(self,publisher,message,ns,kind):
        if self.jammed(ns):
            self.dropped[ns]+=1
            if self.dropped[ns]%25==1:self.log.write('dropped',ns=ns,message=kind,count=self.dropped[ns],t=self.now())
            return
        publisher.publish(message)
    def state(self,ns,msg):
        try:self.phase[ns]=json.loads(msg.data).get('phase')
        except ValueError:return
        self.relay(self.relay_state[ns],msg,ns,'state')
    def station_uplink(self,msg):
        for ns,publisher in self.uplink.items():self.relay(publisher,msg,ns,'uplink')
    def tick(self):
        now=self.now()
        if now<=0.:return
        # The intruder starts once every guardian has held its watch post for a few seconds.
        if self.started is None:
            watching=len(self.phase)==len(GUARDIANS) and all(p=='watch' for p in self.phase.values())
            self.watch_since=(self.watch_since or now) if watching else None
            if watching and now-self.watch_since>=INTRUDER['watch_before_start']:
                self.started=now;self.jamming=True;self.log.write('intruder_start',t=now,jammer=JAMMER)
        twist=Twist()
        if self.started is not None and self.intruder:
            p=self.intruder['p'];s=INTRUDER['start'];a=INTRUDER['aim'];d=[a[0]-s[0],a[1]-s[1]];n=math.hypot(*d)
            if p[1]>INTRUDER['stop_y']:
                twist.linear.x=INTRUDER['speed']*d[0]/n;twist.linear.y=INTRUDER['speed']*d[1]/n
                twist.linear.z=0.5*(s[2]-p[2])  # Hold its altitude.
            if self.jamming and p[1]<JAMMER['off_y']:self.jamming=False;self.log.write('jammer_off',t=now)
        self.drive.publish(twist)
        for ns in self.pose:  # Record each guardian's link going down and coming back, for the evidence.
            jammed=self.jammed(ns)
            if jammed!=self.link.get(ns,False):self.link[ns]=jammed;self.log.write('link',ns=ns,jammed=jammed,t=now)
        if self.intruder and self.started is not None:
            self.log.write('intruder',t=now,p=self.intruder['p'],v=self.intruder['v'],jamming=self.jamming)
        if now>=self.next_scan:self.next_scan=now+SENSORS['guardian']['period'];self.scan(now)
    def detect(self,sensor,origin):
        """A noisy detection of the intruder from origin, with a camera label when close enough, or None."""
        if self.started is None or not self.intruder:return None
        p=self.intruder['p'];r=math.dist(origin,p)
        if r>sensor['range'] or self.rng.random()>sensor['pd']:return None
        label=None
        if r<=sensor.get('classify_range',-1):label='drone' if self.rng.random()<sensor['accuracy'] else 'bird'
        return {'p':[c+self.rng.gauss(0,sensor['sigma']) for c in p],'label':label,'sigma':sensor['sigma'],'t':self.now()}
    def scan(self,now):
        for ns,pose in self.pose.items():
            d=self.detect(SENSORS['guardian'],pose)
            if d is None:continue
            message=String();message.data=json.dumps({'source':ns,'detections':[d]})
            self.onboard[ns].publish(message)
            self.relay(self.station_detections,message,ns,'detections')
        if self.carrier:
            d=self.detect(SENSORS['station'],self.carrier['p'])
            if d:message=String();message.data=json.dumps({'source':'station','detections':[d]});self.station_detections.publish(message)

def main():
    p=argparse.ArgumentParser();p.add_argument('--log',required=True);p.add_argument('--seed',type=int,default=1)
    p.add_argument('--carrier',type=float,nargs=2,required=True);p.add_argument('--deck',type=float,required=True)
    args=p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init();node=World(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
