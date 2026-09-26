#!/usr/bin/env python3
"""SITL-only guardian station on the carrier: fuses detections, sets the protection posture, orders the
guardians and relocates the carrier within its delegated authority, and asks the center for recovery.

It never commands a vehicle's flight directly: orders go over the (jammable) uplink to each guardian's own
adapter, which keeps its C++ supervisor, its PX4 failsafes and its onboard keep-clear reflex.
"""
import argparse
import json
import math
import sys
import rclpy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from fleet_station_node import Station
from guardian import RED,GREEN,Posture,Tracker,assess,station_orders
from guardian_layout import CFG,GUARDIANS,RELOCATE,free_space

class GuardianStation(Station):
    def __init__(self,args):
        super().__init__(args,name='guardian_station')
        self.tracker=Tracker(CFG);self.posture=Posture(CFG);self.orders={};self.recovery=None;self.recovery_by=None
        self.held=False;self.clear_since=None;self.pending_since=-1e9;self.relocating=False;self.relocated=False
        self.heard={};self.logged_tracks=set();self.last_orders={}
        self.posts={g['ns']:[float(g['post'][0]),float(g['post'][1]),g['altitude']] for g in GUARDIANS}
        self.uplink=self.create_publisher(String,'/station/uplink',10)
        self.center_report=self.create_publisher(String,'/center/report',10)
        self.create_subscription(String,'/station/detections',self.on_detections,10)
        self.create_subscription(String,'/center/decision',self.on_decision,10)
    def on_state(self,drone,msg):
        self.heard[drone]=self.now();super().on_state(drone,msg)
    def on_detections(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        for d in data['detections']:self.tracker.update(d['t'],[(d['p'],data['source'],d.get('label'),d.get('sigma',CFG.sigma))])
    def on_decision(self,msg):
        try:decision=json.loads(msg.data)['decision']
        except (ValueError,KeyError):return
        self.log.write('center_decision',decision=decision,t=self.now())
        if decision in ('recover','resume') and self.recovery is None and self.held:
            self.recovery=decision;self.recovery_by='center';self.log.write('recovery',by='center',decision=decision,t=self.now())
    def send_center(self,now,kind,**fields):
        message=String();message.data=json.dumps({'kind':kind,**fields});self.center_report.publish(message)
        self.log.write('center_report',report=kind,t=now,**fields)
    def station_pose(self):
        c=self.carrier or {'e':0.,'n':0.,'yaw':0.,'speed':0.}
        return [c['e'],c['n'],0.],[c['speed']*math.cos(c['yaw']),c['speed']*math.sin(c['yaw']),0.]
    def tick(self):
        now=self.now()
        if now>0.:self.protect(now)
        super().tick()
    def protect(self,now):
        self.tracker.update(now,[])  # Drop tracks that have gone quiet.
        tracks=self.tracker.confirmed();sp,sv=self.station_pose()
        for tr in tracks:
            if tr.id not in self.logged_tracks:
                self.logged_tracks.add(tr.id)
                self.log.write('confirmed',track=tr.id,p=tr.p,v=tr.v,sources=sorted(tr.sources),cls=tr.cls,t=now)
        levels=[assess(tr,now,sp,sv,CFG) for tr in tracks]
        before=self.posture.state
        if self.posture.update(now,[l for l,_,_ in levels]):
            state=self.posture.state;self.log.write('posture',state=state,t=now,tracks=[tr.id for tr in tracks])
            self.send_center(now,'posture',state=state)
            if state==RED:
                self.log.write('crew_alert',t=now)
                if not self.relocated and not self.relocating:
                    self.relocating=True;self.log.write('relocate',t=now,carrier=self.carrier,target_x=RELOCATE['x'])
            if before==RED:self.held=True
            if state==GREEN and self.held:self.clear_since=now
        if self.held and self.posture.state==GREEN and self.recovery is None:
            if now-self.pending_since>=5.:self.send_center(now,'clear_after_red');self.pending_since=now
            if self.clear_since is not None and now-self.clear_since>=CFG.center_timeout:  # Center unreachable.
                self.recovery='recover';self.recovery_by='station (delegated)';self.log.write('recovery',by=self.recovery_by,t=now)
        watching={d:s for d,s in self.states.items() if s.get('phase')=='watch'}
        view={d:{'p':s['position'],'post':self.posts[d]} for d,s in watching.items()}
        def free_for(name):
            others=[s['position'] for d,s in self.states.items() if d!=name and s.get('armed')]
            return lambda p:free_space(view[name]['p'],p,others)
        orders=station_orders(now,self.posture.state,tracks,view,CFG,free_for=free_for,recovery=self.recovery,
                              held=self.held and self.recovery is None,previous=self.orders)
        for name,order in orders.items():
            if order['action'] in ('recover','resume'):order['authority']=self.recovery_by
            last=self.last_orders.get(name)
            if last is None or last['action']!=order['action'] or last.get('target')!=order.get('target'):
                self.log.write('order',ns=name,layer='station',t=now,own=view[name]['p'],
                               threats=[tr.predict(now) for tr in tracks],**{k:v for k,v in order.items() if k!='tracks'})
                self.last_orders[name]=order
        self.orders=orders
    def carrier_speed(self,now):
        """Relocation on RED is the station's delegated protective action: drive east out of the threat's path."""
        if self.relocating and self.carrier and self.carrier['e']>=RELOCATE['x']:
            self.relocating=False;self.relocated=True;self.log.write('relocated',t=now,carrier=self.carrier)
        return RELOCATE['speed'] if self.relocating else 0.
    def publish_clearance(self,now):
        message=String()
        message.data=json.dumps({'clearance':self.clear,'orders':self.orders,'posture':self.posture.state,'t':now})
        self.uplink.publish(message)

def main():
    p=argparse.ArgumentParser();p.add_argument('--drones',required=True);p.add_argument('--log',required=True)
    p.add_argument('--speed',type=float,default=0.);p.add_argument('--max-east',type=float,default=14.)
    args=p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init();node=GuardianStation(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
