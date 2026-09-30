#!/usr/bin/env python3
"""Independent read-only PX4 observer, survives mission-node termination."""
import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from px4_msgs.msg import VehicleLocalPosition,VehicleStatus,VehicleCommandAck,VehicleLandDetected,FailsafeFlags,SensorGps
from mission_interfaces.msg import Decision,Perception
from sensor_msgs.msg import Image,LaserScan
from common import SENSOR_QOS,JsonLog,mission_topic,px4_topic

class Observer(Node):
    def __init__(self,args):
        ns=args.ns;super().__init__('mission_evidence_observer'+(f'_{ns}' if ns else ''));self.log=JsonLog(args.log)
        self.counts={};self.previous_status=None;self.last_pose_log=0;self.previous_gps=None
        for name,typ,callback in [('vehicle_status',VehicleStatus,self.status),
                                  ('vehicle_local_position',VehicleLocalPosition,self.pose),
                                  ('vehicle_land_detected',VehicleLandDetected,self.land),
                                  ('vehicle_command_ack',VehicleCommandAck,self.ack),
                                  ('failsafe_flags',FailsafeFlags,self.failsafe),
                                  ('vehicle_gps_position',SensorGps,self.gps)]:
            self.create_subscription(typ,px4_topic(name,typ,ns=ns),callback,SENSOR_QOS)
        self.create_subscription(Decision,mission_topic('decision',ns),self.decision,10)
        self.create_subscription(Perception,mission_topic('perception',ns),lambda m:self.count('perception'),SENSOR_QOS)
        self.create_subscription(Image,mission_topic('camera/image',ns),lambda m:self.count('image'),SENSOR_QOS)
        self.create_subscription(LaserScan,mission_topic('lidar/scan',ns),lambda m:self.count('scan'),SENSOR_QOS)
        self.create_timer(1.,lambda:self.log.write('counts',**self.counts))
    def count(self,name):self.counts[name]=self.counts.get(name,0)+1
    def status(self,m):
        self.count('status');state=(m.nav_state,m.arming_state,m.failsafe,m.pre_flight_checks_pass)
        if state!=self.previous_status:
            self.log.write('status',timestamp=int(m.timestamp),nav_state=int(m.nav_state),arming_state=int(m.arming_state),
                           failsafe=bool(m.failsafe),preflight=bool(m.pre_flight_checks_pass));self.previous_status=state
    def pose(self,m):
        self.count('position')
        if m.timestamp-self.last_pose_log>100000:
            self.log.write('position',timestamp=int(m.timestamp),ned=[float(m.x),float(m.y),float(m.z)],
                           valid=bool(m.xy_valid and m.z_valid),xy_valid=bool(m.xy_valid),
                           z_valid=bool(m.z_valid));self.last_pose_log=m.timestamp
    def land(self,m):
        self.count('land');self.log.write('land',landed=bool(m.landed),ground_contact=bool(m.ground_contact))
    def ack(self,m):self.log.write('ack',command=int(m.command),result=int(m.result),timestamp=int(m.timestamp))
    def gps(self,m):
        self.count('gps');state=(int(m.fix_type),bool(m.vel_ned_valid),int(m.satellites_used))
        if state!=self.previous_gps:
            self.log.write('gps',fix_type=state[0],velocity_valid=state[1],satellites=state[2],timestamp=int(m.timestamp))
            self.previous_gps=state
    def failsafe(self,m):
        self.count('failsafe_flags')
        if m.offboard_control_signal_lost:self.log.write('offboard_signal_lost',timestamp=int(m.timestamp))
    def decision(self,m):
        self.count('decision')
        if m.sequence%5==0 or m.mode in ('LAND','COMPLETE'):self.log.write('decision',sequence=int(m.sequence),mode=m.mode,reason=m.reason,
                                         position=[float(v) for v in m.position_enu],waypoint=int(m.waypoint),camera_age=float(m.camera_age))

def main():
    p=argparse.ArgumentParser();p.add_argument('--log',required=True);p.add_argument('--ns',default='');args=p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init();node=Observer(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
