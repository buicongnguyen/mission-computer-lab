#!/usr/bin/env python3
"""SITL-only ROS 2 adapter around the tested C++ supervisor; never use on hardware."""
import argparse
from collections import deque
import math
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import (VehicleLocalPosition,VehicleStatus,SensorCombined,SensorGps,
                          OffboardControlMode,TrajectorySetpoint,VehicleCommand)
from mission_interfaces.msg import Perception,Decision
from common import ROOT,SENSOR_QOS,JsonLog,mission_topic,px4_topic,stamp_seconds,ros_seconds
from world import astar,occupancy,replan,route_blocked,GOAL
from supervisor_client import exchange

class Mission(Node):
    def __init__(self,args):
        # Fleet vehicles share the ROS graph, so each adapter lives under its PX4 namespace.
        self.ns=getattr(args,'ns','') or ''
        super().__init__('mission_supervisor_adapter'+(f'_{self.ns}' if self.ns else ''))
        # World-frame placement: the vehicle's spawn point (its PX4 local origin), goal and cruise altitude.
        self.spawn=list(getattr(args,'spawn',None) or [0.,0.,0.]);self.goal=tuple(getattr(args,'goal',None) or GOAL)
        self.altitude=getattr(args,'altitude',None) or 3.
        # PX4 instance N has MAV_SYS_ID N+1 and ignores commands addressed to any other system.
        self.system_id=getattr(args,'system_id',None) or 1
        # Freshness runs on one monotonic clock: simulation time when launched with use_sim_time (SITL,
        # where PX4 also runs on simulation time), otherwise the monotonic wall clock (real hardware).
        self.sim_time=self.get_parameter('use_sim_time').value
        self.log=JsonLog(args.log);self.start=0. if self.sim_time else time.monotonic();self.sequence=0
        self.pose=None;self.status=None;self.origin=None;self.scan=None;self.perception=None
        self.last={'pose':0.,'imu':0.,'gps':0.,'status':0.};self.last_px4_stamp={}
        self.targets=None;self.waypoint=0;self.stream_since=None;self.last_request=-1e9
        self.land_requested=False;self.last_mode=None;self.flight_seen=False;self.offboard_seen=False;self.handed_off=False
        self.last_tick=-math.inf
        self.payload_stamps={};self.payload_times={}
        self.blocked=set();self.mapped_scan=None;self.track=deque(maxlen=40);self.model_sha256=args.model_sha256
        self.core=subprocess.Popen([str(ROOT/'build/mission_supervisor')],stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        for name,typ,key in [('vehicle_local_position',VehicleLocalPosition,'pose'),
                             ('vehicle_status',VehicleStatus,'status'),('sensor_combined',SensorCombined,'imu'),
                             ('vehicle_gps_position',SensorGps,'gps')]:
            self.create_subscription(typ,px4_topic(name,typ,ns=self.ns),lambda msg,k=key:self.telemetry(k,msg),SENSOR_QOS)
        self.create_subscription(LaserScan,mission_topic('lidar/scan',self.ns),self.lidar,SENSOR_QOS)
        self.create_subscription(Perception,mission_topic('perception',self.ns),self.vision,SENSOR_QOS)
        self.control=self.create_publisher(OffboardControlMode,px4_topic('offboard_control_mode',OffboardControlMode,'in',self.ns),10)
        self.setpoint=self.create_publisher(TrajectorySetpoint,px4_topic('trajectory_setpoint',TrajectorySetpoint,'in',self.ns),10)
        self.command=self.create_publisher(VehicleCommand,px4_topic('vehicle_command',VehicleCommand,'in',self.ns),10)
        self.decisions=self.create_publisher(Decision,mission_topic('decision',self.ns),10)
        self.create_timer(0.05,self.tick)
    def telemetry(self,key,msg):
        # Receipt freshness is accepted only when the firmware's source timestamp advances.
        if msg.timestamp<=self.last_px4_stamp.get(key,-1):return
        self.last_px4_stamp[key]=msg.timestamp
        # Loss of fix may keep producing fresh packets. Those packets must not
        # count as fresh navigation data.
        if key=='gps' and (msg.fix_type<3 or not msg.vel_ned_valid):return
        self.last[key]=self.now()
        if key=='pose':self.pose=msg
        elif key=='status':self.status=msg
    def payload_time(self,key,msg):
        stamp=stamp_seconds(msg.header.stamp);wall_now=ros_seconds(self)
        # Reject future/replayed data; never clamp a future timestamp to "fresh".
        if not math.isfinite(stamp) or stamp<=0 or stamp>wall_now or stamp<=self.payload_stamps.get(key,0.):
            return False
        elapsed=self.now()
        capture=elapsed-(wall_now-stamp)
        if capture<0:return False
        self.payload_stamps[key]=stamp;self.payload_times[key]=capture
        return True
    def lidar(self,msg):
        if not (0<len(msg.ranges)<=10000 and all(math.isfinite(v) for v in
                (msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)) and
                msg.angle_increment>0 and 0<=msg.range_min<msg.range_max):return
        if any(math.isnan(r) or r<msg.range_min for r in msg.ranges):return
        if self.payload_time('lidar',msg):self.scan=msg
    def vision(self,msg):
        if not math.isfinite(msg.inference_ms) or msg.inference_ms<0:return
        # Only results from the model the boot gate verified may count as fresh vision.
        if self.model_sha256 and msg.model_sha256!=self.model_sha256:return
        if self.payload_time('camera',msg):self.perception=msg
    def scan_points(self,position):
        s=self.scan;points=[]
        for i,r in enumerate(s.ranges):
            if math.isfinite(r) and s.range_min<=r<s.range_max:
                angle=s.angle_min+i*s.angle_increment
                points.append({'x':position[0]+r*math.cos(angle),'y':position[1]+r*math.sin(angle),'hit':True})
        return points
    def position_at(self,when):
        # Place a scan where the vehicle was when it was captured, not where it is now.
        return min(self.track,key=lambda item:abs(item[0]-when))[1] if self.track else None
    def map_scan(self,position):
        """Add the latest scan to the map and replan if it closes the rest of the route; False if no route remains."""
        self.mapped_scan=self.scan
        before=len(self.blocked)
        occupancy(self.scan_points(self.position_at(self.payload_times['lidar']) or position),blocked=self.blocked)
        ahead=self.targets[max(self.waypoint,1):]
        if len(self.blocked)==before or self.waypoint<1 or not ahead or not route_blocked(ahead,self.blocked):return True
        route=replan(position,self.goal,self.blocked)
        self.log.write('replan',path=route,position=position)
        if route:self.targets=self.targets[:self.waypoint]+[[float(x),float(y),self.altitude] for x,y in route]
        return bool(route)
    def send_command(self,command,p1=0.,p2=0.):
        m=VehicleCommand();m.timestamp=self.get_clock().now().nanoseconds//1000
        m.command=command;m.param1=float(p1);m.param2=float(p2)
        m.target_system=self.system_id;m.target_component=1;m.source_system=42;m.source_component=191;m.from_external=True
        self.command.publish(m);self.log.write('command',command=int(command),param1=p1,param2=p2)
    def tick(self):
        now=self.now()
        # The supervisor rejects a sample that does not advance in time (and exits); under simulated time a
        # timer can fire twice on one clock value, so such a tick is skipped rather than sent.
        if self.sim_time:
            if now<=self.last_tick:return
            self.last_tick=now
        if self.handed_off:return
        if not(self.pose and self.status and self.scan and self.perception):return
        offboard=self.status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED:
            self.flight_seen=True;self.offboard_seen|=offboard
        if self.flight_seen and (self.status.failsafe or self.status.arming_state!=VehicleStatus.ARMING_STATE_ARMED):
            self.handed_off=True;self.log.write('handoff',reason='autopilot_failsafe_or_disarm')
            return  # Never rearm or fight an autopilot failsafe after flight has begun.
        if self.offboard_seen and not offboard:
            self.handed_off=True;self.log.write('handoff',reason='external_mode_change')
            return  # A pilot or GCS left OFFBOARD without a failsafe; never switch back.
        if not(self.pose.xy_valid and self.pose.z_valid):
            if self.stream_since is not None:self.log.write('estimator_invalid');self.stream_since=None
            if self.flight_seen:self.handed_off=True
            return  # Stop offboard proof-of-life; its 2 s pre-stream restarts if the estimate recovers.
        if self.origin is None:
            self.origin=[self.pose.y,self.pose.x,self.pose.z]
            self.blocked=occupancy(self.scan_points(self.spawn[:2]),blocked=set(self.reserved_cells()));self.mapped_scan=self.scan
            path=astar((round(self.spawn[0]),round(self.spawn[1])),self.goal,self.blocked)
            if not path:raise RuntimeError('no planned path')
            self.targets=[[self.spawn[0],self.spawn[1],self.altitude]]+[[float(x),float(y),self.altitude] for x,y in path[1:]]
            self.log.write('plan',path=path,source='ROS LaserScan',origin=self.origin)
        # Until the vehicle arms, re-zero altitude on the ground so estimator drift cannot trip the
        # geofence floor (-0.1 m) before takeoff; the horizontal plan frame stays fixed.
        if not self.flight_seen:self.origin[2]=self.pose.z
        position=[self.pose.y-self.origin[0]+self.spawn[0],self.pose.x-self.origin[1]+self.spawn[1],self.origin[2]-self.pose.z+self.spawn[2]]
        self.track.append((now,position))
        if self.scan is not self.mapped_scan and not self.map_scan(position):
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND);self.land_requested=True
            self.handed_off=True;self.log.write('handoff',reason='no_safe_route')
            return  # Scans closed every route to the goal; land where the vehicle is.
        # Mapping stays in the estimate's frame (the payload scans come from the same estimate); control and
        # supervision use the navigation position, which a guardian corrects with station fixes when spoofed.
        position=self.navigation(now,position)
        if self.stream_since is None:self.stream_since=now
        target,complete=self.next_target(now,position)
        camera_stamp=self.payload_times['camera'];lidar_stamp=self.payload_times['lidar']
        vision_stamp=min(camera_stamp,lidar_stamp)
        # VehicleStatus is published at about 2 Hz, so a 0.5 s threshold on it
        # would produce false holds from ordinary scheduling jitter. Position
        # telemetry is the high-rate link heartbeat; status has a 1.5 s bound.
        link_stamp=self.last['pose'] if now-self.last['status']<=1.5 else self.last['status']
        fields=[self.sequence,now,self.last['imu'],self.last['gps'],vision_stamp,
                link_stamp,0.95,*position,*target,
                float(self.perception.inference_ms),int(complete)]
        try:mode,reason,velocity=exchange(self.core,fields,0.5)
        except RuntimeError as error:  # Keep the supervisor's own reason: it names the rejected input.
            reason=self.core.stderr.read().strip()[:500] if self.core.poll() is not None else ''
            self.log.write('supervisor_error',error=str(error),stderr=reason,fields=fields);raise
        d=Decision();d.header.stamp=self.get_clock().now().to_msg();d.header.frame_id='local_enu'
        d.sequence=self.sequence;d.mode=mode;d.reason=reason;d.position_enu=position;d.target_enu=target
        d.velocity_enu=velocity;d.camera_age=max(0.,now-camera_stamp);d.lidar_age=max(0.,now-lidar_stamp)
        d.waypoint=self.waypoint;self.decisions.publish(d)
        if mode!=self.last_mode:self.log.write('transition',mode=mode,reason=reason,uptime=now);self.last_mode=mode
        if self.sequence%5==0 or mode in ('LAND','COMPLETE'):self.log.write('decision',mode=mode,reason=reason,position=position,target=target,
                                           velocity=velocity,waypoint=self.waypoint,camera_age=d.camera_age,
                                           gps_age=now-self.last['gps'],imu_age=now-self.last['imu'])
        self.sequence+=1
        self.after_decision(now,position,mode,target)
        if mode in ('LAND','COMPLETE'):
            if not self.land_requested:
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND);self.land_requested=True
            self.handed_off=True;self.log.write('handoff',reason='terminal_'+mode.lower())
            return
        timestamp=self.get_clock().now().nanoseconds//1000
        heartbeat=OffboardControlMode();heartbeat.timestamp=timestamp;heartbeat.velocity=True
        self.control.publish(heartbeat)
        command=TrajectorySetpoint();command.timestamp=timestamp;command.position=[math.nan]*3
        command.velocity=[velocity[1],velocity[0],-velocity[2]];command.acceleration=[math.nan]*3
        command.jerk=[math.nan]*3;command.yaw=0.;command.yawspeed=math.nan;self.setpoint.publish(command)
        if (mode=='ACTIVE' and self.status.pre_flight_checks_pass and
                now-self.stream_since>2 and now-self.last_request>2 and self.may_request_flight()):
            if self.status.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1,6)
            if self.status.arming_state!=VehicleStatus.ARMING_STATE_ARMED:
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1)
            self.last_request=now
    def now(self):
        return (self.get_clock().now().nanoseconds/1e9 if self.sim_time else time.monotonic())-self.start
    def next_target(self,now,position):
        """Current setpoint and whether the mission is complete; fleet vehicles override this."""
        target=self.targets[min(self.waypoint,len(self.targets)-1)]
        if self.waypoint<len(self.targets) and self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED and math.dist(position,target)<0.35:
            self.waypoint+=1;target=self.targets[min(self.waypoint,len(self.targets)-1)]
        return target,self.waypoint>=len(self.targets)
    def navigation(self,now,position):return position  # A guardian switches to station fixes when spoofed.
    def reserved_cells(self):return ()  # Cells the plan must avoid besides obstacles (a guardian: other posts).
    def may_request_flight(self):return True  # A fleet vehicle waits for its launch slot.
    def after_decision(self,now,position,mode,target):pass  # A fleet vehicle reports its state.
    def close(self):
        if self.core.poll() is None:
            self.core.stdin.close()
            try:self.core.wait(timeout=2)
            except subprocess.TimeoutExpired:self.core.kill();self.core.wait()
        self.log.close()

def main():
    p=argparse.ArgumentParser();p.add_argument('--allow-sitl',action='store_true');p.add_argument('--log',required=True)
    p.add_argument('--model-sha256',help='Accept perception only from this verified model')
    args=p.parse_args(remove_ros_args(sys.argv)[1:])
    if not args.allow_sitl:p.error('This adapter is simulation-only. Launch through scripts/run_sitl.sh.')
    rclpy.init();node=Mission(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
