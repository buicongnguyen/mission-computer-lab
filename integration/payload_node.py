#!/usr/bin/env python3
"""Publish synthetic payload sensors as real typed ROS 2 messages at 10 Hz."""
import argparse
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from px4_msgs.msg import VehicleLocalPosition
from common import SENSOR_QOS, px4_topic, ros_seconds
from perception import camera_frame
from world import lidar

class Payload(Node):
    def __init__(self,args):
        super().__init__('virtual_payload')
        self.start=ros_seconds(self); self.args=args; self.position=[0.,0.,0.]
        self.create_subscription(VehicleLocalPosition,px4_topic('vehicle_local_position',VehicleLocalPosition),self.pose,SENSOR_QOS)
        self.images=self.create_publisher(Image,'/mission/camera/image',SENSOR_QOS)
        self.scans=self.create_publisher(LaserScan,'/mission/lidar/scan',SENSOR_QOS)
        self.create_timer(0.1,self.tick)
    def pose(self,msg): self.position=[msg.y,msg.x,-msg.z]
    def tick(self):
        now=ros_seconds(self); elapsed=now-self.start
        stamp=self.get_clock().now().to_msg()
        if not (self.args.camera_drop_at<=elapsed<self.args.camera_drop_at+self.args.camera_drop_for):
            rgb=(camera_frame(elapsed)[0].transpose(1,2,0)*255).astype('uint8')
            image=Image(); image.header.stamp=stamp; image.header.frame_id='synthetic_camera_optical'
            image.height=48; image.width=64; image.encoding='rgb8'; image.step=64*3
            image.data=rgb.tobytes(); self.images.publish(image)
        scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='synthetic_lidar_enu'
        scan.angle_min=0.; scan.angle_max=2*math.pi*(71/72); scan.angle_increment=2*math.pi/72
        scan.range_min=0.05; scan.range_max=14.; scan.scan_time=0.1
        scan.ranges=[float(p['range']) for p in lidar(self.position)]
        self.scans.publish(scan)

def main():
    p=argparse.ArgumentParser();p.add_argument('--camera-drop-at',type=float,default=1e9)
    p.add_argument('--camera-drop-for',type=float,default=0.8);args=p.parse_args()
    rclpy.init();node=Payload(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
