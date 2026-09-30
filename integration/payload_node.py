#!/usr/bin/env python3
"""Publish synthetic payload sensors as real typed ROS 2 messages at 10 Hz."""

import argparse
import math
import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from px4_msgs.msg import VehicleLocalPosition
from common import SENSOR_QOS, mission_topic, px4_topic, ros_seconds
from perception import camera_frame
from world import lidar


class Payload(Node):
    def __init__(self, args):
        ns = args.ns
        super().__init__('virtual_payload' + (f'_{ns}' if ns else ''))
        # PX4 local position is relative to the vehicle's spawn point; the ideal LiDAR works in world ENU.
        self.start = ros_seconds(self)
        self.args = args
        self.position = None
        self.truth = None
        self.drop_until = -1.0
        # The runner sends SIGUSR1 once the vehicle is airborne, so the dropout is always tested in flight.
        signal.signal(signal.SIGUSR1, self.drop_camera)
        self.create_subscription(
            VehicleLocalPosition,
            px4_topic('vehicle_local_position', VehicleLocalPosition, ns=ns),
            self.pose,
            SENSOR_QOS,
        )
        if args.truth:
            # A real LiDAR measures the real surroundings whatever the GNSS says. Computed from a GNSS-dragged
            # estimate, the ideal scan of a guardian holding its post on station fixes grazed a cylinder: ranges fell
            # below the minimum, the adapter rejected the scans as stale vision, and the supervisor landed it.
            self.create_subscription(Odometry, args.truth, self.on_truth, 10)
        self.images = self.create_publisher(Image, mission_topic('camera/image', ns), SENSOR_QOS)
        self.scans = self.create_publisher(LaserScan, mission_topic('lidar/scan', ns), SENSOR_QOS)
        self.create_timer(0.1, self.tick)

    def drop_camera(self, *_):
        self.drop_until = ros_seconds(self) - self.start + self.args.camera_drop_for
        print(f'camera dropout for {self.args.camera_drop_for} s', flush=True)

    def pose(self, msg):
        e, n, u = self.args.spawn
        self.position = [msg.y + e, msg.x + n, -msg.z + u]

    def on_truth(self, msg):
        p = msg.pose.pose.position
        self.truth = [p.x, p.y, p.z]  # Gazebo's world frame is ENU, as the scan is.

    def tick(self):
        now = ros_seconds(self)
        elapsed = now - self.start
        stamp = self.get_clock().now().to_msg()
        if elapsed >= self.drop_until:
            rgb = (camera_frame(elapsed)[0].transpose(1, 2, 0) * 255).astype('uint8')
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = 'synthetic_camera_optical'
            image.height = 48
            image.width = 64
            image.encoding = 'rgb8'
            image.step = 64 * 3
            image.data = rgb.tobytes()
            self.images.publish(image)
        position = self.truth or self.position
        if position is None:
            return  # No scan until the vehicle's position is known.
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'synthetic_lidar_enu'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi * (71 / 72)
        scan.angle_increment = 2 * math.pi / 72
        scan.range_min = 0.05
        scan.range_max = 14.0
        scan.scan_time = 0.1
        scan.ranges = [float(p['range']) for p in lidar(position)]
        self.scans.publish(scan)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--camera-drop-for', type=float, default=0.8)
    p.add_argument('--ns', default='')
    p.add_argument('--spawn', type=float, nargs=3, default=[0.0, 0.0, 0.0])
    p.add_argument('--truth', help='Gazebo odometry topic of this airframe: the scan is computed from its true pose')
    args = p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Payload(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
