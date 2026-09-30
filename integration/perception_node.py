#!/usr/bin/env python3
"""ROS image subscriber -> actual ONNX CPU invocation -> typed perception message."""

import argparse
import hashlib
from pathlib import Path
import numpy as np
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from mission_interfaces.msg import Perception
from cryptography.exceptions import InvalidSignature
from common import SENSOR_QOS, JsonLog, mission_topic
from perception import Detector
from security import boot_gate, load_public_key


class PerceptionNode(Node):
    def __init__(self, args):
        super().__init__('onnx_perception' + (f'_{args.ns}' if args.ns else ''))
        self.log = JsonLog(args.log)
        try:
            payload, stages = boot_gate(Path(args.model), load_public_key(args.public_key))
        except (OSError, ValueError, InvalidSignature) as error:
            # Refuse to publish perception from an unverified model; the runner treats exit as failure.
            self.log.write('boot_rejected', reason=str(error) or type(error).__name__)
            self.log.close()
            raise SystemExit(3) from error
        self.hash = hashlib.sha256(payload).hexdigest()
        self.log.write('boot', stages=stages, model_sha256=self.hash)
        self.detector = Detector(payload)
        self.seq = 0
        self.pub = self.create_publisher(Perception, mission_topic('perception', args.ns), SENSOR_QOS)
        self.create_subscription(Image, mission_topic('camera/image', args.ns), self.frame, SENSOR_QOS)

    def frame(self, msg):
        if msg.encoding != 'rgb8' or (msg.height, msg.width, msg.step) != (48, 64, 192):
            self.log.write('rejected_image', reason='unexpected_layout')
            return
        if len(msg.data) != 48 * 192:
            self.log.write('rejected_image', reason='truncated')
            return
        frame = (
            np.frombuffer(bytes(msg.data), dtype=np.uint8)
            .reshape(48, 64, 3)
            .transpose(2, 0, 1)[None]
            .astype(np.float32)
            / 255
        )
        result = self.detector.infer(frame)
        output = Perception()
        output.header = msg.header
        output.sequence = self.seq
        self.seq += 1
        output.detected = result['bbox'] is not None
        output.bbox = result['bbox'] or [0, 0, 0, 0]
        output.inference_ms = float(result['inference_ms'])
        output.model_sha256 = self.hash
        output.execution_provider = 'CPUExecutionProvider'
        self.pub.publish(output)
        if self.seq % 10 == 0:
            self.log.write('inference', sequence=self.seq, **result)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model', required=True)
    p.add_argument('--public-key', required=True)
    p.add_argument('--log', required=True)
    p.add_argument('--ns', default='')
    args = p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = PerceptionNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
