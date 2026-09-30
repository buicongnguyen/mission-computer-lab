"""Shared ROS contracts; imports only after the ROS workspace is sourced."""

import functools
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


def px4_topic(name, message_type, direction='out', ns=''):
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    return (f'/{ns}' if ns else '') + f'/fmu/{direction}/{name}' + (f'_v{version}' if version else '')


def mission_topic(name, ns=''):
    # Each fleet vehicle has its own payload and decision topics under its PX4 namespace.
    return (f'/{ns}' if ns else '') + f'/mission/{name}'


# What a malformed JSON message raises when a callback reads it: bad JSON, a missing key, a wrong type or length.
MALFORMED = (ValueError, KeyError, TypeError, IndexError, AttributeError)


def drop_malformed(topic):
    """Decorator for a callback that reads a JSON message. A malformed message is logged as dropped and the
    callback returns False, instead of raising: an exception in a callback ends rclpy.spin and takes the node
    down, and a dead guardian adapter hands its vehicle to a PX4 failsafe. The node keeps its last good state,
    so its freshness gates see the missing update as data going stale. The runner fails any flight whose logs
    contain a dropped message, so a real fault here is still reported."""

    def wrap(callback):
        @functools.wraps(callback)
        def guarded(self, *args):
            try:
                return callback(self, *args)
            except MALFORMED as error:
                self.log.write('dropped_message', topic=topic, error=f'{type(error).__name__}: {error}'[:200])
                return False

        return guarded

    return wrap


def finite_number(value):
    """A finite number from a JSON message, or ValueError (booleans are not numbers here)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'expected a finite number, got {value!r}'[:80])
    return value


def finite_vector(value, length=3):
    """A list of `length` finite numbers from a JSON message, or ValueError."""
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'expected {length} numbers, got {value!r}'[:80])
    for item in value:
        finite_number(item)
    return value


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def ros_seconds(node):
    return node.get_clock().now().nanoseconds / 1e9


class JsonLog:
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.file = Path(path).open('w', encoding='utf-8')

    def write(self, kind, **fields):
        self.file.write(json.dumps({'wall_time': time.time(), 'kind': kind, **fields}, allow_nan=False) + '\n')
        self.file.flush()

    def close(self):
        self.file.close()
