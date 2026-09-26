"""Clocked contracts for the station/carrier streams (ROS-independent for tests)."""
import math

STREAM_TIMEOUT = 1.0
LAND_TIMEOUT = 2.0


class FreshInput:
    def __init__(self, timeout=STREAM_TIMEOUT):
        self.timeout = timeout
        self.stamp = None

    def accept(self, stamp, now):
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or not 0 <= now - stamp <= self.timeout
                or (self.stamp is not None and stamp <= self.stamp)):
            return False
        self.stamp = stamp
        return True

    def fresh(self, now):
        return self.stamp is not None and 0 <= now - self.stamp <= self.timeout
