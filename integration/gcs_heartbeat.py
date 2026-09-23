#!/usr/bin/env python3
"""Local SITL GCS heartbeat; does not arm or send navigation commands."""
import time
from pymavlink import mavutil

link=mavutil.mavlink_connection('udpin:127.0.0.1:14550',source_system=255,source_component=190)
last=0.
try:
    while True:
        message=link.recv_match(blocking=True,timeout=0.1)
        if message and message.get_type()=='HEARTBEAT':
            print(f'PX4 heartbeat system={message.get_srcSystem()} mode={message.custom_mode}',flush=True)
        if time.monotonic()-last>=1:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            last=time.monotonic()
except KeyboardInterrupt:
    pass
finally:
    link.close()
