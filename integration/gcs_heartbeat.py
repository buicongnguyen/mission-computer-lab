#!/usr/bin/env python3
"""Local SITL GCS heartbeat; does not arm or send navigation commands.

Default: answer the single PX4 instance that talks to UDP 14550. With --ports, send a heartbeat to
each listed PX4 instance port (18570 + instance) so every vehicle of a fleet sees a connected GCS;
answering on 14550 would reach only whichever instance spoke last.
"""
import argparse
import time
from pymavlink import mavutil

parser=argparse.ArgumentParser();parser.add_argument('--ports',help='Comma-separated PX4 MAVLink ports, e.g. 18570,18571')
args=parser.parse_args()
if args.ports:
    links=[mavutil.mavlink_connection(f'udpout:127.0.0.1:{int(port)}',source_system=255,source_component=190)
           for port in args.ports.split(',')]
else:
    links=[mavutil.mavlink_connection('udpin:127.0.0.1:14550',source_system=255,source_component=190)]
last=0.
try:
    while True:
        for link in links:
            message=link.recv_match(blocking=False)
            if message and message.get_type()=='HEARTBEAT':
                print(f'PX4 heartbeat system={message.get_srcSystem()} mode={message.custom_mode}',flush=True)
        if time.monotonic()-last>=1:
            for link in links:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            last=time.monotonic()
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    for link in links:link.close()
