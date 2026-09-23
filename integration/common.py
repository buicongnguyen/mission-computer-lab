"""Shared ROS contracts; imports only after the ROS workspace is sourced."""
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

SENSOR_QOS=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)

def px4_topic(name,message_type,direction='out'):
    version=getattr(message_type,'MESSAGE_VERSION',0)
    return f'/fmu/{direction}/{name}'+(f'_v{version}' if version else '')

def stamp_seconds(stamp): return stamp.sec+stamp.nanosec/1e9

def ros_seconds(node): return node.get_clock().now().nanoseconds/1e9

class JsonLog:
    def __init__(self,path):
        Path(path).parent.mkdir(parents=True,exist_ok=True)
        self.file=Path(path).open('w',encoding='utf-8')
    def write(self,kind,**fields):
        self.file.write(json.dumps({'wall_time':time.time(),'kind':kind,**fields},allow_nan=False)+'\n')
        self.file.flush()
    def close(self): self.file.close()
