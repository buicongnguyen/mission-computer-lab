#!/usr/bin/env python3
"""SITL-only command and control (the center): hears station reports after a link delay and answers after a
simulated operator decision time. It acknowledges posture changes and decides recovery after an event,
which the station is not delegated to decide while the center can be reached."""
import argparse
import json
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from common import JsonLog
from guardian import Center
from guardian_layout import CENTER

class CenterNode(Node):
    def __init__(self,args):
        super().__init__('guardian_center');self.log=JsonLog(args.log)
        # With --unreachable nothing crosses the station-center link, in either direction.
        self.reachable=not args.unreachable
        self.center=Center(CENTER['latency'],CENTER['decision_time'],reachable=lambda now:self.reachable,after_red='recover')
        if not self.reachable:self.log.write('unreachable')
        self.create_subscription(String,'/center/report',self.report,10)
        self.decisions=self.create_publisher(String,'/center/decision',10);self.pending=[]
        self.create_timer(0.1,self.tick)
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def report(self,msg):
        try:report=json.loads(msg.data)
        except ValueError:return
        self.center.send(self.now(),report)
        if self.reachable:self.log.write('report',t=self.now(),report=report.get('kind'),state=report.get('state'),request=report.get('request'))
    def tick(self):
        now=self.now()
        for arrival,decision,request in self.center.step(now):
            self.pending.append((arrival,decision,request))
            self.log.write('decision',t=now,decision=decision,request=request,delivered_at=arrival)
        for item in [p for p in self.pending if p[0]<=now]:
            # The decision names the request it answers, so the station can ignore an answer to an old question.
            self.pending.remove(item);message=String();message.data=json.dumps({'decision':item[1],'request':item[2],'t':now})
            self.decisions.publish(message)

def main():
    p=argparse.ArgumentParser();p.add_argument('--log',required=True)
    p.add_argument('--unreachable',action='store_true',help='Simulate a station-center link that is down throughout')
    args=p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init();node=CenterNode(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
