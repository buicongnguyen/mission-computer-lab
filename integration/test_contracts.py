"""Regression checks for failures found at the real ROS/PX4 boundary."""
from collections import deque
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace,MethodType
import unittest
from unittest.mock import patch,Mock
from builtin_interfaces.msg import Time
from mission_node import Mission
from observer_node import Observer
from common import JsonLog,px4_topic,SENSOR_QOS
from mission_interfaces.msg import Decision
from px4_msgs.msg import VehicleStatus,VehicleLocalPosition
from rclpy.qos import ReliabilityPolicy

class TelemetryContracts(unittest.TestCase):
    def setUp(self):
        self.adapter=SimpleNamespace(last_px4_stamp={},last={'gps':0.,'pose':0.},start=100.)
    def gps(self,timestamp,fix=3,velocity=True,now=105.):
        message=SimpleNamespace(timestamp=timestamp,fix_type=fix,vel_ned_valid=velocity)
        with patch('mission_node.time.monotonic',return_value=now):
            Mission.telemetry(self.adapter,'gps',message)
    def test_fresh_packet_with_no_fix_does_not_refresh_navigation(self):
        self.gps(1);self.gps(2,fix=0,now=106.)
        self.assertEqual(self.adapter.last['gps'],5.)
        self.gps(3,now=107.);self.assertEqual(self.adapter.last['gps'],7.)
    def test_velocity_invalid_is_not_healthy_gps(self):
        self.gps(1,velocity=False)
        self.assertEqual(self.adapter.last['gps'],0.)
    def test_repeated_or_old_timestamp_does_not_refresh_health(self):
        self.gps(10);self.gps(10,now=110.);self.gps(9,now=111.)
        self.assertEqual(self.adapter.last['gps'],5.)
    def test_actual_firmware_topic_versions_and_sensor_qos(self):
        self.assertEqual(px4_topic('vehicle_status',VehicleStatus),'/fmu/out/vehicle_status_v1')
        self.assertEqual(px4_topic('vehicle_local_position',VehicleLocalPosition),'/fmu/out/vehicle_local_position')
        self.assertEqual(SENSOR_QOS.reliability,ReliabilityPolicy.BEST_EFFORT)
    def test_ros_fixed_array_is_serializable_by_observer(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'evidence.jsonl';log=JsonLog(path)
            observer=SimpleNamespace(log=log,count=lambda name:None)
            decision=Decision();decision.position_enu=[1.,2.,3.];decision.mode='ACTIVE'
            Observer.decision(observer,decision);log.close()
            self.assertEqual(json.loads(path.read_text())['position'],[1.,2.,3.])

class MissionBoundaries(unittest.TestCase):
    def adapter(self):
        a=SimpleNamespace(start=100.,payload_stamps={},payload_times={},handed_off=False,
            flight_seen=False,offboard_seen=False,pose=SimpleNamespace(x=0.,y=0.,z=0.,xy_valid=True,z_valid=True),
            status=SimpleNamespace(arming_state=1,nav_state=0,failsafe=False,pre_flight_checks_pass=True),
            scan=object(),perception=SimpleNamespace(inference_ms=1.),origin=[0.,0.,0.],
            targets=[[1.,0.,3.]],waypoint=0,stream_since=0.,last_request=-100.,
            last={'imu':5.,'gps':5.,'pose':5.,'status':5.},sequence=1,core=Mock(),
            last_mode=None,log=Mock(),land_requested=False,send_command=Mock(),
            decisions=Mock(),control=Mock(),setpoint=Mock(),get_clock=Mock(),blocked=set(),track=deque(maxlen=40),
            model_sha256=None)
        a.mapped_scan=a.scan  # No new scan unless a test supplies one.
        for method in ('payload_time','scan_points','position_at','map_scan'):
            setattr(a,method,MethodType(getattr(Mission,method),a))
        a.get_clock.return_value.now.return_value.to_msg.return_value=Time(sec=105)
        a.get_clock.return_value.now.return_value.nanoseconds=105000000000
        return a
    def test_future_and_replayed_payloads_do_not_refresh(self):
        a=self.adapter()
        msg=SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=201)),inference_ms=1.)
        with patch('mission_node.ros_seconds',return_value=200.),patch('mission_node.time.monotonic',return_value=105.):
            self.assertFalse(a.payload_time('camera',msg))
            msg.header.stamp=Time(sec=199,nanosec=900000000)
            self.assertTrue(a.payload_time('camera',msg))
            accepted=a.payload_times['camera']
            self.assertFalse(a.payload_time('camera',msg))
            self.assertEqual(a.payload_times['camera'],accepted)
        # Receipt time is monotonic; changing the ROS wall clock does not renew old data.
        with patch('mission_node.ros_seconds',return_value=190.),patch('mission_node.time.monotonic',return_value=106.):
            self.assertFalse(a.payload_time('camera',msg))
            self.assertEqual(a.payload_times['camera'],accepted)
    def tick(self,a,mode):
        a.payload_times={'camera':5.,'lidar':5.}
        with patch('mission_node.time.monotonic',return_value=105.),patch('mission_node.exchange',return_value=(mode,'test',[0.,0.,0.])) as core:
            Mission.tick(a)
            return core.call_count
    def test_no_arm_or_mode_request_during_hold_or_failed_preflight(self):
        for mode,ready in [('INIT',True),('HOLD',True),('ACTIVE',False)]:
            a=self.adapter();a.status.pre_flight_checks_pass=ready
            self.tick(a,mode);a.send_command.assert_not_called()
        a=self.adapter();self.tick(a,'ACTIVE');self.assertEqual(a.send_command.call_count,2)
    def test_complete_and_land_permanently_handoff_and_log_terminal_sample(self):
        for mode in ('COMPLETE','LAND'):
            a=self.adapter();self.tick(a,mode)
            self.assertTrue(a.handed_off)
            self.assertEqual(a.send_command.call_count,1)
            self.assertTrue(any(c.args[0]=='decision' for c in a.log.write.call_args_list))
            self.assertEqual(self.tick(a,'ACTIVE'),0)
            a.control.publish.assert_not_called()
    def test_autopilot_failsafe_cannot_resume_mission_after_flag_clears(self):
        a=self.adapter();a.flight_seen=True;a.status.arming_state=2;a.status.failsafe=True
        self.assertEqual(self.tick(a,'ACTIVE'),0)
        a.status.failsafe=False
        self.assertEqual(self.tick(a,'ACTIVE'),0);a.send_command.assert_not_called()
    def test_operator_mode_change_after_offboard_is_never_reverted(self):
        a=self.adapter();a.status.arming_state=2;a.status.nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        self.tick(a,'ACTIVE')  # Armed before OFFBOARD is part of normal entry; request it once.
        self.assertFalse(a.handed_off);self.assertEqual(a.send_command.call_count,1)
        a=self.adapter();a.status.arming_state=2;a.status.nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.tick(a,'ACTIVE');self.assertTrue(a.offboard_seen);a.send_command.assert_not_called()
        a.status.nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL
        self.assertEqual(self.tick(a,'ACTIVE'),0);self.assertTrue(a.handed_off)
        a.status.nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.assertEqual(self.tick(a,'ACTIVE'),0);a.send_command.assert_not_called()
        self.assertEqual(a.control.publish.call_count,1)
    def test_preflight_estimator_loss_restarts_offboard_proof_of_life(self):
        a=self.adapter();a.pose.xy_valid=False
        for _ in range(3):self.assertEqual(self.tick(a,'ACTIVE'),0)
        self.assertIsNone(a.stream_since);self.assertFalse(a.handed_off)
        self.assertEqual([c.args[0] for c in a.log.write.call_args_list].count('estimator_invalid'),1)
        a.pose.xy_valid=True;self.tick(a,'ACTIVE')
        a.send_command.assert_not_called();a.control.publish.assert_called_once()
    @staticmethod
    def scan_at(distance,angle):
        return SimpleNamespace(ranges=[distance],angle_min=angle,angle_increment=0.1,range_min=0.05,range_max=14.)
    def test_new_scan_that_closes_the_route_replans_around_it(self):
        a=self.adapter();a.targets=[[0.,0.,3.],[1.,0.,3.],[2.,0.,3.],[3.,0.,3.]];a.waypoint=1
        a.payload_times={'lidar':5.};a.track.append((5.,[1.,0.,3.]))
        a.scan=self.scan_at(1.,0.)  # A return at (2,0), one cell ahead of the vehicle.
        self.assertTrue(a.map_scan([1.,0.,3.]))
        self.assertIn((2,0),a.blocked)
        self.assertFalse(any((round(x),round(y)) in a.blocked for x,y,_ in a.targets[1:]))
        self.assertEqual(a.targets[-1][:2],[9.,9.])
        self.assertTrue(any(c.args[0]=='replan' for c in a.log.write.call_args_list))
    def test_scan_is_mapped_where_it_was_captured(self):
        a=self.adapter();a.track.extend([(4.9,[0.,0.,3.]),(5.4,[3.,0.,3.])]);a.payload_times={'lidar':4.95}
        a.scan=self.scan_at(2.,0.)
        a.map_scan([3.,0.,3.])
        self.assertIn((2,0),a.blocked);self.assertNotIn((5,0),a.blocked)
    def test_no_remaining_route_lands_and_hands_off(self):
        a=self.adapter();a.targets=[[0.,0.,3.],[1.,0.,3.]];a.waypoint=1;a.blocked={(9,9)}
        a.scan=self.scan_at(0.5,0.)  # Closes the route ahead while the goal is already mapped as blocked.
        self.assertEqual(self.tick(a,'ACTIVE'),0)
        self.assertTrue(a.handed_off);a.send_command.assert_called_once()
        self.assertTrue(any(c.kwargs.get('reason')=='no_safe_route' for c in a.log.write.call_args_list))
    def test_perception_from_an_unverified_model_is_not_fresh(self):
        a=self.adapter();a.model_sha256='a'*64
        msg=SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=199)),inference_ms=1.,model_sha256='b'*64)
        with patch('mission_node.ros_seconds',return_value=200.),patch('mission_node.time.monotonic',return_value=105.):
            Mission.vision(a,msg);self.assertNotIn('camera',a.payload_times)
            msg.model_sha256='a'*64;Mission.vision(a,msg);self.assertIn('camera',a.payload_times)
    def test_ground_altitude_drift_before_arming_is_rezeroed(self):
        a=self.adapter();a.pose.z=0.2  # NED: the estimate has drifted 0.2 m below the first sample.
        a.payload_times={'camera':5.,'lidar':5.}
        with patch('mission_node.time.monotonic',return_value=105.),patch('mission_node.exchange',return_value=('INIT','test',[0.,0.,0.])) as core:
            Mission.tick(a)
        self.assertEqual(core.call_args.args[1][9],0.)  # Relative altitude sent to the supervisor.
    def test_terminal_observer_sample_is_not_lost_to_decimation(self):
        observer=SimpleNamespace(log=Mock(),count=lambda name:None)
        msg=Decision();msg.sequence=1;msg.mode='COMPLETE'
        Observer.decision(observer,msg);observer.log.write.assert_called_once()

if __name__=='__main__':unittest.main()
