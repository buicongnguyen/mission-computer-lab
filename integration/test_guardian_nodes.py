"""Guardian SITL nodes run as real ROS nodes without Gazebo or PX4: logging, fusion, posture, orders and relay."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace,MethodType
import unittest
from unittest.mock import Mock
import rclpy
from std_msgs.msg import String
from center_node import CenterNode
from guardian import Tracker,authorised
from guardian_layout import CFG,GUARDIANS,INTRUDER,RELOCATE
from guardian_mission_node import GuardianMission
from guardian_station_node import GuardianStation
from guardian_world_node import World

def message(data):
    m=String();m.data=json.dumps(data);return m

def intruder_at(t):
    s,a=INTRUDER['start'],INTRUDER['aim'];d=[a[0]-s[0],a[1]-s[1]];n=math.hypot(*d)
    return [s[0]+d[0]/n*INTRUDER['speed']*t,s[1]+d[1]/n*INTRUDER['speed']*t,s[2]]

class GuardianNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):self.folder=tempfile.TemporaryDirectory();self.nodes=[]
    def tearDown(self):
        for node in self.nodes:node.log.close();node.destroy_node()
        self.folder.cleanup()
    def path(self,name):return str(Path(self.folder.name)/name)
    def records(self,name):return [json.loads(l) for l in Path(self.path(name)).read_text().splitlines()]
    def station(self):
        s=GuardianStation(SimpleNamespace(drones=','.join(g['ns'] for g in GUARDIANS),log=self.path('station.jsonl'),speed=0.,max_east=14.))
        self.nodes.append(s);s.carrier={'e':0.,'n':-1.5,'yaw':0.,'speed':0.}
        for g in GUARDIANS:
            s.states[g['ns']]={'phase':'watch','position':[float(g['post'][0]),float(g['post'][1]),g['altitude']],
                               'armed':True,'layer':g['altitude'],'pad_error':None}
            s.flown.add(g['ns']);s.clear[g['ns']]['launch']=True
        return s
    def fly(self,s,start,until,source='px4_1'):
        t=start
        while t<=until:
            p=intruder_at(t)
            if math.dist(p,s.states[source]['position'])<8. or math.dist(p[:2],[0.,-1.5])<6.:
                s.on_detections(message({'source':source,'detections':[{'p':p,'label':'drone','sigma':0.15,'t':t+1.}]}))
            s.protect(t+1.);t=round(t+0.2,6)
    def test_station_raises_red_relocates_and_orders_keep_clear_without_closing(self):
        s=self.station();self.fly(s,0.,16.)
        self.assertEqual(s.posture.state,'RED');self.assertTrue(s.relocating)
        self.assertEqual(s.carrier_speed(20.),RELOCATE['speed'])
        records=self.records('station.jsonl');kinds={r['kind'] for r in records}
        self.assertLessEqual({'confirmed','posture','crew_alert','relocate','center_report','order'},kinds)
        keep=[r for r in records if r['kind']=='order' and r['action']=='keep_clear']
        self.assertTrue(keep)
        for r in keep:
            for q in r['threats']:self.assertGreaterEqual(math.dist(r['target'],q),math.dist(r['own'],q)-1e-6)
        payload=json.loads(json.dumps({'clearance':s.clear,'orders':s.orders,'posture':s.posture.state}))
        self.assertEqual(set(payload['orders']),{g['ns'] for g in GUARDIANS})
    def test_recovery_waits_for_the_center_and_carries_its_authority(self):
        s=self.station();self.fly(s,0.,40.);now=41.
        while s.posture.state!='GREEN' and now<120.:now+=0.2;s.protect(now)  # The intruder has gone.
        s.protect(now+5.)
        self.assertEqual(s.posture.state,'GREEN');self.assertTrue(s.held);self.assertIsNone(s.recovery)
        self.assertTrue(any(r['kind']=='center_report' and r['report']=='clear_after_red' for r in self.records('station.jsonl')))
        self.assertTrue(all(o['action']=='hold' for o in s.orders.values()))
        s.on_decision(message({'decision':'recover'}));s.protect(now+6.)
        self.assertEqual((s.recovery,s.recovery_by),('recover','center'))
        for o in s.orders.values():self.assertEqual((o['action'],o['authority']),('recover','center'))
    def test_station_lands_guardians_under_delegation_only_after_the_center_timeout(self):
        s=self.station();self.fly(s,0.,40.);now=41.
        while s.posture.state!='GREEN' and now<120.:now+=0.2;s.protect(now)
        cleared=s.clear_since;self.assertIsNotNone(cleared)
        s.protect(cleared+CFG.center_timeout-1.);self.assertIsNone(s.recovery)
        s.protect(cleared+CFG.center_timeout+0.5)
        self.assertEqual((s.recovery,s.recovery_by),('recover','station (delegated)'))
        self.assertTrue(authorised('recover','station (delegated)'))
    def test_center_logs_reports_and_answers_after_its_delays(self):
        c=CenterNode(SimpleNamespace(log=self.path('center.jsonl')));self.nodes.append(c)
        c.report(message({'kind':'clear_after_red'}));c.report(message({'kind':'posture','state':'RED'}))
        self.assertEqual([r['report'] for r in self.records('center.jsonl')],['clear_after_red','posture'])
        now=c.now();self.assertEqual(c.center.step(now),[])
        decided=c.center.step(now+10.);self.assertEqual(sorted(d for _,d in decided),['acknowledge','recover'])
    def test_world_drops_links_only_inside_the_jamming_zone(self):
        w=World(SimpleNamespace(log=self.path('world.jsonl'),seed=1,carrier=[0.,-1.5],deck=0.6));self.nodes.append(w)
        w.uplink={g['ns']:Mock() for g in GUARDIANS}
        w.pose={'px4_0':[1.,2.,4.],'px4_1':[0.,12.,5.],'px4_2':[12.,8.,3.]};w.jamming=True
        w.station_uplink(message({'orders':{}}))
        self.assertEqual({ns:m.publish.called for ns,m in w.uplink.items()},{'px4_0':False,'px4_1':True,'px4_2':True})
        self.assertEqual(w.dropped['px4_0'],1)
        w.jamming=False;w.station_uplink(message({'orders':{}}));self.assertTrue(w.uplink['px4_0'].publish.called)
        w.started=1.;w.intruder={'p':[0.,6.,4.],'v':[0.,0.,0.]}
        self.assertTrue(any(w.detect({'range':8.,'pd':0.9,'sigma':0.15},[0.,12.,5.]) for _ in range(5)))
        self.assertIsNone(w.detect({'range':8.,'pd':1.,'sigma':0.15},[0.,30.,5.]))
    def guardian(self,order,link_age,now=100.):
        g=SimpleNamespace(ns='px4_0',post=[1.,2.,4.],altitude=4.,onboard=Tracker(CFG),order=order,last_uplink=now-link_age,
                          carrier=(0.,-1.5,0.,0.,0.),blocked=set(),reflex=None,action='watch',layer='station',hold_at=None,
                          guard_target=None,log=Mock(),start_return=Mock(return_value=[1.,-1.,4.]))
        for name in ('guard','log_decision'):setattr(g,name,MethodType(getattr(GuardianMission,name),g))
        return g
    def feed_track(self,g,now):
        for k in range(12):  # An intruder closing on the post from the north at 1.2 m/s.
            t=now-2.4+k*0.2;g.onboard.update(t,[([0.6,2.+1.2*(now+3.-t),4.],'px4_0','drone',0.15)])
    def test_jammed_guardian_keeps_clear_on_its_own_without_closing(self):
        g=self.guardian({'action':'watch','target':[1.,2.,4.]},link_age=5.);self.feed_track(g,100.)
        target=g.guard(100.,[1.,2.,4.])
        self.assertEqual((g.action,g.layer),('keep_clear','onboard'))
        logged=g.log.write.call_args.kwargs
        for q in logged['threats']:self.assertGreaterEqual(math.dist(target,q),math.dist([1.,2.,4.],q)-1e-6)
    def test_guardian_follows_station_orders_and_center_recovery(self):
        g=self.guardian({'action':'keep_clear','target':[4.,0.5,4.],'until':200.},link_age=0.1)
        self.assertEqual(g.guard(100.,[1.,2.,4.]),[4.,0.5,4.]);self.assertEqual(g.layer,'station')
        g=self.guardian({'action':'recover','target':None,'authority':'center'},link_age=0.1)
        self.assertEqual(g.guard(100.,[1.,2.,4.]),[1.,-1.,4.]);g.start_return.assert_called_once()
        self.assertEqual(g.log.write.call_args.kwargs['layer'],'center')
        self.assertTrue(authorised('recover','center'))
        g=self.guardian({'action':'watch','target':[1.,2.,4.]},link_age=CFG.lost_link+1)
        g.guard(100.,[1.2,2.,4.]);self.assertEqual((g.action,g.layer),('lost_link_hold','onboard'))

if __name__=='__main__':unittest.main()
