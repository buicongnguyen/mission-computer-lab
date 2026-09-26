"""Guardian decision logic (tools/guardian.py) and the fast simulator built on it."""
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import guardian as G
import guardian_sim as S

ROOT=Path(__file__).resolve().parents[1]
CFG=S.CFG

def detections_along(p0,v,times,sigma,rng,source='s'):
    return [(t,[a+b*t+rng.gauss(0,sigma) for a,b in zip(p0,v)],source) for t in times]

class Geometry(unittest.TestCase):
    def test_closest_approach(self):
        t,d=G.closest_approach([100.,50.,0.],[-10.,0.,0.])
        self.assertAlmostEqual(t,10.);self.assertAlmostEqual(d,50.)
        self.assertEqual(G.closest_approach([100.,0.,0.],[10.,0.,0.]),(0.,100.))  # Receding: now is closest.
        self.assertEqual(G.closest_approach([3.,4.,0.],[0.,0.,0.]),(0.,5.))

class Tracking(unittest.TestCase):
    def feed(self,tracker,dets):
        for t,p,source in sorted(dets,key=lambda d:d[0]):tracker.update(t,[(p,source,None,10.)])
    def test_confirms_and_estimates_a_straight_target(self):
        tracker=G.Tracker(CFG);rng=random.Random(1)
        self.feed(tracker,detections_along([2000.,0.,40.],[-18.,0.,0.],[i*0.5 for i in range(40)],10.,rng))
        tracks=tracker.confirmed();self.assertEqual(len(tracks),1)
        self.assertLess(math.dist(tracks[0].v,[-18.,0.,0.]),4.)  # Kalman velocity, not a two-point spike.
    def test_two_sensors_make_one_track(self):
        tracker=G.Tracker(CFG);rng=random.Random(2);times=[i*0.5 for i in range(30)]
        dets=(detections_along([1500.,300.,40.],[-15.,-3.,0.],times,10.,rng,'a')+
              detections_along([1500.,300.,40.],[-15.,-3.,0.],[t+0.25 for t in times],10.,rng,'b'))
        self.feed(tracker,dets);self.assertEqual(len(tracker.confirmed()),1)
        self.assertEqual(tracker.confirmed()[0].sources,{'a','b'})
    def test_clutter_does_not_confirm_implausible_tracks(self):
        tracker=G.Tracker(CFG);rng=random.Random(3);confirmed=set()
        for k in range(600):  # Five random points a second over a 1.6 km disk for two minutes.
            t=k*0.2
            tracker.update(t,[([rng.uniform(-800,800),rng.uniform(-800,800),rng.uniform(0,200)],'clutter',None,10.)])
            confirmed|={tr.id for tr in tracker.confirmed() if tr.speed>CFG.gate_speed}
        self.assertEqual(confirmed,set())

class Assessment(unittest.TestCase):
    def track(self,p,v,cls=None,n=10,now=0.):
        tr=G.Track(1,now,p,'s');tr.v=list(v);tr.n=n;tr.confirmed=True
        if cls:tr.labels={cls:5}
        return tr
    def test_inbound_drone_is_danger_after_persisting(self):
        tr=self.track([1500.,0.,40.],[-18.,0.,0.],'drone')
        self.assertEqual(G.assess(tr,0.,[0,0,0],[0,0,0],CFG)[0],'watch')
        tr.t=CFG.slow_persist;tr.p=tr.predict(CFG.slow_persist)
        self.assertEqual(G.assess(tr,CFG.slow_persist,[0,0,0],[0,0,0],CFG)[0],'danger')
    def test_fast_track_is_danger_at_once_only_with_a_steady_speed(self):
        steady=self.track([3000.,0.,200.],[-250.,0.,-10.]);steady.speeds=[248.,251.,250.]
        self.assertEqual(G.assess(steady,0.,[0,0,0],[0,0,0],CFG)[0],'danger')
        chain=self.track([3000.,0.,200.],[-100.,0.,0.]);chain.speeds=[136.,80.,100.]  # Clutter lined up by chance.
        self.assertEqual(G.assess(chain,0.,[0,0,0],[0,0,0],CFG)[0],'watch')
    def test_bird_whose_speed_estimate_hovers_at_the_threshold_never_raises_red(self):
        tr=self.track([1365.,0.,40.],[-12.1,0.,0.])
        for k in range(60):  # 30 s of estimates alternating just above and below slow_speed.
            now=k*0.5;tr.t=now;tr.v=[-(CFG.slow_speed+(0.1 if k%2 else -3.)),0.,0.]
            self.assertEqual(G.assess(tr,now,[0,0,0],[0,0,0],CFG)[0],'watch',now)
    def test_weaving_drone_heading_roughly_inbound_counts(self):
        v=[-18*math.cos(math.radians(20)),18*math.sin(math.radians(20)),0.]  # 20 degrees off the line.
        tr=self.track([1500.,0.,40.],v,'drone');G.assess(tr,0.,[0,0,0],[0,0,0],CFG)
        tr.t=10.;tr.p=tr.predict(10.)
        self.assertEqual(G.assess(tr,10.,[0,0,0],[0,0,0],CFG)[0],'danger')
    def test_birds_slow_unknowns_coasting_and_young_tracks_do_not_raise_red(self):
        cases=[self.track([900.,0.,40.],[-8.,0.,0.],'bird'),           # Classed as a bird.
               self.track([900.,0.,40.],[-8.,0.,0.]),                  # Slow, unclassified, outside inner radius.
               self.track([1500.,0.,40.],[-250.,0.,0.],n=CFG.confirm_hits)]  # Not yet supported beyond confirmation.
        for tr in cases:
            for now in (0.,20.):tr.t=now;self.assertEqual(G.assess(tr,now,[0,0,0],[0,0,0],CFG)[0],'watch')
        coasting=self.track([3000.,0.,200.],[-250.,0.,0.],now=0.)
        self.assertEqual(G.assess(coasting,3.,[0,0,0],[0,0,0],CFG)[0],'watch')
    def test_posture_raises_at_once_and_steps_down_slowly(self):
        p=G.Posture(CFG);self.assertTrue(p.update(0.,['danger']));self.assertEqual(p.state,G.RED)
        p.update(CFG.clear_time-1,[]);self.assertEqual(p.state,G.RED)
        p.update(CFG.clear_time+1,['watch']);self.assertEqual(p.state,G.AMBER)
        p.update(2*CFG.clear_time+2,[]);self.assertEqual(p.state,G.GREEN)

class KeepClear(unittest.TestCase):
    def test_never_closes_on_any_threat(self):
        rng=random.Random(4)
        for _ in range(300):
            own=[rng.uniform(-500,500),rng.uniform(-500,500),80.]
            threats=[([rng.uniform(-1500,1500),rng.uniform(-1500,1500),rng.uniform(20,60)],
                      [rng.uniform(-20,20),rng.uniform(-20,20),0.]) for _ in range(rng.randint(1,4))]
            point,_=G.keep_clear(own,threats,CFG)
            for tp,_ in threats:self.assertGreaterEqual(math.dist(point,tp),math.dist(own,tp)-1e-9)
    def test_respects_free_space_and_climbs_away_from_low_threats(self):
        own=[0.,0.,80.];threat=([600.,0.,40.],[-18.,0.,0.])
        point,miss=G.keep_clear(own,[threat],CFG,free=lambda p:p[1]>=0)
        self.assertGreaterEqual(point[1],0.);self.assertGreater(miss,CFG.clear_radius)
        self.assertLessEqual(point[2],CFG.max_altitude)
    def test_episode_is_kept_and_replanned_but_never_dropped_to_hold(self):
        tr=G.Track(1,0.,[800.,0.,40.],'s');tr.v=[-18.,0.,0.];tr.labels={'drone':5}
        first=G.keep_clear_step([0.,0.,80.],None,[tr],0.,CFG,CFG.order_horizon,lambda p:True)
        self.assertEqual(first['action'],'keep_clear')
        kept=G.keep_clear_step(first['target'],first,[tr],1.,CFG,CFG.order_horizon,lambda p:True)
        self.assertIs(kept,first)  # Target still clear: the same order stands.
        tr.v=[-18.*first['target'][0]/800.,-18.*first['target'][1]/800.,0.]  # The threat turns toward the target.
        again=G.keep_clear_step([0.,0.,80.],first,[tr],1.,CFG,CFG.order_horizon,lambda p:True)
        self.assertEqual(again['action'],'keep_clear')
    def test_onboard_acts_alone_only_for_imminent_conflicts_or_lost_links(self):
        order={'action':'watch','target':[0.,900.,80.]}
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],0.,order,CFG)[:3],('watch',[0.,900.,80.],'station'))
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],CFG.lost_link+1,order,CFG)[0],'lost_link_hold')
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],CFG.lost_link_return+1,order,CFG,rally=[0,0,80])[0],'lost_link_return')
        tr=G.Track(1,0.,[0.,1100.,50.],'s');tr.v=[0.,-18.,0.]
        action,_,layer,_=G.onboard_decide(0.,[0.,900.,80.],[tr],0.,order,CFG)
        self.assertEqual((action,layer),('keep_clear','onboard'))

class StationAndCenter(unittest.TestCase):
    def test_orders_follow_posture_and_authority(self):
        view={'g0':{'p':[0.,900.,80.],'post':[0.,900.,80.]},'g4':{'p':[0.,0.,60.],'post':[0.,0.,60.]}}
        o=G.station_orders(0.,G.GREEN,[],view,CFG);self.assertEqual(o['g0']['action'],'watch')
        o=G.station_orders(0.,G.RED,[],view,CFG,hazard=([0,0,0],200.))
        self.assertEqual((o['g0']['action'],o['g4']['action']),('hold','disperse'))
        o=G.station_orders(0.,G.GREEN,[],view,CFG,held=True);self.assertEqual(o['g0']['action'],'hold')
        o=G.station_orders(0.,G.GREEN,[],view,CFG,recovery='recover',held=True);self.assertEqual(o['g0']['action'],'recover')
        self.assertTrue(G.authorised('recover','center') and G.authorised('recover','station (delegated)'))
        self.assertFalse(G.authorised('recover','station') or G.authorised('resume','station') or G.authorised('keep_clear','center'))
    def test_center_answers_after_latency_and_decision_time_and_retries_a_dead_link(self):
        c=G.Center(1.5,10.);c.send(0.,{'kind':'clear_after_red'})
        self.assertEqual(c.step(5.),[])
        self.assertEqual(c.step(11.6),[(13.,'recover')])
        down=G.Center(1.5,10.,reachable=lambda now:False);down.send(0.,{'kind':'posture','state':'RED'})
        self.assertEqual(down.step(100.),[])
    def test_integrity_alarm_needs_a_persistent_offset(self):
        n=G.NavIntegrity(CFG)
        for k in range(20):n.update(k,[0.,0.],[30.,0.])  # Within the threshold: never an alarm.
        self.assertEqual(n.state,'ok')
        for k in range(CFG.integrity_persistence):n.update(20+k,[0.,0.],[90.,0.])
        self.assertEqual(n.state,'spoofed')

class Simulator(unittest.TestCase):
    """Seeded end-to-end runs; the published report covers many more seeds."""
    def test_hybrid_design_protects_the_fleet_and_keeps_its_invariants(self):
        for scenario in ('intruder','jamming','combined'):
            for seed in (0,1):
                r=S.run(scenario,'hybrid',seed)
                self.assertTrue(r['guardians_safe'],(scenario,seed,r['min_separation_m']))
                self.assertTrue(r['never_closed'] and r['authority_ok'],(scenario,seed))
                self.assertGreater(r['warning_s'],15.,(scenario,seed))
    def test_architectures_differ_where_the_design_says_they_should(self):
        station=S.run('intruder','station_only',0);hybrid=S.run('intruder','hybrid',0)
        self.assertGreater(hybrid['warning_s'],station['warning_s']+30.)  # Guardians see low flyers far out.
        jammed=S.run('jamming','hybrid',0);self.assertIsNotNone(jammed['jam_fallback_s'])
        self.assertLessEqual(jammed['jam_fallback_s'],CFG.lost_link+0.5)
        spoof=S.run('spoofing','hybrid',0);blind=S.run('spoofing','networked',0)
        self.assertLess(spoof['spoof_detect_s'],60.);self.assertLess(spoof['spoof_drift_m'],120.)
        self.assertGreater(blind['spoof_drift_m'],400.)  # Without the cross-check the drag-off succeeds.
        for arch in S.ARCHITECTURES:self.assertEqual(S.run('birds',arch,0)['false_red'],0,arch)
    def test_runs_are_reproducible(self):
        self.assertEqual(S.run('swarm','hybrid',3),S.run('swarm','hybrid',3))

class PublishedReport(unittest.TestCase):
    def test_report_matches_the_current_decision_code(self):
        path=ROOT/'artifacts/guardian/report.json'
        report=json.loads(path.read_text(encoding='utf-8'))
        for name,digest in report['source_sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT/'tools'/name).read_bytes()).hexdigest(),digest,
                             f'{name} changed since the report was generated; rerun tools/guardian_sim.py')
        self.assertEqual(set(report['summary']),set(S.SCENARIOS))
        for scenario in S.SCENARIOS:
            self.assertEqual(set(report['summary'][scenario]),set(S.ARCHITECTURES))
            for row in report['summary'][scenario].values():
                self.assertEqual(row['runs'],report['seeds'])
                self.assertEqual((row['never_closed_rate'],row['authority_ok_rate']),(1.0,1.0))

if __name__=='__main__':unittest.main()
