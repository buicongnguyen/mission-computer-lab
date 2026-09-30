"""Guardian decision logic (tools/guardian.py) and the fast simulator built on it."""
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import guardian as G
import guardian_sim as S

ROOT=Path(__file__).resolve().parents[1]
CFG=S.CFG

def detections_along(p0,v,times,sigma,rng,source='s'):
    return [(t,[a+b*t+rng.gauss(0,sigma) for a,b in zip(p0,v)],source) for t in times]

class Geometry(unittest.TestCase):
    def test_retained_move_is_replanned_when_its_path_becomes_blocked(self):
        previous={'action':'keep_clear','target':[300.,0.,80.],'until':200.}
        free=Mock(return_value=False)
        order=G.keep_clear_step([0.,0.,80.],previous,[],100.,CFG,CFG.order_horizon,free)
        self.assertTrue(free.called);self.assertIsNot(order,previous)
        self.assertEqual(order['target'],[0.,0.,80.])
        self.assertNotIn('miss',order)
        json.dumps(order,allow_nan=False)  # A blocked route with no tracks must remain safe to log/send.
        good=G.keep_clear_step([0.,0.,80.],previous,[],100.,CFG,CFG.order_horizon,lambda p:True)
        self.assertIs(good,previous)
    def test_vehicle_revalidates_station_move_against_current_free_space(self):
        order={'action':'keep_clear','target':[300.,0.,80.],'until':200.}
        action,target,layer,detail=G.onboard_decide(100.,[0.,0.,80.],[],0.,order,CFG,free=lambda p:False)
        self.assertEqual((action,target,layer),('keep_clear',[0.,0.,80.],'onboard'))
        self.assertNotIn('miss',detail)
        json.dumps(detail,allow_nan=False)
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
    def test_a_late_measurement_is_compared_with_where_the_track_was_then(self):
        tracker=G.Tracker(CFG);rng=random.Random(5)
        self.feed(tracker,detections_along([6000.,0.,200.],[-250.,0.,0.],[i*0.5 for i in range(21)],5.,rng))
        tr=tracker.confirmed()[0];before=(tr.predict(10.),list(tr.v))
        tracker.update(9.6,[([6000.-250.*9.6,0.,200.],'late',None,5.)])  # Truth 0.4 s ago, arriving now.
        self.assertLess(math.dist(tr.predict(10.),before[0]),5.)
        self.assertLess(math.dist(tr.v,before[1]),5.)
    def test_a_tied_class_vote_goes_to_the_class_that_needs_more_protection(self):
        tr=G.Track(1,0.,[0.,0.,0.],'s');tr.labels={'bird':2,'drone':2}
        self.assertEqual(tr.cls,'drone')

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
        bird=self.track([900.,0.,40.],[-8.,0.,0.],'bird')
        for now in (0.,20.):bird.t=now;self.assertEqual(G.assess(bird,now,[0,0,0],[0,0,0],CFG)[0],'bird')
        cases=[self.track([900.,0.,40.],[-8.,0.,0.]),                  # Slow, unclassified, outside inner radius.
               self.track([1500.,0.,40.],[-250.,0.,0.],n=CFG.confirm_hits)]  # Not yet supported beyond confirmation.
        for tr in cases:
            for now in (0.,20.):tr.t=now;self.assertEqual(G.assess(tr,now,[0,0,0],[0,0,0],CFG)[0],'watch')
        coasting=self.track([3000.,0.,200.],[-250.,0.,0.],now=0.)
        self.assertEqual(G.assess(coasting,3.,[0,0,0],[0,0,0],CFG)[0],'watch')
    def test_a_relocating_station_is_judged_where_it_will_stop(self):
        # The station drives east to (400, 0) at 8 m/s; the drone flies straight at that stop point.
        tr=self.track([400.,1500.,40.],[0.,-18.,0.],'drone')
        t,d=G.station_approach(tr.p,tr.v,[0.,0.,0.],[8.,0.,0.],[400.,0.,0.])
        self.assertLess(d,1.);self.assertAlmostEqual(t,1500./18.,delta=0.5)
        forever=G.station_approach(tr.p,tr.v,[0.,0.,0.],[8.,0.,0.])[1]  # As if it never stopped.
        self.assertGreater(forever,CFG.protect_radius)
        G.assess(tr,0.,[0.,0.,0.],[8.,0.,0.],CFG,station_goal=[400.,0.,0.])
        tr.t=CFG.slow_persist;tr.p=tr.predict(CFG.slow_persist)
        self.assertEqual(G.assess(tr,CFG.slow_persist,[8*CFG.slow_persist,0.,0.],[8.,0.,0.],CFG,station_goal=[400.,0.,0.])[0],'danger')
    def test_posture_ignores_birds(self):
        p=G.Posture(CFG);self.assertFalse(p.update(0.,['bird','bird']));self.assertEqual(p.state,G.GREEN)
    def test_posture_raises_at_once_and_steps_down_slowly(self):
        p=G.Posture(CFG);self.assertTrue(p.update(0.,['danger']));self.assertEqual(p.state,G.RED)
        p.update(CFG.clear_time-1,[]);self.assertEqual(p.state,G.RED)
        p.update(CFG.clear_time+1,['watch']);self.assertEqual(p.state,G.AMBER)
        p.update(2*CFG.clear_time+2,[]);self.assertEqual(p.state,G.GREEN)

class KeepClear(unittest.TestCase):
    def test_never_closes_on_any_threat_anywhere_along_the_move(self):
        rng=random.Random(4)
        for _ in range(300):
            own=[rng.uniform(-500,500),rng.uniform(-500,500),80.]
            threats=[([rng.uniform(-1500,1500),rng.uniform(-1500,1500),rng.uniform(20,60)],
                      [rng.uniform(-20,20),rng.uniform(-20,20),0.]) for _ in range(rng.randint(1,4))]
            point,_=G.keep_clear(own,threats,CFG)
            for tp,_ in threats:  # The closest point of the whole straight path, not just its end.
                self.assertGreaterEqual(G.segment_miss(G.sub(own,tp),G.sub(point,own),1.),math.dist(own,tp)-1e-6)
    def test_a_guardian_at_its_ceiling_descends_away_from_a_threat_crossing_above(self):
        import dataclasses
        cfg=dataclasses.replace(CFG,max_altitude=80.,min_altitude=30.)
        point,_=G.keep_clear([0.,0.,80.],[([600.,0.,110.],[-18.,0.,0.])],cfg,free=lambda p:math.hypot(p[0],p[1])<1.)
        self.assertEqual(point,[0.,0.,80.-cfg.climb_step])  # Boxed in sideways, at the ceiling: only down is open.
        self.assertEqual(G.keep_clear([0.,0.,60.],[([600.,0.,110.],[-18.,0.,0.])],CFG,free=lambda p:math.hypot(p[0],p[1])<1.)[0][2]>=60.,True)
    def test_a_steady_fast_object_is_judged_on_its_path_not_where_it_is_now(self):
        # A 250 m/s object 1.5 km north-west of a guardian, diving to a point 400 m south of it.
        own=[0.,0.,80.];p=[-900.,1200.,200.];aim=[0.,-400.,0.]
        tr=G.Track(1,0.,p,'s');tr.v=[(a-b)/math.dist(p,aim)*250. for a,b in zip(aim,p)];tr.speeds=[250.,251.,249.]
        self.assertTrue(G.steady_fast(tr,CFG));self.assertEqual(G.relevant(own,[tr],0.,CFG),[])
        fast=G.fast_paths([tr],0.,CFG);hold=G.predicted_miss(own,own,fast,CFG,60.)
        north=[0.,250.,80.]  # Out of the impact area, but toward where the object is now.
        self.assertFalse(G.opens_range(own,north,[p]))  # The position rule would forbid it,
        self.assertGreater(G.predicted_miss(own,north,fast,CFG,60.),hold)  # yet it widens the predicted miss.
        self.assertTrue(G.safe_move(own,north,[],fast,CFG))
        west=[-250.,0.,80.]  # Toward the dive path: it shortens the predicted miss and is refused.
        self.assertLess(G.predicted_miss(own,west,fast,CFG,60.),hold);self.assertFalse(G.safe_move(own,west,[],fast,CFG))
        tr.speeds=[250.,120.,249.]  # Not steady: judged where it is, like any other track.
        self.assertEqual(G.fast_paths([tr],0.,CFG),[]);self.assertEqual(len(G.relevant(own,[tr],0.,CFG)),1)
    def test_dispersal_takes_the_step_that_leaves_the_area_fastest_when_its_edge_is_out_of_reach(self):
        own=[0.,0.,80.];hazard=([0.,-300.,0.],100.)  # The nearest edge point, (0, -180), is out of reach.
        reach=lambda q:math.dist(q[:2],own[:2])<=150.
        q=G.disperse_point(own,hazard,[],reach,CFG)
        self.assertAlmostEqual(q[1],CFG.keep_clear_step/2,places=6);self.assertAlmostEqual(q[0],0.,places=6)
        self.assertIsNone(G.disperse_point(own,hazard,[],lambda q:False,CFG))  # Nothing safe gains distance.
        q=G.disperse_point(own,hazard,[],lambda q:True,CFG)  # Otherwise the nearest point just outside the edge.
        self.assertAlmostEqual(math.dist(q[:2],hazard[0][:2]),1.2*hazard[1],places=6)
    def test_predicted_miss_is_exact_for_a_fast_object(self):
        # A 250 m/s object passes straight through a hovering guardian; a coarse time grid reported 250 m.
        self.assertLess(G.predicted_miss([0.,0.,80.],[0.,0.,80.],[([-5000.,0.,80.],[250.,0.,0.])],CFG,60.),1e-6)
        miss=G.predicted_miss([0.,0.,80.],[0.,500.,80.],[([-5000.,0.,80.],[250.,0.,0.])],CFG,60.)
        self.assertAlmostEqual(miss,20*CFG.guardian_speed,delta=1.)  # Where the guardian is when the object crosses.
    def test_relevant_tracks_leave_out_birds_and_tracks_that_stay_far_away(self):
        near=G.Track(1,0.,[600.,200.,80.],'s');near.v=[-10.,0.,0.]
        far=G.Track(2,0.,[600.,2000.,80.],'s');far.v=[-10.,0.,0.]
        bird=G.Track(3,0.,[300.,0.,80.],'s');bird.v=[-5.,0.,0.];bird.labels={'bird':5}
        self.assertEqual(G.relevant([0.,0.,80.],[near,far,bird],0.,CFG),[near.p])
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
    def test_a_kept_move_is_replanned_once_the_rest_of_it_would_close_on_a_track(self):
        tr=G.Track(1,0.,[600.,200.,80.],'s');tr.v=[-10.,0.,0.];tr.labels={'drone':5}  # Passes 200 m north.
        previous={'action':'keep_clear','target':[250.,0.,80.],'until':100.,'miss':0.,'t_cpa':10.,'tracks':[1]}
        order=G.keep_clear_step([0.,0.,80.],previous,[tr],0.,CFG,CFG.order_horizon,lambda p:True)
        self.assertIsNot(order,previous);self.assertTrue(G.opens_range([0.,0.,80.],order['target'],[tr.p]))
    def test_onboard_acts_alone_only_for_imminent_conflicts_or_lost_links(self):
        order={'action':'watch','target':[0.,900.,80.]}
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],0.,order,CFG)[:3],('watch',[0.,900.,80.],'station'))
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],CFG.lost_link+1,order,CFG)[0],'lost_link_hold')
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],CFG.lost_link_return+1,order,CFG,rally=[0,0,80])[0],'lost_link_return')
        tr=G.Track(1,0.,[0.,1100.,50.],'s');tr.v=[0.,-18.,0.]
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[tr],0.,order,CFG)[0],'watch')  # Just confirmed: not yet trusted.
        tr.n=CFG.confirm_hits+2
        action,_,layer,_=G.onboard_decide(0.,[0.,900.,80.],[tr],0.,order,CFG)
        self.assertEqual((action,layer),('keep_clear','onboard'))
    def test_a_guardian_does_not_fly_a_station_move_that_closes_on_what_it_sees(self):
        tr=G.Track(1,0.,[600.,1100.,80.],'s');tr.v=[-10.,0.,0.];tr.labels={'drone':5}
        stale={'action':'keep_clear','target':[250.,900.,80.],'until':50.}  # Planned from an old report.
        action,target,layer,_=G.onboard_decide(0.,[0.,900.,80.],[tr],0.,stale,CFG)
        self.assertEqual((action,layer),('keep_clear','onboard'));self.assertTrue(G.opens_range([0.,900.,80.],target,[tr.p]))
        fine={'action':'keep_clear','target':[-250.,900.,80.],'until':50.}
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[tr],0.,fine,CFG)[2],'station')
    def test_a_guardian_on_station_fixes_lands_when_they_stop(self):
        order={'action':'watch','target':[0.,900.,80.]}
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],0.,order,CFG,fix_age=CFG.fix_timeout-1)[0],'watch')
        self.assertEqual(G.onboard_decide(0.,[0.,900.,80.],[],0.,order,CFG,fix_age=CFG.fix_timeout+1)[:3],('land_in_place',None,'onboard'))
        self.assertTrue(G.authorised('land_in_place','onboard'));self.assertFalse(G.authorised('land_in_place','station'))

class StationAndCenter(unittest.TestCase):
    def test_orders_follow_posture_and_authority(self):
        view={'g0':{'p':[0.,900.,80.],'post':[0.,900.,80.]},'g4':{'p':[0.,0.,60.],'post':[0.,0.,60.]}}
        o=G.station_orders(0.,G.GREEN,[],view,CFG);self.assertEqual(o['g0']['action'],'watch')
        o=G.station_orders(0.,G.RED,[],view,CFG,hazard=([0,0,0],200.))
        self.assertEqual((o['g0']['action'],o['g4']['action']),('hold','disperse'))
        self.assertGreaterEqual(math.dist(o['g4']['target'][:2],[0,0]),200.)  # Out of the area, even from its centre.
        o=G.station_orders(0.,G.GREEN,[],view,CFG,held=True);self.assertEqual(o['g0']['action'],'hold')
        o=G.station_orders(0.,G.GREEN,[],view,CFG,recovery='recover',held=True);self.assertEqual(o['g0']['action'],'recover')
        self.assertTrue(G.authorised('recover','center') and G.authorised('recover','station (delegated)'))
        self.assertFalse(G.authorised('recover','station') or G.authorised('resume','station') or G.authorised('keep_clear','center'))
    def test_dispersal_is_guarded_like_any_other_move(self):
        tr=G.Track(1,0.,[700.,0.,80.],'s');tr.v=[-10.,0.,0.];tr.labels={'drone':5}
        view={'g4':{'p':[0.,0.,60.],'post':[0.,0.,60.]}}
        o=G.station_orders(0.,G.RED,[tr],view,CFG,free_for=lambda name:(lambda p:p[1]>=0),hazard=([0.,0.,0.],200.))['g4']
        self.assertEqual(o['action'],'disperse');self.assertGreaterEqual(o['target'][1],0.)
        self.assertTrue(G.opens_range([0.,0.,60.],o['target'],[tr.p]))
        tr.v=[0.,0.,-100.];self.assertEqual(G.impact_point(tr,0.,[500.,500.,0.])[:2],[700.,0.])  # Where it comes down.
    def test_center_answers_after_latency_and_decision_time_with_the_request_it_answers(self):
        c=G.Center(1.5,10.);c.send(0.,{'kind':'clear_after_red','request':7})
        self.assertEqual(c.step(5.),[])
        self.assertEqual(c.step(11.6),[(13.,'recover',7)])
        down=G.Center(1.5,10.,reachable=lambda now:False);down.send(0.,{'kind':'posture','state':'RED'})
        self.assertEqual(down.step(100.),[])
    def test_center_retries_a_decision_until_the_link_returns(self):
        c=G.Center(1.5,10.,reachable=lambda now:now<1. or now>30.);c.send(0.,{'kind':'clear_after_red','request':1})
        self.assertEqual([c.step(t/10) for t in range(0,300)],[[]]*300)  # Decided at 11.5 s, link down until 30 s.
        delivered=[d for t in range(300,340) for d in c.step(t/10)]
        self.assertEqual(len(delivered),1);self.assertEqual(delivered[0][1:],('recover',1))
    def test_integrity_alarm_needs_a_persistent_offset(self):
        n=G.NavIntegrity(CFG)
        for k in range(20):n.update(k,[0.,0.],[30.,0.])  # Within the threshold: never an alarm.
        self.assertEqual(n.state,'ok')
        for k in range(CFG.integrity_persistence):n.update(20+k,[0.,0.],[90.,0.])
        self.assertEqual(n.state,'spoofed')
    def test_integrity_ignores_old_reports_and_clears_after_consistent_observations(self):
        n=G.NavIntegrity(CFG)
        for k in range(10):n.update(k,[0.,0.],[500.,0.],age=CFG.integrity_max_age+1)  # A silent guardian's last report.
        self.assertEqual(n.state,'ok')
        for k in range(CFG.integrity_persistence):n.update(10+k,[0.,0.],[90.,0.])
        for k in range(CFG.integrity_clear-1):n.update(20+k,[0.,0.],[5.,0.])
        self.assertEqual(n.state,'spoofed');n.update(30,[0.,0.],[5.,0.]);self.assertEqual(n.state,'ok')

class Simulator(unittest.TestCase):
    """Seeded end-to-end runs; the published report covers many more seeds."""
    def test_hybrid_design_protects_the_fleet_and_keeps_its_invariants(self):
        for scenario in ('intruder','jamming','combined'):
            for seed in (0,1):
                r=S.run(scenario,'hybrid',seed)
                self.assertTrue(r['guardians_safe'],(scenario,seed,r['min_separation_m']))
                self.assertTrue(r['never_closed'] and r['authority_ok'],(scenario,seed))
                self.assertGreater(r['warning_s'],15.,(scenario,seed))
    def test_the_station_never_mistakes_a_bird_for_its_own_guardian(self):
        # Seed 6 once handed a picket's station track to a bird beside it, flagged the picket as spoofed and
        # steered it hundreds of metres by fixes of the bird.
        for seed in (5,6,7):
            r=S.run('birds','hybrid',seed)
            self.assertEqual((r['false_integrity'],r['false_red']),(0,0),seed);self.assertLess(r['healthy_nav_error_m'],1.,seed)
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
        for scenario in ('intruder','swarm','jamming','combined','center_loss'):
            self.assertEqual(report['summary'][scenario]['hybrid']['false_integrity_total'],0,scenario)

if __name__=='__main__':unittest.main()
