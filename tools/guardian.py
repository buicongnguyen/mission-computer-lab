"""Guardian decision logic, shared by the fast simulator (tools/guardian_sim.py) and the PX4 SITL nodes.

The design is deliberately non-kinetic. A guardian senses, reports, warns, keeps clear, shelters and falls
back when its links fail. Nothing here steers toward a threat: keep_clear() only accepts points that do not
reduce the range to any threat, and the tests hold every caller to that. How to respond to the threat itself
is decided by the center (command and control) and by authorised systems outside this lab.

Three layers decide on three time scales:
  onboard  (under a second)  imminent keep-clear, lost-link procedure
  station  (seconds)         fuse tracks, set the protection posture, order guardians, alert the crew, move
  center   (tens of seconds) acknowledge, authorise recovery or resumption, anything beyond protection
"""
from dataclasses import dataclass
import math

GREEN,AMBER,RED='GREEN','AMBER','RED'
INF=float('inf')

# Who may decide each action. The station's own authority is delegated by the center before the mission;
# 'station (delegated)' marks a center decision the station takes because the center cannot be reached.
AUTHORITY={
    'keep_clear':{'onboard','station'},'lost_link_hold':{'onboard'},'lost_link_return':{'onboard'},
    'hold':{'station'},'watch':{'station'},'disperse':{'station'},'crew_alert':{'station'},
    'relocate':{'station'},'navigate_by_station':{'station'},'posture':{'station'},
    'acknowledge':{'center'},'resume':{'center'},'recover':{'center','station (delegated)'},
}

@dataclass(frozen=True,kw_only=True)
class Config:
    """Scale-specific parameters: the fast simulator uses metres to kilometres, SITL a 20 m arena."""
    gate:float                 # minimum association gate (m)
    gate_speed:float           # m/s: the fastest plausible target, the initial velocity uncertainty of a track
    accel_noise:float          # m^2/s^3: process noise of the constant-velocity filter (manoeuvre allowance)
    sigma:float                # m: measurement noise assumed when a detection does not carry its own
    gate_chi2:float=16.        # normalised squared residual accepted for association
    confirm_hits:int=3         # a track is confirmed after this many hits ...
    confirm_window:float=3.0   # ... within this many seconds
    drop_tentative:float=2.5   # s without a hit before an unconfirmed track is dropped
    drop_confirmed:float=6.0   # s without a hit before a confirmed track is dropped
    protect_radius:float       # horizontal radius (m) of the protected zone around the station
    warn_horizon:float         # s: a track predicted to enter the zone within this time raises RED
    slow_persist:float         # s a slow track must keep predicting entry before it counts as danger
    heading_tolerance:float=25.  # degrees: a track heading this close to the station counts as inbound
    slow_speed:float           # m/s below which an unclassified track is treated as possibly a bird ...
    inner_radius:float         # ... and can raise RED only inside this range (m)
    clear_radius:float         # m: predicted miss distance guardians keep from every track
    safe_radius:float          # m: separation the acceptance checks require (smaller than clear_radius)
    order_horizon:float        # s: the station orders keep-clear for conflicts predicted within this time
    reflex_horizon:float       # s: a guardian acts on its own for conflicts predicted within this time
    keep_clear_step:float      # m: length of one keep-clear move (half-length moves are also offered)
    climb_step:float=0.        # m: a keep-clear move may also climb this much ...
    max_altitude:float=INF     # ... but never above this ceiling (m)
    guardian_speed:float       # m/s used to predict a keep-clear move
    fast_speed:float           # m/s above which a track is classed as fast
    clear_time:float           # s without danger before RED steps down; without tracks before AMBER does
    lost_link:float            # s without an uplink before a guardian holds on its own
    lost_link_return:float     # s without an uplink before it flies to its rally point
    center_timeout:float       # s after a clear the station waits for the center before landing guardians
    integrity_threshold:float  # m between reported and independently observed position
    integrity_persistence:int=3  # consecutive observations beyond the threshold before an alarm

def closest_approach(rel_p,rel_v):
    """Time (not before now) and distance of closest approach for a relative position and velocity."""
    vv=sum(c*c for c in rel_v)
    t=0. if vv<1e-12 else max(0.,-sum(p*v for p,v in zip(rel_p,rel_v))/vv)
    return t,math.hypot(*(p+v*t for p,v in zip(rel_p,rel_v)))

def sub(a,b):return [x-y for x,y in zip(a,b)]

class Track:
    """Constant-velocity Kalman filter; the axes share one 2x2 covariance because they share noise models."""
    def __init__(self,tid,now,p,source,label=None,sigma=1.,speed=1.):
        self.id=tid;self.p=list(p);self.v=[0.,0.,0.];self.t=now;self.born=now;self.danger_since=None
        self.P=[sigma*sigma,0.,speed*speed];self.inbound_at=None;self.speeds=[]
        self.hits=[now];self.n=1;self.sources={source};self.confirmed=False;self.confirmed_at=None;self.labels={}
        self.vote(label)
    def propagate(self,dt,q):
        """Covariance of the prediction dt seconds ahead: (pp, pv, vv)."""
        pp,pv,vv=self.P
        return (pp+2*dt*pv+dt*dt*vv+q*dt**3/3,pv+dt*vv+q*dt*dt/2,vv+q*dt)
    def correct(self,now,z,r2,q):
        dt=now-self.t
        if dt>0:self.p=self.predict(now);self.P=list(self.propagate(dt,q));self.t=now
        pp,pv,vv=self.P;s=pp+r2;k0,k1=pp/s,pv/s;res=sub(z,self.p)
        self.p=[a+k0*c for a,c in zip(self.p,res)];self.v=[a+k1*c for a,c in zip(self.v,res)]
        self.P=[pp*(1-k0),pv*(1-k0),vv-k1*pv]
    def vote(self,label):
        if label:self.labels[label]=self.labels.get(label,0)+1
    def predict(self,now):return [p+v*(now-self.t) for p,v in zip(self.p,self.v)]
    @property
    def speed(self):return math.hypot(*self.v)
    @property
    def cls(self):
        """Majority of the class labels camera-equipped sensors attached, once there are at least three."""
        if sum(self.labels.values())<3:return 'unknown'
        return max(sorted(self.labels),key=lambda k:self.labels[k])

class Tracker:
    """Nearest-neighbour association in normalised residual, Kalman smoothing, M-of-N confirmation,
    and merging of tracks that have converged on the same object."""
    def __init__(self,cfg):self.cfg=cfg;self.tracks=[];self.next_id=1
    def update(self,now,detections):
        """detections: (position, source[, class label or None[, measurement sigma]])."""
        cfg=self.cfg
        for p,source,*extra in detections:
            label=extra[0] if extra else None;sigma=extra[1] if len(extra)>1 else cfg.sigma;r2=sigma*sigma
            best,best_d=None,INF
            for tr in self.tracks:
                dt=max(0.,now-tr.t)
                # A young track may only take detections reachable at a plausible speed, so random clutter
                # cannot chain into a confirmed track flying at hundreds of metres per second.
                if len(tr.hits)<cfg.confirm_hits and math.dist(p,tr.p)>cfg.gate+cfg.gate_speed*dt:continue
                pp=tr.propagate(dt,cfg.accel_noise)[0]
                d2=sum(c*c for c in sub(p,tr.predict(now)))
                score=d2/(pp+r2)
                if (score<=cfg.gate_chi2 or d2<=cfg.gate*cfg.gate) and score<best_d:best,best_d=tr,score
            if best is None:
                self.tracks.append(Track(self.next_id,now,p,source,label,sigma,cfg.gate_speed));self.next_id+=1;continue
            best.correct(now,p,r2,cfg.accel_noise);best.speeds=(best.speeds+[best.speed])[-3:]
            best.hits.append(now);best.n+=1;best.sources.add(source);best.vote(label)
            best.hits=[h for h in best.hits if now-h<=cfg.confirm_window] if not best.confirmed else best.hits[-cfg.confirm_hits:]
            if not best.confirmed and len(best.hits)>=cfg.confirm_hits:best.confirmed=True;best.confirmed_at=now
        self.tracks=[tr for tr in self.tracks if now-tr.t<=(cfg.drop_confirmed if tr.confirmed else cfg.drop_tentative)]
        # Two confirmed tracks that sit together and move together are one object: keep the older.
        for a in sorted(self.confirmed(),key=lambda tr:tr.id):
            for b in self.confirmed():
                if b.id>a.id and b in self.tracks and a in self.tracks and \
                        math.dist(a.predict(now),b.predict(now))<2*cfg.gate and math.dist(a.v,b.v)<0.25*max(a.speed,b.speed,1.):
                    for k,n in b.labels.items():a.labels[k]=a.labels.get(k,0)+n
                    self.tracks.remove(b)
        return self.confirmed()
    def confirmed(self):return [tr for tr in self.tracks if tr.confirmed]

def heading_error(track,p,station_p,station_v):
    """Angle in degrees between a track's ground velocity (relative to the station) and the line to it."""
    rel=sub(track.v,station_v)[:2];to=sub(station_p,p)[:2]
    n=math.hypot(*rel)*math.hypot(*to)
    return 180. if n<1e-9 else math.degrees(math.acos(max(-1.,min(1.,(rel[0]*to[0]+rel[1]*to[1])/n))))

def assess(track,now,station_p,station_v,cfg):
    """Threat level of one track for the protected zone: ('danger' or 'watch', time to CPA, miss distance).

    Inbound means a predicted closest approach inside the zone, or a heading within heading_tolerance of
    the station (a weaving drone's straight-line prediction misses by hundreds of metres at long range).
    A fast track with a steady speed estimate counts as danger at once; clutter that lines up by chance
    does not hold a steady speed. A slow one must stay inbound for slow_persist seconds, and one no camera
    has classified as a drone may raise RED only inside inner_radius, because a circling bird's heading
    points at the station now and then. A track classified as a bird never counts."""
    p=track.predict(now)
    t,d=closest_approach(sub(p,station_p)[:2],sub(track.v,station_v)[:2])
    rng=math.dist(p[:2],station_p[:2])
    # Coasting, kinematically implausible, or not yet supported beyond its confirming hits (clutter can
    # line up four points by chance; it rarely keeps doing so) unless a camera has classed it a drone.
    if now-track.t>2. or track.speed>cfg.gate_speed or (track.n<cfg.confirm_hits+2 and track.cls!='drone'):
        track.danger_since=None;return 'watch',t,d
    inbound=((d<cfg.protect_radius or heading_error(track,p,station_p,station_v)<cfg.heading_tolerance)
             and track.cls!='bird' and rng/max(track.speed,1e-6)<cfg.warn_horizon)
    # Hysteresis bridges only a flickering heading. The bird gate below is judged on every update, so a
    # bird whose noisy speed estimate hovers around slow_speed cannot keep the persistence timer running.
    if inbound:track.inbound_at=now
    inbound=inbound or (track.inbound_at is not None and now-track.inbound_at<=2.)
    eligible=track.speed>=cfg.slow_speed or track.cls=='drone' or rng<cfg.inner_radius
    entering=inbound and eligible
    track.danger_since=(track.danger_since if track.danger_since is not None else now) if entering else None
    steady=len(track.speeds)>=3 and max(track.speeds)<=1.25*min(track.speeds)
    danger=entering and ((track.speed>=cfg.fast_speed and steady) or now-track.danger_since>=cfg.slow_persist)
    return ('danger' if danger else 'watch'),t,d

class Posture:
    """GREEN with no confirmed tracks, AMBER while any is watched, RED at once on danger; steps down slowly."""
    def __init__(self,cfg):self.cfg=cfg;self.state=GREEN;self.last_danger=-INF;self.last_track=-INF;self.red_seen=False
    def update(self,now,levels):
        if 'danger' in levels:self.last_danger=now
        if levels:self.last_track=now
        target=(RED if now-self.last_danger<self.cfg.clear_time else
                AMBER if now-self.last_track<self.cfg.clear_time else GREEN)
        changed=target!=self.state;self.state=target;self.red_seen|=target==RED
        return changed

def predicted_miss(start,goal,threats,cfg,horizon):
    """Smallest predicted 3-D distance to any threat while flying start -> goal and holding there."""
    length=math.dist(start,goal);worst=INF;steps=24
    for k in range(steps+1):
        t=horizon*k/steps;f=1. if length<1e-9 else min(1.,cfg.guardian_speed*t/length)
        g=[a+(b-a)*f for a,b in zip(start,goal)]
        for tp,tv in threats:worst=min(worst,math.dist(g,[p+v*t for p,v in zip(tp,tv)]))
    return worst

def keep_clear(own_p,threats,cfg,free=lambda p:True,bearings=16,horizon=None):
    """Best point for one keep-clear move and its predicted miss distance.

    threats: (position, velocity) predictions. A candidate is rejected if it is closer than own_p to any
    threat, so the move can never close on one; staying put is always an option."""
    horizon=horizon or max(cfg.order_horizon,cfg.reflex_horizon)
    options=[(predicted_miss(own_p,own_p,threats,cfg,horizon),0.,list(own_p))]
    heights=[own_p[2]]+([min(cfg.max_altitude,own_p[2]+cfg.climb_step)] if own_p[2]+1e-9<cfg.max_altitude and cfg.climb_step>0 else [])
    candidates=[[own_p[0],own_p[1],z] for z in heights[1:]]
    for step in (cfg.keep_clear_step,cfg.keep_clear_step/2):
        for i in range(bearings):
            a=2*math.pi*i/bearings
            candidates+=[[own_p[0]+step*math.cos(a),own_p[1]+step*math.sin(a),z] for z in heights]
    for cand in candidates:
        if not free(cand) or any(math.dist(cand,tp)<math.dist(own_p,tp) for tp,_ in threats):continue
        options.append((predicted_miss(own_p,cand,threats,cfg,horizon),math.dist(own_p,cand),cand))
    miss,_,point=max(options,key=lambda o:(round(o[0],6),-o[1]))
    return point,miss

def conflicts(own_p,tracks,now,cfg,horizon):
    """Tracks predicted to pass within clear_radius of own_p inside the horizon (own motion ignored).

    Birds only need ordinary collision avoidance, so they conflict inside half the safe radius."""
    out=[]
    for tr in tracks:
        t,d=closest_approach(sub(tr.predict(now),own_p),tr.v)
        if t<horizon and d<(cfg.safe_radius/2 if tr.cls=='bird' else cfg.clear_radius):out.append((tr,t,d))
    return out

def keep_clear_step(own_p,previous,tracks,now,cfg,horizon,free):
    """Keep-clear decision with an episode latch: a new keep-clear order, the previous one kept, or None.

    Once a guardian starts keeping clear, the episode lasts until its conflict has passed. Inside it the
    move is kept while its target stays clear, and re-planned from where the guardian is if not; it never
    drops back to holding. Without the latch a guardian that moved far enough to clear the prediction was
    told to hold, stopped halfway, was back in conflict as the threat weaved, and never completed a move."""
    episode=previous is not None and previous.get('action')=='keep_clear' and now<previous.get('until',-INF)
    if episode:
        at_target=conflicts(previous['target'],tracks,now,cfg,horizon)
        if not at_target:return previous
        return keep_clear_order(own_p,conflicts(own_p,tracks,now,cfg,horizon) or at_target,tracks,now,cfg,free)
    near=conflicts(own_p,tracks,now,cfg,horizon)
    return keep_clear_order(own_p,near,tracks,now,cfg,free) if near else None

def keep_clear_order(own_p,near,tracks,now,cfg,free):
    point,miss=keep_clear(own_p,[(tr.predict(now),tr.v) for tr in tracks],cfg,free)
    return {'action':'keep_clear','target':point,'miss':miss,'until':now+max(t for _,t,_ in near)+cfg.clear_time/2,
            't_cpa':min(t for _,t,_ in near),'tracks':[tr.id for tr,_,_ in near]}

def onboard_decide(now,own_p,tracks,link_age,order,cfg,free=lambda p:True,rally=None,previous=None):
    """What the guardian does itself: (action, target, layer, detail).

    It acts alone only for a conflict inside the reflex horizon or a lost uplink; otherwise it follows
    the station's last order. `previous` is its own last onboard keep-clear, if any."""
    o=keep_clear_step(own_p,previous,tracks,now,cfg,cfg.reflex_horizon,free)
    if o:return 'keep_clear',o['target'],'onboard',o
    if link_age>cfg.lost_link_return and rally is not None:return 'lost_link_return',list(rally),'onboard',{}
    if link_age>cfg.lost_link:return 'lost_link_hold',None,'onboard',{}
    return order.get('action','watch'),order.get('target'),'station',{}

def station_orders(now,posture,tracks,guardians,cfg,free_for=lambda name:(lambda p:True),recovery=None,
                   hazard=None,held=False,previous=None):
    """Orders for every guardian: {name: {'action', 'target', 'reason'}}.

    guardians: {name: {'p': reported position, 'post': watch post}}. `recovery` is 'recover' or 'resume'
    once the center (or delegation) has decided; `held` keeps guardians where they are after a RED event
    until then. `hazard` is (centre, radius) of a predicted impact area to disperse from."""
    orders={};previous=previous or {}
    for name,g in guardians.items():
        o=keep_clear_step(g['p'],previous.get(name),tracks,now,cfg,cfg.order_horizon,free_for(name))
        if o:
            orders[name]=o if o is previous.get(name) else {**o,'reason':'predicted_conflict'}
        elif posture==RED and hazard and math.dist(g['p'][:2],hazard[0][:2])<hazard[1]:
            c,r=hazard;dx,dy=g['p'][0]-c[0],g['p'][1]-c[1];n=math.hypot(dx,dy) or 1.
            orders[name]={'action':'disperse','target':[c[0]+dx/n*r*1.2,c[1]+dy/n*r*1.2,g['p'][2]],'reason':'impact_area'}
        elif recovery and posture==GREEN:
            orders[name]={'action':recovery,'target':g['post'] if recovery=='resume' else None,'reason':'center_decision'}
        elif posture==GREEN and not held:orders[name]={'action':'watch','target':g['post'],'reason':'routine'}
        else:orders[name]={'action':'hold','target':None,'reason':'posture_'+posture.lower()}
    return orders

class Center:
    """Remote command and control: reports arrive after a link delay; answers follow an operator decision.

    reachable(now) is False while the station-center link is down; nothing crosses it then."""
    def __init__(self,latency,decision_time,reachable=lambda now:True,after_red='recover'):
        self.latency=latency;self.decision_time=decision_time;self.reachable=reachable;self.after_red=after_red
        self.inbox=[];self.outbox=[];self.log=[]
    def send(self,now,report):
        if self.reachable(now):self.inbox.append((now+self.latency,report))
    def step(self,now):
        """Deliver due reports to the operator and due decisions to the station; returns decisions."""
        for item in [i for i in self.inbox if i[0]<=now]:
            self.inbox.remove(item);arrived,report=item
            decision={'posture':'acknowledge','clear_after_red':self.after_red}.get(report['kind'])
            if decision:self.outbox.append((arrived+self.decision_time,decision,report))
        delivered=[]
        for item in [o for o in self.outbox if o[0]<=now]:
            self.outbox.remove(item);ready,decision,report=item
            self.log.append({'t':ready,'decision':decision,'report':report['kind']})
            if self.reachable(ready):delivered.append((ready+self.latency,decision))
            else:self.outbox.append((ready+1.,decision,report))  # Retry until the link returns.
        return delivered

class NavIntegrity:
    """Compares a guardian's reported (GNSS) position with an independent observation, such as the station's
    own sensor track of it. A slow drag-off can pass the vehicle's own consistency checks; this cannot."""
    def __init__(self,cfg):self.cfg=cfg;self.count=0;self.state='ok';self.since=None;self.residual=0.
    def update(self,now,reported,observed):
        if observed is None:return self.state
        self.residual=math.dist(reported[:2],observed[:2])
        self.count=self.count+1 if self.residual>self.cfg.integrity_threshold else 0
        if self.state=='ok' and self.count>=self.cfg.integrity_persistence:self.state='spoofed';self.since=now
        return self.state

def authorised(action,layer):return layer in AUTHORITY.get(action,set())
