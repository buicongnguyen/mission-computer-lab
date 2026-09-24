"""Planar virtual LiDAR, grid planning, and a small position/velocity Kalman filter."""
import heapq
import math
import numpy as np

OBSTACLES = [(4.0,3.0,1.0), (6.0,6.0,1.1), (2.0,7.0,0.8)]
GOAL = (9,9)

def lidar(position, obstacles=OBSTACLES, rays=72, max_range=14):
    hits=[]
    px,py=position[:2]
    for i in range(rays):
        angle=2*math.pi*i/rays
        dx,dy=math.cos(angle),math.sin(angle)
        distance=max_range
        hit=False
        for cx,cy,r in obstacles:
            ox,oy=px-cx,py-cy
            b=ox*dx+oy*dy
            disc=b*b-(ox*ox+oy*oy-r*r)
            if disc>=0:
                near=-b-math.sqrt(disc)
                if 0<=near<distance: distance,hit=near,True
        hits.append({'x':px+dx*distance,'y':py+dy*distance,'range':distance,'hit':hit})
    return hits

def occupancy(scan, inflation=1.0, blocked=None):
    # One scan sees only unoccluded surfaces (the (6,6) cylinder is mostly hidden from the
    # origin). Callers keep adding scans as the vehicle moves and replan when a route closes.
    blocked=set() if blocked is None else blocked
    for point in scan:
        if point['hit']:
            px,py=point['x'],point['y']
            for x in range(max(-2,math.floor(px-inflation)),min(12,math.ceil(px+inflation))+1):
                for y in range(max(-2,math.floor(py-inflation)),min(12,math.ceil(py+inflation))+1):
                    if math.hypot(x-px,y-py) <= inflation:
                        blocked.add((x,y))
    return blocked

def route_blocked(cells, blocked):
    return any((round(x),round(y)) in blocked for x,y,*_ in cells)

def replan(position, goal, blocked, radius=2):
    """Plan from the nearest free cell to the current position; [] when no route exists."""
    px,py=position[:2]
    free=[(math.hypot(x-px,y-py),(x,y)) for x in range(round(px)-radius,round(px)+radius+1)
          for y in range(round(py)-radius,round(py)+radius+1)
          if -2<=x<=12 and -2<=y<=12 and (x,y) not in blocked]
    return astar(min(free)[1],goal,blocked) if free else []

def astar(start, goal, blocked):
    if start in blocked or goal in blocked: return []
    frontier=[(0,0,start)]
    parent={start:None}; cost={start:0}
    while frontier:
        _,g,node=heapq.heappop(frontier)
        if g != cost[node]: continue
        if node==goal:
            result=[]
            while node is not None: result.append(node); node=parent[node]
            return list(reversed(result))
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nxt=(node[0]+dx,node[1]+dy)
            if not(-2<=nxt[0]<=12 and -2<=nxt[1]<=12) or nxt in blocked: continue
            ng=g+1
            if ng<cost.get(nxt,math.inf):
                cost[nxt]=ng; parent[nxt]=node
                h=abs(goal[0]-nxt[0])+abs(goal[1]-nxt[1])
                heapq.heappush(frontier,(ng+h,ng,nxt))
    return []

class Localizer:
    def __init__(self):
        self.state=np.zeros(6)
        self.cov=np.eye(6)*0.05
    def step(self, acceleration, gnss, dt):
        f=np.eye(6); f[:3,3:]=np.eye(3)*dt
        b=np.vstack((np.eye(3)*dt*dt/2,np.eye(3)*dt))
        self.state=f@self.state+b@acceleration
        self.cov=f@self.cov@f.T+np.eye(6)*0.0005
        if gnss is not None:
            h=np.zeros((3,6)); h[:,:3]=np.eye(3)
            r=np.eye(3)*0.01
            k=self.cov@h.T@np.linalg.inv(h@self.cov@h.T+r)
            self.state+=k@(gnss-h@self.state)
            i=np.eye(6)-k@h
            self.cov=i@self.cov@i.T+k@r@k.T
        return self.state[:3].copy(), float(np.trace(self.cov[:3,:3]))
