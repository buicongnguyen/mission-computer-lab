"""Acceptance checks that remain active under python -O."""
from contextlib import closing
import json
import math
import sqlite3


def _reject_constant(value):
    raise ValueError('Nonfinite JSON: '+value)


def _finite_float(text):
    # parse_constant only sees NaN/Infinity literals; 1e999 overflows to inf through parse_float.
    value=float(text)
    if not math.isfinite(value):raise ValueError('Nonfinite JSON: '+text)
    return value


def strict_loads(text):
    return json.loads(text,parse_constant=_reject_constant,parse_float=_finite_float)


def read_records(path, live=False):
    if not path.exists():return []
    lines=path.read_text(encoding='utf-8').splitlines(keepends=True)
    if live and lines and not lines[-1].endswith('\n'):lines.pop()
    records=[]
    for line in lines:
        item=strict_loads(line)
        if not isinstance(item,dict):raise ValueError('JSONL record must be an object')
        records.append(item)
    return records


def require_matrix(results, expected):
    names=[r['scenario'] for r in results]
    if len(names)!=len(expected) or set(names)!=set(expected):
        raise ValueError('Exactly one result per required scenario is required')
    for result in results:
        if result.get('passed') is not True or not result.get('checks') or not all(v is True for v in result['checks'].values()):
            raise ValueError('All named checks must explicitly pass: '+result['scenario'])


def flight_cycle_checks(records,gps_loss_at=None):
    """Require ordered independent observations, not disconnected flags."""
    statuses=[r for r in records if r['kind']=='status']
    positions=[r for r in records if r['kind']=='position']
    lands=[r for r in records if r['kind']=='land']
    offboard=next((r['wall_time'] for r in statuses if r['arming_state']==2 and r['nav_state']==14),math.inf)
    airborne=next((r['wall_time'] for r in positions if r['wall_time']>=offboard and r['valid'] and -r['ned'][2]>2),math.inf)
    landing=next((r['wall_time'] for r in statuses if r['wall_time']>airborne and r['arming_state']==2 and r['nav_state']==18),math.inf)
    touchdown=next((r['wall_time'] for r in lands if r['wall_time']>landing and r['landed']),math.inf)
    disarmed=statuses[-1] if statuses else {}
    ordered=bool(disarmed.get('arming_state')==1 and disarmed['wall_time']>=touchdown and math.isfinite(touchdown))
    return {'ordered_flight_cycle':ordered,
            'landed_at_end':bool(lands and lands[-1]['landed'] and math.isfinite(touchdown)),
            'final_pose_near_ground':bool(positions and positions[-1].get('z_valid',positions[-1]['valid']) and
                positions[-1]['wall_time']>=landing and abs(positions[-1]['ned'][2])<0.5),
            'finite_positions':bool(positions and all(all(math.isfinite(v) for v in p['ned']) for p in positions)),
            'estimator_valid_before_fault':bool(positions and all(p['valid'] or
                (gps_loss_at is not None and p['wall_time']>=gps_loss_at) for p in positions))}


def bag_has_topics(folder):
    if not (folder/'metadata.yaml').is_file():return False
    seen=set()
    for database in folder.glob('*.db3'):
        # sqlite3's own context manager only ends a transaction; closing() releases the file.
        with closing(sqlite3.connect(database.as_uri()+'?mode=ro',uri=True)) as connection:
            seen.update(row[0] for row in connection.execute(
                'SELECT DISTINCT topics.name FROM topics JOIN messages ON messages.topic_id=topics.id'))
    return {'/mission/decision','/mission/perception','/mission/lidar/scan'}<=seen
