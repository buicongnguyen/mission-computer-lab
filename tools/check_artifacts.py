"""Fail CI on incomplete evidence, including when Python optimization is enabled."""
import json
import math
from pathlib import Path
import sys
from evidence_contracts import require_matrix

SCENARIOS={'nominal','camera_dropout','gps_dropout','link_dropout','inference_overrun',
           'low_battery','imu_dropout','companion_crash'}
SECURITY={'valid_update','tampered_payload','rollback','wrong_device','wrong_signer','modified_manifest'}
COMMON_CHECKS={'terminated_before_timeout','expected_terminal_mode','no_obstacle_collision','bounded_estimate_error'}

def validate(r):
    if r.get('schema_version')!=1:raise ValueError('Unsupported evidence schema')
    require_matrix(r['results'],SCENARIOS)
    cases=r['security']
    if len(cases)!=len(SECURITY) or {c['case'] for c in cases}!=SECURITY:
        raise ValueError('Exactly six distinct security cases required')
    for case in cases:
        expected=case['case']=='valid_update'
        if case.get('passed') is not True or case.get('accepted') is not expected or case.get('expected') is not expected:
            raise ValueError('Security case did not produce its required outcome')
    for scenario in r['results']:
        required=set(COMMON_CHECKS);name=scenario['scenario']
        if name in ('camera_dropout','inference_overrun'):required.add('hold_then_recover')
        if name in ('gps_dropout','link_dropout','imu_dropout'):required.add('hold_then_land')
        if name=='companion_crash':required.add('independent_watchdog')
        if not required<=scenario['checks'].keys():raise ValueError('Missing required acceptance checks: '+name)
        times=[f['time'] for f in scenario['trace']]
        if not times or not all(math.isfinite(t) for t in times) or not all(a<b for a,b in zip(times,times[1:])):
            raise ValueError('Nonempty finite strictly increasing trace required')
        if not scenario['events']:raise ValueError('Missing mission events')
    if r['environment']['providers']!=['CPUExecutionProvider']:
        raise ValueError('This reference requires the CPU execution provider')

def check(path):
    validate(json.loads((path/'report.json').read_text()))
    print('Evidence checks passed: eight distinct scenarios and six distinct security cases')

if __name__=='__main__': check(Path(sys.argv[1]))
