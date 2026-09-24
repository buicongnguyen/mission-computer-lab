from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import sqlite3
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from evidence_contracts import require_matrix,read_records,flight_cycle_checks,bag_has_topics,strict_loads
from check_artifacts import validate,SCENARIOS,SECURITY,COMMON_CHECKS,COMPLETING,BOOT

class EvidenceTests(unittest.TestCase):
    def test_empty_or_partial_rosbag_does_not_prove_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);(folder/'metadata.yaml').write_text('test')
            with sqlite3.connect(folder/'test.db3') as db:
                db.execute('CREATE TABLE topics (id INTEGER, name TEXT)')
                db.execute('CREATE TABLE messages (topic_id INTEGER)')
                for i,name in enumerate(['/mission/decision','/mission/perception','/mission/lidar/scan']):
                    db.execute('INSERT INTO topics VALUES (?, ?)',(i,name))
                db.commit()
                self.assertFalse(bag_has_topics(folder))
                db.executemany('INSERT INTO messages VALUES (?)',[(0,),(1,)]);db.commit()
                self.assertFalse(bag_has_topics(folder))
                db.execute('INSERT INTO messages VALUES (2)');db.commit()
                self.assertTrue(bag_has_topics(folder))
    def test_duplicate_empty_or_truthy_checks_are_rejected(self):
        valid=[{'scenario':s,'passed':True,'checks':{'test':True}} for s in ('a','b')]
        require_matrix(valid,{'a','b'})
        for bad in [valid+[valid[0]],[valid[0],valid[0]],
                    [dict(valid[0],checks={}),valid[1]],
                    [dict(valid[0],checks={'test':1}),valid[1]]]:
            with self.assertRaises(ValueError):require_matrix(bad,{'a','b'})
    def test_only_unfinished_live_tail_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'log.jsonl'
            path.write_text('{"kind":"ok"}\n{"ki')
            self.assertEqual(len(read_records(path,live=True)),1)
            with self.assertRaises(ValueError):read_records(path)
            path.write_text('invalid\n{"kind":"ok"}\n')
            with self.assertRaises(ValueError):read_records(path,live=True)
    def test_disconnected_ground_flags_cannot_prove_landing(self):
        records=[{'kind':'land','wall_time':0.,'landed':True},
            {'kind':'status','wall_time':1.,'arming_state':2,'nav_state':14},
            {'kind':'position','wall_time':2.,'valid':True,'ned':[0,0,-3]},
            {'kind':'status','wall_time':3.,'arming_state':2,'nav_state':18},
            {'kind':'status','wall_time':5.,'arming_state':1,'nav_state':18}]
        self.assertFalse(flight_cycle_checks(records)['ordered_flight_cycle'])
        records.insert(-1,{'kind':'land','wall_time':4.,'landed':True})
        records.append({'kind':'position','wall_time':5.1,'valid':True,'ned':[0,0,0]})
        self.assertTrue(all(flight_cycle_checks(records).values()))
        records[-1].update(valid=False,z_valid=True)
        self.assertFalse(flight_cycle_checks(records)['estimator_valid_before_fault'])
        self.assertTrue(all(flight_cycle_checks(records,gps_loss_at=2.5).values()))
        records[2]['valid']=False
        self.assertFalse(flight_cycle_checks(records,gps_loss_at=2.5)['estimator_valid_before_fault'])
    def test_invalid_security_or_nan_trace_rejected(self):
        terminal=lambda s:'COMPLETE' if s in COMPLETING else 'LAND'
        report={'schema_version':1,'boot':[{'stage':stage,'status':status} for stage,status in BOOT],
            'results':[{'scenario':s,'passed':True,'checks':dict.fromkeys(COMMON_CHECKS|{'hold_then_recover','hold_then_land','independent_watchdog'},True),
            'trace':[{'time':0.},{'time':1.}],'events':[{'mode':'ACTIVE'},{'mode':'REPLAN'},{'mode':terminal(s)}],
            'summary':{'terminal_mode':terminal(s),'min_obstacle_clearance_m':1.2,'max_localization_error_m':0.1}} for s in SCENARIOS],
            'security':[{'case':s,'passed':True,'accepted':s=='valid_update','expected':s=='valid_update'} for s in SECURITY],
            'environment':{'providers':['CPUExecutionProvider']}}
        validate(report)
        bad=deepcopy(report);del bad['results'][0]['checks']['terminated_before_timeout']
        with self.assertRaises(ValueError):validate(bad)
        bad=deepcopy(report);bad['results'][0]['trace'][0]['time']=float('nan')
        with self.assertRaises(ValueError):validate(bad)
        bad=deepcopy(report);bad['security'][0]['accepted']=not bad['security'][0]['accepted']
        with self.assertRaises(ValueError):validate(bad)
    def test_pass_flags_are_cross_checked_against_recorded_data(self):
        # Each mutated report below still claims every check passed.
        base={'schema_version':1,'boot':[{'stage':stage,'status':status} for stage,status in BOOT],
            'results':[{'scenario':s,'passed':True,'checks':dict.fromkeys(COMMON_CHECKS|{'hold_then_recover','hold_then_land','independent_watchdog'},True),
            'trace':[{'time':0.}],'events':[{'mode':'COMPLETE' if s in COMPLETING else 'LAND'}],
            'summary':{'terminal_mode':'COMPLETE' if s in COMPLETING else 'LAND','min_obstacle_clearance_m':1.,'max_localization_error_m':.1}} for s in SCENARIOS],
            'security':[{'case':s,'passed':True,'accepted':s=='valid_update','expected':s=='valid_update'} for s in SECURITY],
            'environment':{'providers':['CPUExecutionProvider']}}
        validate(base)
        for mutate in (lambda r:r.update(schema_version=True),
                       lambda r:r['boot'][2].update(status='FAILED'),
                       lambda r:r['results'][0]['summary'].update(terminal_mode='HOLD'),
                       lambda r:r['results'][0]['events'].append({'mode':'HOLD'}),
                       lambda r:r['results'][0]['summary'].update(min_obstacle_clearance_m=-3),
                       lambda r:r['results'][0]['summary'].update(max_localization_error_m=float('nan'))):
            bad=deepcopy(base);mutate(bad)
            with self.assertRaises(ValueError):validate(bad)
    def test_overflowing_numbers_are_not_accepted_as_finite(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'log.jsonl'
            path.write_text('{"kind":"inference","inference_ms":1e999}\n')
            with self.assertRaisesRegex(ValueError,'Nonfinite'):read_records(path)
        with self.assertRaisesRegex(ValueError,'Nonfinite'):strict_loads('[-1e400]')
        self.assertEqual(strict_loads('{"a":1.5}'),{'a':1.5})

if __name__=='__main__':unittest.main()
