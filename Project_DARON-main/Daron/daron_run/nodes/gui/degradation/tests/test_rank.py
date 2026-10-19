import copy, json, math, tempfile, unittest
from pathlib import Path
from dataclasses import replace
from crater_rank import Policy,Reference,aggregate,calibrate,rank,ObservationStore,CandidateQueue
from crater_rank.calibration import fit_weight


def observation(cid='A',t=0,slope=20,rim=.5,**extra):
    o=dict(mission_id='test',observation_id=cid+':'+str(t),crater_id=cid,
           timestamp_s=t,identity_confirmed=True,acquisition_verified=True,
           protocol_id='p',comparison_group='g',view_cell='v',support_id='s',
           slope_source='observed_depth',slope_deg=slope,rim_score=rim,
           geometry_flags=[],rim_status='valid',
           quality=dict(bbox_min_side_px=80,depth_valid_fraction=.95,wall_sectors=6,
                        wall_windows=18,sync_ms=1,background_delta_deg=1,
                        saturation_fraction=0,bbox_truncated=False,
                        wall_windows_definition='positive_rising_windows'))
    o.update(extra);return o


def rows(cid='A',slope=20,rim=.5):
    return [observation(cid,t,slope,rim) for t in (0,5,10,15,20,25)]


P=Policy(bootstrap_replicates=80)
REF=Reference('r',('p','g','v','s'),0,40,0,1,'test-only')


class RankingTests(unittest.TestCase):
    def group(self,obs,policy=P):return aggregate(obs,policy)['groups'][0]
    def test_direction_and_numeric_magnitude(self):
        result=rank(self.group(rows('A',30,.8)+rows('B',10,.2)),REF,P)
        self.assertEqual([x['crater_id'] for x in result['ranked']],['A','B'])
        self.assertAlmostEqual(result['ranked'][0]['index'],22.5)
        self.assertEqual(result['pairwise'][0]['decision'],'stable_across_weights_and_resampling')
    def test_missing_not_zero(self):
        a=aggregate(rows(slope=None),P)
        self.assertFalse(a['groups']);self.assertIn('missing_or_nonfinite_slope_deg',a['audit'][0]['gate_rejections'])
    def test_missing_acquisition(self):
        self.assertFalse(aggregate([observation(acquisition_verified=False)],P)['groups'])
    def test_duplicate_invariance(self):
        self.assertEqual(aggregate(rows(),P),aggregate(rows()*20,P))
    def test_conflicting_duplicate(self):
        with self.assertRaises(ValueError):aggregate([observation(),observation(slope=30)],P)
    def test_repeated_frames_one_bucket(self):
        dense=rows()+[observation(t=i/1000,slope=20,rim=.5) for i in range(1,1000)]
        a=rank(self.group(rows()),REF,P);b=rank(self.group(dense),REF,P)
        self.assertEqual(a['ranked'][0]['index'],b['ranked'][0]['index'])
        self.assertEqual(b['ranked'][0]['blocks'],6)
    def test_no_max_frame_cherry_pick(self):
        g=self.group(rows()+[observation(t=.1,slope=80,rim=100)])
        self.assertEqual(g['craters'][0]['slope_deg'],20)
    def test_short_clip_pending(self):
        self.assertEqual(self.group(rows()[:2])['craters'][0]['status'],'pending')
    def test_protocol_split(self):
        data=rows()+[dict(o,crater_id='B',observation_id='B'+o['observation_id'],view_cell='v2') for o in rows()]
        self.assertEqual(len(aggregate(data,P)['groups']),2)
    def test_reference_mismatch(self):
        g=self.group(rows());g['protocol_key'][0]='another'
        with self.assertRaises(ValueError):rank(g,REF,P)
    def test_constant_calibration_rejected(self):
        g=self.group(sum([rows(c) for c in 'ABCD'],[]))
        with self.assertRaises(ValueError):calibrate(g,'r','test',P)
    def test_fixed_reference_no_new_peer_drift(self):
        a=rank(self.group(rows()),REF,P)['ranked'][0]
        b=rank(self.group(rows()+rows('B',80,2)),REF,P)
        aa=next(r for r in b['ranked'] if r['crater_id']=='A')
        self.assertEqual(a['index'],aa['index']);self.assertEqual(a['repeatability_interval95'],aa['repeatability_interval95'])
        self.assertTrue(next(r for r in b['ranked'] if r['crater_id']=='B')['reference_extrapolation'])
    def test_weight_crossover(self):
        a=rank(self.group(rows('A',32,.2)+rows('B',8,.8)),REF,P)
        self.assertAlmostEqual(a['pairwise'][0]['weight_crossover'],.5)
        self.assertIsNone(a['pairwise'][0]['robust_better'])
    def test_ties_same_rank(self):
        a=rank(self.group(rows('A')+rows('B')),REF,P)
        self.assertEqual([r['rank'] for r in a['ranked']],[1,1])
        self.assertEqual(a['pairwise'][0]['p_a_better_at_selected_weight'],.5)
    def test_unstable_slope_pending(self):
        a=self.group([observation(t=i*5,slope=10+i*7) for i in range(6)])
        self.assertIn('unstable_observed_slope',a['craters'][0]['reasons'])
    def test_far_not_rejected_by_range(self):
        a=observation(range_m=5)
        self.assertTrue(aggregate([a],P)['groups'])
    def test_aliases_and_cycle(self):
        a=aggregate(rows('A')+rows('B'),P,{'B':'A'})
        self.assertEqual(len(a['groups'][0]['craters']),1)
        with self.assertRaises(ValueError):aggregate(rows(),P,{'A':'B','B':'A'})
    def test_missing_quality_and_mesh(self):
        a=observation(slope_source='mesh');a['quality'].pop('background_delta_deg')
        b=aggregate([a],P)['audit'][0]['gate_rejections']
        self.assertIn('slope_must_be_observed_depth',b);self.assertIn('missing_quality_background_delta_deg',b)
    def test_revisit_bootstrap_no_crash(self):
        a=self.group([observation(t=i*100) for i in range(6)])
        self.assertEqual(rank(a,REF,P)['ranked'][0]['repeatability_interval95'],[50.,50.])
    def test_store_restart_deduplicate(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as d:
            path=Path(d)/'obs.db';s=ObservationStore(path)
            self.assertTrue(s.add(observation()));s.close();s=ObservationStore(path)
            self.assertFalse(s.add(observation()))
            with self.assertRaises(ValueError):s.add(observation(slope=30))
            self.assertEqual(len(s.observations()),1);s.close()
    def test_queue_bounded_and_fair(self):
        q=CandidateQueue(max_pending=2)
        self.assertTrue(q.offer('A',0,'a',.5));self.assertTrue(q.offer('B',0,'b',.5))
        self.assertFalse(q.offer('C',0,'c',1));self.assertTrue(q.offer('A',1,'aa',.8))
        self.assertEqual(q.pop()['crater_id'],'B');self.assertEqual(q.pop()['token'],'aa')
        self.assertFalse(q.offer('A',2,'a2',1));self.assertTrue(q.offer('A',6,'a6',1))
    def test_invalid_config(self):
        for kw in ({'block_seconds':0},{'min_blocks':1.5},{'max_sync_ms':-1}):
            with self.assertRaises(ValueError):Policy(**kw)
        with self.assertRaises(ValueError):rank(self.group(rows()),REF,P,float('nan'))
    def test_weight_fit_uses_external_labels(self):
        features={str(i):[i/7,1-i/7] for i in range(8)}
        labels=[dict(a=str(i),b=str(j),better=str(j)) for i in range(8) for j in range(i+1,8)]
        r=fit_weight(features,labels)
        self.assertGreater(r['candidate_slope_weight'],.5)
        self.assertEqual(r['mean_crater_held_out_agreement'],1)
        self.assertFalse(r['automatically_approved'])


if __name__=='__main__':unittest.main()
