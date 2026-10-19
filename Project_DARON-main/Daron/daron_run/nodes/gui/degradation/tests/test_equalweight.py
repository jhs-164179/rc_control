import copy
import unittest
from test_rank import observation, rows, REF, P
from crater_rank.equalweight import evaluate, index
from crater_rank.core import aggregate
from integration.record_adapter import make_observation


class FixedWeightTests(unittest.TestCase):
    def test_fixed_formula_and_missing(self):
        r=index(30,.8,REF)
        self.assertAlmostEqual(r['index'],22.5)
        self.assertAlmostEqual(r['slope_contribution']+r['rim_contribution'],r['index'])
        for val in (None,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError): index(val,.8,REF)
    def test_unverified_only_preview_never_ranked(self):
        data=[dict(o,acquisition_verified=False) for o in rows()]
        before=copy.deepcopy(data)
        result=evaluate(data,REF,P)
        self.assertFalse(result['ranked'])
        self.assertEqual(len(result['preview']),1)
        self.assertIn('acquisition_unverified',result['preview'][0]['reasons'])
        self.assertEqual(data,before)
    def test_weak_geometry_not_previewed(self):
        data=[dict(o,geometry_flags=['weak_height_signal_relative_to_ground_residual']) for o in rows()]
        result=evaluate(data,REF,P)
        self.assertFalse(result['ranked']);self.assertFalse(result['preview'])
        self.assertIn('geometry_weak_height_signal_relative_to_ground_residual',result['audit'][0]['gate_rejections'])
    def test_short_clip_not_confident_zero_width_interval(self):
        result=evaluate(rows()[:1],REF,P)
        self.assertFalse(result['ranked'])
        self.assertNotIn('repeatability_interval95',result['preview'][0])
    def test_shuffle_duplicate_and_frozen_reference(self):
        data=rows('A',30,.8)+rows('B',10,.2)
        a=evaluate(data,REF,P);b=evaluate(list(reversed(data))*2,REF,P)
        self.assertEqual(a,b)
        self.assertEqual(a['weights'],{'observed_wall_p95_deg':.5,'image_rim_S':.5})
    def test_unknown_window_definition_and_flags_rejected(self):
        o=observation();o['quality'].pop('wall_windows_definition');o.pop('geometry_flags')
        a=aggregate([o],P)['audit'][0]['gate_rejections']
        self.assertIn('unverified_positive_window_count',a)
        self.assertIn('missing_geometry_flags',a)
    def test_adapter_counts_rising_only_and_invalid_rim(self):
        g={'slopes':{'windows':[{'rise_m':1},{'rise_m':-1},{'rise_m':0}],
                     'valid_sectors':[1],'angle_p95_deg':20},'quality_flags':[]}
        o=make_observation(mission_id='m',frame_id=1,crater_id='c',timestamp_s=0,
                           protocol_id='p',comparison_group='g',view_cell_id='v',support_id='s',
                           geometry=g,rim={'status':'rejected','rim_sharpness_raw':1},quality={})
        self.assertEqual(o['quality']['wall_windows'],1)
        self.assertIsNone(o['rim_score'])


if __name__=='__main__': unittest.main()
