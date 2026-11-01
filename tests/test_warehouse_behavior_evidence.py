"""Validation of retained execution evidence; enable with explicit experiment path."""
import hashlib,importlib.util,json,math,os,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];EXPERIMENT=os.environ.get('DYNAMUHSIM_BEHAVIOR_EVIDENCE')


def rows(path):return [json.loads(l) for l in path.read_text().splitlines()]


@unittest.skipUnless(EXPERIMENT,'Requires an explicit saved experiment; no synthetic acceptance')
class SavedEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.b=Path(EXPERIMENT)

    def test_recording_schema_identity_and_monotonic_sim_time(self):
        for name in ['baseline','candidate_v2']:
            data=rows(self.b/name/'human_states.jsonl');times={i:[] for i in range(1,7)}
            for r in data:
                self.assertTrue({'time','id','x','y','yaw','vx','vy','speed','group_id','goal','behavior_state','post_orca_velocity','post_projection_velocity','executed_displacement','constraints'}<=set(r));self.assertIn(r['id'],times);times[r['id']].append(r['time'])
            self.assertTrue(all(all(b>a for a,b in zip(ts,ts[1:])) for ts in times.values()));self.assertEqual(len({len(ts) for ts in times.values()}),1)

    def test_controller_reexecution_reproduces_native_states_and_goal_events(self):
        for filename in ['baseline_final.receipt.json','candidate_v2_native.receipt.json']:
            r=json.loads((self.b/'replay'/filename).read_text());self.assertEqual(r['baseline_max_xy_error_m'],0.);self.assertEqual(r['baseline_max_yaw_error_rad'],0.);self.assertTrue(r['baseline_goal_clock_events_equal']);self.assertEqual(hashlib.sha256(Path(r['source']).read_bytes()).hexdigest(),r['source_sha256'])

    def test_matched_input_and_time_alignment(self):
        a=rows(self.b/'baseline/controller.jsonl');b=rows(self.b/'candidate_v2/controller.jsonl');self.assertEqual(len(a),len(b));self.assertEqual(a[0]['before'],b[0]['before'])
        for r,s in zip(a,b):self.assertEqual((r['time'],r['state_time'],r['robot_input']),(s['time'],s['state_time'],s['robot_input']))
        self.assertEqual((self.b/'baseline/config.json').read_bytes(),(self.b/'candidate_v2/config.json').read_bytes())

    def test_saved_pose_timeline_matches_gazebo_ground_truth(self):
        for name in ['baseline','candidate_v2']:
            original={r['time']:r for r in rows(self.b/name/'native/episode/gazebo_poses.jsonl')}
            for r in rows(self.b/name/'pose_timeline.jsonl'):
                for h in r['humans']:
                    p=original[r['time']]['poses'][f"common_{h['id']}_proxy"]
                    for key in ['x','y','yaw']:self.assertEqual(h[key],p[key])

    def test_pose_playback_reconstruction_and_decodable_frames(self):
        from PIL import Image
        r=json.loads((self.b/'replay/playback_verification_v2.json').read_text())
        for v in r['reconstruction'].values():self.assertEqual(v['max_xy_error_m'],0.);self.assertEqual(v['max_yaw_error_rad'],0.)
        for v in r['media'].values():
            p=Path(v['path']);self.assertGreater(p.stat().st_size,0);self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),v['sha256'])
            with Image.open(p) as im:
                self.assertEqual(im.n_frames,v['frames'])
                for i in range(im.n_frames):im.seek(i);im.load()

    def test_adapter_interpolation_alignment_against_observed_pose(self):
        for name in ['baseline','candidate_v2']:
            r=json.loads((self.b/'replay'/f'{name}_adapter_execution_alignment.json').read_text());self.assertLess(r['max_xy_error_m'],1e-8);self.assertLess(r['max_yaw_error_rad'],1e-8);self.assertGreater(r['human_samples'],1000)

    def test_recompute_metrics_from_saved_raw_controller_outputs(self):
        spec=importlib.util.spec_from_file_location('_saved_behavior_evidence',ROOT/'tools/warehouse_human_evidence.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        for name in ['baseline','candidate_v2']:
            path=self.b/name/'controller.jsonl';digest=hashlib.sha256(path.read_bytes()).hexdigest();actual=mod.measure(mod.controller_timeline(path),ROOT,path,'DETERMINISTIC_CONTROLLER_REEXECUTION');saved=json.loads((self.b/'comparison_final'/f'{name}_controller_metrics.json').read_text())
            for key in ['path_length_m','goal_progress_same_target_m','total_absolute_yaw_rad','yaw_sign_switches_gt_01radps','stop_duration_person_s']:self.assertAlmostEqual(actual[key],saved[key],places=10)
            self.assertEqual(actual['linear_sample_swept_clearance_m'],saved['linear_sample_swept_clearance_m']);self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),digest)

if __name__=='__main__':unittest.main()
