"""Regression checks for safe reruns and the Gazebo/map/TF synchronization contract."""
import copy
import importlib.util
import io
import json
from contextlib import redirect_stderr
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from dynamuhsim.simulation.warehouse_sync import check_map_world, configure_navigation, synchronized_pose_sample


class SynchronizationTests(unittest.TestCase):
    def test_map_matches_native_geometry_and_wrong_origin_is_rejected(self):
        src = ROOT / 'experiments/social_navigation/warehouse'
        receipt = check_map_world(src/'warehouse.sdf', src/'warehouse.yaml', src/'layout.json')
        self.assertEqual(receipt['occupied_cell_mismatches'], 0)
        self.assertEqual(receipt['static_obstacles'], 42)
        with tempfile.TemporaryDirectory() as d:
            out = Path(d)
            shutil.copyfile(src/'warehouse.pgm', out/'warehouse.pgm')
            meta = yaml.safe_load((src/'warehouse.yaml').read_text())
            meta['origin'][0] += .1
            (out/'wrong.yaml').write_text(yaml.safe_dump(meta))
            with self.assertRaisesRegex(ValueError, 'raster mismatch'):
                check_map_world(src/'warehouse.sdf', out/'wrong.yaml', src/'layout.json')

    def test_localization_profile_preserves_every_mppi_parameter(self):
        config = yaml.safe_load((ROOT/'experiments/social_navigation/warehouse/nav2_warehouse.yaml').read_text())
        before = copy.deepcopy(config)
        actual = configure_navigation(config)
        self.assertEqual(config, before)
        self.assertEqual(actual['controller_server']['ros__parameters']['FollowPath'], before['controller_server']['ros__parameters']['FollowPath'])
        self.assertFalse(actual['controller_server']['ros__parameters']['goal_checker']['stateful'])
        self.assertLess(actual['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'], .2)
        self.assertEqual(actual['amcl']['ros__parameters']['robot_model_type'], 'nav2_amcl::OmniMotionModel')
        self.assertTrue(actual['amcl']['ros__parameters']['do_beamskip'])

    def test_tf_comparison_requests_scene_time_instead_of_latest(self):
        calls = []
        def lookup(parent, child, timestamp):
            calls.append((parent, child, timestamp))
            return NS(transform=NS(translation=NS(x=1., y=2.), rotation=None))
        scene = dict(time=2.5, poses={'mecanum_robot':dict(x=1., y=2., yaw=0.)})
        rclpy = NS(time=NS(Time=lambda **kw: kw))
        result = synchronized_pose_sample(NS(lookup_transform=lookup), rclpy, scene, lambda rotation:0.)
        self.assertEqual([c[2] for c in calls], [dict(nanoseconds=2500000000)]*2)
        self.assertEqual(result['xy_error_m'], 0.)

    def test_cli_existing_capture_never_overwrites_or_starts_ros(self):
        spec = importlib.util.spec_from_file_location('capture_cli_test', ROOT/'tools/capture_episode.py')
        cli = importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        with tempfile.TemporaryDirectory() as d:
            run = Path(d);(run/'episode').mkdir();sentinel=run/'keep.log';sentinel.write_text('original')
            for args in [['prepare','--run-dir',str(run),'--scenario','crossing','--seed','42'], ['run','--run-dir',str(run)]]:
                errors=io.StringIO()
                with patch.object(cli,'execute_capture') as execute, redirect_stderr(errors), self.assertRaises(SystemExit) as stopped:
                    cli.main(args)
                self.assertEqual(stopped.exception.code,2)
                self.assertNotIn('Traceback',errors.getvalue())
                execute.assert_not_called()
            self.assertEqual(sentinel.read_text(),'original')
            self.assertNotEqual(cli.fresh_run_directory(run),cli.fresh_run_directory(run))

    def test_capture_kernel_restores_exact_locked_bookkeeping_only(self):
        spec=importlib.util.spec_from_file_location('native_capture_builder',ROOT/'experiments/social_navigation/navigation/shared_nav2_pilot/runtime.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        import hashlib
        path=module.prepare_capture_kernel()
        expected=json.loads((module.COMMON/'inputs/crowd_bookkeeping_patch.json').read_text())
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),expected['generated_sha256'])

    def test_rviz_overlay_changes_only_copied_resources(self):
        from dynamuhsim.simulation.rviz_assets import prepare_rviz_overlay
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'system_share';shader=Path('ogre_media/materials/glsl120/indexed_8bit_image.frag')
            (source/shader).parent.mkdir(parents=True);(source/shader).write_text('original system shader')
            assets=Path(d)/'assets';assets.mkdir()
            package=NS(get_package_share_directory=lambda name:str(source))
            with patch.dict(sys.modules,{'ament_index_python.packages':package}):
                receipt=prepare_rviz_overlay(ROOT,assets)
            self.assertEqual((source/shader).read_text(),'original system shader')
            patched=(assets/'rviz_overlay/share/rviz_rendering'/shader).read_text()
            self.assertIn('layout(binding = 0)',patched);self.assertIn('layout(binding = 1)',patched)
            self.assertNotEqual(receipt['original_shader_sha256'],receipt['patched_shader_sha256'])

    def test_termination_enters_cleanup_and_restores_signal_handlers(self):
        import signal
        spec=importlib.util.spec_from_file_location('capture_signal_test',ROOT/'tools/capture_episode.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        before=signal.getsignal(signal.SIGTERM)
        context=NS(plan=dict(scenario='crossing',seed=42,gui=False),run_dir=Path('/tmp/unused'))
        def terminate(*args,**kwargs):
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM,None)
        with patch.object(cli,'CaptureContext',return_value=context),patch.object(cli,'run_trial',side_effect=terminate):
            with self.assertRaisesRegex(RuntimeError,'SIGTERM'):
                cli.execute_capture(Path('/tmp/unused'))
        self.assertEqual(signal.getsignal(signal.SIGTERM),before)


if __name__ == '__main__':
    unittest.main()
