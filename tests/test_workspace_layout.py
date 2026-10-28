"""Current capability paths, legacy receipts and strict live math bindings."""
import hashlib,importlib.util,json,os,tempfile,unittest
from pathlib import Path,PurePosixPath
from unittest.mock import patch
from dynamuhsim.workspace import STAGE_GROUPS,stage_directory,source_path,logical_source_name
from dynamuhsim.artifacts.layout import report_target,logical_report_name
from dynamuhsim.analysis.evidence import resolve_sources
ROOT=Path(__file__).resolve().parents[1]

class WorkspaceLayout(unittest.TestCase):
 def test_complete_stage_map_and_current_sources(self):
  self.assertEqual(len(STAGE_GROUPS),29);self.assertEqual(set(STAGE_GROUPS.values()),{'navigation','perception','datasets','replay'})
  for stage,group in STAGE_GROUPS.items():self.assertTrue((ROOT/'experiments/social_navigation'/group/stage).is_dir())
 def test_legacy_and_current_ids_choose_current_and_never_shadow_missing_math(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);old=root/'experiments/social_navigation/benchmark/wm_episode_batch';new=root/'experiments/social_navigation/datasets/wm_episode_batch';old.mkdir(parents=True);(old/'loader.py').write_text('legacy')
   name='experiments/social_navigation/benchmark/wm_episode_batch/loader.py';self.assertEqual(source_path(root,name),old/'loader.py');new.mkdir(parents=True)
   self.assertEqual(source_path(root,name),new/'loader.py');self.assertFalse(source_path(root,name).exists())
   self.assertEqual(logical_source_name('experiments/social_navigation/datasets/wm_episode_batch/loader.py'),name)
 def test_hash_resolver_uses_new_file_but_does_not_accept_wrong_bytes(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);path=root/'experiments/social_navigation/datasets/wm_episode_batch/loader.py';path.parent.mkdir(parents=True);path.write_bytes(b'current');name='experiments/social_navigation/benchmark/wm_episode_batch/loader.py';wanted=hashlib.sha256(b'current').hexdigest()
   found,missing=resolve_sources({name:wanted},roots=[root]);self.assertFalse(missing);self.assertEqual(found[name]['data'],b'current');path.write_bytes(b'changed');self.assertEqual(resolve_sources({name:wanted},roots=[root])[1],[name])
 def test_report_alias_resolves_both_ids_and_rejects_other_targets(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);stage=root/'experiments/social_navigation/datasets/wm_episode_batch';stage.mkdir(parents=True);archive=root/'experiments/archive/social_navigation/reports/wm_episode_batch';archive.mkdir(parents=True);alias=stage/'reports';alias.symlink_to(os.path.relpath(archive,stage),target_is_directory=True)
   old=PurePosixPath('experiments/social_navigation/benchmark/wm_episode_batch/reports/audit.json');current=PurePosixPath('experiments/social_navigation/datasets/wm_episode_batch/reports/audit.json')
   self.assertEqual(report_target(root,old),archive/'audit.json');self.assertEqual(report_target(root,current),archive/'audit.json');self.assertEqual(logical_report_name(str(current)),str(old));alias.unlink();alias.symlink_to('../outside')
   with self.assertRaises(ValueError):report_target(root,old)
 def test_history_hydration_without_current_stage_retains_legacy_location(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);name=PurePosixPath('experiments/social_navigation/benchmark/wm_episode_batch/reports/audit.json');self.assertEqual(report_target(root,name),root/name)
 def test_classifier_refuses_changed_live_math_and_changed_corpus(self):
  path=stage_directory(ROOT,'depth_person_classifier')/'integrity.py';spec=importlib.util.spec_from_file_location('layout_classifier_integrity',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);stage=root/'experiments/social_navigation/perception/depth_person_classifier';stage.mkdir(parents=True);math=stage/'model.py';math.write_bytes(b'original model');data=root/'corpus';data.mkdir();(data/'sample').write_bytes(b'sensor')
   frozen=dict(source_sha256={'experiments/social_navigation/benchmark/depth_person_classifier/model.py':hashlib.sha256(math.read_bytes()).hexdigest()},corpus_path='corpus',corpus_sha256={'sample':hashlib.sha256(b'sensor').hexdigest()});(stage/'frozen.json').write_text(json.dumps(frozen))
   with patch.object(module,'REPO',root),patch.object(module,'HERE',stage):
    module.verify();math.write_bytes(b'changed model')
    with self.assertRaisesRegex(AssertionError,'Frozen executable source changed'):module.verify()
    math.write_bytes(b'original model');(data/'sample').write_bytes(b'changed')
    with self.assertRaisesRegex(AssertionError,'Original corpus changed'):module.verify()
 def test_replay_accepts_explicit_recorded_roots_without_changing_world_policy(self):
  import xml.etree.ElementTree as ET
  path=stage_directory(ROOT,'wm_dataset_replay')/'run.py';spec=importlib.util.spec_from_file_location('layout_replay',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  with tempfile.TemporaryDirectory() as directory:
   base=Path(directory);raw=base/'legacy_episode';raw.mkdir();runtime=base/'legacy_runtime';runtime.mkdir();out=base/'new_replay';out.mkdir()
   (runtime/'warehouse.sdf').write_text('<sdf><world name="old"><physics><real_time_factor>0</real_time_factor></physics><plugin name="gz::sim::systems::Physics"/><plugin name="gz::sim::systems::SceneBroadcaster"/><model name="common_observer"/><model name="person_proxy"><link><visual/><sensor/></link></model><gui><plugin filename="MinimalScene"><camera_pose>old</camera_pose></plugin></gui></world></sdf>')
   (runtime/'mecanum_camera.sdf').write_text('<sdf><model name="mecanum_robot"><pose>old</pose><plugin name="old_controller"/><link><sensor/></link></model></sdf>')
   self.assertEqual(module.make_world(out,raw,runtime),raw);world=ET.parse(out/'replay.sdf').getroot().find('world')
   self.assertEqual(world.get('name'),'wm_recorded_replay');self.assertEqual([x.get('name') for x in world.findall('plugin')],['gz::sim::systems::SceneBroadcaster','wm_recorded::Replay'])
   self.assertEqual(world.find("plugin[@name='wm_recorded::Replay']/poses").text,str(raw/'gazebo_poses.jsonl'));self.assertFalse(world.findall('.//sensor'));self.assertFalse(world.findall("model[@name='common_observer']"));self.assertEqual(world.find("model[@name='mecanum_robot']/pose").text,'-28 0 0.04 0 0 0')
if __name__=='__main__':unittest.main()
