"""Fresh checkout baselines and runtime generation need no historical output cache."""
import ast,hashlib,json,os,shutil,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
from experiments.gazebo_benchmark.core.benchmark_world_generator import BenchmarkWorldGenerator
ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path('experiments/gazebo_benchmark/configs')
SUITES=Path('experiments/gazebo_benchmark/suites')

class BenchmarkInputs(unittest.TestCase):
 def test_baselines_match_original_bytes_and_have_no_generated_dependencies(self):
  folder=ROOT/CONFIG/'fixtures';manifest=json.loads((folder/'manifest.json').read_text());self.assertEqual(len(manifest['files']),7)
  for name,row in manifest['files'].items():
   data=(folder/name).read_bytes();self.assertEqual(len(data),row['bytes']);self.assertEqual(hashlib.sha256(data).hexdigest(),row['sha256']);self.assertEqual(hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest(),row['git_blob']);xml=ET.fromstring(data)
   for node in xml.iter():
    if node.text:self.assertNotIn('configs/generated',node.text)
 def test_all_fixed_consumers_resolve_from_a_relocated_checkout_without_generated(self):
  with tempfile.TemporaryDirectory() as directory:
   relocated=Path(directory)/'clone';shutil.copytree(ROOT/CONFIG/'fixtures',relocated/CONFIG/'fixtures');consumers=set();seen=set()
   for path in (ROOT/SUITES).rglob('*.py'):
    tree=ast.parse(path.read_text());assignments=[n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(c,ast.Constant) and isinstance(c.value,str) and 'configs/fixtures/' in c.value for c in ast.walk(n.value))]
    if not assignments:continue
    consumers.add(path.name);scope=dict(os=os,Path=Path,root_dir=str(relocated),ROOT_DIR=str(relocated),__file__=str(relocated/path.relative_to(ROOT)),SCRIPT_DIR=str((relocated/path.relative_to(ROOT)).parent))
    for n in assignments:
     for count in [0,2,5,10,24]:
      scope['num_humans']=count;value=Path(eval(compile(ast.Expression(n.value),str(path),'eval'),scope));self.assertTrue(value.is_file(),str(value));self.assertTrue(value.is_relative_to(relocated/CONFIG/'fixtures'));seen.add(value.name)
    # All explicit repository defaults now resolve from the calling checkout.
    for n in ast.walk(tree):
     if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ['root_dir','ROOT_DIR'] for t in n.targets):
      self.assertEqual(Path(eval(compile(ast.Expression(n.value),str(path),'eval'),scope)),relocated)
   self.assertEqual(len(consumers),9);self.assertEqual(len(seen),7);self.assertFalse((relocated/CONFIG/'generated').exists())
 def test_generator_builds_new_world_and_robot_without_a_tracked_cache(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory)
   for name in ['src/custom_corridor/worlds/corridor_090.sdf','src/custom_corridor/models/turtlebot3_burger/model.sdf']:
    target=root/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,target)
   generator=BenchmarkWorldGenerator(str(root));world=Path(generator.generate_world_with_humans(2,seed=123,real_time_factor=.5));robot=Path(generator.generate_robot_model(depth_fps=10));w=ET.parse(world).getroot();ET.parse(robot)
   self.assertEqual([n.text for n in w.iter('human_count')],['2']);self.assertEqual([n.text for n in w.iter('real_time_factor')],['0.5']);self.assertEqual(len([n for n in w.iter('actor') if n.get('name','').startswith('human_')]),2);self.assertTrue(robot.is_file());self.assertFalse((root/CONFIG/'fixtures').exists())
if __name__=='__main__':unittest.main()
