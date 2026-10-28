"""Command parity across all maintained worker factories, without launching workers."""
import importlib.util, json, os, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

class WorkerCommands(unittest.TestCase):
    def test_stage_contracts_match_recorded_commands_with_one_declared_mount_repair(self):
        fixtures=json.loads((ROOT/'tests/fixtures/worker_commands.json').read_text())
        for row in fixtures:
            with self.subTest(stage=row['stage'],model=row['model']):
                path=ROOT/'experiments/social_navigation/navigation'/row['stage']/'runtime.py'
                spec=importlib.util.spec_from_file_location('_command_'+row['stage'],path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                args=[Path('/fixture/runtime'),Path('/fixture/socket'),row['model']]+([] if row['stage']=='shared_nav2_pilot' else ['crossing_near'])+['fixturetoken1234567890']
                actual=json.loads(json.dumps(module.worker_command(*args)).replace(str(ROOT),'/fixture/repository'))
                expected=row['result']
                if expected[0][0]=='docker':expected[0][expected[0].index('--user')+1]=f'{os.getuid()}:{os.getgid()}'
                if row['stage']=='common_gazebo' and row['model']!='soc_guarded_orca':
                    command=expected[0];common=next(i for i,value in enumerate(command) if value.endswith(':/common:ro'))
                    command[common+1:common+1]=['-v','/fixture/repository/experiments/archive/social_navigation/reports/common_response/run01/protocol.json:/archive/social_navigation/reports/common_response/run01/protocol.json:ro']
                self.assertEqual(actual,expected)
