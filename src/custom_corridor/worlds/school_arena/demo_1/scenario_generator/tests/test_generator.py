import copy
import random
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.scenario_generator import FAMILIES, ScenarioGenerator
from src.route_sampler import RouteSampler
from src.validators import ValidationError, validate_episode


class GeneratorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.generator = ScenarioGenerator()

    def test_ten_seeds_each_and_determinism(self):
        for family in FAMILIES:
            episodes = [self.generator.sample(family, seed) for seed in range(10)]
            self.assertEqual(episodes[3], self.generator.sample(family, 3))
            self.assertGreater(len({str(ep) for ep in episodes}), 1)
            for episode in episodes:
                self.assertEqual(sum(h['role'] == 'event_agent' for h in episode['humans']),
                                 0 if family == 'empty' else 1)

    def test_empty_hunav_ros_parameters(self):
        scenario = self.generator.sample("empty", 100)
        with tempfile.TemporaryDirectory() as temp:
            output = self.generator.write(scenario, Path(temp) / "ep_000001", "ep_000001")
            params = yaml.safe_load((output / "humans.yaml").read_text())["hunav_loader"]["ros__parameters"]
            sample = self.generator.inputs.hunav["hunav_loader"]["ros__parameters"]
            self.assertEqual(set(params), set(sample) - {"agents", "global_goals", *sample["agents"]})
            self.assertNotIn("agents", params)
            self.assertNotIn("global_goals", params)

    def test_high_density_and_output_schema(self):
        scenario = self.generator.sample('crossing', 90210, 'high')
        self.assertGreaterEqual(len(scenario['humans']), 8)
        with tempfile.TemporaryDirectory() as temp:
            output = self.generator.write(scenario, Path(temp) / 'ep_000001', 'ep_000001')
            metadata = yaml.safe_load((output / 'metadata.yaml').read_text())
            params = yaml.safe_load((output / 'humans.yaml').read_text())['hunav_loader']['ros__parameters']
            self.assertEqual(metadata['humans']['total'], len(params['agents']))
            self.assertEqual(len(list(output.glob('*__agent_*_bt.xml'))), len(params['agents']))
            models = ET.parse(output / 'school_floor.world').getroot().find('world').findall('model')
            names = {model.get('name') for model in models}
            self.assertTrue(set(params['agents']) <= names)
            self.assertFalse({'demo_primary', 'demo_crossing'} & names)
            nav = yaml.safe_load((output / 'nav2_school.yaml').read_text())
            self.assertEqual(nav['amcl']['ros__parameters']['initial_pose']['x'],
                             scenario['robot']['start_pose']['x'])

    def test_written_config_same_seed(self):
        scenario = self.generator.sample("head_on", 42)
        with tempfile.TemporaryDirectory() as temp:
            first = self.generator.write(scenario, Path(temp) / "ep_000001", "ep_000001")
            second = self.generator.write(scenario, Path(temp) / "ep_000002", "ep_000002")
            for filename in ("scenario.yaml", "humans.yaml", "school_floor.world", "nav2_school.yaml"):
                self.assertEqual((first / filename).read_bytes(), (second / filename).read_bytes())

    def test_invalid_spawn_rejected(self):
        scenario = copy.deepcopy(self.generator.sample('head_on', 42))
        scenario['humans'][0]['spawn'].update(
            x=scenario['robot']['start_pose']['x'],
            y=scenario['robot']['start_pose']['y'])
        routes = RouteSampler(self.generator.inputs, random.Random(42))
        with self.assertRaises(ValidationError):
            validate_episode(self.generator.inputs, routes, self.generator.occupancy, scenario)


if __name__ == '__main__':
    unittest.main()
