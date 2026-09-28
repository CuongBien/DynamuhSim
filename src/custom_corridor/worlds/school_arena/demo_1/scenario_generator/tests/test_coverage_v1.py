"""Dataset V1 generation contracts across new scenario families."""
import copy
import random
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DEMO))

from src.scenario_generator import FAMILIES, ScenarioGenerator
from src.route_sampler import RouteSampler
from src.validators import ValidationError, validate_episode
from dataset_runner.episode_loader import EpisodeLoader


class CoverageV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.generator = ScenarioGenerator()

    def test_new_scenarios_twenty_seeds_three_densities(self):
        for family in FAMILIES[5:]:
            variants = set()
            for seed in range(20):
                density = ("low", "medium", "high")[seed % 3]
                with self.subTest(family=family, seed=seed, density=density):
                    episode = self.generator.sample(family, seed, density)
                    self.assertEqual(episode, self.generator.sample(family, seed, density))
                    self.assertEqual(episode["scenario_family"], family)
                    self.assertEqual(episode["density"], density)
                    self.assertGreaterEqual(len([h for h in episode["humans"]
                                                 if h["role"] == "event_agent"]), 1)
                    routes = RouteSampler(self.generator.inputs, random.Random(seed))
                    validate_episode(self.generator.inputs, routes,
                                     self.generator.occupancy, episode)
                    if family == "multi_crossing":
                        events = [h for h in episode["humans"] if h["role"] == "event_agent"]
                        sides = {1 if self.generator.inputs.nodes[h["start_node"]].y >
                                 self.generator.inputs.nodes[c].y else -1
                                 for h, c in zip(events, episode["event"]["conflict_nodes"])}
                        self.assertEqual(sides, {-1, 1})
                    variants.add(str(episode))
            self.assertGreater(len(variants), 1, family)

    def test_all_families_write_runner_readable_files(self):
        loader = EpisodeLoader()
        with tempfile.TemporaryDirectory() as temp:
            for index, family in enumerate(FAMILIES):
                with self.subTest(family=family):
                    scenario = self.generator.sample(family, 1000 + index, "medium")
                    path = self.generator.write(scenario, Path(temp) / f"ep_{index:06d}",
                                                f"ep_{index:06d}")
                    context = loader.load(path)
                    self.assertEqual(context.scenario_family, family)
                    self.assertEqual(context.metadata["humans"]["total"], len(scenario["humans"]))

    def test_weighted_choice_is_configured_and_deterministic(self):
        weights = self.generator.inputs.config["scenario_weights"]
        self.assertEqual(set(weights), set(FAMILIES))
        self.assertEqual(self.generator.weighted_family(42),
                         self.generator.weighted_family(42))
        self.assertGreater(len({self.generator.weighted_family(seed)
                                for seed in range(100)}), 1)
        self.generator.inputs.config["scenario_weights"] = {
            "blind_corner": 0.0, "door_bottleneck": 1.0,
        }
        try:
            self.assertEqual({self.generator.weighted_family(seed)
                              for seed in range(20)}, {"door_bottleneck"})
        finally:
            self.generator.inputs.config["scenario_weights"] = weights

    def test_key_semantic_constraints_reject_mutations(self):
        mutations = {
            "merge": lambda ep: ep["event"].update(merge_node=ep["robot"]["start_node"]),
            "blind_corner": lambda ep: ep["event"].update(
                human_incoming=ep["event"]["robot_incoming"]),
            "door_bottleneck": lambda ep: ep["event"].update(doorway="absent_door"),
            "walking_group": lambda ep: ep["humans"][1].update(group_id=99),
            "class_change_burst": lambda ep: ep["event"]["source_rooms"].__setitem__(1, "room00"),
            "mixed_interaction": lambda ep: ep["humans"].__setitem__(
                slice(1, None), []),
        }
        for family, mutate in mutations.items():
            with self.subTest(family=family):
                scenario = copy.deepcopy(self.generator.sample(family, 7))
                mutate(scenario)
                routes = RouteSampler(self.generator.inputs, random.Random(7))
                with self.assertRaises(ValidationError):
                    validate_episode(self.generator.inputs, routes,
                                     self.generator.occupancy, scenario)



if __name__ == "__main__":
    unittest.main()
