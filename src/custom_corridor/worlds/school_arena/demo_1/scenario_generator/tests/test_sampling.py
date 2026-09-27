import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.models import load_inputs
from src.zone_sampler import ZoneSampler, contains
from src.route_sampler import RouteSampler


class SamplingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = load_inputs(ROOT / "configs/generator.yaml")

    def test_zones(self):
        sampler = ZoneSampler(self.data, random.Random(42))
        node = sampler.choose_node(node_type="classroom", zone_id="room01")
        self.assertEqual(node.id, "room01_center")
        self.assertTrue(contains(self.data.zones["room01"], node.x, node.y))

    def test_graph_routes(self):
        routes = RouteSampler(self.data, random.Random(42))
        path = routes.shortest("room01_center", "room12_center")
        self.assertTrue(routes.valid(path))
        self.assertIn("room01_door", path)
        self.assertGreater(routes.length(path), 10)
        self.assertTrue(any(name == "corridor_a" for name, _ in routes.corridor_chains()))


if __name__ == "__main__":
    unittest.main()
