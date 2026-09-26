import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.models import ConfigError, load_inputs


class InputTests(unittest.TestCase):
    def test_existing_demo_files_load(self):
        data = load_inputs(ROOT / "configs/generator.yaml")
        self.assertEqual(len([z for z in data.zones.values() if z.type == "classroom"]), 15)
        self.assertIn("room01_center", data.nodes)
        self.assertIn("hunav_loader", data.hunav)
        self.assertTrue(data.paths["map_image"].is_file())

    def test_missing_input_rejected(self):
        with self.assertRaises(ConfigError):
            load_inputs(ROOT / "configs/missing.yaml")


if __name__ == "__main__":
    unittest.main()
