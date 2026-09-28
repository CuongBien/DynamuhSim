import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from dataset_production.plan import load_config, make_plan
from dataset_production.prepare import ensure_episode

ROOT = Path(__file__).resolve().parents[2]


class PrepareTests(unittest.TestCase):
    def test_atomic_generate_and_resume_skip(self):
        base = load_config(ROOT / "configs/dataset_v1.yaml")
        with tempfile.TemporaryDirectory() as temp:
            cfg = replace(base, output_dir=Path(temp),
                          target_episodes={key: 1 for key in base.target_episodes})
            row = make_plan(cfg)[0]
            self.assertTrue(ensure_episode(cfg, row))
            self.assertFalse(ensure_episode(cfg, row))
            path = cfg.output_dir / "episodes" / row["episode_id"] / "scenario.yaml"
            saved = path.read_text()
            path.write_text(saved.replace(row["scenario_family"], "wrong_family", 1))
            with self.assertRaises(ValueError):
                ensure_episode(cfg, row)


if __name__ == "__main__":
    unittest.main()
