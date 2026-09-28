import unittest
from pathlib import Path

from dataset_production.balance import BEHAVIORS, balance_preview
from dataset_production.plan import load_config, read_plan

ROOT = Path(__file__).resolve().parents[2]


class BalanceTests(unittest.TestCase):
    def test_planned_dataset_represents_all_families_densities_behaviors(self):
        config = load_config(ROOT / "configs/dataset_v1.yaml")
        rows = read_plan(config.output_dir / "production_plan.csv")
        report = balance_preview(config, rows)
        self.assertEqual(report["planned_episodes"], 605)
        self.assertEqual(report["warnings"], [])
        self.assertEqual(len(report["scenario_counts"]), 24)
        self.assertTrue(all(report["behavior_counts"][behavior] >= 5
                            for behavior in BEHAVIORS))


if __name__ == "__main__":
    unittest.main()
