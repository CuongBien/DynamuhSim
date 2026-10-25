import unittest
import hashlib
import json
from pathlib import Path

from dataset_production.balance import BEHAVIORS, balance_preview
from dataset_production.plan import load_config, make_plan

ROOT = Path(__file__).resolve().parents[2]


class BalanceTests(unittest.TestCase):
    def test_planned_dataset_represents_all_families_densities_behaviors(self):
        config = load_config(ROOT / "configs/dataset_v1.yaml")
        rows = make_plan(config)
        # Plan identity from the archived CSV; runtime status is excluded.
        fields = ("episode_id", "scenario_family", "seed", "density", "scenario_sha256")
        identity = [{key: row[key] for key in fields} for row in rows]
        self.assertEqual(hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest(),
                         "5ca78811ab28f38e9fc6c27586d69742df8cfa5a829d0e48d8b29778156bed9d")
        report = balance_preview(config, rows)
        self.assertEqual(report["planned_episodes"], 605)
        self.assertEqual(report["warnings"], [])
        self.assertEqual(len(report["scenario_counts"]), 24)
        self.assertTrue(all(report["behavior_counts"][behavior] >= 5
                            for behavior in BEHAVIORS))


if __name__ == "__main__":
    unittest.main()
