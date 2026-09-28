import copy
import tempfile
import unittest
from collections import Counter
from pathlib import Path

from dataset_production.plan import (
    check_plan_identity, extend_for_deficits, load_config, make_plan, read_plan, write_plan,
)

ROOT = Path(__file__).resolve().parents[2]


class ProductionPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "configs/dataset_v1.yaml")

    def test_full_plan_is_deterministic_and_has_no_duplicate_config(self):
        first = make_plan(self.config)
        second = make_plan(self.config)
        self.assertEqual(first, second)
        self.assertEqual(len(first), 605)
        self.assertEqual(len({r["episode_id"] for r in first}), 605)
        self.assertEqual(len({r["scenario_sha256"] for r in first}), 605)
        counts = Counter(r["scenario_family"] for r in first)
        self.assertEqual(len(counts), 24)
        self.assertEqual(set(r["density"] for r in first), {"low", "medium", "high"})
        for family in counts:
            self.assertEqual({"low", "medium", "high"},
                             {r["density"] for r in first if r["scenario_family"] == family})

    def test_fill_missing_uses_new_seed_and_preserves_existing_rows(self):
        expected = make_plan(self.config)
        rows = [dict(row, status="PASS") for row in expected]
        rows[0]["status"] = "FAIL_INFRASTRUCTURE"
        extended = extend_for_deficits(self.config, rows)
        self.assertEqual(len(extended), len(rows) + 1)
        self.assertEqual(extended[:len(rows)], rows)
        self.assertEqual(extended[-1]["scenario_family"], rows[0]["scenario_family"])
        self.assertEqual(extended[-1]["status"], "PLANNED")
        self.assertNotIn(extended[-1]["seed"], {row["seed"] for row in rows})
        self.assertEqual(extended, extend_for_deficits(self.config, rows))

    def test_resume_preserves_status_but_rejects_identity_drift(self):
        expected = make_plan(self.config)
        stored = copy.deepcopy(expected)
        stored[0]["status"] = "PASS"
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "production_plan.csv"
            write_plan(path, stored)
            loaded = read_plan(path)
            check_plan_identity(loaded, expected)
            loaded[0]["seed"] = str(int(loaded[0]["seed"]) + 1)
            with self.assertRaises(ValueError):
                check_plan_identity(loaded, expected)


if __name__ == "__main__":
    unittest.main()
