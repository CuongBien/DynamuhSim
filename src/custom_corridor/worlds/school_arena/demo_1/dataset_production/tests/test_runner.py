import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from dataset_production.classification import ProductionDecision
from dataset_production.plan import load_config, make_plan, read_plan, write_plan
from dataset_production.runner import run_plan

ROOT = Path(__file__).resolve().parents[2]


class RunnerTests(unittest.TestCase):
    def test_resume_skips_pass_retries_infrastructure_and_keeps_batch(self):
        base = load_config(ROOT / "configs/dataset_v1.yaml")
        with tempfile.TemporaryDirectory() as temp:
            cfg = replace(base, output_dir=Path(temp),
                          target_episodes={key: 1 for key in base.target_episodes})
            rows = make_plan(cfg)
            write_plan(cfg.output_dir / "production_plan.csv", rows)
            calls = []

            def backend(path):
                calls.append(path.name)
                if path.name == rows[1]["episode_id"] and calls.count(path.name) == 1:
                    return ProductionDecision("FAIL_INFRASTRUCTURE", "", "bag corrupt")
                return ProductionDecision("PASS", "timeout", "observed timeout")

            run_plan(cfg, rows, backend, resume=False, max_episodes=3)
            self.assertEqual([r["status"] for r in rows[:3]],
                             ["PASS", "FAIL_INFRASTRUCTURE", "PASS"])
            first_id = rows[0]["episode_id"]
            run_plan(cfg, read_plan(cfg.output_dir / "production_plan.csv"),
                     backend, resume=True, max_episodes=1)
            updated = read_plan(cfg.output_dir / "production_plan.csv")
            self.assertEqual(updated[0]["status"], "PASS")
            self.assertEqual(calls.count(first_id), 1)
            self.assertEqual(updated[1]["status"], "PASS")
            self.assertEqual(updated[1]["attempts"], "2")


if __name__ == "__main__":
    unittest.main()
