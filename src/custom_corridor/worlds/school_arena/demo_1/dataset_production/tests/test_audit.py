import csv
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from dataset_production.audit import audit
from dataset_production.freeze import freeze
from dataset_production.plan import load_config, make_plan, write_plan
from dataset_production.prepare import ensure_episode

ROOT = Path(__file__).resolve().parents[2]


class AuditTests(unittest.TestCase):
    def test_fake_pass_without_runtime_artifacts_is_rejected(self):
        base = load_config(ROOT / "configs/dataset_v1.yaml")
        with tempfile.TemporaryDirectory() as temp:
            config = replace(base, output_dir=Path(temp))
            row = dict(make_plan(config)[0], status="PASS", outcome="success")
            write_plan(config.output_dir / "production_plan.csv", [row])
            ensure_episode(config, row)
            with (config.output_dir / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=("episode_id",))
                writer.writeheader()
                writer.writerow({"episode_id": row["episode_id"]})
            report = audit(config, write=False)
            self.assertEqual(report["state"], "INCOMPLETE")
            self.assertTrue(any("execution.yaml" in issue for issue in report["integrity_issues"]))
            with self.assertRaises(ValueError):
                freeze(config)


if __name__ == "__main__":
    unittest.main()
