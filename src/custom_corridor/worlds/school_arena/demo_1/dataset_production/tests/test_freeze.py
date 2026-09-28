import csv
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from dataset_production.freeze import freeze, make_splits
from dataset_production.plan import load_config, make_plan, write_plan

ROOT = Path(__file__).resolve().parents[2]


class FreezeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        base = load_config(ROOT / "configs/dataset_v1.yaml")
        cls.config = replace(base, target_episodes={key: 18 for key in base.target_episodes})
        cls.rows = make_plan(cls.config)

    def test_unfinished_plan_cannot_split(self):
        with self.assertRaises(ValueError):
            make_splits(self.rows, self.config)

    def test_episode_splits_and_fingerprint_are_stable_and_detect_tampering(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config = replace(self.config, output_dir=root)
            rows = [dict(row, status="PASS", outcome="timeout") for row in self.rows]
            write_plan(root / "production_plan.csv", rows)
            with (root / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=("episode_id", "outcome"))
                writer.writeheader()
                writer.writerows({"episode_id": row["episode_id"], "outcome": row["outcome"]}
                                 for row in rows)
            with patch("dataset_production.audit.audit", return_value={"state": "COMPLETE"}):
                first = freeze(config)
                self.assertEqual(first, freeze(config))
            self.assertEqual(first["episode_count"], len(rows))
            files = [root / "splits" / f"{name}.txt" for name in ("train", "val", "test")]
            self.assertEqual(sum(len(path.read_text().splitlines()) for path in files), len(rows))
            self.assertTrue(all(path.read_text().strip() for path in files))
            files[0].write_text(files[0].read_text() + "ep_999999\n")
            with patch("dataset_production.audit.audit", return_value={"state": "COMPLETE"}):
                with self.assertRaises(ValueError):
                    freeze(config)


if __name__ == "__main__":
    unittest.main()
