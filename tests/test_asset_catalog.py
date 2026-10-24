import copy
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.artifacts.catalog import create_catalog, verify_catalog, verify_install, PACKAGE, POLICY, WORLDS, BENCHMARK


class AssetCatalogTests(unittest.TestCase):
    def fixture(self, directory):
        repo = Path(directory) / "repo"
        repo.mkdir()
        subprocess.run(["git", "init", "-q", str(repo)], check=True)
        files = {POLICY: "# payload policy\npayload\n", WORLDS + "/base.sdf": "runtime world", WORLDS + "/payload/episode.bin": "recorded data",
                 PACKAGE + "/launch/native.launch.py": "# launch", PACKAGE + "/models/robot/model.sdf": "robot",
                 BENCHMARK + "/socnavgym_nav2_pilot/reports/gate01/validation.json": '{"status":"PASS"}',
                 BENCHMARK + "/old_stage/reports/snapshot/driver.py": "recorded data",
                 "protocol/frozen.json": json.dumps(dict(source_sha256={BENCHMARK + "/old_stage/reports/snapshot/driver.py": "0"*64}))}
        for name, data in files.items():
            path = repo / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(data)
        subprocess.run(["git", "-C", str(repo), "add", "."], check=True)
        subprocess.run(["git", "-C", str(repo), "-c", "user.name=Catalog Test", "-c", "user.email=catalog@example.invalid", "commit", "-qm", "assets"], check=True)
        head = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
        return repo, Path(directory) / "store", head

    def test_inventory_uses_committed_blobs_even_if_worktree_is_dirty(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, head = self.fixture(directory)
            payload = repo / WORLDS / "payload/episode.bin"
            payload.write_text("uncommitted change")
            (payload.parent / "untracked.bin").write_text("not pushed")
            catalog = create_catalog(repo, store)
            self.assertEqual(catalog["revision"], head)
            self.assertEqual(catalog["files"], 3)
            group = next(row for row in catalog["entries"] if row["id"] == "world:payload")
            inventory = json.loads(gzip.decompress((store / group["inventory_object"]).read_bytes()))
            self.assertEqual(inventory["entries"]["episode.bin"]["sha256"], hashlib.sha256(b"recorded data").hexdigest())
            self.assertEqual(payload.read_text(), "uncommitted change")
            self.assertEqual(verify_catalog(repo, store, catalog)["status"], "PASS")

    def test_catalog_is_repeatable_and_retains_gate_and_freeze_blockers(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, head = self.fixture(directory)
            first = create_catalog(repo, store)
            second = create_catalog(repo, store)
            self.assertEqual(first, second)
            self.assertEqual(len(list((store / "catalogs/objects").glob("*.gz"))), 3)
            gate = next(row for row in first["entries"] if row["id"] == "reports:socnavgym_nav2_pilot")
            old = next(row for row in first["entries"] if row["id"] == "reports:old_stage")
            self.assertEqual(len(gate["current_gate_files"]), 1)
            self.assertEqual(old["historical_freeze_references"], {"protocol/frozen.json": 1})
            self.assertTrue(all(row["removal_allowed"] is False and row["payload_externalized"] is False for row in first["entries"]))
            self.assertEqual(first["unique_payload_blobs"], 2)

    def test_catalog_cannot_authorize_removal_or_drop_groups(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, head = self.fixture(directory)
            catalog = create_catalog(repo, store)
            bad = copy.deepcopy(catalog)
            bad["entries"][0]["removal_allowed"] = True
            with self.assertRaises(ValueError): verify_catalog(repo, store, bad)
            bad = copy.deepcopy(catalog)
            bad["entries"].pop()
            with self.assertRaises(ValueError): verify_catalog(repo, store, bad)

    def test_corrupt_inventory_and_tampered_consumer_metadata_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, head = self.fixture(directory)
            catalog = create_catalog(repo, store)
            bad = copy.deepcopy(catalog)
            gate = next(row for row in bad["entries"] if row["id"] == "reports:socnavgym_nav2_pilot")
            gate["current_gate_files"] = []
            with self.assertRaisesRegex(ValueError, "consumer"): verify_catalog(repo, store, bad)
            obj = store / catalog["entries"][0]["inventory_object"]
            obj.write_bytes(b"corrupt")
            with self.assertRaisesRegex(ValueError, "hash"): verify_catalog(repo, store, catalog)

    def test_rehashed_but_wrong_per_file_sha_is_rejected_against_git(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, head = self.fixture(directory)
            catalog = create_catalog(repo, store)
            group = catalog["entries"][0]
            inventory = json.loads(gzip.decompress((store / group["inventory_object"]).read_bytes()))
            next(iter(inventory["entries"].values()))["sha256"] = "0"*64
            raw = json.dumps(inventory,sort_keys=True,indent=2).encode()+b"\n"
            compressed = gzip.compress(raw,mtime=0)
            group["inventory_sha256"] = hashlib.sha256(raw).hexdigest()
            group["inventory_gzip_sha256"] = hashlib.sha256(compressed).hexdigest()
            group["inventory_object"] = "catalogs/objects/"+group["inventory_sha256"]+".json.gz"
            (store / group["inventory_object"]).write_bytes(compressed)
            with self.assertRaisesRegex(ValueError, "SHA256"): verify_catalog(repo, store, catalog)

    def test_lean_and_full_installs_detect_stale_payloads_and_corrupt_runtime_files(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, head = self.fixture(directory)
            full = Path(directory) / "full"
            shutil.copytree(repo / PACKAGE, full)
            self.assertEqual(verify_install(repo, full, legacy=True)["status"], "PASS")
            self.assertEqual(verify_install(repo, full)["status"], "FAIL")
            lean = Path(directory) / "lean"
            shutil.copytree(full, lean)
            shutil.rmtree(lean / "worlds/payload")
            self.assertEqual(verify_install(repo, lean)["status"], "PASS")
            (lean / "worlds/base.sdf").write_text("corrupt")
            result = verify_install(repo, lean)
            self.assertEqual(result["different"], ["worlds/base.sdf"])

    def test_symlink_install_has_same_committed_runtime_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, head = self.fixture(directory)
            share = Path(directory) / "share"
            for name in ["config/world_payloads.txt", "worlds/base.sdf", "launch/native.launch.py", "models/robot/model.sdf"]:
                target = share / name
                target.parent.mkdir(parents=True,exist_ok=True)
                target.symlink_to(repo / PACKAGE / name)
            result = verify_install(repo, share)
            self.assertEqual(result["status"], "PASS")
            self.assertEqual(result["expected_world_files"], 1)

    def test_cli_help_does_not_require_ros(self):
        process = subprocess.run([sys.executable,str(ROOT/'tools/catalog_assets.py'),'--help'],capture_output=True,text=True)
        self.assertEqual(process.returncode,0,process.stderr)


if __name__ == '__main__': unittest.main()
