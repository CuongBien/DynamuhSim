import copy
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
import test_asset_catalog as fixtures
from dynamuhsim.artifacts.catalog import create_catalog, WORLDS
from dynamuhsim.artifacts.history import pack_history, verify_history, verify_history_index, restore_history


class HistoryArchiveTests(unittest.TestCase):
    def fixture(self, directory):
        repo, store, head = fixtures.AssetCatalogTests().fixture(directory)
        extra = repo / WORLDS / "payload"
        (extra / "empty").write_bytes(b"")
        (extra / "runner.sh").write_bytes(b"#!/bin/sh\nexit 7\n")
        (extra / "runner.sh").chmod(0o755)
        (extra / "large.bin").write_bytes(b"large streaming payload" * 55000)
        subprocess.run(["git", "-C", str(repo), "add", "."], check=True)
        subprocess.run(["git", "-C", str(repo), "-c", "user.name=Archive Test", "-c", "user.email=archive@example.invalid", "commit", "-qm", "modes and streaming"], check=True)
        catalog = create_catalog(repo, store)
        return repo, store, catalog

    def packed(self, directory):
        repo, store, catalog = self.fixture(directory)
        index = pack_history(repo, store, catalog)
        bundle = next(row for row in index["bundles"] if row["id"] == "world:payload")
        return repo, store, catalog, index, bundle

    def rewrite_archive(self, store, bundle, transform):
        old_id = bundle["object_id"]
        manifest = json.loads((store / "history/manifests" / (old_id + ".json")).read_text())
        rows = []
        with tarfile.open(store / "history/objects" / (old_id + ".tar.gz"), "r:gz") as archive:
            for member in archive:
                rows.append((member, archive.extractfile(member).read()))
        rows = transform(rows)
        buffer = io.BytesIO()
        with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
            for member, data in rows: archive.addfile(member, io.BytesIO(data) if member.isfile() else None)
        data = buffer.getvalue()
        object_id = hashlib.sha256(data).hexdigest()
        manifest.update(object_id=object_id, archive_bytes=len(data))
        (store / "history/objects" / (object_id + ".tar.gz")).write_bytes(data)
        (store / "history/manifests" / (object_id + ".json")).write_text(json.dumps(manifest))
        return object_id

    def test_deterministic_dedup_and_restore_without_git_or_inventory_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog, index, bundle = self.packed(directory)
            self.assertEqual(index, pack_history(repo, store, catalog))
            self.assertEqual(len(list((store / "history/objects").glob("*.gz"))), 3)
            self.assertEqual(verify_history_index(store, index)["files"], 6)
            shutil.rmtree(repo)
            shutil.rmtree(store / "catalogs")
            output = Path(directory) / "restore"
            with patch("subprocess.Popen", side_effect=AssertionError("Git must not be needed")):
                self.assertEqual(verify_history_index(store, index)["status"], "PASS")
                receipt = restore_history(store, bundle["object_id"], output)
            restored = output / "payload" / bundle["root"]
            self.assertEqual((restored / "episode.bin").read_bytes(), b"recorded data")
            self.assertEqual((restored / "empty").read_bytes(), b"")
            self.assertEqual((restored / "runner.sh").stat().st_mode & 0o777, 0o755)
            self.assertEqual((restored / "large.bin").stat().st_size, 1265000)
            self.assertEqual(receipt["verified_payload_files"], 4)
            self.assertFalse(receipt["removal_allowed"])

    def test_dirty_and_untracked_files_do_not_enter_backup(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog = self.fixture(directory)
            path = repo / WORLDS / "payload/episode.bin"
            path.write_bytes(b"private local edit")
            (path.parent / "extra").write_bytes(b"untracked")
            index = pack_history(repo, store, catalog)
            bundle = next(row for row in index["bundles"] if row["id"] == "world:payload")
            output = Path(directory) / "restore"
            restore_history(store, bundle["object_id"], output)
            self.assertEqual((output / "payload" / bundle["root"] / "episode.bin").read_bytes(), b"recorded data")
            self.assertFalse((output / "payload" / bundle["root"] / "extra").exists())
            self.assertEqual(path.read_bytes(), b"private local edit")

    def test_corrupt_archive_rejected_before_creating_restore(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog, index, bundle = self.packed(directory)
            path = store / "history/objects" / (bundle["object_id"] + ".tar.gz")
            path.write_bytes(path.read_bytes()[:-8] + b"BADBYTES")
            output = Path(directory) / "restore"
            with self.assertRaisesRegex(ValueError, "hash"): restore_history(store, bundle["object_id"], output)
            self.assertFalse(output.exists())

    def test_same_size_rehashed_member_corruption_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog, index, bundle = self.packed(directory)
            def corrupt(rows):
                return [(member, b"X" * len(data) if member.name.endswith("episode.bin") else data) for member, data in rows]
            object_id = self.rewrite_archive(store, bundle, corrupt)
            with self.assertRaisesRegex(ValueError, "SHA256"): verify_history(store, object_id)

    def test_traversal_duplicate_missing_link_and_mode_members_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog, index, bundle = self.packed(directory)
            def altered(kind):
                def change(rows):
                    member, data = rows[-1]
                    member = copy.copy(member)
                    if kind == "traversal": member.name = "../outside"
                    if kind == "duplicate": return rows + [rows[-1]]
                    if kind == "missing": return rows[:-1]
                    if kind == "symlink": member.type, member.linkname = tarfile.SYMTYPE, "../../outside"
                    if kind == "hardlink": member.type, member.linkname = tarfile.LNKTYPE, rows[-2][0].name
                    if kind == "mode": member.mode = 0o777
                    return rows[:-1] + [(member, data)]
                return change
            for kind in ["traversal", "duplicate", "missing", "symlink", "hardlink", "mode"]:
                with self.subTest(kind=kind):
                    object_id = self.rewrite_archive(store, bundle, altered(kind))
                    output = Path(directory) / ("restore-" + kind)
                    with self.assertRaises(ValueError): restore_history(store, object_id, output)
                    self.assertFalse(output.exists())
            self.assertFalse((Path(directory) / "outside").exists())

    def test_restore_refuses_existing_empty_directory_file_symlink_and_store(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog, index, bundle = self.packed(directory)
            output = Path(directory) / "existing"
            output.mkdir()
            with self.assertRaises(FileExistsError): restore_history(store, bundle["object_id"], output)
            file = Path(directory) / "file"
            file.write_text("keep")
            with self.assertRaises(FileExistsError): restore_history(store, bundle["object_id"], file)
            link = Path(directory) / "dangling"
            link.symlink_to(Path(directory) / "absent")
            with self.assertRaises(FileExistsError): restore_history(store, bundle["object_id"], link)
            with self.assertRaisesRegex(ValueError, "separate"): restore_history(store, bundle["object_id"], store / "new")
            self.assertEqual(file.read_text(), "keep")

    def test_tampered_manifest_inventory_and_provenance_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog, index, bundle = self.packed(directory)
            path = store / "history/manifests" / (bundle["object_id"] + ".json")
            original = path.read_bytes()
            manifest = json.loads(original)
            manifest["group"]["current_gate_files"] = ["fake_gate.json"]
            path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "mismatch"): verify_history(store, bundle["object_id"])
            manifest = json.loads(original)
            manifest["inventory"]["entries"]["episode.bin"]["sha256"] = "0" * 64
            path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "inventory"): verify_history(store, bundle["object_id"])

    def test_index_rejects_missing_duplicate_swapped_groups_counts_and_removal_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog, index, bundle = self.packed(directory)
            for kind in ["missing", "duplicate", "swapped", "counts", "flag", "catalog"]:
                bad = copy.deepcopy(index)
                if kind == "missing": bad["bundles"].pop()
                if kind == "duplicate": bad["bundles"][0] = bad["bundles"][1]
                if kind == "swapped": bad["bundles"][0]["object_id"] = bad["bundles"][1]["object_id"]
                if kind == "counts": bad["archive_bytes"] += 1
                if kind == "flag": bad["removal_allowed"] = True
                if kind == "catalog": bad["catalog"]["revision"] = "0" * 40
                with self.subTest(kind=kind), self.assertRaises(ValueError): verify_history_index(store, bad)

    def test_inventory_cache_corruption_prevents_pack(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog = self.fixture(directory)
            (store / catalog["entries"][0]["inventory_object"]).write_bytes(b"broken")
            with self.assertRaises(ValueError): pack_history(repo, store, catalog)
            self.assertEqual(list((store / "history/objects").iterdir()), [])

    def test_committed_symlink_is_rejected_and_source_link_is_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, catalog = self.fixture(directory)
            link = repo / WORLDS / "payload/link"
            link.symlink_to("episode.bin")
            subprocess.run(["git", "-C", str(repo), "add", "."], check=True)
            subprocess.run(["git", "-C", str(repo), "-c", "user.name=Archive Test", "-c", "user.email=archive@example.invalid", "commit", "-qm", "unsupported link"], check=True)
            catalog = create_catalog(repo, store)
            with self.assertRaisesRegex(ValueError, "regular Git files"): pack_history(repo, store, catalog)
            self.assertTrue(link.is_symlink())
            self.assertEqual(link.readlink().as_posix(), "episode.bin")

    def test_cli_help_without_ros_and_existing_output_is_preserved(self):
        help = subprocess.run([sys.executable, str(ROOT / "tools/archive_history.py"), "--help"], capture_output=True, text=True)
        self.assertEqual(help.returncode, 0, help.stderr)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "receipt.json"
            output.write_text("keep")
            process = subprocess.run([sys.executable, str(ROOT / "tools/archive_history.py"), "verify", "--store", directory,
                                      "--index", str(Path(directory)/"absent.json"), "--output", str(output)], capture_output=True, text=True)
            self.assertNotEqual(process.returncode, 0)
            self.assertIn("Preserve existing output", process.stderr)
            self.assertEqual(output.read_text(), "keep")


if __name__ == "__main__": unittest.main()
