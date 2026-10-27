import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.artifacts import store as storage
from dynamuhsim.artifacts.store import pack_capture, verify, restore, evict_raw
from dynamuhsim.analysis.evidence import dump_json, sha256_file, load_json


class ArtifactTests(unittest.TestCase):
    def fixture(self, directory, closed=True):
        repo = Path(directory) / "repo"
        repo.mkdir()
        subprocess.run(["git", "init", "-q", str(repo)], check=True)
        (repo / ".gitignore").write_text("generated/\n")
        (repo / "producer.py").write_text("# producer\n")
        subprocess.run(["git", "-C", str(repo), "add", "."], check=True)
        subprocess.run(["git", "-C", str(repo), "-c", "user.name=Artifact Test", "-c", "user.email=artifact@example.invalid", "commit", "-qm", "fixture"], check=True)
        head = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
        run = repo / "generated/capture"
        for name, data in [("episode/paired_depth/on/1.npy", b"ON"*2000), ("episode/paired_depth/off/1.npy", b"OFF"*2000),
                           ("episode/raw_bag/one.mcap", b"MCAP"*1000), ("episode/raw_bag/metadata.yaml", b"metadata"),
                           ("runtime/ros_logs/session/log.txt", b"log"), ("runtime/shutdown_bin/gz", b"#!/bin/sh\nexit 0\n")]:
            path = run / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        (run / "runtime/shutdown_bin/gz").chmod(0o755)
        (run / "runtime/ros_logs/latest").symlink_to(run / "runtime/ros_logs/session", target_is_directory=True)
        dump_json(run / "episode/result.json", dict(head=head, source_sha256={"producer.py": sha256_file(repo / "producer.py")},
                 cleanup=dict(status="PASS", remaining=[]), recording=dict(returncode=0 if closed else 1, metadata_present=closed)))
        return repo, run, Path(directory) / "store"

    def test_pack_is_deduplicated_and_restore_preserves_files_modes_and_portable_links(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            original = (run / "episode/paired_depth/off/1.npy").read_bytes()
            first = pack_capture(run, repo, store)
            second = pack_capture(run, repo, store)
            self.assertEqual(first["object_id"], second["object_id"])
            self.assertEqual(len(list((store / "objects").glob("*.tar.gz"))), 1)
            output = Path(directory) / "restored"
            receipt = restore(store, first["object_id"], output)
            self.assertEqual((output / "run/episode/paired_depth/off/1.npy").read_bytes(), original)
            self.assertEqual((output / "source/producer.py").read_bytes(), b"# producer\n")
            self.assertEqual((output / "run/runtime/shutdown_bin/gz").stat().st_mode & 0o777, 0o755)
            self.assertEqual(os.readlink(output / "run/runtime/ros_logs/latest"), "session")
            self.assertEqual((output / "run/runtime/ros_logs/latest/log.txt").read_bytes(), b"log")
            self.assertEqual(receipt["verified_links"], 1)
            with self.assertRaises(FileExistsError): restore(store, first["object_id"], output)

    def test_restore_preserves_empty_directories_and_internal_directory_links(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            (run / "runtime/empty").mkdir(mode=0o750)
            (run / "runtime/empty_link").symlink_to("empty", target_is_directory=True)
            (run / "episode/unused/nested").mkdir(parents=True)
            manifest = pack_capture(run, repo, store)
            output = Path(directory) / "restored"
            restore(store, manifest["object_id"], output)
            self.assertTrue((output / "run/runtime/empty_link").is_dir())
            self.assertEqual((output / "run/runtime/empty").stat().st_mode & 0o777, 0o750)
            self.assertTrue((output / "run/episode/unused/nested").is_dir())
            self.assertEqual(evict_raw(store, manifest["object_id"], run, repo)["state"], "evicted")

    def test_legacy_file_only_bundle_still_restores_and_evicts(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            original_scan = storage._inventory
            def legacy_scan(path, **kwargs):
                entries, paths = original_scan(path)
                return {name: row for name, row in entries.items() if row["type"] != "directory"}, paths
            with patch.object(storage, "_inventory", side_effect=legacy_scan):
                manifest = pack_capture(run, repo, store)
            restore(store, manifest["object_id"], Path(directory) / "restored")
            self.assertEqual(evict_raw(store, manifest["object_id"], run, repo)["state"], "evicted")

    def test_corrupt_archive_is_rejected_before_restore_creates_output(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            manifest = pack_capture(run, repo, store)
            archive = store / "objects" / (manifest["object_id"] + ".tar.gz")
            archive.write_bytes(archive.read_bytes()[:-10] + b"corrupt")
            output = Path(directory) / "restored"
            with self.assertRaises(ValueError): restore(store, manifest["object_id"], output)
            self.assertFalse(output.exists())
            self.assertTrue((run / "episode/paired_depth/off/1.npy").exists())

    def test_unsafe_manifest_path_and_external_symlink_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            manifest = pack_capture(run, repo, store)
            path = store / "manifests" / (manifest["object_id"] + ".json")
            manifest["entries"]["run/../../escape"] = dict(type="file", bytes=0, sha256="0"*64, mode=0o644)
            dump_json(path, manifest)
            with self.assertRaises(ValueError): verify(store, manifest["object_id"])
            (run / "external").symlink_to(repo / "producer.py")
            with self.assertRaises(ValueError): pack_capture(run, repo, Path(directory) / "other_store")

    def test_store_inside_source_and_incomplete_provenance_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            with self.assertRaises(ValueError): pack_capture(run, repo, run / "store")
            result = load_json(run / "episode/result.json")
            result["source_sha256"]["unavailable.bin"] = "0" * 64
            dump_json(run / "episode/result.json", result)
            with self.assertRaises(FileNotFoundError): pack_capture(run, repo, store)

    def test_eviction_requires_round_trip_then_removes_only_ignored_raw_and_is_resumable(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            manifest = pack_capture(run, repo, store)
            object_id = manifest["object_id"]
            with self.assertRaises((ValueError, FileNotFoundError)): evict_raw(store, object_id, run, repo)
            self.assertTrue((run / "episode/paired_depth/on/1.npy").exists())
            restore(store, object_id, Path(directory) / "restored")
            result_hash = sha256_file(run / "episode/result.json")
            reference = evict_raw(store, object_id, run, repo)
            self.assertEqual(reference["state"], "evicted")
            self.assertEqual(len(reference["raw_files"]), 3)
            self.assertFalse((run / "episode/paired_depth/off/1.npy").exists())
            self.assertEqual(sha256_file(run / "episode/result.json"), result_hash)
            self.assertTrue((run / "episode/raw_bag/metadata.yaml").exists())
            self.assertTrue((run / "runtime/ros_logs/latest/log.txt").exists())
            self.assertEqual(evict_raw(store, object_id, run, repo)["state"], "evicted")
            verify(store, object_id)
            with self.assertRaises(ValueError): pack_capture(run, repo, store)

    def test_changed_raw_or_tracked_files_block_the_entire_eviction(self):
        for mutation in ["raw", "tracked", "extra", "metadata"]:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                repo, run, store = self.fixture(directory)
                manifest = pack_capture(run, repo, store)
                restore(store, manifest["object_id"], Path(directory) / "restored")
                path = run / "episode/paired_depth/off/1.npy"
                if mutation == "raw": path.write_bytes(b"changed")
                elif mutation == "tracked": subprocess.run(["git", "-C", str(repo), "add", "-f", str(path)], check=True)
                elif mutation == "extra": (run / "episode/paired_depth/off/2.npy").write_bytes(b"new")
                else: (run / "episode/raw_bag/metadata.yaml").write_bytes(b"changed metadata")
                with self.assertRaises(ValueError): evict_raw(store, manifest["object_id"], run, repo)
                self.assertTrue((run / "episode/paired_depth/on/1.npy").exists())
                self.assertTrue(path.exists())
                self.assertFalse((run / "artifact_ref.json").exists())

    def test_capture_changes_during_packing_are_not_published(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory)
            original_scan = storage._inventory
            calls = []
            def scan(path, **kwargs):
                calls.append(True)
                if len(calls) == 2:
                    (run / "episode/paired_depth/off/1.npy").write_bytes(b"writer changed file")
                return original_scan(path, **kwargs)
            with patch.object(storage, "_inventory", side_effect=scan):
                with self.assertRaisesRegex(RuntimeError, "changed during packing"):
                    pack_capture(run, repo, store)
            self.assertEqual(list((store / "objects").glob("*.tar.gz")), [])
            self.assertEqual((run / "episode/paired_depth/off/1.npy").read_bytes(), b"writer changed file")

    def test_failed_capture_can_be_preserved_but_cannot_be_evicted(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, run, store = self.fixture(directory, closed=False)
            manifest = pack_capture(run, repo, store)
            restore(store, manifest["object_id"], Path(directory) / "restored")
            with self.assertRaises(ValueError): evict_raw(store, manifest["object_id"], run, repo)
            self.assertTrue((run / "episode/paired_depth/off/1.npy").exists())


if __name__ == "__main__": unittest.main()
