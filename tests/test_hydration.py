import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
import test_history_archive as fixtures
from dynamuhsim.artifacts.hydrate import hydrate_history, _publish_directory


class HydrationTests(unittest.TestCase):
    def fixture(self, directory):
        repo, store, catalog, index, bundle = fixtures.HistoryArchiveTests().packed(directory)
        checkout = Path(directory) / "checkout"
        checkout.mkdir()
        return repo, store, index, bundle, checkout

    def test_archive_and_git_restore_same_paths_bytes_and_executable_modes(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            for source in ["archive", "git"]:
                target = checkout / source
                target.mkdir()
                result = hydrate_history(target, index, bundle["id"], source=source, store=store, git_repo=repo)
                self.assertTrue(result["changed"])
                self.assertEqual(result["state"], "present_verified")
                payload = target / bundle["root"]
                self.assertEqual((payload / "episode.bin").read_bytes(), b"recorded data")
                self.assertEqual((payload / "runner.sh").stat().st_mode & 0o777, 0o755)
                self.assertEqual((payload / "empty").read_bytes(), b"")
                again = hydrate_history(target, index, bundle["id"], source=source, store=store, git_repo=repo)
                self.assertFalse(again["changed"])
                self.assertFalse(list(payload.parent.glob(".hydrate-*")))

    def test_check_missing_is_read_only_and_archive_hydration_does_not_call_git(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            shutil.rmtree(repo)
            shutil.rmtree(store / "catalogs")
            with patch("subprocess.Popen", side_effect=AssertionError("No Git needed")):
                report = hydrate_history(checkout, index, bundle["id"], store=store, check=True)
                self.assertEqual(report["state"], "absent")
                self.assertEqual(list(checkout.iterdir()), [])
                self.assertTrue(hydrate_history(checkout, index, bundle["id"], store=store)["changed"])

    def test_existing_empty_partial_modified_or_extra_payload_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            payload = checkout / bundle["root"]
            payload.mkdir(parents=True)
            with self.assertRaisesRegex(ValueError, "Existing payload differs"):
                hydrate_history(checkout,index,bundle["id"],store=store)
            payload.rmdir()
            hydrate_history(checkout,index,bundle["id"],store=store)
            (payload / "episode.bin").write_bytes(b"edited")
            with self.assertRaises(ValueError): hydrate_history(checkout,index,bundle["id"],source="git",git_repo=repo)
            self.assertEqual((payload / "episode.bin").read_bytes(), b"edited")
            (payload / "episode.bin").write_bytes(b"recorded data")
            (payload / "extra").write_bytes(b"keep")
            with self.assertRaises(ValueError): hydrate_history(checkout,index,bundle["id"],store=store)
            self.assertEqual((payload / "extra").read_bytes(), b"keep")

    def test_symlink_ancestor_or_destination_cannot_redirect_hydration(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            outside = Path(directory) / "outside"
            outside.mkdir()
            (checkout / "src").symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, "symlink"): hydrate_history(checkout,index,bundle["id"],store=store)
            self.assertEqual(list(outside.iterdir()), [])
            (checkout / "src").unlink()
            payload = checkout / bundle["root"]
            payload.parent.mkdir(parents=True)
            payload.symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError): hydrate_history(checkout,index,bundle["id"],store=store)

    def test_corrupt_archive_and_git_inventory_mismatch_fail_before_target_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            path = store / "history/objects" / (bundle["object_id"]+".tar.gz")
            path.write_bytes(b"broken")
            with self.assertRaises(ValueError): hydrate_history(checkout,index,bundle["id"],store=store)
            self.assertFalse((checkout / bundle["root"]).exists())
            with patch("dynamuhsim.artifacts.hydrate.tree_identity", return_value="0"*40):
                with self.assertRaisesRegex(ValueError, "inventory"):
                    hydrate_history(checkout,index,bundle["id"],source="git",git_repo=repo)
            self.assertFalse((checkout / bundle["root"]).exists())

    def test_atomic_publish_never_replaces_even_an_empty_existing_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            source, target = Path(directory)/"staged", Path(directory)/"existing"
            source.mkdir(); target.mkdir()
            (source/"data").write_text("staged")
            with self.assertRaises(FileExistsError): _publish_directory(source,target)
            self.assertTrue(source.is_dir())
            self.assertEqual(list(target.iterdir()), [])

    def test_publish_race_preserves_destination_and_cleans_own_staging(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            def raced(source, target):
                target.mkdir()
                (target/"user.txt").write_text("concurrent edit")
                _publish_directory(source,target)
            with patch("dynamuhsim.artifacts.hydrate._publish_directory", side_effect=raced):
                with self.assertRaises(FileExistsError): hydrate_history(checkout,index,bundle["id"],store=store)
            payload=checkout/bundle["root"]
            self.assertEqual((payload/"user.txt").read_text(),"concurrent edit")
            self.assertFalse(list(payload.parent.glob(".hydrate-*")))

    def test_unknown_group_swapped_bundle_and_missing_store_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            with self.assertRaisesRegex(ValueError,"Unknown"): hydrate_history(checkout,index,"missing",store=store)
            with self.assertRaisesRegex(ValueError,"store"): hydrate_history(checkout,index,bundle["id"])
            bad=copy.deepcopy(index)
            bad["bundles"][0]["object_id"]=bundle["object_id"]
            with self.assertRaises(ValueError): hydrate_history(checkout,bad,bad["bundles"][0]["id"],store=store)

    def test_cli_help_and_check_missing_exit_code(self):
        help=subprocess.run([sys.executable,str(ROOT/'tools/hydrate_history.py'),'--help'],capture_output=True,text=True)
        self.assertEqual(help.returncode,0,help.stderr)
        with tempfile.TemporaryDirectory() as directory:
            repo, store, index, bundle, checkout = self.fixture(directory)
            index_path=Path(directory)/"index.json";index_path.write_text(json.dumps(index))
            report=Path(directory)/"report.json"
            result=subprocess.run([sys.executable,str(ROOT/'tools/hydrate_history.py'),'--checkout',str(checkout),'--index',str(index_path),
                                   '--store',str(store),'--group',bundle['id'],'--check','--report',str(report)],capture_output=True,text=True)
            self.assertEqual(result.returncode,1,result.stderr)
            self.assertEqual(json.loads(report.read_text())["state"],"absent")
            self.assertEqual(list(checkout.iterdir()),[])


if __name__ == '__main__': unittest.main()
