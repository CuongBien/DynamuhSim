"""Archive aliases keep old report consumers working without general symlink access."""
import os
from pathlib import Path
import tempfile
import unittest

from dynamuhsim.artifacts.hydrate import _target
from dynamuhsim.artifacts.layout import ARCHIVE, BENCHMARK, logical_report_name


class ReportLayoutTests(unittest.TestCase):
    def fixture(self, directory):
        checkout = Path(directory) / "checkout"; checkout.mkdir()
        archive = checkout / ARCHIVE / "camera_height_ab"; archive.mkdir(parents=True)
        alias = checkout / BENCHMARK / "camera_height_ab/reports"; alias.parent.mkdir(parents=True)
        alias.symlink_to(os.path.relpath(archive, alias.parent), target_is_directory=True)
        return checkout, archive, alias

    def test_approved_alias_reads_controls_and_publishes_under_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            checkout, archive, alias = self.fixture(directory)
            (archive / "freeze.json").write_bytes(b"unchanged frozen control")
            root, control = _target(checkout, str(BENCHMARK / "camera_height_ab/reports/freeze.json"))
            self.assertEqual((root, control), (checkout, archive / "freeze.json"))
            self.assertEqual(control.read_bytes(), (alias / "freeze.json").read_bytes())
            _, media = _target(checkout, str(BENCHMARK / "camera_height_ab/reports/new.png"))
            media.write_bytes(b"new approved media")
            self.assertEqual((archive / "new.png").read_bytes(), (alias / "new.png").read_bytes())

    def test_wrong_alias_and_symlinks_inside_archive_are_rejected(self):
        for mode in ["wrong_alias", "archive_symlink", "child_symlink", "alias_parent"]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                checkout, archive, alias = self.fixture(directory)
                outside = Path(directory) / "outside"; outside.mkdir()
                if mode == "wrong_alias":
                    alias.unlink(); alias.symlink_to(outside, target_is_directory=True)
                elif mode == "archive_symlink":
                    archive.rmdir(); archive.symlink_to(outside, target_is_directory=True)
                elif mode == "child_symlink":
                    (archive / "freeze.json").symlink_to(outside / "secret")
                else:
                    parent = alias.parent; saved = parent.with_name("saved"); parent.rename(saved)
                    parent.symlink_to(saved, target_is_directory=True)
                with self.assertRaisesRegex(ValueError, "symlink"):
                    _target(checkout, str(BENCHMARK / "camera_height_ab/reports/freeze.json"))

    def test_historical_support_alias_keeps_freeze_sources_and_report_hydration_separate(self):
        from dynamuhsim.artifacts.layout import SUPPORT
        for stage in ["gpu_runtime", "shutdown_fix"]:
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as directory:
                checkout = Path(directory) / "checkout"; checkout.mkdir()
                physical = checkout / SUPPORT / stage; physical.mkdir(parents=True)
                alias = checkout / BENCHMARK / stage; alias.parent.mkdir(parents=True)
                alias.symlink_to(os.path.relpath(physical, alias.parent), target_is_directory=True)
                _, old_source = _target(checkout, str(BENCHMARK / stage / "original.py"))
                self.assertEqual(old_source, physical / "original.py")
                _, report = _target(checkout, str(BENCHMARK / stage / "reports/run/result.json"))
                self.assertEqual(report, physical / "reports/run/result.json")
                alias.unlink(); alias.symlink_to(Path(directory) / "outside", target_is_directory=True)
                with self.assertRaisesRegex(ValueError, "symlink"):
                    _target(checkout, str(BENCHMARK / stage / "original.py"))

    def test_probe_restrictions_use_the_same_logical_names_for_archive_reads(self):
        logical = str(BENCHMARK / "depth_person_classifier/reports/test_opened.json")
        self.assertEqual(logical_report_name(str(ARCHIVE / "depth_person_classifier/test_opened.json")), logical)
        unknown = str(ARCHIVE / "unknown/test_opened.json")
        self.assertEqual(logical_report_name(unknown), unknown)


if __name__ == "__main__":
    unittest.main()
