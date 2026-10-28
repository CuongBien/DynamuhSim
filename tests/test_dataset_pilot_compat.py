"""The old pilot entrypoints must reuse the frozen WM exporter without drift."""
import hashlib
import json
from pathlib import Path
from dynamuhsim.workspace import stage_directory,source_path
import runpy
import subprocess
import tempfile
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
PILOT = stage_directory(ROOT,'dataset_pilot')
BATCH = PILOT.parent / "wm_episode_batch"


class DatasetPilotCompatTests(unittest.TestCase):
    def test_math_exports_are_the_same_function_objects_as_the_frozen_engine(self):
        from experiments.social_navigation.datasets.dataset_pilot import dataset_core as current
        from experiments.social_navigation.datasets.wm_episode_batch.tools.legacy import dataset_core as frozen
        for name in current.__all__:
            with self.subTest(function=name):
                self.assertIs(getattr(current, name), getattr(frozen, name))

    def test_old_cli_and_import_entrypoints_forward_name_arguments_and_exports(self):
        for name in ("build", "audit"):
            for module_name in ("__main__", "pilot_compat_test"):
                with self.subTest(script=name, module_name=module_name):
                    path = PILOT / (name + ".py")
                    argv = [str(path), "--episode", "/fixture/episode with spaces", "--help"]
                    api = object()
                    original_path = sys.path[:]
                    with patch.object(sys, "argv", argv), patch.object(runpy, "run_path", return_value={"exported_api": api}) as run:
                        namespace = dict(__name__=module_name, __file__=str(path))
                        exec(compile(path.read_text(), str(path), "exec"), namespace)
                        run.assert_called_once_with(str(BATCH / "tools/legacy" / (name + ".py")), run_name=module_name)
                        self.assertEqual(sys.argv, argv)
                        self.assertIs(namespace["exported_api"], api)
                    self.assertEqual(sys.path, original_path)

    def test_entrypoint_restores_import_path_on_engine_failure(self):
        for name in ("build", "audit"):
            path = PILOT / (name + ".py")
            original_path = sys.path[:]
            with patch.object(runpy, "run_path", side_effect=RuntimeError("engine failed")):
                with self.assertRaisesRegex(RuntimeError, "engine failed"):
                    exec(compile(path.read_text(), str(path), "exec"), dict(__name__="__main__", __file__=str(path)))
            self.assertEqual(sys.path, original_path)

    def test_frozen_engine_still_matches_all_five_export_pins(self):
        frozen = json.loads((BATCH / "export_frozen.json").read_text())["source_sha256"]
        pins = {name: digest for name, digest in frozen.items() if "/tools/legacy/" in name}
        self.assertEqual(len(pins), 5)
        for name, digest in pins.items():
            self.assertEqual(hashlib.sha256(source_path(ROOT,name).read_bytes()).hexdigest(), digest, name)

    def test_math_shim_rejects_changed_dependency_even_if_its_own_hash_stays_pinned(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            engine = root / BATCH.relative_to(ROOT) / "tools/legacy/dataset_core.py"
            engine.parent.mkdir(parents=True)
            engine.write_text("changed algorithm")
            wrapper = root / PILOT.relative_to(ROOT) / "dataset_core.py"
            with self.assertRaisesRegex(RuntimeError, "Frozen normalizer math changed"):
                exec(compile((PILOT / "dataset_core.py").read_text(), str(wrapper), "exec"), dict(__file__=str(wrapper)))

    def test_existing_contract_suite_passes_through_compatibility_imports(self):
        result = subprocess.run([sys.executable, str(PILOT / "test_contract.py")], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Ran 8 tests", result.stderr)


if __name__ == "__main__":
    unittest.main()
