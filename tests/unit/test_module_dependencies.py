import ast
import importlib
import pkgutil
import sys
from pathlib import Path

import narrow_nav.core


def test_core_modules_import_only_stdlib_or_core():
    names = [module.name for module in pkgutil.iter_modules(narrow_nav.core.__path__)]
    assert names
    for name in names:
        module = importlib.import_module(f"narrow_nav.core.{name}")
        tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(
                    alias.name.split(".")[0] in sys.stdlib_module_names
                    for alias in node.names
                )
            elif isinstance(node, ast.ImportFrom):
                if node.level == 1:
                    assert node.module in names
                else:
                    assert node.level == 0
                    assert node.module.split(".")[0] in sys.stdlib_module_names


def test_human_backends_do_not_import_the_physics_engine_or_adapter():
    human_dir = Path(narrow_nav.core.__file__).resolve().parent.parent / "humans"
    for file in human_dir.rglob("*.py"):
        for node in ast.walk(ast.parse(file.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                modules = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                modules = [node.module or ""]
                if not node.module or node.module == "narrow_nav":
                    modules.extend(alias.name for alias in node.names)
            else:
                continue
            assert not any(
                module.split(".")[0] in ("mujoco", "simulation")
                or module.startswith("narrow_nav.simulation")
                for module in modules
            ), file
