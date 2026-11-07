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
