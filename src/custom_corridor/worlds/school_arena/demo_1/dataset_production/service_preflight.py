"""Prepare the dedicated HuNav container before the persistent user service starts."""
from __future__ import annotations

import logging

from dataset_runner.hunav_manager import DockerHuNavRuntime, HuNavConfig
from dataset_runner.scripts.smoke_runtime_integration import HuNavPreflight


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    config = HuNavConfig.from_yaml()
    runtime = DockerHuNavRuntime(config)
    HuNavPreflight(config, runtime).check(reset_container=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
