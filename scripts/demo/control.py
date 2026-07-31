#!/usr/bin/env python3

import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SCRIPTS_ROOT = ROOT.parent
sys.path.insert(0, str(SCRIPTS_ROOT))
os.environ["ENERGY_RACE_CONFIG"] = str(ROOT / "mission_config.json")

from control import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
