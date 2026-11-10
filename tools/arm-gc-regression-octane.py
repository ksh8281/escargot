#!/usr/bin/env python3
"""Run the stock Octane gate before the complete regression comparison."""
import importlib.util
from pathlib import Path
spec = importlib.util.spec_from_file_location('repair', Path(__file__).with_name('arm-gc-regression-fix.py'))
repair = importlib.util.module_from_spec(spec)
spec.loader.exec_module(repair)
repair.main(only_octane=True)
