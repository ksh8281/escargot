#!/usr/bin/env python3
"""Measure the rebased collector with the control's allocation divisor."""
import importlib.util
import os
from pathlib import Path

if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
    print('Allocation-divisor comparison targets ARM64.', flush=True)
    raise SystemExit(0)

path = Path(__file__).with_name('arm-cold-epoch-compare.py')
spec = importlib.util.spec_from_file_location('comparison', path)
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)
comparison.CONFIG['settings']['env']['GC_FREE_SPACE_DIVISOR'] = '12'
comparison.CONFIG['scope'] = (
    'Native ARM64 allocation-divisor counterfactual: current master full GC '
    'versus rebased major/minor GC, both with free-space divisor 12. Preserve '
    'all other accepted settings and the alternating three-pair timing and '
    'separate ABBA memory protocols. This is an experiment, not an adopted '
    'replacement for the FSD14 baseline.')
comparison.main()
