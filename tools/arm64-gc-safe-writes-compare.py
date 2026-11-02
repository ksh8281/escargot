#!/usr/bin/env python3
"""Measure ordinary dirty tracking without ARM64 read-only store emulation."""
import importlib.util
import os
from pathlib import Path

if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
    print('Read-only store comparison targets ARM64.', flush=True)
    raise SystemExit(0)

path = Path(__file__).with_name('arm-cold-epoch-compare.py')
spec = importlib.util.spec_from_file_location('comparison', path)
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)
comparison.CONFIG['settings']['env']['GC_BENCH_NUMERIC_RO'] = '0'
comparison.CONFIG['scope'] = (
    'Native ARM64 counterfactual with read-only store emulation disabled. '
    'Use ordinary dirty-page tracking while preserving the accepted FSD14 '
    'and all other settings. Three alternating timing pairs against frozen '
    'master full GC, plus separate ABBA memory. The original intermittent '
    'JSHint failure remains unresolved; passing this run does not prove a fix.')
comparison.main()
