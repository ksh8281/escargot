#!/usr/bin/env python3
"""Measure bounded retries after consecutive abandoned minor collections."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess

if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
    print('Abandoned-minor comparison targets ARM64.', flush=True)
    raise SystemExit(0)
source = Path.cwd()
spec = importlib.util.spec_from_file_location('comparison', source / 'tools/arm-cold-epoch-compare.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)
for name, expected in comparison.CONFIG['expected_source_files_sha256'].items():
    assert comparison.sha256(source / name) == expected, name
patch = source / 'tools/arm64-gc-abort-backoff.patch'
gc = source / 'third_party/GCutil'
subprocess.run(['git', 'apply', '--check', str(patch)], cwd=gc, check=True)
subprocess.run(['git', 'apply', str(patch)], cwd=gc, check=True)
engine = Path(os.environ['ESCARGOT_ENGINE']).resolve()
subprocess.run(['cmake', '--build', str(engine.parent), '-j4'], check=True)
report = Path(os.environ['BENCHMARK_OUTPUT_DIR'])
shutil.copy2(patch, report / 'abort-backoff.patch')
shutil.copy2(engine, report / 'abort-backoff-engine')
comparison.CONFIG['expected_source_files_sha256']['third_party/GCutil/alloc.c'] = comparison.sha256(gc / 'alloc.c')
comparison.CONFIG['experiment_patch_sha256'] = comparison.sha256(patch)
comparison.CONFIG['settings']['env']['GC_BENCH_ABORT_BACKOFF'] = '1'
comparison.CONFIG['scope'] = (
    'Native ARM64 abandoned-minor counterfactual. Apply the recorded patch to '
    'the accepted GC revision; after two consecutive failed minor attempts, '
    'use the existing bounded admission closure and tracked-full rearm. '
    'Preserve every other accepted setting. Three alternating timing pairs '
    'against frozen master full GC, plus separate ABBA memory. No correctness '
    'fix for the original intermittent JSHint failure is claimed.')
comparison.main()
