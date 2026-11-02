#!/usr/bin/env python3
"""Complete the ARM64 comparison using the original measurement protocol."""
import os
from pathlib import Path
import runpy

if os.environ['BENCHMARK_ARCHITECTURE'] == 'arm64':
    runpy.run_path(str(Path(__file__).with_name('arm-cold-epoch-compare.py')),
                  run_name='__main__')
else:
    print('ARM32 comparison retained from run 41372172891.', flush=True)
