#!/usr/bin/env python3
# Temporary remote-only diagnostic; the PR branch does not contain this file.
import os
from pathlib import Path

harness = Path(os.environ['BENCHMARK_HARNESS'])
experiment = harness / 'tools/arm-v8-lite-experiment.py'
source = experiment.read_text()
anchor = '    drivers = monthly.build_memory_drivers(wtb)'
assert source.count(anchor) == 1
source = source.replace(anchor, anchor + "\n    drivers = {'octane': drivers['octane']}")
namespace = {'__name__': 'arm_octane_gc_diagnostic', '__file__': str(experiment)}
exec(compile(source, str(experiment), 'exec'), namespace)
namespace['main'](mode='memory', engine_names=('escargot',))
