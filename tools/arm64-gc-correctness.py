#!/usr/bin/env python3
"""Repeat the frozen WTB input and preserve failures for collector diagnosis."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

source = Path.cwd()
report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / 'gc-correctness'
report.mkdir(exist_ok=True)
if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
    print('ARM64 correctness investigation; skipping ARM32 measurements.', flush=True)
    raise SystemExit(0)
spec = importlib.util.spec_from_file_location('comparison', source / 'tools/arm-cold-epoch-compare.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)
config = comparison.CONFIG
for name, expected in config['expected_source_files_sha256'].items():
    assert comparison.sha256(source / name) == expected, name
wtb = Path(os.environ['BENCHMARK_HARNESS']) / 'test/web-tooling-benchmark'
bundle = wtb / 'dist/memory.js'
assert comparison.sha256(bundle) == 'd7b797cd37f85de78509c8fc374ea12c653828e14eda6d9d5fd60d67a88633bf'
text = bundle.read_text()
needle = '''      } else {
        throw err;
      }
    }

    // Loop over the listed "internals"'''
assert text.count(needle) == 1
instrumented = report / 'memory-catch.js'
instrumented.write_text(text.replace(needle, needle.replace('        throw err;',
    "        print('JSHINT_ORIGINAL_STACK '+err.stack);\n        throw err;")))
original = report / 'original-work.js'
original.write_text("var benchmarkCheckpoint=function(){};\nvar workStart=Date.now();\nload('dist/memory.js');\nprint('WTB_FIXED_WORK_MS '+(Date.now()-workStart));\n")
catch = report / 'catch-work.js'
catch.write_text(original.read_text().replace("'dist/memory.js'", json.dumps(str(instrumented))))
clean = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
clean.update(LD_PRELOAD='', ESCARGOT_BENCH_SEED='1')
engine = os.environ['ESCARGOT_ENGINE']
data = {'scope': 'Correctness repetitions only; no performance conclusions.',
        'source': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'binary_sha256': comparison.sha256(engine), 'configuration': config,
        'bundle_sha256': comparison.sha256(bundle), 'cases': []}
# Original driver first; catch-only logging then records the original throw site.
# Periodic full-GC controls use the same candidate binary and free-space divisor.
for index in range(24):
    full = index % 6 == 5
    diagnostic = index >= 12
    label = f'{index+1:02d}-' + ('full14' if full else 'accepted') + ('-catch' if diagnostic else '-original')
    env = {**clean, **({'GC_FREE_SPACE_DIVISOR': '14'} if full else config['settings']['env'])}
    command = [engine] + ([] if full else config['settings']['flags'])
    row = {'label': label, 'full': full, 'catch_logging': diagnostic}
    try:
        result, contents = comparison.process_measure(label, 'candidate', command, wtb,
            catch if diagnostic else original, env, report, int(os.environ['BENCHMARK_CPU']),
            next(x for x in sorted(os.sched_getaffinity(0)) if x != int(os.environ['BENCHMARK_CPU'])), not full)
        counts = json.loads(re.search(r'^MEM_WORK_JSON (.*)$', contents, re.M).group(1))
        assert len(counts) == 18 and set(counts.values()) == {3}, counts
        row.update(passed=True, counts=counts, audit=result['mode_audit'])
    except Exception as error:
        row.update(passed=False, error=repr(error))
        shutil.copy2(engine, report / 'failed-engine')
    data['cases'].append(row)
    (report / 'correctness.json').write_text(json.dumps(data, indent=2)+'\n')
    print('CORRECTNESS', json.dumps(row), flush=True)
    if sum(not case['passed'] for case in data['cases']) >= 4:
        break
shutil.copy2(bundle, report / 'memory.js')
print('CORRECTNESS COMPLETE', flush=True)
if any(not case['passed'] for case in data['cases']):
    raise SystemExit(1)
