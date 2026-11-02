#!/usr/bin/env python3
"""Compare completed traces with a fresh root traversal on native ARM64."""
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

source = Path.cwd()
report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / 'gc-closure'
report.mkdir(exist_ok=True)
if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
    print('Root-closure diagnostics target ARM64.', flush=True)
    raise SystemExit(0)
spec = importlib.util.spec_from_file_location('comparison', source / 'tools/arm-cold-epoch-compare.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)
config = comparison.CONFIG
for name, expected in config['expected_source_files_sha256'].items():
    assert comparison.sha256(source / name) == expected, name
engine = Path(os.environ['ESCARGOT_ENGINE']).resolve()
original_hash = comparison.sha256(engine)
patch = source / 'tools/arm64-gc-closure.patch'
subprocess.run(['git', 'apply', '--check', str(patch)], cwd=source / 'third_party/GCutil', check=True)
subprocess.run(['git', 'apply', str(patch)], cwd=source / 'third_party/GCutil', check=True)
print('CLOSURE BUILD START', flush=True)
subprocess.run(['cmake', '--build', str(engine.parent), '-j4'], check=True)
print('CLOSURE BUILD COMPLETE', flush=True)
wtb = Path(os.environ['BENCHMARK_HARNESS']) / 'test/web-tooling-benchmark'
bundle = wtb / 'dist/memory.js'
assert comparison.sha256(bundle) == 'd7b797cd37f85de78509c8fc374ea12c653828e14eda6d9d5fd60d67a88633bf'
driver = report / 'work.js'
driver.write_text("var benchmarkCheckpoint=function(){};\nvar workStart=Date.now();\nload('dist/memory.js');\nprint('WTB_FIXED_WORK_MS '+(Date.now()-workStart));\n")
clean = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
clean.update(LD_PRELOAD='', ESCARGOT_BENCH_SEED='1', GC_VERIFY_MINOR='1')
cpu = int(os.environ['BENCHMARK_CPU'])
monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
data = {'scope': 'Fresh root closure diagnostics only; free-list objects excluded. Timings are not performance results.',
        'source': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'original_binary_sha256': original_hash, 'verified_binary_sha256': comparison.sha256(engine),
        'patch_sha256': comparison.sha256(patch), 'configuration': config, 'cases': []}
for index in range(12):
    full = index % 4 == 3
    label = f'{index+1:02d}-' + ('full14' if full else 'accepted')
    env = {**clean, **({'GC_FREE_SPACE_DIVISOR': '14'} if full else config['settings']['env'])}
    row = {'label': label, 'full': full}
    command = [str(engine)] + ([] if full else config['settings']['flags'])
    try:
        result, contents = comparison.process_measure(label, 'candidate', command, wtb,
            driver, env, report, cpu, monitor, not full)
        counts = json.loads(re.search(r'^MEM_WORK_JSON (.*)$', contents, re.M).group(1))
        assert len(counts) == 18 and set(counts.values()) == {3}, counts
        row.update(passed=True, counts=counts, audit=result['mode_audit'])
    except Exception as error:
        contents = (report / (label+'.log')).read_text(errors='replace')
        row.update(passed=False, error=repr(error), evidence=[x for x in contents.splitlines()
                   if x.startswith(('SHADOW_PHASE', 'SHADOW_MISSING', 'SOURCE_WORD', 'TARGET_FIRST', 'Uncaught'))])
        shutil.copy2(engine, report / 'failed-engine')
    data['cases'].append(row)
    (report / 'closure.json').write_text(json.dumps(data, indent=2)+'\n')
    print('CLOSURE RESULT', json.dumps(row), flush=True)
    if not row['passed']:
        break
print('CLOSURE COMPLETE', flush=True)
