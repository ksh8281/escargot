#!/usr/bin/env python3
"""Reproduce the ARM64 WTB failure without using these runs as scores."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

source = Path.cwd()
report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / 'gc-wtb-diagnosis'
report.mkdir(exist_ok=True)
if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
    print('ARM32 baseline already complete; this diagnostic targets ARM64.', flush=True)
    raise SystemExit(0)
config = json.loads((source / 'tools/arm-cold-epoch-settings.json').read_text())
for name, expected in config['expected_source_files_sha256'].items():
    assert hashlib.sha256((source / name).read_bytes()).hexdigest() == expected, name
wtb = Path(os.environ['BENCHMARK_HARNESS']) / 'test/web-tooling-benchmark'
bundle = wtb / 'dist/memory.js'
shutil.copy2(bundle, report / 'memory.js')
lines = bundle.read_text().splitlines()
for line in [352504, 346950, 178679]:
    (report / f'source-{line}.txt').write_text('\n'.join(
        f'{i+1}: {lines[i]}' for i in range(line-12, line+10)) + '\n')
driver = report / 'work.js'
driver.write_text("var benchmarkCheckpoint=function(s){print('WORK_PHASE '+s);};\n"
                  "load('dist/memory.js');\nprint('DIAGNOSTIC_COMPLETE');\n")
clean = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
clean.update(LD_PRELOAD='', ESCARGOT_BENCH_SEED='1')
cases = [
    ('accepted-a', True, {}),
    ('accepted-b', True, {}),
    ('full14', False, {'GC_FREE_SPACE_DIVISOR': '14'}),
    ('numeric-off', True, {'GC_BENCH_NUMERIC_RO': '0', 'GC_BENCH_NUMERIC_RO_OR': '0'}),
    ('manual-off', True, {'GC_BENCH_MANUAL_CODEBLOCK': '0'}),
    ('sparse-off', True, {'GC_GENERATIONAL_SPARSE_SOURCES': '0'}),
    ('source-batch-off', True, {'GC_BENCH_SOURCE_BATCH': '0'}),
]
results = {'scope': 'Correctness diagnostics only; no performance comparisons.',
           'source': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
           'configuration': config, 'bundle_sha256': hashlib.sha256(bundle.read_bytes()).hexdigest(),
           'cases': []}
for label, generational, overrides in cases:
    env = {**clean, **(config['settings']['env'] if generational else {}), **overrides}
    command = ['taskset', '-c', os.environ['BENCHMARK_CPU'], os.environ['ESCARGOT_ENGINE']]
    if generational:
        command += config['settings']['flags']
    command.append(str(driver))
    print('DIAGNOSTIC START', label, flush=True)
    start = time.monotonic()
    with (report / (label + '.log')).open('w') as stream:
        process = subprocess.Popen(command, cwd=wtb, env=env, stdout=stream, stderr=subprocess.STDOUT)
        try:
            while True:
                try:
                    code = process.wait(timeout=30)
                    break
                except subprocess.TimeoutExpired:
                    print('DIAGNOSTIC PROGRESS', label, round(time.monotonic()-start), flush=True)
                    if time.monotonic()-start > 600:
                        process.kill()
                        code = process.wait()
                        break
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    contents = (report / (label + '.log')).read_text(errors='replace')
    row = {'case': label, 'exit': code, 'completed': 'DIAGNOSTIC_COMPLETE' in contents,
           'overrides': overrides, 'generational': generational,
           'work_phases': [x for x in contents.splitlines() if x.startswith('WORK_PHASE ')],
           'errors': [x for x in contents.splitlines() if x.startswith('Uncaught ')]}
    results['cases'].append(row)
    (report / 'diagnosis.json').write_text(json.dumps(results, indent=2)+'\n')
    print('DIAGNOSTIC RESULT', json.dumps(row), flush=True)
