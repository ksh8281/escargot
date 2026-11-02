#!/usr/bin/env python3
"""Attribute ARM64 collector costs without changing the accepted runtime."""
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess

if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
    print('GC phase profiling targets ARM64.', flush=True)
    raise SystemExit(0)
source = Path.cwd()
report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / 'gc-phase-profile'
report.mkdir(exist_ok=True)
spec = importlib.util.spec_from_file_location('comparison', source / 'tools/arm-cold-epoch-compare.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)
for name, expected in comparison.CONFIG['expected_source_files_sha256'].items():
    assert comparison.sha256(source / name) == expected, name
engine = Path(os.environ['ESCARGOT_ENGINE']).resolve()
shutil.copy2(engine, report / 'profiled-engine')
octane = Path(os.environ['BENCHMARK_HARNESS']) / 'test/octane'
assert comparison.sha256(octane / 'run.js') == 'b4b10b05e3ccefd6f49d57763ec67ef746c3e7c093ed0fdd602c7f126af5c12f'
text = (octane / 'run.js').read_text()
needle = 'NotifyResult: PrintResult,'
assert text.count(needle) == 1
driver = report / 'phase-run.js'
driver.write_text(text.replace(needle, "NotifyStart: function(name){print('PHASE_START '+name);}, " + needle))
clean = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
clean.update(ESCARGOT_GC_CYCLE_TRACE='1', ESCARGOT_BENCH_SEED='1', LD_PRELOAD='')
cpu = int(os.environ['BENCHMARK_CPU'])
monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
data = {'scope': 'Native ARM64 cost attribution with cycle tracing and sampling; not baseline performance evidence.',
        'binary_sha256': comparison.sha256(engine), 'configuration': comparison.CONFIG,
        'source': comparison.output(['git', 'rev-parse', 'HEAD']), 'cpu': cpu, 'cases': [], 'perf_setup': []}
restore = {}
perf = shutil.which('perf')
try:
    if perf:
        for key, value in [('kernel.perf_event_paranoid', '-1'), ('kernel.kptr_restrict', '0')]:
            old = subprocess.run(['sysctl', '-n', key], capture_output=True, text=True)
            if old.returncode == 0:
                change = subprocess.run(['sudo', '-n', 'sysctl', '-w', key + '=' + value], capture_output=True, text=True)
                data['perf_setup'].append({'key': key, 'returncode': change.returncode, 'output': change.stdout + change.stderr})
                if change.returncode == 0:
                    restore[key] = old.stdout.strip()
        probe = subprocess.run([perf, 'stat', '-a', '-C', str(cpu), '-e', 'cpu-clock', '--', 'true'], capture_output=True, text=True)
        data['perf_setup'].append({'probe_code': probe.returncode, 'output': probe.stdout + probe.stderr})
        if probe.returncode:
            perf = None
    cases = [('full12', False, {'GC_FREE_SPACE_DIVISOR': '12'}),
             ('accepted', True, {}),
             ('numeric-off', True, {'GC_BENCH_NUMERIC_RO': '0'}),
             ('batch-four', True, {'GC_BENCH_FAULT_BATCH_PAGES': '4'})]
    for label, gen, overrides in cases:
        env = {**clean, **(comparison.CONFIG['settings']['env'] if gen else {}), **overrides}
        sampler = None
        with (report / (label + '.perf.log')).open('w') as perf_log:
            try:
                if perf:
                    sampler = subprocess.Popen(['taskset', '-c', str(monitor), perf, 'record',
                        '-a', '-C', str(cpu), '-e', 'cpu-clock', '-F', '199',
                        '--call-graph', 'dwarf,4096', '-o', str(report / (label + '.data'))],
                        stdout=perf_log, stderr=perf_log)
                row, contents = comparison.process_measure(label, 'candidate' if gen else 'original',
                    [str(engine)] + (comparison.CONFIG['settings']['flags'] if gen else []),
                    octane, driver, env, report, cpu, monitor, gen)
                comparison.score_result('octane', row, contents)
                phases = {}
                current = None
                for line in contents.splitlines():
                    if line.startswith('PHASE_START '):
                        current = line.split(' ', 1)[1]
                    elif current and line.startswith(current + ': '):
                        current = None
                    match = re.match(r'GC_CYCLE_(MARK|RECLAIM) (.*)', line)
                    if current and match:
                        event = json.loads(match[2])
                        key = ('full' if event['full'] else 'minor') + '_' + match[1].lower()
                        totals = phases.setdefault(current, {}).setdefault(key,
                            {'count': 0, 'ms': 0, 'allocated': 0, 'new_live_estimate': 0, 'abandoned': 0})
                        for field in totals:
                            totals[field] += 1 if field == 'count' else event.get(field, 0)
                row['phases'] = phases
                data['cases'].append(row)
                print('PHASE SUMMARY', label, json.dumps(phases), flush=True)
            finally:
                if sampler:
                    if sampler.poll() is None:
                        sampler.send_signal(signal.SIGINT)
                    sampler.wait(timeout=30)
                    if sampler.returncode in (0, -signal.SIGINT, 128 + signal.SIGINT):
                        with (report / (label + '.perf.txt')).open('w') as output:
                            subprocess.run([perf, 'report', '--stdio', '--comms', engine.name,
                                '--percent-limit', '0.2', '-i', str(report / (label + '.data'))],
                                stdout=output, stderr=subprocess.STDOUT, check=True)
                    else:
                        raise RuntimeError('perf record failed: ' + str(sampler.returncode))
            comparison.write_json(report / 'profile.json', data)
finally:
    for key, value in restore.items():
        result = subprocess.run(['sudo', '-n', 'sysctl', '-w', key + '=' + value], capture_output=True, text=True)
        data['perf_setup'].append({'restore': key, 'returncode': result.returncode, 'output': result.stdout + result.stderr})
    comparison.write_json(report / 'profile.json', data)
print('GC PHASE PROFILE COMPLETE', flush=True)
