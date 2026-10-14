#!/usr/bin/env python3
"""Paired ARM32 cost measurement with identical, explicit full collections."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import signal
import statistics
import subprocess
import sys
import time


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def json_line(prefix, text):
    found = re.findall(r'^' + re.escape(prefix) + r' (\{.*\})$', text, re.M)
    if len(found) != 1:
        raise RuntimeError(f'Expected one {prefix} record, found {len(found)}')
    return json.loads(found[0])


def validate_output(mode, text, count=18):
    work = json_line('MEM_WORK_JSON', text)
    calls = json_line('MEM_GC_JSON', text)
    audit = json_line('FULLGC_AUDIT', text)
    if len(work) != count or set(work.values()) != {3}:
        raise RuntimeError(f'Unexpected fixed work: {work}')
    expected = {'time_limit': 999999, 'full_frequency': 0,
                'automatic_disabled': 1, 'adaptive': 0,
                'incremental': int(mode == 'mprotect'),
                'actual_vdb': int(mode == 'mprotect')}
    for key, value in expected.items():
        if audit[key] != value:
            raise RuntimeError(f'{mode} control mismatch: {audit}')
    return {'work_counts': work, 'gc_calls': calls, 'gc_audit': audit,
            'work_signature': hashlib.sha256(json.dumps(work, sort_keys=True).encode()).hexdigest()}


def drivers(harness, out):
    octane = harness / 'test/octane'
    manifest = re.findall(r"load\(base_dir \+ '([^']+)'\);", (octane / 'run.js').read_text())
    if len(manifest) != 20 or manifest[0] != 'base.js':
        raise RuntimeError(f'Unexpected Octane manifest: {manifest}')
    loads = ['load(' + json.dumps(str(harness / 'tools/test/octane/base_mem.js')) + ');']
    loads += ['load(' + json.dumps(name) + ');' for name in manifest[1:]]
    octane_text = "benchmarkCheckpoint('baseline');\n" + '\n'.join(loads) + "\ngc(); gc();\n"
    octane_text += """benchmarkCheckpoint('loaded');
var fixedCounts = {}; var fixedGCCounts = {};
BenchmarkSuite.prototype.RunSingleBenchmark = function(benchmark, data) {
  var begin = new Date();
  for (var i = 0; i < 3; ++i) {
    benchmark.run(); fixedCounts[benchmark.name] = (fixedCounts[benchmark.name] || 0) + 1;
  }
  var elapsed = new Date() - begin;
  var rms = benchmark.rmsResult != null ? benchmark.rmsResult() : 0;
  this.NotifyStep(new BenchmarkResult(benchmark, elapsed * 1000 / 3, rms));
  return null;
};
BenchmarkSuite.RunSuites({
  NotifyStart: function(name) { print('OCTANE_START ' + name); },
  NotifyResult: function(name, score) {
    benchmarkCheckpoint('before_gc:' + name);
    gc(); gc(); fixedGCCounts[name] = (fixedGCCounts[name] || 0) + 2;
    print('OCTANE_RESULT ' + name + ' ' + score);
    benchmarkCheckpoint('test:' + name);
  },
  NotifyError: function(name, error) { throw Error(name + ': ' + error); },
  NotifyScore: function(score) { print('OCTANE_FIXED_SCORE ' + score); }
});
print('MEM_WORK_JSON ' + JSON.stringify(fixedCounts));
print('MEM_GC_JSON ' + JSON.stringify(fixedGCCounts));
benchmarkCheckpoint('workload_end');
gc(); gc();
benchmarkCheckpoint('after_gc');
"""
    wtb = harness / 'test/web-tooling-benchmark'
    if not (wtb / 'dist/memory.js').is_file():
        raise RuntimeError('The shared WTB memory bundle is missing')
    wtb_text = """var originalCheckpoint = benchmarkCheckpoint;
var fixedGCCounts = {};
benchmarkCheckpoint = function(label) {
  if (label === 'loaded' || label.indexOf('test:') === 0) {
    if (label !== 'loaded') originalCheckpoint('before_gc:' + label);
    gc(); gc(); fixedGCCounts[label] = (fixedGCCounts[label] || 0) + 2;
  }
  originalCheckpoint(label);
};
benchmarkCheckpoint('baseline');
load('dist/memory.js');
print('MEM_GC_JSON ' + JSON.stringify(fixedGCCounts));
benchmarkCheckpoint('workload_end');
gc(); gc();
benchmarkCheckpoint('after_gc');
"""
    result = {}
    for name, directory, contents in [('octane', octane, octane_text), ('web_tooling', wtb, wtb_text)]:
        memory = out / f'{name}-memory.js'
        perf = out / f'{name}-perf.js'
        memory.write_text(contents)
        perf.write_text('function benchmarkCheckpoint(label) {}\n' + contents)
        result[name] = (directory, memory, perf)
    return result


def timed_run(label, command, directory, env, script, out, cpu):
    stdout = out / f'{label}.stdout'
    stderr = out / f'{label}.stderr'
    counters = out / f'{label}.perf.csv'
    args = ['perf', 'stat', '-x,', '-o', str(counters), '-e',
            'cycles:u,cycles:k,instructions,task-clock,page-faults,minor-faults',
            'taskset', '-c', str(cpu), '/usr/bin/time', '-f', 'PEAK_KIB %M ELAPSED %e',
            *command, str(script)]
    started = last_update = time.monotonic()
    print('START', label, flush=True)
    with stdout.open('w') as log, stderr.open('w') as error:
        process = subprocess.Popen(args, cwd=directory, env=env, stdout=log, stderr=error)
        while process.poll() is None:
            now = time.monotonic()
            if now - last_update >= 30:
                print('PROGRESS', label, f'{now-started:.0f}s', flush=True)
                last_update = now
            time.sleep(0.25)
    text = stdout.read_text() + stderr.read_text()
    if process.returncode:
        raise RuntimeError(f'{label} failed: {text[-2000:]}')
    values = {}
    for line in counters.read_text().splitlines():
        columns = line.split(',')
        if len(columns) > 2 and columns[2] in ('cycles:u', 'cycles:k', 'instructions', 'task-clock', 'page-faults', 'minor-faults'):
            values[columns[2]] = float(columns[0])
    if not {'cycles:u', 'cycles:k', 'task-clock'} <= values.keys():
        raise RuntimeError(f'Missing native perf counters: {counters.read_text()}')
    values.update(wall_seconds=time.monotonic() - started,
                  peak_kib=int(re.search(r'PEAK_KIB (\d+)', text).group(1)))
    print('DONE', label, f'{(values["cycles:u"]+values["cycles:k"])/1e9:.3f} Gcycles', flush=True)
    return values, text


def run_cost_experiment():
    architecture = os.environ['BENCHMARK_ARCHITECTURE']
    out = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / f'matched-gc-cost-{architecture}'
    out.mkdir(parents=True, exist_ok=True)
    smoke = os.environ.get('MATCHED_COST_LOCAL_SMOKE') == '1'
    if architecture != 'arm32':
        write_json(out / 'measurements.json', {'skipped': True, 'target': 'arm32',
                                              'reason': 'The requested management-cost comparison targets ARM32'})
        print('ARM32 comparison complete; no ARM64 policy experiment requested', flush=True)
        return
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor_cpu = next(value for value in sorted(os.sched_getaffinity(0)) if value != cpu)
    command = [os.environ['ESCARGOT_ENGINE']]
    env = dict(os.environ)
    for key in ('GC_ENABLE_INCREMENTAL', 'GC_DISABLE_INCREMENTAL', 'GC_PRINT_STATS',
                'GC_PRINT_VERBOSE_STATS', 'GC_FULL_FREQUENCY', 'GC_PAUSE_TIME_TARGET', 'GC_PROFILE_MPROTECT'):
        env.pop(key, None)
    env.update(GC_USE_GETWRITEWATCH='0', GC_FREE_SPACE_DIVISOR='12',
               GC_INCREMENTAL_START_THRESHOLD='10M', ESCARGOT_GC_TIME_UNLIMITED='1',
               ESCARGOT_GC_FULL_EVERY_COLLECTION='1', ESCARGOT_GC_DISABLE_AUTOMATIC='1',
               ESCARGOT_FULLGC_AUDIT='1', LC_ALL='C.UTF-8', LANG='C.UTF-8')
    modes = [('ordinary', command), ('mprotect', [*command, '--gc-incremental'])]
    data = {'architecture': architecture, 'local_smoke': smoke, 'host': platform.node(),
            'vm_max_map_count': int(Path('/proc/sys/vm/max_map_count').read_text()),
            'compiled_vma_soft_limit': 250000,
            'cpu': cpu, 'monitor_cpu': monitor_cpu, 'protocol': {
                'automatic_gc': False, 'time_limit': 999999, 'full_frequency': 0,
                'free_space_divisor': 12, 'incremental_start_threshold': '10M',
                'work_iterations': 3, 'perf_and_memory_separate': True,
                'memory_interval_ms': 250}, 'perf': [], 'memory': [], 'probes': {}}
    for mode, args in modes:
        probe = subprocess.run(['taskset', '-c', str(cpu), *args, '-e',
                                'var a=[]; for(var i=0;i<500000;i++) a.push({x:i}); gc(); gc(); print(a.length);'],
                               env={**env, 'GC_PRINT_VERBOSE_STATS': '1'}, capture_output=True, text=True)
        text = probe.stdout + probe.stderr
        (out / f'{mode}-full-gc-probe.log').write_text(text)
        if probe.returncode:
            raise RuntimeError(f'{mode} probe failed: {text[-2000:]}')
        audit = json_line('FULLGC_AUDIT', text)
        expected = {'time_limit': 999999, 'full_frequency': 0, 'automatic_disabled': 1,
                    'adaptive': 0, 'incremental': int(mode == 'mprotect'), 'actual_vdb': int(mode == 'mprotect')}
        if any(audit[k] != v for k, v in expected.items()):
            raise RuntimeError(f'Unmatched probe controls: {audit}')
        full_count = text.count('Initiating full world-stop collection!')
        if full_count != audit['collections']:
            raise RuntimeError(f'Non-full collection detected: {full_count}, {audit}')
        data['probes'][mode] = {**audit, 'full_collection_count': full_count}
    fixtures = drivers(harness, out)
    if smoke:
        fixtures = {'octane': fixtures['octane']}
    data['driver_sha256'] = {name: hashlib.sha256(memory.read_bytes()).hexdigest()
                            for name, (_, memory, _) in fixtures.items()}
    data['wtb_bundle_sha256'] = hashlib.sha256((harness / 'test/web-tooling-benchmark/dist/memory.js').read_bytes()).hexdigest()
    repetitions = 1 if smoke else int(os.environ.get('BENCHMARK_REPETITIONS', '3'))
    def check_pair(rows):
        for suite in fixtures:
            selected = [row for row in rows if row['suite'] == suite]
            if selected:
                keys = [(row['work_counts'], row['gc_calls'], row['gc_audit']['collections']) for row in selected]
                if any(value != keys[0] for value in keys):
                    raise RuntimeError(f'{suite}: different work or full-GC counts between modes')
    for rep in range(repetitions):
        for mode, args in (modes if rep % 2 == 0 else modes[::-1]):
            for suite, (directory, _, script) in fixtures.items():
                label = f'perf-{suite}-{mode}-{rep}'
                values, text = timed_run(label, args, directory, env, script, out, cpu)
                data['perf'].append({'suite': suite, 'mode': mode, 'rep': rep,
                                     **values, **validate_output(mode, text)})
                check_pair(data['perf'])
                write_json(out / 'measurements.json', data)
    if not smoke:
        spec = importlib.util.spec_from_file_location('cost_memory', harness / 'tools/engine_memory_benchmark.py')
        memory = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(memory)
        memory.INTERVAL_SECONDS = 0.25
        original = memory.frozen_snapshot
        def snapshot(pid, cpu):
            values, _, status = original(pid, cpu)
            smaps = Path(f'/proc/{pid}/smaps').read_text()
            values['vma_count'] = len(re.findall(r'^[0-9a-f]+-[0-9a-f]+ ', smaps, re.M))
            values['pte_kib'] = int(re.search(r'^VmPTE:\s+(\d+)', status, re.M).group(1))
            values['anonymous_kib'] = sum(map(int, re.findall(r'^Anonymous:\s+(\d+)', smaps, re.M)))
            return values, smaps, status
        memory.frozen_snapshot = snapshot
        for rep in range(repetitions):
            for mode, args in (modes if rep % 2 == 0 else modes[::-1]):
                for suite, (directory, script, _) in fixtures.items():
                    label = f'memory-{suite}-{mode}-{rep}'
                    result = memory.measure_memory(label, 'escargot', args, directory, env,
                                                   script, out, cpu, monitor_cpu)
                    result['sampling_interval_ms'] = 250
                    text = (out / result['log']).read_text()
                    data['memory'].append({'suite': suite, 'mode': mode, 'rep': rep,
                                           **result, **validate_output(mode, text)})
                    check_pair(data['memory'])
                    write_json(out / 'measurements.json', data)
    data['validated'] = True
    write_json(out / 'measurements.json', data)
    for suite in fixtures:
        ordinary = [row for row in data['perf'] if row['suite'] == suite and row['mode'] == 'ordinary']
        protected = [row for row in data['perf'] if row['suite'] == suite and row['mode'] == 'mprotect']
        base = statistics.median(row['cycles:u'] + row['cycles:k'] for row in ordinary)
        current = statistics.median(row['cycles:u'] + row['cycles:k'] for row in protected)
        print('MATCHED_COST_RESULT', suite, f'CPU overhead {(current/base-1)*100:.3f}%', flush=True)


def main():
    if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm32':
        return run_cost_experiment()
    original = int(Path('/proc/sys/vm/max_map_count').read_text())
    required = 1000000
    changed = original < required
    out = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / 'matched-gc-cost-arm32'
    out.mkdir(parents=True, exist_ok=True)
    def interrupted(signum, frame):
        raise KeyboardInterrupt()
    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        if changed:
            subprocess.run(['sudo', '-n', 'sysctl', '-w', f'vm.max_map_count={required}'], check=True)
        effective = int(Path('/proc/sys/vm/max_map_count').read_text())
        if effective < required:
            raise RuntimeError(f'Insufficient kernel VMA budget: {effective}')
        write_json(out / 'kernel-budget.json', {'original': original, 'effective': effective,
                   'compiled_gc_soft_limit': 250000, 'changed': changed})
        run_cost_experiment()
    finally:
        if changed:
            subprocess.run(['sudo', '-n', 'sysctl', '-w', f'vm.max_map_count={original}'], check=True)
            print('Restored vm.max_map_count', original, flush=True)
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    main()
