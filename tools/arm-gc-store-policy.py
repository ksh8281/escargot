#!/usr/bin/env python3
"""Measure numeric write policies on the native ARM benchmark host."""
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import struct

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('regression', HERE / 'arm-gc-regression-fix.py')
regression = importlib.util.module_from_spec(spec)
spec.loader.exec_module(regression)
policy, common = regression.p, regression.c
MODES = {'full': 3, 'on': 3, 'same': 4, 'off': 0}
WTB_ORDERS = [('full', 'on', 'same', 'off'), ('same', 'off', 'on', 'full'),
              ('off', 'same', 'full', 'on')]
GUARD_ORDERS = [('on', 'same', 'off', 'full'), ('same', 'full', 'on', 'off'),
                ('off', 'on', 'full', 'same')]


def json_line(contents, name):
    return json.loads(re.search(r'^' + name + r' (.+)$', contents, re.M)[1])


def audit(contents, mode, require_active):
    events = policy.a.audits(contents)
    gc = events['GENERATIONAL_AUDIT']
    numeric = json_line(contents, 'NUMERIC_RO_AUDIT')
    manual = json_line(contents, 'MANUAL_KIND_AUDIT')
    assert events['PHONE_MODE_AUDIT']['free_space_divisor'] == 12
    if mode == 'full':
        assert gc['minor'] == 0 and gc['incremental'] == 0
    else:
        assert gc['full_frequency'] == 2 and numeric[0] == MODES[mode]
        assert events['ALLOCATION_PATTERN_AUDIT']['mode'] == 4
        if require_active:
            assert gc['minor'] > 0
        if mode in ('same', 'off'):
            assert numeric[4] == 0
    return {'gc': gc, 'numeric': numeric, 'manual': manual,
            'admission': events['ADMISSION_AUDIT']}


def workloads(harness, report):
    fixed = next((directory, driver) for name, directory, driver in
                 regression.prepare_drivers(harness, report) if name == 'fixed_work')
    octane = harness / 'test/octane'
    text = (octane / 'run.js').read_text()
    select = 'NotifyScore: PrintScore });'
    deterministic = 'BenchmarkSuite.config.doDeterministic = undefined;'
    assert text.count(select) == text.count(deterministic) == 1
    text = text.replace(deterministic, 'BenchmarkSuite.config.doDeterministic = true;')
    codeload = report / 'codeload-fixed.js'
    codeload.write_text(text.replace(select, 'NotifyScore: PrintScore }, '
        'BenchmarkSuite.suites.filter(function(s){return s.name!=="CodeLoad";})'
        '.map(function(s){return s.name;}));'))
    raytrace = report / 'raytrace.js'
    raytrace.write_text(policy.a.driver_text('octane-raytrace-like'))
    return {'wtb': fixed, 'codeload': (octane, codeload), 'raytrace': (report, raytrace)}


def record(name, row, text):
    if name == 'wtb':
        regression.parse_result('fixed_work', row, text)
        assert all(count == 3 for count in row['work_counts'].values())
        row['metric'] = row['work_milliseconds']
    elif name == 'codeload':
        assert 'Score (version ' in text
        row['metric'] = float(re.search(r'^CodeLoad: ([0-9.]+)$', text, re.M)[1])
    else:
        result = policy.a.parse_bench(text, 'octane-raytrace-like')
        assert result['iterations'] == 500000 and len(result['timesMs']) == 3
        row['metric'] = result['medianMs']
        row['work'] = result
    assert row['metric'] > 0


def main():
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    assert platform.machine() == 'aarch64' and arch in ('arm32', 'arm64')
    assert int(os.environ['BENCHMARK_REPETITIONS']) == len(WTB_ORDERS)
    source = Path.cwd()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR'])
    report = root / ('store-policy-' + arch)
    report.mkdir()
    binary = source.parent / ('build-' + arch) / 'escargot'
    with binary.open('rb') as stream:
        hdr = stream.read(20)
    assert hdr[:4] == b'\x7fELF' and hdr[4] == (1 if arch == 'arm32' else 2)
    assert struct.unpack('<H', hdr[18:20])[0] == (40 if arch == 'arm32' else 183)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    harness = Path(os.environ['BENCHMARK_HARNESS'])
    tests = workloads(harness, report)
    hashes = {name: common.sha256(driver) for name, (_, driver) in tests.items()}
    hashes['wtb_bundle'] = common.sha256(tests['wtb'][0] / 'dist/memory.js')
    data = {'arch': arch, 'cpu': cpu, 'build_sha': common.output(['git','rev-parse','HEAD'],source),
            'gc_sha': common.output(['git','rev-parse','HEAD'],source/'third_party/GCutil'),
            'binary_sha256': common.sha256(binary), 'inputs': hashes, 'rows': []}
    environments = policy.environments()
    common.parse_audit = audit
    for name, (cwd, driver) in tests.items():
        orders = WTB_ORDERS if name == 'wtb' else GUARD_ORDERS
        schedule = ([(0, 'same')] if name == 'wtb' else []) + [
            (i, mode) for i, modes in enumerate(orders, 1) for mode in modes]
        for block, mode in schedule:
            label = f'{name}-{block}-{mode}'
            env = dict(environments['full' if mode == 'full' else 'refined'],
                       GC_DIAG_COHORT='7', GC_BENCH_NUMERIC_RO=str(MODES[mode]),
                       GC_BENCH_MANUAL_CODEBLOCK='1')
            command = [str(binary)]
            if arch == 'arm32':
                lib = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
                env['LD_LIBRARY_PATH'] = lib
                command = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path', lib, *command]
            if mode != 'full':
                command += ['--gc-generational']
            try:
                row, contents = common.process_measure(label, mode, command, cwd, driver, env,
                                                       report, cpu, monitor, False)
                record(name, row, contents)
                row['status'] = 'passed'
                print('METRIC', arch, label, row['metric'], 'peak_KiB', row['peak_rss_kib'],
                      'minors', row['mode_audit']['gc']['minor'], flush=True)
            except (RuntimeError, AssertionError, ValueError, KeyError, TypeError) as exc:
                row = {'status': 'failed', 'error': repr(exc)}
                print('FAILED', arch, label, repr(exc), flush=True)
            row.update(workload=name, block=block, mode=mode, label=label)
            data['rows'].append(row)
            common.write_json(report/'measurements.json', data)
    failed = [row['label'] for row in data['rows'] if row['status'] != 'passed']
    print('ARCH_COMPLETE', arch, 'failures', failed, flush=True)


if __name__ == '__main__':
    main()
