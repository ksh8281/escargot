#!/usr/bin/env python3
"""Attribute always-full-GC VDB cost on the native ARM64 benchmark host."""

import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys
import time


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def run(command, *, cwd=None, env=None, stdout=None, stderr=None, label='command'):
    print('START', label, flush=True)
    started = time.monotonic()
    process = subprocess.Popen(command, cwd=cwd, env=env, stdout=stdout, stderr=stderr)
    while True:
        try:
            code = process.wait(timeout=30)
            break
        except subprocess.TimeoutExpired:
            print('PROGRESS', label, round(time.monotonic() - started), 'seconds', flush=True)
    if code:
        raise subprocess.CalledProcessError(code, command)
    print('DONE', label, round(time.monotonic() - started, 2), 'seconds', flush=True)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def counters(path):
    values = {}
    for line in path.read_text().splitlines():
        fields = line.split(',')
        if len(fields) < 3 or not fields[0] or fields[0].startswith('#'):
            continue
        try:
            count = float(fields[0])
        except ValueError:
            continue
        values[fields[2]] = {'count': count,
                            'running_percent': float(fields[4]) if len(fields) > 4 and fields[4] else None}
    return values


def main(report):
    harness = Path(os.environ['BENCHMARK_HARNESS'])
    cpu = int(os.environ['BENCHMARK_CPU'])
    sys.path.insert(0, str(harness / 'tools'))
    monthly = load_module('monthly_cause', harness / 'tools/monthly-engine-benchmark.py')
    monthly.ROOT = harness
    loader, runtime, metadata = monthly.install_glibc()
    engine = Path(os.environ['ESCARGOT_ENGINE'])
    shutil.copy2(engine, report / 'escargot-arm64')
    command = [str(loader), '--library-path',
               f'{runtime}:/usr/icu78-64/lib:{monthly.SYSTEM_LIBRARY_PATH}', str(engine)]
    cost = load_module('cost_helpers', Path(__file__).with_name('fullgc-cost-helpers.py'))
    fixture = cost.drivers(harness, report)['web_tooling']
    cwd, _, script = fixture
    environment = dict(os.environ)
    for key in ('GC_ENABLE_INCREMENTAL', 'GC_DISABLE_INCREMENTAL', 'GC_PRINT_STATS',
                'GC_PRINT_VERBOSE_STATS', 'GC_PROFILE_MPROTECT', 'GC_FULL_FREQUENCY',
                'GC_PAUSE_TIME_TARGET'):
        environment.pop(key, None)
    environment.update(GC_USE_GETWRITEWATCH='0', GC_FREE_SPACE_DIVISOR='12',
                       GC_INCREMENTAL_START_THRESHOLD='10M', GC_MPROTECT_COARSE='1',
                       ESCARGOT_GC_TIME_UNLIMITED='1', ESCARGOT_GC_FULL_EVERY_COLLECTION='1',
                       ESCARGOT_GC_DISABLE_AUTOMATIC='1', ESCARGOT_FULLGC_AUDIT='1',
                       LC_ALL='C.UTF-8', LANG='C.UTF-8', DEBUGINFOD_URLS='')
    write_json(report / 'protocol.json', {'engine_command': command, 'runtime': metadata,
                                        'cpu': cpu, 'gc_policy': '41 synchronous full collections; dirty tracking only',
                                        'instructions_are_split_by_privilege': True})
    with (report / 'perf-list.txt').open('w') as output:
        subprocess.run(['perf', 'list', '--details'], stdout=output, stderr=subprocess.STDOUT, check=True)
    with (report / 'host.txt').open('w') as output:
        subprocess.run(['lscpu'], stdout=output, check=True)
    event_inventory = {}
    for device in sorted(Path('/sys/bus/event_source/devices').glob('*')):
        if not (device / 'events').is_dir():
            continue
        event_inventory[device.name] = {event.name: event.read_text().strip()
                                        for event in sorted((device / 'events').glob('*'))}
    write_json(report / 'pmu-events.json', event_inventory)
    probes = {}

    def available(event):
        result = subprocess.run(['perf', 'stat', '-x,', '-e', event, '--',
                                 'taskset', '-c', str(cpu), 'sleep', '0.02'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        probes[event] = {'returncode': result.returncode, 'output': result.stderr}
        return result.returncode == 0 and '<not supported>' not in result.stderr

    memory_events = []
    for name in ('dtlb_walk', 'l1d_tlb_refill', 'l1d_cache_refill', 'l2d_cache_refill'):
        candidate = next((f'{device}/{name}/u' for device, events in event_inventory.items()
                          if name in events and ('arm' in device or 'cpu' in device)), None)
        if candidate and available(candidate):
            memory_events.append(candidate)
    if not any('tlb' in event for event in memory_events):
        for event in ('dTLB-load-misses:u', 'dTLB-store-misses:u'):
            if available(event):
                memory_events.append(event)
    write_json(report / 'event-probes.json', probes)
    groups = {'compute': ['cycles:u', 'cycles:k', 'instructions:u', 'instructions:k',
                          'branches:u', 'branch-misses:u'],
              'memory': ['cycles:u', 'instructions:u', *memory_events]}
    write_json(report / 'event-groups.json', groups)
    print('EVENT_GROUPS', json.dumps(groups), flush=True)
    rows = []
    audits = []
    modes = ('ordinary', 'mprotect')
    for group, events in groups.items():
        for repetition in range(3):
            for mode in modes if repetition % 2 == 0 else modes[::-1]:
                label = f'{group}-{mode}-{repetition}'
                args = command + ([] if mode == 'ordinary' else ['--gc-incremental'])
                output = report / f'{label}.stdout'
                error = report / f'{label}.stderr'
                stat = report / f'{label}.perf.csv'
                with output.open('w') as stdout, error.open('w') as stderr:
                    run(['perf', 'stat', '-x,', '-o', str(stat), '-e',
                         ','.join([*events, 'task-clock', 'page-faults', 'minor-faults']),
                         '--', 'taskset', '-c', str(cpu), *args, str(script)],
                        cwd=cwd, env=environment, stdout=stdout, stderr=stderr, label=label)
                audit = cost.validate_output(mode, output.read_text() + error.read_text())
                assert audit['gc_audit']['collections'] == 41
                assert audit['gc_audit']['partial_collections'] == 0
                if audits:
                    assert audits[0]['work_counts'] == audit['work_counts']
                    assert audits[0]['gc_calls'] == audit['gc_calls']
                audits.append(audit)
                row = {'group': group, 'mode': mode, 'repetition': repetition,
                       'counters': counters(stat), 'audit': audit}
                assert set(events) <= row['counters'].keys(), row
                rows.append(row)
                write_json(report / 'measurements.json', rows)
                print('COUNTERS', label, json.dumps(row['counters']), flush=True)
    summaries = {}
    for group, events in groups.items():
        summaries[group] = {mode: {event: statistics.median(
            row['counters'][event]['count'] for row in rows
            if row['group'] == group and row['mode'] == mode)
            for event in events} for mode in modes}
    write_json(report / 'summary.json', summaries)
    print('COUNTER_SUMMARY', json.dumps(summaries), flush=True)
    for repetition in range(2):
        for mode in modes if repetition == 0 else modes[::-1]:
            label = f'profile-{mode}-{repetition}'
            args = command + ([] if mode == 'ordinary' else ['--gc-incremental'])
            output = report / f'{label}.stdout'
            error = report / f'{label}.stderr'
            data = report / f'{label}.data'
            with output.open('w') as stdout, error.open('w') as stderr:
                run(['taskset', '-c', str(cpu), 'perf', 'record', '-F', '1999',
                     '-e', 'cycles:u,cycles:k', '-g', '--call-graph', 'fp',
                     '-o', str(data), '--', *args, str(script)], cwd=cwd, env=environment,
                    stdout=stdout, stderr=stderr, label=label)
            audit = cost.validate_output(mode, output.read_text() + error.read_text())
            assert audit['gc_audit']['collections'] == 41 and audit['gc_audit']['partial_collections'] == 0
            write_json(report / f'{label}-audit.json', audit)
            for style, extra in [('flat', ['--no-children', '--call-graph', 'none']),
                                 ('callers', ['--children', '--call-graph', 'graph,0.05,caller'])]:
                with (report / f'{label}-{style}.txt').open('w') as output:
                    subprocess.run(['perf', 'report', '--stdio', '--stdio-color', 'never',
                                    '-i', str(data), '--percent-limit', '0.02', '--sort', 'dso,symbol',
                                    '--show-total-period', '--show-nr-samples',
                                    '--column-widths', '0,0,0,32,110', *extra],
                                   stdout=output, stderr=subprocess.STDOUT, env=environment, check=True)
            print('PROFILE_READY', label, flush=True)


if __name__ == '__main__':
    if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
        print('ARM32_SKIPPED: investigating the ARM64 userspace delta', flush=True)
        sys.exit(0)
    report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / 'fullgc-arm64-cause'
    report.mkdir(parents=True, exist_ok=True)
    saved = {key: int(Path('/proc/sys/' + key.replace('.', '/')).read_text())
             for key in ('kernel.perf_event_paranoid', 'kernel.kptr_restrict', 'vm.max_map_count')}
    target = {'kernel.perf_event_paranoid': -1, 'kernel.kptr_restrict': 0,
              'vm.max_map_count': max(saved['vm.max_map_count'], 1000000)}
    try:
        for key, value in target.items():
            if saved[key] != value:
                subprocess.run(['sudo', '-n', 'sysctl', '-w', f'{key}={value}'], check=True)
        main(report)
    finally:
        for key, value in saved.items():
            if target[key] != value:
                subprocess.run(['sudo', '-n', 'sysctl', '-w', f'{key}={value}'], check=True)
        write_json(report / 'host-settings.json', {
            'original': saved, 'during': target,
            'restored': {key: int(Path('/proc/sys/' + key.replace('.', '/')).read_text())
                         for key in saved}})
