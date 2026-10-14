#!/usr/bin/env python3
"""Compare VDB off/on with automatic, always-full collection on native ARM."""

import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess


def main():
    for key in ('GC_ENABLE_INCREMENTAL', 'GC_DISABLE_INCREMENTAL', 'GC_PRINT_STATS',
                'GC_PRINT_VERBOSE_STATS', 'GC_PROFILE_MPROTECT', 'GC_FULL_FREQUENCY',
                'GC_PAUSE_TIME_TARGET', 'ESCARGOT_GC_DISABLE_AUTOMATIC'):
        os.environ.pop(key, None)
    os.environ.update(GC_USE_GETWRITEWATCH='0', GC_FREE_SPACE_DIVISOR='12',
                      GC_INCREMENTAL_START_THRESHOLD='10M', GC_MPROTECT_COARSE='1',
                      ESCARGOT_GC_TIME_UNLIMITED='1',
                      ESCARGOT_GC_FULL_EVERY_COLLECTION='1', ESCARGOT_FULLGC_AUDIT='1')
    harness = Path(os.environ['BENCHMARK_HARNESS'])
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR'])
    architecture = os.environ['BENCHMARK_ARCHITECTURE']
    bitmap_engine = os.environ['ESCARGOT_ENGINE']
    interval_engine = build_interval_engine(root, bitmap_engine)
    summaries = {}
    # Reverse order on the second architecture to reduce a consistent order bias.
    modes = ('ordinary', 'interval', 'bitmap')
    for mode in modes:
        os.environ['ESCARGOT_ENGINE'] = str(interval_engine) if mode == 'interval' else bitmap_engine
        os.environ['BENCHMARK_OUTPUT_DIR'] = str(root / mode)
        spec = importlib.util.spec_from_file_location('fullgc_' + mode,
                                                    harness / 'tools/arm-v8-lite-experiment.py')
        experiment = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(experiment)
        original_scores = experiment.run_scores
        original_write = experiment.write_results

        def write_results(report, data):
            if 'protocol' in data:
                data['protocol']['gc_policy'] = 'Identical ordinary synchronous full GC; VDB disabled, interval registry, or committed bitmap'
                data['protocol']['vdb_mode'] = mode
            original_write(report, data)

        def run_scores(monthly, engines, harness, wtb, report, data, repetitions):
            if mode != 'ordinary':
                for name, command, bridge in engines:
                    assert name == bridge == 'escargot'
                    command.append('--gc-incremental')
                    data['engines'][name]['command'] = command
            return focused_scores(experiment, monthly, engines, wtb, report, data, repetitions)

        experiment.write_results = write_results
        experiment.run_scores = run_scores
        print('FULLGC_MODE_START', mode, architecture, flush=True)
        experiment.main(mode='both', engine_names=('escargot',))
        report = root / mode / f'v8-lite-{architecture}-scores-memory'
        audits = {}
        for log in sorted(report.glob('escargot-*.log')):
            records = re.findall(r'^FULLGC_AUDIT (\{.*\})$', log.read_text(errors='replace'), re.M)
            if records:
                assert len(records) == 1, log
                audit = json.loads(records[0])
                assert audit['automatic_disabled'] == 0 and audit['full_frequency'] == 0, (log, audit)
                assert audit['time_limit'] == 999999 and audit['partial_collections'] == 0, (log, audit)
                assert audit['adaptive'] == 0, (log, audit)
                if mode == 'ordinary':
                    assert audit['incremental'] == audit['actual_vdb'] == 0, (log, audit)
                elif 'web_tooling' in log.name or 'octane' in log.name:
                    assert audit['incremental'] == audit['actual_vdb'] == 1, (log, audit)
                audits[log.name] = audit
        assert audits
        (report / 'mode-audit.json').write_text(json.dumps(audits, indent=2) + '\n')
        data = json.loads((report / 'measurements.json').read_text())
        assert data['scores_validated'] and data['memory_validated'], data
        engine = data['engines']['escargot']
        summaries[mode] = {'scores': {suite: result['value'] for suite, result in engine['scores'].items()},
                           'memory': {suite: {key: result[key] for key in ('average_rss_kib', 'peak_rss_kib', 'work_signature')}
                                      for suite, result in engine['memory'].items()},
                           'audits': audits}
        print('FULLGC_MODE_VALIDATED', mode, json.dumps(summaries[mode]), flush=True)
    for suite in summaries['ordinary']['memory']:
        for mode in ('interval', 'bitmap'):
            assert summaries['ordinary']['memory'][suite]['work_signature'] == summaries[mode]['memory'][suite]['work_signature']
    (root / f'fullgc-comparison-{architecture}.json').write_text(json.dumps(summaries, indent=2) + '\n')
    os.environ['BENCHMARK_OUTPUT_DIR'] = str(root)
    run_fixed_cost(root, harness, architecture, data['engines']['escargot']['command'], bitmap_engine, interval_engine)


def focused_scores(experiment, monthly, engines, wtb, report, data, repetitions):
    data['score_protocol'] = 'Monthly Web Tooling driver; fresh CPU-pinned processes'
    for name, command, _ in engines:
        result = data['engines'][name].setdefault('scores', {}).setdefault(
            'web_tooling', {'metric': 'score_runs_per_second', 'samples': []})
        for repetition in range(repetitions):
            label = f'{name}-web_tooling-{data["architecture"]}-score-{repetition + 1}'
            sample = monthly.measure(label, command, wtb, os.environ, script='dist/cli.js', kind='web_tooling')
            result['samples'].append(sample)
            import statistics
            result['value'] = statistics.median(s['score_runs_per_second'] for s in result['samples'])
            print(label + ': score_runs_per_second = ' + str(sample['score_runs_per_second']), flush=True)
            experiment.write_results(report, data)
    data['scores_validated'] = True
    data['failures'] = []
    original_drivers = monthly.build_memory_drivers
    def drivers(bundle):
        return {'web_tooling': original_drivers(bundle)['web_tooling']}
    monthly.build_memory_drivers = drivers


def build_interval_engine(report, engine):
    import hashlib
    import shlex
    engine = Path(engine)
    build = engine.parent
    commands = subprocess.check_output(['ninja', '-t', 'commands', 'escargot'],
                                       cwd=build, text=True).splitlines()
    gc = Path(__file__).resolve().parent.parent / 'third_party/GCutil'
    before = '710224c0fe10c73b69be4ade5241bd8937f82fef'
    directory = report / 'interval-control'
    directory.mkdir(exist_ok=True)
    source = directory / 'vdb_isolate.c'
    source.write_bytes(subprocess.check_output(['git', 'show', before + ':vdb_isolate.c'], cwd=gc))
    raw = shlex.split(next(line for line in commands if ' -c ' in line and '/GCutil/vdb_isolate.c' in line))
    compile_args = [raw[0]]
    i = 1
    while i < len(raw):
        if raw[i] in ('-o', '-MF', '-MT'):
            i += 2
        elif raw[i] == '-MD':
            i += 1
        else:
            compile_args.append(raw[i]); i += 1
    compile_args[-1] = str(source)
    obj = directory / 'vdb_isolate.o'
    compile_args += ['-o', str(obj)]
    subprocess.run(compile_args, cwd=build, check=True)
    link = shlex.split(commands[-1].split(' && ')[1])
    binary = directory / 'escargot-interval'
    link[link.index('-o') + 1] = str(binary)
    archive = next(i for i, value in enumerate(link) if value.endswith('/libgc-lib.a'))
    link.insert(archive, str(obj))
    subprocess.run(link, cwd=build, check=True)
    (directory / 'control.json').write_text(json.dumps({
        'gc_reference': before, 'unit_replaced': 'vdb_isolate.c',
        'binary': str(binary), 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'compile_command': compile_args, 'link_command': link}, indent=2) + '\n')
    print('INTERVAL_CONTROL_BUILT', binary, flush=True)
    return binary


def run_fixed_cost(root, harness, architecture, engine_command, bitmap_engine, interval_engine):
    import statistics
    directory = root / ('fixed-registry-cost-' + architecture)
    directory.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location('fullgc_cost',
                                                Path(__file__).with_name('fullgc-cost-helpers.py'))
    cost = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cost)
    fixtures = cost.drivers(harness, directory)
    environment = dict(os.environ, ESCARGOT_GC_DISABLE_AUTOMATIC='1')
    cpu = int(os.environ['BENCHMARK_CPU'])
    new = [argument for argument in engine_command if argument != '--gc-incremental']
    old = [str(interval_engine) if argument == bitmap_engine else argument for argument in new]
    assert str(interval_engine) in old
    modes = [('ordinary', new), ('interval', [*old, '--gc-incremental']),
             ('bitmap', [*new, '--gc-incremental']), ('interval_off', old)]
    rows = []
    saved_paranoid = int(Path('/proc/sys/kernel/perf_event_paranoid').read_text())
    try:
        if saved_paranoid > 0:
            subprocess.run(['sudo', '-n', 'sysctl', '-w', 'kernel.perf_event_paranoid=0'], check=True)
        for repetition in range(3):
            order = modes if repetition == 0 else (modes[::-1] if repetition == 1 else modes[2:] + modes[:2])
            for mode, args in order:
                cwd, _, script = fixtures['web_tooling']
                values, output = cost.timed_run(f'{mode}-{repetition}', args, cwd,
                                               environment, script, directory, cpu)
                audit = cost.validate_output('ordinary' if mode in ('ordinary', 'interval_off') else 'mprotect', output)
                assert audit['gc_audit']['partial_collections'] == 0, audit
                assert audit['gc_audit']['collections'] == 41, audit
                if rows:
                    assert rows[0]['work_counts'] == audit['work_counts']
                    assert rows[0]['gc_calls'] == audit['gc_calls']
                rows.append({'mode': mode, 'repetition': repetition, **values, **audit})
                (directory / 'measurements.json').write_text(json.dumps(rows, indent=2) + '\n')
            medians = {mode: statistics.median(row['cycles:u'] + row['cycles:k']
                                             for row in rows if row['mode'] == mode)
                       for mode, _ in modes}
            result = {'repetitions': repetition + 1, 'median_cycles': medians,
                      'vdb_overhead_percent': {'interval': (medians['interval'] / medians['interval_off'] - 1) * 100,
                                               'bitmap': (medians['bitmap'] / medians['ordinary'] - 1) * 100},
                      'active_vdb_change_percent': (medians['bitmap'] / medians['interval'] - 1) * 100}
            (directory / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
            print('REGISTRY_FIXED_FULLGC_COST', json.dumps(result), flush=True)
    finally:
        if saved_paranoid > 0:
            subprocess.run(['sudo', '-n', 'sysctl', '-w', 'kernel.perf_event_paranoid=' + str(saved_paranoid)], check=True)
        (directory / 'perf-permissions.json').write_text(json.dumps({
            'original': saved_paranoid,
            'restored': int(Path('/proc/sys/kernel/perf_event_paranoid').read_text())}, indent=2) + '\n')


if __name__ == '__main__':
    import sys
    import signal
    def terminate(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, terminate)
    if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
        print('REGISTRY_ARM32_SKIPPED: validating ARM64 first', flush=True)
        sys.exit(0)
    original_limit = int(Path('/proc/sys/vm/max_map_count').read_text())
    required_limit = 1000000
    changed = original_limit < required_limit
    report = Path(os.environ['BENCHMARK_OUTPUT_DIR'])
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    budget = {'original_vm_max_map_count': original_limit,
              'compiled_gc_soft_limit': 250000, 'changed': changed}
    try:
        if changed:
            subprocess.run(['sudo', '-n', 'sysctl', '-w',
                            'vm.max_map_count=' + str(required_limit)], check=True)
        budget['effective_vm_max_map_count'] = int(Path('/proc/sys/vm/max_map_count').read_text())
        assert budget['effective_vm_max_map_count'] >= required_limit
        main()
    finally:
        if changed:
            subprocess.run(['sudo', '-n', 'sysctl', '-w',
                            'vm.max_map_count=' + str(original_limit)], check=True)
        budget['restored_vm_max_map_count'] = int(Path('/proc/sys/vm/max_map_count').read_text())
        (report / f'mapping-budget-{arch}.json').write_text(json.dumps(budget, indent=2) + '\n')
        print('MAPPING_BUDGET', json.dumps(budget), flush=True)
