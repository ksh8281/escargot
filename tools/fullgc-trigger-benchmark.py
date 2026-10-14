#!/usr/bin/env python3
"""Compare the pre-block collection guard with natural, always-full GC."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import signal
import statistics
import subprocess
import sys


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def audit_output(text, enabled):
    records = re.findall(r'^FULLGC_AUDIT (\{.*\})$', text, re.M)
    assert len(records) == 1, records
    audit = json.loads(records[0])
    expected = {'time_limit': 999999, 'full_frequency': 0,
                'automatic_disabled': 0, 'adaptive': 0,
                'incremental': int(enabled), 'actual_vdb': int(enabled),
                'partial_collections': 0}
    assert all(audit[key] == value for key, value in expected.items()), audit
    return audit


def build_old_guard(report, engine):
    engine = Path(engine)
    build = engine.parent
    commands = subprocess.check_output(['ninja', '-t', 'commands', 'escargot'],
                                       cwd=build, text=True).splitlines()
    gc = Path(__file__).resolve().parent.parent / 'third_party/GCutil'
    before = 'f61237639b2ea6c9e7fe988686ab089fdacd107c'
    old = subprocess.check_output(['git', 'show', before + ':alloc.c'], cwd=gc)
    current = (gc / 'alloc.c').read_bytes()
    assert current.replace(
        b'GC_dont_gc || GC_incremental_collection\n      || GC_disable_automatic_collection',
        b'GC_dont_gc || GC_incremental || GC_disable_automatic_collection') == old
    directory = report / 'old-guard-control'
    directory.mkdir(exist_ok=True)
    source = directory / 'alloc.c'
    source.write_bytes(old)
    raw = shlex.split(next(line for line in commands
                          if ' -c ' in line and '/GCutil/alloc.c' in line))
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
    obj = directory / 'alloc.o'
    compile_args += ['-o', str(obj)]
    subprocess.run(compile_args, cwd=build, check=True)
    link = shlex.split(commands[-1].split(' && ')[1])
    binary = directory / 'escargot-old-guard'
    link[link.index('-o') + 1] = str(binary)
    archive = next(i for i, value in enumerate(link) if value.endswith('/libgc-lib.a'))
    link.insert(archive, str(obj))
    subprocess.run(link, cwd=build, check=True)
    write_json(directory / 'control.json', {
        'gc_reference': before, 'unit_replaced': 'alloc.c',
        'only_difference': 'GC_should_collect_before_hblk_alloc incremental guard',
        'binary': str(binary), 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'compile_command': compile_args, 'link_command': link})
    print('OLD_GUARD_CONTROL_BUILT', binary, flush=True)
    return binary


def compare(monthly, engines, harness, wtb, report, data, repetitions):
    from engine_memory_benchmark import aggregate, measure_memory
    assert repetitions == 3, repetitions
    assert len(engines) == 1
    _, command, _ = engines[0]
    old_engine = build_old_guard(report, os.environ['ESCARGOT_ENGINE'])
    old_command = [str(old_engine) if argument == os.environ['ESCARGOT_ENGINE'] else argument
                   for argument in command]
    assert str(old_engine) in old_command
    modes = [('ordinary', command, False),
             ('old_guard', [*old_command, '--gc-incremental'], True),
             ('matched', [*command, '--gc-incremental'], True)]
    summary = {mode: {'score_samples': [], 'memory_samples': []} for mode, _, _ in modes}
    summary_path = report / 'trigger-comparison.json'
    data['protocol'].update(gc_policy='Always synchronous full GC; automatic collection enabled',
                            comparison='VDB off, VDB on with old guard, VDB on with matched guard',
                            score_repetitions=3, memory_repetitions=1,
                            score_order='Cyclic rotation across modes for three rounds')
    for repetition in range(repetitions):
        ordered = modes[repetition:] + modes[:repetition]
        for mode, args, enabled in ordered:
            label = f'escargot-{mode}-web_tooling-arm64-score-{repetition + 1}'
            sample = monthly.measure(label, args, wtb, os.environ,
                                     script='dist/cli.js', kind='web_tooling')
            sample['gc_audit'] = audit_output((report / sample['log']).read_text(), enabled)
            summary[mode]['score_samples'].append(sample)
            summary[mode]['score'] = statistics.median(
                row['score_runs_per_second'] for row in summary[mode]['score_samples'])
            write_json(summary_path, summary)
            print('TRIGGER_SCORE', mode, repetition + 1,
                  sample['score_runs_per_second'], sample['gc_audit'], flush=True)
    directory, driver, count, iterations = monthly.build_memory_drivers(wtb)['web_tooling']
    signature = None
    for mode, args, enabled in modes:
        label = f'escargot-{mode}-web_tooling-arm64-memory-1'
        sample = measure_memory(label, 'escargot', args, directory, os.environ,
                                driver, report, data['cpu'], data['monitor_cpu'])
        assert len(sample['work_counts']) == count
        assert set(sample['work_counts'].values()) == {iterations}
        sample['gc_audit'] = audit_output((report / sample['log']).read_text(), enabled)
        if signature is None:
            signature = sample['work_signature']
        assert signature == sample['work_signature']
        summary[mode]['memory_samples'].append(sample)
        summary[mode]['memory'] = aggregate(summary[mode]['memory_samples'])
        write_json(summary_path, summary)
        print('TRIGGER_MEMORY', mode, sample['gc_audit'], flush=True)
    data.update(scores_validated=True, memory_validated=True, failures=[], trigger_comparison=summary)
    data['memory_driver_sha256'] = monthly.sha256(driver)
    data['wtb_score_bundle_sha256'] = monthly.sha256(wtb / 'dist/cli.js')
    data['wtb_memory_bundle_sha256'] = monthly.sha256(wtb / 'dist/memory.js')


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
    spec = importlib.util.spec_from_file_location('trigger_experiment',
                                                harness / 'tools/arm-v8-lite-experiment.py')
    experiment = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(experiment)
    experiment.run_scores = compare
    experiment.main(mode='score', engine_names=('escargot',))


if __name__ == '__main__':
    def terminate(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, terminate)
    if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
        print('TRIGGER_ARM32_SKIPPED: comparing ARM64 policy', flush=True)
        sys.exit(0)
    original_limit = int(Path('/proc/sys/vm/max_map_count').read_text())
    required_limit = 1000000
    changed = original_limit < required_limit
    report = Path(os.environ['BENCHMARK_OUTPUT_DIR'])
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
        write_json(report / 'mapping-budget-arm64.json', budget)
        print('MAPPING_BUDGET', json.dumps(budget), flush=True)
