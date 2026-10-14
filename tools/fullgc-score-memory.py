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
    summaries = {}
    # Reverse order on the second architecture to reduce a consistent order bias.
    modes = ('ordinary', 'current') if architecture == 'arm32' else ('current', 'ordinary')
    for mode in modes:
        os.environ['BENCHMARK_OUTPUT_DIR'] = str(root / mode)
        spec = importlib.util.spec_from_file_location('fullgc_' + mode,
                                                    harness / 'tools/arm-v8-lite-experiment.py')
        experiment = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(experiment)
        original_scores = experiment.run_scores
        original_write = experiment.write_results

        def write_results(report, data):
            if 'protocol' in data:
                data['protocol']['gc_policy'] = 'Automatic full GC only; no marking time limit; matched allocation budget'
                data['protocol']['vdb_mode'] = mode
            original_write(report, data)

        def run_scores(monthly, engines, harness, wtb, report, data, repetitions):
            if mode == 'current':
                for name, command, bridge in engines:
                    assert name == bridge == 'escargot'
                    command.append('--gc-incremental')
                    data['engines'][name]['command'] = command
            return original_scores(monthly, engines, harness, wtb, report, data, repetitions)

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
        assert summaries['ordinary']['memory'][suite]['work_signature'] == summaries['current']['memory'][suite]['work_signature']
    (root / f'fullgc-comparison-{architecture}.json').write_text(json.dumps(summaries, indent=2) + '\n')
    os.environ['BENCHMARK_OUTPUT_DIR'] = str(root)


if __name__ == '__main__':
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
