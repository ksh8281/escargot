#!/usr/bin/env python3
"""Measure final full-GC VDB cost on SunSpider with matched off/on runs."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def make_driver(suite, report):
    entries = []
    digest = hashlib.sha256()
    for name in (suite / 'LIST').read_text().splitlines():
        script = suite / (name + '.js')
        data = suite / (name + '-data.js')
        entries.append({'name': name, 'script': str(script),
                        'data': str(data) if data.is_file() else None})
        for path in (script, data):
            if path.is_file():
                digest.update(path.name.encode() + b'\0' + path.read_bytes())
    assert len(entries) == 26
    driver = report / 'sunspider-driver.js'
    driver.write_text('''(function() {
var entries = ''' + json.dumps(entries) + ''';
var result = {}, counts = {}, collections = {};
for (var i = 0; i < entries.length; ++i) {
    var test = entries[i];
    if (test.data) load(test.data);
    var start = new Date();
    load(test.script);
    result[test.name] = new Date() - start;
    counts[test.name] = 1;
    gc(); collections[test.name] = 1;
}
print('SUNSPIDER_JSON ' + JSON.stringify(result));
print('MEM_WORK_JSON ' + JSON.stringify(counts));
print('MEM_GC_JSON ' + JSON.stringify(collections));
})();
''')
    return driver, digest.hexdigest()


def measure(command, root, report, cpu, repetitions):
    report.mkdir(parents=True, exist_ok=True)
    driver, signature = make_driver(
        root / 'test/vendortest/SunSpider/tests/sunspider-1.0.2', report)
    cost = module('sunspider_cost', Path(__file__).with_name('fullgc-cost-helpers.py'))
    base_env = dict(os.environ)
    for key in ('GC_ENABLE_INCREMENTAL', 'GC_DISABLE_INCREMENTAL', 'GC_PRINT_STATS',
                'GC_PRINT_VERBOSE_STATS', 'GC_PROFILE_MPROTECT', 'GC_FULL_FREQUENCY',
                'GC_PAUSE_TIME_TARGET', 'GC_MPROTECT_VERIFY_ELIGIBILITY',
                'ESCARGOT_GC_DISABLE_AUTOMATIC'):
        base_env.pop(key, None)
    base_env.update(GC_USE_GETWRITEWATCH='0', GC_FREE_SPACE_DIVISOR='12',
                    GC_MPROTECT_COARSE='1', ESCARGOT_GC_TIME_UNLIMITED='1',
                    ESCARGOT_GC_FULL_EVERY_COLLECTION='1', ESCARGOT_FULLGC_AUDIT='1')
    modes = [('off', False, '10M', False),
             ('on_default', True, '10M', False),
             ('on_forced', True, '0', False),
             ('fixed_off', False, '0', True),
             ('fixed_on', True, '0', True)]
    rows = []
    result = {'repetitions': repetitions, 'rows': rows, 'suite_sha256': signature,
              'protocol': '26 scripts once per fresh process; Date/load timing; '
                          'data loading and one explicit full GC per script outside score; '
                          'one discarded warmup per mode; rotated order',
              'command': command, 'cpu': cpu}
    saved = int(Path('/proc/sys/kernel/perf_event_paranoid').read_text())
    try:
        if saved > 0:
            subprocess.run(['sudo', '-n', 'sysctl', '-w', 'kernel.perf_event_paranoid=0'], check=True)
        for rep in range(-1, repetitions):
            shift = max(rep, 0) % len(modes)
            ordered = modes[shift:] + modes[:shift]
            if rep >= 0 and rep % 2:
                ordered = ordered[::-1]
            for mode, enabled, threshold, fixed in ordered:
                env = base_env | {'GC_INCREMENTAL_START_THRESHOLD': threshold}
                if fixed:
                    env['ESCARGOT_GC_DISABLE_AUTOMATIC'] = '1'
                args = [*command, *(['--gc-incremental'] if enabled else [])]
                values, text = cost.timed_run(f'{mode}-{rep}', args, root, env,
                                               driver, report, cpu)
                score = cost.json_line('SUNSPIDER_JSON', text)
                work = cost.json_line('MEM_WORK_JSON', text)
                calls = cost.json_line('MEM_GC_JSON', text)
                audit = cost.json_line('FULLGC_AUDIT', text)
                assert len(score) == len(work) == len(calls) == 26
                assert set(work) == set(calls) == set(score)
                assert set(work.values()) == set(calls.values()) == {1}
                assert all(value >= 0 for value in score.values())
                expected = {'time_limit': 999999, 'full_frequency': 0,
                            'automatic_disabled': int(fixed), 'adaptive': 0,
                            'partial_collections': 0}
                if not enabled or threshold == '0':
                    expected.update(incremental=int(enabled), actual_vdb=int(enabled))
                assert all(audit[key] == value for key, value in expected.items()), audit
                row = {'rep': rep, 'mode': mode, **values,
                       'score_ms': sum(score.values()), 'subtests_ms': score,
                       'cycles': values['cycles:u'] + values['cycles:k'],
                       'gc_audit': audit}
                if rep < 0:
                    continue
                rows.append(row)
                summary = {}
                for name, *_ in modes:
                    sample = [x for x in rows if x['mode'] == name]
                    if sample:
                        summary[name] = {
                            **{key: statistics.median(x[key] for x in sample)
                               for key in ('score_ms', 'cycles', 'peak_kib')},
                            'score_samples_ms': [x['score_ms'] for x in sample],
                            'gc_counts': [x['gc_audit']['collections'] for x in sample],
                            'actual_vdb': [x['gc_audit']['actual_vdb'] for x in sample]}
                result['summary'] = summary
                write_json(report / 'sunspider-comparison.json', result)
                print('SUNSPIDER_SAMPLE', mode, rep, row['score_ms'], audit, flush=True)
        fixed_counts = {x['gc_audit']['collections'] for x in rows
                        if x['mode'].startswith('fixed_')}
        assert len(fixed_counts) == 1, fixed_counts
        result['fixed_gc_count'] = fixed_counts.pop()
        result['overhead_percent'] = {}
        for name, control in (('on_default', 'off'), ('on_forced', 'off'),
                              ('fixed_on', 'fixed_off')):
            result['overhead_percent'][name] = {
                key: 100 * (result['summary'][name][key] / result['summary'][control][key] - 1)
                for key in ('score_ms', 'cycles', 'peak_kib')}
        write_json(report / 'sunspider-comparison.json', result)
        print('SUNSPIDER_SUMMARY', json.dumps({k: v for k, v in result.items() if k != 'rows'}), flush=True)
        return result
    finally:
        if saved > 0:
            subprocess.run(['sudo', '-n', 'sysctl', '-w', 'kernel.perf_event_paranoid=' + str(saved)], check=True)
        write_json(report / 'perf-permissions.json',
                   {'original': saved, 'restored': int(Path('/proc/sys/kernel/perf_event_paranoid').read_text())})


def compare(monthly, engines, harness, wtb, report, data, repetitions):
    assert len(engines) == 1
    data['gc_source_revision'] = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=Path(__file__).resolve().parents[1] / 'third_party/GCutil', text=True).strip()
    data['sunspider_comparison'] = measure(engines[0][1], harness, report,
                                           data['cpu'], repetitions * 3)
    data.update(scores_validated=True, failures=[])


def main():
    harness = Path(os.environ['BENCHMARK_HARNESS'])
    experiment = module('sunspider_experiment', harness / 'tools/arm-v8-lite-experiment.py')
    experiment.run_scores = compare
    experiment.main(mode='score', engine_names=('escargot',))


if __name__ == '__main__':
    def terminate(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, terminate)
    main()
