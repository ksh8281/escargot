#!/usr/bin/env python3
"""Compare full, early always-on and syntax-hinted GC at the same FSD."""
import datetime
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import statistics

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('allocation', HERE / 'arm-allocation-patterns-compare.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
c = a.c
MODES = ['full', 'always', 'pattern']


def policy_audit(contents, mode, require_active=True):
    audits = a.audits(contents)
    common = audits['PHONE_MODE_AUDIT']
    gc = audits['GENERATIONAL_AUDIT']
    hint = audits['ALLOCATION_PATTERN_AUDIT']
    assert common['free_space_divisor'] == 12, audits
    assert hint['mode'] == MODES.index(mode), audits
    if mode == 'full':
        assert common['incremental'] == 0 and gc['minor'] == 0, audits
    else:
        assert gc['full_frequency'] == 2 and hint['activations'] == 1, audits
        if mode == 'pattern':
            assert hint['hints'] >= 4096, audits
        else:
            assert hint['hints'] == 0 and audits['ADMISSION_AUDIT']['closings'] == 0, audits
    return {'common': common, 'generational': gc, 'pattern': hint,
            'admission': [audits['ADMISSION_AUDIT']]}


def environments():
    clean = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
    common = {**clean, **c.CONFIG['settings']['env'],
              'GC_FREE_SPACE_DIVISOR': '12', 'ESCARGOT_BENCH_SEED': '1', 'LD_PRELOAD': ''}
    return {mode: {**common, 'ESCARGOT_BENCH_ALLOCATION_POLICY': mode,
                   'GC_GENERATIONAL_ADMISSION': '1' if mode == 'pattern' else '0'}
            for mode in MODES}


def summarize(rows):
    groups = {mode: [r for r in rows if r['mode'] == mode] for mode in MODES}
    medians = {mode: statistics.median(r['benchmark']['medianMs'] for r in group)
               for mode, group in groups.items()}
    return {'median_ms': medians,
            'time_change_vs_full_pct': {m: (medians[m] / medians['full'] - 1) * 100 for m in MODES},
            'paired_time_change_vs_full_pct': {
                m: [(groups[m][i]['benchmark']['medianMs'] / groups['full'][i]['benchmark']['medianMs'] - 1) * 100
                    for i in range(3)] for m in MODES if m != 'full'},
            'peak_rss_kib': {m: statistics.median(r['peak_rss_kib'] for r in g) for m, g in groups.items()},
            'process_cpu_seconds': {m: statistics.median(r['user_seconds'] + r['system_seconds'] for r in g)
                                    for m, g in groups.items()},
            'audits': {m: [r['audits'] for r in g] for m, g in groups.items()}}


def main():
    assert platform.machine() == 'aarch64'
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = root / ('allocation-policy-' + arch)
    report.mkdir(exist_ok=True)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    assert int(os.environ['BENCHMARK_REPETITIONS']) == 3
    engine = Path(os.environ['ESCARGOT_ENGINE']).resolve()
    manifest = json.loads((HERE / 'allocation-policy-source.json').read_text())
    c.parse_audit = policy_audit
    data = {'status': 'preparing', 'architecture': arch, 'cpu': cpu, 'monitor_cpu': monitor,
            'measured_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
            'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
            'binary_sha256': c.sha256(engine), 'manifest': manifest,
            'benchmark_sha256': c.sha256(HERE / 'deqp-allocation-patterns.js'),
            'cmake_options': json.loads((root / ('build-' + arch + '.json')).read_text())['cmake_options'],
            'protocol': 'Same binary, FSD12, original six workloads without a resident fixture. Three cyclic-order blocks: full/always/pattern, always/pattern/full, pattern/full/always. Scale5, warmup1, samples3; process medians then median across three processes. Always starts after the first full and disables cost admission. Pattern enables small-literal/simple-new bytecode hints, activates after 4096 hints, uses existing cost admission, closes after two full cycles without hints and waits for 4096 new hints plus existing backoff before retry. Separate single-sample traces. No forced GC, perf sampling or RSS sampler during scores. Process peak RSS and CPU include reference checks, warmup and startup.',
            'rows': [], 'diagnostics': [], 'summary': {}}
    def save():
        c.write_json(report / 'measurements.json', data)
    save()
    try:
        assert data['gc_revision'] == manifest['gc_revision']
        for name, expected in manifest['files'].items():
            assert c.sha256(source / name) == expected, name
        shutil.copy2(engine, report / 'measured-engine')
        envs = environments()
        prefix = []
        if arch == 'arm32':
            runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
            loader = os.environ['BENCHMARK_RUNTIME_LOADER']
            prefix = [loader, '--library-path', runtime]
            for env in envs.values():
                env['LD_LIBRARY_PATH'] = runtime
            data['runtime'] = {'loader_sha256': c.sha256(loader),
                               'libc_sha256': c.sha256(Path(runtime) / 'libc.so.6')}
        commands = {m: [*prefix, str(engine)] + ([] if m == 'full' else ['--gc-generational']) for m in MODES}
        data['status'] = 'timing'
        for name in a.NAMES:
            driver = report / (name + '.js')
            driver.write_text(a.driver_text(name))
            for repeat in range(3):
                order = MODES[repeat:] + MODES[:repeat]
                for mode in order:
                    label = f'{name}-{repeat + 1}-{mode}'
                    row, contents = c.process_measure(label, mode, commands[mode], report,
                        driver, envs[mode], report, cpu, monitor, False)
                    row.update(workload=name, repeat=repeat + 1,
                               benchmark=a.parse_bench(contents, name), audits=a.audits(contents))
                    data['rows'].append(row)
                    print('POLICY RESULT', name, mode, json.dumps(row['benchmark']), flush=True)
                    save()
            data['summary'][name] = summarize([r for r in data['rows'] if r['workload'] == name])
            print('POLICY SUMMARY', name, json.dumps(data['summary'][name]), flush=True)
            save()
        data['status'] = 'diagnostics'
        for name in a.NAMES:
            driver = report / (name + '-trace.js')
            driver.write_text(a.driver_text(name, True))
            for mode in MODES:
                env = {**envs[mode], 'ESCARGOT_GC_CYCLE_TRACE': '1'}
                label = name + '-trace-' + mode
                row, contents = c.process_measure(label, mode, commands[mode], report,
                    driver, env, report, cpu, monitor, False)
                row.update(workload=name, benchmark=a.parse_bench(contents, name),
                           audits=a.audits(contents), measured_phases=a.phases(contents))
                data['diagnostics'].append(row)
                save()
        data['status'] = 'passed'
    except BaseException as error:
        data['status'] = 'failed'
        data['error'] = repr(error)
        raise
    finally:
        save()
    print('ALLOCATION POLICY COMPARISON COMPLETE', arch, flush=True)


if __name__ == '__main__':
    main()
