#!/usr/bin/env python3
"""Compare delayed, early and site-filtered allocation hints at the same FSD."""
import datetime
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import statistics
import struct

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('allocation', HERE / 'arm-allocation-patterns-compare.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
c = a.c
MODES = ['full', 'pattern', 'early', 'refined']
MODE_IDS = {'full': 0, 'pattern': 2, 'early': 3, 'refined': 4}


def policy_audit(contents, mode, require_active=True):
    audits = a.audits(contents)
    common = audits['PHONE_MODE_AUDIT']
    gc = audits['GENERATIONAL_AUDIT']
    hint = audits['ALLOCATION_PATTERN_AUDIT']
    assert common['free_space_divisor'] == 12, audits
    assert hint['mode'] == MODE_IDS[mode], audits
    if mode == 'full':
        assert common['incremental'] == 0 and gc['minor'] == 0, audits
    else:
        assert gc['full_frequency'] == 2, audits
        # A filter may correctly leave a workload inactive; never force it on.
        assert hint['activations'] in (0, 1), audits
        if hint['activations']:
            threshold = 4096 if mode == 'pattern' else 64
            refined = audits['ALLOCATION_REFINED_AUDIT']
            assert refined['activation_hints'] >= threshold, audits
            if mode == 'refined':
                assert refined['activation_evidence'] >= 64, audits
        else:
            assert common['incremental'] == 0 and gc['minor'] == 0, audits
    return {'common': common, 'generational': gc, 'pattern': hint,
            'refined': audits['ALLOCATION_REFINED_AUDIT'],
            'admission': [audits['ADMISSION_AUDIT']]}


def cold_driver_text(name, expected):
    text = a.driver_text(name).replace('"scale": 5', '"scale": 0.2')
    text = text.replace('"samples": 3', '"samples": 1').replace('"warmups": 1', '"warmups": 0')
    probe = "if(!close(w.fn(513),reference(w.name,513)))throw Error('Probe mismatch: '+w.name);"
    assert text.count(probe) == 1
    text = text.replace(probe, '')
    needle = 'var expected=reference(w.name,n);'
    assert text.count(needle) == 1
    return text.replace(needle, 'var expected=' + json.dumps(expected) + ';')


def environments():
    clean = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
    common = {**clean, **c.CONFIG['settings']['env'],
              'GC_FREE_SPACE_DIVISOR': '12', 'ESCARGOT_BENCH_SEED': '1', 'LD_PRELOAD': ''}
    return {mode: {**common, 'ESCARGOT_BENCH_ALLOCATION_POLICY': mode,
                   'GC_GENERATIONAL_ADMISSION': '1' if mode != 'full' else '0'}
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
    report = root / ('allocation-refined-' + arch)
    report.mkdir(exist_ok=True)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    assert int(os.environ['BENCHMARK_REPETITIONS']) == 3
    # ESCARGOT_ENGINE is a shell launcher on ARM32; the loader needs ELF.
    engine = source.parent / ('build-' + arch) / 'escargot'
    header = engine.read_bytes()[:20]
    expected_class, expected_machine = (1, 40) if arch == 'arm32' else (2, 183)
    assert header[:4] == b'\x7fELF' and header[4] == expected_class
    assert struct.unpack('<H', header[18:20])[0] == expected_machine
    manifest = json.loads((HERE / 'allocation-refined-source.json').read_text())
    c.parse_audit = policy_audit
    data = {'status': 'preparing', 'architecture': arch, 'cpu': cpu, 'monitor_cpu': monitor,
            'measured_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
            'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
            'binary_sha256': c.sha256(engine), 'manifest': manifest,
            'benchmark_sha256': c.sha256(HERE / 'deqp-allocation-patterns.js'),
            'cmake_options': json.loads((root / ('build-' + arch + '.json')).read_text())['cmake_options'],
            'protocol': 'Same binary/FSD12, original six workloads, no resident fixture. Full disables generational GC; pattern uses 4096 hints; early lowers only the initial threshold to 64; refined filters retained/reference-heavy allocation shapes, requires 64 executions of one site, and preserves cost-failure backoff. Three cyclic-order blocks for each workload (four modes, not a fully balanced Latin square). Steady: scale5/warmup1/samples3. Cold: scale0.2/warmup0/samples1 with no probe or reference execution before the timer; expected checksums prevalidated by the full-mode scalar reference. Three fresh-process repetitions for both. Cold JS timing includes first invocation and lazy compilation, excludes startup; process CPU includes startup and all compilation. Separate steady single-sample traces. No forced GC, perf or RSS sampler during timing.',
            'rows': [], 'cold_rows': [], 'diagnostics': [], 'summary': {}, 'cold_summary': {}}

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
        data['status'] = 'cold'
        for name in a.NAMES:
            driver = report / (name + '-cold.js')
            driver.write_text(cold_driver_text(name, manifest['cold_expected'][name]))
            for repeat in range(3):
                order = MODES[repeat:] + MODES[:repeat]
                for mode in order:
                    label = f'{name}-cold-{repeat + 1}-{mode}'
                    row, contents = c.process_measure(label, mode, commands[mode], report,
                        driver, envs[mode], report, cpu, monitor, False)
                    row.update(workload=name, repeat=repeat + 1,
                               benchmark=a.parse_bench(contents, name), audits=a.audits(contents))
                    data['cold_rows'].append(row)
                    print('COLD RESULT', name, mode, json.dumps(row['benchmark']), flush=True)
                    save()
            data['cold_summary'][name] = summarize([r for r in data['cold_rows'] if r['workload'] == name])
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
    print('ALLOCATION REFINED COMPARISON COMPLETE', arch, flush=True)


if __name__ == '__main__':
    main()
