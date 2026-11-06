#!/usr/bin/env python3
"""Compare the six Gist allocation workloads on native ARM targets."""
import datetime
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import shutil
import statistics

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('comparison', HERE / 'arm-cold-epoch-compare.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
NAMES = ['deqp-split-shift', 'deqp-vector-chain', 'deqp-rgba-read',
         'deqp-float-buffer-cycle', 'octane-raytrace-like', 'wtb-acorn-node-shape']


def driver_text(name, diagnostic=False, resident=False):
    text = (HERE / 'deqp-allocation-patterns.js').read_text()
    needle = 'var w=workloads[wi],n=Math.max(1,Math.floor(w.base*scale));'
    assert text.count(needle) == 1
    text = text.replace(needle, needle + '\n if(w.name!==config.only)continue;')
    if diagnostic:
        start = 'var start=Date.now();checksum=w.fn(n);var elapsed=Date.now()-start;'
        assert text.count(start) == 1
        text = text.replace(start, "log('MEASURE_BEGIN');" + start + "log('MEASURE_END');")
    if resident:
        needle = 'var results=[];'
        assert text.count(needle) == 1
        text = text.replace(needle,
            'var resident=[];for(var ri=0;ri<65536;ri++)'
            'resident.push({index:ri,values:[ri,ri+1,ri+2,ri+3]});'
            'global.__allocationBenchResident=resident;\n' + needle)
    config = dict(scale=5, samples=1 if diagnostic else 3, warmups=1, only=name)
    return 'var BENCH_CONFIG=' + json.dumps(config) + ';\n' + text


def parse_bench(contents, name):
    rows = re.findall(r'^ALLOCATION BENCH (\{.*\})$', contents, re.M)
    assert len(rows) == 1, (name, rows)
    row = json.loads(rows[0])
    assert row['name'] == name and row['medianMs'] > 0, row
    assert abs(row['checksum'] - row['expected']) <= 1e-10 * max(1, abs(row['expected'])), row
    assert 'ALLOCATION BENCH COMPLETE ' in contents
    return row


def audits(contents):
    return {name: json.loads(value) for name, value in re.findall(
        r'^(\w+_AUDIT) (\{.*\})$', contents, re.M)}


def phases(contents):
    active = False
    totals = {}
    for line in contents.splitlines():
        if line == 'MEASURE_BEGIN':
            active = True
        elif line == 'MEASURE_END':
            active = False
        elif active:
            match = re.match(r'GC_CYCLE_(MARK|RECLAIM) (.*)', line)
            if match:
                event = json.loads(match[2])
                key = ('full' if event['full'] else 'minor') + '_' + match[1].lower()
                row = totals.setdefault(key, {'count': 0, 'ms': 0, 'abandoned': 0,
                                               'allocated': 0, 'new_live_estimate': 0})
                for field in row:
                    row[field] += 1 if field == 'count' else event.get(field, 0)
    return totals


def main():
    assert platform.machine() == 'aarch64'
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = root / ('allocation-' + arch)
    report.mkdir(exist_ok=True)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    assert int(os.environ['BENCHMARK_REPETITIONS']) == 3
    data = {'status': 'preparing', 'architecture': arch, 'cpu': cpu, 'monitor_cpu': monitor,
            'measured_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'candidate_revision': c.output(['git', 'rev-parse', 'HEAD'], source),
            'candidate_gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
            'configuration': c.CONFIG, 'benchmark_sha256': c.sha256(HERE / 'deqp-allocation-patterns.js'),
            'protocol': 'Six workloads isolated in fresh processes, each with no fixture and with a synthetic immutable resident graph of 65536 objects plus four-number arrays. Fixture construction precedes warmup and scored samples but is included in process RSS/CPU. Scale 5, one warmup, three samples per process, three alternating pairs. JS sample medians exclude reference checks; process RSS and CPU include startup, reference, warmup and all samples. No forced GC or RSS sampler. Separate cycle traces use research binary in full12 and accepted gen14 modes; they are not the frozen baseline and not acceptance timings. Inactive generational mode is recorded, never forced.',
            'rows': [], 'diagnostics': [], 'summary': {}}
    def save():
        c.write_json(report / 'measurements.json', data)
    save()
    try:
        for name, expected in c.CONFIG['expected_source_files_sha256'].items():
            assert c.sha256(source / name) == expected, name
        assert data['candidate_gc_revision'] == c.CONFIG['gc_revision']
        original, options = c.control_build(source, arch, root, 'original')
        candidate = source.parent / ('build-' + arch) / 'escargot'
        data['cmake_options'] = options
        for mode, binary in [('original', original), ('candidate', candidate)]:
            data[mode + '_binary_sha256'] = c.sha256(binary)
            shutil.copy2(binary, report / (mode + '-engine'))
        clean = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
        clean.update(ESCARGOT_BENCH_SEED='1', LD_PRELOAD='')
        prefix = []
        if arch == 'arm32':
            runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
            loader = os.environ['BENCHMARK_RUNTIME_LOADER']
            prefix = [loader, '--library-path', runtime]
            clean['LD_LIBRARY_PATH'] = runtime
            data['runtime'] = {'loader_sha256': c.sha256(loader),
                               'libc_sha256': c.sha256(Path(runtime) / 'libc.so.6')}
        envs = {'original': {**clean, 'GC_FREE_SPACE_DIVISOR': '12'},
                'candidate': {**clean, **c.CONFIG['settings']['env']}}
        commands = {'original': [*prefix, str(original)],
                    'candidate': [*prefix, str(candidate), *c.CONFIG['settings']['flags']]}
        data['status'] = 'timing'
        cases = [(resident, name) for resident in [False, True] for name in NAMES]
        for resident, name in cases:
            case = ('resident-' if resident else 'standalone-') + name
            driver = report / (case + '.js')
            driver.write_text(driver_text(name, resident=resident))
            for repeat in range(3):
                order = ['original', 'candidate'] if repeat % 2 == 0 else ['candidate', 'original']
                for mode in order:
                    label = f'{case}-{repeat + 1}-{mode}'
                    row, contents = c.process_measure(label, mode, commands[mode], report,
                        driver, envs[mode], report, cpu, monitor, False)
                    row.update(workload=name, case=case, resident=resident, repeat=repeat + 1,
                               benchmark=parse_bench(contents, name), audits=audits(contents))
                    data['rows'].append(row)
                    print('ALLOCATION RESULT', name, mode, json.dumps(row['benchmark']), flush=True)
                    save()
            groups = {mode: [r for r in data['rows'] if r['case'] == case and r['mode'] == mode]
                      for mode in commands}
            original_ms = statistics.median(r['benchmark']['medianMs'] for r in groups['original'])
            candidate_ms = statistics.median(r['benchmark']['medianMs'] for r in groups['candidate'])
            paired = [(groups['candidate'][i]['benchmark']['medianMs'] /
                       groups['original'][i]['benchmark']['medianMs'] - 1) * 100 for i in range(3)]
            data['summary'][case] = {'original_ms': original_ms, 'candidate_ms': candidate_ms,
                'time_change_pct': (candidate_ms / original_ms - 1) * 100,
                'paired_time_change_pct': paired,
                'peak_rss_kib': {m: statistics.median(r['peak_rss_kib'] for r in g) for m, g in groups.items()},
                'candidate_gc': [r['audits']['GENERATIONAL_AUDIT'] for r in groups['candidate']],
                'candidate_young': [r['audits']['YOUNG_MARK_AUDIT'] for r in groups['candidate']]}
            print('WORKLOAD SUMMARY', case, json.dumps(data['summary'][case]), flush=True)
            save()
        data['status'] = 'diagnostics'
        for resident, name in cases:
            case = ('resident-' if resident else 'standalone-') + name
            driver = report / (case + '-trace.js')
            driver.write_text(driver_text(name, True, resident))
            for mode in ['original', 'candidate']:
                env = {**envs[mode], 'ESCARGOT_GC_CYCLE_TRACE': '1'}
                command = [*prefix, str(candidate)] + (c.CONFIG['settings']['flags'] if mode == 'candidate' else [])
                label = case + '-trace-' + mode
                row, contents = c.process_measure(label, mode, command, report, driver,
                    env, report, cpu, monitor, False)
                row.update(workload=name, case=case, resident=resident, benchmark=parse_bench(contents, name),
                           audits=audits(contents), measured_phases=phases(contents),
                           binary_role='research binary full12' if mode == 'original' else 'research binary gen14')
                data['diagnostics'].append(row)
                save()
        data['status'] = 'passed'
    except BaseException as error:
        data['status'] = 'failed'
        data['error'] = repr(error)
        raise
    finally:
        save()
    print('ALLOCATION COMPARISON COMPLETE', arch, flush=True)


if __name__ == '__main__':
    main()
