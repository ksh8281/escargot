#!/usr/bin/env python3
"""Compare quiet-phase suspension delays in one native diagnostic executable."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import statistics
import struct

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('sideeffects', HERE / 'arm-gc-sideeffects-compare.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
r, p, c = m.r, m.p, m.c


def quiet_probe_source(original):
    replacements = {
        'STATIC MAY_THREAD_LOCAL unsigned GC_pattern_quiet_fulls;':
            'STATIC MAY_THREAD_LOCAL unsigned GC_pattern_quiet_fulls;\n'
            'STATIC MAY_THREAD_LOCAL unsigned GC_pattern_quiet_limit = 2;',
        '  GC_pattern_mode = mode;':
            '  GC_pattern_mode = mode;\n'
            '  GC_pattern_quiet_limit = GETENV("GC_DIAG_QUIET_FULLS") != NULL\n'
            '      && atoi(GETENV("GC_DIAG_QUIET_FULLS")) == 1 ? 1 : 2;',
        'GC_pattern_quiet_fulls < 2': 'GC_pattern_quiet_fulls < GC_pattern_quiet_limit',
        'GC_pattern_quiet_fulls >= 2': 'GC_pattern_quiet_fulls >= GC_pattern_quiet_limit',
    }
    for before, after in replacements.items():
        assert original.count(before) == 1, before
        original = original.replace(before, after)
    return original


def summarize(rows):
    # Reuse metric parsing/direction without calling either policy full GC.
    projected = [dict(x, mode='full' if x['quiet_fulls'] == 2 else 'refined') for x in rows]
    result = m.summarize(projected)
    for name, item in result.items():
        item['medians'] = {'current_two': item['medians']['full'],
                           'early_one': item['medians']['refined']}
        groups = {v: [x for x in rows if x['name'] == name and x['quiet_fulls'] == n]
                  for v, n in [('current_two', 2), ('early_one', 1)]}
        item['peak_rss_kib'] = {v: statistics.median(x['peak_rss_kib'] for x in g)
                                for v, g in groups.items()}
        item['rss_change_pct'] = (item['peak_rss_kib']['early_one']
                                 / item['peak_rss_kib']['current_two'] - 1) * 100
    return result


def main():
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    assert arch in ('arm32', 'arm64') and int(os.environ['BENCHMARK_REPETITIONS']) == 3
    report = root / ('gc-quiet-phase-' + arch)
    report.mkdir()
    binary = source.parent / ('build-' + arch) / 'escargot'
    alloc = source / 'third_party/GCutil/alloc.c'
    original = alloc.read_text()
    probe = quiet_probe_source(original)
    (report / 'diagnostic-alloc.c').write_text(probe)
    original_binary_sha256 = c.sha256(binary)
    engine = report / 'measured-engine'
    try:
        alloc.write_text(probe)
        print('BUILD same-binary quiet-phase diagnostic', arch, flush=True)
        if arch == 'arm32':
            container = ('escargot-arm-benchmark-' + os.environ['GITHUB_RUN_ID']
                         + '-' + os.environ['GITHUB_RUN_ATTEMPT'])
            c.run(['docker', 'start', '--attach', container])
        else:
            c.run(['cmake', '--build', str(binary.parent), '-j', '2'])
        shutil.copy2(binary, engine)
    finally:
        alloc.write_text(original)
    assert not c.output(['git', 'status', '--porcelain'], source)
    header = engine.read_bytes()[:20]
    assert header[:4] == b'\x7fELF' and header[4] == (1 if arch == 'arm32' else 2)
    assert struct.unpack('<H', header[18:20])[0] == (40 if arch == 'arm32' else 183)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    env = p.environments()['refined']
    prefix = []
    if arch == 'arm32':
        runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
        prefix = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path', runtime]
        env['LD_LIBRARY_PATH'] = runtime
    command = [*prefix, str(engine), '--gc-generational']
    c.parse_audit = m.audit
    stock = r.prepare_drivers(harness, report)
    data = {
        'status': 'running', 'arch': arch,
        'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
        'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
        'original_binary_sha256': original_binary_sha256, 'binary_sha256': c.sha256(engine),
        'original_alloc_sha256': hashlib.sha256(original.encode()).hexdigest(),
        'diagnostic_alloc_sha256': hashlib.sha256(probe.encode()).hexdigest(),
        'inputs': json.loads((root / 'benchmark-inputs.json').read_text()),
        'build': json.loads((root / ('build-' + arch + '.json')).read_text()),
        'protocol': 'Diagnostic only, not a production policy change or full-GC comparison. '
            'Temporarily patch alloc.c to select one/two hintless full collections before '
            'quiet suspension, rebuild in the same native toolchain, copy the executable '
            'and restore source. Both policies use that identical executable and command, '
            'with GC_DIAG_QUIET_FULLS set to equal-length values 1/2. Native defaults, FSD12, '
            'seed1, CPU pinning, no profiler/cycle trace/RSS sampler. Three alternating '
            'SunSpider26 pairs, two original-order Octane17 pairs, and one pair for each '
            'of six allocation patterns (scale5/warmup1/samples3). Reverse each pair order '
            'across architectures. Pattern fixed-work peak RSS is a memory guard; adaptive '
            'Octane RSS is supplementary. Record failures and finish other workloads. '
            'This is a bounded screen: retain production unchanged if gains are mixed, '
            'material losses repeat, or correctness/memory/benefit guards deteriorate. '
            'Inactive WTB is excluded because static analysis and activation are unchanged.',
        'rows': [], 'failures': [],
    }

    def save():
        data['summary'] = summarize(data['rows'])
        c.write_json(report / 'measurements.json', data)

    workloads = [(n, cwd, driver, 3 if n == 'sunspider' else 2)
                 for n, cwd, driver in stock if n in ('sunspider', 'octane')]
    for name in p.a.NAMES:
        driver = report / (name + '.js')
        driver.write_text(p.a.driver_text(name))
        workloads.append((name, report, driver, 1))
    save()
    for index, (name, cwd, driver, repeats) in enumerate(workloads):
        for repeat in range(repeats):
            order = [2, 1] if (index + repeat + (arch == 'arm64')) % 2 == 0 else [1, 2]
            for limit in order:
                label = name + '-' + str(repeat + 1) + '-quiet' + str(limit)
                try:
                    row, text = c.process_measure(label, 'refined', command, cwd, driver,
                        {**env, 'GC_DIAG_QUIET_FULLS': str(limit)}, report, cpu, monitor, False)
                    row.update(name=name, repeat=repeat + 1, quiet_fulls=limit,
                               status='passed', audits=p.a.audits(text))
                    if name in p.a.NAMES:
                        row['benchmark'] = p.a.parse_bench(text, name)
                    else:
                        r.parse_result(name, row, text)
                    data['rows'].append(row)
                    print('QUIET PHASE RESULT', label,
                          row.get('score', row.get('total_milliseconds', row.get('benchmark'))), flush=True)
                except Exception as error:
                    data['failures'].append({'label': label, 'error': repr(error)})
                    print('QUIET PHASE FAILURE', label, repr(error), flush=True)
                save()
    data['status'] = 'failed' if data['failures'] else 'passed'
    save()
    assert not data['failures'], data['failures']
    assert len(data['rows']) == 22
    print('QUIET PHASE COMPLETE', arch, len(data['rows']), flush=True)


if __name__ == '__main__':
    main()
