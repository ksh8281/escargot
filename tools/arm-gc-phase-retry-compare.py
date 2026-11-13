#!/usr/bin/env python3
"""Compare quiet-phase suspension delays in one native diagnostic executable."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import struct
import subprocess
import time

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


def profile_terser():
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = root / ('terser-profile-' + arch)
    report.mkdir()
    if arch == 'arm32':
        c.write_json(report / 'measurements.json', {
            'status': 'skipped', 'arch': arch,
            'reason': 'Repeated loss was observed on ARM64; no ARM32 timing repeated.'})
        return
    assert arch == 'arm64' and os.environ['BENCHMARK_PROFILE'] == 'true'
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    binary = source.parent / 'build-arm64/escargot'
    engine = report / 'measured-engine'
    shutil.copy2(binary, engine)
    header = engine.read_bytes()[:20]
    assert header[:5] == b'\x7fELF\x02' and struct.unpack('<H', header[18:20])[0] == 183
    r.prepare_drivers(harness, report)
    text = (report / 'wtb-score.js').read_text().replace(
        'count:b.count}', 'count:b.count,sample_seconds:b.stats.sample}')
    text = m.inactive_prefix_driver(text, 'terser', 16)
    text = text.replace('if(b.name==="terser")', 'if(b.name==="chai"||b.name==="terser")')
    needle = 'var completedPrefix=0;'
    assert text.count(needle) == 1
    text = text.replace(needle,
        "suite.forEach(function(b){b.on('start',function(){"
        "print('PROFILE_BEGIN '+b.name+' '+Date.now());});});\n"
        "suite.on('cycle',function(e){print('PROFILE_END '+e.target.name+' '+Date.now());});\n"
        + needle)
    driver = report / 'terser-prefix-profile.js'
    driver.write_text(text)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    c.parse_audit = m.audit
    data = {'status': 'running', 'arch': arch,
        'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
        'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
        'binary_sha256': c.sha256(engine), 'rows': [], 'failures': [],
        'protocol': 'ARM64 diagnostic only: full A, refined, identical full B. '
            'Original first sixteen WTB cases and all eighteen live roots, twenty samples '
            'for Chai/Terser and three for other cases, matching the reduced-sampling '
            'screen. Same production executable/default flags/FSD12/seed1. Perf cycles '
            'at 199 Hz with monotonic timestamps, GC cycle tracing, and line-buffered '
            'stdout/stderr for ordered stage markers. These instrumented scores are '
            'not acceptance measurements. No engine or policy source changes. ARM32 '
            'build occurs in the shared workflow but its timing is deliberately skipped.'}
    c.write_json(report / 'measurements.json', data)
    for control in ['full_a', 'refined', 'full_b']:
        mode = 'refined' if control == 'refined' else 'full'
        perf_data = report / (control + '.perf.data')
        command = ['perf', 'record', '-F', '199', '-e', 'cycles', '--clockid', 'mono',
                   '-o', str(perf_data), '--', 'stdbuf', '-oL', '-eL', str(engine)]
        if mode == 'refined':
            command.append('--gc-generational')
        offset = time.time() - time.monotonic()
        try:
            row, output = c.process_measure('profile-' + control, mode, command,
                harness / 'test/web-tooling-benchmark', driver,
                {**p.environments()[mode], 'ESCARGOT_GC_CYCLE_TRACE': '1'},
                report, cpu, monitor, False)
            parsed = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', output, re.M)[1])
            tests = parsed['tests']
            assert len(tests) == 16 and tests[-1]['name'] == 'terser'
            assert all(t['samples'] == len(t['sample_seconds'])
                       == (20 if t['name'] in ('chai', 'terser') else 3) for t in tests)
            audits = p.a.audits(output)
            assert audits['ALLOCATION_REFINED_AUDIT']['sites'] == 0
            assert audits['ALLOCATION_PATTERN_AUDIT']['hints'] == 0
            assert audits['GENERATIONAL_AUDIT']['minor'] == 0
            begin = re.search(r'^PROFILE_BEGIN terser (\d+)$', output, re.M)
            end = re.search(r'^PROFILE_END terser (\d+)$', output, re.M)
            assert begin and end and begin.end() < end.start()
            segment = output[begin.end():end.start()]
            marks = [json.loads(x) for x in re.findall(r'^GC_CYCLE_MARK (.*)$', segment, re.M)]
            reclaims = [json.loads(x) for x in re.findall(r'^GC_CYCLE_RECLAIM (.*)$', segment, re.M)]
            assert marks and all(x['full'] and not x['abandoned'] for x in marks)
            start, stop = float(begin[1]) / 1000 - offset, float(end[1]) / 1000 - offset
            profile = subprocess.check_output(['perf', 'report', '--stdio', '--no-children',
                '--percent-limit', '0.3', '--sort', 'symbol,dso', '--time', f'{start:.6f},{stop:.6f}',
                '-i', str(perf_data)], text=True)
            (report / (control + '-terser-perf.txt')).write_text(profile)
            row.update(control=control, target=tests[-1], tests=tests, audits=audits,
                epoch_minus_monotonic=offset, target_start_monotonic=start,
                target_end_monotonic=stop, marks=marks, reclaims=reclaims,
                target_gc={'full_collections': len(marks), 'mark_ms': sum(x['ms'] for x in marks),
                    'reclaim_ms': sum(x['ms'] for x in reclaims),
                    'allocated_bytes': sum(x['allocated'] for x in marks),
                    'mean_heap_bytes': statistics.mean(x['heap'] for x in reclaims)})
            data['rows'].append(row)
            print('TERSER PROFILE RESULT', control, row['target']['hz'], row['target_gc'], flush=True)
        except Exception as error:
            data['failures'].append({'control': control, 'error': repr(error)})
            data['status'] = 'failed'
            raise
        finally:
            c.write_json(report / 'measurements.json', data)
    data['status'] = 'passed'
    c.write_json(report / 'measurements.json', data)


def main():
    if int(os.environ['BENCHMARK_REPETITIONS']) == 1:
        profile_terser()
        return
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
