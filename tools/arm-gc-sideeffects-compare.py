#!/usr/bin/env python3
"""Compare automatic GC admission with same-build full GC in stock order."""
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import struct

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('repair', HERE / 'arm-gc-regression-fix.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)
p, c = r.p, r.c


def inactive_prefix_driver(text):
    # Keep every suite closure alive and run the original preceding cases.
    # Only extend the target's sampling; abort after its completed cycle.
    needle = '\nsuite.run();\n'
    assert text.count(needle) == 1 and 'suite.splice(' not in text
    text = text.replace(needle,
        "\nsuite.forEach(function(b){if(b.name==='prettier'){"
        "b.minSamples=20;b.options.minSamples=20;}});\n"
        "var completedPrefix=0;suite.on('cycle',function(){"
        "if(++completedPrefix===14)suite.abort();});\nsuite.run();\n")
    needle = 'if (suite.aborted) return;\n  const hz = gmean(suite.map(benchmark => benchmark.hz));'
    assert text.count(needle) == 1
    text = text.replace(needle,
        'if (suite.aborted && completedPrefix !== 14) return;\n'
        '  const hz = gmean(suite.filter(b=>b.stats.sample.length).map(benchmark => benchmark.hz));')
    needle = 'score:hz, tests:suite.map('
    assert text.count(needle) == 1
    return text.replace(needle, 'score:hz, tests:suite.filter(b=>b.stats.sample.length).map(')


def summarize_inactive_prefix(rows):
    groups = {control: [x for x in rows if x['control'] == control]
              for control in ['full_a', 'full_b', 'refined']}
    result = {'diagnostic': True, 'higher_is_better': True,
              'counts': {k: len(v) for k, v in groups.items()}, 'complete': False}
    if not all(len(v) == 3 for v in groups.values()):
        return result
    scores = {k: [x['score'] for x in v] for k, v in groups.items()}
    medians = {k: statistics.median(v) for k, v in scores.items()}
    pct = lambda a, b: [(x / y - 1) * 100 for x, y in zip(scores[a], scores[b])]
    versus_a, versus_b = pct('refined', 'full_a'), pct('refined', 'full_b')
    result.update(complete=True, medians=medians,
        full_b_vs_full_a_pct=pct('full_b', 'full_a'),
        refined_vs_full_a_pct=versus_a, refined_vs_full_b_pct=versus_b,
        median_change_vs_a_pct=(medians['refined'] / medians['full_a'] - 1) * 100,
        median_change_vs_b_pct=(medians['refined'] / medians['full_b'] - 1) * 100,
        losing_to_both_controls=sum(a < -2 and b < -2 for a, b in zip(versus_a, versus_b)))
    return result


def audit(contents, mode, require_active=True):
    result = p.policy_audit(contents, mode, require_active)
    audits = p.a.audits(contents)
    paused = audits['TRACKING_PAUSE_AUDIT']['paused']
    assert paused in (0, 1), audits
    if paused:
        assert mode == 'refined' and not result['common']['incremental'], audits
        assert result['pattern']['activations'] == 1, audits
    if mode == 'refined' and result['pattern']['activations']:
        # A deliberate pause is distinct from the VM-map-limit fallback.
        assert result['common']['incremental'] or paused, audits
    return result


def summarize(rows):
    result = {}
    for name in dict.fromkeys(x['name'] for x in rows):
        if name == 'prettier_prefix':
            result[name] = summarize_inactive_prefix([x for x in rows if x['name'] == name])
            continue
        groups = {m: [x for x in rows if x['name'] == name and x['mode'] == m]
                  for m in ['full', 'refined']}
        if not all(groups.values()):
            continue
        pattern = 'benchmark' in groups['full'][0]
        timed = pattern or name == 'fixed_work'
        metric = lambda x: (x['benchmark']['medianMs'] if pattern else
                            x['work_milliseconds'] if timed else x['score'])
        medians = {m: statistics.median(map(metric, group)) for m, group in groups.items()}
        item = {'medians': medians, 'higher_is_better': not timed,
                'change_pct': (medians['refined'] / medians['full'] - 1) * 100,
                'pairs': min(map(len, groups.values())), 'cases': {}}
        if not pattern:
            def cases(row):
                return ({t['name']: t['hz'] for t in row['tests']}
                        if name == 'web_tooling' else row['test_milliseconds']
                        if name == 'fixed_work' else row['subtests'])
            for case in cases(groups['full'][0]):
                values = {m: [cases(x)[case] for x in g] for m, g in groups.items()}
                change = (statistics.median(values['refined']) /
                          statistics.median(values['full']) - 1) * 100
                paired = [(a / b - 1) * 100 for a, b in
                          zip(values['refined'], values['full'])]
                item['cases'][case] = {'change_pct': change, 'paired_pct': paired,
                    'repeated_loss': len(paired) >= 3
                    and (change > 2 if timed else change < -2)
                    and sum(x > 2 if timed else x < -2 for x in paired) >= (len(paired) + 1) // 2}
        result[name] = item
    return result


def main():
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    assert arch in ('arm32', 'arm64')
    pairs = int(os.environ['BENCHMARK_REPETITIONS'])
    assert pairs in (1, 3, 5)
    report = root / ('gc-sideeffects-' + arch)
    report.mkdir()
    binary = source.parent / ('build-' + arch) / 'escargot'
    header = binary.read_bytes()[:20]
    assert header[:4] == b'\x7fELF'
    assert header[4] == (1 if arch == 'arm32' else 2)
    assert struct.unpack('<H', header[18:20])[0] == (40 if arch == 'arm32' else 183)
    engine = report / 'measured-engine'
    shutil.copy2(binary, engine)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    envs = p.environments()
    prefix = []
    if arch == 'arm32':
        runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
        prefix = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path', runtime]
        for env in envs.values():
            env['LD_LIBRARY_PATH'] = runtime
    commands = {m: [*prefix, str(engine)] + ([] if m == 'full' else ['--gc-generational'])
                for m in ['full', 'refined']}
    c.parse_audit = audit
    stock = r.prepare_drivers(harness, report)
    wtb = report / 'wtb-score.js'
    text = wtb.read_text().replace('count:b.count}', 'count:b.count,sample_seconds:b.stats.sample}')
    assert 'sample_seconds' in text
    wtb.write_text(text)
    data = {'status': 'running', 'arch': arch,
            'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
            'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
            'binary_sha256': c.sha256(engine),
            'inputs': json.loads((root / 'benchmark-inputs.json').read_text()),
            'build': json.loads((root / ('build-' + arch + '.json')).read_text()),
            'protocol': 'Same binary and native default build flags; FSD12, seed1. Three alternating fresh-process full/refined pairs each of stock WTB18 (three samples) and original-order Octane17. One paired check of each existing allocation workload at scale5, warmup1, samples3; these pattern pairs are benefit guards, not a new stable baseline. No RSS sampler, perf, cycle trace, or local map-limit override. Stop a grossly losing first WTB pair before spending the remaining validation budget. Any engine, parser, or unexpected fallback error fails the run.',
            'rows': [], 'failures': []}

    def save():
        data['summary'] = summarize(data['rows'])
        c.write_json(report / 'measurements.json', data)

    if pairs != 3:
        data['protocol'] = ('Focused follow-up: one fresh-process pair each of whole stock WTB18 and original-order Octane17; three alternating isolated CodeLoad pairs with every Octane preload retained; one cycle-traced original prefix through Splay, retaining all later suite roots; one pair per existing allocation workload at scale5. The one-pair stock guards and modified diagnostic contexts are not a new stable full-suite baseline. Same binary/default flags/FSD12/seed1; reject engine, checksum, parser and unexpected fallback errors.')

    if pairs == 5:
        data['protocol'] = ('Compiler/inactive-path follow-up: one fresh-process pair each of original workloads/order WTB18 with three samples (upstream uses20), and whole original-order Octane17; five alternating isolated CodeLoad pairs retaining all Octane preloads; two alternating fixed-work WTB18 pairs with exactly three calls per case; one existing allocation-pattern pair each at scale5. Fixed work includes cold compilation and is not the original timed-suite context. CodeLoad runs first and a repeated loss beyond2% stops expensive guards; stock one-pair guards, two fixed-work pairs and isolated cases do not replace the historical full-suite baseline. Same binary/default flags/FSD12/seed1; no perf/RSS sampler/cycle trace. Errors and unexpected fallback fail the run.')

    if pairs == 1:
        data['protocol'] = ('Inactive Prettier diagnosis: three rotated fresh-process blocks '
            'of identical full-GC controls A/B and refined, nine processes per architecture. '
            'Keep all WTB18 suite roots and original preceding execution order; earlier cases '
            'retain three samples, Prettier uses twenty, then abort after its completed cycle. '
            'Only Prettier is scored. A/B use identical executable, command, driver and environment. '
            'Same binary/default flags/FSD12/seed1; no profiler/RSS sampler/cycle trace. '
            'Reject hints, activation, minors, errors or fallback. This diagnostic is neither '
            'an original full-suite result nor a replacement for the historical anchor.')

    def measure(name, directory, driver, mode, repeat, pattern=False):
        label = name + '-' + str(repeat) + '-' + mode
        try:
            row, contents = c.process_measure(label, mode, commands[mode], directory,
                driver, {**envs[mode], **({'ESCARGOT_GC_CYCLE_TRACE': '1'}
                    if name == 'splay_prefix' else {})}, report, cpu, monitor, False)
            row.update(name=name, repeat=repeat, status='passed', audits=p.a.audits(contents))
            if pattern:
                row['benchmark'] = p.a.parse_bench(contents, name)
            elif name in ('codeload', 'splay_prefix'):
                cases = ['CodeLoad'] if name == 'codeload' else ['Splay', 'SplayLatency']
                row['subtests'] = {case: float(re.search(r'^' + case + r': ([0-9.]+)$', contents, re.M)[1]) for case in cases}
                assert all(value > 0 for value in row['subtests'].values())
                row['score'] = row['subtests'][cases[0]]
                row['diagnostic'] = True
            else:
                r.parse_result(name, row, contents)
                if name == 'web_tooling':
                    assert all(len(t['sample_seconds']) == 3 for t in row['tests'])
            data['rows'].append(row)
            print('SIDE EFFECT RESULT', label, row.get('score', row.get('benchmark')), flush=True)
            save()
        except BaseException as error:
            data['failures'].append({'label': label, 'error': repr(error)})
            save()
            raise

    save()
    try:
        if pairs == 1:
            _, directory, _ = next(x for x in stock if x[0] == 'web_tooling')
            driver = report / 'prettier-prefix.js'
            driver.write_text(inactive_prefix_driver(wtb.read_text()))
            controls = ['full_a', 'refined', 'full_b']
            for repeat in range(3):
                for control in controls[repeat:] + controls[:repeat]:
                    mode = 'refined' if control == 'refined' else 'full'
                    label = 'prettier-prefix-' + str(repeat + 1) + '-' + control
                    try:
                        row, contents = c.process_measure(label, mode, commands[mode], directory,
                            driver, envs[mode], report, cpu, monitor, False)
                        parsed = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', contents, re.M)[1])
                        tests = parsed['tests']
                        assert len(tests) == 14 and tests[-1]['name'] == 'prettier'
                        assert all(t['samples'] == len(t['sample_seconds']) ==
                                   (20 if t['name'] == 'prettier' else 3) for t in tests)
                        assert {t['name'] for t in tests} == r.WTB_NAMES - {
                            'source-map', 'terser', 'typescript', 'uglify-js'}
                        audits = p.a.audits(contents)
                        assert audits['ALLOCATION_REFINED_AUDIT']['sites'] == 0
                        assert audits['ALLOCATION_PATTERN_AUDIT']['hints'] == 0
                        assert audits['ALLOCATION_PATTERN_AUDIT']['activations'] == 0
                        assert audits['GENERATIONAL_AUDIT']['minor'] == 0
                        target = tests[-1]
                        assert target['hz'] > 0 and all(s > 0 for s in target['sample_seconds'])
                        row.update(name='prettier_prefix', control=control, repeat=repeat + 1,
                            status='passed', score=target['hz'], tests=tests, audits=audits)
                        data['rows'].append(row)
                        print('INACTIVE PREFIX RESULT', label, target['hz'], flush=True)
                        save()
                    except BaseException as error:
                        data['failures'].append({'label': label, 'error': repr(error)})
                        raise
            data['status'] = 'passed'
            print('INACTIVE PREFIX COMPLETE', arch, len(data['rows']), flush=True)
            return
        if pairs != 3:
            _, directory, _ = next(x for x in stock if x[0] == 'octane')
            original = (directory / 'run.js').read_text()
            needle = 'NotifyScore: PrintScore });'
            assert original.count(needle) == 1
            for name, skip in [
                ('codeload', 'BenchmarkSuite.suites.filter(function(s){return s.name!=="CodeLoad";}).map(function(s){return s.name;})'),
                ('splay_prefix', 'BenchmarkSuite.suites.slice(BenchmarkSuite.suites.map(function(s){return s.name;}).indexOf("Splay")+1).map(function(s){return s.name;})')]:
                if pairs == 5 and name == 'splay_prefix':
                    continue
                driver = report / (name + '.js')
                text = original.replace(needle, 'NotifyScore: PrintScore }, ' + skip + ');')
                if name == 'splay_prefix':
                    text = text.replace('BenchmarkSuite.RunSuites({ NotifyResult:', 'BenchmarkSuite.RunSuites({ NotifyStart: function(name){print("PHASE_START "+name);}, NotifyResult:')
                driver.write_text(text)
                for repeat in range((5 if pairs == 5 else 3) if name == 'codeload' else 1):
                    for mode in (['full', 'refined'] if repeat % 2 == 0 else ['refined', 'full']):
                        measure(name, directory, driver, mode, repeat + 1)
        if pairs == 5 and data['summary']['codeload']['cases']['CodeLoad']['repeated_loss']:
            data['status'] = 'rejected_early'
            save()
            print('Repeated CodeLoad loss; stopped expensive suite guards', flush=True)
            return
        for name in ['web_tooling', 'octane']:
            _, directory, driver = next(x for x in stock if x[0] == name)
            for repeat in range(3 if pairs == 3 else 1):
                for mode in (['full', 'refined'] if repeat % 2 == 0 else ['refined', 'full']):
                    measure(name, directory, driver, mode, repeat + 1)
                summary = data['summary'][name]
                if name == 'web_tooling' and repeat == 0 and summary['change_pct'] < -3:
                    losses = sum(v['change_pct'] < -10 for v in summary['cases'].values())
                    if losses >= 4:
                        data['status'] = 'rejected_early'
                        save()
                        print('Gross first-pair loss; stopped remaining measurements', flush=True)
                        return
        if pairs == 5:
            _, directory, driver = next(x for x in stock if x[0] == 'fixed_work')
            for repeat in range(2):
                for mode in (['full', 'refined'] if repeat % 2 == 0 else ['refined', 'full']):
                    measure('fixed_work', directory, driver, mode, repeat + 1)
        for index, name in enumerate(p.a.NAMES):
            driver = report / (name + '.js')
            driver.write_text(p.a.driver_text(name))
            for mode in (['full', 'refined'] if index % 2 == 0 else ['refined', 'full']):
                measure(name, report, driver, mode, 1, True)
        data['status'] = 'passed'
    except BaseException:
        data['status'] = 'failed'
        raise
    finally:
        save()
    print('SIDE EFFECT COMPARISON COMPLETE', arch, len(data['rows']), flush=True)


if __name__ == '__main__':
    main()
