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


def inactive_prefix_driver(text, target="prettier", count=14):
    # Keep every suite closure alive and run the original preceding cases.
    # Only extend the target's sampling; abort after its completed cycle.
    needle = '\nsuite.run();\n'
    assert text.count(needle) == 1 and 'suite.splice(' not in text
    text = text.replace(needle,
        f"\nsuite.forEach(function(b){{if(b.name==={json.dumps(target)}){{"
        "b.minSamples=20;b.options.minSamples=20;}});\n"
        "var completedPrefix=0;suite.on('cycle',function(){"
        f"if(++completedPrefix==={count})suite.abort();}});\nsuite.run();\n")
    needle = 'if (suite.aborted) return;\n  const hz = gmean(suite.map(benchmark => benchmark.hz));'
    assert text.count(needle) == 1
    text = text.replace(needle,
        f'if (suite.aborted && completedPrefix !== {count}) return;\n'
        '  const hz = gmean(suite.filter(b=>b.stats.sample.length).map(benchmark => benchmark.hz));')
    needle = 'score:hz, tests:suite.map('
    assert text.count(needle) == 1
    return text.replace(needle, 'score:hz, tests:suite.filter(b=>b.stats.sample.length).map(')


def summarize_inactive_prefix(rows, blocks=3):
    groups = {control: [x for x in rows if x['control'] == control]
              for control in ['full_a', 'full_b', 'refined']}
    result = {'diagnostic': True, 'higher_is_better': True,
              'counts': {k: len(v) for k, v in groups.items()}, 'complete': False}
    if not all(len(v) == blocks for v in groups.values()):
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


FOLLOWUP_CASES = ('acorn', 'lebab', 'prepack')


def summarize_followup(rows):
    blocks = max((x['repeat'] for x in rows), default=1)
    cases = {}
    for name in FOLLOWUP_CASES:
        projected = [dict(x, score=next(t['hz'] for t in x['tests']
                                       if t['name'] == name)) for x in rows]
        item = summarize_inactive_prefix(projected, blocks)
        item['repeated_loss'] = (blocks == 3 and item['complete']
            and item['median_change_vs_a_pct'] < -2
            and item['median_change_vs_b_pct'] < -2
            and item['losing_to_both_controls'] >= 2)
        cases[name] = item
    return {'diagnostic': True, 'completed_blocks': len(rows) // 3,
            'cases': cases}


def followup_needs_more(summary):
    blocks = summary['completed_blocks']
    # Escalate only a loss to both controls repeated in every completed block.
    # Early stopping screens for a reproducible loss; it cannot certify absence.
    return blocks < 3 and any(x['complete'] and
        x['losing_to_both_controls'] == blocks for x in summary['cases'].values())


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
        if name == 'remaining_prefix':
            result[name] = summarize_followup([x for x in rows if x['name'] == name])
            continue
        if name == 'esprima_prefix':
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
    if pairs in (1, 5):
        assert text.count('minSamples: 3') == 1
        text = text.replace('minSamples: 3', 'minSamples: 20')
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

    if pairs == 5:
        data['protocol'] = ('Full original-sampling WTB confirmation: one fresh-process full/refined pair '
            'per architecture, opposite order on ARM32/ARM64, all eighteen workloads in original order '
            'and all twenty upstream samples per workload. Same binary/default flags/FSD12/seed1; '
            'no profiler, RSS sampler or cycle trace. Process peak RSS retained. '
            'Engine/checksum/parser/fallback errors fail the run; a gross first-pair loss is '
            'reported as rejected. A single process pair does not replace the historical full anchor or '
            'establish the previous three-pair repeated-loss criterion. Octane and pattern guards are omitted '
            'because the runtime is unchanged from the completed48 checkpoint.')

    if pairs == 1:
        data['protocol'] = ('Adaptive inactive-policy follow-up for Acorn, Lebab and Prepack. '
            'Keep all eighteen WTB suite roots and execute the original first thirteen '
            'workloads with all twenty upstream samples; abort after Prepack. '
            'First block: identical full-GC controls A, refined, B. Repeat in rotated '
            'order only if at least one target loses more than 2% to both controls '
            'in every completed block, up to three blocks. This is a predefined '
            'screen for reproducible loss, not a statistical absence-of-regression test. '
            'A/B use identical executable, command, driver and environment. '
            'Same binary/default flags/FSD12/seed1; no profiler, RSS sampler or cycle '
            'trace. Reject sites, hints, activation, minors, errors or fallback. '
            'Report complete per-block results and the stop reason. No baseline promotion.')

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
            elif name == 'web_tooling':
                parsed = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', contents, re.M)[1])
                assert {t['name'] for t in parsed['tests']} == r.WTB_NAMES
                expected = 20 if pairs == 5 else 3
                assert parsed['score'] > 0 and all(t['hz'] > 0 and
                    t['samples'] == len(t['sample_seconds']) == expected for t in parsed['tests'])
                row.update(score=parsed['score'], tests=parsed['tests'])
            else:
                r.parse_result(name, row, contents)
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
            driver = report / 'remaining-prefix.js'
            driver.write_text(inactive_prefix_driver(wtb.read_text(), target="prepack", count=13))
            controls = ['full_a', 'refined', 'full_b']
            for repeat in range(3):
                for control in controls[repeat:] + controls[:repeat]:
                    mode = 'refined' if control == 'refined' else 'full'
                    label = 'remaining-prefix-' + str(repeat + 1) + '-' + control
                    try:
                        row, contents = c.process_measure(label, mode, commands[mode], directory,
                            driver, envs[mode], report, cpu, monitor, False)
                        parsed = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', contents, re.M)[1])
                        tests = parsed['tests']
                        assert len(tests) == 13 and tests[-1]['name'] == 'prepack'
                        assert all(t['hz'] > 0 and t['samples'] == len(t['sample_seconds']) == 20
                                   and all(s > 0 for s in t['sample_seconds']) for t in tests)
                        assert {t['name'] for t in tests} == r.WTB_NAMES - {
                            'prettier', 'source-map', 'terser', 'typescript', 'uglify-js'}
                        audits = p.a.audits(contents)
                        assert audits['ALLOCATION_REFINED_AUDIT']['sites'] == 0
                        assert audits['ALLOCATION_PATTERN_AUDIT']['hints'] == 0
                        assert audits['ALLOCATION_PATTERN_AUDIT']['activations'] == 0
                        assert audits['GENERATIONAL_AUDIT']['minor'] == 0
                        row.update(name='remaining_prefix', control=control, repeat=repeat + 1,
                            status='passed', tests=tests, audits=audits)
                        data['rows'].append(row)
                        print('FOLLOW-UP RESULT', label,
                              {t['name']: t['hz'] for t in tests if t['name'] in FOLLOWUP_CASES},
                              flush=True)
                        save()
                    except BaseException as error:
                        data['failures'].append({'label': label, 'error': repr(error)})
                        raise
                summary = data['summary']['remaining_prefix']
                if not followup_needs_more(summary):
                    data['stop_reason'] = ('three_block_limit' if repeat == 2 else
                        'no_target_lost_to_both_controls_in_every_completed_block')
                    break
            data['status'] = 'passed'
            print('FOLLOW-UP COMPLETE', arch, len(data['rows']), data['stop_reason'], flush=True)
            return
        for name in (['web_tooling'] if pairs == 5 else ['web_tooling', 'octane']):
            _, directory, driver = next(x for x in stock if x[0] == name)
            for repeat in range(1 if pairs == 5 else 3):
                reverse = (repeat + (pairs == 5 and arch == 'arm64')) % 2
                for mode in (['refined', 'full'] if reverse else ['full', 'refined']):
                    measure(name, directory, driver, mode, repeat + 1)
                summary = data['summary'][name]
                if name == 'web_tooling' and repeat == 0 and summary['change_pct'] < -3:
                    losses = sum(v['change_pct'] < -10 for v in summary['cases'].values())
                    if losses >= 4:
                        data['status'] = 'rejected_early'
                        save()
                        print('Gross first-pair loss; stopped remaining measurements', flush=True)
                        return
        for index, name in enumerate([] if pairs == 5 else p.a.NAMES):
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
