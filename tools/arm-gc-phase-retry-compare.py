#!/usr/bin/env python3
"""Native original-order comparison of allocation-phase retry policy."""
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import struct

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('repair', HERE / 'arm-gc-regression-fix.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)
p, c = r.p, r.c
BASE_GC = 'bb11d1281848c585178730e66759dc9d52f61af5'
BASE_ENGINE = '70b7c54782ac7c75807bb684d42c5d18ebdde473'


def parse_cycles(text):
    result = {}
    for kind in ['MARK', 'RECLAIM']:
        events = [json.loads(x) for x in re.findall(r'^GC_CYCLE_' + kind + r' (.*)$', text, re.M)]
        for full in [False, True]:
            group = [x for x in events if bool(x['full']) == full]
            result[('full_' if full else 'minor_') + kind.lower()] = {
                'count': len(group), 'ms': sum(x['ms'] for x in group),
                'aborts': sum(x.get('abandoned', 0) for x in group)}
    return result


def parse_context_result(name, row, text):
    if name != 'web_tooling':
        r.parse_result(name, row, text)
        return
    raw = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', text, re.M)[1])
    assert len(raw['tests']) == 18
    assert {t['name'] for t in raw['tests']} == r.WTB_NAMES
    assert raw['score'] > 0
    for test in raw['tests']:
        expected = 20 if test['name'] == 'terser' else 3
        assert test['samples'] == expected
        assert len(test['sample_seconds']) == expected
        assert test['hz'] > 0 and all(v > 0 for v in test['sample_seconds'])
    row.update(score=raw['score'], tests=raw['tests'])


def main():
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    assert arch == 'arm32' and int(os.environ['BENCHMARK_REPETITIONS']) == 3
    report = root / 'gc-phase-retry-arm32'
    report.mkdir()
    binary = source.parent / 'build-arm32/escargot'
    header = binary.read_bytes()[:20]
    assert header[:5] == b'\x7fELF\x01' and struct.unpack('<H', header[18:20])[0] == 40
    after = report / 'after-engine'
    before = report / 'before-engine'
    shutil.copy2(binary, after)
    gc = source / 'third_party/GCutil'
    c.run(['git', 'fetch', '--depth', '1', 'origin', BASE_GC], gc)
    assert c.output(['git', 'diff', '--name-only', BASE_GC, 'HEAD'], gc) == 'alloc.c'
    alloc = gc / 'alloc.c'
    candidate = alloc.read_bytes()
    baseline = __import__('subprocess').check_output(['git', 'show', BASE_GC + ':alloc.c'], cwd=gc)
    # Reuse the identical toolchain/build directory for a one-file ablation.
    # The runner's stopped build container still contains all dependencies.
    # Copy each binary before rebuilding, and restore the source even on error.
    try:
        alloc.write_bytes(baseline)
        container = 'escargot-arm-benchmark-' + os.environ['GITHUB_RUN_ID'] + '-' + os.environ['GITHUB_RUN_ATTEMPT']
        c.run(['docker', 'start', '--attach', container])
        shutil.copy2(binary, before)
    finally:
        alloc.write_bytes(candidate)
    assert not c.output(['git', 'status', '--porcelain'], source)
    runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
    prefix = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path', runtime]
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    envs = p.environments()
    c.parse_audit = lambda text, mode, require_active=True: p.policy_audit(
        text, 'full' if mode == 'full' else 'refined', require_active)
    commands = {m: [*prefix, str(after if m == 'after' else before)]
                + ([] if m == 'full' else ['--gc-generational']) for m in ['full', 'before', 'after']}
    for env in envs.values():
        env['LD_LIBRARY_PATH'] = runtime
    stock = r.prepare_drivers(harness, report)
    score_driver = report / 'wtb-score.js'
    score_text = score_driver.read_text()
    score_text = score_text.replace('count:b.count}', 'count:b.count,sample_seconds:b.stats.sample}')
    assert 'sample_seconds' in score_text
    assert score_text.count('\nsuite.run();\n') == 1
    score_driver.write_text(score_text.replace('\nsuite.run();\n',
        "\nsuite.forEach(function(b){if(b.name==='terser'){b.minSamples=20;b.options.minSamples=20;}});\nsuite.run();\n"))
    original = (harness / 'test/octane/run.js').read_text()
    focused = []
    for name in ['Splay']:
        driver = report / (name + '.js')
        driver.write_text(original.replace('BenchmarkSuite.RunSuites(',
            'BenchmarkSuite.suites=BenchmarkSuite.suites.filter(function(s){return s.name===' + json.dumps(name) + ';});\nBenchmarkSuite.RunSuites(', 1))
        focused.append((name, harness / 'test/octane', driver))
    for name in ['cli.js', 'memory.js']:
        shutil.copy2(harness / 'test/web-tooling-benchmark/dist' / name, report / ('input-' + name))
    data = {'status': 'running', 'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
        'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], gc),
        'baseline_engine': BASE_ENGINE, 'baseline_gc': BASE_GC,
        'binary_sha256': {'before': c.sha256(before), 'after': c.sha256(after)},
        'baseline_alloc_sha256': __import__('hashlib').sha256(baseline).hexdigest(),
        'candidate_alloc_sha256': c.sha256(alloc),
        'inputs': json.loads((root / 'benchmark-inputs.json').read_text()),
        'build': json.loads((root / 'build-arm32.json').read_text()),
        'protocol': 'Native ARM32 default build options. Identical build directory/toolchain with only alloc.c reverted to bb11d128 for the control; source restored afterward. Remaining two fresh-process alternating before/after pairs of WTB18 (pair one is retained from run 42792763196) in original order with original preceding workloads and warmup: all cases retain 3 samples except Terser, which uses 20. Record each sample; its first three provide a separate short-window diagnostic. This extended Terser result is not the stock three-sample WTB score. If the first continuation pair loses >10% on Terser and >2% aggregate, stop the rejected candidate early. Otherwise also run three alternating isolated Splay pairs with original preloads, and one complete original-order Octane17 pair as a guard. FSD12, seed1. No RSS sampler, no cycle tracing, no local VM-region-limit override. One Octane pair is diagnostic, not stable whole-suite acceptance. No separate full-GC control or ARM64 run.',
        'prior_run': 42792763196, 'rows': [], 'failures': []}
    def save():
        c.write_json(report / 'measurements.json', data)
    def measure(name, directory, script, mode, repeat, context=False):
        label = ('context-' if context else 'focus-') + name + '-' + str(repeat) + '-' + mode
        env = dict(envs['full' if mode == 'full' else 'refined'])
        try:
            row, text = c.process_measure(label, mode, commands[mode], directory, script, env, report, cpu, monitor, False)
            if context:
                parse_context_result(name, row, text)
            else:
                row['score'] = float(re.search('^' + name + r': ([0-9.]+)$', text, re.M)[1])
            row.update(name=name, repeat=repeat, context=context, status='passed', audits=p.a.audits(text), cycles=parse_cycles(text))
            data['rows'].append(row)
            print('FOCUSED PASS', label, row.get('score'), row['cycles'], flush=True)
        except Exception as error:
            data['failures'].append({'label': label, 'error': repr(error)})
            print('FOCUSED FAIL', label, repr(error), flush=True)
        save()
    save()
    wtb_suite = next(x for x in stock if x[0] == 'web_tooling')
    # Pair one completed in prior_run; only its stock parser rejected it.
    # Continue the remaining pairs without repeating those engine executions.
    for rep in range(1, 3):
        order = ['before', 'after'] if rep % 2 == 0 else ['after', 'before']
        for mode in order:
            measure(*wtb_suite, mode, rep + 1, True)
        if data['failures']:
            break
        pair = {x['mode']: x for x in data['rows']
                if x['name'] == 'web_tooling' and x['repeat'] == rep + 1}
        if rep == 1:
            terser_scores = {m: next(t['hz'] for t in row['tests'] if t['name'] == 'terser')
                             for m, row in pair.items()}
            if (terser_scores['after'] < terser_scores['before'] * 0.90
                    and pair['after']['score'] < pair['before']['score'] * 0.98):
                data['early_stop'] = 'Negative first WTB pair; no further acceptance claim'
                break
    if not data['failures'] and not data.get('early_stop'):
        splay = next(x for x in focused if x[0] == 'Splay')
        for rep in range(3):
            for mode in (['before', 'after'] if rep % 2 == 0 else ['after', 'before']):
                measure(*splay, mode, rep + 1)
        octane = next(x for x in stock if x[0] == 'octane')
        for mode in ['after', 'before']:
            measure(*octane, mode, 1, True)
    data['status'] = 'failed' if data['failures'] else ('rejected_early' if data.get('early_stop') else 'passed')
    save()
    assert not data['failures'], data['failures']
    print('FOCUSED COMPARISON COMPLETE', len(data['rows']), flush=True)


if __name__ == '__main__':
    main()
