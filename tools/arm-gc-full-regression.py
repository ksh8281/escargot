#!/usr/bin/env python3
"""Run complete suites for the frozen allocation-policy experiment."""
import datetime
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import shutil
import statistics
import struct

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('policy', HERE / 'arm-allocation-refined-compare.py')
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
c = p.c
MODES = ['full', 'pattern', 'refined']
WTB_NAMES = {'acorn', 'babel', 'babel-minify', 'babylon', 'buble', 'chai',
             'coffeescript', 'espree', 'esprima', 'jshint', 'lebab', 'postcss',
             'prepack', 'prettier', 'source-map', 'terser', 'typescript', 'uglify-js'}


def prepare_drivers(harness, report):
    wtb = harness / 'test/web-tooling-benchmark'
    cli = (wtb / 'dist/cli.js').read_text()
    needle = '  console.log(`Geometric mean: ${align(hz.toFixed(2), 5, "right")} runs/s`);'
    assert cli.count(needle) == 1
    cli = cli.replace(needle, needle + "\n  print('WTB_SCORE_JSON ' + JSON.stringify({score:hz, tests:suite.map(b => ({name:b.name,hz:b.hz,samples:b.stats.sample.length,count:b.count}))}));", 1)
    score = report / 'wtb-score.js'
    score.write_text(cli)
    fixed = report / 'wtb-fixed.js'
    fixed.write_text("var workStart=Date.now(),lastCheckpoint=0,loadedAt=0,fixedTimes={};\n"
        "var benchmarkCheckpoint=function(name){var now=Date.now();"
        "if(name==='loaded'){loadedAt=now;}else{fixedTimes[name.slice(5)]=now-lastCheckpoint;}"
        "lastCheckpoint=now;};\nload('dist/memory.js');\n"
        "print('WTB_FIXED_WORK_MS '+(Date.now()-workStart));\n"
        "print('WTB_FIXED_TESTS '+JSON.stringify(fixedTimes));\n"
        "print('WTB_LOAD_MS '+(loadedAt-workStart));\n")
    sun_dir = harness / 'test/vendortest/SunSpider/tests/sunspider-1.0.2'
    entries = []
    for name in (sun_dir / 'LIST').read_text().splitlines():
        data = sun_dir / (name + '-data.js')
        entries.append({'name': name, 'script': str(sun_dir / (name + '.js')),
                        'data': str(data) if data.is_file() else None})
    assert len(entries) == 26
    sun = report / 'sunspider.js'
    sun.write_text('(function(){var entries=' + json.dumps(entries) + ';var result={};\n'
        'for(var i=0;i<entries.length;i++){var test=entries[i];if(test.data)load(test.data);'
        'var start=new Date();load(test.script);result[test.name]=new Date()-start;gc();}\n'
        "print('SUNSPIDER_JSON '+JSON.stringify(result));})();\n")
    return [('sunspider', harness, sun), ('octane', harness / 'test/octane', Path('run.js')),
            ('web_tooling', wtb, score), ('fixed_work', wtb, fixed)]


def parse_result(suite, row, contents):
    c.score_result(suite, row, contents)
    if suite == 'web_tooling':
        assert {x['name'] for x in row['tests']} == WTB_NAMES
        assert row['score'] > 0 and all(x['hz'] > 0 for x in row['tests'])
    elif suite == 'fixed_work':
        assert set(row['work_counts']) == WTB_NAMES
        row['test_milliseconds'] = json.loads(re.search(r'^WTB_FIXED_TESTS (.*)$', contents, re.M)[1])
        assert set(row['test_milliseconds']) == WTB_NAMES
        assert all(v >= 0 for v in row['test_milliseconds'].values())
        row['load_milliseconds'] = float(re.search(r'^WTB_LOAD_MS (.*)$', contents, re.M)[1])


def summary(rows):
    result = {}
    for suite in ['sunspider', 'octane', 'web_tooling', 'fixed_work']:
        metric = {'sunspider': 'total_milliseconds', 'octane': 'score',
                  'web_tooling': 'score', 'fixed_work': 'work_milliseconds'}[suite]
        groups = {m: [r for r in rows if r['suite'] == suite and r['mode'] == m] for m in MODES}
        item = {'metric': metric, 'higher_is_better': suite in ('octane', 'web_tooling'),
                'attempts': {m: len(g) for m, g in groups.items()},
                'failures': {m: sum(r['status'] != 'passed' for r in g) for m, g in groups.items()}}
        # Partial suites and incomplete repetitions never become acceptance scores.
        complete = all(len(g) == 3 and all(r['status'] == 'passed' for r in g) for g in groups.values())
        item['comparable'] = complete
        if complete:
            item['median'] = {m: statistics.median(r[metric] for r in g) for m, g in groups.items()}
            item['change_vs_full_pct'] = {m: (v / item['median']['full'] - 1) * 100 for m, v in item['median'].items()}
        result[suite] = item
    return result


def main():
    assert platform.machine() == 'aarch64'
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = root / ('gc-full-regression-' + arch)
    report.mkdir(exist_ok=True)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    assert int(os.environ['BENCHMARK_REPETITIONS']) == 3
    engine = source.parent / ('build-' + arch) / 'escargot'
    header = engine.read_bytes()[:20]
    elf_class, machine = (1, 40) if arch == 'arm32' else (2, 183)
    assert header[:4] == b'\x7fELF' and header[4] == elf_class
    assert struct.unpack('<H', header[18:20])[0] == machine
    manifest = json.loads((HERE / 'allocation-refined-source.json').read_text())
    assert c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil') == manifest['gc_revision']
    for name, sha in manifest['files'].items():
        assert c.sha256(source / name) == sha, name
    c.parse_audit = p.policy_audit
    envs = p.environments()
    prefix = []
    if arch == 'arm32':
        runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
        loader = os.environ['BENCHMARK_RUNTIME_LOADER']
        prefix = [loader, '--library-path', runtime]
        for env in envs.values():
            env['LD_LIBRARY_PATH'] = runtime
    commands = {m: [*prefix, str(engine)] + ([] if m == 'full' else ['--gc-generational']) for m in MODES}
    suites = prepare_drivers(harness, report)
    wtb = harness / 'test/web-tooling-benchmark'
    # Preserve generated inputs, including bundles needed to reproduce failures.
    for name in ['cli.js', 'memory.js']:
        shutil.copy2(wtb / 'dist' / name, report / ('input-' + name))
    shutil.copy2(engine, report / 'measured-engine')
    inputs = json.loads((root / 'benchmark-inputs.json').read_text())
    assert inputs['wtb_cli_sha256'] == c.sha256(wtb / 'dist/cli.js')
    assert inputs['wtb_memory_sha256'] == c.sha256(wtb / 'dist/memory.js')
    data = {'status': 'running', 'architecture': arch, 'cpu': cpu, 'monitor_cpu': monitor,
            'revision': c.output(['git', 'rev-parse', 'HEAD'], source), 'manifest': manifest,
            'gc_revision': manifest['gc_revision'], 'binary_sha256': c.sha256(engine),
            'harness_revision': c.output(['git', 'rev-parse', 'HEAD'], harness),
            'measured_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'cmake_options': json.loads((root / ('build-' + arch + '.json')).read_text())['cmake_options'],
            'inputs': inputs, 'driver_sha256': {s: c.sha256(d / f if not f.is_absolute() else f) for s, d, f in suites},
            'protocol': 'Same frozen runtime as the refined six-workload experiment. Native ARM32 then ARM64. Same binary/FSD12, full versus pattern4096 versus refined site64, shared settings and seed1. Three fresh-process cyclic-order blocks full/pattern/refined, pattern/refined/full, refined/full/pattern. Complete SunSpider26 with upstream-style timed load and untimed per-test gc; stock Octane17 including latency outputs with its default warmup; full WTB18 with exactly3 deterministic samples; separate WTB18 fixed-work three fn calls each, no forced GC, checkpoint timestamps expose per-item durations and load time. Score runs have no RSS or perf sampler. Process peak RSS/CPU captured, but adaptive score memory is not equal-work memory; fixed WTB is the equal-work comparison. Errors retained per attempt, no replacement runs; continue subsequent attempts and both architectures. A completed workflow is not a correctness pass: inspect failures and status. Partial or incomplete suite results are never aggregated as acceptance scores.',
            'rows': [], 'failures': [], 'summary': {}}
    def save():
        data['summary'] = summary(data['rows'])
        c.write_json(report / 'measurements.json', data)
    save()
    for suite, directory, script in suites:
        for repeat in range(3):
            for mode in MODES[repeat:] + MODES[:repeat]:
                label = f'{suite}-{repeat + 1}-{mode}'
                try:
                    row, contents = c.process_measure(label, mode, commands[mode], directory,
                        script, envs[mode], report, cpu, monitor, False)
                    row.update(suite=suite, mode=mode, repeat=repeat + 1)
                    parse_result(suite, row, contents)
                    row.update(status='passed', audits=p.a.audits(contents))
                    data['rows'].append(row)
                    print('SUITE PASS', suite, repeat + 1, mode, flush=True)
                except Exception as error:
                    contents = (report / (label + '.log')).read_text(errors='replace') if (report / (label + '.log')).exists() else ''
                    failure = {'suite': suite, 'mode': mode, 'repeat': repeat + 1,
                               'status': 'failed', 'error': repr(error), 'log': label + '.log',
                               'audits': p.a.audits(contents), 'log_tail': contents[-6000:]}
                    data['rows'].append(failure)
                    data['failures'].append(failure)
                    print('SUITE FAILED', suite, repeat + 1, mode, repr(error), flush=True)
                save()
    data['status'] = 'completed_with_failures' if data['failures'] else 'passed'
    save()
    print('FULL REGRESSION COMPLETE', arch, data['status'], json.dumps(data['summary']), flush=True)


if __name__ == '__main__':
    main()
