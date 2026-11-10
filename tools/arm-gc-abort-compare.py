#!/usr/bin/env python3
"""Focused native comparison for accounting of abandoned minor collections."""
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
BASE_ENGINE = 'bcf60de381b14b13251534573dc5ba0993d2651a'


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


def main():
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    assert arch == 'arm32' and int(os.environ['BENCHMARK_REPETITIONS']) == 3
    report = root / 'gc-abort-arm32'
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
    original = (harness / 'test/octane/run.js').read_text()
    focused = []
    for name in ['Splay', 'PdfJS', 'CodeLoad']:
        driver = report / (name + '.js')
        driver.write_text(original.replace('BenchmarkSuite.RunSuites(',
            'BenchmarkSuite.suites=BenchmarkSuite.suites.filter(function(s){return s.name===' + json.dumps(name) + ';});\nBenchmarkSuite.RunSuites(', 1))
        focused.append((name, harness / 'test/octane', driver))
    cli = (report / 'wtb-score.js').read_text()
    assert cli.count('\nsuite.run();\n') == 1
    terser = report / 'terser.js'
    terser.write_text(cli.replace('\nsuite.run();\n', '\nfor(var ti=suite.length-1;ti>=0;ti--)if(suite[ti].name!=="terser")suite.splice(ti,1);\nsuite.run();\n'))
    focused.insert(1, ('terser', harness / 'test/web-tooling-benchmark', terser))
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
        'protocol': 'Native ARM32; identical build directory/toolchain with only alloc.c reverted to the frozen baseline for control; source restored after building. Full control is baseline binary with GC off. Focused original-preload Splay, Terser, PdfJS, CodeLoad: 3 fresh-process cyclic full/before/after blocks. These isolate workloads and do not preserve preceding suite heap history. Then one full original Octane and WTB before/after pair preserves suite order and history; single pairs are context diagnostics, not stable aggregate acceptance. FSD12, seed1, original warmup, WTB 3 samples. No RSS sampler. Trace enabled only on focused cases.',
        'rows': [], 'failures': []}
    def save():
        c.write_json(report / 'measurements.json', data)
    def measure(name, directory, script, mode, repeat, context=False):
        label = ('context-' if context else 'focus-') + name + '-' + str(repeat) + '-' + mode
        env = dict(envs['full' if mode == 'full' else 'refined'])
        if not context:
            env['ESCARGOT_GC_CYCLE_TRACE'] = '1'
        try:
            row, text = c.process_measure(label, mode, commands[mode], directory, script, env, report, cpu, monitor, False)
            if context:
                r.parse_result(name, row, text)
            elif name == 'terser':
                row['score'] = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', text, re.M)[1])['score']
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
    for name, directory, script in focused:
        for rep in range(3):
            modes = ['full', 'before', 'after']
            for mode in modes[rep:] + modes[:rep]:
                measure(name, directory, script, mode, rep + 1)
    for name, directory, script in stock:
        if name not in ['octane', 'web_tooling']:
            continue
        for mode in (['before', 'after'] if name == 'octane' else ['after', 'before']):
            measure(name, directory, script, mode, 1, True)
    data['status'] = 'failed' if data['failures'] else 'passed'
    save()
    assert not data['failures'], data['failures']
    print('FOCUSED COMPARISON COMPLETE', len(data['rows']), flush=True)


if __name__ == '__main__':
    main()
