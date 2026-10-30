#!/usr/bin/env python3
"""A/B/C the interpreter program-counter changes inside one CI benchmark run.

Runs under tools/run-arm-development-benchmark.py (branch `bench`). The checked
out revision is the full change (`pcvol`); the two other variants differ only in
src/interpreter/ByteCodeInterpreter.cpp and are rebuilt here, so all three
binaries share the host, the compiler options and the CPU, and the samples are
interleaved in rotated order.
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

VARIANTS = [
    ('base', '25450dd9d51ddddad384c48f0c553851369c6f09'),
    ('pc', '563fe7f14c61954464812ecc7081b4845d8ad34f'),
]
FILES = ['src/interpreter/ByteCodeInterpreter.cpp']
LABELS = ['base', 'pc', 'pcvol']


def run(args, log, cwd=None):
    print('+', ' '.join(map(str, args)), flush=True)
    with log.open('w') as output:
        process = subprocess.Popen(list(map(str, args)), cwd=cwd, stdout=output, stderr=subprocess.STDOUT)
        while True:
            try:
                code = process.wait(timeout=60)
                break
            except subprocess.TimeoutExpired:
                print(log.name + ': ' + log.read_text(errors='replace')[-300:], flush=True)
        if code:
            print(log.read_text(errors='replace')[-3000:], flush=True)
            raise subprocess.CalledProcessError(code, args)
    return log.read_text(errors='replace')


def main():
    source = Path.cwd()
    work = source.parent
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    harness = Path(os.environ['BENCHMARK_HARNESS'])
    cpu = os.environ['BENCHMARK_CPU']
    repetitions = int(os.environ['BENCHMARK_REPETITIONS'])
    report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / ('pc-pointer-' + arch)
    report.mkdir()
    engine = work / ('build-' + arch) / 'escargot'
    shutil.copy2(engine, report / 'pcvol')

    saved = {f: (source / f).read_bytes() for f in FILES}
    run(['git', 'fetch', '--depth', '1', 'origin', *[sha for _, sha in VARIANTS]], report / 'variants-fetch.log')
    try:
        for label, revision in VARIANTS:
            for f in FILES:
                content = subprocess.check_output(['git', 'show', revision + ':' + f])
                if (source / f).read_bytes() != content:
                    (source / f).write_bytes(content)
            if arch == 'arm32':
                container = f"escargot-arm-benchmark-{os.environ['GITHUB_RUN_ID']}-{os.environ['GITHUB_RUN_ATTEMPT']}"
                info = json.loads(subprocess.check_output(['docker', 'inspect', container]))[0]
                assert any(m['Source'] == str(work) and m['Destination'] == '/work' for m in info['Mounts'])
                run(['docker', 'start', '-a', container], report / (label + '-build.log'))
            else:
                run(['cmake', '--build', work / ('build-' + arch), '--target', 'escargot', '--parallel', '8'],
                    report / (label + '-build.log'))
            shutil.copy2(engine, report / label)
    finally:
        for f, content in saved.items():
            (source / f).write_bytes(content)

    def cmd(label):
        command = [str(report / label)]
        if arch == 'arm32':
            command = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path',
                       os.environ['BENCHMARK_RUNTIME_DIRECTORY'], *command]
        return command

    data = {'architecture': arch, 'variants': dict(VARIANTS), 'labels': LABELS,
            'protocol': 'Octane run.js and WTB dist/cli.js, fresh process per sample, '
                        'labels in rotated/reversed order per repetition, same CPU and compiler options',
            'octane': {}, 'wtb': {}, 'binaries': {}}

    def save():
        (report / 'results.json').write_text(json.dumps(data, indent=2) + '\n')

    for label in LABELS:
        binary = report / label
        data['binaries'][label] = {'sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
                                   'size': binary.stat().st_size}
        run(['size', '-A', binary], report / (label + '-size.txt'))

    def order(rep):
        a = LABELS if rep % 2 == 0 else list(reversed(LABELS))
        shift = rep % len(a)
        return a[shift:] + a[:shift]

    octane = harness / 'test/octane'
    wtb = harness / 'test/web-tooling-benchmark'
    for repetition in range(repetitions):
        for label in order(repetition):
            log = report / f'{label}-octane-{repetition}.log'
            started = time.monotonic()
            text = run(['taskset', '-c', cpu, *cmd(label), 'run.js'], log, cwd=octane)
            total = re.search(r'Score \(version [^)]+\):\s*([0-9.]+)', text)
            assert total, 'Octane score missing'
            suites = {m[1]: float(m[2]) for m in re.finditer(r'^([A-Za-z0-9]+):\s*([0-9.]+)\s*$', text, re.M)}
            item = {'score': float(total[1]), 'suites': suites, 'seconds': time.monotonic() - started,
                    'log': log.name, 'round': repetition}
            data['octane'].setdefault(label, []).append(item)
            save()
            print('OCTANE_RESULT', arch, label, json.dumps({'score': item['score'], 'round': repetition}), flush=True)
        for label in order(repetition + 1):
            log = report / f'{label}-wtb-{repetition}.log'
            started = time.monotonic()
            text = run(['taskset', '-c', cpu, *cmd(label), 'dist/cli.js'], log, cwd=wtb)
            mean = re.search(r'Geometric mean:\s*([0-9.]+)\s+runs/s', text)
            assert mean, 'WTB geometric mean missing'
            tools = {m[1]: float(m[2]) for m in re.finditer(r'^\s*([a-z0-9-]+):\s*([0-9.]+)\s+runs/s', text, re.M)}
            item = {'score': float(mean[1]), 'tools': tools, 'seconds': time.monotonic() - started,
                    'log': log.name, 'round': repetition}
            data['wtb'].setdefault(label, []).append(item)
            save()
            print('WTB_RESULT', arch, label, json.dumps({'score': item['score'], 'round': repetition}), flush=True)
    data['validated'] = True
    save()
    print('PC_POINTER_DONE', arch, flush=True)


if __name__ == '__main__':
    main()
