#!/usr/bin/env python3
"""Paired addition experiment for the upstream ARM development workflow."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys

WTB = set('acorn babel babel-minify babylon buble chai coffeescript espree esprima jshint lebab postcss prepack prettier source-map terser typescript uglify-js'.split())
OCTANE = set('Richards DeltaBlue Crypto RayTrace EarleyBoyer RegExp Splay SplayLatency NavierStokes PdfJS Mandreel MandreelLatency Gameboy CodeLoad Box2D zlib Typescript'.split())


def run(command, log=None):
    print('+', ' '.join(map(str, command)), flush=True)
    if log is None:
        subprocess.run(command, check=True)
        return
    with log.open('w') as output:
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, bufsize=1) as process:
            for line in process.stdout:
                print(line, end='', flush=True)
                output.write(line)
            if process.wait():
                raise RuntimeError(f'Command failed; see {log}')


def binary_info(binary):
    symbols = subprocess.check_output(['nm', '-S', '--size-sort', '-C', str(binary)], text=True)
    selected = {}
    for line in symbols.splitlines():
        fields = line.split(None, 3)
        if len(fields) == 4 and any(x in fields[3] for x in
                ('Interpreter::interpret(', 'InterpreterSlowPath::plusSlowCase(',
                 'InterpreterSlowPath::plusAdaptiveSlowCase(')):
            selected[fields[3]] = int(fields[1], 16)
    sizes = subprocess.check_output(['size', str(binary)], text=True)
    print(sizes, flush=True)
    return {'sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
            'file_bytes': binary.stat().st_size, 'size_output': sizes, 'symbols': selected}


def main():
    source = Path.cwd()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / f'final-plus-{arch}'
    report.mkdir()
    harness = Path(os.environ['BENCHMARK_HARNESS'])
    binary = report / 'candidate'
    shutil.copy2(source.parent / f'build-{arch}' / 'escargot', binary)
    command = [str(binary)]
    if arch == 'arm32':
        command = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path',
                   os.environ['BENCHMARK_RUNTIME_DIRECTORY'], str(binary)]
    data = {'candidate_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
            'architecture': arch, 'repetitions': int(os.environ['BENCHMARK_REPETITIONS']),
            'validated': False,
            'protocol': 'final candidate only; fresh processes; all suites required; no baseline rebuild or execution',
            'reference_run_url': 'https://github.com/Samsung/escargot/actions/runs/40659021726',
            'binaries': {'candidate': binary_info(binary)}, 'scores': {'candidate': {}}}
    output = report / 'results.json'
    sys.path.insert(0, str(harness / 'tools'))
    spec = importlib.util.spec_from_file_location('monthly', harness / 'tools/monthly-engine-benchmark.py')
    monthly = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(monthly)
    monthly.REPORT = report
    monthly.ROOT = harness
    suites = [('octane', harness / 'test/octane', 'run.js', 'score', 'subtests_scores', OCTANE),
              ('web_tooling', harness / 'test/web-tooling-benchmark', 'dist/cli.js',
               'score_runs_per_second', 'subtests_runs_per_second', WTB)]
    try:
        for iteration in range(data['repetitions']):
            for suite, directory, driver, metric, subtests, expected in suites:
                label = f'candidate-{suite}-{iteration + 1}'
                result = monthly.measure(label, command, directory, os.environ, script=driver, kind=suite)
                log = (report / result['log']).read_text()
                pattern = r'^\s*([a-z][a-z0-9-]*):\s*[0-9.]+\s+runs/s$' if suite == 'web_tooling' else r'^([A-Za-z][A-Za-z0-9-]*): [0-9.]+$'
                completed = re.findall(pattern, log, re.MULTILINE)
                if set(result[subtests]) != expected or len(completed) != len(expected):
                    raise RuntimeError(f'{label}: incomplete/duplicate suite results: {completed}')
                bucket = data['scores']['candidate'].setdefault(suite, {'metric': metric, 'samples': []})
                bucket['samples'].append(result)
                bucket['median'] = statistics.median(x[metric] for x in bucket['samples'])
                output.write_text(json.dumps(data, indent=2) + '\n')
                print(f'{label}: {metric}={result[metric]}, validated {len(expected)} results', flush=True)
        data['validated'] = True
    finally:
        output.write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps({'architecture': arch, 'medians': {suite: bucket['median'] for suite, bucket in data['scores']['candidate'].items()}, 'binaries': data['binaries']}, indent=2), flush=True)


if __name__ == '__main__':
    main()
