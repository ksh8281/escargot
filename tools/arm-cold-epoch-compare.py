#!/usr/bin/env python3
"""Native ARM comparison of original, 10 MiB, and 4 MiB activation."""

import datetime
import errno
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import pty
import re
import selectors
import shlex
import signal
import statistics
import struct
import subprocess
import sys
import time
import tty


CONFIG = json.loads(Path(__file__).with_name('arm-cold-epoch-settings.json').read_text())


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def output(command, cwd=None):
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def run(command, cwd=None, env=None):
    print('+ ' + shlex.join(map(str, command)), flush=True)
    subprocess.run(command, cwd=cwd, env=env, check=True)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def control_build(source, architecture, root_report, mode):
    assert mode in ['original', 'previous']
    work = source.parent
    control = work / ('cold-epoch-' + mode)
    repository = CONFIG['baseline_repository']
    revision = CONFIG['baseline_revision' if mode == 'original' else 'previous_revision']
    if not control.exists():
        run(['git', 'init', '-q', str(control)])
        run(['git', 'remote', 'add', 'origin',
             'https://github.com/' + repository + '.git'], control)
        run(['git', 'fetch', '--depth', '1', 'origin', revision], control)
        run(['git', 'checkout', '--detach', '-q', 'FETCH_HEAD'], control)
        run(['git', 'submodule', 'update', '--init', '--jobs', '8',
             'third_party/GCutil', 'third_party/walrus'], control)
    assert output(['git', 'rev-parse', 'HEAD'], control) == revision
    if mode == 'previous':
        assert output(['git', 'rev-parse', 'HEAD'], control / 'third_party/GCutil') == CONFIG['gc_revision']
        for name, expected in CONFIG['expected_source_files_sha256'].items():
            if name != 'src/heap/Heap.cpp':
                assert sha256(control / name) == expected, name
        old_heap = (control / 'src/heap/Heap.cpp').read_text()
        old_call = 'GC_enable_generational(10 * 1024 * 1024);'
        assert old_heap.count(old_call) == 1
        assert old_heap.replace(old_call, 'GC_enable_generational(4 * 1024 * 1024);') == (source / 'src/heap/Heap.cpp').read_text()
    options = json.loads((root_report / ('build-' + architecture + '.json')).read_text())['cmake_options']
    build = work / ('build-' + mode + '-' + architecture)
    if architecture == 'arm32':
        image = (root_report / 'arm32-image.txt').read_text().strip()
        script = root_report / ('build-' + mode + '-arm32.py')
        script.write_text(
            'import subprocess\n'
            'commands = ' + repr([
                ['gcc', '--version'],
                ['cmake', '-S', '/work/' + control.name, '-B', '/work/' + build.name,
                 '-GNinja', *options],
                ['cmake', '--build', '/work/' + build.name, '--target', 'escargot', '--parallel', '8'],
            ]) + '\n'
            'for command in commands:\n'
            ' print("+", command, flush=True)\n'
            ' subprocess.run(command, check=True)\n')
        install = ('set -euo pipefail\nexport DEBIAN_FRONTEND=noninteractive\n'
                   'apt-get update\n'
                   'apt-get install -y --no-install-recommends ca-certificates cmake ninja-build '
                   'pkg-config libicu-dev gcc g++ make python3 file binutils util-linux\n'
                   'exec setpriv --reuid="$BENCHMARK_UID" --regid="$BENCHMARK_GID" '
                   '--clear-groups python3 -u /report/' + script.name + '\n')
        run(['docker', 'run', '--rm', '--platform', 'linux/arm/v7',
             '-v', str(work) + ':/work', '-v', str(root_report) + ':/report',
             '-w', '/work', '-e', 'BENCHMARK_UID=' + str(os.getuid()),
             '-e', 'BENCHMARK_GID=' + str(os.getgid()), image, 'bash', '-c', install])
    else:
        run(['gcc', '--version'])
        run(['cmake', '-S', str(control), '-B', str(build), '-GNinja', *options])
        run(['cmake', '--build', str(build), '--target', 'escargot', '--parallel', '8'])
    binary = build / 'escargot'
    assert binary.is_file(), binary
    return binary, options


def parse_audit(contents, mode, require_active=True):
    common = re.findall(r'^PHONE_MODE_AUDIT (.*)$', contents, re.M)
    assert len(common) == 1, ('Missing mode report', mode)
    result = {'common': json.loads(common[0])}
    expected_divisor = (int(CONFIG['settings']['env']['GC_FREE_SPACE_DIVISOR'])
                        if mode in ['candidate', 'previous'] else 12)
    assert result['common']['free_space_divisor'] == expected_divisor, result
    if mode == 'original':
        assert result['common']['incremental'] == 0, result
        return result
    rows = re.findall(r'^GENERATIONAL_AUDIT (.*)$', contents, re.M)
    assert len(rows) == 1, ('Missing collector report', mode)
    result['generational'] = gc = json.loads(rows[0])
    result['admission'] = [json.loads(x) for x in re.findall(
        r'^ADMISSION_AUDIT (.*)$', contents, re.M)]
    assert len(result['admission']) == 1, result
    if mode == 'current_full':
        assert gc['minor'] == 0 and gc['incremental'] == 0 and gc['vdb'] == 0, gc
    elif require_active:
        assert gc['minor'] > 0 and gc['full_frequency'] == 2, gc
        assert gc['time_limit'] == 999999, gc
        if gc['incremental'] != 1 or gc['vdb'] != 1:
            # Admission can legitimately finish an untracked parked epoch.
            admission = result['admission'][0]
            assert admission['parks'] > 0, result
            assert (admission['untracked_fulls'] > 0
                    or admission['full_rw_epochs'] > 0), result
    return result


def process_measure(label, mode, command, directory, script, env, report, cpu, monitor_cpu, require_active):
    """Read status for execution checks, without sampling RSS during scores/work."""
    master, slave = pty.openpty()
    tty.setraw(slave)
    selector = selectors.DefaultSelector()
    selector.register(master, selectors.EVENT_READ)
    log_path = report / (label + '.log')
    timing_path = report / (label + '.time.txt')
    original_affinity = os.sched_getaffinity(0)
    started = time.monotonic()
    next_status = started
    next_progress = started + 30
    metadata = []
    engine_pid = None
    process = None
    print('START', label, flush=True)
    try:
        os.sched_setaffinity(0, {monitor_cpu})
        process = subprocess.Popen(
            ['taskset', '-c', str(cpu), '/usr/bin/time', '-f', '%M %U %S', '-o', str(timing_path),
             *map(str, command), str(script)], cwd=directory, env=env,
            stdout=slave, stderr=slave, stdin=subprocess.DEVNULL, start_new_session=True)
        os.close(slave)
        slave = None
        with log_path.open('wb') as log:
            while process.poll() is None or selector.get_map():
                for key, _ in selector.select(0.1):
                    try:
                        data = os.read(master, 65536)
                    except OSError as error:
                        if error.errno != errno.EIO:
                            raise
                        data = b''
                    if not data:
                        selector.unregister(master)
                    else:
                        log.write(data)
                        log.flush()
                now = time.monotonic()
                if now >= next_status and process.poll() is None:
                    try:
                        children = Path(f'/proc/{process.pid}/task/{process.pid}/children').read_text().split()
                        if children:
                            engine_pid = int(children[0])
                            tasks = []
                            for path in Path(f'/proc/{engine_pid}/task').glob('*/status'):
                                status = path.read_text()
                                affinity = re.search(r'^Cpus_allowed_list:\s+(.*)$', status, re.M).group(1)
                                tracer = int(re.search(r'^TracerPid:\s+(\d+)', status, re.M).group(1))
                                assert affinity == str(cpu) and tracer == 0, (affinity, tracer)
                                tasks.append({'tid': int(path.parent.name), 'affinity': affinity, 'tracer': tracer})
                            frequency = Path(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq')
                            if tasks:
                                metadata.append({'seconds': now - started, 'tasks': tasks,
                                                 'frequency_khz': int(frequency.read_text()) if frequency.exists() else None})
                    except (FileNotFoundError, ProcessLookupError):
                        pass
                    next_status = now + 0.5
                if now >= next_progress:
                    print('PROGRESS', label, round(now - started), 's', flush=True)
                    next_progress = now + 30
                if now - started > 1800:
                    raise TimeoutError(label + ' exceeded 30 minutes')
        code = process.wait()
        elapsed = time.monotonic() - started
        if code:
            print(log_path.read_text(errors='replace')[-6000:], flush=True)
            raise RuntimeError(label + ' exited ' + str(code))
        assert metadata and all(x['tasks'] for x in metadata), label
        peak, user, system = timing_path.read_text().split()
        contents = log_path.read_text(errors='replace')
        result = {'label': label, 'mode': mode, 'seconds': elapsed,
                  'peak_rss_kib': int(peak), 'user_seconds': float(user), 'system_seconds': float(system),
                  'execution_samples': metadata, 'command': list(map(str, command)) + [str(script)],
                  'environment_overrides': {k: v for k, v in env.items() if k.startswith(('GC_', 'ESCARGOT_'))},
                  'log': log_path.name, 'mode_audit': parse_audit(contents, mode, require_active),
                  'rss_sampler': False}
        print('RESULT', label, round(elapsed, 3), 's', 'peakKiB', peak, flush=True)
        return result, contents
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        os.sched_setaffinity(0, original_affinity)
        selector.close()
        os.close(master)
        if slave is not None:
            os.close(slave)


def score_result(suite, result, contents):
    if suite == 'octane':
        result['score'] = float(re.search(r'Score \(version [^)]+\):\s*([0-9.]+)', contents).group(1))
        result['subtests'] = {m.group(1): float(m.group(2)) for m in re.finditer(
            r'^([A-Za-z][A-Za-z0-9-]*): ([0-9.]+)$', contents, re.M)}
        expected = {'Richards', 'DeltaBlue', 'Crypto', 'RayTrace', 'EarleyBoyer',
                    'RegExp', 'Splay', 'SplayLatency', 'NavierStokes', 'PdfJS',
                    'Mandreel', 'MandreelLatency', 'Gameboy', 'CodeLoad',
                    'Box2D', 'zlib', 'Typescript'}
        assert set(result['subtests']) == expected, result['subtests']
    elif suite == 'web_tooling':
        raw = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', contents, re.M).group(1))
        assert len(raw['tests']) == 18 and all(x['samples'] == 3 for x in raw['tests']), raw
        result.update(score=raw['score'], tests=raw['tests'])
    elif suite == 'fixed_work':
        counts = json.loads(re.search(r'^MEM_WORK_JSON (.*)$', contents, re.M).group(1))
        assert len(counts) == 18 and all(v == 3 for v in counts.values()), counts
        result.update(work_milliseconds=float(re.search(r'^WTB_FIXED_WORK_MS ([0-9.]+)', contents, re.M).group(1)),
                      work_counts=counts, work_signature=hashlib.sha256(json.dumps(counts, sort_keys=True).encode()).hexdigest())
    else:
        tests = json.loads(re.search(r'^SUNSPIDER_JSON (.*)$', contents, re.M).group(1))
        assert len(tests) == 26 and all(v >= 0 for v in tests.values()), tests
        result.update(total_milliseconds=sum(tests.values()), subtests=tests)


def main():
    assert platform.machine() == 'aarch64', platform.machine()
    source = Path.cwd().resolve()
    root_report = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    architecture = os.environ['BENCHMARK_ARCHITECTURE']
    assert architecture in ['arm32', 'arm64']
    report = root_report / ('cold-epoch-' + architecture)
    report.mkdir(exist_ok=True)
    repetitions = int(os.environ['BENCHMARK_REPETITIONS'])
    assert repetitions == 3, repetitions
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor_cpu = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    sys.path.insert(0, str(harness / 'tools'))
    spec = importlib.util.spec_from_file_location('monthly_gc_ci', harness / 'tools/monthly-engine-benchmark.py')
    monthly = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(monthly)
    monthly.ROOT, monthly.REPORT = harness, report
    from engine_memory_benchmark import aggregate, measure_memory
    data = {'status': 'preparing', 'architecture': architecture, 'host': platform.node(),
            'platform': platform.platform(), 'native': True, 'cpu': cpu, 'monitor_cpu': monitor_cpu,
            'measured_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'configuration': CONFIG, 'repetitions': repetitions, 'memory_repetitions': 2,
            'rows': [], 'memory': {}, 'pointer_bits': 32 if architecture == 'arm32' else 64,
            'candidate_revision': output(['git', 'rev-parse', 'HEAD'], source),
            'candidate_gc_revision': output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil')}
    write_json(report / 'measurements.json', data)
    try:
        assert data['candidate_gc_revision'] == CONFIG['gc_revision']
        for name, expected in CONFIG['expected_source_files_sha256'].items():
            assert sha256(source / name) == expected, name
        baseline, options = control_build(source, architecture, root_report, 'original')
        previous, previous_options = control_build(source, architecture, root_report, 'previous')
        assert previous_options == options
        candidate = source.parent / ('build-' + architecture) / 'escargot'
        for name, binary in [('original', baseline), ('previous', previous), ('candidate', candidate)]:
            header = binary.read_bytes()[:20]
            expected_class, expected_machine = (1, 40) if architecture == 'arm32' else (2, 183)
            assert header[:4] == b'\x7fELF' and header[4] == expected_class
            assert struct.unpack('<H', header[18:20])[0] == expected_machine
            data[name + '_binary_sha256'] = sha256(binary)
        data['cmake_options'] = options
        common_env = {k: v for k, v in os.environ.items() if not k.startswith(('GC_', 'ESCARGOT_'))}
        common_env.update(ESCARGOT_BENCH_SEED='1', LD_PRELOAD='')
        envs = {'original': common_env,
                'candidate': {**common_env, **CONFIG['settings']['env']},
                'previous': {**common_env, **CONFIG['settings']['env']}}
        if architecture == 'arm32':
            runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
            loader = os.environ['BENCHMARK_RUNTIME_LOADER']
            prefix = [loader, '--library-path', runtime]
            for env in envs.values():
                env['LD_LIBRARY_PATH'] = runtime
            data['runtime'] = {'loader_sha256': sha256(loader),
                               'libc_sha256': sha256(Path(runtime) / 'libc.so.6'),
                               'image': (root_report / 'arm32-image.txt').read_text().strip()}
        else:
            prefix = []
            data['runtime'] = {'ldd': output(['ldd', '--version']).splitlines()[0],
                               'libc_sha256': sha256('/lib/aarch64-linux-gnu/libc.so.6'),
                               'loader_sha256': sha256('/lib/ld-linux-aarch64.so.1')}
        commands = {'original': [*prefix, str(baseline)],
                    'candidate': [*prefix, str(candidate), *CONFIG['settings']['flags']],
                    'previous': [*prefix, str(previous), *CONFIG['settings']['flags']]}
        wtb = harness / 'test/web-tooling-benchmark'
        cli = (wtb / 'dist/cli.js').read_text()
        needle = '  console.log(`Geometric mean: ${align(hz.toFixed(2), 5, "right")} runs/s`);'
        assert cli.count(needle) == 1
        cli = cli.replace(needle, needle + "\n  print('WTB_SCORE_JSON ' + JSON.stringify({score:hz, tests:suite.map(b => ({name:b.name,hz:b.hz,samples:b.stats.sample.length,count:b.count}))}));", 1)
        score_driver = report / 'wtb-score.js'
        score_driver.write_text(cli)
        fixed_driver = report / 'wtb-work.js'
        fixed_driver.write_text("var benchmarkCheckpoint=function(){};\nvar workStart=Date.now();\nload('dist/memory.js');\nprint('WTB_FIXED_WORK_MS '+(Date.now()-workStart));\n")
        suites = [('octane', harness / 'test/octane', 'run.js'),
                  ('fixed_work', wtb, fixed_driver)]
        drivers = monthly.build_memory_drivers(wtb)
        data['input_sha256'] = {'wtb_original_cli': sha256(wtb / 'dist/cli.js'),
                                'wtb_score_driver': sha256(score_driver),
                                'wtb_memory_bundle': sha256(wtb / 'dist/memory.js'),
                                'wtb_work_driver': sha256(fixed_driver),
                                'octane_run': sha256(harness / 'test/octane/run.js')}
        data['score_protocol'] = 'Stock Octane: three runs each of original, previous10MiB, candidate4MiB in a three-position Latin-square order. Fixed WTB: three alternating original/candidate pairs. No RSS sampler in timing. WTB score loops omitted; fixed work loads memory.js and invokes18 fn3 times including loading, without checkpoint waits or explicit GC.'
        data['memory_protocol'] = 'Separate fixed-work-smaps-rollup-v2, 10ms sampling on monitor CPU; active-window averages exclude checkpoint waits; peak is whole-process ru_maxrss; two explicit fulls only after workload_end.'
        for path in (harness / 'test/octane').glob('*.js'):
            path.read_bytes()
        (wtb / 'dist/memory.js').read_bytes()
        data['status'] = 'scores'
        write_json(report / 'measurements.json', data)
        for repeat in range(repetitions):
            for suite, directory, script in suites:
                if suite == 'octane':
                    modes = ['original', 'previous', 'candidate']
                    order = modes[repeat:] + modes[:repeat]
                else:
                    order = ['original', 'candidate'] if repeat % 2 == 0 else ['candidate', 'original']
                for mode in order:
                    label = f'{architecture}-{suite}-{repeat + 1}-{mode}'
                    result, contents = process_measure(label, mode, commands[mode], directory, script,
                                                       envs[mode], report, cpu, monitor_cpu, True)
                    write_json(report / (label + '.json'), result)
                    score_result(suite, result, contents)
                    result.update(suite=suite, repetition=repeat + 1)
                    write_json(report / (label + '.json'), result)
                    data['rows'].append(result)
                    write_json(report / 'measurements.json', data)
                    print('VALUE', label, result.get('score', result.get('work_milliseconds', result.get('total_milliseconds'))), flush=True)
        data['status'] = 'memory'
        write_json(report / 'measurements.json', data)
        for suite in ['octane', 'web_tooling']:
            directory, script, count, iterations = drivers[suite]
            for memory_repeat, mode in enumerate(
                    ['original', 'previous', 'candidate', 'candidate', 'previous', 'original']):
                label = f'{architecture}-{suite}-memory-{memory_repeat + 1}-{mode}'
                sample = measure_memory(label, 'escargot', commands[mode], directory, envs[mode],
                                        script, report, cpu, monitor_cpu)
                assert len(sample['work_counts']) == count
                assert all(v == iterations for v in sample['work_counts'].values())
                contents = (report / sample['log']).read_text()
                sample['mode_audit'] = parse_audit(contents, mode)
                statuses = list(report.glob(label + '.checkpoint-*.status.txt'))
                assert statuses, label
                sample['checkpoint_tracer_pids'] = []
                for path in statuses:
                    tracer = int(re.search(r'^TracerPid:\s+(\d+)', path.read_text(), re.M).group(1))
                    assert tracer == 0, (label, tracer)
                    sample['checkpoint_tracer_pids'].append(tracer)
                previous = data['memory'].get(suite, {}).get(mode, {}).get('samples', [])
                data['memory'].setdefault(suite, {})[mode] = aggregate([*previous, sample])
                write_json(report / 'measurements.json', data)
                print('MEMORY', label, 'averageKiB', sample['average_rss_kib'],
                      'peakKiB', sample['peak_rss_kib'], flush=True)
            assert len({sample['work_signature']
                        for group in data['memory'][suite].values()
                        for sample in group['samples']}) == 1
        data['status'] = 'complete'
        print('EARLY TRACKING COMPLETE', architecture, flush=True)
    except BaseException as error:
        data.update(status='failed', error=type(error).__name__ + ': ' + str(error))
        raise
    finally:
        write_json(report / 'measurements.json', data)


if __name__ == '__main__':
    main()
