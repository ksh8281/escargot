#!/usr/bin/env python3
"""Compare eager and lazy mprotect preparation with identical full-GC policy."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import signal
import statistics
import subprocess
import sys

BEFORE = '19f8935220ef44a992b632b3c47a4b8ce670ad01'

def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')

def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def instrument(source):
    old = '''GC_protect_heap(void)
{
#  ifdef GC_COARSE_MPROTECT'''
    new = '''GC_protect_heap(void)
{
#  ifndef NO_CLOCK
  CLOCK_TYPE start = CLOCK_TYPE_INITIALIZER;
  CLOCK_TYPE aged = CLOCK_TYPE_INITIALIZER;
  CLOCK_TYPE prepared = CLOCK_TYPE_INITIALIZER;
  GC_bool timed = GC_PRINT_STATS_FLAG;
  if (timed)
    GET_TIME(start);
#  endif
#  ifdef GC_COARSE_MPROTECT'''
    assert source.count(old) == 1
    source = source.replace(old, new)
    age = '    GC_coarse_advance_age();' if '    GC_coarse_advance_age();' in source else '    GC_coarse_advance_age(GC_gc_no);'
    assert source.count(age) == 1
    source = source.replace(age, age + '\n#    ifndef NO_CLOCK\n    if (timed)\n      GET_TIME(aged);\n#    endif')
    old = '''  CLOCK_TYPE start = CLOCK_TYPE_INITIALIZER;
  GC_bool timed = GC_PRINT_STATS_FLAG;
  if (timed)
    GET_TIME(start);'''
    new = '''  if (timed)
    GET_TIME(prepared);'''
    assert source.count(old) == 1
    source = source.replace(old, new)
    old = '''    GC_COND_LOG_PRINTF("Mprotect heap protection took %lu ms %lu ns\\n",
                       MS_TIME_DIFF(now, start), NS_FRAC_TIME_DIFF(now, start));'''
    new = '''    GC_COND_LOG_PRINTF("Mprotect stages: age %lu ms %lu ns, sync %lu ms %lu ns, visit %lu ms %lu ns\\n",
                       MS_TIME_DIFF(aged, start), NS_FRAC_TIME_DIFF(aged, start),
                       MS_TIME_DIFF(prepared, aged), NS_FRAC_TIME_DIFF(prepared, aged),
                       MS_TIME_DIFF(now, prepared), NS_FRAC_TIME_DIFF(now, prepared));
''' + old
    assert source.count(old) == 1
    return source.replace(old, new)

def build_control(report, engine, mode, profile=False):
    engine = Path(engine)
    build = engine.parent
    commands = subprocess.check_output(['ninja', '-t', 'commands', 'escargot'], cwd=build, text=True).splitlines()
    gc = Path(__file__).resolve().parent.parent / 'third_party/GCutil'
    directory = report / (mode + ('-profile' if profile else '-control'))
    directory.mkdir(exist_ok=True)
    objects, compile_commands = [], []
    # The allocation/free hooks and the bitmap implementation are paired.
    for unit in ('os_dep', 'allchblk'):
        source_text = (subprocess.check_output(['git', 'show', BEFORE + ':' + unit + '.c'], cwd=gc, text=True)
                       if mode == 'before' else (gc / (unit + '.c')).read_text())
        if profile and unit == 'os_dep':
            source_text = instrument(source_text)
        source = directory / (unit + '.c'); source.write_text(source_text)
        raw = shlex.split(next(line for line in commands if ' -c ' in line and '/GCutil/' + unit + '.c' in line))
        args, i = [raw[0]], 1
        while i < len(raw):
            if raw[i] in ('-o', '-MF', '-MT'): i += 2
            elif raw[i] == '-MD': i += 1
            else: args.append(raw[i]); i += 1
        args[-1] = str(source)
        obj = directory / (unit + '.o'); args += ['-o', str(obj)]
        subprocess.run(args, cwd=build, check=True)
        objects.append(str(obj)); compile_commands.append(args)
    link = shlex.split(commands[-1].split(' && ')[1])
    binary = directory / 'escargot'; link[link.index('-o') + 1] = str(binary)
    archive = next(i for i, value in enumerate(link) if value.endswith('/libgc-lib.a'))
    link[archive:archive] = objects
    subprocess.run(link, cwd=build, check=True)
    write_json(directory / 'control.json', {'before_gc': BEFORE, 'mode': mode, 'profiling': profile,
               'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
               'compile_commands': compile_commands, 'link_command': link})
    print('PROTECTION_CONTROL_BUILT', mode, profile, binary, flush=True)
    return binary

def audit_output(text, enabled, automatic=True):
    records = re.findall(r'^FULLGC_AUDIT (\{.*\})$', text, re.M)
    assert len(records) == 1, records
    audit = json.loads(records[0])
    expected = {'time_limit': 999999, 'full_frequency': 0,
                'automatic_disabled': int(not automatic), 'adaptive': 0,
                'incremental': int(enabled), 'actual_vdb': int(enabled), 'partial_collections': 0}
    assert all(audit[key] == value for key, value in expected.items()), audit
    return audit

def profile_stages(report, command, engine, wtb, cpu):
    result = {}
    script = report / 'protection-phase.js'
    script.write_text("function benchmarkCheckpoint(label) {print('GC_PHASE '+label);}\nbenchmarkCheckpoint('baseline');\nload('dist/memory.js');\nbenchmarkCheckpoint('done');\n")
    env = dict(os.environ, GC_PRINT_STATS='1', GC_PROFILE_MPROTECT='1', ESCARGOT_FULLGC_PHASE_AUDIT='1')
    pattern = r'Mprotect stages: age (\d+) ms (\d+) ns, sync (\d+) ms (\d+) ns, visit (\d+) ms (\d+) ns'
    for mode in ('before', 'after'):
        profile_engine = build_control(report, engine, mode, profile=True)
        args = [str(profile_engine) if value == engine else value for value in command]
        log = report / (mode + '-protection-stages.log')
        with log.open('w') as stream:
            process = subprocess.Popen(['taskset', '-c', str(cpu), *args, '--gc-incremental', str(script)],
                                       cwd=wtb, env=env, stdout=stream, stderr=subprocess.STDOUT)
            while True:
                try:
                    code = process.wait(timeout=45); break
                except subprocess.TimeoutExpired:
                    print('PROTECTION_PROFILE_PROGRESS', mode, flush=True)
        assert code == 0, (mode, log)
        text = log.read_text()
        work = json.loads(re.search(r'^MEM_WORK_JSON (\{.*\})$', text, re.M)[1])
        assert len(work) == 18 and set(work.values()) == {3}, work
        audit = audit_output(text, True)
        tests = {}; pending = []
        for line in text.splitlines():
            match = re.search(pattern, line)
            if match:
                nums = list(map(int, match.groups()))
                pending.append([nums[i] + nums[i + 1] / 1e6 for i in (0, 2, 4)])
            elif line.startswith('GC_PHASE_AUDIT '):
                label = json.loads(line[len('GC_PHASE_AUDIT '):])['label']
                if pending:
                    tests[label] = {'passes': len(pending),
                        **{name + '_ms': sum(row[i] for row in pending) for i, name in enumerate(('age', 'sync', 'visit'))}}
                    pending = []
        stages = [[int(match[i]) + int(match[i+1]) / 1e6 for i in (0,2,4)]
                  for match in re.findall(pattern, text)]
        assert stages
        result[mode] = {'passes': len(stages), 'gc_audit': audit, 'tests': tests,
            'total_ms': sum(sum(row) for row in stages),
            **{name + '_ms': sum(row[i] for row in stages) for i, name in enumerate(('age', 'sync', 'visit'))}}
        print('PROTECTION_STAGES', mode, json.dumps(result[mode]), flush=True)
        write_json(report / 'protection-stages.json', result)

def fixed_cost(report, command, engine, before, harness, cpu):
    directory = report / 'fixed-fullgc'; directory.mkdir(exist_ok=True)
    cost = load_module('protection_cost', Path(__file__).with_name('fullgc-cost-helpers.py'))
    cwd, _, script = cost.drivers(harness, directory)['web_tooling']
    env = dict(os.environ, ESCARGOT_GC_DISABLE_AUTOMATIC='1')
    old = [str(before) if value == engine else value for value in command]
    modes = [('ordinary', old), ('ordinary_after', command), ('before', [*old, '--gc-incremental']), ('after', [*command, '--gc-incremental'])]
    rows = []
    saved = int(Path('/proc/sys/kernel/perf_event_paranoid').read_text())
    try:
        if saved > 0:
            subprocess.run(['sudo', '-n', 'sysctl', '-w', 'kernel.perf_event_paranoid=0'], check=True)
        for rep in range(3):
            for mode, args in modes[rep:] + modes[:rep]:
                values, text = cost.timed_run(f'{mode}-{rep}', args, cwd, env, script, directory, cpu)
                audit = cost.validate_output('ordinary' if mode.startswith('ordinary') else 'mprotect', text)
                assert audit['gc_audit']['collections'] == 41, audit
                if rows:
                    assert rows[0]['work_counts'] == audit['work_counts'] and rows[0]['gc_calls'] == audit['gc_calls']
                rows.append({'mode': mode, 'rep': rep, **values, **audit})
                write_json(directory / 'measurements.json', rows)
            medians = {mode: statistics.median(row['cycles:u'] + row['cycles:k'] for row in rows if row['mode'] == mode)
                       for mode, _ in modes}
            summary = {'repetitions': rep + 1, 'median_cycles': medians,
                       'vdb_overhead_percent': {mode: 100*(medians[mode]/medians['ordinary']-1) for mode in ('before','after')},
                       'ordinary_code_change_percent': 100*(medians['ordinary_after']/medians['ordinary']-1)}
            write_json(directory / 'summary.json', summary)
            print('PROTECTION_FIXED_COST', json.dumps(summary), flush=True)
    finally:
        if saved > 0:
            subprocess.run(['sudo','-n','sysctl','-w','kernel.perf_event_paranoid='+str(saved)],check=True)
        write_json(directory/'perf-permissions.json',{'original':saved,'restored':int(Path('/proc/sys/kernel/perf_event_paranoid').read_text())})

def compare(monthly, engines, harness, wtb, report, data, repetitions):
    from engine_memory_benchmark import aggregate, measure_memory
    assert repetitions == 3 and len(engines) == 1
    _, command, _ = engines[0]
    engine = os.environ['ESCARGOT_ENGINE']
    before = build_control(report, engine, 'before')
    old = [str(before) if value == engine else value for value in command]
    modes = [('ordinary', old, False), ('before', [*old,'--gc-incremental'],True), ('after',[*command,'--gc-incremental'],True)]
    summary = {mode:{'score_samples':[],'memory_samples':[]} for mode,_,_ in modes}
    data['protocol'].update(gc_policy='Automatic synchronous full GC in every mode',
        comparison='VDB off, byte permission scans, word permission scans and cached section lookup; unchanged grace policy',
        score_repetitions=3, memory_repetitions=1, score_order='Cyclic rotation across modes')
    for rep in range(3):
        for mode,args,enabled in modes[rep:]+modes[:rep]:
            label = f'escargot-{mode}-web_tooling-arm64-score-{rep+1}'
            sample = monthly.measure(label,args,wtb,os.environ,script='dist/cli.js',kind='web_tooling')
            sample['gc_audit'] = audit_output((report/sample['log']).read_text(),enabled)
            summary[mode]['score_samples'].append(sample)
            summary[mode]['score'] = statistics.median(row['score_runs_per_second'] for row in summary[mode]['score_samples'])
            write_json(report/'protection-comparison.json',summary)
            print('PROTECTION_SCORE',mode,rep+1,sample['score_runs_per_second'],sample['gc_audit'],flush=True)
    directory,driver,count,iterations = monthly.build_memory_drivers(wtb)['web_tooling']
    signature = None
    for mode,args,enabled in modes:
        sample = measure_memory(f'escargot-{mode}-web_tooling-arm64-memory-1','escargot',args,directory,os.environ,driver,report,data['cpu'],data['monitor_cpu'])
        assert len(sample['work_counts']) == count and set(sample['work_counts'].values()) == {iterations}
        sample['gc_audit'] = audit_output((report/sample['log']).read_text(),enabled)
        if signature is None: signature = sample['work_signature']
        assert sample['work_signature'] == signature
        summary[mode]['memory_samples'].append(sample)
        summary[mode]['memory'] = aggregate(summary[mode]['memory_samples'])
        write_json(report/'protection-comparison.json',summary)
        print('PROTECTION_MEMORY',mode,sample['gc_audit'],flush=True)
    data.update(scores_validated=True,memory_validated=True,failures=[],protection_comparison=summary)
    data['memory_driver_sha256'] = monthly.sha256(driver)
    data['wtb_score_bundle_sha256'] = monthly.sha256(wtb/'dist/cli.js')
    data['wtb_memory_bundle_sha256'] = monthly.sha256(wtb/'dist/memory.js')
    profile_stages(report,command,engine,wtb,data['cpu'])
    fixed_cost(report,command,engine,before,harness,data['cpu'])

def main():
    for key in ('GC_ENABLE_INCREMENTAL','GC_DISABLE_INCREMENTAL','GC_PRINT_STATS','GC_PRINT_VERBOSE_STATS','GC_PROFILE_MPROTECT','GC_FULL_FREQUENCY','GC_PAUSE_TIME_TARGET','ESCARGOT_GC_DISABLE_AUTOMATIC','GC_MPROTECT_VERIFY_ELIGIBILITY'):
        os.environ.pop(key,None)
    os.environ.update(GC_USE_GETWRITEWATCH='0',GC_FREE_SPACE_DIVISOR='12',GC_INCREMENTAL_START_THRESHOLD='10M',GC_MPROTECT_COARSE='1',ESCARGOT_GC_TIME_UNLIMITED='1',ESCARGOT_GC_FULL_EVERY_COLLECTION='1',ESCARGOT_FULLGC_AUDIT='1')
    harness=Path(os.environ['BENCHMARK_HARNESS'])
    experiment=load_module('protection_experiment',harness/'tools/arm-v8-lite-experiment.py')
    experiment.run_scores=compare
    experiment.main(mode='score',engine_names=('escargot',))

if __name__ == '__main__':
    def terminate(*_): raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,terminate)
    if os.environ['BENCHMARK_ARCHITECTURE'] != 'arm64':
        print('PROTECTION_ARM32_SKIPPED',flush=True); sys.exit(0)
    original=int(Path('/proc/sys/vm/max_map_count').read_text())
    changed=original<1000000
    report=Path(os.environ['BENCHMARK_OUTPUT_DIR'])
    try:
        if changed: subprocess.run(['sudo','-n','sysctl','-w','vm.max_map_count=1000000'],check=True)
        main()
    finally:
        if changed: subprocess.run(['sudo','-n','sysctl','-w','vm.max_map_count='+str(original)],check=True)
        write_json(report/'mapping-budget-arm64.json',{'original':original,'restored':int(Path('/proc/sys/vm/max_map_count').read_text()),'compiled_soft_limit':250000})
