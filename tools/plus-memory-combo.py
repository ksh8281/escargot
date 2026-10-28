#!/usr/bin/env python3
import hashlib,importlib.util,json,os,re,shutil,subprocess,sys
from pathlib import Path
PARENT='f4582b94d0ee7a9a3b21aeb08ca406b20209fef8'
FILES=['src/codecache/CodeCache.cpp','src/interpreter/ByteCode.h','src/interpreter/ByteCodeGenerator.cpp','src/interpreter/ByteCodeInterpreter.cpp','src/runtime/EncodedValue.h']
OCT=set('Richards DeltaBlue Crypto RayTrace EarleyBoyer RegExp Splay SplayLatency NavierStokes PdfJS Mandreel MandreelLatency Gameboy CodeLoad Box2D zlib Typescript'.split())
WTB=set('acorn babel babel-minify babylon buble chai coffeescript espree esprima jshint lebab postcss prepack prettier source-map terser typescript uglify-js'.split())
def run(args,log):
    print('+',' '.join(map(str,args)),flush=True)
    with log.open('w') as f:subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT,check=True)
def main():
    source=Path.cwd();work=source.parent;arch=os.environ['BENCHMARK_ARCHITECTURE'];harness=Path(os.environ['BENCHMARK_HARNESS'])
    report=Path(os.environ['BENCHMARK_OUTPUT_DIR'])/('memory-combo-'+arch);report.mkdir()
    engine=work/('build-'+arch)/'escargot';shutil.copy2(engine,report/'outline')
    saved={f:(source/f).read_bytes() for f in FILES}
    run(['git','fetch','--depth','1','origin',PARENT],report/'parent-fetch.log')
    try:
        for label,selected in [('heaponly',FILES[:-1]),('parent',[FILES[-1]])]:
            for f in selected:(source/f).write_bytes(subprocess.check_output(['git','show',PARENT+':'+f]))
            if arch=='arm32':
                container=f"escargot-arm-benchmark-{os.environ['GITHUB_RUN_ID']}-{os.environ['GITHUB_RUN_ATTEMPT']}"
                info=json.loads(subprocess.check_output(['docker','inspect',container]))[0]
                assert any(m['Source']==str(work) and m['Destination']=='/work' for m in info['Mounts'])
                run(['docker','start','-a',container],report/(label+'-build.log'))
            else:run(['cmake','--build',work/('build-'+arch),'--target','escargot','--parallel','8'],report/(label+'-build.log'))
            shutil.copy2(engine,report/label)
    finally:
        for f,content in saved.items():(source/f).write_bytes(content)
    def cmd(label):
        command=[str(report/label)]
        if arch=='arm32':command=[os.environ['BENCHMARK_RUNTIME_LOADER'],'--library-path',os.environ['BENCHMARK_RUNTIME_DIRECTORY'],*command]
        return command
    labels=['parent','heaponly','outline']
    def order_for(repetition):
        order=labels if arch=='arm32' else list(reversed(labels))
        return order[repetition:]+order[:repetition]
    data={'architecture':arch,'parent_revision':PARENT,'candidate_revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'protocol':'same-master three-way comparison: unchanged master, heap-store fix only, heap-store fix plus addition specialization; three fresh-process score samples and three fixed-work memory samples; rotated order','scores':{},'memory':{},'binaries':{}}
    def save():(report/'results.json').write_text(json.dumps(data,indent=2)+'\n')
    for label in labels:
        data['binaries'][label]={'sha256':hashlib.sha256((report/label).read_bytes()).hexdigest()}
        section=next(x for x in subprocess.check_output(['readelf','-SW',str(report/label)],text=True).splitlines() if re.search(r'\] \.text\s',x)).split();idx=section.index('.text')
        offset=int(section[idx+3],16);length=int(section[idx+4],16)
        with (report/label).open('rb') as f:f.seek(offset);text_bytes=f.read(length)
        data['binaries'][label].update(text_bytes=length,text_sha256=hashlib.sha256(text_bytes).hexdigest())
        run(['size','-A',report/label],report/(label+'-size.txt'));run(['nm','-S','-C',report/label],report/(label+'-symbols.txt'))
    sys.path.insert(0,str(harness/'tools'))
    spec=importlib.util.spec_from_file_location('monthly',harness/'tools/monthly-engine-benchmark.py');monthly=importlib.util.module_from_spec(spec);spec.loader.exec_module(monthly)
    monthly.REPORT=report;monthly.ROOT=harness
    # Pin the memory protocol to the source revision. The bench branch's older
    # v2 monitor does not retain the explicit per-result GC counts.
    memory_path=source/'tools/engine_memory_benchmark.py'
    spec=importlib.util.spec_from_file_location('combo_memory',memory_path);memory=importlib.util.module_from_spec(spec);spec.loader.exec_module(memory)
    assert memory.METHOD=='fixed-work-smaps-rollup-v3'
    data['memory_monitor_sha256']=hashlib.sha256(memory_path.read_bytes()).hexdigest()
    prior_text={'parent':'9c102b25f2b55447b3c8ac82f28856211a94bd0691aee6044acb9ef447945e6a','heaponly':'1c93daa0311dfc851f38a2c140b51873b4ac9192c5e3a784e36c1d939f45191c','outline':'afd1865635c4fe0bbb19acee5407667e01c7593f8e077f08872548b24dcf5f6f'}
    reuse_scores=arch=='arm32' and all(data['binaries'][label]['text_sha256']==prior_text[label] for label in labels)
    if reuse_scores:data['score_reference_run']=40720449751
    print('REUSE_ARM32_SCORES',reuse_scores,flush=True)
    for repetition in range(0 if reuse_scores else 3):
        for label in order_for(repetition):
            for suite,directory,script,key,expected in [('octane',harness/'test/octane','run.js','subtests_scores',OCT),('web_tooling',harness/'test/web-tooling-benchmark','dist/cli.js','subtests_runs_per_second',WTB)]:
                result=monthly.measure(f'{label}-{suite}-{repetition}',cmd(label),directory,os.environ,script=script,kind=suite)
                assert set(result[key])==expected
                data['scores'].setdefault(label,{}).setdefault(suite,[]).append(result);save()
    begin="benchmarkCheckpoint('baseline');\n";end="benchmarkCheckpoint('workload_end');\ngc();gc();benchmarkCheckpoint('after_gc');\n"
    octane=harness/'test/octane';wtb=harness/'test/web-tooling-benchmark'
    shutil.copyfile(source/'tools/test/octane/base_mem.js',octane/'base_mem.js')
    loads=re.findall(r"load\(base_dir \+ '([^']+)'\);",(octane/'run.js').read_text());assert len(loads)==20 and loads[0]=='base.js'
    driver=report/'octane-memory.js'
    driver.write_text(begin+'\n'.join('load('+json.dumps('base_mem.js' if f=='base.js' else f)+');' for f in loads)+'''\nbenchmarkCheckpoint('loaded');var memoryCounts={};var memoryGCCounts={};
var memoryRun=BenchmarkSuite.prototype.RunSingleBenchmark;
BenchmarkSuite.prototype.RunSingleBenchmark=function(benchmark,data){memoryCounts[benchmark.name]=(memoryCounts[benchmark.name]||0)+1;return memoryRun.call(this,benchmark,data);};
BenchmarkSuite.RunSuites({NotifyResult:function(name){gc();memoryGCCounts[name]=(memoryGCCounts[name]||0)+1;benchmarkCheckpoint('test:'+name);},NotifyError:function(name,error){throw Error(name+': '+error);},NotifyScore:function(){print('MEM_WORK_JSON '+JSON.stringify(memoryCounts));print('MEM_GC_JSON '+JSON.stringify(memoryGCCounts));}});
'''+end)
    web=report/'wtb-memory.js';web.write_text(begin+"load('dist/memory.js');\n"+end)
    cpu=int(os.environ['BENCHMARK_CPU']);monitor=next(c for c in sorted(os.sched_getaffinity(0)) if c!=cpu)
    data['memory_method']=memory.METHOD;signatures={}
    for repetition in range(3):
        for label in order_for(repetition):
            for suite,directory,script,iterations in [('octane',octane,driver,1),('web_tooling',wtb,web,3)]:
                sample=memory.measure_memory(f'{label}-{suite}-memory-{repetition}','escargot',cmd(label),directory,os.environ,script,report,cpu,monitor)
                assert len(sample['work_counts'])==18 and all(v==iterations for v in sample['work_counts'].values())
                if suite=='octane':
                    names=[p['label'][5:] for p in sample['checkpoints'] if p['label'].startswith('test:')]
                    assert set(names)==OCT and len(names)==17 and sample['gc_calls']==dict.fromkeys(names,1)
                if suite in signatures:assert sample['work_signature']==signatures[suite]
                signatures[suite]=sample['work_signature']
                data['memory'].setdefault(label,{}).setdefault(suite,[]).append(sample);save()
    if arch=='arm64':
        # The earlier three-way run already completed ARM32 profiles. Collect
        # only the missing same-base ARM64 WTB pair, with no all-symbol annotate.
        revision='06aeed8f1c190e392ca5a9d4198e49d39759009b'
        run(['git','fetch','--depth','1','origin',revision],report/'pr-fetch.log')
        try:
            for f in FILES:(source/f).write_bytes(subprocess.check_output(['git','show',revision+':'+f]))
            run(['cmake','--build',work/('build-'+arch),'--target','escargot','--parallel','8'],report/'pr-build.log')
            shutil.copy2(engine,report/'pr')
        finally:
            for f,content in saved.items():(source/f).write_bytes(content)
        profile=report/'wtb-profile.js'
        profile.write_text("function benchmarkCheckpoint(name){print('PHASE '+JSON.stringify({name:name,time:Date.now()/1000}));}\nload('dist/memory.js');benchmarkCheckpoint('done');\n")
        import time
        for label in ['parent','pr']:
            name='wtb-'+label;clock_offset=time.time()-time.monotonic()
            command=['taskset','-c',str(cpu),*cmd(label),str(profile)]
            log=report/(name+'-record.log')
            print('+ perf record',label,flush=True)
            with log.open('w') as f:subprocess.run(['perf','record','-q','-k','CLOCK_MONOTONIC','-e','cycles:u','-F','999','-g','--call-graph','fp','-o',str(report/(name+'.data')),'--',*command],cwd=wtb,stdout=f,stderr=subprocess.STDOUT,check=True)
            contents=log.read_text();assert len(json.loads(re.search(r'MEM_WORK_JSON (.*)',contents)[1]))==18
            phases=[json.loads(x[6:]) for x in contents.splitlines() if x.startswith('PHASE ')]
            (report/(name+'-phases.json')).write_text(json.dumps({'clock_offset':clock_offset,'phases':phases},indent=2)+'\n')
            run(['perf','report','--stdio','--no-children','--call-graph','none','--percent-limit','0.5','-i',report/(name+'.data')],report/(name+'-report.txt'))
            for prev,phase in zip(phases,phases[1:]):
                target=phase['name'].removeprefix('test:')
                if target not in ['terser','source-map','uglify-js']:continue
                span=f"{prev['time']-clock_offset:.9f},{phase['time']-clock_offset:.9f}"
                run(['perf','report','--stdio','--no-children','--call-graph','none','--percent-limit','0.5','--time',span,'-i',report/(name+'.data')],report/(name+'-'+target+'-report.txt'))
    data['validated']=True;save();print('MEMORY_COMBO_DONE',arch,flush=True)
if __name__=='__main__':main()
