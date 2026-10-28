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
        for f in FILES:(source/f).write_bytes(subprocess.check_output(['git','show',PARENT+':'+f]))
        if arch=='arm32':
            container=f"escargot-arm-benchmark-{os.environ['GITHUB_RUN_ID']}-{os.environ['GITHUB_RUN_ATTEMPT']}"
            info=json.loads(subprocess.check_output(['docker','inspect',container]))[0]
            assert any(m['Source']==str(work) and m['Destination']=='/work' for m in info['Mounts'])
            run(['docker','start','-a',container],report/'parent-build.log')
        else:run(['cmake','--build',work/('build-'+arch),'--target','escargot','--parallel','8'],report/'parent-build.log')
        shutil.copy2(engine,report/'parent')
    finally:
        for f,content in saved.items():(source/f).write_bytes(content)
    def cmd(label):
        command=[str(report/label)]
        if arch=='arm32':command=[os.environ['BENCHMARK_RUNTIME_LOADER'],'--library-path',os.environ['BENCHMARK_RUNTIME_DIRECTORY'],*command]
        return command
    data={'architecture':arch,'parent_revision':PARENT,'candidate_revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'protocol':'same-master paired comparison; three fresh-process score samples and three fixed-work memory samples; alternating order','scores':{},'memory':{},'binaries':{}}
    def save():(report/'results.json').write_text(json.dumps(data,indent=2)+'\n')
    for label in ['parent','outline']:
        data['binaries'][label]={'sha256':hashlib.sha256((report/label).read_bytes()).hexdigest()}
        run(['size','-A',report/label],report/(label+'-size.txt'));run(['nm','-S','-C',report/label],report/(label+'-symbols.txt'))
    sys.path.insert(0,str(harness/'tools'))
    spec=importlib.util.spec_from_file_location('monthly',harness/'tools/monthly-engine-benchmark.py');monthly=importlib.util.module_from_spec(spec);spec.loader.exec_module(monthly)
    monthly.REPORT=report;monthly.ROOT=harness
    for repetition in range(3):
        order=['parent','outline'] if (repetition+(arch=='arm64'))%2==0 else ['outline','parent']
        for label in order:
            for suite,directory,script,key,expected in [('octane',harness/'test/octane','run.js','subtests_scores',OCT),('web_tooling',harness/'test/web-tooling-benchmark','dist/cli.js','subtests_runs_per_second',WTB)]:
                result=monthly.measure(f'{label}-{suite}-{repetition}',cmd(label),directory,os.environ,script=script,kind=suite)
                assert set(result[key])==expected
                data['scores'].setdefault(label,{}).setdefault(suite,[]).append(result);save()
    begin="benchmarkCheckpoint('baseline');\n";end="benchmarkCheckpoint('workload_end');\ngc();gc();benchmarkCheckpoint('after_gc');\n"
    octane=harness/'test/octane';wtb=harness/'test/web-tooling-benchmark'
    shutil.copyfile(harness/'tools/test/octane/base_mem.js',octane/'base_mem.js')
    loads=re.findall(r"load\(base_dir \+ '([^']+)'\);",(octane/'run.js').read_text());assert len(loads)==20 and loads[0]=='base.js'
    driver=report/'octane-memory.js'
    driver.write_text(begin+'\n'.join('load('+json.dumps('base_mem.js' if f=='base.js' else f)+');' for f in loads)+'''\nbenchmarkCheckpoint('loaded');var memoryCounts={};var memoryGCCounts={};
var memoryRun=BenchmarkSuite.prototype.RunSingleBenchmark;
BenchmarkSuite.prototype.RunSingleBenchmark=function(benchmark,data){memoryCounts[benchmark.name]=(memoryCounts[benchmark.name]||0)+1;return memoryRun.call(this,benchmark,data);};
BenchmarkSuite.RunSuites({NotifyResult:function(name){gc();memoryGCCounts[name]=(memoryGCCounts[name]||0)+1;benchmarkCheckpoint('test:'+name);},NotifyError:function(name,error){throw Error(name+': '+error);},NotifyScore:function(){print('MEM_WORK_JSON '+JSON.stringify(memoryCounts));print('MEM_GC_JSON '+JSON.stringify(memoryGCCounts));}});
'''+end)
    web=report/'wtb-memory.js';web.write_text(begin+"load('dist/memory.js');\n"+end)
    cpu=int(os.environ['BENCHMARK_CPU']);monitor=next(c for c in sorted(os.sched_getaffinity(0)) if c!=cpu)
    data['memory_method']=monthly.METHOD;signatures={}
    for repetition in range(3):
        order=['parent','outline'] if (repetition+(arch=='arm64'))%2==0 else ['outline','parent']
        for label in order:
            for suite,directory,script,iterations in [('octane',octane,driver,1),('web_tooling',wtb,web,3)]:
                sample=monthly.measure_memory(f'{label}-{suite}-memory-{repetition}','escargot',cmd(label),directory,os.environ,script,report,cpu,monitor)
                assert len(sample['work_counts'])==18 and all(v==iterations for v in sample['work_counts'].values())
                if suite=='octane':
                    names=[p['label'][5:] for p in sample['checkpoints'] if p['label'].startswith('test:')]
                    assert set(names)==OCT and len(names)==17 and sample['gc_calls']==dict.fromkeys(names,1)
                if suite in signatures:assert sample['work_signature']==signatures[suite]
                signatures[suite]=sample['work_signature']
                data['memory'].setdefault(label,{}).setdefault(suite,[]).append(sample);save()
    data['validated']=True;save();print('MEMORY_COMBO_DONE',arch,flush=True)
if __name__=='__main__':main()
