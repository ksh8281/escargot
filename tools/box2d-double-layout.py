#!/usr/bin/env python3
import hashlib,json,os,re,shutil,subprocess,time
from pathlib import Path
PARENT='7e2717172d9cecb14f170282285db46d91fda41f'
CURRENT='2cb05d9bf6be16ac7d3a1fa514e6183d1d77ae8e'
FILES=['src/interpreter/ByteCode.h','src/interpreter/ByteCodeGenerator.cpp','src/interpreter/ByteCodeInterpreter.cpp','src/runtime/EncodedValue.h']
def run(args,log,cwd=None):
    print('+',' '.join(map(str,args)),flush=True)
    with log.open('w') as f:
        p=subprocess.Popen(list(map(str,args)),cwd=cwd,stdout=f,stderr=subprocess.STDOUT)
        while True:
            try:code=p.wait(timeout=30);break
            except subprocess.TimeoutExpired:
                print(log.name+': '+log.read_text(errors='replace')[-500:],flush=True)
        if code:raise subprocess.CalledProcessError(code,args)
    print(log.name+': '+log.read_text(errors='replace')[-1000:],flush=True)
def main():
    source=Path.cwd();work=source.parent;arch=os.environ['BENCHMARK_ARCHITECTURE'];harness=Path(os.environ['BENCHMARK_HARNESS'])
    report=Path(os.environ['BENCHMARK_OUTPUT_DIR'])/('box2d-layout-'+arch);report.mkdir()
    engine=work/('build-'+arch)/'escargot';shutil.copy2(engine,report/'double_likely')
    saved={f:(source/f).read_bytes() for f in FILES}
    run(['git','fetch','--depth','1','origin',PARENT,CURRENT],report/'variants-fetch.log')
    try:
        for label,revision in [('current',CURRENT),('parent',PARENT)]:
            for f in FILES:
                content=subprocess.check_output(['git','show',revision+':'+f])
                if (source/f).read_bytes()!=content:(source/f).write_bytes(content)
            if arch=='arm32':
                container=f"escargot-arm-benchmark-{os.environ['GITHUB_RUN_ID']}-{os.environ['GITHUB_RUN_ATTEMPT']}"
                info=json.loads(subprocess.check_output(['docker','inspect',container]))[0]
                assert any(m['Source']==str(work) and m['Destination']=='/work' for m in info['Mounts'])
                run(['docker','start','-a',container],report/(label+'-build.log'))
            else:run(['cmake','--build',work/('build-'+arch),'--target','escargot','--parallel','8'],report/(label+'-build.log'))
            shutil.copy2(engine,report/label)
    finally:
        for f,content in saved.items():(source/f).write_bytes(content)
    labels=['parent','current','double_likely'];cpu=os.environ['BENCHMARK_CPU'];octane=harness/'test/octane'
    def cmd(label):
        command=[str(report/label)]
        if arch=='arm32':command=[os.environ['BENCHMARK_RUNTIME_LOADER'],'--library-path',os.environ['BENCHMARK_RUNTIME_DIRECTORY'],*command]
        return command
    data={'architecture':arch,'parent_revision':PARENT,'current_revision':CURRENT,'candidate_revision':'f9dc244fb6abe78c7734d247f8c01138209597eb','protocol':'Box2D only: five score rounds in rotated/reversed order, three fixed-work perf rounds; fresh process per sample; same CPU, source baseline and compiler options','scores':{},'perf':{},'binaries':{}}
    def save():(report/'results.json').write_text(json.dumps(data,indent=2)+'\n')
    for label in labels:
        binary=report/label
        data['binaries'][label]={'sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
        run(['size','-A',binary],report/(label+'-size.txt'));run(['nm','-S','-C',binary],report/(label+'-symbols.txt'))
        if arch=='arm64':
            symbol='_ZN8Escargot11Interpreter9interpretEPNS_14ExecutionStateEPNS_13ByteCodeBlockEmPNS_5ValueE'
            run(['objdump','-dl','--disassemble='+symbol,binary],report/(label+'-interpreter.asm'))
    score=report/'box2d-score.js';score.write_text("load('base.js');load('box2d.js');BenchmarkSuite.RunSuites({NotifyResult:function(n,s){print('BOX2D_SCORE '+s);},NotifyError:function(n,e){throw Error(n+': '+e);},NotifyScore:function(s){print('BOX2D_DONE '+s);}});\n")
    fixed=report/'box2d-fixed.js';fixed.write_text("load('base.js');load('box2d.js');BenchmarkSuite.ResetRNG();setupBox2D();runBox2D();for(var i=0;i<60;i++)runBox2D();tearDownBox2D();print('BOX2D_FIXED_DONE 61');\n")
    data['drivers']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [score,fixed]}
    def order(rep):
        a=labels if rep%2==0 else list(reversed(labels));shift=rep%len(a);return a[shift:]+a[:shift]
    for repetition in range(int(os.environ['BENCHMARK_REPETITIONS'])):
        for label in order(repetition):
            log=report/f'{label}-score-{repetition}.log';started=time.monotonic()
            run(['taskset','-c',cpu,*cmd(label),score],log,cwd=octane)
            contents=log.read_text();m=re.search(r'BOX2D_SCORE ([0-9.]+)',contents);assert m and 'BOX2D_DONE' in contents
            item={'score':float(m[1]),'seconds':time.monotonic()-started,'log':log.name,'round':repetition}
            data['scores'].setdefault(label,[]).append(item);save()
            print('BOX2D_SCORE_RESULT',arch,label,json.dumps(item),flush=True)
    if os.environ.get('BENCHMARK_PROFILE')=='true':
        for repetition in range(3):
            for label in order(repetition):
                output=report/f'{label}-perf-{repetition}.csv';log=report/f'{label}-fixed-{repetition}.log'
                run(['perf','stat','-x',',','-o',output,'-e','cycles,instructions,branches,branch-misses','--','taskset','-c',cpu,*cmd(label),fixed],log,cwd=octane)
                assert 'BOX2D_FIXED_DONE 61' in log.read_text()
                data['perf'].setdefault(label,[]).append({'counters':output.name,'log':log.name,'round':repetition});save()
        if arch=='arm64':
            for label in labels:
                profile=report/(label+'-perf.data');log=report/(label+'-record.log')
                run(['perf','record','-e','cycles','-F','199','-o',profile,'--','taskset','-c',cpu,*cmd(label),fixed],log,cwd=octane)
                run(['perf','report','--stdio','--no-children','-i',profile],report/(label+'-perf-report.txt'))
                run(['perf','annotate','--stdio','-i',profile,'--symbol','Escargot::Interpreter::interpret(Escargot::ExecutionState*, Escargot::ByteCodeBlock*, unsigned long, Escargot::Value*)'],report/(label+'-perf-annotate.txt'))
    data['validated']=True;save();print('BOX2D_LAYOUT_DONE',arch,flush=True)
if __name__=='__main__':main()
