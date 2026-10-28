#!/usr/bin/env python3
import csv,hashlib,importlib.util,json,os,re,shutil,statistics,subprocess,sys
from pathlib import Path
PARENT='f4582b94d0ee7a9a3b21aeb08ca406b20209fef8'
FILES=['src/codecache/CodeCache.cpp','src/interpreter/ByteCode.h','src/interpreter/ByteCodeGenerator.cpp','src/interpreter/ByteCodeInterpreter.cpp','src/runtime/EncodedValue.h']
WTB=set('acorn babel babel-minify babylon buble chai coffeescript espree esprima jshint lebab postcss prepack prettier source-map terser typescript uglify-js'.split())
OCT=set('Richards DeltaBlue Crypto RayTrace EarleyBoyer RegExp Splay SplayLatency NavierStokes PdfJS Mandreel MandreelLatency Gameboy CodeLoad Box2D zlib Typescript'.split())
def run(args,log,cwd=None,check=True):
 print('+',' '.join(map(str,args)),flush=True)
 with log.open('w') as f:return subprocess.run(list(map(str,args)),cwd=cwd,stdout=f,stderr=subprocess.STDOUT,check=check)
def main():
 source=Path.cwd();work=source.parent;arch=os.environ['BENCHMARK_ARCHITECTURE'];harness=Path(os.environ['BENCHMARK_HARNESS']);report=Path(os.environ['BENCHMARK_OUTPUT_DIR'])/('rootcause-'+arch);report.mkdir()
 engine=work/('build-'+arch)/'escargot';shutil.copy2(engine,report/'pr'); saved={f:(source/f).read_bytes() for f in FILES}
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
 data={'architecture':arch,'parent_revision':PARENT,'pr_revision':'06aeed8f1c190e392ca5a9d4198e49d39759009b','protocol':'same-base paired diagnosis; original baseline not rerun; full standard suites once and fixed-work profiles three times','scores':{},'fixed':{},'binaries':{}}
 for label in ['parent','pr']:
  data['binaries'][label]={'sha256':hashlib.sha256((report/label).read_bytes()).hexdigest()}
  run(['size','-A',report/label],report/(label+'-size.txt'));run(['nm','-S','-C',report/label],report/(label+'-symbols.txt'))
 sys.path.insert(0,str(harness/'tools'));spec=importlib.util.spec_from_file_location('monthly',harness/'tools/monthly-engine-benchmark.py');monthly=importlib.util.module_from_spec(spec);spec.loader.exec_module(monthly);monthly.REPORT=report;monthly.ROOT=harness
 for label in (['parent','pr'] if arch=='arm32' else ['pr','parent']):
  data['scores'][label]={}
  for suite,directory,script,key,expected in [('octane',harness/'test/octane','run.js','subtests_scores',OCT),('wtb',harness/'test/web-tooling-benchmark','dist/cli.js','subtests_runs_per_second',WTB)]:
   result=monthly.measure(label+'-'+suite,cmd(label),directory,os.environ,script=script,kind='octane' if suite=='octane' else 'web_tooling');assert set(result[key])==expected
   data['scores'][label][suite]=result;(report/'results.json').write_text(json.dumps(data,indent=2)+'\n')
 checkpoint="function benchmarkCheckpoint(name){print('PHASE '+JSON.stringify({name:name,time:Date.now()/1000}));}\n"
 (report/'wtb-fixed.js').write_text(checkpoint+"load('dist/memory.js');benchmarkCheckpoint('done');\n")
 loads=re.findall(r"load\(base_dir \+ '([^']+)'\);",(harness/'test/octane/run.js').read_text());assert len(loads)==20
 fixed=checkpoint+'\n'.join('load('+json.dumps(f)+');' for f in loads)+'''\nBenchmarkSuite.ResetRNG();benchmarkCheckpoint('loaded');
var runs={RegExp:24,Earley:60,Boyer:60,Box2D:60,NavierStokes:30};var counts={};
for(var si=0;si<BenchmarkSuite.suites.length;si++){var suite=BenchmarkSuite.suites[si];for(var bi=0;bi<suite.benchmarks.length;bi++){var b=suite.benchmarks[bi];b.Setup();var count=runs[b.name]||1;for(var it=0;it<count;it++)b.run();b.TearDown();counts[b.name]=count;benchmarkCheckpoint('test:'+b.name);}}
print('MEM_WORK_JSON '+JSON.stringify(counts));benchmarkCheckpoint('done');\n'''
 (report/'octane-fixed.js').write_text(fixed)
 for suite,directory in [('octane',harness/'test/octane'),('wtb',harness/'test/web-tooling-benchmark')]:
  data['fixed'][suite]={}
  for iteration in range(3):
   for label in (['parent','pr'] if iteration%2==0 else ['pr','parent']):
    name=f'{suite}-{label}-{iteration}';command=['taskset','-c',os.environ['BENCHMARK_CPU'],*cmd(label),str(report/(suite+'-fixed.js'))]
    run(['perf','stat','-x',';','-o',report/(name+'.csv'),'-e','cycles:u,instructions:u,branches:u,branch-misses:u,task-clock,context-switches','--',*command],report/(name+'.log'),directory)
    contents=(report/(name+'.log')).read_text();assert 'MEM_WORK_JSON ' in contents
    counts=json.loads(re.search(r'MEM_WORK_JSON (.*)',contents)[1]);assert len(counts)==18
    stats={r[2]:float(r[0]) for r in csv.reader(open(report/(name+'.csv')),delimiter=';') if len(r)>3 and not r[0].startswith('<')}
    phases=[json.loads(x[6:]) for x in contents.splitlines() if x.startswith('PHASE ')]
    samples=data['fixed'][suite].setdefault(label,[]);samples.append({'stats':stats,'phases':phases,'counts':counts});(report/'results.json').write_text(json.dumps(data,indent=2)+'\n')
  for label in ['parent','pr']:
   name=f'{suite}-{label}';command=['taskset','-c',os.environ['BENCHMARK_CPU'],*cmd(label),str(report/(suite+'-fixed.js'))]
   run(['perf','record','-q','-k','CLOCK_REALTIME','-e','cycles:u','-F','999','-g','--call-graph','fp','-o',report/(name+'.data'),'--',*command],report/(name+'-record.log'),directory)
   run(['perf','report','--stdio','--no-children','--call-graph','none','--percent-limit','0.5','-i',report/(name+'.data')],report/(name+'-report.txt'))
   phases=[json.loads(x[6:]) for x in (report/(name+'-record.log')).read_text().splitlines() if x.startswith('PHASE ')]
   for prev,phase in zip(phases,phases[1:]):
    target=phase['name'].removeprefix('test:')
    if target not in ['RegExp','Box2D','NavierStokes','terser','source-map','uglify-js']:continue
    span=str(prev['time'])+','+str(phase['time'])
    run(['perf','report','--stdio','--no-children','--call-graph','none','--percent-limit','0.5','--time',span,'-i',report/(name+'.data')],report/(name+'-'+target+'-report.txt'),check=False)
   run(['perf','annotate','--stdio','--no-source','-i',report/(name+'.data')],report/(name+'-annotate.txt'),check=False)
 data['validated']=True;(report/'results.json').write_text(json.dumps(data,indent=2)+'\n');print('ROOTCAUSE_DONE',arch,flush=True)
if __name__=='__main__':main()
