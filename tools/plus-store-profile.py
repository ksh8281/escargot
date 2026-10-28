#!/usr/bin/env python3
"""Isolate heap-store code generation on the development runner."""
import csv,json,os,shutil,statistics,subprocess
from pathlib import Path

def run(args,log,cwd=None):
 print('+',' '.join(map(str,args)),flush=True)
 with log.open('w') as f:
  subprocess.run(list(map(str,args)),cwd=cwd,stdout=f,stderr=subprocess.STDOUT,check=True)

def main():
 if os.environ['BENCHMARK_ARCHITECTURE']!='arm32':
  print('ARM32 diagnosis complete; skipping ARM64 profiling',flush=True);return
 source=Path.cwd();work=source.parent; report=Path(os.environ['BENCHMARK_OUTPUT_DIR'])/'store-profile-arm32';report.mkdir()
 header=source/'src/runtime/EncodedValue.h'; saved=header.read_bytes(); engine=work/'build-arm32/escargot'; harness=Path(os.environ['BENCHMARK_HARNESS'])/'test/octane'
 container=f"escargot-arm-benchmark-{os.environ['GITHUB_RUN_ID']}-{os.environ['GITHUB_RUN_ATTEMPT']}"
 info=json.loads(subprocess.check_output(['docker','inspect',container]))[0]
 assert any(m['Source']==str(work) and m['Destination']=='/work' for m in info['Mounts'])
 variants=['tag','flag','initial']
 try:
  shutil.copy2(engine,report/'tag')
  for label,revision in [('flag','7ca2e66a0a4007803c701e9fec5ca3cfb9bf3c9c'),('initial','0d3b48fc517591458e418b2445cc59c080b93874')]:
   run(['git','fetch','--depth','1','origin',revision],report/(label+'-fetch.log'))
   content=subprocess.check_output(['git','show',revision+':src/runtime/EncodedValue.h'])
   header.write_bytes(content);(report/(label+'-EncodedValue.h')).write_bytes(content)
   run(['docker','start','-a',container],report/(label+'-build.log'))
   shutil.copy2(engine,report/label)
 finally:header.write_bytes(saved)
 (report/'tag-EncodedValue.h').write_bytes(saved)
 (report/'regexp.js').write_text('load("base.js");load("regexp.js");BenchmarkSuite.ResetRNG();RegExpSetup();for(var i=0;i<8;i++)RegExpRun();print("REGEXP_DONE");\n')
 (report/'earley.js').write_text('load("base.js");load("earley-boyer.js");BenchmarkSuite.ResetRNG();for(var i=0;i<30;i++){BgL_earleyzd2benchmarkzd2();BgL_nboyerzd2benchmarkzd2();}print("EARLEY_DONE");\n')
 def command(label,suite):return ['taskset','-c',os.environ['BENCHMARK_CPU'],os.environ['BENCHMARK_RUNTIME_LOADER'],'--library-path',os.environ['BENCHMARK_RUNTIME_DIRECTORY'],str(report/label),str(report/(suite+'.js'))]
 results={}
 for suite in ['regexp','earley']:
  for iteration in range(3):
   for label in variants[iteration:]+variants[:iteration]:
    name=f'{suite}-{label}-{iteration}'
    run(['perf','stat','-x',';','-o',report/(name+'.csv'),'-e','cycles:u,instructions:u,branches:u,branch-misses:u,task-clock,context-switches','--',*command(label,suite)],report/(name+'.log'),harness)
    assert suite.upper()+'_DONE' in (report/(name+'.log')).read_text()
    print(name,'done',flush=True)
  results[suite]={}
  for label in variants:
   rows=[{r[2]:float(r[0]) for r in csv.reader(open(report/f'{suite}-{label}-{i}.csv'),delimiter=';') if len(r)>3 and not r[0].startswith('<')} for i in range(3)]
   results[suite][label]={'samples':rows,'medians':{k:statistics.median(x[k] for x in rows) for k in rows[0]}}
   run(['perf','record','-q','-e','cycles:u','-F','499','-g','--call-graph','fp','-o',report/f'{suite}-{label}.data','--',*command(label,suite)],report/f'{suite}-{label}-record.log',harness)
   run(['perf','report','--stdio','--no-children','--call-graph','none','--percent-limit','0.5','-i',report/f'{suite}-{label}.data'],report/f'{suite}-{label}-report.txt')
   run(['perf','annotate','--stdio','--no-source','-i',report/f'{suite}-{label}.data'],report/f'{suite}-{label}-annotate.txt')
  (report/'results.json').write_text(json.dumps(results,indent=2)+'\n')
  print(suite,json.dumps({v:results[suite][v]['medians'] for v in variants}),flush=True)
if __name__=='__main__':main()
