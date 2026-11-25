#!/usr/bin/env python3
"""Replay archived ARM64 binaries and cross the large allocation and sampling options."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import urllib.request
import zipfile

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("residency",HERE/"arm64-codeload-residency.py")
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
common,store=m.common,m.store

# Archive digest from the public upstream workflow. Verify the individual
# binaries too, before executing either file.
FIXTURES={
  "baseline": (14975975304, "b0d403f6c658bd6474ed56f802fe1ee1a69f89aa4ef9847361506f8704e4c4f6",
               {"escargot-arm64.stripped":"c9ecd0c66eb92e5b67d6cf6d03da627e4444348cd520cb89f57ec7808e75bf3f"}),
  "repeat": (14981449491,"24b393dc19da9237a9f33dcf39e15737fce3b1d855e00a6317ef0306c98e34d2",
             {"escargot-arm64.stripped":"530c17ea8b05ed61d59c248c105dd9b35b89b0b7ccf0d106b50c929692277e60",
              "store-policy-arm64/control-engine":"44d68707889c7542e6ecbcbad8ead75665c1f3f6a5024e54cc7516cfad4d9baa"})}
OFFSETS={"eval":0x22bcb0,"mark":0x3a1980,"grow":0x3a6720}


def fetch(work, key):
    identity, digest, files=FIXTURES[key]
    archive=work/(key+".zip")
    errors=[]
    for url in [f"https://api.github.com/repos/Samsung/escargot/actions/artifacts/{identity}/zip",
                f"https://nightly.link/Samsung/escargot/actions/artifacts/{identity}.zip"]:
        try:
            with urllib.request.urlopen(url,timeout=60) as source, archive.open('wb') as dest:
                shutil.copyfileobj(source,dest)
            if common.sha256(archive)!=digest:
                raise ValueError('incorrect archive digest')
            break
        except (OSError,ValueError) as error:
            errors.append(str(error))
    else:
        raise RuntimeError(errors)
    outputs={}
    with zipfile.ZipFile(archive) as handle:
        for name,expected in files.items():
            target=work/(key+"-"+Path(name).name)
            with handle.open(name) as source,target.open('wb') as dest:
                shutil.copyfileobj(source,dest)
            assert common.sha256(target)==expected
            target.chmod(0o755)
            outputs[Path(name).name]=target
    return outputs


def replay(work, report, binary, driver, octane, cpu, monitor):
    common.parse_audit=store.audit
    envs=store.policy.environments()
    rows=[]
    def run(label,target,mode,gen,env):
        samples,errors=[],[]
        done=threading.Event()
        thread=threading.Thread(target=m.sample,args=(target,monitor,done,samples,errors,threading.get_native_id()))
        thread.start()
        try:
            row,log=common.process_measure(label,mode,[str(target)]+(["--gc-generational"] if gen else []),
                                         octane,driver,env,report,cpu,monitor,False)
        finally:
            done.set()
            thread.join()
        store.record("codeload",row,log)
        row.update(sample_count=len(samples),sample_errors=errors)
        common.write_json(report/(label+"-smaps.json"),samples)
        rows.append(row)
        common.write_json(report/"measurements.json",rows)
        return row
    def environment(mode,cohort,defer):
        return dict(envs["full" if mode=="full" else "refined"],GC_DIAG_COHORT=cohort,
                    GC_BENCH_NUMERIC_RO=str(store.MODES[mode]),GC_BENCH_MANUAL_CODEBLOCK="1",
                    GC_GENERATIONAL_LARGE_DEFER=defer)
    backup=work/'release'
    binary.rename(backup)
    try:
        for key in ("baseline","repeat"):
            inputs=fetch(work,key)
            shutil.copy2(inputs["escargot-arm64.stripped"],binary)
            controls=[('full','7','1'),('off','7','1'),('off','7','0'),('off','1','1'),('off','0','1')]
            for block,order in enumerate([controls,controls[::-1],controls],1):
                for mode,cohort,defer in order:
                    run(f"{key}-{block}-{mode}-{cohort}-{defer}",binary,mode,mode=="off",environment(mode,cohort,defer))
            probes=m.module("probes","codeload-uprobes.py").Probes(binary,cpu)
            diag=report/(key+"-uprobe-status.txt")
            try:
                try:
                    probes.enable(OFFSETS,diag)
                except (OSError,subprocess.SubprocessError,ValueError) as exc:
                    with diag.open('a') as f:print(repr(exc),file=f)
                else:
                    for block,order in enumerate([controls[:3],controls[:3][::-1]],1):
                        for mode,cohort,defer in order:
                            label=f"probe-{key}-{block}-{mode}-{defer}"
                            probes.start()
                            try:
                                run(label,binary,mode,mode=="off",environment(mode,cohort,defer))
                            finally:
                                probes.stop(report/(label+".trace"))
            finally:
                probes.close()
            if not list(report.glob(f"probe-{key}-*.trace")) and shutil.which("gdb"):
                for mode in ("full","off"):
                    label=f"debugger-{key}-{mode}"
                    script=work/(label+".gdb")
                    gdb_code=(HERE/"codeload-gdb.py").read_text().replace("__OUTPUT__",repr(str(report/(label+".json"))))
                    script.write_text("set pagination off\nset confirm off\nset startup-with-shell off\nset disable-randomization off\nstarti\npython\n"+gdb_code+"\nend\ncontinue\n")
                    env=environment(mode,"7","1")
                    cmd=["taskset","-c",str(cpu),"gdb","-q","--batch","-nx","-x",str(script),"--args",str(binary)]
                    if mode=="off":cmd.append("--gc-generational")
                    with (report/(label+".log")).open("w") as log:
                        try:
                            subprocess.run([*cmd,str(driver)],env=env,cwd=octane,stdout=log,stderr=subprocess.STDOUT,timeout=90)
                        except subprocess.TimeoutExpired:
                            print("debugger diagnostic timed out",file=log)
            if key=="repeat":
                target=report/"control-engine"
                shutil.copy2(inputs["control-engine"],target)
                for block in (1,2):
                    cells=[("00","7","1"),("01","7","1"),("10","7","1"),("11","7","1"),
                           ("10","1","1"),("11","1","1"),("10","7","0"),("11","7","0")]
                    for cell,cohort,defer in (cells if block==1 else reversed(cells)):
                        mode="off" if cell[0]=="1" else "full"
                        env=environment("off",cohort,defer)
                        env["CODELOAD_CONTROL"]=cell
                        run(f"cross-{block}-{cell}-{cohort}-{defer}",target,mode,True,env)
    finally:
        os.replace(backup,binary)


def main():
    if os.environ["BENCHMARK_ARCHITECTURE"]=="arm32":return
    report=Path(os.environ["BENCHMARK_OUTPUT_DIR"])/"store-policy-arm64"
    report.mkdir()
    octane,driver=store.workloads(Path(os.environ["BENCHMARK_HARNESS"]),report)["codeload"]
    assert common.sha256(driver)==m.phases.EXPECTED_DRIVER
    cpu=int(os.environ["BENCHMARK_CPU"])
    monitor=next(c for c in sorted(os.sched_getaffinity(0)) if c!=cpu)
    binary=Path.cwd().parent/"build-arm64/escargot"
    with tempfile.TemporaryDirectory(dir=binary.parent) as folder:
        replay(Path(folder),report,binary,driver,octane,cpu,monitor)


if __name__=="__main__":
    main()
