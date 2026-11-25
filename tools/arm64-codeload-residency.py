#!/usr/bin/env python3
"""Measure the original process at its RSS peak and isolate the compiler walk."""
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import threading
import time

HERE = Path(__file__).resolve().parent


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, HERE / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


phases = module("phases", "arm64-codeload-phases.py")
resident = module("resident", "codeload-rss.py")
store = phases.store
common = store.common


def sample(binary, monitor, done, events, failures, main_tid):
    os.sched_setaffinity(0, {monitor})
    pid = None
    while not done.is_set():
        try:
            if pid is None:
                parents = Path(f"/proc/self/task/{main_tid}/children").read_text().split()
                for parent in parents:
                    for child in Path(f"/proc/{parent}/task/{parent}/children").read_text().split():
                        if os.path.realpath(f"/proc/{child}/exe") == str(binary):
                            pid = int(child)
            if pid:
                before = time.monotonic()
                entry = resident.snapshot(pid)
                status = Path(f"/proc/{pid}/status").read_text()
                hwm = re.search(r"^VmHWM:\s+(\d+)", status, re.M)
                if entry and hwm:
                    entry.update(time=before, end=time.monotonic(), hwm=int(hwm[1]))
                    events.append(entry)
        except (FileNotFoundError, ProcessLookupError):
            pass
        except Exception as exc:
            failures.append(repr(exc))
            break
        done.wait(.006 if pid else .002)


def relink(binary, work):
    build = binary.parent
    commands = json.loads((build / "compile_commands.json").read_text())
    objects = []
    eval_line = "Value GlobalObject::eval(ExecutionState& state, const Value& arg)\n{\n    if (arg.isString()) {"
    walk_line = "        findRefinedAllocationSites(m_ast, m_sites);"
    grow_line = "  GC_add_to_heap(space, sz);"
    finish_line = "  GC_start_reclaim(FALSE);"
    patches = {
        "GlobalObject.cpp": [(eval_line, 'extern "C" void codeload_eval();\n' + eval_line + '\n        codeload_eval();')],
        "ByteCodeGenerator.cpp": [
            ("bool RefinedAllocationAnalysis::contains", 'extern "C" int codeload_walk();\nextern "C" void codeload_walk_end();\nbool RefinedAllocationAnalysis::contains'),
            (walk_line, "        if (codeload_walk()) {\n" + walk_line + "\n            codeload_walk_end();\n        }")],
        "alloc.c": [(grow_line, grow_line + '\n  { extern void codeload_event(int,unsigned long,unsigned long,unsigned long,unsigned long,unsigned long,unsigned long,unsigned long); codeload_event(1,GC_gc_no,GC_heapsize,GC_unmapped_bytes,GC_large_free_bytes,GC_bytes_allocd,GC_dont_gc,sz); }'),
                    (finish_line, finish_line + '\n  { extern void codeload_event(int,unsigned long,unsigned long,unsigned long,unsigned long,unsigned long,unsigned long,unsigned long); codeload_event(2,GC_gc_no,GC_heapsize,GC_unmapped_bytes,GC_large_free_bytes,GC_bytes_allocd,GC_dont_gc,GC_atomic_in_use+GC_composite_in_use); }')]
    }
    for filename, substitutions in patches.items():
        entry, = [e for e in commands if Path(e["file"]).name == filename]
        path = Path(entry["file"])
        source = path.read_text()
        for old, new in substitutions:
            assert source.count(old) == 1, old
            source = source.replace(old, new)
        edited = work / filename
        edited.write_text(source)
        cmd = shlex.split(entry["command"])
        obj = work / (filename + ".o")
        cmd[cmd.index("-o") + 1] = str(obj)
        cmd[cmd.index(str(path))] = str(edited)
        cmd[1:1] = ["-iquote", str(path.parent)]
        subprocess.run(cmd, cwd=build, check=True)
        objects.append(str(obj))
    obj = work / "observe.o"
    subprocess.run(["cc", "-O2", "-I" + str(HERE.parent / "third_party/GCutil/include"),
                    "-c", HERE / "codeload-events.c", "-o", obj], check=True)
    line = common.output(["ninja", "-t", "commands", "escargot"], build).splitlines()[-1]
    link = [x for x in shlex.split(line.split(" && ")[1]) if not x.startswith("-Wl,--dependency-file=")]
    link[1:1] = [*objects, str(obj), "-Wl,--wrap=GC_allocation_pattern_mode",
                 "-Wl,--wrap=GC_enable_generational", "-Wl,--wrap=GC_configure_allocation_pattern"]
    target = work / "control-engine"
    link[link.index("-o") + 1] = str(target)
    subprocess.run(link, cwd=build, check=True)
    return target


def main():
    if os.environ["BENCHMARK_ARCHITECTURE"] == "arm32":
        return
    binary = Path.cwd().parent / "build-arm64/escargot"
    report = Path(os.environ["BENCHMARK_OUTPUT_DIR"]) / "store-policy-arm64"
    report.mkdir()
    octane, driver = store.workloads(Path(os.environ["BENCHMARK_HARNESS"]), report)["codeload"]
    assert common.sha256(driver) == phases.EXPECTED_DRIVER
    cpu = int(os.environ["BENCHMARK_CPU"])
    monitor = next(c for c in sorted(os.sched_getaffinity(0)) if c != cpu)
    common.parse_audit = store.audit
    environments = store.policy.environments()
    rows = []
    def run(label, target, mode, env, flag, observe=False):
        events, errors = [], []
        done = threading.Event()
        if observe:
            worker = threading.Thread(target=sample, args=(target, monitor, done, events, errors, threading.get_native_id()))
            worker.start()
        try:
            row, log = common.process_measure(label, mode, [str(target)] + (["--gc-generational"] if flag else []),
                octane, driver, env, report, cpu, monitor, False)
        finally:
            if observe:
                done.set()
                worker.join()
        store.record("codeload", row, log)
        if observe:
            row.update(sample_errors=errors, sample_count=len(events))
            if events:
                common.write_json(report / (label + "-smaps.json"), events)
        rows.append(row)
        common.write_json(report / "measurements.json", rows)
    def original(label, mode, extra=None):
        env = dict(environments["refined" if mode == "off" else "full"], GC_DIAG_COHORT="7",
                   GC_BENCH_NUMERIC_RO=str(store.MODES[mode]), GC_BENCH_MANUAL_CODEBLOCK="1")
        env.update(extra or {})
        run(label, binary, mode, env, mode == "off", True)
    for block, order in enumerate([("full","off"),("off","full"),("full","off")], 1):
        for mode in order:
            original(f"original-{block}-{mode}", mode)
    for padding in (0, 8, 16, 24, 40, 56, 72, 88):
        for mode in ("off","full"):
            original(f"padding-{padding}-{mode}", mode, {"BENCH_PADDING": "x" * padding})
    for block in range(1, 3):
        for cohort, defer in ([("7","0"),("7","1"),("1","1"),("0","1")] if block==1 else
                              [("0","1"),("1","1"),("7","1"),("7","0")]):
            original(f"policy-{block}-{cohort}-{defer}", "off", {"GC_DIAG_COHORT":cohort,
                                                                  "GC_GENERATIONAL_LARGE_DEFER":defer})
    probes = module("probes", "codeload-uprobes.py").Probes(binary,cpu)
    try:
        try:
            probes.enable()
        except (ValueError, OSError, subprocess.SubprocessError, StopIteration, KeyError) as exc:
            (report / "probe-status.txt").write_text(repr(exc) + "\n" + str(getattr(exc,"stderr","")))
        else:
            for block, order in enumerate([("full","off"),("off","full"),("full","off")],1):
                for mode in order:
                    label = f"probe-{block}-{mode}"
                    probes.start()
                    try:
                        original(label,mode)
                    finally:
                        probes.stop(report/(label+".trace"))
    finally:
        probes.close()
    with tempfile.TemporaryDirectory(dir=binary.parent) as folder:
        target = relink(binary, Path(folder))
        # Digits select collector, phase mask (load, Closure, jQuery), and full AST walk.
        cases = ["000","071","100","170","171","111","121","141","131","151","161"]
        for block, order in enumerate([cases, list(reversed(cases))],1):
            for cell in order:
                env = dict(environments["refined"], GC_DIAG_COHORT="7", GC_BENCH_NUMERIC_RO="0",
                           GC_BENCH_MANUAL_CODEBLOCK="1", CODELOAD_CONTROL=cell)
                run(f"split-{block}-{cell}", target, "off" if cell[0]=="1" else "full", env, True)


if __name__ == "__main__":
    main()
