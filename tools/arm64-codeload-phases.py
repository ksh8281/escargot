#!/usr/bin/env python3
"""Locate heap growth and the RSS high-water mark within deterministic CodeLoad."""
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("store", HERE / "arm-gc-store-policy.py")
store = importlib.util.module_from_spec(spec)
spec.loader.exec_module(store)
EXPECTED_DRIVER = "8985e59e9ccd6d18ea5bd47abf257ae105b860bdfda6e57738a4a8bdd599cdca"
MARKERS = """
CodeLoad.benchmarks.forEach(function(bench) {
  var original = bench.run;
  var iteration = 0;
  bench.run = function() {
    if (iteration === 0 || iteration % (bench.name === 'CodeLoadClosure' ? 100 : 10) === 0)
      print('ITER ' + bench.name + ' ' + iteration);
    original();
    iteration++;
    if (iteration === bench.deterministicIterations)
      print('DONE ' + bench.name + ' ' + iteration);
  };
});
"""


def checkpoints(log):
    full = minor = capacity = 0
    events = []
    for line in log.splitlines():
        if line.startswith("GENERATIONAL_AUDIT "):
            break
        if line.startswith("GC_CYCLE_RECLAIM "):
            cycle = json.loads(line.split(" ", 1)[1])
            full += cycle["full"]
            minor += 1 - cycle["full"]
        elif match := re.search(r"Grow heap to (\d+) KiB", line):
            capacity = int(match[1])
        elif match := re.fullmatch(r"PHASE (.+) rss_kib=(\d+) hwm_kib=(\d+)", line):
            events.append(dict(phase=match[1], full=full, minor=minor,
                               heap_capacity_kib=capacity, rss_kib=int(match[2]),
                               hwm_kib=int(match[3])))
    return events


def measure(harness, binary, report, cpu):
    common = store.common
    octane, driver = store.workloads(harness, report)["codeload"]
    assert common.sha256(driver) == EXPECTED_DRIVER
    chunks = report / "codeload-chunks.js"
    src = driver.read_text()
    assert src.count("BenchmarkSuite.RunSuites(") == 1
    chunks.write_text(src.replace("BenchmarkSuite.RunSuites(", MARKERS + "\nBenchmarkSuite.RunSuites("))
    probe = report / "phase-probe.so"
    subprocess.run(["cc", "-Wall", "-Wextra", "-Werror", "-O2", "-fPIC", "-shared",
                    HERE / "codeload-phase-probe.c", "-ldl", "-o", probe], check=True)
    affinity = sorted(os.sched_getaffinity(0))
    monitor = next(c for c in affinity if c != cpu)
    common.parse_audit = store.audit
    envs = store.policy.environments()
    data = dict(binary_sha256=common.sha256(binary), driver_sha256=common.sha256(driver),
                chunks_sha256=common.sha256(chunks), probe_sha256=common.sha256(probe),
                octane_inputs={p.name: common.sha256(p) for p in sorted(octane.glob("*.js"))},
                cpu=cpu, rows=[])
    # Repeat the original four-way comparison first. The two observers then
    # narrow the interval, with a sample-only cohort control for late activation.
    for stage in ("plain", "trace", "chunks"):
        for block, order in enumerate(store.GUARD_ORDERS, 1):
            if stage != "plain":
                order = [m for m in order if m in ("full", "off")]
                if stage == "chunks":
                    order.insert(block % 3, "sample-only")
            for policy in order:
                mode = "off" if policy == "sample-only" else policy
                env = dict(envs["full" if mode == "full" else "refined"],
                           GC_DIAG_COHORT="1" if policy == "sample-only" else "7",
                           GC_BENCH_NUMERIC_RO=str(store.MODES[mode]), GC_BENCH_MANUAL_CODEBLOCK="1")
                command = [str(binary)]
                if mode != "full":
                    command.append("--gc-generational")
                if stage != "plain":
                    env.update(ESCARGOT_GC_CYCLE_TRACE="1", GC_PRINT_STATS="1")
                    command = ["env", "LD_PRELOAD=" + str(probe), *command]
                label = f"{stage}-{block}-{policy}"
                script = chunks if stage == "chunks" else driver
                row, contents = common.process_measure(label, mode, command, octane, script,
                                                       env, report, cpu, monitor, False)
                store.record("codeload", row, contents)
                if stage != "plain":
                    row["checkpoints"] = checkpoints(contents)
                    names = [event["phase"] for event in row["checkpoints"]]
                    assert "begin suites" in names and names[-1] == "end codeload"
                    assert row["checkpoints"][-1]["heap_capacity_kib"] > 0
                    if stage == "chunks":
                        assert "step DONE CodeLoadClosure 1600" in names and "step DONE CodeLoadJQuery 100" in names
                if policy == "sample-only":
                    assert store.json_line(contents, "ALLOCATION_PATTERN_AUDIT")["activations"] == 0
                row.update(stage=stage, policy=policy, block=block, status="passed")
                data["rows"].append(row)
                common.write_json(report / "measurements.json", data)
                print(label, row["peak_rss_kib"], row["mode_audit"]["gc"], flush=True)
    assert len(data["rows"]) == 27


def counterfactuals(binary, harness, report, cpu):
    common = store.common
    envs = store.policy.environments()
    monitor = next(c for c in sorted(os.sched_getaffinity(0)) if c != cpu)
    octane = harness / "test/octane"
    driver = report / "codeload-fixed.js"
    assert common.sha256(driver) == EXPECTED_DRIVER
    build = binary.parent
    flags = ["GC_enable_generational", "GC_configure_allocation_pattern", "GC_allocation_pattern_mode"]
    obj = report / "controls.o"
    subtree = HERE.parent / "third_party/GCutil"
    subprocess.run(["cc", "-O2", "-fPIC", "-I" + str(subtree / "include"),
                    "-c", HERE / "codeload-controls.c", "-o", obj], check=True)
    line = common.output(["ninja", "-t", "commands", "escargot"], build).splitlines()[-1]
    link = shlex.split(line.split(" && ")[1])
    link = [arg for arg in link if not arg.startswith("-Wl,--dependency-file=")]
    control = report / "control-engine"
    link[link.index("-o") + 1] = str(control)
    link[1:1] = [str(obj)] + ["-Wl,--wrap=" + name for name in flags]
    subprocess.run(link, cwd=build, check=True)
    rows = []
    for block, order in enumerate([("00", "10", "11", "01"), ("11", "01", "00", "10"),
                                    ("01", "11", "10", "00")], 1):
        for cell in order:
            gc = cell[0] == "1"
            env = dict(envs["refined"], GC_DIAG_COHORT="7", GC_BENCH_NUMERIC_RO="0",
                       GC_BENCH_MANUAL_CODEBLOCK="1", CODELOAD_CONTROL=cell)
            label = f"cross-{block}-{cell}"
            row, log = common.process_measure(label, "off" if gc else "full",
                [str(control), "--gc-generational"], octane, driver, env, report, cpu, monitor, False)
            store.record("codeload", row, log)
            pattern = store.json_line(log, "ALLOCATION_PATTERN_AUDIT")
            assert pattern["mode"] == (4 if gc else 0) and pattern["hints"] == 0
            row.update(cell=cell, block=block)
            rows.append(row)
            common.write_json(report / "controls.json", rows)
    spec = importlib.util.spec_from_file_location("resident", HERE / "codeload-rss.py")
    resident = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(resident)
    summaries = {}
    previous = os.sched_getaffinity(0)
    try:
        os.sched_setaffinity(0, {monitor})
        for block, order in enumerate([("full", "off"), ("off", "full"), ("full", "off")], 1):
            for mode in order:
                gen = mode == "off"
                env = dict(envs["refined" if gen else "full"], GC_DIAG_COHORT="7",
                           GC_BENCH_NUMERIC_RO=str(store.MODES[mode]), GC_BENCH_MANUAL_CODEBLOCK="1")
                label = f"mappings-{block}-{mode}"
                summaries[label] = resident.observe(binary, gen, octane, driver, env, cpu, report, label)
                common.write_json(report / "mapping-peaks.json", summaries)
    finally:
        os.sched_setaffinity(0, previous)


def main():
    if os.environ["BENCHMARK_ARCHITECTURE"] == "arm32":
        return
    assert os.environ["BENCHMARK_ARCHITECTURE"] == "arm64" and platform.machine() == "aarch64"
    assert int(os.environ["BENCHMARK_REPETITIONS"]) == 3
    report = Path(os.environ["BENCHMARK_OUTPUT_DIR"]) / "store-policy-arm64"
    report.mkdir()
    harness = Path(os.environ["BENCHMARK_HARNESS"])
    binary = Path.cwd().parent / "build-arm64/escargot"
    cpu = int(os.environ["BENCHMARK_CPU"])
    measure(harness, binary, report, cpu)
    counterfactuals(binary, harness, report, cpu)


if __name__ == "__main__":
    main()
