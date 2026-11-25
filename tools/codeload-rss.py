"""Sample resident pages by mapping while the stock CodeLoad process runs."""
import json
import os
from pathlib import Path
import re
import subprocess
import time

MAP = re.compile(r"^([0-9a-f]+)-([0-9a-f]+) ([-rwxps]+) \S+ \S+ \d+\s*(.*)$")
CAGE_SIZE = 1 << 32


def snapshot(pid):
    regions = []
    for line in Path(f"/proc/{pid}/smaps").read_text().splitlines():
        match = MAP.match(line)
        if match:
            regions.append([int(match[1], 16), int(match[2], 16), match[3], match[4], 0])
        elif line.startswith("Rss:"):
            regions[-1][-1] = int(line.split()[1])
    # The unused tail of the compressed-pointer reservation ends on the next
    # 4-GiB boundary. All CodeLoad heaps are far smaller than half the cage.
    if any(left[1] > right[0] for left, right in zip(regions, regions[1:])):
        return None
    cages = {end - CAGE_SIZE for start, end, perm, _, _ in regions
             if perm.startswith("---") and end % CAGE_SIZE == 0
             and CAGE_SIZE // 2 < end - start <= CAGE_SIZE}
    if len(cages) != 1:
        return None
    base, = cages
    totals = dict(cage=0, brk=0, anon=0, file=0, other=0)
    for start, end, _, name, rss in regions:
        if base <= start and end <= base + CAGE_SIZE:
            group = "cage"
        elif name == "[heap]":
            group = "brk"
        elif not name or name.startswith("[anon"):
            group = "anon"
        elif name.startswith("/"):
            group = "file"
        else:
            group = "other"
        totals[group] += rss
    return dict(kib=totals, cage_base=base, total=sum(totals.values()))


def observe(binary, gen, cwd, driver, env, cpu, report, label):
    log = report / (label + ".log")
    start = time.monotonic()
    events = []
    with log.open("wb") as stream:
        process = subprocess.Popen(["taskset", "-c", str(cpu), str(binary),
                                    *(["--gc-generational"] if gen else []), str(driver)],
                                   env=env, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT)
        try:
            while process.poll() is None:
                try:
                    sample = snapshot(process.pid)
                except (FileNotFoundError, ProcessLookupError):
                    break
                if sample:
                    sample["elapsed"] = time.monotonic() - start
                    events.append(sample)
                if time.monotonic() - start > 120:
                    raise TimeoutError(label)
                time.sleep(.009)
            if process.wait(timeout=10):
                raise RuntimeError(label)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    if not events or not re.search(r"^Score \(version 9\): [\d.]+$", log.read_text(), re.M):
        raise ValueError(label + " missing samples or workload score")
    (report / (label + ".json")).write_text(json.dumps(events) + "\n")
    return max(events, key=lambda sample: sample["total"])
