#!/usr/bin/env python3
"""Compare and profile GC admission policies on native ARM."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import struct
import subprocess
import time

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('sideeffects', HERE / 'arm-gc-sideeffects-compare.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
r, p, c = m.r, m.p, m.c


def quiet_probe_source(original):
    replacements = {
        'STATIC MAY_THREAD_LOCAL unsigned GC_pattern_quiet_fulls;':
            'STATIC MAY_THREAD_LOCAL unsigned GC_pattern_quiet_fulls;\n'
            'STATIC MAY_THREAD_LOCAL unsigned GC_pattern_quiet_limit = 2;',
        '  GC_pattern_mode = mode;':
            '  GC_pattern_mode = mode;\n'
            '  GC_pattern_quiet_limit = GETENV("GC_DIAG_QUIET_FULLS") != NULL\n'
            '      && atoi(GETENV("GC_DIAG_QUIET_FULLS")) == 1 ? 1 : 2;',
        'GC_pattern_quiet_fulls < 2': 'GC_pattern_quiet_fulls < GC_pattern_quiet_limit',
        'GC_pattern_quiet_fulls >= 2': 'GC_pattern_quiet_fulls >= GC_pattern_quiet_limit',
    }
    for before, after in replacements.items():
        assert original.count(before) == 1, before
        original = original.replace(before, after)
    return original


def summarize(rows):
    # Reuse metric parsing/direction without calling either policy full GC.
    projected = [dict(x, mode='full' if x['quiet_fulls'] == 2 else 'refined') for x in rows]
    result = m.summarize(projected)
    for name, item in result.items():
        item['medians'] = {'current_two': item['medians']['full'],
                           'early_one': item['medians']['refined']}
        groups = {v: [x for x in rows if x['name'] == name and x['quiet_fulls'] == n]
                  for v, n in [('current_two', 2), ('early_one', 1)]}
        item['peak_rss_kib'] = {v: statistics.median(x['peak_rss_kib'] for x in g)
                                for v, g in groups.items()}
        item['rss_change_pct'] = (item['peak_rss_kib']['early_one']
                                 / item['peak_rss_kib']['current_two'] - 1) * 100
    return result


def profile_terser():
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = root / ('terser-profile-' + arch)
    report.mkdir()
    if arch == 'arm32':
        c.write_json(report / 'measurements.json', {
            'status': 'skipped', 'arch': arch,
            'reason': 'Repeated loss was observed on ARM64; no ARM32 timing repeated.'})
        return
    assert arch == 'arm64' and os.environ['BENCHMARK_PROFILE'] == 'true'
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    binary = source.parent / 'build-arm64/escargot'
    engine = report / 'measured-engine'
    shutil.copy2(binary, engine)
    header = engine.read_bytes()[:20]
    assert header[:5] == b'\x7fELF\x02' and struct.unpack('<H', header[18:20])[0] == 183
    r.prepare_drivers(harness, report)
    text = (report / 'wtb-score.js').read_text().replace(
        'count:b.count}', 'count:b.count,sample_seconds:b.stats.sample}')
    text = m.inactive_prefix_driver(text, 'terser', 16)
    text = text.replace('if(b.name==="terser")', 'if(b.name==="chai"||b.name==="terser")')
    needle = 'var completedPrefix=0;'
    assert text.count(needle) == 1
    text = text.replace(needle,
        "suite.forEach(function(b){b.on('start',function(){"
        "print('PROFILE_BEGIN '+b.name+' '+Date.now());});});\n"
        "suite.on('cycle',function(e){print('PROFILE_END '+e.target.name+' '+Date.now());});\n"
        + needle)
    driver = report / 'terser-prefix-profile.js'
    driver.write_text(text)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    c.parse_audit = m.audit
    data = {'status': 'running', 'arch': arch,
        'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
        'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
        'binary_sha256': c.sha256(engine), 'rows': [], 'failures': [],
        'protocol': 'ARM64 diagnostic only: full A, refined, identical full B. '
            'Original first sixteen WTB cases and all eighteen live roots, twenty samples '
            'for Chai/Terser and three for other cases, matching the reduced-sampling '
            'screen. Same production executable/default flags/FSD12/seed1. Perf cycles '
            'at 199 Hz with monotonic timestamps, GC cycle tracing, and line-buffered '
            'stdout/stderr for ordered stage markers. These instrumented scores are '
            'not acceptance measurements. No engine or policy source changes. ARM32 '
            'build occurs in the shared workflow but its timing is deliberately skipped.'}
    c.write_json(report / 'measurements.json', data)
    for control in ['full_a', 'refined', 'full_b']:
        mode = 'refined' if control == 'refined' else 'full'
        perf_data = report / (control + '.perf.data')
        command = ['perf', 'record', '-F', '199', '-e', 'cycles', '--clockid', 'mono',
                   '-o', str(perf_data), '--', 'stdbuf', '-oL', '-eL', str(engine)]
        if mode == 'refined':
            command.append('--gc-generational')
        offset = time.time() - time.monotonic()
        try:
            row, output = c.process_measure('profile-' + control, mode, command,
                harness / 'test/web-tooling-benchmark', driver,
                {**p.environments()[mode], 'ESCARGOT_GC_CYCLE_TRACE': '1'},
                report, cpu, monitor, False)
            parsed = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', output, re.M)[1])
            tests = parsed['tests']
            assert len(tests) == 16 and tests[-1]['name'] == 'terser'
            assert all(t['samples'] == len(t['sample_seconds'])
                       == (20 if t['name'] in ('chai', 'terser') else 3) for t in tests)
            audits = p.a.audits(output)
            assert audits['ALLOCATION_REFINED_AUDIT']['sites'] == 0
            assert audits['ALLOCATION_PATTERN_AUDIT']['hints'] == 0
            assert audits['GENERATIONAL_AUDIT']['minor'] == 0
            begin = re.search(r'^PROFILE_BEGIN terser (\d+)$', output, re.M)
            end = re.search(r'^PROFILE_END terser (\d+)$', output, re.M)
            assert begin and end and begin.end() < end.start()
            segment = output[begin.end():end.start()]
            marks = [json.loads(x) for x in re.findall(r'^GC_CYCLE_MARK (.*)$', segment, re.M)]
            reclaims = [json.loads(x) for x in re.findall(r'^GC_CYCLE_RECLAIM (.*)$', segment, re.M)]
            assert marks and all(x['full'] and not x['abandoned'] for x in marks)
            start, stop = float(begin[1]) / 1000 - offset, float(end[1]) / 1000 - offset
            profile = subprocess.check_output(['perf', 'report', '--stdio', '--no-children',
                '--percent-limit', '0.3', '--sort', 'symbol,dso', '--time', f'{start:.6f},{stop:.6f}',
                '-i', str(perf_data)], text=True)
            (report / (control + '-terser-perf.txt')).write_text(profile)
            row.update(control=control, target=tests[-1], tests=tests, audits=audits,
                epoch_minus_monotonic=offset, target_start_monotonic=start,
                target_end_monotonic=stop, marks=marks, reclaims=reclaims,
                target_gc={'full_collections': len(marks), 'mark_ms': sum(x['ms'] for x in marks),
                    'reclaim_ms': sum(x['ms'] for x in reclaims),
                    'allocated_bytes': sum(x['allocated'] for x in marks),
                    'mean_heap_bytes': statistics.mean(x['heap'] for x in reclaims)})
            data['rows'].append(row)
            print('TERSER PROFILE RESULT', control, row['target']['hz'], row['target_gc'], flush=True)
        except Exception as error:
            data['failures'].append({'control': control, 'error': repr(error)})
            data['status'] = 'failed'
            raise
        finally:
            c.write_json(report / 'measurements.json', data)
    data['status'] = 'passed'
    c.write_json(report / 'measurements.json', data)


COHORT_PATCH = (
    '--- a/alloc.c\n'
    '+++ b/alloc.c\n'
    '@@ -19,6 +19,18 @@\n'
    ' \n'
    ' #include "private/gc_priv.h"\n'
    ' #include "private/vdb_isolate.h"\n'
    '+GC_INNER void GC_cohort_configure(void);\n'
    '+GC_INNER void GC_cohort_finish(void);\n'
    '+GC_INNER void GC_cohort_admit(void);\n'
    '+GC_INNER void GC_cohort_schedule(GC_bool static_hot);\n'
    '+GC_INNER void GC_cohort_trace_activation(void);\n'
    '+GC_INNER extern MAY_THREAD_LOCAL GC_bool GC_cohort_negative;\n'
    '+GC_INNER void GC_cohort_disable(void);\n'
    '+GC_INNER extern MAY_THREAD_LOCAL unsigned GC_cohort_mode;\n'
    '+GC_INNER extern MAY_THREAD_LOCAL GC_bool GC_cohort_ready;\n'
    '+GC_INNER extern MAY_THREAD_LOCAL word GC_cohort_phase;\n'
    '+STATIC MAY_THREAD_LOCAL GC_bool GC_cohort_engaged;\n'
    '+STATIC MAY_THREAD_LOCAL word GC_cohort_rejected_phase;\n'
    ' \n'
    ' /*\n'
    '  * Separate free lists are maintained for different sized objects up\n'
    '@@ -907,6 +919,9 @@\n'
    '   GC_ASSERT(!GC_is_initialized && mode <= 4);\n'
    '   GC_pattern_reset_sites();\n'
    '   GC_pattern_mode = mode;\n'
    '+  GC_cohort_configure();\n'
    '+  GC_cohort_engaged = FALSE;\n'
    '+  GC_cohort_rejected_phase = 0;\n'
    '   GC_pattern_hints[0] = GC_pattern_hints[1] = GC_pattern_hints[2] = 0;\n'
    '   GC_pattern_total = GC_pattern_last_full = GC_pattern_close_anchor = 0;\n'
    '   GC_pattern_quiet_fulls = 0;\n'
    '@@ -1067,6 +1082,9 @@\n'
    ' GC_INNER void\n'
    ' GC_admission_deinit(void)\n'
    ' {\n'
    '+  GC_cohort_disable();\n'
    '+  GC_cohort_engaged = FALSE;\n'
    '+  GC_cohort_rejected_phase = 0;\n'
    '   GC_pattern_reset_sites();\n'
    '   GC_pattern_mode = 0;\n'
    '   GC_pattern_hints[0] = GC_pattern_hints[1] = GC_pattern_hints[2] = 0;\n'
    '@@ -1138,11 +1156,13 @@\n'
    '     if (GC_pattern_mode >= 2) {\n'
    '       word recent = GC_pattern_total - GC_pattern_last_full;\n'
    '       GC_pattern_last_full = GC_pattern_total;\n'
    '-      if (recent != 0)\n'
    '+      if (recent != 0 || (GC_cohort_engaged && GC_cohort_ready))\n'
    '         GC_pattern_quiet_fulls = 0;\n'
    '       else if (GC_pattern_activations != 0 && GC_pattern_quiet_fulls < 2)\n'
    '         ++GC_pattern_quiet_fulls;\n'
    '-      pattern_quiet = GC_pattern_quiet_fulls >= 2;\n'
    '+      pattern_quiet = GC_pattern_quiet_fulls >= 2\n'
    '+          || (GC_cohort_mode == 3 && GC_cohort_engaged\n'
    '+              && GC_cohort_negative && recent == 0);\n'
    '     }\n'
    '     if (GC_admission_closed) {\n'
    '       word delta = live > GC_admission_anchor\n'
    '@@ -1168,7 +1188,10 @@\n'
    '         GC_bool fresh_phase = GC_pattern_fresh_evidence >= 256\n'
    '                               && GC_gc_no - GC_pattern_closed_gc >= 2;\n'
    '         retry = fresh_phase\n'
    '-                || (GC_admission_probe_fulls == 0 && evidence >= required);\n'
    '+                || (GC_admission_probe_fulls == 0 && evidence >= required)\n'
    '+                || (GC_cohort_engaged && GC_cohort_ready\n'
    '+                    && GC_cohort_phase != GC_cohort_rejected_phase\n'
    '+                    && GC_admission_probe_fulls == 0);\n'
    '       } else if (GC_pattern_mode >= 2) {\n'
    '         retry = GC_admission_probe_fulls == 0\n'
    '                 && GC_pattern_total - GC_pattern_close_anchor >= 4096;\n'
    '@@ -1192,6 +1215,7 @@\n'
    ' #endif\n'
    '             )) {\n'
    '       GC_admission_closed = TRUE;\n'
    '+      if (GC_cohort_engaged) GC_cohort_rejected_phase = GC_cohort_phase;\n'
    '       GC_admission_anchor = live;\n'
    '       GC_pattern_close_anchor = GC_pattern_total;\n'
    '       if (GC_pattern_mode == 4) {\n'
    '@@ -2260,6 +2284,7 @@\n'
    '     GC_young_reclaim_active = GC_prepare_young_reclaim();\n'
    ' #endif\n'
    '   clear_all_fl_marks();\n'
    '+  GC_cohort_finish();\n'
    ' \n'
    '   GC_VERBOSE_LOG_PRINTF("Bytes recovered before sweep - f.l. count = %ld\\n",\n'
    '                         (long)GC_bytes_found);\n'
    '@@ -2312,6 +2337,9 @@\n'
    '       COMMA_IF_USE_MUNMAP(TO_KiB_UL(GC_unmapped_bytes)),\n'
    '       TO_KiB_UL(GC_our_mem_bytes - GC_heapsize + sizeof(GC_arrays)));\n'
    '   GC_DBGLOG_PRINT_HEAP_IN_USE();\n'
    '+  GC_cohort_admit();\n'
    '+  if (GC_is_full_gc)\n'
    '+    GC_cohort_schedule(GC_pattern_total != GC_pattern_last_full);\n'
    '   GC_admission_cost_finish(GC_is_full_gc);\n'
    '   GC_admission_finish(GC_is_full_gc);\n'
    '   GC_admission_skip_protect = FALSE;\n'
    '@@ -2353,11 +2381,14 @@\n'
    '       && (GC_used_heap_size_after_full >= GC_incremental_start_threshold\n'
    '           || (GC_pattern_mode == 2 && GC_pattern_total >= 4096)\n'
    '           || (GC_pattern_mode == 3 && GC_pattern_total >= 64)\n'
    '-          || (GC_pattern_mode == 4 && GC_pattern_evidence >= 64))) {\n'
    '+          || (GC_pattern_mode == 4 && (GC_pattern_evidence >= 64\n'
    '+              || (GC_cohort_mode >= 2 && GC_cohort_ready))))) {\n'
    '     /* All reachable objects were traced by this ordinary full collection.\n'
    '        Start dirty tracking only after all live objects have been traced;\n'
    '        the first protection pass covers every pointer-bearing heap range. */\n'
    '     GC_bool resuming = GC_admission_vdb_parked;\n'
    '+    if (GC_cohort_mode >= 2 && GC_cohort_ready && GC_pattern_evidence < 64)\n'
    '+      GC_cohort_engaged = TRUE;\n'
    '     GC_incremental_start_pending = FALSE;\n'
    '     GC_incremental = TRUE;\n'
    '     GC_handle_protected_regions_limit();\n'
    '@@ -2369,6 +2400,7 @@\n'
    '       }\n'
    '       if (GC_incremental && GC_pattern_mode != 0 && !resuming) {\n'
    '         ++GC_pattern_activations;\n'
    '+        if (GC_cohort_engaged) GC_cohort_trace_activation();\n'
    '         GC_pattern_activation_gc = GC_gc_no;\n'
    '         GC_pattern_activation_hints = GC_pattern_total;\n'
    '         GC_pattern_activation_evidence = GC_pattern_evidence;\n'
    '--- a/malloc.c\n'
    '+++ b/malloc.c\n'
    '@@ -17,6 +17,181 @@\n'
    ' #include "private/gc_priv.h"\n'
    ' \n'
    ' #include <string.h>\n'
    '+\n'
    '+/* Diagnostic cohort sampler: encoded addresses never own sampled objects.\n'
    '+ * Random-spaced allocation samples with a bounded reservoir, checked only\n'
    '+ * after complete full marking/finalization and free-list mark clearing. */\n'
    '+GC_INNER MAY_THREAD_LOCAL unsigned GC_cohort_mode;\n'
    '+STATIC MAY_THREAD_LOCAL GC_bool GC_cohort_sampling;\n'
    '+GC_INNER MAY_THREAD_LOCAL GC_bool GC_cohort_ready;\n'
    '+GC_INNER MAY_THREAD_LOCAL GC_bool GC_cohort_negative;\n'
    '+STATIC MAY_THREAD_LOCAL GC_bool GC_cohort_batch_negative, GC_cohort_batch_idle;\n'
    '+STATIC MAY_THREAD_LOCAL word GC_cohort_last[6];\n'
    '+GC_INNER MAY_THREAD_LOCAL word GC_cohort_phase;\n'
    '+STATIC MAY_THREAD_LOCAL GC_bool GC_cohort_trace, GC_cohort_batch_good;\n'
    '+STATIC MAY_THREAD_LOCAL unsigned GC_cohort_streak;\n'
    '+STATIC MAY_THREAD_LOCAL word GC_cohort_previous_live;\n'
    '+STATIC MAY_THREAD_LOCAL unsigned GC_cohort_gap, GC_cohort_rng;\n'
    '+STATIC MAY_THREAD_LOCAL word GC_cohort_seen;\n'
    '+STATIC MAY_THREAD_LOCAL struct {\n'
    '+  GC_hidden_pointer hidden;\n'
    '+  word size;\n'
    '+  unsigned kind;\n'
    '+} GC_cohort_samples[128];\n'
    '+\n'
    '+STATIC unsigned GC_cohort_random(void)\n'
    '+{\n'
    '+  unsigned x = GC_cohort_rng;\n'
    '+  x ^= x << 13; x ^= x >> 17; x ^= x << 5;\n'
    '+  return GC_cohort_rng = x;\n'
    '+}\n'
    '+\n'
    '+GC_INNER void GC_cohort_configure(void)\n'
    '+{\n'
    '+  const char *s = GETENV("GC_DIAG_COHORT");\n'
    '+  GC_cohort_mode = s != NULL ? (unsigned)atoi(s) : 0;\n'
    '+  GC_cohort_sampling = GC_cohort_mode != 0\n'
    '+      && (GC_cohort_mode < 6 || GC_cohort_mode == 7);\n'
    '+  GC_cohort_trace = GETENV("GC_DIAG_COHORT_TRACE") != NULL;\n'
    '+  GC_cohort_ready = GC_cohort_batch_good = GC_cohort_negative = FALSE;\n'
    '+  GC_cohort_batch_negative = GC_cohort_batch_idle = FALSE;\n'
    '+  BZERO(GC_cohort_last, sizeof(GC_cohort_last));\n'
    '+  GC_cohort_phase = GC_cohort_streak = GC_cohort_previous_live = 0;\n'
    '+  GC_cohort_gap = 1;\n'
    '+  GC_cohort_rng = 0x1f123bb5;\n'
    '+  GC_cohort_seen = 0;\n'
    '+  BZERO(GC_cohort_samples, sizeof(GC_cohort_samples));\n'
    '+}\n'
    '+\n'
    '+GC_INLINE void GC_cohort_record(void *object, word size, unsigned kind)\n'
    '+{\n'
    '+  if (UNLIKELY(GC_cohort_sampling) && GC_generational && !GC_dont_gc\n'
    '+      && !IS_UNCOLLECTABLE(kind) && object != NULL && --GC_cohort_gap == 0) {\n'
    '+    word slot;\n'
    '+    GC_cohort_gap = 1 + (GC_cohort_random() & 511);\n'
    '+    ++GC_cohort_seen;\n'
    '+    slot = GC_cohort_seen <= 128 ? GC_cohort_seen - 1\n'
    '+                                : GC_cohort_random() % GC_cohort_seen;\n'
    '+    if (slot < 128) {\n'
    '+      GC_cohort_samples[slot].hidden = GC_HIDE_POINTER(object);\n'
    '+      GC_cohort_samples[slot].size = size;\n'
    '+      GC_cohort_samples[slot].kind = kind;\n'
    '+    }\n'
    '+  }\n'
    '+}\n'
    '+\n'
    '+GC_INNER void GC_cohort_finish(void)\n'
    '+{\n'
    '+  word count, alive = 0, bytes = 0, alive_bytes = 0;\n'
    '+  word pointer_count = 0, pointer_alive = 0;\n'
    '+  if (!GC_cohort_sampling || !GC_is_full_gc) return;\n'
    '+  count = GC_cohort_seen < 128 ? GC_cohort_seen : 128;\n'
    '+  for (word i = 0; i < count; ++i) {\n'
    '+    const void *p = GC_REVEAL_POINTER(GC_cohort_samples[i].hidden);\n'
    '+    hdr *h = HDR(p);\n'
    '+    GC_bool pointer_kind = GC_obj_kinds[GC_cohort_samples[i].kind].ok_descriptor != 0;\n'
    '+    GC_bool live = h != NULL && !IS_FORWARDING_ADDR_OR_NIL(h)\n'
    '+        && !HBLK_IS_FREE(h) && h->hb_sz == GC_cohort_samples[i].size\n'
    '+        && GC_is_marked(p);\n'
    '+    bytes = SIZET_SAT_ADD(bytes, GC_cohort_samples[i].size);\n'
    '+    pointer_count += pointer_kind;\n'
    '+    if (live) {\n'
    '+      ++alive; alive_bytes = SIZET_SAT_ADD(alive_bytes, GC_cohort_samples[i].size);\n'
    '+      pointer_alive += pointer_kind;\n'
    '+    }\n'
    '+  }\n'
    '+  GC_cohort_batch_good = bytes != GC_WORD_MAX && count >= 64 && pointer_count >= 32\n'
    '+      && alive * 8 <= count && pointer_alive * 8 <= pointer_count\n'
    '+      && alive_bytes <= bytes / 4 && GC_bytes_allocd >= 1024 * 1024\n'
    '+      && !GC_find_leak_inner && !GC_is_enumerate_reachable_objects;\n'
    '+  GC_cohort_batch_negative = count >= 64\n'
    '+      && (alive * 8 > count || alive_bytes > bytes / 4\n'
    '+          || (pointer_count >= 32 && pointer_alive * 8 > pointer_count));\n'
    '+  GC_cohort_batch_idle = GC_bytes_allocd < 1024 * 1024;\n'
    '+  GC_cohort_last[0] = count; GC_cohort_last[1] = alive;\n'
    '+  GC_cohort_last[2] = pointer_count; GC_cohort_last[3] = pointer_alive;\n'
    '+  GC_cohort_last[4] = bytes; GC_cohort_last[5] = alive_bytes;\n'
    '+  if (GC_cohort_trace) GC_err_printf("COHORT {\\"gc\\":%lu,\\"count\\":%lu,\\"seen\\":%lu,\\"alive\\":%lu,\\"bytes\\":%lu,\\"alive_bytes\\":%lu,\\"pointer_count\\":%lu,\\"pointer_alive\\":%lu,\\"allocated\\":%lu,\\"heap\\":%lu,\\"free\\":%lu,\\"incremental\\":%d}\\n",\n'
    '+      (unsigned long)GC_gc_no, (unsigned long)count, (unsigned long)GC_cohort_seen,\n'
    '+      (unsigned long)alive, (unsigned long)bytes, (unsigned long)alive_bytes,\n'
    '+      (unsigned long)pointer_count, (unsigned long)pointer_alive,\n'
    '+      (unsigned long)GC_bytes_allocd, (unsigned long)GC_heapsize,\n'
    '+      (unsigned long)GC_large_free_bytes, GC_incremental);\n'
    '+  GC_cohort_seen = 0;\n'
    '+  BZERO(GC_cohort_samples, sizeof(GC_cohort_samples));\n'
    '+}\n'
    '+\n'
    '+GC_INNER void GC_cohort_admit(void)\n'
    '+{\n'
    '+  word live;\n'
    '+  GC_bool good, was_ready;\n'
    '+  if (!GC_cohort_mode || !GC_is_full_gc) return;\n'
    '+  live = GC_composite_in_use + GC_atomic_in_use;\n'
    '+  good = GC_cohort_batch_good && live >= 4 * 1024 * 1024\n'
    '+      && GC_cohort_previous_live != 0\n'
    '+      && live <= GC_cohort_previous_live + GC_cohort_previous_live / 8;\n'
    '+  GC_cohort_negative = GC_cohort_batch_negative || GC_cohort_batch_idle\n'
    '+      || (GC_cohort_previous_live != 0\n'
    '+          && live > GC_cohort_previous_live + GC_cohort_previous_live / 8);\n'
    '+  GC_cohort_previous_live = live;\n'
    '+  was_ready = GC_cohort_ready;\n'
    '+  if (good) {\n'
    '+    if (GC_cohort_streak < 2) ++GC_cohort_streak;\n'
    '+  } else if (GC_cohort_mode != 3 || GC_cohort_negative || !was_ready) {\n'
    '+    GC_cohort_streak = 0;\n'
    '+  }\n'
    '+  /* Inconclusive samples cannot start a trial, but do not erase evidence\n'
    '+     for an ongoing low-survival phase. Contradictory data does. */\n'
    '+  GC_cohort_ready = GC_cohort_streak >= 2;\n'
    '+  if (GC_cohort_ready && !was_ready) ++GC_cohort_phase;\n'
    '+  if (GC_cohort_trace) GC_err_printf("COHORT_PHASE gc=%lu good=%u ready=%u phase=%lu live=%lu\\n",\n'
    '+      (unsigned long)GC_gc_no, (unsigned)good, (unsigned)GC_cohort_ready,\n'
    '+      (unsigned long)GC_cohort_phase, (unsigned long)live);\n'
    '+}\n'
    '+\n'
    '+GC_INNER void GC_cohort_schedule(GC_bool static_hot)\n'
    '+{\n'
    '+  GC_bool enabled;\n'
    '+  word live;\n'
    '+  if (GC_cohort_mode < 6) return;\n'
    '+  live = GC_composite_in_use + GC_atomic_in_use;\n'
    '+  /* The fallback analysis cannot admit a smaller live heap, and current\n'
    '+     static evidence already keeps the existing policy active. Resume\n'
    '+     sampling at the next full boundary when either situation changes. */\n'
    '+  enabled = !static_hot\n'
    '+      && (GC_cohort_mode == 7 || live >= 4 * 1024 * 1024);\n'
    '+  if (GC_cohort_trace && enabled != GC_cohort_sampling)\n'
    '+    GC_err_printf("COHORT_SAMPLING gc=%lu enabled=%u static_hot=%u live=%lu\\n",\n'
    '+        (unsigned long)GC_gc_no, (unsigned)enabled, (unsigned)static_hot,\n'
    '+        (unsigned long)live);\n'
    '+  if (!enabled && GC_cohort_sampling) {\n'
    '+    GC_cohort_seen = GC_cohort_streak = 0;\n'
    '+    GC_cohort_ready = GC_cohort_batch_good = FALSE;\n'
    '+    BZERO(GC_cohort_samples, sizeof(GC_cohort_samples));\n'
    '+  }\n'
    '+  GC_cohort_sampling = enabled;\n'
    '+}\n'
    '+\n'
    '+GC_INNER void GC_cohort_trace_activation(void)\n'
    '+{\n'
    '+  GC_err_printf("COHORT_ACTIVATION {\\"gc\\":%lu,\\"phase\\":%lu,\\"ready\\":%u,\\"count\\":%lu,\\"alive\\":%lu,\\"pointer_count\\":%lu,\\"pointer_alive\\":%lu,\\"bytes\\":%lu,\\"alive_bytes\\":%lu}\\n",\n'
    '+      (unsigned long)GC_gc_no, (unsigned long)GC_cohort_phase,\n'
    '+      (unsigned)GC_cohort_ready, (unsigned long)GC_cohort_last[0],\n'
    '+      (unsigned long)GC_cohort_last[1], (unsigned long)GC_cohort_last[2],\n'
    '+      (unsigned long)GC_cohort_last[3], (unsigned long)GC_cohort_last[4],\n'
    '+      (unsigned long)GC_cohort_last[5]);\n'
    '+}\n'
    '+\n'
    '+GC_INNER void GC_cohort_disable(void)\n'
    '+{\n'
    '+  GC_cohort_mode = 0;\n'
    '+  GC_cohort_sampling = FALSE;\n'
    '+  GC_cohort_seen = 0;\n'
    '+  GC_cohort_ready = GC_cohort_batch_good = GC_cohort_negative = FALSE;\n'
    '+  GC_cohort_batch_negative = GC_cohort_batch_idle = FALSE;\n'
    '+  BZERO(GC_cohort_last, sizeof(GC_cohort_last));\n'
    '+  BZERO(GC_cohort_samples, sizeof(GC_cohort_samples));\n'
    '+}\n'
    ' \n'
    ' STATIC MAY_THREAD_LOCAL GC_bool GC_large_defer_started;\n'
    ' STATIC MAY_THREAD_LOCAL word GC_large_defer_epoch;\n'
    '@@ -151,6 +326,7 @@\n'
    '   }\n'
    '   /* FIXME: Do we need some way to reset `GC_max_large_allocd_bytes`? */\n'
    '   result = h->hb_body;\n'
    '+  GC_cohort_record(result, lb_adjusted, (unsigned)kind);\n'
    '   GC_ASSERT((ADDR(result) & align_m1) == 0);\n'
    '   return result;\n'
    ' }\n'
    '@@ -276,6 +452,7 @@\n'
    '   *opp = obj_link(op);\n'
    '   obj_link(op) = NULL;\n'
    '   GC_bytes_allocd += GRANULES_TO_BYTES((word)lg);\n'
    '+  GC_cohort_record(op, GRANULES_TO_BYTES((word)lg), (unsigned)kind);\n'
    '   return op;\n'
    ' }\n'
    ' \n'
    '@@ -428,6 +605,7 @@\n'
    '       if (kind != PTRFREE)\n'
    '         obj_link(op) = NULL;\n'
    '       GC_bytes_allocd += GRANULES_TO_BYTES((word)lg);\n'
    '+      GC_cohort_record(op, GRANULES_TO_BYTES((word)lg), (unsigned)kind);\n'
    '       UNLOCK();\n'
    '       GC_ASSERT((ADDR(op) & align_m1) == 0);\n'
    '       return op;\n'
    '@@ -496,6 +674,7 @@\n'
    '       *opp = obj_link(op);\n'
    '       obj_link(op) = NULL;\n'
    '       GC_bytes_allocd += GRANULES_TO_BYTES((word)lg);\n'
    '+      GC_cohort_record(op, GRANULES_TO_BYTES((word)lg), (unsigned)kind);\n'
    '       /*\n'
    '        * Mark bit was already set on free list.  It will be cleared only\n'
    '        * temporarily during a collection, as a result of the normal\n'
)


def cohort_audit(contents, mode, require_active=True):
    audits = p.a.audits(contents)
    common, gc = audits['PHONE_MODE_AUDIT'], audits['GENERATIONAL_AUDIT']
    hint, ref = audits['ALLOCATION_PATTERN_AUDIT'], audits['ALLOCATION_REFINED_AUDIT']
    paused = audits['TRACKING_PAUSE_AUDIT']['paused']
    assert mode == 'refined' and common['free_space_divisor'] == 12
    assert hint['mode'] == 4 and gc['full_frequency'] == 2
    assert hint['activations'] in (0, 1) and paused in (0, 1)
    if hint['activations']:
        assert common['incremental'] or paused, (
            'Tracking disabled unexpectedly after activation', common, gc, ref)
        if ref['activation_evidence'] < 64:
            activation = json.loads(re.search(r'^COHORT_ACTIVATION (.*)$', contents, re.M)[1])
            assert activation['ready'] == 1 and activation['count'] >= 64
            assert activation['pointer_count'] >= 32
            assert activation['alive'] * 8 <= activation['count']
            assert activation['pointer_alive'] * 8 <= activation['pointer_count']
            assert activation['alive_bytes'] * 4 <= activation['bytes']
    else:
        assert common['incremental'] == 0 and gc['minor'] == 0
    return {'common': common, 'generational': gc, 'pattern': hint,
            'refined': ref, 'admission': [audits['ADMISSION_AUDIT']]}


def cohort_probe():
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    assert arch in ('arm32', 'arm64')
    report = root / ('cohort-' + arch)
    report.mkdir()
    gc_source = source / 'third_party/GCutil'
    original = {n: (gc_source / n).read_bytes() for n in ['malloc.c', 'alloc.c']}
    binary = source.parent / ('build-' + arch) / 'escargot'
    original_binary_sha = c.sha256(binary)
    engine = report / 'measured-engine'
    patch = report / 'cohort-diagnostic.patch'
    patch.write_text(COHORT_PATCH)
    try:
        c.run(['git', 'apply', '--check', str(patch)], cwd=gc_source)
        c.run(['git', 'apply', str(patch)], cwd=gc_source)
        hashes = {n: c.sha256(gc_source / n) for n in original}
        print('BUILD bounded cohort admission diagnostic', arch, flush=True)
        if arch == 'arm32':
            container = ('escargot-arm-benchmark-' + os.environ['GITHUB_RUN_ID']
                         + '-' + os.environ['GITHUB_RUN_ATTEMPT'])
            c.run(['docker', 'start', '--attach', container])
        else:
            c.run(['cmake', '--build', str(binary.parent), '-j', '2'])
        shutil.copy2(binary, engine)
    finally:
        for name, data in original.items():
            (gc_source / name).write_bytes(data)
    assert not c.output(['git', 'status', '--porcelain'], source)
    header = engine.read_bytes()[:20]
    assert header[:4] == b'\x7fELF' and header[4] == (1 if arch == 'arm32' else 2)
    assert struct.unpack('<H', header[18:20])[0] == (40 if arch == 'arm32' else 183)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    env = p.environments()['refined']
    prefix = []
    if arch == 'arm32':
        runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
        prefix = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path', runtime]
        env['LD_LIBRARY_PATH'] = runtime
    command = [*prefix, str(engine), '--gc-generational']
    stock = r.prepare_drivers(harness, report)
    text = (report / 'wtb-score.js').read_text().replace(
        'count:b.count}', 'count:b.count,sample_seconds:b.stats.sample}')
    needle = '\nsuite.run();\n'
    assert text.count(needle) == 1
    text = text.replace(needle, '\nsuite.forEach(function(b){if(b.name==="chai"||b.name==="terser")'
        '{b.minSamples=20;b.options.minSamples=20;}});' + needle)
    (report / 'wtb-score.js').write_text(text)
    octane = (harness / 'test/octane/run.js').read_text()
    assert octane.count('BenchmarkSuite.config.doDeterministic = undefined;') == 1
    octane = octane.replace('BenchmarkSuite.config.doDeterministic = undefined;',
                           'BenchmarkSuite.config.doDeterministic = true;')
    assert octane.count('NotifyScore: PrintScore });') == 1
    octane = octane.replace('NotifyScore: PrintScore });',
        'NotifyScore: PrintScore }, BenchmarkSuite.suites.filter(function(s){'
        'return s.name!=="CodeLoad";}).map(function(s){return s.name;}));')
    code_driver = report / 'codeload-fixed.js'
    code_driver.write_text(octane)
    workloads = [('CodeLoad', harness / 'test/octane', code_driver, 1)]
    workloads += [(name, cwd, driver, 1)
                  for name, cwd, driver in stock if name in ('fixed_work', 'web_tooling')]
    for name in p.a.NAMES:
        driver = report / (name + '.js')
        driver.write_text(p.a.driver_text(name))
        workloads.append((name, report, driver, 1))
    c.parse_audit = cohort_audit
    data = {'status': 'running', 'arch': arch,
        'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
        'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], gc_source),
        'original_binary_sha256': original_binary_sha, 'binary_sha256': c.sha256(engine),
        'diagnostic_sources_sha256': hashes, 'patch_sha256': c.sha256(patch),
        'inputs': json.loads((root / 'benchmark-inputs.json').read_text()),
        'build': json.loads((root / ('build-' + arch + '.json')).read_text()),
        'protocol': 'Bounded native screen, not a promoted runtime or full-GC comparison. '
            'Same diagnostic executable with GC_DIAG_COHORT=0/7; both refined, native '
            'default flags, FSD12, seed1, CPU pinned, normal ASLR, no trace/profiler/RSS sampler. '
            'Original WTB18 roots/order: one fresh-process pair, Chai/Terser '
            'twenty samples and other cases three. One fixed WTB18x3-call pair and one '
            'deterministic CodeLoad pair retaining all Octane preloads. Opposite orders '
            'across architectures. GC source restored after build; helper contains an unused '
            'mode3 experiment. Metadata sampler uses saturating byte totals on both word sizes. '
            'Sampling is advisory; normal full-GC and heap-growth rules unchanged. '
            'Fallback sampling pauses while '
            'static hints are active, and resumes when they go quiet. Startup sampling '
            'continues unchanged when static evidence is absent. '
            'Six existing allocation workloads guard analysis overhead. '
            'Remaining full regressions required before promotion.',
        'rows': [], 'failures': []}
    def save():
        projected = [dict(x, mode='full' if x['variant'] == 0 else 'refined') for x in data['rows']]
        data['summary'] = m.summarize(projected)
        for item in data['summary'].values():
            item['medians'] = {'current': item['medians']['full'],
                               'cohort': item['medians']['refined']}
        c.write_json(report / 'measurements.json', data)
    save()
    for index, (name, cwd, driver, repeats) in enumerate(workloads):
        for repeat in range(repeats):
            order = [0, 7] if (index + repeat + (arch == 'arm64')) % 2 == 0 else [7, 0]
            for variant in order:
                label = name + '-' + str(repeat + 1) + '-cohort' + str(variant)
                try:
                    row, output = c.process_measure(label, 'refined', command, cwd, driver,
                        {**env, 'GC_DIAG_COHORT': str(variant)}, report, cpu, monitor, False)
                    row.update(name=name, repeat=repeat + 1, variant=variant,
                               status='passed', audits=p.a.audits(output))
                    if variant == 0:
                        assert not re.search(r'^COHORT_ACTIVATION ', output, re.M)
                    if name == 'CodeLoad':
                        row['score'] = float(re.search(r'^CodeLoad: ([0-9.]+)$', output, re.M)[1])
                        row['subtests'] = {'CodeLoad': row['score']}
                    elif name == 'web_tooling':
                        parsed = json.loads(re.search(r'^WTB_SCORE_JSON (.*)$', output, re.M)[1])
                        row.update(score=parsed['score'], tests=parsed['tests'])
                        assert {t['name'] for t in row['tests']} == r.WTB_NAMES
                        assert all(t['samples'] == len(t['sample_seconds'])
                            == (20 if t['name'] in ('chai', 'terser') else 3) for t in row['tests'])
                    elif name in p.a.NAMES:
                        row['benchmark'] = p.a.parse_bench(output, name)
                    else:
                        r.parse_result(name, row, output)
                    data['rows'].append(row)
                    print('COHORT RESULT', label, row.get('score', row.get('work_milliseconds')),
                          row['peak_rss_kib'], flush=True)
                except Exception as error:
                    data['failures'].append({'label': label, 'error': repr(error)})
                    print('COHORT FAILURE', label, repr(error), flush=True)
                save()
    data['status'] = 'failed' if data['failures'] else 'passed'
    save()
    assert not data['failures'] and len(data['rows']) == 18, data['failures']
    print('COHORT COMPLETE', arch, len(data['rows']), flush=True)


def cohort_crash_probe():
    # Change only the fatal return instruction after exec, then detach.
    # Normal write faults, libraries and allocator code remain untraced.
    import ctypes
    import errno
    import pty
    import resource
    import selectors
    import signal
    import tty
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = root / ('cohort-crash-' + arch)
    report.mkdir()
    if arch != 'arm64':
        c.write_json(report / 'measurements.json', {'status': 'skipped', 'arch': arch})
        return
    gdb = shutil.which('gdb')
    if not gdb:
        subprocess.run(['sudo', '-n', 'apt-get', 'update'], check=True)
        subprocess.run(['sudo', '-n', 'apt-get', 'install', '-y', 'gdb'], check=True)
        gdb = shutil.which('gdb')
    assert gdb
    subprocess.run(['sudo', '-n', gdb, '-nx', '-nh', '-batch', '-ex', 'quit'], check=True)
    gc_source = source / 'third_party/GCutil'
    original = {n: (gc_source / n).read_bytes() for n in ['malloc.c', 'alloc.c']}
    patch = report / 'cohort.patch'
    patch.write_text(COHORT_PATCH)
    engine = report / 'measured-engine'
    retained = Path('/home/sh8281.kim/actions-runner-2/_work2/_temp/'
                    'escargot-arm-report.fo6br531/cohort-arm64/measured-engine')
    failed_sha = 'c06479d7a7a5bd3c770d537abf83f109fabdb140a70b89e56af498e6bf33c425'
    try:
        c.run(['git', 'apply', '--check', str(patch)], cwd=gc_source)
        c.run(['git', 'apply', str(patch)], cwd=gc_source)
        source_hashes = {n: c.sha256(gc_source / n) for n in original}
        if retained.is_file() and c.sha256(retained) == failed_sha:
            shutil.copy2(retained, engine)
            binary_origin = 'Exact retained failed executable, SHA-256 checked'
        else:
            binary = source.parent / 'build-arm64/escargot'
            c.run(['cmake', '--build', str(binary.parent), '-j', '2'])
            shutil.copy2(binary, engine)
            binary_origin = 'Rebuilt identical GC sources; not the exact failed ELF'
    finally:
        for n, data in original.items():
            (gc_source / n).write_bytes(data)
    assert not c.output(['git', 'status', '--porcelain'], source)
    disassembly = c.output(['objdump', '-d', '--disassemble=GC_default_on_abort', str(engine)])
    (report / 'abort-disassembly.txt').write_text(disassembly + '\n')
    returns = re.findall(r'^\s*([0-9a-f]+):\s+d65f03c0\s+ret\s*$', disassembly, re.M)
    assert len(returns) == 1, returns
    fatal_return = int(returns[0], 16)
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    r.prepare_drivers(harness, report)
    driver = report / 'wtb-score.js'
    text = driver.read_text().replace('count:b.count}', 'count:b.count,sample_seconds:b.stats.sample}')
    needle = '\nsuite.run();\n'
    assert text.count(needle) == 1
    text = text.replace(needle, '\nsuite.forEach(function(b){if(b.name==="chai"||b.name==="terser")'
        '{b.minSamples=20;b.options.minSamples=20;}});' + needle)
    driver.write_text(text)
    assert c.sha256(driver) == 'cd6656ded9d78137521a1c4002340c1a091c0f8ac550d12a7c38008880ef8caf'
    configured = p.environments()['refined']
    # Keep the environment's ordered shape and string lengths, but remove
    # inherited CI credentials before allowing a core to be captured.
    safe = {'PATH', 'HOME', 'LANG', 'LANGUAGE', 'LC_ALL', 'LC_CTYPE', 'TZ',
            'LD_LIBRARY_PATH', 'LD_PRELOAD', 'PWD', 'TMPDIR'}
    env = {k: v if k.startswith(('GC_', 'ESCARGOT_')) or k in safe else 'x' * len(v)
           for k, v in configured.items()}
    env['LD_PRELOAD'] = ''
    assert not any(k.startswith(('GC_', 'ESCARGOT_')) and 'TOKEN' in k for k in env)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    os.sched_setaffinity(0, {monitor})
    libc = ctypes.CDLL(None, use_errno=True)
    libc.ptrace.restype = ctypes.c_long
    libc.ptrace.argtypes = [ctypes.c_uint, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
    def ptrace(request, pid, address=0, value=0):
        ctypes.set_errno(0)
        result = libc.ptrace(request, pid, ctypes.c_void_p(address), ctypes.c_void_p(value))
        if result == -1 and ctypes.get_errno():
            raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))
        return result & ((1 << 64) - 1)
    data = {'status': 'running', 'arch': arch, 'diagnostic_not_acceptance': True,
        'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
        'binary_sha256': c.sha256(engine), 'binary_origin': binary_origin,
        'driver_sha256': c.sha256(driver), 'gc_sources_sha256': source_hashes,
        'source_directory': str(source), 'fatal_return_offset': fatal_return,
        'protocol': 'Original failed WTB driver byte-for-byte, PTY, no preload. '
            'At exec stop replace only the fatal GC callback return with an ARM64 '
            'self-branch, then detach before any engine code executes. No normal '
            'SIGSEGV is traced. On a fatal log attach GDB and capture the original '
            'signal context, collector state and core. CI values are replaced by '
            'equal-length inert strings; public engine settings are preserved. '
            'Twelve mode7 fresh processes, normal ASLR; last eight add inert layout '
            'padding. No scores accepted. Stop on first captured error.',
        'rows': []}
    c.write_json(report / 'measurements.json', data)
    def child_setup():
        os.sched_setaffinity(0, {cpu})
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        ptrace(0, 0)  # PTRACE_TRACEME; exec will stop before engine startup.
    paddings = [None] * 4 + [0, 64, 256, 1024, 4096, 8192, 16384, 32768]
    for trial, padding in enumerate(paddings, 1):
        label = 'trial-' + str(trial) + '-mode7'
        trial_env = dict(env, GC_DIAG_COHORT='7')
        if padding is not None:
            trial_env['GC_REPLAY_LAYOUT'] = 'L' * padding
        log = report / (label + '.log')
        core = report / (label + '.core')
        trace = report / (label + '.gdb.txt')
        started = time.monotonic()
        last_progress = started
        master, slave = pty.openpty()
        tty.setraw(slave)
        selector = selectors.DefaultSelector()
        selector.register(master, selectors.EVENT_READ)
        process = None
        captured = False
        debug_code = None
        try:
            process = subprocess.Popen([str(engine), '--gc-generational', str(driver)],
                cwd=harness / 'test/web-tooling-benchmark', env=trial_env,
                stdin=subprocess.DEVNULL, stdout=slave, stderr=slave,
                start_new_session=True, preexec_fn=child_setup)
            os.close(slave)
            slave = None
            waited, status = os.waitpid(process.pid, os.WUNTRACED)
            assert waited == process.pid and os.WIFSTOPPED(status) and os.WSTOPSIG(status) == signal.SIGTRAP
            maps_path = Path('/proc') / str(process.pid) / 'maps'
            maps = maps_path.read_text()
            mapped = [line.split(None, 5) for line in maps.splitlines() if str(engine) in line]
            base = min(int(x[0].split('-')[0], 16) - int(x[2], 16) for x in mapped)
            address = base + fatal_return
            word = ptrace(1, process.pid, address)  # PEEK, preserve the adjacent instruction.
            assert word & 0xffffffff == 0xd65f03c0, hex(word)
            ptrace(4, process.pid, address, (word & 0xffffffff00000000) | 0x14000000)
            assert ptrace(1, process.pid, address) & 0xffffffff == 0x14000000
            ptrace(17, process.pid)  # DETACH: normal execution is not traced.
            print('CRASH REPLAY START', label, 'padding', padding, 'base', hex(base), flush=True)
            output = bytearray()
            with log.open('wb') as stream:
                while process.poll() is None or selector.get_map():
                    for key, _ in selector.select(0.2):
                        try:
                            chunk = os.read(master, 65536)
                        except OSError as error:
                            if error.errno != errno.EIO:
                                raise
                            chunk = b''
                        if not chunk:
                            selector.unregister(master)
                        else:
                            stream.write(chunk)
                            stream.flush()
                            output.extend(chunk)
                    now = time.monotonic()
                    if now - last_progress >= 30:
                        last_progress = now
                        status_text = (Path('/proc') / str(process.pid) / 'status').read_text() if process.poll() is None else ''
                        if status_text:
                            assert re.search(r'^TracerPid:\s+0$', status_text, re.M), status_text
                        print('CRASH REPLAY PROGRESS', label, round(now-started), 's', flush=True)
                    if b'Unexpected segmentation fault' in output and process.poll() is None:
                        print('FATAL ERROR; CAPTURING ORIGINAL SIGNAL STACK', label, flush=True)
                        mapping_text = maps_path.read_text()
                        (report / (label + '.maps')).write_text(mapping_text)
                        libraries = set()
                        for line in mapping_text.splitlines():
                            fields = line.split(None, 5)
                            if len(fields) == 6 and fields[5].startswith('/'):
                                library = Path(fields[5])
                                if '.so' in library.name and library.is_file():
                                    libraries.add(library)
                        for library in libraries:
                            target = report / 'sysroot' / str(library).lstrip('/')
                            target.parent.mkdir(parents=True, exist_ok=True)
                            shutil.copy2(library, target)
                        data['library_files'] = sorted(map(str, libraries))
                        commands = report / (label + '.gdb')
                        commands.write_text('set pagination off\nset confirm off\nset auto-load off\n'
                            'set print elements 32\nthread apply all bt full 60\ninfo registers\n'
                            'generate-core-file ' + str(core) + '\n'
                            'python\n'
                            'for name in ["GC_gc_no", "GC_incremental", "GC_is_full_gc", "GC_mark_state", '
                            '"GC_young_mark_active", "GC_young_old_valid", "GC_young_baseline_pending", '
                            '"GC_admission_closed", "GC_admission_vdb_parked"]:\n'
                            ' try: gdb.execute("p " + name)\n'
                            ' except gdb.error as error: print(error)\n'
                            'end\n'
                            'set {unsigned int}' + hex(address) + ' = 0xd65f03c0\ndetach\nquit\n')
                        with trace.open('w') as debug_output:
                            debug = subprocess.run(['sudo', '-n', gdb, '-nx', '-nh', '-q', '-batch',
                                str(engine), '-p', str(process.pid), '-x', str(commands)],
                                stdout=debug_output, stderr=subprocess.STDOUT, timeout=55)
                        debug_code = debug.returncode
                        captured = core.is_file() and core.stat().st_size > 0
                        process.wait(timeout=30)
                        break
                    if now - started > 900:
                        raise TimeoutError('Crash replay exceeded 900 seconds')
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            if slave is not None:
                os.close(slave)
            selector.close()
            os.close(master)
        row = {'trial': trial, 'mode': 7, 'padding': padding, 'load_base': hex(base),
            'engine_returncode': process.returncode, 'seconds': time.monotonic()-started,
            'captured': captured, 'debugger_returncode': debug_code, 'log': log.name,
            'environment_shape': [[k, len(v)] for k, v in trial_env.items()]}
        if core.is_file():
            subprocess.run(['sudo', '-n', 'chown', str(os.getuid()) + ':' + str(os.getgid()), str(core)], check=True)
            row['core_bytes'] = core.stat().st_size
            c.run(['gzip', '-1', str(core)])
            row['compressed_core'] = core.name + '.gz'
        if process.returncode == 0:
            cohort_audit(log.read_text(errors='replace'), 'refined')
            row['audit'] = 'passed'
        data['rows'].append(row)
        data['status'] = 'captured' if captured else 'running'
        c.write_json(report / 'measurements.json', data)
        print('CRASH REPLAY RESULT', label, row['engine_returncode'], 'captured', captured, flush=True)
        if captured or process.returncode != 0:
            data['status'] = 'captured' if captured else 'failure_without_capture'
            c.write_json(report / 'measurements.json', data)
            return
    data['status'] = 'not_reproduced'
    c.write_json(report / 'measurements.json', data)


def main():
    if int(os.environ["BENCHMARK_REPETITIONS"]) == 5:
        cohort_probe()
        return
    if int(os.environ['BENCHMARK_REPETITIONS']) == 1:
        cohort_crash_probe()
        return
    source = Path.cwd().resolve()
    root = Path(os.environ['BENCHMARK_OUTPUT_DIR']).resolve()
    harness = Path(os.environ['BENCHMARK_HARNESS']).resolve()
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    assert arch in ('arm32', 'arm64') and int(os.environ['BENCHMARK_REPETITIONS']) == 3
    report = root / ('gc-quiet-phase-' + arch)
    report.mkdir()
    binary = source.parent / ('build-' + arch) / 'escargot'
    alloc = source / 'third_party/GCutil/alloc.c'
    original = alloc.read_text()
    probe = quiet_probe_source(original)
    (report / 'diagnostic-alloc.c').write_text(probe)
    original_binary_sha256 = c.sha256(binary)
    engine = report / 'measured-engine'
    try:
        alloc.write_text(probe)
        print('BUILD same-binary quiet-phase diagnostic', arch, flush=True)
        if arch == 'arm32':
            container = ('escargot-arm-benchmark-' + os.environ['GITHUB_RUN_ID']
                         + '-' + os.environ['GITHUB_RUN_ATTEMPT'])
            c.run(['docker', 'start', '--attach', container])
        else:
            c.run(['cmake', '--build', str(binary.parent), '-j', '2'])
        shutil.copy2(binary, engine)
    finally:
        alloc.write_text(original)
    assert not c.output(['git', 'status', '--porcelain'], source)
    header = engine.read_bytes()[:20]
    assert header[:4] == b'\x7fELF' and header[4] == (1 if arch == 'arm32' else 2)
    assert struct.unpack('<H', header[18:20])[0] == (40 if arch == 'arm32' else 183)
    cpu = int(os.environ['BENCHMARK_CPU'])
    monitor = next(x for x in sorted(os.sched_getaffinity(0)) if x != cpu)
    env = p.environments()['refined']
    prefix = []
    if arch == 'arm32':
        runtime = os.environ['BENCHMARK_RUNTIME_DIRECTORY']
        prefix = [os.environ['BENCHMARK_RUNTIME_LOADER'], '--library-path', runtime]
        env['LD_LIBRARY_PATH'] = runtime
    command = [*prefix, str(engine), '--gc-generational']
    c.parse_audit = m.audit
    stock = r.prepare_drivers(harness, report)
    data = {
        'status': 'running', 'arch': arch,
        'revision': c.output(['git', 'rev-parse', 'HEAD'], source),
        'gc_revision': c.output(['git', 'rev-parse', 'HEAD'], source / 'third_party/GCutil'),
        'original_binary_sha256': original_binary_sha256, 'binary_sha256': c.sha256(engine),
        'original_alloc_sha256': hashlib.sha256(original.encode()).hexdigest(),
        'diagnostic_alloc_sha256': hashlib.sha256(probe.encode()).hexdigest(),
        'inputs': json.loads((root / 'benchmark-inputs.json').read_text()),
        'build': json.loads((root / ('build-' + arch + '.json')).read_text()),
        'protocol': 'Diagnostic only, not a production policy change or full-GC comparison. '
            'Temporarily patch alloc.c to select one/two hintless full collections before '
            'quiet suspension, rebuild in the same native toolchain, copy the executable '
            'and restore source. Both policies use that identical executable and command, '
            'with GC_DIAG_QUIET_FULLS set to equal-length values 1/2. Native defaults, FSD12, '
            'seed1, CPU pinning, no profiler/cycle trace/RSS sampler. Three alternating '
            'SunSpider26 pairs, two original-order Octane17 pairs, and one pair for each '
            'of six allocation patterns (scale5/warmup1/samples3). Reverse each pair order '
            'across architectures. Pattern fixed-work peak RSS is a memory guard; adaptive '
            'Octane RSS is supplementary. Record failures and finish other workloads. '
            'This is a bounded screen: retain production unchanged if gains are mixed, '
            'material losses repeat, or correctness/memory/benefit guards deteriorate. '
            'Inactive WTB is excluded because static analysis and activation are unchanged.',
        'rows': [], 'failures': [],
    }

    def save():
        data['summary'] = summarize(data['rows'])
        c.write_json(report / 'measurements.json', data)

    workloads = [(n, cwd, driver, 3 if n == 'sunspider' else 2)
                 for n, cwd, driver in stock if n in ('sunspider', 'octane')]
    for name in p.a.NAMES:
        driver = report / (name + '.js')
        driver.write_text(p.a.driver_text(name))
        workloads.append((name, report, driver, 1))
    save()
    for index, (name, cwd, driver, repeats) in enumerate(workloads):
        for repeat in range(repeats):
            order = [2, 1] if (index + repeat + (arch == 'arm64')) % 2 == 0 else [1, 2]
            for limit in order:
                label = name + '-' + str(repeat + 1) + '-quiet' + str(limit)
                try:
                    row, text = c.process_measure(label, 'refined', command, cwd, driver,
                        {**env, 'GC_DIAG_QUIET_FULLS': str(limit)}, report, cpu, monitor, False)
                    row.update(name=name, repeat=repeat + 1, quiet_fulls=limit,
                               status='passed', audits=p.a.audits(text))
                    if name in p.a.NAMES:
                        row['benchmark'] = p.a.parse_bench(text, name)
                    else:
                        r.parse_result(name, row, text)
                    data['rows'].append(row)
                    print('QUIET PHASE RESULT', label,
                          row.get('score', row.get('total_milliseconds', row.get('benchmark'))), flush=True)
                except Exception as error:
                    data['failures'].append({'label': label, 'error': repr(error)})
                    print('QUIET PHASE FAILURE', label, repr(error), flush=True)
                save()
    data['status'] = 'failed' if data['failures'] else 'passed'
    save()
    assert not data['failures'], data['failures']
    assert len(data['rows']) == 22
    print('QUIET PHASE COMPLETE', arch, len(data['rows']), flush=True)


if __name__ == '__main__':
    main()
