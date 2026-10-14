#!/usr/bin/env python3
"""Measure the current hot-page policy with natural GC on native ARM."""

import importlib.util
import json
import os
from pathlib import Path
import re


def main():
    for key in ('GC_ENABLE_INCREMENTAL', 'GC_DISABLE_INCREMENTAL', 'GC_PRINT_STATS',
                'GC_PRINT_VERBOSE_STATS', 'GC_PROFILE_MPROTECT', 'GC_FULL_FREQUENCY',
                'GC_PAUSE_TIME_TARGET', 'ESCARGOT_GC_TIME_UNLIMITED',
                'ESCARGOT_GC_FULL_EVERY_COLLECTION', 'ESCARGOT_GC_DISABLE_AUTOMATIC'):
        os.environ.pop(key, None)
    os.environ.update(GC_USE_GETWRITEWATCH='0', GC_FREE_SPACE_DIVISOR='12',
                      GC_INCREMENTAL_START_THRESHOLD='10M', GC_MPROTECT_COARSE='1',
                      ESCARGOT_HOTPAGE_AUDIT='1')
    harness = Path(os.environ['BENCHMARK_HARNESS'])
    spec = importlib.util.spec_from_file_location('arm_scores_memory',
                                                harness / 'tools/arm-v8-lite-experiment.py')
    experiment = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(experiment)
    original = experiment.run_scores

    def incremental_scores(monthly, engines, harness, wtb, report, data, repetitions):
        for name, command, bridge in engines:
            assert name == 'escargot' and bridge == 'escargot'
            command.append('--gc-incremental')
            data['engines'][name]['command'] = command
        return original(monthly, engines, harness, wtb, report, data, repetitions)

    experiment.run_scores = incremental_scores
    experiment.main(mode='both', engine_names=('escargot',))
    arch = os.environ['BENCHMARK_ARCHITECTURE']
    report = Path(os.environ['BENCHMARK_OUTPUT_DIR']) / f'v8-lite-{arch}-scores-memory'
    audits = {}
    for log in sorted(report.glob('escargot-*.log')):
        records = re.findall(r'^HOTPAGE_AUDIT (\{.*\})$', log.read_text(errors='replace'), re.M)
        if records:
            assert len(records) == 1, log
            audits[log.name] = json.loads(records[0])
    (report / 'mode-audit.json').write_text(json.dumps(audits, indent=2) + '\n')
    assert audits, 'Missing mode audit'
    for name, audit in audits.items():
        assert audit['automatic_disabled'] == 0 and audit['full_frequency'] > 0, (name, audit)
    measured = {name: value for name, value in audits.items()
                if 'web_tooling' in name or 'octane' in name}
    assert measured and all(a['incremental'] == 1 and a['actual_vdb'] == 1
                            for a in measured.values()), measured
    print('CURRENT_POLICY_VALIDATED', json.dumps(measured), flush=True)


if __name__ == '__main__':
    main()
