#!/usr/bin/env python3
"""Restore the mapping limit left by the cancelled full-GC comparison."""

import json
import os
from pathlib import Path
import subprocess


baseline = 65530
temporary_limit = 1000000
setting = Path('/proc/sys/vm/max_map_count')
observed = int(setting.read_text())
assert observed in (baseline, temporary_limit), observed
if observed != baseline:
    subprocess.run(['sudo', '-n', 'sysctl', '-w',
                    'vm.max_map_count=' + str(baseline)], check=True)
restored = int(setting.read_text())
assert restored == baseline, restored
record = {'observed_vm_max_map_count': observed,
          'restored_vm_max_map_count': restored,
          'baseline_evidence_run': 38532811238,
          'cancelled_comparison_run': 38541184578}
output = Path(os.environ['BENCHMARK_OUTPUT_DIR'])
(output / ('mapping-restore-' + os.environ['BENCHMARK_ARCHITECTURE'] + '.json')).write_text(
    json.dumps(record, indent=2) + '\n')
print('RESTORE_MAPPING_BUDGET', json.dumps(record), flush=True)
