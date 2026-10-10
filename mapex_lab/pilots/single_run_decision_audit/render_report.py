"""Small auditable report; technical smoke never becomes a scientific result."""
import csv
import json
from pathlib import Path
import numpy as np
from .contract import atomic_json
from .recorder import restore_state

def render(run_root, audit_root, shadow_root, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root, out = Path(run_root), Path(output)
    out.mkdir(parents=True, exist_ok=False)
    terminal = json.loads((root/'terminal.json').read_text())
    manifest = json.loads((root/'manifest.json').read_text())
    with (root/'snapshots.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    first, last = rows[0]['snapshot_id'], terminal['final_snapshot_id']
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for axis, sid, label in zip(axes, [first, last], ['Initial', 'Terminal']):
        meta, arrays = restore_state(root, sid)
        axis.imshow(arrays['obs_map'][500:703, 500:763], origin='lower', cmap='gray', vmin=0, vmax=1)
        axis.plot(arrays['pose_list'][:, 1]-500, arrays['pose_list'][:, 0]-500, color='tab:red')
        axis.set_title(label+' / '+manifest['purpose'])
        axis.set_xlabel('P1 column'); axis.set_ylabel('P1 row')
    fig.tight_layout()
    fig.savefig(str(out/'initial_terminal.png'), dpi=150)
    plt.close(fig)
    mapping = [
      ('native epoch/predictions/all candidates', 'native_decisions.csv + all_candidates_native.csv + objects/', (root/'native_decisions.csv').exists()),
      ('dispatch/control/actual sensor', 'native_dispatch.csv + native_steps.csv + raw_scans.csv', (root/'raw_scans.csv').exists()),
      ('dense immutable states', 'snapshots.csv + snapshots/ + objects/', bool(rows)),
      ('Layer0 + prediction/uncertainty/visibility/IG/ranking', 'shadow/*/shadow.json', Path(shadow_root).exists()),
      ('primary sensor oracle / P2 structural sensitivity', 'shadow/*/shadow.json:paired,candidates', Path(shadow_root).exists()),
      ('realized gain / prefix-tail / replay', 'audit/executed_gain.csv + coverage_steps.csv + audit_summary.json', (Path(audit_root)/'audit_summary.json').exists()),
      ('optional corner opportunities', 'NOT_COMPUTED_EXPLORATORY', True),
      ('alternate action realized gain', 'OUT_OF_SCOPE_REQUIRES_BRANCH_AUTHORITY', False),
    ]
    atomic_json(out/'availability.json', [{'output': name, 'path': path, 'status': 'AVAILABLE' if exists else 'GAP_OR_OUT_OF_SCOPE'}
                                         for name, path, exists in mapping])
    lines = ['# MX072 R5 '+manifest['purpose'], '',
             'Terminal: '+terminal['status']+'. No full-exploration claim.', '',
             'This is a technical smoke artifact when purpose=TECHNICAL_SMOKE; it is not scientific trajectory evidence.', '',
             '| Output | Path | Availability |', '|---|---|---|']
    lines += ['| '+name+' | '+path+' | '+('AVAILABLE' if exists else 'GAP/OUT_OF_SCOPE')+' |' for name,path,exists in mapping]
    lines += ['', 'One-run descriptive, conditional on preregistered support. Variance mass is separate from m².',
              'P2 is structural/raster sensitivity; the primary prediction contrast uses SENSOR_GEOMETRY_P1.',
              'Source ray-origin quirks, known-map errors and hypothetical versus realized gains remain distinct.',
              'Corner opportunities are not computed. Simulated seconds are NA_SIM_TIME_UNDEFINED.']
    (out/'README.md').write_text('\n'.join(lines)+'\n', encoding='utf8')
