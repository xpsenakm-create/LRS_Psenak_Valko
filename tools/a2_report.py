#!/usr/bin/env python3
"""
a2_report.py
============
Tabuľka výsledkov A2 letov zo summary.json (vytvára ich a2_plot.py) a jej vloženie do
dokumentácie medzi značky <!-- RESULTS_TABLE_BEGIN --> a <!-- RESULTS_TABLE_END -->.

Použitie:
    python3 tools/a2_report.py results/A2/flight_seed42 results/A2/flight_seed73 ...
    python3 tools/a2_report.py results/A2/flight_* --doc docs/A2/A2_lokalizacia.md
"""

import argparse
import json
import os
import re
import sys

BEGIN, END = '<!-- RESULTS_TABLE_BEGIN -->', '<!-- RESULTS_TABLE_END -->'


def scenario_of(run_dir):
    """First line of scenario.log that describes the scenario (legs and delays)."""
    path = os.path.join(run_dir, 'logs', 'scenario.log')
    if not os.path.isfile(path):
        return ''
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = re.search(r'Scenario: (.*)$', line)
            if m:
                return m.group(1).strip()
    return ''


def table(run_dirs):
    # compact (fits an A4 page in the PDF); the scenario text is printed below the table
    rows = ['| beh | koniec | trasa | RMSE xy / z | max xy | RMSE yaw | xy / yaw v ±3σ | '
            'max chyba GPS výpadok | max chyba VO výpadok | outlier d² | pristátie od štartu |',
            '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    notes = []
    for d in run_dirs:
        p = os.path.join(d, 'summary.json')
        if not os.path.isfile(p):
            print(f'preskakujem {d}: chýba summary.json', file=sys.stderr)
            continue
        with open(p, encoding='utf-8') as f:
            s = json.load(f)
        name = os.path.basename(os.path.normpath(d))
        if name.startswith('flight_'):
            name = name[len('flight_'):]
        gps = ', '.join(f'{g["max_xy"]:.2f} m ({g["length"]:.0f} s)' for g in s.get('gps_gaps', [])) or '–'
        vo = ', '.join(f'{g["max_xy"]:.2f} m ({g["length"]:.0f} s)' for g in s.get('vo_gaps', [])) or '–'
        rej = ', '.join(f'{r["d2"]:.0f}' for r in s.get('rejected', [])) or '–'
        scen = scenario_of(d).replace('|', '/')
        if scen:
            for n in notes:
                if n[1] == scen:
                    n[0].append(name)
                    break
            else:
                notes.append(([name], scen))
        rows.append(
            f'| `{name}` | {s.get("final_state", "?")} | '
            f'{s.get("route_s", float("nan")):.0f} s | {s["rmse_xy"]:.2f} / {s["rmse_z"]:.2f} m | {s["max_xy"]:.2f} m | '
            f'{s["rmse_yaw_deg"]:.2f}° | {100 * s["inside3_xy"]:.0f} / {100 * s["inside3_yaw"]:.0f} % | '
            f'{gps} | {vo} | {rej} | {s.get("landed_from_start", float("nan")):.2f} m |')
    if notes:
        rows += ['', 'Scenáre: ' + '; '.join(', '.join(f'`{n}`' for n in names) + f' — {scen}'
                                              for names, scen in notes) + '.']
    return '\n'.join(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='run directories with summary.json')
    ap.add_argument('--doc', help='markdown file to update between the RESULTS_TABLE markers')
    args = ap.parse_args()
    t = table(args.runs)
    print(t)
    if args.doc:
        with open(args.doc, encoding='utf-8') as f:
            text = f.read()
        if BEGIN not in text or END not in text:
            sys.exit(f'Chyba: v {args.doc} chýbajú značky {BEGIN} / {END}')
        text = re.sub(re.escape(BEGIN) + '.*?' + re.escape(END), lambda _: f'{BEGIN}\n{t}\n{END}', text, flags=re.S)
        with open(args.doc, 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'--> tabuľka vložená do {args.doc}')


if __name__ == '__main__':
    main()
