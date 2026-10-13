#!/usr/bin/env python3
"""
plot_rosbag.py
==============
Vykreslí prejdenú a požadovanú dráhu z ROS 2 bagu alebo CSV logu:
  1. Samostatný pôdorys: Štvorec (W1..W4) so smerovaním nosa na POI
  2. Samostatný pôdorys: Osmička (Lemniskáta) s dotyčnicovým smerovaním nosa
  3. Odchýlka natočenia nosa (Yaw error [°]) voči POI / dotyčnici
  4. Priebeh Yaw v čase (požadovaný vs nameraný)
  5. Výškový profil (Z vs Čas)
  6. Polohová odchýlka (Tracking Error [m])

Použitie:
    python3 plot_rosbag.py                      # automaticky nájde outdoor_mission_log.csv
    python3 plot_rosbag.py cesta/k/bagu         # z ROS 2 bagu
    python3 plot_rosbag.py --csv moj_log.csv    # z vlastného CSV
    python3 plot_rosbag.py --no-show            # iba uloží obrázky bez GUI
"""

import argparse
import csv
import math
import os
import sys

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("Chyba: Matplotlib nie je nainštalovaný. Spustite: pip install matplotlib", file=sys.stderr)
    sys.exit(1)


def parse_csv_log(csv_path):
    """Načíta dáta z CSV logu vygenerovaného outdoor_mission_node."""
    print(f"--> Načítavam dáta z CSV: {csv_path}")
    data = {
        't': [], 'state': [],
        'cmd_x': [], 'cmd_y': [], 'cmd_z': [], 'cmd_yaw': [],
        'x': [], 'y': [], 'z': [], 'yaw': [],
        'yaw_err': [], 'poi_err': [],
        'track_err': [], 'rel_alt': []
    }
    def to_float(val, default=0.0):
        if val is None or val == '' or str(val).lower() == 'nan':
            return default
        try:
            return float(val)
        except (ValueError, TypeError):
            return default

    def to_float_or_nan(val):
        if val is None or val == '' or str(val).lower() == 'nan':
            return float('nan')
        try:
            return float(val)
        except (ValueError, TypeError):
            return float('nan')

    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                # Ak je riadok neúplný (napr. prerušený pri Ctrl+C)
                if not row or row.get('t_s') is None or row.get('x') is None:
                    continue

                t = to_float(row.get('t_s'))
                state = str(row.get('state') or '').strip()
                cx = to_float(row.get('cmd_x'))
                cy = to_float(row.get('cmd_y'))
                cz = to_float(row.get('cmd_z'))
                cyaw = to_float(row.get('cmd_yaw_deg'))

                x = to_float(row.get('x'))
                y = to_float(row.get('y'))
                z = to_float(row.get('z'))
                yaw = to_float(row.get('yaw_deg'))

                yerr = to_float_or_nan(row.get('yaw_err_deg'))
                perr = to_float_or_nan(row.get('poi_bearing_err_deg'))

                err = to_float(row.get('track_err_m'))
                rel = to_float(row.get('rel_alt'), default=z)

                data['t'].append(t)
                data['state'].append(state)
                data['cmd_x'].append(cx)
                data['cmd_y'].append(cy)
                data['cmd_z'].append(cz)
                data['cmd_yaw'].append(cyaw)
                data['x'].append(x)
                data['y'].append(y)
                data['z'].append(z)
                data['yaw'].append(yaw)
                data['yaw_err'].append(yerr)
                data['poi_err'].append(perr)
                data['track_err'].append(err)
                data['rel_alt'].append(rel)
            except (ValueError, KeyError, TypeError):
                continue
    return data


def parse_rosbag(bag_path):
    """Načíta dáta z ROS 2 bag priečinka pomocou rosbag2_py."""
    print(f"--> Načítavam dáta z ROS 2 bagu: {bag_path}")
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        print(f"Upozornenie: rosbag2_py alebo rclpy nie je dostupné ({e}).", file=sys.stderr)
        return None

    storage_options = rosbag2_py.StorageOptions(uri=bag_path)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Chyba pri otváraní bagu: {e}", file=sys.stderr)
        return None

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    wanted_topics = {
        '/mavros/local_position/pose',
        '/mavros/setpoint_raw/local',
        '/mission/intended_path'
    }

    flown = {'t': [], 'x': [], 'y': [], 'z': [], 'yaw': []}
    target = {'t': [], 'cmd_x': [], 'cmd_y': [], 'cmd_z': [], 'cmd_yaw': []}

    t0 = None

    while reader.has_next():
        topic, data_bytes, stamp_ns = reader.read_next()
        if topic not in wanted_topics:
            continue

        if t0 is None:
            t0 = stamp_ns
        t_sec = (stamp_ns - t0) * 1e-9

        msg_type_name = type_map.get(topic)
        if not msg_type_name:
            continue

        msg_cls = get_message(msg_type_name)
        msg = deserialize_message(data_bytes, msg_cls)

        if topic == '/mavros/local_position/pose':
            pos = msg.pose.position
            ori = msg.pose.orientation
            yaw_rad = math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y),
                                 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))
            flown['t'].append(t_sec)
            flown['x'].append(pos.x)
            flown['y'].append(pos.y)
            flown['z'].append(pos.z)
            flown['yaw'].append(math.degrees(yaw_rad))

        elif topic == '/mavros/setpoint_raw/local':
            target['t'].append(t_sec)
            target['cmd_x'].append(msg.position.x)
            target['cmd_y'].append(msg.position.y)
            target['cmd_z'].append(msg.position.z)
            target['cmd_yaw'].append(math.degrees(msg.yaw))

    if not flown['t']:
        print("V bagu sa nenašli žiadne dáta pre /mavros/local_position/pose!", file=sys.stderr)
        return None

    # Rozdelenie fáz na základe času alebo polohy
    data = {
        't': flown['t'],
        'state': ['UNKNOWN'] * len(flown['t']),
        'x': flown['x'], 'y': flown['y'], 'z': flown['z'], 'yaw': flown['yaw'],
        'cmd_x': target['cmd_x'], 'cmd_y': target['cmd_y'], 'cmd_z': target['cmd_z'],
        'cmd_t': target['t'], 'cmd_yaw': target['cmd_yaw'],
        'yaw_err': [], 'poi_err': [], 'track_err': []
    }
    return data


def filter_by_states(data, valid_states):
    """Vráti podsadu dát pre dané letové stavy."""
    indices = [i for i, s in enumerate(data.get('state', [])) if s in valid_states]
    sub = {}
    for k, v in data.items():
        if isinstance(v, list) and len(v) == len(data['t']):
            sub[k] = [v[i] for i in indices]
        else:
            sub[k] = v
    return sub


def detect_geometry(data, user_square=None, user_amp=None):
    """Zistí alebo automaticky deteguje dĺžku štvorca a amplitúdu osmičky."""
    if user_square is not None:
        square_side = user_square
    else:
        sq_x = [abs(data['cmd_x'][i]) for i in range(len(data.get('cmd_x', [])))
                if data.get('state', [''])[i] == 'SQUARE']
        square_side = round(2.0 * max(sq_x), 2) if sq_x else 5.0

    if user_amp is not None:
        amplitude = user_amp
    else:
        f8_x = [abs(data['cmd_x'][i]) for i in range(len(data.get('cmd_x', [])))
                if data.get('state', [''])[i] == 'FIGURE8']
        amplitude = round(max(f8_x), 2) if f8_x else 3.0

    return square_side, amplitude


def plot_single_square(data_sq, square_side=5.0, output_file=None):
    """Vykreslí samostatný detailný graf štvorca s orientáciou na POI."""
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_title(f'A1.3 — Pôdorys dráhy: Štvorec ({square_side}×{square_side} m) s orientáciou nosa na POI', fontsize=12, fontweight='bold')

    half = square_side / 2.0
    w_corners = [(-half, half), (half, half), (half, -half), (-half, -half), (-half, half)]
    cx = [p[0] for p in w_corners]
    cy = [p[1] for p in w_corners]
    ax.plot(cx, cy, 'k--', linewidth=1.5, label=f'Požadovaný štvorec ({square_side}×{square_side} m)')

    # Nameraná dráha štvorca
    if data_sq.get('x'):
        ax.plot(data_sq['x'], data_sq['y'], 'b-', linewidth=2.0, label='Prejdená dráha drona')

        # Vektorové šípky natočenia nosa (každých N bodov)
        step = max(1, len(data_sq['x']) // 25)
        for i in range(0, len(data_sq['x']), step):
            px, py = data_sq['x'][i], data_sq['y'][i]
            yaw_rad = math.radians(data_sq['yaw'][i])
            u = 0.4 * math.cos(yaw_rad)
            v = 0.4 * math.sin(yaw_rad)
            ax.arrow(px, py, u, v, head_width=0.12, head_length=0.15, fc='cyan', ec='darkblue', alpha=0.8)

    # POI
    ax.scatter([0.0], [0.0], color='red', s=200, marker='*', zorder=6, label='POI (Point of Interest)')

    # Rohy W1..W4
    labels = [
        f'W1 (-{half:.1f}, {half:.1f})',
        f'W2 ({half:.1f}, {half:.1f})',
        f'W3 ({half:.1f}, -{half:.1f})',
        f'W4 (-{half:.1f}, -{half:.1f})'
    ]
    for i, (wx, wy) in enumerate(w_corners[:4]):
        ax.scatter([wx], [wy], color='darkorange', s=80, zorder=5)
        ax.annotate(f" {labels[i]}", (wx, wy), fontsize=9, fontweight='bold')

    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.axis('equal')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='best', fontsize=9)

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"==> Samostatný graf štvorca uložený do: {output_file}")
    plt.close(fig)


def plot_single_figure8(data_f8, amplitude=3.0, output_file=None):
    """Vykreslí samostatný detailný graf osmičky s dotyčnicovým yaw."""
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_title(f'A1.3 — Pôdorys dráhy: Osmička (Figure-8 lemniskáta A={amplitude} m)', fontsize=12, fontweight='bold')

    # Teoretická ideálna osmička
    t_vals = [2.0 * math.pi * i / 200.0 for i in range(201)]
    ideal_x = [amplitude * math.sin(t) for t in t_vals]
    ideal_y = [amplitude * math.sin(t) * math.cos(t) for t in t_vals]
    ax.plot(ideal_x, ideal_y, 'k--', linewidth=1.5, label=f'Požadovaná osmička (A={amplitude} m)')

    # Nameraná dráha osmičky
    if data_f8.get('x'):
        ax.plot(data_f8['x'], data_f8['y'], 'm-', linewidth=2.0, label='Prejdená dráha drona')

        # Vektorové šípky natočenia nosa pozdĺž dotyčnice
        step = max(1, len(data_f8['x']) // 25)
        for i in range(0, len(data_f8['x']), step):
            px, py = data_f8['x'][i], data_f8['y'][i]
            yaw_rad = math.radians(data_f8['yaw'][i])
            u = 0.35 * math.cos(yaw_rad)
            v = 0.35 * math.sin(yaw_rad)
            ax.arrow(px, py, u, v, head_width=0.1, head_length=0.12, fc='yellow', ec='purple', alpha=0.8)

    ax.scatter([0.0], [0.0], color='red', s=180, marker='*', zorder=6, label='Stred osmičky (POI)')

    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.axis('equal')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='best', fontsize=9)

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"==> Samostatný graf osmičky uložený do: {output_file}")
    plt.close(fig)


def plot_single_yaw_error(data, output_file=None):
    """Vykreslí samostatný detailný graf odchýlky natočenia nosa drona."""
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title('A1.3 — Uhlová odchýlka natočenia nosa drona (Yaw Error [°])', fontsize=12, fontweight='bold')

    # Odchýlka voči POI počas štvorca
    t_poi = []
    err_poi = []
    t_f8 = []
    err_f8 = []

    for i in range(len(data['t'])):
        st = data['state'][i] if 'state' in data else ''
        t = data['t'][i]

        if st in ('TO_START', 'SQUARE') and i < len(data.get('poi_err', [])):
            val = data['poi_err'][i]
            if not math.isnan(val):
                t_poi.append(t)
                err_poi.append(abs(val))

        elif st == 'FIGURE8' and i < len(data.get('yaw_err', [])):
            val = data['yaw_err'][i]
            if not math.isnan(val):
                t_f8.append(t)
                err_f8.append(abs(val))

    if err_poi:
        avg_poi = sum(err_poi) / len(err_poi)
        max_poi = max(err_poi)
        ax.plot(t_poi, err_poi, 'b-', linewidth=1.8, label=f'Chyba voči POI počas štvorca (Priemer: {avg_poi:.1f}°, Max: {max_poi:.1f}°)')

    if err_f8:
        avg_f8 = sum(err_f8) / len(err_f8)
        max_f8 = max(err_f8)
        ax.plot(t_f8, err_f8, 'm-', linewidth=1.8, label=f'Chyba voči dotyčnici počas osmičky (Priemer: {avg_f8:.1f}°, Max: {max_f8:.1f}°)')

    ax.axhline(10.0, color='orange', linestyle='--', alpha=0.7, label='Referenčná tolerancia (10°)')
    ax.set_xlabel('Čas [s]')
    ax.set_ylabel('|Chyba Yaw| [°]')
    ax.set_ylim(bottom=0.0)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='best', fontsize=9)

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"==> Samostatný graf chyby yaw uložený do: {output_file}")
    plt.close(fig)


def plot_comprehensive(data, square_side=5.0, amplitude=3.0, output_file=None, show=True):
    """Vykreslí komplexný 6-panelový graf misie."""
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.25)
    fig.suptitle('A1.3 Outdoor GPS Misia — Komplexné vyhodnotenie letu', fontsize=16, fontweight='bold')

    has_states = 'state' in data and any(s in ('SQUARE', 'FIGURE8') for s in data['state'])

    # 1. Pôdorys: ŠTVOREC (W1..W4)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_title(f'1. Dráha štvorca ({square_side}×{square_side} m, POI tracking)', fontweight='bold')
    half = square_side / 2.0
    w_corners = [(-half, half), (half, half), (half, -half), (-half, -half), (-half, half)]
    ax1.plot([p[0] for p in w_corners], [p[1] for p in w_corners], 'k--', linewidth=1.5, label=f'Požadovaný štvorec ({square_side}×{square_side} m)')

    data_sq = filter_by_states(data, {'TO_START', 'SQUARE'}) if has_states else data
    if data_sq.get('x'):
        ax1.plot(data_sq['x'], data_sq['y'], 'b-', linewidth=2.0, label='Nameraná dráha')
    ax1.scatter([0.0], [0.0], color='red', s=160, marker='*', zorder=6, label='POI')
    for wx, wy in w_corners[:4]:
        ax1.scatter([wx], [wy], color='darkorange', s=50, zorder=5)
    ax1.set_xlabel('East [m]')
    ax1.set_ylabel('North [m]')
    ax1.axis('equal')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='best', fontsize=8)

    # 2. Pôdorys: OSMIČKA (Lemniskáta)
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_title(f'2. Dráha osmičky (Figure-8 A={amplitude} m)', fontweight='bold')
    t_vals = [2.0 * math.pi * i / 200.0 for i in range(201)]
    ax2.plot([amplitude * math.sin(t) for t in t_vals], [amplitude * math.sin(t) * math.cos(t) for t in t_vals],
             'k--', linewidth=1.5, label=f'Požadovaná osmička (A={amplitude} m)')

    data_f8 = filter_by_states(data, {'FIGURE8'}) if has_states else data
    if data_f8.get('x'):
        ax2.plot(data_f8['x'], data_f8['y'], 'm-', linewidth=2.0, label='Nameraná dráha')
    ax2.scatter([0.0], [0.0], color='red', s=160, marker='*', zorder=6, label='Stred (POI)')
    ax2.set_xlabel('East [m]')
    ax2.set_ylabel('North [m]')
    ax2.axis('equal')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='best', fontsize=8)

    # 3. Odchýlka natočenia nosa (YAW ERROR)
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.set_title('3. Uhlová odchýlka natočenia nosa (|Chyba Yaw| [°])', fontweight='bold')
    t_poi, err_poi, t_f8, err_f8 = [], [], [], []
    for i in range(len(data['t'])):
        st = data['state'][i] if has_states else ''
        t = data['t'][i]
        if st in ('TO_START', 'SQUARE') and i < len(data.get('poi_err', [])):
            v = data['poi_err'][i]
            if not math.isnan(v): t_poi.append(t); err_poi.append(abs(v))
        elif st == 'FIGURE8' and i < len(data.get('yaw_err', [])):
            v = data['yaw_err'][i]
            if not math.isnan(v): t_f8.append(t); err_f8.append(abs(v))

    if err_poi:
        ax3.plot(t_poi, err_poi, 'b-', linewidth=1.5, label=f'Štvorec -> POI (Ø {sum(err_poi)/len(err_poi):.1f}°)')
    if err_f8:
        ax3.plot(t_f8, err_f8, 'm-', linewidth=1.5, label=f'Osmička -> Dotyčnica (Ø {sum(err_f8)/len(err_f8):.1f}°)')
    ax3.axhline(10.0, color='orange', linestyle='--', alpha=0.6, label='Tolerancia (10°)')
    ax3.set_xlabel('Čas [s]')
    ax3.set_ylabel('|Chyba Yaw| [°]')
    ax3.set_ylim(bottom=0.0)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='best', fontsize=8)

    # 4. Priebeh Yaw v čase (Požadovaný vs Nameraný)
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.set_title('4. Priebeh Yaw v čase (ENU [°])', fontweight='bold')
    if data.get('cmd_yaw'):
        ax4.plot(data['t'], data['cmd_yaw'], 'r--', linewidth=1.3, label='Požadovaný yaw')
    ax4.plot(data['t'], data['yaw'], 'b-', linewidth=1.4, alpha=0.8, label='Nameraný yaw')
    ax4.set_xlabel('Čas [s]')
    ax4.set_ylabel('Yaw [°]')
    ax4.grid(True, linestyle=':', alpha=0.6)
    ax4.legend(loc='best', fontsize=8)

    # 5. Výškový profil (Z vs Čas)
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.set_title('5. Výškový profil (Z [m] vs Čas)', fontweight='bold')
    if data.get('cmd_z'):
        ax5.plot(data['t'], data['cmd_z'], 'r--', linewidth=1.3, label='Cieľová výška (5.0 m)')
    ax5.plot(data['t'], data['z'], 'b-', linewidth=1.8, label='Nameraná výška')
    ax5.set_xlabel('Čas [s]')
    ax5.set_ylabel('Výška Z [m]')
    ax5.set_ylim(-0.5, 6.5)
    ax5.grid(True, linestyle=':', alpha=0.6)
    ax5.legend(loc='best', fontsize=8)

    # 6. Polohová odchýlka (Tracking Error [m])
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.set_title('6. Polohová odchýlka sledovania (Error [m] vs Čas)', fontweight='bold')
    if data.get('track_err'):
        ax6.plot(data['t'], data['track_err'], 'k-', linewidth=1.5, label='Chyba polohy')
        ax6.axhline(0.3, color='orange', linestyle='--', label='Tolerancia (0.3 m)')
        ax6.axhline(2.0, color='red', linestyle='--', label='Limit prerušenia (2.0 m)')
    ax6.set_xlabel('Čas [s]')
    ax6.set_ylabel('Chyba [m]')
    ax6.set_ylim(bottom=0.0)
    ax6.grid(True, linestyle=':', alpha=0.6)
    ax6.legend(loc='best', fontsize=8)

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"==> Hlavný prehľadový graf bol uložený do: {output_file}")

    if show:
        print("==> Otváram interaktívne okno grafu...")
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Vykreslenie prejdenej dráhy z ROS 2 bagu alebo CSV logu.")
    parser.add_argument("bag_path", nargs="?", default=None, help="Cesta k priečinku ROS 2 bagu.")
    parser.add_argument("--csv", default=None, help="Cesta k CSV log súboru (napr. outdoor_mission_log.csv).")
    parser.add_argument("--output", "-o", default="outdoor_mission_plot.png", help="Názov hlavného obrázka.")
    parser.add_argument("--square", "--square-side", type=float, default=None, help="Dĺžka strany štvorca v metroch (ak nezadané, deteguje sa z dát).")
    parser.add_argument("--amplitude", "--amp", type=float, default=None, help="Amplitúda/polomer osmičky A v metroch (ak nezadané, deteguje sa z dát).")
    parser.add_argument("--no-show", action="store_true", help="Nezobrazovať interaktívne grafické okno, iba uložiť obrázky.")

    args = parser.parse_args()

    data = None
    if args.csv:
        if not os.path.exists(args.csv):
            print(f"Chyba: CSV súbor nebol nájdený: {args.csv}", file=sys.stderr)
            sys.exit(1)
        data = parse_csv_log(args.csv)
    elif args.bag_path:
        if not os.path.exists(args.bag_path):
            print(f"Chyba: Bag priečinok nebol nájdený: {args.bag_path}", file=sys.stderr)
            sys.exit(1)
        data = parse_rosbag(args.bag_path)
    else:
        default_csv = "outdoor_mission_log.csv"
        if os.path.exists(default_csv):
            print(f"Nebol zadaný argument, našiel som {default_csv}. Použijem ho.")
            data = parse_csv_log(default_csv)
        else:
            parser.print_help()
            sys.exit(1)

    if not data or not data.get('t'):
        print("Chyba: Nepodarilo sa načítať žiadne platné dáta na vykreslenie.", file=sys.stderr)
        sys.exit(1)

    # Detekcia alebo nastavenie geometrických parametrov
    square_side, amplitude = detect_geometry(data, args.square, args.amplitude)
    print(f"--> Parametre geometrie: Štvorec S = {square_side:.1f} m, Osmička A = {amplitude:.1f} m")

    # 1. Vygenerovanie samostatných detailných grafov pre dokumentáciu:
    has_states = 'state' in data and any(s in ('SQUARE', 'FIGURE8') for s in data['state'])
    data_sq = filter_by_states(data, {'TO_START', 'SQUARE'}) if has_states else data
    data_f8 = filter_by_states(data, {'FIGURE8'}) if has_states else data

    plot_single_square(data_sq, square_side=square_side, output_file="plot_1_square.png")
    plot_single_figure8(data_f8, amplitude=amplitude, output_file="plot_2_figure8.png")
    plot_single_yaw_error(data, output_file="plot_3_yaw_error.png")

    # 2. Vygenerovanie hlavného súhrnného 6-panelového grafu:
    plot_comprehensive(data, square_side=square_side, amplitude=amplitude, output_file=args.output, show=not args.no_show)


if __name__ == "__main__":
    main()
