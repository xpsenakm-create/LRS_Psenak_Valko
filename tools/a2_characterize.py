#!/usr/bin/env python3
"""
a2_characterize.py
==================
A2 §1 -- charakterizácia senzorov z ROS 2 bagov (offline, samostatný proces; ground truth sa
číta iba tu, nikdy v estimátore):

  * frekvencie z HLAVIČIEK správ (sim čas) aj z času príjmu (wall, závisí od real-time faktora),
  * frame_id / child_frame_id,
  * šum GPS = GPS − truth (stojaci dron),
  * drift VO = VO poloha/yaw − truth v čase, chyba VO rýchlosti v base_link,
  * bias gyra = w_z − skutočná rýchlosť yaw, kým sa dron neotáča.

Viac bagov (napr. seed 42, seed 73, znova seed 42) porovná medzi sebou: iný seed = iný bias/šum,
rovnaký seed = rovnaké poškodenie (porovnáva k-tu GPS chybu od začiatku bagu).

Použitie:
    python3 tools/a2_characterize.py BAG [BAG ...] [--labels seed42 seed73 seed42b] [--out DIR] [--no-show]
Bag pre porovnanie seedov nahrávaj od spustenia sensor_simulator (prvá GPS vzorka v bagu).
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a2_bag import interp_truth, read_bag, stamp, truth_arrays, wrap, yaw_of  # noqa: E402

TOPICS = ['/localization/gps', '/localization/vo', '/localization/imu', '/evaluation/ground_truth']


def rates(t_stamp, t_recv):
    """Mean rate from header stamps (gaps = outages excluded) and from arrival (wall) time."""
    d = np.diff(t_stamp)
    d = d[(d > 0) & (d < 5.0 * np.median(d[d > 0]))] if np.any(d > 0) else d
    stamp_rate = len(d) / np.sum(d) if len(d) else float('nan')
    span = t_recv[-1] - t_recv[0] if len(t_recv) > 1 else float('nan')
    return stamp_rate, (len(t_recv) - 1) / span if span > 0 else float('nan')


def analyse(path):
    b = read_bag(path, TOPICS)
    truth = truth_arrays(b.get('/evaluation/ground_truth'))
    if truth is None:
        sys.exit(f'{path}: chýba /evaluation/ground_truth')
    r = {'path': path}
    gps_m = b.get('/localization/gps', [])
    vo_m = b.get('/localization/vo', [])
    imu_m = b.get('/localization/imu', [])
    r['frames'] = {
        'gps': gps_m[0][1].header.frame_id if gps_m else '-',
        'vo': f"{vo_m[0][1].header.frame_id} -> {vo_m[0][1].child_frame_id}" if vo_m else '-',
        'imu': imu_m[0][1].header.frame_id if imu_m else '-',
    }
    gps = np.array([[stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                    for _, m in gps_m])
    vo = np.array([[stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
                    yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x, m.twist.twist.linear.y,
                    m.twist.twist.linear.z] for _, m in vo_m])
    imu = np.array([[stamp(m.header), m.angular_velocity.z] for _, m in imu_m])
    r['rates'] = {
        'gps': rates(gps[:, 0], np.array([x[0] for x in gps_m])),
        'vo': rates(vo[:, 0], np.array([x[0] for x in vo_m])),
        'imu': rates(imu[:, 0], np.array([x[0] for x in imu_m])),
        'truth': rates(truth['t'], truth['recv']),
    }
    sim_span = truth['t'][-1] - truth['t'][0]
    wall_span = truth['recv'][-1] - truth['recv'][0]
    r['rtf'] = sim_span / wall_span if wall_span > 0 else float('nan')

    # stationary = on the ground and not moving (truth); the gyro bias is measured there
    speed = np.linalg.norm(truth['v_enu'], axis=1)
    still = (speed < 0.02) & (truth['p'][:, 2] < 0.2)
    r['t0'] = truth['t'][0]

    # GPS noise
    pg, _, _, _ = interp_truth(truth, gps[:, 0])
    eg = gps[:, 1:4] - pg
    ok = np.linalg.norm(eg, axis=1) < 4.0                 # outliers (injected) excluded
    gstill = np.interp(gps[:, 0], truth['t'], still.astype(float)) > 0.5
    sel = ok & gstill if np.sum(ok & gstill) > 10 else ok
    r['gps_err'] = eg
    r['gps_t'] = gps[:, 0]
    r['gps_mean'] = eg[sel].mean(axis=0)
    r['gps_std'] = eg[sel].std(axis=0)
    r['gps_n'] = int(np.sum(sel))

    # VO drift (vo_odom started aligned with the truth when the simulator started)
    pv, yv, vv, _ = interp_truth(truth, vo[:, 0])
    r['vo_t'] = vo[:, 0]
    r['vo_pos_err'] = vo[:, 1:4] - pv
    r['vo_yaw_err'] = wrap(vo[:, 4] - yv)
    c, s = np.cos(yv), np.sin(yv)
    vb_true = np.stack([c * vv[:, 0] + s * vv[:, 1], -s * vv[:, 0] + c * vv[:, 1], vv[:, 2]], axis=1)
    r['vo_vel_err'] = vo[:, 5:8] - vb_true
    T = vo[:, 0] - vo[0, 0]
    r['vo_drift_rate'] = [np.polyfit(T, r['vo_pos_err'][:, i] - r['vo_pos_err'][0, i], 1)[0] for i in range(3)]
    r['vo_yaw_rate'] = np.polyfit(T, r['vo_yaw_err'] - r['vo_yaw_err'][0], 1)[0]

    # gyro: w_z - true yaw rate while the drone is not turning
    _, _, _, wz_true = interp_truth(truth, imu[:, 0])
    istill = np.interp(imu[:, 0], truth['t'], still.astype(float)) > 0.5
    ew = imu[:, 1] - wz_true
    r['imu_t'] = imu[:, 0]
    r['imu_w'] = imu[:, 1]
    r['imu_still'] = istill
    r['bias'] = float(np.mean(ew[istill])) if np.any(istill) else float(np.mean(ew))
    r['gyro_std'] = float(np.std(ew[istill])) if np.any(istill) else float(np.std(ew))
    r['still_s'] = float(np.sum(istill) * np.median(np.diff(imu[:, 0])))
    return r


def report(r, label):
    print(f'=== {label}: {r["path"]}')
    print(f'  frames: GPS "{r["frames"]["gps"]}", VO "{r["frames"]["vo"]}", IMU "{r["frames"]["imu"]}"')
    for k in ('gps', 'vo', 'imu', 'truth'):
        st, rc = r['rates'][k]
        print(f'  {k:5s}: {st:6.1f} Hz from header stamps (sim), {rc:6.1f} Hz arrival (wall)')
    print(f'  real-time factor of this run: {r["rtf"]:.2f} (wall rates = stamp rates x RTF)')
    m, s = r['gps_mean'], r['gps_std']
    print(f'  GPS - truth ({r["gps_n"]} samples): mean ({m[0]:+.2f}, {m[1]:+.2f}, {m[2]:+.2f}) m, '
          f'std ({s[0]:.2f}, {s[1]:.2f}, {s[2]:.2f}) m   [config 0.6, 0.6, 0.9]')
    d = r['vo_pos_err'][-1] - r['vo_pos_err'][0]
    dr = r['vo_drift_rate']
    print(f'  VO drift over {r["vo_t"][-1] - r["vo_t"][0]:.0f} s: ({d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}) m, '
          f'rate ({dr[0] * 1000:+.1f}, {dr[1] * 1000:+.1f}, {dr[2] * 1000:+.1f}) mm/s; '
          f'yaw drift {math.degrees(r["vo_yaw_rate"]) * 60:+.2f} deg/min')
    ve = r['vo_vel_err']
    print(f'  VO body velocity error: mean ({ve[:, 0].mean():+.3f}, {ve[:, 1].mean():+.3f}, {ve[:, 2].mean():+.3f}), '
          f'std ({ve[:, 0].std():.3f}, {ve[:, 1].std():.3f}, {ve[:, 2].std():.3f}) m/s')
    print(f'  gyro Z bias (not turning, {r["still_s"]:.0f} s): {r["bias"]:+.4f} rad/s, noise std {r["gyro_std"]:.4f} rad/s')


def plot(r, label, out, plt):
    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    t0 = r['t0']
    ax = axs[0, 0]
    for i, n in enumerate('xyz'):
        ax.plot(r['gps_t'] - t0, r['gps_err'][:, i], '.', ms=3, label=f'GPS {n} − truth')
    ax.set_ylim(-4, 4)
    ax.set_title('GPS šum (5 Hz)')
    ax.set_ylabel('m')
    ax.legend(fontsize=8)
    ax = axs[0, 1]
    e = r['gps_err'][np.linalg.norm(r['gps_err'], axis=1) < 4.0]
    ax.hist(e[:, 0], bins=30, alpha=0.6, label=f'x σ={e[:, 0].std():.2f}')
    ax.hist(e[:, 1], bins=30, alpha=0.6, label=f'y σ={e[:, 1].std():.2f}')
    ax.hist(e[:, 2], bins=30, alpha=0.6, label=f'z σ={e[:, 2].std():.2f}')
    ax.set_title('Histogram chyby GPS')
    ax.legend(fontsize=8)
    ax = axs[1, 0]
    for i, n in enumerate('xyz'):
        ax.plot(r['vo_t'] - t0, r['vo_pos_err'][:, i], lw=1, label=f'VO {n} − truth')
    ax2 = ax.twinx()
    ax2.plot(r['vo_t'] - t0, np.degrees(r['vo_yaw_err']), 'k--', lw=1, label='VO yaw − truth [°]')
    ax2.set_ylabel('°')
    ax.set_title('Drift VO (vo_odom sa od mapy postupne vzďaľuje)')
    ax.set_ylabel('m')
    ax.set_xlabel('t [s] (sim)')
    ax.legend(fontsize=8, loc='upper left')
    ax2.legend(fontsize=8, loc='lower left')
    ax = axs[1, 1]
    ax.plot(r['imu_t'] - t0, r['imu_w'], lw=0.5, color='0.6', label='IMU w_z')
    ax.plot(r['imu_t'][r['imu_still']] - t0, r['imu_w'][r['imu_still']], lw=0.5, color='C0', label='w_z (neotáča sa)')
    ax.axhline(r['bias'], color='r', lw=1.5, label=f'bias {r["bias"]:+.4f} rad/s')
    ax.axhline(0.0, color='k', lw=0.8)
    ax.set_ylim(-0.1, 0.1)
    ax.set_title('Gyro Z: statický bias')
    ax.set_xlabel('t [s] (sim)')
    ax.legend(fontsize=8)
    for a in axs.flat:
        a.grid(alpha=0.3)
    fig.suptitle(f'A2 §1 charakterizácia senzorov -- {label}')
    fig.tight_layout()
    path = os.path.join(out, f'a2_sensors_{label}.png')
    fig.savefig(path, dpi=110)
    print(f'  --> {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--labels', nargs='*')
    ap.add_argument('--out', default='.')
    ap.add_argument('--no-show', action='store_true')
    args = ap.parse_args()
    labels = args.labels if args.labels and len(args.labels) == len(args.bags) else \
        [os.path.basename(os.path.dirname(os.path.abspath(b.rstrip('/')))) or f'bag{i}' for i, b in enumerate(args.bags)]
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(args.out, exist_ok=True)
    res = []
    for b, lab in zip(args.bags, labels):
        r = analyse(b)
        report(r, lab)
        plot(r, lab, args.out, plt)
        res.append(r)
    if len(res) > 1:
        print('\n=== porovnanie behov (skryté parametre poškodenia)')
        print(f'  {"run":>10} {"gyro bias":>11} {"VO yaw drift":>14} {"VO vel. bias (x, y, z)":>26} {"1. GPS error (x, y, z)":>26}')
        for r, lab in zip(res, labels):
            vb = r['vo_vel_err'].mean(axis=0)
            g = r['gps_err'][0]
            print(f'  {lab:>10} {r["bias"]:+11.4f} {math.degrees(r["vo_yaw_rate"]) * 60:+9.2f}°/min '
                  f'({vb[0]:+.3f}, {vb[1]:+.3f}, {vb[2]:+.3f}) m/s ({g[0]:+.3f}, {g[1]:+.3f}, {g[2]:+.3f}) m')
        for i in range(len(res)):
            for j in range(i + 1, len(res)):
                a, c = res[i]['gps_err'], res[j]['gps_err']
                n = 0
                while n < min(len(a), len(c)) and np.max(np.abs(a[n] - c[n])) < 1e-3:
                    n += 1
                print(f'  {labels[i]} vs {labels[j]}: gyro bias difference {abs(res[i]["bias"] - res[j]["bias"]):.4f} rad/s, '
                      f'identical leading GPS samples: {n}')
        print('  Pozn.: GPS aj VO zdieľajú jeden RNG, ktorý čerpá pri KAŽDEJ správe truth (100 Hz, best effort);\n'
              '  jedna stratená správa pri záťaži posunie celú ďalšiu postupnosť šumu. Skryté parametre\n'
              '  (bias gyra z Gazeba, počiatočné biasy VO) sa pri rovnakom seede zopakujú.')
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
