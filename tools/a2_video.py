#!/usr/bin/env python3
"""
a2_video.py
===========
Video A2 letu z ROS 2 bagu (offline, samostatný proces -- ground truth sa číta iba tu).
Náhrada nahrávky obrazovky na obhajobu: v jednom zábere je vidieť pôdorys (truth, odhad, GPS,
carrot, 3σ kružnica), chyba odhadu s pásmom ±3σ, stav senzorov (GPS / VO ON-OFF), stav misie
a udalosti estimátora (LOST / BACK / REJECTED) presne v čase, keď nastali.

Použitie:
    python3 tools/a2_video.py results/A2/<beh>/bag [--out video.mp4] [--speed 2] [--fps 15]
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a2_bag import interp_truth, read_bag, stamp, truth_arrays, wall_to_sim  # noqa: E402

TOPICS = ['/evaluation/ground_truth', '/estimator/debug', '/estimator/odometry', '/estimator/innovation',
          '/estimator/events', '/localization/gps', '/localization/vo', '/mission/state', '/mission/carrot']


def gaps(t, min_gap):
    d = np.diff(t)
    return [(t[i], t[i + 1]) for i in np.where(d > min_gap)[0]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', help='mp4 file (default: <run>/a2_flight.mp4)')
    ap.add_argument('--speed', type=float, default=2.0, help='simulation seconds per video second')
    ap.add_argument('--fps', type=int, default=15)
    args = ap.parse_args()
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.bag.rstrip('/'))), 'a2_flight.mp4')

    import cv2
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    b = read_bag(args.bag, TOPICS)
    truth = truth_arrays(b.get('/evaluation/ground_truth'))
    if truth is None or not b.get('/estimator/debug'):
        sys.exit('Chyba: bag neobsahuje /evaluation/ground_truth alebo /estimator/debug')
    dbg = np.array([m.data for _, m in b['/estimator/debug']])
    E = dict(zip('t x y z vx vy vz yaw bg sx sy sz svx svy svz syaw sbg'.split(), dbg.T))
    te = E['t']
    odom_recv = np.array([r for r, _ in b['/estimator/odometry']])
    odom_t = np.array([stamp(m.header) for _, m in b['/estimator/odometry']])
    to_sim = wall_to_sim({'recv': odom_recv, 't': odom_t})
    p_true, _, _, _ = interp_truth(truth, te)
    exy = np.hypot(E['x'] - p_true[:, 0], E['y'] - p_true[:, 1])
    sxy = np.maximum(E['sx'], E['sy'])

    gps = np.array([[stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y]
                    for _, m in b.get('/localization/gps', [])])
    vo_t = np.array([stamp(m.header) for _, m in b.get('/localization/vo', [])])
    inn = np.array([m.data for _, m in b.get('/estimator/innovation', [])])
    rejected = inn[(inn[:, 1] == 0) & (inn[:, 2] == 2)] if len(inn) else np.zeros((0, 8))
    carrot = np.array([[stamp(m.header), m.point.x, m.point.y] for _, m in b.get('/mission/carrot', [])])
    states = [(float(to_sim(r)), m.data) for r, m in b.get('/mission/state', [])]
    events = [(float(to_sim(r)), m.data) for r, m in b.get('/estimator/events', [])]
    gps_gaps = gaps(gps[:, 0], 1.0) if len(gps) else []
    vo_gaps = gaps(vo_t, 0.5) if len(vo_t) else []

    t0, t1 = te[0], te[-1]
    rel = lambda t: np.asarray(t) - t0   # noqa: E731
    pad = 1.0
    xs = np.concatenate([truth['p'][:, 0], E['x']])
    ys = np.concatenate([truth['p'][:, 1], E['y']])
    cx, cy = 0.5 * (xs.min() + xs.max()), 0.5 * (ys.min() + ys.max())
    half = 0.5 * max(xs.max() - xs.min(), ys.max() - ys.min()) + pad

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.25], height_ratios=[1.0, 1.0])
    ax_map = fig.add_subplot(gs[:, 0])
    ax_err = fig.add_subplot(gs[0, 1])
    ax_txt = fig.add_subplot(gs[1, 1])
    writer = None
    frames = np.arange(t0, t1, args.speed / args.fps)
    print(f'Renderujem {len(frames)} snímok ({rel(t1):.0f} s simulácie, {args.speed:g}x) -> {out}')
    for k, t in enumerate(frames):
        i = int(np.searchsorted(te, t))
        i = min(max(i, 1), len(te) - 1)
        it = int(np.searchsorted(truth['t'], t))
        gps_on = not any(a < t < c for a, c in gps_gaps)
        vo_on = not any(a < t < c for a, c in vo_gaps)
        state = next((s for ts, s in reversed(states) if ts <= t), 'WAIT_ESTIMATE')

        ax_map.cla()
        ax_map.plot(truth['p'][:it, 0], truth['p'][:it, 1], 'k-', lw=1.2, label='truth')
        ax_map.plot(E['x'][:i], E['y'][:i], 'C0-', lw=1.0, label='EKF odhad')
        g = gps[(gps[:, 0] <= t) & (gps[:, 0] > t - 3.0)] if len(gps) else gps
        if len(g):
            ax_map.plot(g[:, 1], g[:, 2], '.', color='0.6', ms=5, label='GPS (posledné 3 s)')
        for r in rejected[rejected[:, 0] <= t]:
            gg = gps[np.argmin(np.abs(gps[:, 0] - r[0]))]
            ax_map.plot(gg[1], gg[2], 'rx', ms=12, mew=2.5, label=f'odmietnutý GPS (d²={r[3]:.0f})')
        if len(carrot):
            j = int(np.searchsorted(carrot[:, 0], t))
            if j > 0:
                ax_map.plot(carrot[j - 1, 1], carrot[j - 1, 2], 'o', color='C1', ms=7, mfc='none', mew=2,
                            label='carrot (príkaz)')
        ax_map.add_patch(plt.Circle((E['x'][i], E['y'][i]), 3 * sxy[i], color='C0', alpha=0.15, lw=0))
        ax_map.plot(E['x'][i], E['y'][i], 'o', color='C0', ms=7)
        yaw = E['yaw'][i]
        ax_map.arrow(E['x'][i], E['y'][i], 0.6 * np.cos(yaw), 0.6 * np.sin(yaw), color='C0', width=0.03)
        ax_map.set_xlim(cx - half, cx + half)
        ax_map.set_ylim(cy - half, cy + half)
        ax_map.set_aspect('equal')
        ax_map.grid(alpha=0.3)
        ax_map.set_xlabel('x ENU [m]')
        ax_map.set_ylabel('y ENU [m]')
        ax_map.set_title('Pôdorys (modrá kružnica = 3σ odhadu)')
        h, l = ax_map.get_legend_handles_labels()
        uniq = dict(zip(l, h))
        ax_map.legend(uniq.values(), uniq.keys(), fontsize=8, loc='upper right')

        ax_err.cla()
        ax_err.plot(rel(te[:i]), exy[:i], 'C0', lw=0.9, label='|chyba xy| odhad − truth')
        ax_err.fill_between(rel(te[:i]), 0, 3 * sxy[:i], color='C0', alpha=0.15, label='3σ')
        for a, c in gps_gaps:
            if a < t:
                ax_err.axvspan(rel(a), rel(min(c, t)), color='red', alpha=0.12, lw=0)
        for a, c in vo_gaps:
            if a < t:
                ax_err.axvspan(rel(a), rel(min(c, t)), color='orange', alpha=0.25, lw=0)
        ax_err.set_xlim(0, rel(t1))
        ax_err.set_ylim(0, max(1.2, 1.1 * float(np.max(exy))))
        ax_err.set_ylabel('m')
        ax_err.set_xlabel('čas [s] (sim)   červené = GPS vypnuté, oranžové = VO vypnuté')
        ax_err.legend(fontsize=8, loc='upper left')
        ax_err.grid(alpha=0.3)

        ax_txt.cla()
        ax_txt.axis('off')
        ax_txt.text(0.0, 0.95, f't = {rel(t):5.1f} s    misia: {state}', fontsize=13, family='monospace',
                    transform=ax_txt.transAxes, va='top')
        ax_txt.text(0.0, 0.80, 'GPS: ' + ('ON ' if gps_on else 'OFF'), fontsize=13, family='monospace',
                    color='green' if gps_on else 'red', transform=ax_txt.transAxes, va='top', weight='bold')
        ax_txt.text(0.3, 0.80, 'VO: ' + ('ON ' if vo_on else 'OFF'), fontsize=13, family='monospace',
                    color='green' if vo_on else 'red', transform=ax_txt.transAxes, va='top', weight='bold')
        ax_txt.text(0.0, 0.68, f'σxy = {sxy[i]:.2f} m   b_g = {E["bg"][i]:+.4f} rad/s   '
                    f'chyba xy = {exy[i]:.2f} m', fontsize=10, family='monospace', transform=ax_txt.transAxes,
                    va='top')
        recent = [e for e in events if e[0] <= t][-6:]
        for n, (te_, msg) in enumerate(recent):
            color = 'red' if ('REJECTED' in msg or 'LOST' in msg) else 'black'
            short = msg.split(' -- ')[0].split('; a valid fix')[0]
            ax_txt.text(0.0, 0.52 - 0.085 * n, f'[{rel(te_):5.1f}] {short[:100]}', fontsize=7.5, family='monospace',
                        color=color, transform=ax_txt.transAxes, va='top')
        fig.suptitle('A2: 8-stavový EKF (GPS + VO + gyro) – let riadený iba odhadom', fontsize=12)
        fig.tight_layout()

        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        if writer is None:
            writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (img.shape[1], img.shape[0]))
            if not writer.isOpened():
                sys.exit('Chyba: OpenCV nevie zapisovať mp4')
        writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if k % 100 == 0:
            print(f'  {k}/{len(frames)}')
    if writer is not None:
        writer.release()
    print(f'--> video: {out}')


if __name__ == '__main__':
    main()
