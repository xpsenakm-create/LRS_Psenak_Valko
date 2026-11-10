#!/usr/bin/env python3
"""
a2_plot.py
==========
Grafy a súhrn A2 letu z ROS 2 bagu (offline, samostatný proces -- ground truth sa číta iba tu):

  a2_trajectory.png   pôdorys: truth, odhad, carrot (príkaz), GPS body, VO dráha, outlier
  a2_errors.png       chyby x, y, z, yaw voči truth s pásmom ±3σ z kovariancie P,
                      výpadky GPS/VO vyznačené, stavy misie
  a2_filter.png       GPS d² = rᵀS⁻¹r s bránou (odmietnuté body), odhad biasu gyra,
                      σ polohy, výška

Použitie:
    python3 tools/a2_plot.py results/A2/<beh>/bag [--out results/A2/<beh>] [--no-show]
"""

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a2_bag import interp_truth, read_bag, stamp, truth_arrays, wall_to_sim, wrap  # noqa: E402

TOPICS = ['/evaluation/ground_truth', '/estimator/debug', '/estimator/innovation', '/estimator/odometry',
          '/localization/gps', '/localization/vo', '/mission/state', '/mission/leg', '/mission/carrot',
          '/estimator/events']


def gaps(t, min_gap):
    """Intervals where consecutive stamps are further apart than min_gap."""
    d = np.diff(t)
    return [(t[i], t[i + 1]) for i in np.where(d > min_gap)[0]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', help='output directory (default: parent of the bag)')
    ap.add_argument('--no-show', action='store_true')
    args = ap.parse_args()
    out = args.out or os.path.dirname(os.path.abspath(args.bag.rstrip('/')))
    import matplotlib
    if args.no_show:
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

    p_true, yaw_true, _, _ = interp_truth(truth, te)
    err = np.stack([E['x'], E['y'], E['z']], axis=1) - p_true
    e_yaw = wrap(E['yaw'] - yaw_true)

    gps = np.array([[stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                    for _, m in b.get('/localization/gps', [])])
    vo = np.array([[stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y]
                   for _, m in b.get('/localization/vo', [])])
    inn = np.array([m.data for _, m in b.get('/estimator/innovation', [])])
    carrot = np.array([[m.point.x, m.point.y, m.point.z] for _, m in b.get('/mission/carrot', [])])
    states = [(to_sim(r), m.data) for r, m in b.get('/mission/state', [])]
    gps_gaps = gaps(gps[:, 0], 1.0) if len(gps) else []
    vo_gaps = gaps(vo[:, 0], 0.5) if len(vo) else []

    # time window of interest: from the estimator start to the end of the bag
    t0 = te[0]
    rel = lambda t: np.asarray(t) - t0   # noqa: E731
    route = [t for t, s in states if s == 'ROUTE']
    landed = [t for t, s in states if s in ('DONE', 'ABORTED', 'STOPPED', 'FAULT')]
    flying = (te >= route[0]) & (te <= (landed[0] if landed else te[-1])) if route else np.ones_like(te, bool)

    # ---------------------------------------------------------------- summary
    gps_inn = inn[inn[:, 1] == 0] if len(inn) else np.zeros((0, 8))
    rejected = gps_inn[gps_inn[:, 2] == 2]
    exy = np.hypot(err[:, 0], err[:, 1])
    summary = {'bag': os.path.abspath(args.bag), 'final_state': states[-1][1] if states else None}
    print(f'Bag: {args.bag}')
    print(f'  estimate frame/topic: /estimator/odometry, {len(te)} samples, {rel(te[-1]):.1f} s')
    if route:
        t_end = landed[0] if landed else te[-1]
        print(f'  ROUTE -> {states[-1][1]}: {t_end - route[0]:.1f} s (sim)')
        summary['route_s'] = float(t_end - route[0])
    print(f'  flight RMSE: xy {np.sqrt(np.mean(exy[flying] ** 2)):.3f} m, z {np.sqrt(np.mean(err[flying, 2] ** 2)):.3f} m, '
          f'yaw {np.degrees(np.sqrt(np.mean(e_yaw[flying] ** 2))):.2f} deg; max xy {np.max(exy[flying]):.3f} m')
    inside = np.mean((np.abs(err[:, 0]) <= 3 * E['sx']) & (np.abs(err[:, 1]) <= 3 * E['sy']))
    print(f'  errors inside 3 sigma (x and y): {100 * inside:.1f} %; finite: {bool(np.all(np.isfinite(dbg)))}')
    yaw_inside = np.mean(np.abs(e_yaw[flying]) <= 3 * E['syaw'][flying])
    print(f'  yaw: inside 3 sigma {100 * yaw_inside:.1f} %, max |error| {np.degrees(np.max(np.abs(e_yaw[flying]))):.2f} deg')
    summary.update(rmse_xy=float(np.sqrt(np.mean(exy[flying] ** 2))), rmse_z=float(np.sqrt(np.mean(err[flying, 2] ** 2))),
                   rmse_yaw_deg=float(np.degrees(np.sqrt(np.mean(e_yaw[flying] ** 2)))), max_xy=float(np.max(exy[flying])),
                   inside3_xy=float(inside), inside3_yaw=float(yaw_inside),
                   max_yaw_deg=float(np.degrees(np.max(np.abs(e_yaw[flying])))),
                   finite=bool(np.all(np.isfinite(dbg))), gps_gaps=[], vo_gaps=[], rejected=[])
    for a, c in gps_gaps:
        sel = (te >= a) & (te <= c)
        print(f'  GPS gap {rel(a):.1f}-{rel(c):.1f} s ({c - a:.1f} s): max xy error {np.max(exy[sel]):.3f} m, '
              f'sigma_xy at the end {np.max(E["sx"][sel]):.3f} m')
        summary['gps_gaps'].append({'start': float(rel(a)), 'length': float(c - a), 'max_xy': float(np.max(exy[sel]))})
    for a, c in vo_gaps:
        sel = (te >= a) & (te <= c + 2.0)
        print(f'  VO gap  {rel(a):.1f}-{rel(c):.1f} s ({c - a:.1f} s): max xy error {np.max(exy[sel]):.3f} m')
        summary['vo_gaps'].append({'start': float(rel(a)), 'length': float(c - a), 'max_xy': float(np.max(exy[sel]))})
    for r in rejected:
        print(f'  GPS REJECTED at {rel(r[0]):.2f} s: d2 = {r[3]:.1f} > gate {r[4]:.2f}, |r| = {np.linalg.norm(r[5:8]):.2f} m')
        summary['rejected'].append({'t': float(rel(r[0])), 'd2': float(r[3]), 'r': float(np.linalg.norm(r[5:8]))})
    if len(gps_inn):
        ok = gps_inn[gps_inn[:, 2] == 0]
        print(f'  GPS updates: {len(gps_inn)}, rejected {len(rejected)}, mean d2 of accepted {np.mean(ok[:, 3]):.2f} (ideal 3)')
    print(f'  gyro bias estimate at the end: {E["bg"][-1]:+.4f} +- {E["sbg"][-1]:.4f} rad/s')
    summary['bias_end'] = float(E['bg'][-1])
    if landed and route:
        sel = te >= landed[0]
        if np.any(sel):
            o = p_true[np.argmax(te >= route[0])]
            print(f'  landed (truth) {np.hypot(*(p_true[sel][0, :2] - o[:2])):.2f} m from the route start')
            summary['landed_from_start'] = float(np.hypot(*(p_true[sel][0, :2] - o[:2])))
    with open(os.path.join(out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1)

    # ---------------------------------------------------------------- shading helper
    def shade(ax):
        for a, c in gps_gaps:
            ax.axvspan(rel(a), rel(c), color='red', alpha=0.10, lw=0)
        for a, c in vo_gaps:
            ax.axvspan(rel(a), rel(c), color='orange', alpha=0.18, lw=0)
        for r in rejected:
            ax.axvline(rel(r[0]), color='red', ls=':', lw=1)
        for t, s in states:
            if s in ('TAKEOFF', 'ROUTE', 'LAND'):
                ax.axvline(rel(t), color='gray', ls='--', lw=0.6)
        ax.grid(alpha=0.3)

    # ---------------------------------------------------------------- 1. trajectory
    fig, ax = plt.subplots(figsize=(7, 9))
    if len(gps):
        ax.plot(gps[:, 1], gps[:, 2], '.', color='0.75', ms=3, label='GPS (5 Hz)')
    if len(vo):
        ax.plot(vo[:, 1], vo[:, 2], '-', color='C2', lw=0.8, alpha=0.7, label='VO pose (vo_odom, drifts)')
    ax.plot(truth['p'][:, 0], truth['p'][:, 1], 'k-', lw=1.2, label='truth')
    ax.plot(E['x'], E['y'], 'C0-', lw=1, label='EKF estimate')
    if len(carrot):
        ax.plot(carrot[:, 0], carrot[:, 1], 'C1--', lw=0.8, label='carrot (command)')
    for a, c in gps_gaps:
        sel = (truth['t'] >= a) & (truth['t'] <= c)
        ax.plot(truth['p'][sel, 0], truth['p'][sel, 1], 'r-', lw=3, alpha=0.5)
    for r in rejected:
        g = gps[np.argmin(np.abs(gps[:, 0] - r[0]))]
        ax.plot(g[1], g[2], 'rx', ms=12, mew=2, label='rejected GPS (outlier)')
    ax.plot([], [], 'r-', lw=3, alpha=0.5, label='GPS outage (truth)')
    ax.set_aspect('equal')
    ax.set_xlabel('x ENU [m]')
    ax.set_ylabel('y ENU [m]')
    ax.set_title('A2: pôdorys letu (frame map)')
    ax.legend(fontsize=8, loc='best')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'a2_trajectory.png'), dpi=120)

    # ---------------------------------------------------------------- 2. errors
    fig, axs = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    for ax, (i, name, s) in zip(axs[:3], ((0, 'x', 'sx'), (1, 'y', 'sy'), (2, 'z', 'sz'))):
        ax.plot(rel(te), err[:, i], 'C0', lw=0.8, label=f'{name} odhad − truth')
        ax.fill_between(rel(te), -3 * E[s], 3 * E[s], color='C0', alpha=0.15, label='±3σ (P)')
        ax.set_ylabel(f'{name} chyba [m]')
        ax.set_ylim(-1.5, 1.5)
        shade(ax)
    axs[0].legend(loc='upper right', fontsize=8)
    axs[3].plot(rel(te), np.degrees(e_yaw), 'C1', lw=0.8, label='yaw odhad − truth')
    axs[3].fill_between(rel(te), -3 * np.degrees(E['syaw']), 3 * np.degrees(E['syaw']), color='C1', alpha=0.15)
    axs[3].set_ylabel('yaw chyba [°]')
    axs[3].set_ylim(-10, 10)
    axs[3].set_xlabel('čas od štartu odhadu [s] (sim)')
    shade(axs[3])
    axs[0].set_title('Chyby odhadu (červené = výpadok GPS, oranžové = výpadok VO, bodkovaná = odmietnutý GPS)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'a2_errors.png'), dpi=120)

    # ---------------------------------------------------------------- 3. filter internals
    fig, axs = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    ax = axs[0]
    if len(gps_inn):
        ok = gps_inn[gps_inn[:, 2] != 2]
        ax.semilogy(rel(ok[:, 0]), ok[:, 3], 'C0.', ms=3, label='GPS d² prijaté')
        ax.semilogy(rel(rejected[:, 0]), rejected[:, 3], 'rx', ms=10, mew=2, label='odmietnuté')
        ax.axhline(gps_inn[0, 4], color='r', ls='--', lw=1, label=f'brána {gps_inn[0, 4]:.2f} (χ², 3 DOF, 99.9 %)')
    ax.set_ylabel('d² = rᵀS⁻¹r')
    ax.legend(fontsize=8, loc='upper right')
    shade(ax)
    ax = axs[1]
    ax.plot(rel(te), E['bg'], 'C2', label='odhad biasu gyra b_g')
    ax.fill_between(rel(te), E['bg'] - 3 * E['sbg'], E['bg'] + 3 * E['sbg'], color='C2', alpha=0.15)
    ax.set_ylabel('b_g [rad/s]')
    ax.legend(fontsize=8)
    shade(ax)
    ax = axs[2]
    ax.plot(rel(te), E['sx'], label='σx')
    ax.plot(rel(te), E['sy'], label='σy')
    ax.plot(rel(te), E['sz'], label='σz')
    ax.set_ylabel('σ polohy [m]')
    ax.legend(fontsize=8)
    shade(ax)
    ax = axs[3]
    ax.plot(rel(truth['t']), truth['p'][:, 2], 'k', lw=1, label='z truth')
    ax.plot(rel(te), E['z'], 'C0', lw=0.8, label='z odhad')
    if len(carrot):
        ct = np.array([stamp(m.header) for _, m in b['/mission/carrot']])
        ax.plot(rel(ct), carrot[:, 2], 'C1--', lw=0.8, label='z príkaz (carrot)')
    ax.set_ylabel('z [m]')
    ax.set_xlabel('čas od štartu odhadu [s] (sim)')
    ax.legend(fontsize=8)
    shade(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'a2_filter.png'), dpi=120)
    print(f'--> grafy: {out}/a2_trajectory.png, a2_errors.png, a2_filter.png')
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
