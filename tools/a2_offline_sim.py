#!/usr/bin/env python3
"""
a2_offline_sim.py
=================
Offline test of the A2 estimator WITHOUT Gazebo / ArduPilot.

1. Generates a synthetic flight: wait on the ground, take off to 2 m, two laps of the A2 route
   (0,0) -> (0,-4) -> (2.5,-4) -> (2.5,0) -> (0,0) at 0.5 m/s, land.
2. Corrupts it with the TEACHER'S SensorCorruptor (localization_sensor_sim/sensor_model.py) and the
   same 5 Hz / 30 Hz scheduling as sensor_simulator.py; the gyro gets a seeded +-0.02 rad/s bias and
   0.004 rad/s noise like the Gazebo IMU in fei_lrs_gazebo_localization.world.
3. Shuffles the events by a random transport delay (so GPS/VO/IMU arrive slightly out of order).
4. Replays them through ekf8_replay -- the estimator's own C++ code -- and compares with the truth.

Default scenario (like the defence): 15 s GPS outage from the start of lap 1 leg 3 (a middle leg),
one GPS outlier in lap 2 leg 2, a 3 s VO loss in lap 2 leg 3.

Usage:
    python3 tools/a2_offline_sim.py --replay build/lrs_psenak_valko/ekf8_replay --seeds 1-20
    python3 tools/a2_offline_sim.py --replay ... --seeds 42 --plot offline_42.png
    python3 tools/a2_offline_sim.py --replay ... --seeds 1-20 accel_xy=0.5 vo_yaw_std=0.05
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

ROUTE = [(0.0, -4.0), (2.5, -4.0), (2.5, 0.0), (0.0, 0.0)]
P0 = np.array([-7.0, 13.0, 0.0])
YAW0 = math.pi / 2.0
DT = 0.01


def wrap(a):
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def default_sensor_sim_dir():
    """LRS-URK/localization_sensor_sim; LRS_URK_DIR may also point at LRS-URK/scripts (as in run_a1_3.sh)."""
    candidates = []
    env = os.environ.get('LRS_URK_DIR', '').rstrip('/')
    if env:
        candidates += [env, env[:-len('/scripts')] if env.endswith('/scripts') else env]
    candidates += [os.path.expanduser('~/LRS-URK'), os.path.expanduser('~/Desktop/LRS/LRS-URK')]
    for c in candidates:
        d = os.path.join(c, 'localization_sensor_sim')
        if os.path.isfile(os.path.join(d, 'localization_sensor_sim', 'sensor_model.py')):
            return d
    return None


def load_sensor_model(sensor_sim_dir):
    """Import the teacher's sensor_model.py and read sensors.yaml (public defaults)."""
    import yaml
    if not sensor_sim_dir or not os.path.isfile(os.path.join(sensor_sim_dir, 'localization_sensor_sim', 'sensor_model.py')):
        sys.exit('Chyba: nenašiel som localization_sensor_sim (zadaj --sensor-sim-dir /cesta/k/LRS-URK/localization_sensor_sim)')
    sys.path.insert(0, sensor_sim_dir)
    sys.dont_write_bytecode = True          # leave no __pycache__ in the teacher's repository
    from localization_sensor_sim import sensor_model  # noqa: E402
    with open(os.path.join(sensor_sim_dir, 'config', 'sensors.yaml'), encoding='utf-8') as f:
        params = yaml.safe_load(f)['sensor_simulator']['ros__parameters']
    return sensor_model, params


# ----------------------------------------------------------------------------- truth trajectory
def make_truth(rng, laps=2, speed=0.5, accel=0.5):
    """Piecewise trajectory sampled at 100 Hz.  Returns t, pos (n,3), yaw (n), leg (n)."""
    t, pos, leg = [0.0], [P0.copy()], [-1]

    def hold(seconds, leg_id):
        for _ in range(int(round(seconds / DT))):
            t.append(t[-1] + DT)
            pos.append(pos[-1].copy())
            leg.append(leg_id)

    def move(target, v_max, a_max, leg_id):
        start = pos[-1].copy()
        d = np.linalg.norm(target - start)
        u = (target - start) / d
        s, v = 0.0, 0.0
        while s < d - 1e-6:
            v = min(v_max, v + a_max * DT, math.sqrt(2.0 * a_max * max(d - s, 0.0)) + 1e-3)
            s = min(d, s + v * DT)
            t.append(t[-1] + DT)
            pos.append(start + u * s)
            leg.append(leg_id)

    hold(10.0, -1)                                            # on the ground, estimator converges
    move(P0 + [0, 0, 2.0], 1.0, 1.0, -1)                      # guided takeoff
    hold(3.0, -1)
    i = 0
    for _ in range(laps):
        for (dx, dy) in ROUTE:
            move(P0 + [dx, dy, 2.0], speed, accel, i)
            hold(0.8, i)                                      # arrival tolerance / settling
            i += 1
    hold(2.0, i)
    move(P0.copy(), 0.5, 0.5, i)                              # LAND
    hold(3.0, i)

    t = np.array(t)
    pos = np.array(pos)
    airborne = pos[:, 2] > 0.05

    def wobble(amplitude):
        """Slow position-hold error (sum of 3 sinusoids, periods 3-10 s), like a real multicopter."""
        w = sum(np.sin(2.0 * math.pi * t / rng.uniform(3.0, 10.0) + rng.uniform(0.0, 2.0 * math.pi))
                for _ in range(3))
        return amplitude / math.sqrt(1.5) * w * airborne

    pos[:, 0] += wobble(0.04)
    pos[:, 1] += wobble(0.04)
    pos[:, 2] += wobble(0.03)
    yaw = YAW0 + wobble(math.radians(1.0))
    return t, pos, wrap(yaw), np.array(leg)


# ----------------------------------------------------------------------------- sensors
def make_events(seed, sm, params, t, pos, yaw, windows, rng):
    """List of (arrival, kind, stamp, values...) like the ROS graph would deliver them."""
    cfg = {k: params[k] for k in (
        'gps_position_stddev', 'gps_outlier_min_m', 'gps_outlier_max_m', 'vo_scale_stddev',
        'vo_initial_velocity_bias_stddev', 'vo_velocity_bias_random_walk_stddev',
        'vo_velocity_noise_stddev', 'vo_position_step_noise_stddev', 'vo_initial_yaw_rate_bias_stddev',
        'vo_yaw_bias_random_walk_stddev', 'vo_yaw_step_noise_stddev', 'vo_pose_position_noise_stddev',
        'vo_pose_yaw_noise_stddev')}
    corruptor = sm.SensorCorruptor(seed, cfg)
    gps_period, vo_period = 1.0 / params['gps_rate_hz'], 1.0 / params['vo_rate_hz']

    def off(name, stamp):
        a, b = windows.get(name, (None, None))
        return a is not None and a <= stamp < b

    events = []
    next_gps = next_vo = None
    outlier_pending, outlier_time = False, windows.get('outlier', (None,))[0]
    outlier_stamp = None
    for k in range(len(t)):
        stamp = float(t[k])
        corruptor.advance(pos[k], float(yaw[k]), stamp)
        if outlier_time is not None and not outlier_pending and outlier_stamp is None and stamp >= outlier_time:
            outlier_pending = True
        due, next_gps = sm.advance_schedule(next_gps, gps_period, stamp)
        if due and not off('gps', stamp):
            z = corruptor.gps_measurement(pos[k], outlier=outlier_pending)
            if outlier_pending:
                outlier_pending, outlier_stamp = False, stamp
            events.append((stamp + rng.uniform(0.002, 0.010), 'gps', stamp, *z))
        due, next_vo = sm.advance_schedule(next_vo, vo_period, stamp)
        if due and not off('vo', stamp):
            s = corruptor.noisy_vo_sample()
            events.append((stamp + rng.uniform(0.002, 0.010), 'vo', stamp, s.yaw, *s.body_velocity))

    # Gazebo IMU: own 100 Hz clock (half a period after the truth), static seeded bias + noise
    imu_rng = np.random.default_rng(seed + 100003)
    bias = imu_rng.choice([-1.0, 1.0]) * imu_rng.normal(0.02, 0.004)
    yaw_u = np.unwrap(yaw)
    rate = np.gradient(yaw_u, t)
    t_imu = t[:-1] + 0.005
    w_true = np.interp(t_imu, t, rate)
    w = w_true + bias + imu_rng.normal(0.0, 0.004, len(t_imu))
    for ti, wi in zip(t_imu, w):
        events.append((ti + rng.uniform(0.0005, 0.004), 'imu', ti, wi))
    events.sort(key=lambda e: e[0])
    return events, bias, outlier_stamp


def write_events(path, events):
    with open(path, 'w', encoding='utf-8') as f:
        for e in events:
            f.write(e[1] + ',' + ','.join(f'{v:.9f}' for v in e[2:]) + '\n')


def read_csv(path):
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]} if rows else {}


# ----------------------------------------------------------------------------- evaluation
def evaluate(t, pos, yaw, est, inn, bias, windows, outlier_stamp):
    te = est['t']
    p_true = np.stack([np.interp(te, t, pos[:, i]) for i in range(3)], axis=1)
    yaw_true = np.interp(te, t, np.unwrap(yaw))
    e = np.stack([est['x'], est['y'], est['z']], axis=1) - p_true
    e_yaw = wrap(est['yaw'] - yaw_true)
    air = p_true[:, 2] > 0.3
    exy = np.hypot(e[:, 0], e[:, 1])
    g0, g1 = windows.get('gps', (None, None))
    in_out = (te >= g0) & (te < g1) if g0 is not None else np.zeros_like(te, bool)
    after = (te >= g1) & (te < g1 + 5.0) if g0 is not None else np.zeros_like(te, bool)
    v0, v1 = windows.get('vo', (None, None))
    in_vo = (te >= v0) & (te < v1 + 2.0) if v0 is not None else np.zeros_like(te, bool)

    gps = inn['sensor'] == 0
    status = inn['status']
    rej = gps & (status == 2)
    outlier_rejected, outlier_d2 = None, float('nan')
    if outlier_stamp is not None:
        idx = np.where(gps & (np.abs(inn['t'] - outlier_stamp) < 1e-6))[0]
        if len(idx):
            outlier_rejected = bool(status[idx[0]] == 2)
            outlier_d2 = inn['d2'][idx[0]]
    normal_rej = int(np.sum(rej)) - (1 if outlier_rejected else 0)
    nis = inn['d2'][gps & (status == 0)]

    # largest non-physical step of the estimate (what the controller would feel as a jump)
    dp = np.diff(np.stack([est['x'], est['y'], est['z']], axis=1), axis=0)
    dtrue = np.diff(p_true, axis=0)
    jump = np.linalg.norm(dp - dtrue, axis=1)
    finite = all(np.all(np.isfinite(est[k])) for k in est)
    inside3 = np.mean((np.abs(e[:, 0]) <= 3 * est['sx']) & (np.abs(e[:, 1]) <= 3 * est['sy']))
    return {
        'finite': finite,
        'rmse_xy': float(np.sqrt(np.mean(exy[air] ** 2))),
        'rmse_z': float(np.sqrt(np.mean(e[air, 2] ** 2))),
        'rmse_yaw_deg': float(np.degrees(np.sqrt(np.mean(e_yaw[air] ** 2)))),
        'max_xy': float(np.max(exy[air])),
        'outage_max_xy': float(np.max(exy[in_out])) if np.any(in_out) else float('nan'),
        'outage_end_sxy': float(np.max(est['sx'][in_out])) if np.any(in_out) else float('nan'),
        'recovery_jump': float(np.max(jump[after[1:]])) if np.any(after) else float('nan'),
        'vo_outage_max_xy': float(np.max(exy[in_vo])) if np.any(in_vo) else float('nan'),
        'max_jump': float(np.max(jump)),
        'step_rms_mm': float(1000.0 * np.sqrt(np.mean(jump[air[1:]] ** 2))),
        'bias_err': float(est['bias'][-1] - bias),
        'outlier_rejected': outlier_rejected,
        'outlier_d2': float(outlier_d2),
        'false_rejections': normal_rej,
        'gps_nis_mean': float(np.mean(nis)) if len(nis) else float('nan'),
        'inside_3sigma': float(inside3),
    }


def plot(path, t, pos, yaw, est, inn, bias, windows, seed):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    te = est['t']
    p_true = np.stack([np.interp(te, t, pos[:, i]) for i in range(3)], axis=1)
    yaw_true = np.interp(te, t, np.unwrap(yaw))
    fig, ax = plt.subplots(3, 2, figsize=(14, 11))
    a = ax[0, 0]
    a.plot(pos[:, 0], pos[:, 1], 'k-', lw=1, label='truth')
    a.plot(est['x'], est['y'], 'C0-', lw=1, label='estimate')
    a.set_aspect('equal'); a.set_title(f'xy (seed {seed})'); a.legend(); a.grid(alpha=0.3)
    for row, (name, i, s) in enumerate((('x', 0, 'sx'), ('y', 1, 'sy'), ('z', 2, 'sz'))):
        a = ax[row, 1] if row < 2 else ax[1, 0]
        err = est[name] - p_true[:, i]
        a.plot(te, err, 'C0', lw=0.8, label=f'{name} error')
        a.fill_between(te, -3 * est[s], 3 * est[s], color='C0', alpha=0.15, label='±3σ')
        a.set_ylabel('m'); a.set_title(f'{name} error'); a.grid(alpha=0.3)
    a = ax[2, 0]
    a.plot(te, np.degrees(wrap(est['yaw'] - yaw_true)), 'C1', lw=0.8, label='yaw error')
    a.fill_between(te, -3 * np.degrees(est['syaw']), 3 * np.degrees(est['syaw']), color='C1', alpha=0.15)
    a2 = a.twinx()
    a2.plot(te, est['bias'], 'C2', lw=1, label='bias estimate')
    a2.axhline(bias, color='C2', ls='--', lw=1, label='true bias')
    a.set_ylabel('yaw error [deg]'); a2.set_ylabel('bias [rad/s]'); a.set_title('yaw / gyro bias')
    a2.legend(loc='lower right'); a.grid(alpha=0.3)
    a = ax[2, 1]
    g = inn['sensor'] == 0
    ok, rej = g & (inn['status'] != 2), g & (inn['status'] == 2)
    a.semilogy(inn['t'][ok], inn['d2'][ok], 'C0.', ms=3, label='GPS d² accepted')
    a.semilogy(inn['t'][rej], inn['d2'][rej], 'rx', ms=8, label='rejected')
    a.axhline(16.27, color='r', ls='--', lw=1, label='gate')
    a.set_title('GPS innovation d² = rᵀS⁻¹r'); a.legend(); a.grid(alpha=0.3)
    for a in ax.flat[1:]:
        for name, color in (('gps', 'red'), ('vo', 'orange')):
            if windows.get(name, (None,))[0] is not None:
                a.axvspan(*windows[name], color=color, alpha=0.08)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'--> plot: {path}')


def parse_seeds(text):
    seeds = []
    for part in text.split(','):
        if '-' in part:
            a, b = part.split('-')
            seeds += list(range(int(a), int(b) + 1))
        else:
            seeds.append(int(part))
    return seeds


def main():
    default_sim = default_sensor_sim_dir()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--replay', required=True, help='path to the ekf8_replay binary')
    ap.add_argument('--sensor-sim-dir', default=default_sim, help='localization_sensor_sim package directory')
    ap.add_argument('--seeds', default='42')
    ap.add_argument('--no-outage', action='store_true', help='no GPS/VO outage and no outlier')
    ap.add_argument('--outlier-after-outage', action='store_true',
                    help='inject the outlier into the first GPS sample after the outage (worst case)')
    ap.add_argument('--plot', help='PNG file (only the first seed is plotted)')
    ap.add_argument('--cache', help='directory to cache generated sensor events per seed (faster tuning)')
    ap.add_argument('--summary', action='store_true', help='print only the summary over all seeds')
    ap.add_argument('--keep', help='directory to keep events/estimate CSVs of the first seed')
    ap.add_argument('overrides', nargs='*', help='estimator overrides key=value (see ekf8_replay.cpp)')
    args = ap.parse_args()

    sm, params = load_sensor_model(args.sensor_sim_dir)
    results = []
    for n, seed in enumerate(parse_seeds(args.seeds)):
        rng = np.random.default_rng(seed + 7)
        t, pos, yaw, leg = make_truth(rng)
        windows = {}
        scenario = 'clean' if args.no_outage else ('late' if args.outlier_after_outage else 'std')
        if not args.no_outage:
            t_leg = {i: float(t[np.argmax(leg == i)]) for i in range(8)}
            windows['gps'] = (t_leg[2], t_leg[2] + params['required_gps_outage_s'])
            windows['vo'] = (t_leg[6], t_leg[6] + params['suggested_vo_outage_s'])
            windows['outlier'] = ((windows['gps'][1] if args.outlier_after_outage else t_leg[5] + 1.0),)
        cache = os.path.join(args.cache, f'seed{seed}_{scenario}') if args.cache else None
        if cache and os.path.isfile(os.path.join(cache, 'meta.npz')):
            meta = np.load(os.path.join(cache, 'meta.npz'))
            bias = float(meta['bias'])
            outlier_stamp = float(meta['outlier_stamp']) if np.isfinite(meta['outlier_stamp']) else None
        else:
            events, bias, outlier_stamp = make_events(seed, sm, params, t, pos, yaw, windows, rng)
            if cache:
                os.makedirs(cache, exist_ok=True)
                write_events(os.path.join(cache, 'events.csv'), events)
                np.savez(os.path.join(cache, 'meta.npz'), bias=bias,
                         outlier_stamp=np.nan if outlier_stamp is None else outlier_stamp)
        with tempfile.TemporaryDirectory() as tmp:
            out = args.keep if (args.keep and n == 0) else tmp
            os.makedirs(out, exist_ok=True)
            events_csv = os.path.join(cache, 'events.csv') if cache else os.path.join(out, 'events.csv')
            if not cache:
                write_events(events_csv, events)
            subprocess.run([args.replay, events_csv, out, *args.overrides],
                           check=True, stdout=subprocess.DEVNULL)
            est = read_csv(os.path.join(out, 'estimate.csv'))
            inn = read_csv(os.path.join(out, 'innovation.csv'))
        r = evaluate(t, pos, yaw, est, inn, bias, windows, outlier_stamp)
        r['seed'], r['bias'] = seed, bias
        results.append(r)
        if args.plot and n == 0:
            plot(args.plot, t, pos, yaw, est, inn, bias, windows, seed)

    cols = ['seed', 'rmse_xy', 'rmse_z', 'rmse_yaw_deg', 'max_xy', 'outage_max_xy', 'outage_end_sxy',
            'vo_outage_max_xy', 'recovery_jump', 'step_rms_mm', 'bias', 'bias_err', 'outlier_d2',
            'outlier_rejected', 'false_rejections', 'gps_nis_mean', 'inside_3sigma', 'finite']
    if not args.summary:
        print(' '.join(f'{c[:13]:>13}' for c in cols))
        for r in results:
            print(' '.join(f'{r[c]:>13.4f}' if isinstance(r[c], float) else f'{str(r[c]):>13}' for c in cols))
    ok = [r for r in results]
    print('mean over seeds: rmse_xy=%.3f rmse_z=%.3f rmse_yaw=%.2f deg step_rms=%.2f mm inside3σ=%.2f outage_end_σ=%.2f'
          % tuple(np.nanmean([r[c] for r in ok]) for c in
                  ('rmse_xy', 'rmse_z', 'rmse_yaw_deg', 'step_rms_mm', 'inside_3sigma', 'outage_end_sxy')))
    print('worst vo_outage_max_xy=%.3f m' % np.nanmax([r['vo_outage_max_xy'] for r in ok]))
    print('\nworst over seeds: max_xy=%.3f m, outage_max_xy=%.3f m, recovery_jump=%.3f m, |bias_err|=%.4f rad/s'
          % (max(r['max_xy'] for r in ok), np.nanmax([r['outage_max_xy'] for r in ok]),
             np.nanmax([r['recovery_jump'] for r in ok]), max(abs(r['bias_err']) for r in ok)))
    print('outliers rejected: %d/%d, false GPS rejections total: %d, all finite: %s, mean NIS %.2f (ideal 3)'
          % (sum(1 for r in ok if r['outlier_rejected']), sum(1 for r in ok if r['outlier_rejected'] is not None),
             sum(r['false_rejections'] for r in ok), all(r['finite'] for r in ok),
             np.nanmean([r['gps_nis_mean'] for r in ok])))


if __name__ == '__main__':
    main()
