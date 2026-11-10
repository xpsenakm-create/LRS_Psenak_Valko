#!/usr/bin/env python3
"""
a2_live.py
==========
Živý dashboard A2 počas letu: pôdorys (truth, odhad, GPS, carrot, 3σ), chyba odhadu s ±3σ,
d² GPS voči bráne, stav senzorov a misie, posledné udalosti estimátora.

Je to SAMOSTATNÝ proces iba na pozeranie: nič nepublikuje a estimátoru ani regulátoru nič
neposiela. Ground truth (/evaluation/ground_truth) číta iba kvôli zobrazeniu chyby -- zadanie to
plotovaciemu procesu dovoľuje. Bez truth (--no-truth) ukáže iba odhad a jeho σ.

Použitie (počas behu ./run_assignment2.sh, to ho spúšťa samo):
    python3 tools/a2_live.py [--no-truth] [--window 90]
"""

import argparse
import math
import threading
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String

GATE_DEFAULT = 16.27


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


class LiveData(Node):
    """Collects everything in plain Python buffers; the GUI thread only reads them under a lock."""

    def __init__(self, use_truth, keep_s):
        super().__init__('a2_live', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        n = int(keep_s * 20) + 100
        self.est = deque(maxlen=n)        # (t, x, y, yaw, sx, sy, syaw, bg)
        self.truth = deque(maxlen=n)      # (t, x, y, yaw)
        self.gps = deque(maxlen=int(keep_s * 5) + 50)   # (t, x, y)
        self.gps_inn = deque(maxlen=int(keep_s * 5) + 50)  # (t, d2, gate, status)
        self.rejected = []                # (t, x, y, d2)
        self.carrot = None
        self.carrot_trail = deque(maxlen=n)
        self.state = 'čakám na /mission/state'
        self.events = deque(maxlen=8)
        self.last = {'gps': -1e9, 'vo': -1e9, 'imu': -1e9}
        self.count = {'gps': deque(maxlen=200), 'vo': deque(maxlen=400), 'imu': deque(maxlen=1200)}
        self.t_now = None
        self._dbg_last = -1e9
        self._truth_last = -1e9

        sd = qos_profile_sensor_data
        latched = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64MultiArray, '/estimator/debug', self.on_debug, 50)
        self.create_subscription(Float64MultiArray, '/estimator/innovation', self.on_innovation, 100)
        self.create_subscription(String, '/estimator/events', self.on_event, latched)
        self.create_subscription(String, '/mission/state', self.on_state, latched)
        self.create_subscription(PointStamped, '/mission/carrot', self.on_carrot, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/gps', self.on_gps, sd)
        self.create_subscription(Odometry, '/localization/vo', lambda m: self.seen('vo', stamp(m.header)), sd)
        self.create_subscription(Imu, '/localization/imu', lambda m: self.seen('imu', stamp(m.header)), sd)
        if use_truth:
            self.create_subscription(Odometry, '/evaluation/ground_truth', self.on_truth, sd)

    def seen(self, name, t):
        with self.lock:
            self.last[name] = t
            self.count[name].append(t)
            if self.t_now is None or t > self.t_now:
                self.t_now = t

    def on_debug(self, m):
        d = m.data
        if len(d) < 17 or d[0] - self._dbg_last < 0.05:      # 20 Hz is plenty for the plots
            return
        self._dbg_last = d[0]
        with self.lock:
            # t, x, y, z, vx, vy, vz, yaw, bg, sx, sy, sz, svx, svy, svz, syaw, sbg
            self.est.append((d[0], d[1], d[2], d[7], d[9], d[10], d[15], d[8]))

    def on_truth(self, m):
        t = stamp(m.header)
        if t - self._truth_last < 0.05:
            return
        self._truth_last = t
        p = m.pose.pose.position
        with self.lock:
            self.truth.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def on_gps(self, m):
        t = stamp(m.header)
        self.seen('gps', t)
        p = m.pose.pose.position
        with self.lock:
            self.gps.append((t, p.x, p.y))

    def on_innovation(self, m):
        d = m.data
        if len(d) < 5 or int(d[1]) != 0:   # GPS only
            return
        with self.lock:
            self.gps_inn.append((d[0], d[3], d[4], int(d[2])))
            if int(d[2]) == 2:
                g = min(self.gps, key=lambda s: abs(s[0] - d[0]), default=None)
                if g is not None:
                    self.rejected.append((d[0], g[1], g[2], d[3]))

    def on_event(self, m):
        with self.lock:
            short = m.data.split(' -- ')[0].split('; a valid fix')[0]
            self.events.append(short)

    def on_state(self, m):
        with self.lock:
            self.state = m.data

    def on_carrot(self, m):
        with self.lock:
            self.carrot = (m.point.x, m.point.y)
            self.carrot_trail.append(self.carrot)


def rate(ts, t_now, window=2.0):
    recent = [t for t in ts if t > t_now - window]
    return (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 and recent[-1] > recent[0] else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--no-truth', action='store_true', help='do not subscribe to /evaluation/ground_truth')
    ap.add_argument('--window', type=float, default=120.0, help='seconds of history in the time plots')
    ap.add_argument('--fps', type=float, default=5.0)
    ap.add_argument('--snapshot', help=argparse.SUPPRESS)          # test: save a PNG instead of a window
    ap.add_argument('--snapshot-after', type=float, default=30.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg' if args.snapshot else 'TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    rclpy.init()
    node = LiveData(not args.no_truth, args.window)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    def spin_quietly():
        try:
            ex.spin()
        except ExternalShutdownException:
            pass

    spin = threading.Thread(target=spin_quietly, daemon=True)
    spin.start()

    def stop():
        rclpy.try_shutdown()          # wakes the executor, ex.spin() returns
        spin.join(timeout=2.0)
        node.destroy_node()

    fig = plt.figure('A2 live – estimátor a misia', figsize=(14, 8.5))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.25], height_ratios=[1, 1, 1])
    ax_map = fig.add_subplot(gs[0:2, 0])
    ax_txt = fig.add_subplot(gs[2, 0])
    ax_xy = fig.add_subplot(gs[0, 1])
    ax_yaw = fig.add_subplot(gs[1, 1], sharex=ax_xy)
    ax_d2 = fig.add_subplot(gs[2, 1], sharex=ax_xy)

    # --- map artists
    l_truth, = ax_map.plot([], [], 'k-', lw=1.2, label='truth')
    l_est, = ax_map.plot([], [], 'C0-', lw=1.0, label='EKF odhad')
    l_carrot_trail, = ax_map.plot([], [], '--', color='C1', lw=0.8, label='carrot (príkaz)')
    l_gps, = ax_map.plot([], [], '.', color='0.55', ms=5, label='GPS (posledné 3 s)')
    l_rej, = ax_map.plot([], [], 'rx', ms=12, mew=2.5, label='odmietnutý GPS')
    l_carrot, = ax_map.plot([], [], 'o', color='C1', ms=8, mfc='none', mew=2)
    l_pos, = ax_map.plot([], [], 'o', color='C0', ms=7)
    l_head, = ax_map.plot([], [], '-', color='C0', lw=2.5)
    circ = plt.Circle((0, 0), 0.0, color='C0', alpha=0.15, lw=0)
    ax_map.add_patch(circ)
    ax_map.set_aspect('equal', adjustable='box')
    ax_map.grid(alpha=0.3)
    ax_map.set_xlabel('x ENU [m]')
    ax_map.set_ylabel('y ENU [m]')
    ax_map.set_title('Pôdorys (modrá kružnica = 3σ)')
    ax_map.legend(fontsize=8, loc='upper right')

    # --- time plots
    l_exy, = ax_xy.plot([], [], 'C0', lw=0.9, label='|chyba xy| (odhad − truth)')
    l_sxy, = ax_xy.plot([], [], 'C0--', lw=0.8, alpha=0.7, label='3σ xy')
    ax_xy.set_ylabel('m')
    ax_xy.grid(alpha=0.3)
    ax_xy.legend(fontsize=8, loc='upper left')
    l_eyaw, = ax_yaw.plot([], [], 'C1', lw=0.9, label='chyba yaw')
    l_syaw_p, = ax_yaw.plot([], [], 'C1--', lw=0.8, alpha=0.7, label='±3σ yaw')
    l_syaw_m, = ax_yaw.plot([], [], 'C1--', lw=0.8, alpha=0.7)
    ax_yaw.set_ylabel('°')
    ax_yaw.grid(alpha=0.3)
    ax_yaw.legend(fontsize=8, loc='upper left')
    l_d2, = ax_d2.semilogy([], [], '.', color='C0', ms=3, label='GPS d² = rᵀS⁻¹r')
    l_d2rej, = ax_d2.semilogy([], [], 'rx', ms=9, mew=2, label='odmietnuté')
    l_gate = ax_d2.axhline(GATE_DEFAULT, color='r', ls='--', lw=1, label='brána')
    ax_d2.set_ylabel('d²')
    ax_d2.set_xlabel('čas [s] (sim)')
    ax_d2.set_ylim(0.01, 500)
    ax_d2.grid(alpha=0.3)
    ax_d2.legend(fontsize=8, loc='upper left')
    spans = []

    ax_txt.axis('off')
    txt_head = ax_txt.text(0.0, 1.0, '', fontsize=12, family='monospace', va='top', transform=ax_txt.transAxes)
    txt_gps = ax_txt.text(0.0, 0.80, '', fontsize=12, family='monospace', va='top', weight='bold',
                          transform=ax_txt.transAxes)
    txt_vo = ax_txt.text(0.35, 0.80, '', fontsize=12, family='monospace', va='top', weight='bold',
                         transform=ax_txt.transAxes)
    txt_info = ax_txt.text(0.0, 0.66, '', fontsize=9, family='monospace', va='top', transform=ax_txt.transAxes)
    txt_ev = ax_txt.text(0.0, 0.36, '', fontsize=7, family='monospace', va='top', transform=ax_txt.transAxes)

    outages = {'gps': [], 'vo': []}   # [start, end or None] in sim time

    def update(_):
        with node.lock:
            est = np.array(node.est) if node.est else None
            truth = np.array(node.truth) if node.truth else None
            gps = np.array(node.gps) if node.gps else None
            inn = np.array(node.gps_inn) if node.gps_inn else None
            rej = list(node.rejected)
            carrot, trail = node.carrot, list(node.carrot_trail)
            state, events = node.state, list(node.events)
            last, t_now = dict(node.last), node.t_now
            counts = {k: list(v) for k, v in node.count.items()}
        if est is None or t_now is None:
            txt_head.set_text('čakám na /estimator/debug ...')
            return []
        t0 = est[0, 0]
        rel = lambda t: np.asarray(t) - t0   # noqa: E731
        on = {'gps': t_now - last['gps'] < 1.0, 'vo': t_now - last['vo'] < 0.5}
        for k in outages:
            if not on[k] and (not outages[k] or outages[k][-1][1] is not None):
                outages[k].append([last[k], None])
            if on[k] and outages[k] and outages[k][-1][1] is None:
                outages[k][-1][1] = last[k]

        # map
        l_est.set_data(est[:, 1], est[:, 2])
        x, y, yaw, sxy = est[-1, 1], est[-1, 2], est[-1, 3], max(est[-1, 4], est[-1, 5])
        l_pos.set_data([x], [y])
        l_head.set_data([x, x + 0.6 * math.cos(yaw)], [y, y + 0.6 * math.sin(yaw)])
        circ.center, circ.radius = (x, y), 3 * sxy
        if truth is not None:
            l_truth.set_data(truth[:, 1], truth[:, 2])
        if gps is not None:
            g = gps[gps[:, 0] > t_now - 3.0]
            l_gps.set_data(g[:, 1], g[:, 2])
        if rej:
            r = np.array(rej)
            l_rej.set_data(r[:, 1], r[:, 2])
        if carrot is not None:
            l_carrot.set_data([carrot[0]], [carrot[1]])
            tr = np.array(trail)
            l_carrot_trail.set_data(tr[:, 0], tr[:, 1])
        pts = [est[:, 1:3]] + ([truth[:, 1:3]] if truth is not None else [])
        allp = np.vstack(pts)
        c = 0.5 * (allp.min(axis=0) + allp.max(axis=0))
        half = 0.5 * float(np.max(allp.max(axis=0) - allp.min(axis=0))) + 1.0
        ax_map.set_xlim(c[0] - half, c[0] + half)
        ax_map.set_ylim(c[1] - half, c[1] + half)

        # time plots
        te = est[:, 0]
        sxy_all = 3 * np.maximum(est[:, 4], est[:, 5])
        l_sxy.set_data(rel(te), sxy_all)
        if truth is not None and len(truth) > 2:
            tx = np.interp(te, truth[:, 0], truth[:, 1])
            ty = np.interp(te, truth[:, 0], truth[:, 2])
            exy = np.hypot(est[:, 1] - tx, est[:, 2] - ty)
            tyaw = np.interp(te, truth[:, 0], np.unwrap(truth[:, 3]))
            eyaw = np.degrees(wrap(est[:, 3] - tyaw))
            l_exy.set_data(rel(te), exy)
            l_eyaw.set_data(rel(te), eyaw)
            ax_xy.set_ylim(0, max(1.0, 1.1 * float(np.max(np.r_[exy, sxy_all[-200:]]))))
            ymax = max(5.0, 1.2 * float(np.max(np.abs(eyaw))))
        else:
            exy = eyaw = None
            ax_xy.set_ylim(0, max(1.0, 1.1 * float(np.max(sxy_all[-200:]))))
            ymax = 5.0
        sy = np.degrees(3 * est[:, 6])
        l_syaw_p.set_data(rel(te), sy)
        l_syaw_m.set_data(rel(te), -sy)
        ax_yaw.set_ylim(-ymax, ymax)
        if inn is not None:
            ok = inn[inn[:, 3] != 2]
            bad = inn[inn[:, 3] == 2]
            l_d2.set_data(rel(ok[:, 0]), np.maximum(ok[:, 1], 0.011))
            l_d2rej.set_data(rel(bad[:, 0]), bad[:, 1])
            l_gate.set_ydata([inn[-1, 2]] * 2)
        span = max(30.0, rel(t_now))
        ax_xy.set_xlim(max(0.0, rel(t_now) - args.window), span + 2.0)
        for s in spans:
            s.remove()
        spans.clear()
        for k, color, alpha in (('gps', 'red', 0.12), ('vo', 'orange', 0.25)):
            for a, b in outages[k]:
                for ax in (ax_xy, ax_yaw, ax_d2):
                    spans.append(ax.axvspan(rel(a), rel(b if b is not None else t_now), color=color, alpha=alpha, lw=0))

        # status
        txt_head.set_text(f't = {rel(t_now):6.1f} s   misia: {state}')
        txt_gps.set_text('GPS: ' + ('ON ' if on['gps'] else 'OFF'))
        txt_gps.set_color('green' if on['gps'] else 'red')
        txt_vo.set_text('VO: ' + ('ON ' if on['vo'] else 'OFF'))
        txt_vo.set_color('green' if on['vo'] else 'red')
        info = (f'frekvencie (stampy): GPS {rate(counts["gps"], t_now):4.1f}  VO {rate(counts["vo"], t_now):4.1f}  '
                f'IMU {rate(counts["imu"], t_now):5.1f} Hz\n'
                f'σxy = {sxy:.2f} m   σyaw = {math.degrees(est[-1, 6]):.2f}°   b_g = {est[-1, 7]:+.5f} rad/s')
        if exy is not None:
            info += f'\nchyba teraz: xy {exy[-1]:.2f} m, yaw {eyaw[-1]:+.2f}°'
        txt_info.set_text(info)
        txt_ev.set_text('\n'.join(e[:105] for e in events[-7:]))
        return []

    fig.tight_layout()
    if args.snapshot:
        import time
        end = time.time() + args.snapshot_after
        while time.time() < end:
            update(None)
            time.sleep(1.0 / args.fps)
        update(None)
        fig.savefig(args.snapshot, dpi=90)
        stop()
        return
    _anim = FuncAnimation(fig, update, interval=1000.0 / args.fps, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop()


if __name__ == '__main__':
    main()
