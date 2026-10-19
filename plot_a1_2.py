#!/usr/bin/env python3
"""
plot_a1_2.py -- plots of one A1.2 indoor mission run.

Reads the files the mission executor writes into a run folder (log/a1_2_runs/<time>/):
  flight_log.csv     20 Hz: state, commanded and measured pose (map frame)
  planned_paths.csv  every A* route that was flown
  mission.csv        the mission file of the run (copied by run_a1_2.sh)
plus the hangar map (map.pcd) and the rack boxes / safety radius from config/planner.yaml.

Figures (saved into the run folder):
  topview.png    floor plan: obstacles, racks (+ safety radius), planned routes, flown track,
                 heading arrows, mission waypoints with their tasks
  overview.png   altitude, yaw, distance to target and speed, clearance to obstacles,
                 arrival error per waypoint, deviation from the planned route

Clearance = distance from the drone centre to the nearest map point (floor excluded) or rack
surface. The drone is a collision risk when clearance < drone radius (prop tips, ~0.33 m).

Usage:
    python3 plot_a1_2.py                    # newest run in log/a1_2_runs
    python3 plot_a1_2.py log/a1_2_runs/20261018_101500
    python3 plot_a1_2.py RUN_DIR --no-show  # only save the PNGs
"""

import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(ROOT, "src", "lrs_psenak_valko")
DEFAULT_MAP = os.path.join(PKG, "maps", "FEI_LRS_PCD", "map.pcd")
DEFAULT_YAML = os.path.join(PKG, "config", "planner.yaml")
DEFAULT_RUNS = os.path.join(ROOT, "log", "a1_2_runs")

# Executor defaults (mission_executor_node.cpp), used for tolerance lines.
HARD_RADIUS = 0.15
SOFT_RADIUS = 0.55

STATE_COLORS = {
    "TAKEOFF": "#9ecae1", "RETAKEOFF": "#9ecae1", "ALIGN_PATH": "#fdd0a2", "TASK": "#c7e9c0",
    "LAND": "#fcbba1", "WAIT_LANDED": "#fcbba1", "WAIT_DISARMED": "#fcbba1",
    "GROUND_WAIT": "#d9d9d9", "ERROR": "#ff0000",
}


# ------------------------------------------------------------------------------ loading
def read_csv(path):
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r.get("t_s") or r.get("leg")]
    return rows


def load_flight(path):
    rows = read_csv(path)
    if not rows:
        sys.exit(f"{path} is empty")
    num = ["t_s", "waypoint", "armed", "cmd_valid", "cmd_x", "cmd_y", "cmd_z", "cmd_yaw_deg",
           "x", "y", "z", "yaw_deg", "speed"]
    data = {k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
            for k in num}
    data["state"] = np.array([r["state"] for r in rows])
    data["mode"] = np.array([r["mode"] for r in rows])
    data["waypoint"] = data["waypoint"].astype(int)
    return data


def load_paths(path):
    legs = {}
    if not os.path.exists(path):
        return legs
    for r in read_csv(path):
        legs.setdefault(int(r["leg"]), {"waypoint": int(r["waypoint"]), "pts": []})
        legs[int(r["leg"])]["pts"].append((float(r["x"]), float(r["y"]), float(r["z"])))
    for leg in legs.values():
        leg["pts"] = np.array(leg["pts"])
    return legs


def load_mission(path):
    wps = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = [c.strip() for c in line.split(",")]
            if fields[0] == "x":
                continue
            wps.append({"x": float(fields[0]), "y": float(fields[1]), "z": float(fields[2]),
                        "precision": fields[3], "task": fields[4]})
    return wps


def run_planner_yaml(run_dir, given=None):
    """planner.yaml the run was flown with (saved in the run folder), else the package one."""
    if given:
        return given
    saved = os.path.join(run_dir, "planner.yaml")
    return saved if os.path.exists(saved) else DEFAULT_YAML


def load_planner_yaml(path):
    """Tiny parser for the two values we need (no PyYAML dependency)."""
    text = open(path).read()
    safety = float(re.search(r"safety_radius:\s*([-\d.]+)", text).group(1))
    nums = [float(v) for v in re.findall(r"[-\d.]+", re.search(r"rack_boxes:\s*\[(.*?)\]", text, re.S).group(1))]
    boxes = [nums[i:i + 7] for i in range(0, len(nums) - len(nums) % 7, 7)]
    return safety, boxes


def load_pcd_ascii(path):
    with open(path) as f:
        header = 0
        for line in f:
            header += 1
            if line.startswith("DATA"):
                if "ascii" not in line:
                    sys.exit(f"{path}: only ASCII PCD is supported")
                break
    return np.loadtxt(path, skiprows=header, dtype=np.float32, usecols=(0, 1, 2))


def box_surface_points(box, step=0.1):
    """Points on the surface of a (yawed) box [cx, cy, cz, sx, sy, sz, yaw]."""
    cx, cy, cz, sx, sy, sz, yaw = box
    ax = [np.arange(-s / 2, s / 2 + 1e-9, step) for s in (sx, sy, sz)]
    pts = []
    for dim in range(3):
        for side in (-1, 1):
            grids = np.meshgrid(*[ax[d] if d != dim else np.array([side * (sx, sy, sz)[d] / 2])
                                  for d in range(3)], indexing="ij")
            pts.append(np.stack([g.ravel() for g in grids], axis=1))
    p = np.concatenate(pts)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([cx + c * p[:, 0] - s * p[:, 1], cy + s * p[:, 0] + c * p[:, 1], cz + p[:, 2]], axis=1)


def box_corners_xy(box):
    cx, cy, _, sx, sy, _, yaw = box
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2)]
    return np.array([(cx + c * x - s * y, cy + s * x + c * y) for x, y in pts])


# ------------------------------------------------------------------------------ analysis
def airborne_mask(data):
    return (data["armed"] > 0.5) & (data["z"] > 0.3)


def clearance(data, obstacles):
    """Distance drone centre -> nearest obstacle point (every sample; floor is not an obstacle)."""
    from scipy.spatial import cKDTree
    tree = cKDTree(obstacles)
    pos = np.stack([data["x"], data["y"], data["z"]], axis=1)
    dist, _ = tree.query(pos)
    return dist


def ground_spans(data):
    """(t_start, t_end) of every period on the ground after the first takeoff (landtakeoff, end)."""
    air = airborne_mask(data)
    if not air.any():
        return []
    t = data["t_s"]
    first = np.argmax(air)
    spans, start = [], None
    for k in range(first, len(t)):
        if not air[k] and start is None:
            start = k
        elif air[k] and start is not None:
            spans.append((t[start], t[k]))
            start = None
    if start is not None:
        spans.append((t[start], t[-1]))
    return [sp for sp in spans if sp[1] - sp[0] > 1.0]


def label_ground(ax, data, spans):
    last_t = data["t_s"][-1]
    for t0, t1 in spans:
        text = "landed\n(end)" if t1 >= last_t - 0.1 else "on ground\n(landtakeoff)"
        ax.text((t0 + t1) / 2, 0.97, text, transform=ax.get_xaxis_transform(), ha="center", va="top",
                fontsize=7, color="0.3", bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="0.7", alpha=0.85))


def arrivals(data, mission):
    """Position error when the executor declared each waypoint reached (entry into TASK)."""
    out = []
    for i, wp in enumerate(mission, start=1):
        idx = np.where((data["waypoint"] == i) & (data["state"] == "TASK"))[0]
        if len(idx) == 0:
            out.append(None)
            continue
        k = idx[0]
        err = math.dist((data["x"][k], data["y"][k], data["z"][k]), (wp["x"], wp["y"], wp["z"]))
        out.append({"t": data["t_s"][k], "err": err})
    return out


def route_deviation(data, legs):
    """3D distance from the drone to the planned route of the leg it is flying (NAVIGATE/ALIGN)."""
    by_wp = {}
    for leg in legs.values():
        by_wp[leg["waypoint"]] = leg["pts"]           # the last route planned for that waypoint
    dev = np.full(len(data["t_s"]), np.nan)
    flying = np.isin(data["state"], ["NAVIGATE", "ALIGN_PATH"])
    for k in np.where(flying)[0]:
        pts = by_wp.get(int(data["waypoint"][k]))
        if pts is None or len(pts) < 2:
            continue
        p = np.array([data["x"][k], data["y"][k], data["z"][k]])
        a, b = pts[:-1], pts[1:]
        ab = b - a
        u = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-9), 0, 1)
        dev[k] = np.min(np.linalg.norm(a + ab * u[:, None] - p, axis=1))
    return dev


def break_wraps(deg):
    """NaN between samples that jump across +-180 deg, so lines do not draw vertical spikes."""
    out = deg.copy()
    jump = np.abs(np.diff(out)) > 180
    out[1:][jump] = np.nan
    return out


def shade_states(ax, data):
    t, st = data["t_s"], data["state"]
    start = 0
    for k in range(1, len(st) + 1):
        if k == len(st) or st[k] != st[start]:
            color = STATE_COLORS.get(st[start])
            if color:
                ax.axvspan(t[start], t[min(k, len(t) - 1)], color=color, alpha=0.35, lw=0)
            start = k


# ------------------------------------------------------------------------------ plotting
def plot_topview(ax, data, legs, mission, boxes, safety, obstacles_xy):
    ax.scatter(obstacles_xy[:, 0], obstacles_xy[:, 1], s=0.3, c="0.55", label="map (0.3-4 m)")
    for b in boxes:
        xy = box_corners_xy(b)
        ax.fill(xy[:, 0], xy[:, 1], color="tab:red", alpha=0.25)
        ax.plot(xy[:, 0], xy[:, 1], color="tab:red", lw=1.2)
        grown = box_corners_xy([b[0], b[1], b[2], b[3] + 2 * safety, b[4] + 2 * safety, b[5], b[6]])
        ax.plot(grown[:, 0], grown[:, 1], color="tab:red", lw=0.8, ls="--")
    ax.plot([], [], color="tab:red", ls="--", lw=0.8, label=f"rack + safety radius {safety:.2f} m")
    for n, leg in enumerate(legs.values()):
        p = leg["pts"]
        ax.plot(p[:, 0], p[:, 1], "--", color="tab:purple", lw=1.2, marker=".", ms=5,
                label="planned A* route" if n == 0 else None)
    air = data["armed"] > 0.5
    ax.plot(np.where(air, data["x"], np.nan), np.where(air, data["y"], np.nan), color="tab:blue",
            lw=1.6, label="flown track")
    # heading arrows every ~3 s while airborne
    step = max(1, int(round(3.0 / max(np.median(np.diff(data["t_s"])), 1e-3))))
    k = np.where(air & (data["z"] > 0.3))[0][::step]
    yaw = np.radians(data["yaw_deg"][k])
    ax.quiver(data["x"][k], data["y"][k], np.cos(yaw), np.sin(yaw), color="tab:blue", scale=25,
              width=0.003, alpha=0.8)
    ax.plot(data["x"][0], data["y"][0], "*", color="tab:green", ms=16, label="start")
    for i, wp in enumerate(mission, start=1):
        marker = "s" if wp["precision"] == "hard" else "o"
        ax.plot(wp["x"], wp["y"], marker, ms=10, mfc="gold", mec="k")
        task = "" if wp["task"] == "-" else f" {wp['task']}"
        # stack labels of waypoints that share (almost) the same x, y
        same = sum(1 for p in mission[:i - 1] if math.hypot(p["x"] - wp["x"], p["y"] - wp["y"]) < 0.6)
        ax.annotate(f"{i}{task}\nz={wp['z']:.2f}", (wp["x"], wp["y"]), xytext=(6, 6 - 26 * same),
                    textcoords="offset points", fontsize=8,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8))
    ax.plot([], [], "s", mfc="gold", mec="k", label="hard waypoint")
    ax.plot([], [], "o", mfc="gold", mec="k", label="soft waypoint")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (map)")
    ax.set_ylabel("y [m] (map)")
    ax.set_title("Top view: planned vs flown (arrows = drone heading)")
    ax.grid(True, alpha=0.4)
    ax.legend(loc="upper right", fontsize=7, framealpha=0.9)


def main():
    ap = argparse.ArgumentParser(description="Plot an A1.2 mission run.")
    ap.add_argument("run_dir", nargs="?", help="run folder (default: newest in log/a1_2_runs)")
    ap.add_argument("--map", default=DEFAULT_MAP)
    ap.add_argument("--planner-yaml", help="default: RUN/planner.yaml, else config/planner.yaml")
    ap.add_argument("--drone-radius", type=float, default=0.33, help="centre to prop tip [m]")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--no-save", action="store_true", help="only show, do not overwrite the PNGs")
    args = ap.parse_args()

    run_dir = args.run_dir
    if not run_dir:
        runs = sorted(glob.glob(os.path.join(DEFAULT_RUNS, "*", "flight_log.csv")))
        if not runs:
            sys.exit(f"no runs found in {DEFAULT_RUNS}")
        run_dir = os.path.dirname(runs[-1])
    print(f"Run: {run_dir}")
    args.planner_yaml = run_planner_yaml(run_dir, args.planner_yaml)

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = load_flight(os.path.join(run_dir, "flight_log.csv"))
    legs = load_paths(os.path.join(run_dir, "planned_paths.csv"))
    mission_path = os.path.join(run_dir, "mission.csv")
    mission = load_mission(mission_path) if os.path.exists(mission_path) else []
    safety, boxes = load_planner_yaml(args.planner_yaml)

    cloud = load_pcd_ascii(args.map)
    walls = cloud[cloud[:, 2] > 0.15]                      # floor excluded: altitude is plotted separately
    racks = np.concatenate([box_surface_points(b) for b in boxes]) if boxes else np.empty((0, 3))
    obstacles = np.concatenate([walls, racks])
    obstacles = np.unique(np.round(obstacles / 0.05) * 0.05, axis=0)
    band = cloud[(cloud[:, 2] > 0.3) & (cloud[:, 2] < 4.0)]
    band_xy = np.unique(np.round(band[:, :2] / 0.05) * 0.05, axis=0)

    clear_all = clearance(data, obstacles)
    air = airborne_mask(data)
    clear = np.where(air, clear_all, np.nan)              # only airborne samples count as a risk
    spans = ground_spans(data)
    arr = arrivals(data, mission)
    t = data["t_s"]

    # ---- summary in the terminal
    print(f"Duration {t[-1] - t[0]:.1f} s, final state {data['state'][-1]}")
    for i, (wp, a) in enumerate(zip(mission, arr), start=1):
        tol = HARD_RADIUS if wp["precision"] == "hard" else SOFT_RADIUS
        if a is None:
            print(f"  wp{i} {wp['precision']:4s} {wp['task']:12s} NOT REACHED")
        else:
            print(f"  wp{i} {wp['precision']:4s} {wp['task']:12s} error {a['err']:.3f} m (tol {tol:.2f}) at t={a['t']:.1f} s")
    if np.isfinite(clear).any():
        k = np.nanargmin(clear)
        verdict = "OK" if clear[k] > args.drone_radius else "COLLISION RISK"
        print(f"  min clearance {clear[k]:.2f} m at t={t[k]:.1f} s, pos=({data['x'][k]:.2f}, {data['y'][k]:.2f}, "
              f"{data['z'][k]:.2f}) -> {verdict} (drone radius {args.drone_radius:.2f} m)")

    # ---- top view
    name = os.path.basename(os.path.normpath(run_dir))
    fig, ax = plt.subplots(figsize=(13, 9.5), num=f"A1.2 top view - {name}")
    plot_topview(ax, data, legs, mission, boxes, safety, band_xy)
    fig.tight_layout()
    if not args.no_save:
        fig.savefig(os.path.join(run_dir, "topview.png"), dpi=150)

    # ---- time series
    fig, axs = plt.subplots(3, 2, figsize=(15, 11), num=f"A1.2 overview - {name}")
    cmd = data["cmd_valid"] > 0.5

    ax = axs[0, 0]
    shade_states(ax, data)
    label_ground(ax, data, spans)
    ax.plot(t, data["z"], label="z measured")
    ax.plot(t, np.where(cmd, data["cmd_z"], np.nan), "--", label="z commanded")
    for i, (wp, a) in enumerate(zip(mission, arr), start=1):
        if a:
            ax.plot(a["t"], wp["z"], "s" if wp["precision"] == "hard" else "o", mfc="gold", mec="k")
            ax.annotate(str(i), (a["t"], wp["z"]), xytext=(3, 5), textcoords="offset points", fontsize=8)
    ax.set_ylabel("z [m]")
    ax.set_title("Altitude (shading: blue takeoff, orange align, green task, red landing)")
    ax.legend(fontsize=8)

    ax = axs[0, 1]
    shade_states(ax, data)
    label_ground(ax, data, spans)
    ax.plot(t, break_wraps(data["yaw_deg"]), label="yaw measured")
    ax.plot(t, break_wraps(np.where(cmd, data["cmd_yaw_deg"], np.nan)), "--", label="yaw commanded")
    ax.set_ylabel("yaw [deg] (map/ENU, 0 = +x, CCW +)")
    ax.set_ylim(-185, 185)
    ax.set_yticks(range(-180, 181, 45))
    ax.set_title("Heading")
    ax.legend(fontsize=8)

    ax = axs[1, 0]
    shade_states(ax, data)
    label_ground(ax, data, spans)
    err = np.where(cmd, np.sqrt((data["x"] - data["cmd_x"]) ** 2 + (data["y"] - data["cmd_y"]) ** 2 +
                                (data["z"] - data["cmd_z"]) ** 2), np.nan)
    ax.plot(t, err, label="|position - current target point|")
    ax.plot(t, data["speed"], alpha=0.7, label="speed [m/s]")
    ax.set_ylabel("[m], [m/s]")
    ax.set_title("Distance to the point being flown to, and speed")
    ax.legend(fontsize=8)

    ax = axs[1, 1]
    shade_states(ax, data)
    label_ground(ax, data, spans)
    ax.plot(t, np.where(air, np.nan, clear_all), color="0.6", lw=1, label="on the ground (no risk)")
    ax.plot(t, clear, color="tab:green", label="clearance to nearest obstacle (airborne)")
    ax.axhline(args.drone_radius, color="tab:red", ls="--", label=f"drone radius {args.drone_radius:.2f} m")
    ax.axhline(safety, color="tab:orange", ls=":", label=f"planner safety radius {safety:.2f} m")
    ax.set_ylim(bottom=0)
    ax.set_ylabel("[m]")
    ax.set_title("Clearance (walls, pillars, racks; floor excluded)")
    ax.legend(fontsize=8)

    ax = axs[2, 0]
    labels, values, colors, tols = [], [], [], []
    for i, (wp, a) in enumerate(zip(mission, arr), start=1):
        labels.append(f"{i}\n{wp['precision']}\n{wp['task']}")
        values.append(a["err"] if a else np.nan)
        tol = HARD_RADIUS if wp["precision"] == "hard" else SOFT_RADIUS
        tols.append(tol)
        colors.append("tab:green" if a and a["err"] <= tol else "tab:red")
    x = np.arange(len(labels))
    ax.bar(x, values, color=colors)
    ax.scatter(x, tols, marker="_", s=600, color="k", zorder=3, label="tolerance")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("error [m]")
    ax.set_title("Position error when each waypoint was declared reached")
    ax.legend(fontsize=8)

    ax = axs[2, 1]
    shade_states(ax, data)
    label_ground(ax, data, spans)
    dev = route_deviation(data, legs)
    ax.plot(t, dev, color="tab:purple", label="distance to planned A* route")
    ax.axhline(safety - args.drone_radius, color="tab:red", ls="--",
               label=f"margin left by safety radius ({safety:.2f} - {args.drone_radius:.2f} m)")
    ax.set_ylim(bottom=0)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("[m]")
    ax.set_title("Deviation from the planned route while navigating")
    ax.legend(fontsize=8, loc="center right")
    if np.isfinite(dev).any():
        print(f"  max deviation from planned route {np.nanmax(dev):.2f} m "
              f"(safety radius leaves {safety - args.drone_radius:.2f} m beyond the drone radius)")

    for a in axs[:2].ravel():
        a.set_xlabel("t [s]")
        a.grid(True, alpha=0.4)
    axs[2, 1].grid(True, alpha=0.4)
    fig.suptitle(f"A1.2 mission {name}")
    fig.text(0.5, 0.003, "Commanded lines are blank during takeoff and landing on purpose: ArduCopter flies "
             "NAV_TAKEOFF / LAND itself and the executor sends no position setpoints then "
             "(a setpoint would cancel the takeoff). Route deviation is only defined while navigating.",
             ha="center", va="bottom", fontsize=8, color="0.3")
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    if not args.no_save:
        fig.savefig(os.path.join(run_dir, "overview.png"), dpi=130)
    print(f"Saved {run_dir}/topview.png and overview.png")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
