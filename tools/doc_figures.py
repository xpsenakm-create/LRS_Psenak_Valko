#!/usr/bin/env python3
"""
doc_figures.py -- result figures for docs/A1_1 and docs/A1_2 (from recorded data only).

A1.1 needs docs/A1_1/data/benchmark.json and benchmark_raw.json (tools/a1_1_benchmark.py) and the
inflated occupancy saved by that run (/tmp/a1_1_occupancy_<r>.pcd, optional), where r is
safety_radius from config/planner.yaml.
A1.2 uses the recording in results/A1_2/ flown with that safety_radius.

    python3 tools/doc_figures.py
"""

import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
import plot_a1_2 as P  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

A11 = os.path.join(ROOT, "docs", "A1_1")
A12 = os.path.join(ROOT, "docs", "A1_2")
BLUE, GREY, RED, GREEN, ORANGE, PURPLE = "#2a78d6", "#9a9a96", "#eb6834", "#2a9d5c", "#e8a33a", "#7a4fd0"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False})


def map_band(cloud, lo=0.3, hi=4.0):
    band = cloud[(cloud[:, 2] > lo) & (cloud[:, 2] < hi)]
    return np.unique(np.round(band[:, :2] / 0.05) * 0.05, axis=0)


def draw_floor(ax, band, boxes, safety=None):
    ax.scatter(band[:, 0], band[:, 1], s=0.2, c=GREY)
    for b in boxes:
        xy = P.box_corners_xy(b)
        ax.fill(xy[:, 0], xy[:, 1], color=RED, alpha=0.18)
        ax.plot(xy[:, 0], xy[:, 1], color=RED, lw=1)
        if safety:
            g = P.box_corners_xy([b[0], b[1], b[2], b[3] + 2 * safety, b[4] + 2 * safety, b[5], b[6]])
            ax.plot(g[:, 0], g[:, 1], color=RED, lw=0.7, ls="--")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)


def route(data, radius, name):
    for r in data:
        if abs(r["radius"] - radius) < 1e-6:
            for x in r["routes"]:
                if x["name"] == name:
                    return x
    return None


# ------------------------------------------------------------------ A1.1
def a1_1(cloud, boxes, safety):
    bench = json.load(open(os.path.join(A11, "data", "benchmark.json"), encoding="utf-8"))
    raw = json.load(open(os.path.join(A11, "data", "benchmark_raw.json"), encoding="utf-8"))
    band = map_band(cloud)
    img = os.path.join(A11, "img")

    # 1) safety radius changes the route over the rack (side view x-z and top view)
    name = "cez regál (stúpanie)"
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={"width_ratios": [1.15, 1]})
    draw_floor(axs[0], band, boxes)
    colors = {0.30: ORANGE, 0.45: BLUE, 0.75: PURPLE}
    rack = boxes[0]
    x0, x1 = rack[0] - rack[3] / 2, rack[0] + rack[3] / 2
    top = rack[2] + rack[5] / 2
    axs[1].fill_between([x0, x1], 0, top, color=RED, alpha=0.18, label="regál (6.5 × 1.05 × 3.76 m)")
    for radius, c in colors.items():
        r = route(bench, radius, name)
        p = np.array(r["path"])
        lab = f"r = {radius:.2f} m: {r['raw_points']} → {r['points']} bodov, {r['length_m']:.2f} m"
        axs[0].plot(p[:, 0], p[:, 1], "-o", color=c, lw=2, ms=4, label=lab)
        axs[1].plot(p[:, 0], p[:, 2], "-o", color=c, lw=2, ms=4, label=f"r = {radius:.2f} m")
        axs[1].plot([x0 - radius, x0 - radius, x1 + radius, x1 + radius], [0, top + radius, top + radius, 0],
                    ls="--", color=c, lw=0.9)
    s, g = np.array(r["start"]), np.array(r["goal"])
    for ax, k in ((axs[0], 1), (axs[1], 2)):
        ax.plot(s[0], s[k], "*", color=GREEN, ms=14)
        ax.plot(g[0], g[k], "s", color="gold", mec="k", ms=9)
    axs[0].set_title("Pôdorys")
    axs[0].legend(fontsize=8, loc="lower left")
    axs[0].set_xlim(0, 15)
    axs[0].set_ylim(1, 13)
    axs[1].set_title("Bočný pohľad (x–z); prerušovane = regál nafúknutý o r")
    axs[1].set_xlabel("x [m]")
    axs[1].set_ylabel("z [m]")
    axs[1].set_xlim(0.5, 13)
    axs[1].set_ylim(0, 6)
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(fontsize=8, loc="upper right")
    fig.suptitle("Väčší safety_radius → trasa stúpa vyššie nad regál a je dlhšia (štart ★ (12, 4, 1), cieľ ■ (4.3, 6.8, 4.8))")
    fig.tight_layout()
    fig.savefig(os.path.join(img, "polomer_trasa.png"), dpi=170)
    plt.close(fig)

    # 2) raw vs simplified for two routes
    names = ["úsek misie wp2→wp3", "dlhá diagonála hangárom"]
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.2))
    for ax, n in zip(axs, names):
        draw_floor(ax, band, boxes, safety)
        rr, ss = route(raw, safety, n), route(bench, safety, n)
        pr, ps = np.array(rr["path"]), np.array(ss["path"])
        ax.plot(pr[:, 0], pr[:, 1], ".", color=ORANGE, ms=6, label=f"A* surová: {len(pr)} bodov, {rr['length_m']:.2f} m")
        ax.plot(ps[:, 0], ps[:, 1], "-o", color=BLUE, lw=2, ms=6,
                label=f"po zjednodušení: {len(ps)} body, {ss['length_m']:.2f} m")
        ax.set_title(n)
        ax.legend(fontsize=8, loc="upper right")
    fig.suptitle(f"Zjednodušenie trasy (priama viditeľnosť) pri safety_radius {safety:.2f} m")
    fig.tight_layout()
    fig.savefig(os.path.join(img, "zjednodusenie.png"), dpi=170)
    plt.close(fig)

    # 3) occupancy slice at z = 2 m: raw voxels + filled racks vs inflated
    occ_file = f"/tmp/a1_1_occupancy_{safety:.2f}.pcd"
    sys.path.insert(0, ROOT)
    import view_3d_a1_2 as V
    if os.path.exists(occ_file):
        occ = V.load_pcd_binary_xyz(occ_file)
        rawv = V.voxelize(np.concatenate([cloud] + [V.box_solid_points(b) for b in boxes]))
        z0 = 2.0
        sl_raw = rawv[np.abs(rawv[:, 2] - z0) < 0.076]
        sl_occ = occ[np.abs(occ[:, 2] - z0) < 0.076]
        fig, ax = plt.subplots(figsize=(10, 7.4))
        ax.scatter(sl_occ[:, 0], sl_occ[:, 1], s=5, marker="s", c="#f6c9b7", label=f"nafúknuté ({safety:.2f} m) = zakázané pre stred drona")
        ax.scatter(sl_raw[:, 0], sl_raw[:, 1], s=5, marker="s", c="#52514e", label="obsadené voxely (mapa + vyplnené regály)")
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9, loc="upper right", markerscale=3)
        ax.set_title("Rez mriežkou vo výške z = 2.0 m (voxel 0.15 m)")
        fig.tight_layout()
        fig.savefig(os.path.join(img, "rez_mriezkou.png"), dpi=150)
        plt.close(fig)

    # 4) 3D view of the tested routes (VTK screenshot)
    tmp = tempfile.mkdtemp()
    ok = [x for x in route_list(bench, safety) if x["ok"] and "strechou" not in x["name"]]
    with open(os.path.join(tmp, "planned_paths.csv"), "w") as f:
        f.write("leg,waypoint,point,x,y,z\n")
        for leg, x in enumerate(ok, start=1):
            for i, p in enumerate(x["path"]):
                f.write(f"{leg},{leg},{i},{p[0]},{p[1]},{p[2]}\n")
    with open(os.path.join(tmp, "mission.csv"), "w") as f:
        for x in ok:
            f.write(f"{x['goal'][0]},{x['goal'][1]},{x['goal'][2]},hard,-\n")
    with open(os.path.join(tmp, "flight_log.csv"), "w") as f:
        f.write("t_s,state,waypoint,armed,mode,cmd_valid,cmd_x,cmd_y,cmd_z,cmd_yaw_deg,x,y,z,yaw_deg,speed\n")
        f.write("0,IDLE,1,0,-,0,0,0,0,0,13,7,0,0,0\n")
    subprocess.run([sys.executable, os.path.join(ROOT, "view_3d_a1_2.py"), tmp, "--routes-only",
                    "--camera", "top", "--cut", "2.6", "--screenshot", os.path.join(img, "trasy_3d.png")], check=True)
    shutil.rmtree(tmp)


def route_list(data, radius):
    for r in data:
        if abs(r["radius"] - radius) < 1e-6:
            return r["routes"]
    return []


# ------------------------------------------------------------------ A1.2
def a1_2(safety):
    runs = sorted(glob.glob(os.path.join(ROOT, "results", "A1_2", f"safety_{safety:.2f}_*")))
    if not runs:
        print("no recorded run in results/")
        return
    run = runs[-1]
    img = os.path.join(A12, "img")
    shutil.copy(os.path.join(run, "topview.png"), os.path.join(img, "vysledok_trasa.png"))
    shutil.copy(os.path.join(run, "overview.png"), os.path.join(img, "vysledok_prehlad.png"))
    subprocess.run([sys.executable, os.path.join(ROOT, "view_3d_a1_2.py"), run,
                    "--camera", "top", "--cut", "2.6", "--screenshot", os.path.join(img, "trasa_3d.png")], check=True)


def main():
    cloud = P.load_pcd_ascii(P.DEFAULT_MAP)
    safety, boxes = P.load_planner_yaml(P.DEFAULT_YAML)
    os.makedirs(os.path.join(A11, "img"), exist_ok=True)
    os.makedirs(os.path.join(A12, "img"), exist_ok=True)
    a1_1(cloud, boxes, safety)
    a1_2(safety)
    print("figures written to docs/A1_1/img and docs/A1_2/img")


if __name__ == "__main__":
    main()
