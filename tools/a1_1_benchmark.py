#!/usr/bin/env python3
"""
a1_1_benchmark.py -- A1.1 measurements without the simulator.

Starts map_planner_node with a given safety radius, sends a fixed set of plan requests, and
saves every report and route to docs/A1_1/data/benchmark.json.  Run it from the workspace root
after `colcon build` and `source install/setup.bash`:

    python3 tools/a1_1_benchmark.py                  # radii 0.30, 0.45, 0.75
    python3 tools/a1_1_benchmark.py --radii 0.45
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import String

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
YAML = os.path.join(ROOT, "src", "lrs_psenak_valko", "config", "planner.yaml")
OUT = os.path.join(ROOT, "docs", "A1_1", "data", "benchmark.json")

# name, start, goal
ROUTES = [
    ("okolo regálu do uličky", (13.0, 7.0, 1.5), (4.5, 9.1, 1.5)),
    ("cez regál (stúpanie)", (12.0, 4.0, 1.0), (4.3, 6.8, 4.8)),
    ("úsek misie wp2→wp3", (8.65, 2.02, 1.0), (4.84, 5.37, 2.0)),
    ("úsek misie wp4→wp5", (2.08, 9.74, 1.75), (8.84, 6.90, 2.0)),
    ("dlhá diagonála hangárom", (16.5, 1.0, 1.0), (1.0, 9.1, 3.0)),
    ("cieľ v regáli (chyba)", (13.0, 7.0, 1.5), (4.32, 6.82, 1.5)),
    ("cieľ mimo mapy (chyba)", (13.0, 7.0, 1.5), (25.0, 7.0, 1.5)),
    ("cieľ nad strechou (diera v mračne)", (13.0, 7.0, 1.5), (9.0, 6.0, 6.35)),
    ("cieľ v uzavretej kancelárii", (13.0, 7.0, 1.5), (3.0, 1.5, 0.8)),
]


def parse_report(text):
    def num(pattern, cast=float):
        m = re.search(pattern, text)
        return cast(m.group(1)) if m else None
    return {
        "ok": "[OK] TRASA NÁJDENÁ" in text,
        "status": num(r"Stav plánovača: (\w+)", str),
        "raw_points": num(r"Body pôvodne \(A\*\)\s*:\s*(\d+)", int),
        "points": num(r"Body po zjednodušení\s*:\s*(\d+)", int),
        "length_m": num(r"Dĺžka trasy \(finálna\)\s*:\s*([\d.]+)"),
        "raw_length_m": num(r"Dĺžka pôvodnej trasy\s*:\s*([\d.]+)"),
        "time_ms": num(r"Čas plánovania\s*:\s*([\d.]+) ms") or num(r"Čas plánovania: ([\d.]+) ms"),
        "expansions": num(r"expanzií: (\d+)", int),
        "report": text,
    }


class Client(Node):
    def __init__(self):
        super().__init__("a1_1_benchmark")
        self.pub = self.create_publisher(PoseArray, "plan_request", 10)
        self.status = None
        self.path = None
        self.create_subscription(String, "planning_status", self._status, 10)
        self.create_subscription(PoseArray, "planned_path", self._path, 10)

    def _status(self, m):
        self.status = m.data

    def _path(self, m):
        self.path = [(p.position.x, p.position.y, p.position.z) for p in m.poses]

    def plan(self, start, goal, timeout=20.0):
        self.status, self.path = None, None
        req = PoseArray()
        req.header.frame_id = "map"
        for p in (start, goal):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p
            pose.orientation.w = 1.0
            req.poses.append(pose)
        self.pub.publish(req)
        end = time.time() + timeout
        while time.time() < end and self.status is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.status and "[OK]" in self.status:     # the path is published before the status
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
        return self.status, self.path


def run_radius(radius, simplify=True):
    tag = "" if simplify else "_raw"
    log = open(os.path.join(os.path.dirname(OUT), f"planner_{radius:.2f}{tag}.log"), "w")
    proc = subprocess.Popen(
        ["ros2", "run", "lrs_psenak_valko", "map_planner_node", "--ros-args", "--params-file", YAML,
         "-p", f"safety_radius:={radius}",
         "-p", f"simplify_path:={'true' if simplify else 'false'}",
         "-p", f"occupancy_pcd_path:=/tmp/a1_1_occupancy_{radius:.2f}.pcd"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    try:
        while time.time() - t0 < 120:
            log.flush()
            text = open(log.name).read()
            if "Ready:" in text:
                break
            if proc.poll() is not None:
                sys.exit(f"planner exited, see {log.name}")
            time.sleep(0.5)
        startup_s = time.time() - t0
        text = open(log.name).read()
        info = {"radius": radius, "startup_s": round(startup_s, 1)}
        for key, pat in [("bounds", r"Map bounds: (.*)"), ("voxels", r"occupied voxels: (\d+)"),
                         ("inflated", r"inflated: (\d+)"), ("grid", r"3D planning grid: (.*)"),
                         ("points_raw", r"Voxelized points: (\d+)")]:
            m = re.search(pat, text)
            info[key] = m.group(1).strip() if m else None
        client = Client()
        time.sleep(1.0)                      # discovery
        results = []
        for name, start, goal in ROUTES:
            status, path = client.plan(start, goal)
            r = parse_report(status or "")
            r.update({"name": name, "start": start, "goal": goal, "path": path if r["ok"] else None})
            results.append(r)
            print(f"  r={radius:.2f} {name:28s} {r['status']}: {r['raw_points']} -> {r['points']} pts, "
                  f"{r['length_m']} m, {r['time_ms']} ms, {r['expansions']} exp")
        client.destroy_node()
        info["routes"] = results
        return info
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radii", type=float, nargs="+", default=[0.30, 0.45, 0.75])
    ap.add_argument("--no-simplify", action="store_true",
                    help="plan without simplification (raw A* routes) -> benchmark_raw.json")
    args = ap.parse_args()
    out = OUT.replace(".json", "_raw.json") if args.no_simplify else OUT
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    rclpy.init()
    data = []
    for radius in args.radii:
        print(f"safety_radius = {radius:.2f} m{' (no simplification)' if args.no_simplify else ''}")
        data.append(run_radius(radius, simplify=not args.no_simplify))
    rclpy.shutdown()
    with open(out, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
