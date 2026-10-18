#!/usr/bin/env python3
"""
view_3d_a1_2.py -- interactive 3D voxel view of one A1.2 mission run (VTK).

Shows the hangar as voxels (walls, pillars, racks), the planned A* route as a glowing green
tube with a translucent safety-radius corridor around it, a green light that travels along the
route (and lights up the voxels it passes), the flown track and the mission waypoints.

Usage:
    python3 view_3d_a1_2.py                       # newest run in log/a1_2_runs
    python3 view_3d_a1_2.py log/a1_2_runs/<run>
    python3 view_3d_a1_2.py RUN --screenshot x.png  # render one image and exit

Mouse: left drag rotate, right drag / wheel zoom, middle drag pan.
Keys:  space play/pause the light   c roof on/off   +/- hide walls above a lower/higher height
       i planner occupancy (inflated) on/off   f flown track on/off   r reset camera   q quit
"""

import argparse
import glob
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
import plot_a1_2 as P  # noqa: E402  (shared loaders)

VOXEL = 0.15
CEILING_Z = 5.0             # voxels above this are the roof; hidden by default so you can look in
LIGHT_SPEED = 1.5           # m/s of the travelling light (real time of the viewer)
GREEN = (0.15, 1.0, 0.3)


def load_pcd_binary_xyz(path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            key, _, value = line.partition(" ")
            header[key] = value
            if key == "DATA":
                break
        if header["DATA"] != "binary" or header.get("FIELDS") != "x y z":
            return None
        n = int(header["POINTS"])
        return np.frombuffer(f.read(n * 12), dtype=np.float32).reshape(n, 3)


def voxelize(points, size=VOXEL):
    idx = np.unique(np.floor(points / size).astype(np.int32), axis=0)
    return (idx + 0.5) * size


def box_solid_points(box, step=VOXEL):
    cx, cy, cz, sx, sy, sz, yaw = box
    g = np.stack(np.meshgrid(*[np.arange(-s / 2 + step / 2, s / 2, step) for s in (sx, sy, sz)],
                             indexing="ij"), axis=-1).reshape(-1, 3)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([cx + c * g[:, 0] - s * g[:, 1], cy + s * g[:, 0] + c * g[:, 1], cz + g[:, 2]], axis=1)


def route_polyline(data, legs, mission):
    """Planned route in flight order, including the vertical takeoff, landtakeoff and landing."""
    pts = []
    air = P.airborne_mask(data)
    if air.any():
        k = np.argmax(air)
        pts.append((data["x"][k], data["y"][k], 0.0))
    for leg in legs.values():
        wp = mission[leg["waypoint"] - 1] if 0 < leg["waypoint"] <= len(mission) else None
        pts.extend(map(tuple, leg["pts"]))
        if wp and wp["task"] in ("landtakeoff", "land"):
            pts.append((wp["x"], wp["y"], 0.0))
            if wp["task"] == "landtakeoff":
                pts.append((wp["x"], wp["y"], wp["z"]))
    clean = [pts[0]] if pts else []
    for p in pts[1:]:
        if math.dist(p, clean[-1]) > 1e-3:
            clean.append(p)
    return np.array(clean)


def main():
    ap = argparse.ArgumentParser(description="3D voxel view of an A1.2 run.")
    ap.add_argument("run_dir", nargs="?")
    ap.add_argument("--map", default=P.DEFAULT_MAP)
    ap.add_argument("--planner-yaml", default=P.DEFAULT_YAML)
    ap.add_argument("--occupancy", default=os.path.join(ROOT, "map_voxels_with_racks_inflated.pcd"))
    ap.add_argument("--screenshot", help="render to this PNG and exit")
    ap.add_argument("--camera", choices=["default", "top"], default="default",
                    help="initial view: oblique from the south-east, or almost from above")
    ap.add_argument("--cut", type=float, default=None, help="hide walls above this height [m]")
    ap.add_argument("--routes-only", action="store_true",
                    help="draw every planned leg as its own tube (planner tests without a flight)")
    args = ap.parse_args()

    run_dir = args.run_dir
    if not run_dir:
        runs = sorted(glob.glob(os.path.join(P.DEFAULT_RUNS, "*", "flight_log.csv")))
        if not runs:
            sys.exit(f"no runs found in {P.DEFAULT_RUNS}")
        run_dir = os.path.dirname(runs[-1])
    name = os.path.basename(os.path.normpath(run_dir))
    print(f"Run: {run_dir}")

    import vtk
    from vtk.util.numpy_support import numpy_to_vtk

    data = P.load_flight(os.path.join(run_dir, "flight_log.csv"))
    legs = P.load_paths(os.path.join(run_dir, "planned_paths.csv"))
    mission_path = os.path.join(run_dir, "mission.csv")
    mission = P.load_mission(mission_path) if os.path.exists(mission_path) else []
    safety, boxes = P.load_planner_yaml(args.planner_yaml)

    cloud = P.load_pcd_ascii(args.map)
    walls = voxelize(cloud[cloud[:, 2] > 0.1])
    racks = voxelize(np.concatenate([box_solid_points(b) for b in boxes])) if boxes else np.empty((0, 3))
    lo, hi = cloud.min(axis=0), cloud.max(axis=0)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.06, 0.07, 0.10)
    renderer.SetBackground2(0.16, 0.18, 0.24)
    renderer.GradientBackgroundOn()
    renderer.SetAmbient(1, 1, 1)

    # ------------------------------------------------------------------ voxels
    def voxel_poly(centers, colors_rgb):
        poly = vtk.vtkPolyData()
        pts = vtk.vtkPoints()
        pts.SetData(numpy_to_vtk(np.ascontiguousarray(centers, dtype=np.float32).reshape(-1, 3), deep=True))
        poly.SetPoints(pts)
        col = numpy_to_vtk(np.ascontiguousarray(colors_rgb, dtype=np.uint8).reshape(-1, 3), deep=True)
        col.SetName("rgb")
        poly.GetPointData().SetScalars(col)
        return poly

    def voxel_actor(centers, colors_rgb, size=VOXEL * 0.92, opacity=1.0):
        poly = voxel_poly(centers, colors_rgb)
        cube = vtk.vtkCubeSource()
        cube.SetXLength(size)
        cube.SetYLength(size)
        cube.SetZLength(size)
        mapper = vtk.vtkGlyph3DMapper()
        mapper.SetInputData(poly)
        mapper.SetSourceConnection(cube.GetOutputPort())
        mapper.ScalingOff()
        mapper.SetScalarModeToUsePointData()
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetOpacity(opacity)
        actor.GetProperty().SetAmbient(0.25)
        actor.GetProperty().SetDiffuse(0.8)
        return actor

    def height_colors(c, base, top):
        t = np.clip((c[:, 2] - 0.0) / 5.0, 0, 1)[:, None]
        return (np.array(base) * (1 - t) + np.array(top) * t)

    # Walls/pillars/roof are one actor; voxels above the "cut" height are hidden so you can look in.
    wall_colors = height_colors(walls, (70, 85, 110), (175, 185, 205))
    wall_actor = voxel_actor(walls, wall_colors)
    view = {"cut": args.cut if args.cut is not None else CEILING_Z}

    def apply_cut():
        keep = walls[:, 2] <= view["cut"]
        wall_actor.GetMapper().SetInputData(voxel_poly(walls[keep], wall_colors[keep]))

    apply_cut()
    rack_actor = voxel_actor(racks, height_colors(racks, (150, 70, 40), (215, 120, 70)))
    for a in (wall_actor, rack_actor):
        renderer.AddActor(a)

    floor = vtk.vtkPlaneSource()
    floor.SetOrigin(lo[0], lo[1], 0)
    floor.SetPoint1(hi[0], lo[1], 0)
    floor.SetPoint2(lo[0], hi[1], 0)
    floor.SetResolution(int(hi[0] - lo[0]), int(hi[1] - lo[1]))
    fm = vtk.vtkPolyDataMapper()
    fm.SetInputConnection(floor.GetOutputPort())
    floor_actor = vtk.vtkActor()
    floor_actor.SetMapper(fm)
    floor_actor.GetProperty().SetColor(0.20, 0.22, 0.26)
    floor_actor.GetProperty().SetEdgeColor(0.30, 0.33, 0.38)
    floor_actor.GetProperty().EdgeVisibilityOn()
    renderer.AddActor(floor_actor)

    occ_actor = None
    occ = load_pcd_binary_xyz(args.occupancy) if os.path.exists(args.occupancy) else None
    if occ is not None:
        occ = occ[(occ[:, 2] > 0.1) & (occ[:, 2] < CEILING_Z)]
        occ_actor = voxel_actor(occ, np.tile([230, 60, 60], (len(occ), 1)), size=VOXEL * 0.6, opacity=0.08)
        occ_actor.VisibilityOff()
        renderer.AddActor(occ_actor)

    # ------------------------------------------------------------------ lines
    def tube_actor(points, radius, color, opacity=1.0, glow=False):
        pts = vtk.vtkPoints()
        for p in points:
            pts.InsertNextPoint(*p)
        line = vtk.vtkPolyLine()
        line.GetPointIds().SetNumberOfIds(len(points))
        for i in range(len(points)):
            line.GetPointIds().SetId(i, i)
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(line)
        poly = vtk.vtkPolyData()
        poly.SetPoints(pts)
        poly.SetLines(cells)
        tube = vtk.vtkTubeFilter()
        tube.SetInputData(poly)
        tube.SetRadius(radius)
        tube.SetNumberOfSides(16)
        tube.CappingOn()
        m = vtk.vtkPolyDataMapper()
        m.SetInputConnection(tube.GetOutputPort())
        a = vtk.vtkActor()
        a.SetMapper(m)
        prop = a.GetProperty()
        prop.SetColor(*color)
        prop.SetOpacity(opacity)
        if glow:
            prop.SetAmbient(1.0)
            prop.SetDiffuse(0.0)
            prop.SetSpecular(0.0)
        return a

    route = route_polyline(data, legs, mission)
    if args.routes_only:
        for leg in legs.values():
            if len(leg["pts"]) >= 2:
                renderer.AddActor(tube_actor(leg["pts"], 0.045, GREEN, glow=True))
                corridor = tube_actor(leg["pts"], safety, (0.2, 0.9, 0.35), opacity=0.10)
                corridor.GetProperty().BackfaceCullingOn()
                renderer.AddActor(corridor)
        route = np.empty((0, 3))            # no travelling light across unrelated routes
    elif len(route) >= 2:
        renderer.AddActor(tube_actor(route, 0.045, GREEN, glow=True))
        corridor = tube_actor(route, safety, (0.2, 0.9, 0.35), opacity=0.10)
        corridor.GetProperty().BackfaceCullingOn()
        renderer.AddActor(corridor)

    air = P.airborne_mask(data)
    flown_pts = np.stack([data["x"], data["y"], data["z"]], axis=1)[air][::4]
    flown_actor = None
    if len(flown_pts) >= 2:
        flown_actor = tube_actor(flown_pts, 0.018, (0.35, 0.75, 1.0))
        renderer.AddActor(flown_actor)

    # ------------------------------------------------------------------ waypoints
    labels = vtk.vtkRenderer()          # overlay layer: labels are never hidden behind walls
    labels.SetLayer(1)
    labels.InteractiveOff()
    labels.SetActiveCamera(renderer.GetActiveCamera())
    for i, wp in enumerate(mission, start=1):
        src = vtk.vtkCubeSource() if wp["precision"] == "hard" else vtk.vtkSphereSource()
        if wp["precision"] == "hard":
            src.SetXLength(0.22)
            src.SetYLength(0.22)
            src.SetZLength(0.22)
        else:
            src.SetRadius(0.13)
            src.SetThetaResolution(20)
            src.SetPhiResolution(20)
        src.SetCenter(wp["x"], wp["y"], wp["z"])
        m = vtk.vtkPolyDataMapper()
        m.SetInputConnection(src.GetOutputPort())
        a = vtk.vtkActor()
        a.SetMapper(m)
        a.GetProperty().SetColor(1.0, 0.82, 0.1)
        a.GetProperty().SetAmbient(0.5)
        renderer.AddActor(a)
        label = vtk.vtkBillboardTextActor3D()
        task = "" if wp["task"] == "-" else f" {wp['task']}"
        label.SetInput(f"{i}{task}")
        label.SetPosition(wp["x"], wp["y"], wp["z"] + 0.35)
        label.GetTextProperty().SetFontSize(16)
        label.GetTextProperty().SetColor(1.0, 0.95, 0.6)
        label.GetTextProperty().BoldOn()
        label.GetTextProperty().SetJustificationToCentered()
        labels.AddActor(label)

    # ------------------------------------------------------------------ travelling green light
    orb_src = vtk.vtkSphereSource()
    orb_src.SetRadius(0.14)
    orb_src.SetThetaResolution(24)
    orb_src.SetPhiResolution(24)
    om = vtk.vtkPolyDataMapper()
    om.SetInputConnection(orb_src.GetOutputPort())
    orb = vtk.vtkActor()
    orb.SetMapper(om)
    orb.GetProperty().SetColor(0.6, 1.0, 0.7)
    orb.GetProperty().SetAmbient(1.0)
    orb.GetProperty().SetDiffuse(0.0)
    halo_src = vtk.vtkSphereSource()
    halo_src.SetRadius(0.35)
    halo_src.SetThetaResolution(24)
    halo_src.SetPhiResolution(24)
    hm = vtk.vtkPolyDataMapper()
    hm.SetInputConnection(halo_src.GetOutputPort())
    halo = vtk.vtkActor()
    halo.SetMapper(hm)
    halo.GetProperty().SetColor(*GREEN)
    halo.GetProperty().SetOpacity(0.25)
    halo.GetProperty().SetAmbient(1.0)
    halo.GetProperty().SetDiffuse(0.0)
    renderer.AddActor(orb)
    renderer.AddActor(halo)

    green_light = vtk.vtkLight()
    green_light.SetLightTypeToSceneLight()
    green_light.SetPositional(True)
    green_light.SetConeAngle(180)
    green_light.SetColor(*GREEN)
    green_light.SetIntensity(1.6)
    green_light.SetAttenuationValues(0.2, 0.0, 0.9)
    renderer.AddLight(green_light)
    head = vtk.vtkLight()
    head.SetLightTypeToHeadlight()
    head.SetIntensity(0.75)
    renderer.AddLight(head)

    if len(route) >= 2:
        seg = np.linalg.norm(np.diff(route, axis=0), axis=1)
        cum = np.concatenate([[0], np.cumsum(seg)])
    else:
        cum = np.array([0.0])
    state = {"s": 0.0, "playing": True}

    def place_light():
        if len(route) < 2:
            return
        s = state["s"] % cum[-1]
        k = min(np.searchsorted(cum, s, side="right") - 1, len(seg) - 1)
        u = (s - cum[k]) / max(seg[k], 1e-9)
        p = route[k] + (route[k + 1] - route[k]) * u
        orb.SetPosition(*p)
        halo.SetPosition(*p)
        green_light.SetPosition(*p)
        green_light.SetFocalPoint(p[0], p[1], p[2] - 1.0)

    place_light()
    if len(route) < 2:                       # nothing to travel along (e.g. --routes-only)
        orb.VisibilityOff()
        halo.VisibilityOff()
        green_light.SetIntensity(0.0)

    # ------------------------------------------------------------------ overlay text
    total = cum[-1] if len(cum) else 0.0
    info = vtk.vtkTextActor()

    def update_info():
        info.SetInput(
            f"A1.2 3D route - {name}\n"
            f"green tube: planned A* route ({total:.1f} m), corridor = safety radius {safety:.2f} m\n"
            "blue: flown track    orange: racks    yellow: waypoints (cube hard, sphere soft)\n"
            f"walls shown up to z = {view['cut']:.2f} m  (+/- change, c roof on/off)\n"
            "space play/pause light   i planner occupancy   f flown track   r reset view   q quit")

    update_info()
    info.GetTextProperty().SetFontSize(14)
    info.GetTextProperty().SetColor(0.9, 0.92, 0.95)
    info.SetDisplayPosition(12, 12)
    renderer.AddViewProp(info)

    # ------------------------------------------------------------------ window
    window = vtk.vtkRenderWindow()
    window.SetNumberOfLayers(2)
    window.AddRenderer(renderer)
    window.AddRenderer(labels)
    window.SetSize(1400, 900)
    window.SetWindowName(f"A1.2 3D route - {name}")
    window.SetMultiSamples(4)

    def reset_camera():
        cam = renderer.GetActiveCamera()
        center = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 1.5)
        cam.SetFocalPoint(*center)
        if args.camera == "top":
            cam.SetPosition(center[0] + 2, center[1] - 7, 26)
        else:
            cam.SetPosition(center[0] + 7, center[1] - 14, 21)
        cam.SetViewUp(0, 0, 1)
        renderer.ResetCameraClippingRange()

    reset_camera()

    if args.screenshot:
        info.VisibilityOff()                 # documentation images without the help text
        window.SetOffScreenRendering(1)
        state["s"] = total * 0.3
        place_light()
        window.Render()
        grab = vtk.vtkWindowToImageFilter()
        grab.SetInput(window)
        grab.Update()
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(args.screenshot)
        writer.SetInputConnection(grab.GetOutputPort())
        writer.Write()
        print(f"Saved {args.screenshot}")
        return

    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(window)
    style = vtk.vtkInteractorStyleTrackballCamera()
    interactor.SetInteractorStyle(style)

    # An observer on the style's CharEvent replaces VTK's built-in keys (w wireframe, f fly-to, ...).
    def on_key(_obj, _event):
        key = interactor.GetKeySym().lower()
        if key == "space":
            state["playing"] = not state["playing"]
        elif key == "c":
            view["cut"] = 99.0 if view["cut"] < 50 else CEILING_Z
            apply_cut()
        elif key in ("plus", "equal", "kp_add"):
            view["cut"] = min(view["cut"] + 0.25, 99.0) if view["cut"] < 50 else CEILING_Z
            apply_cut()
        elif key in ("minus", "kp_subtract"):
            view["cut"] = max((view["cut"] if view["cut"] < 50 else hi[2]) - 0.25, 0.25)
            apply_cut()
        elif key == "i" and occ_actor is not None:
            occ_actor.SetVisibility(not occ_actor.GetVisibility())
        elif key == "f" and flown_actor is not None:
            flown_actor.SetVisibility(not flown_actor.GetVisibility())
        elif key == "r":
            reset_camera()
        elif key in ("q", "escape"):
            window.Finalize()
            interactor.TerminateApp()
            return
        update_info()
        window.Render()

    def on_timer(_obj, _event):
        if state["playing"] and total > 0:
            state["s"] += LIGHT_SPEED * 0.04
            place_light()
            window.Render()

    style.AddObserver("CharEvent", on_key)
    interactor.Initialize()
    interactor.AddObserver("TimerEvent", on_timer)
    interactor.CreateRepeatingTimer(40)
    window.Render()
    interactor.Start()


if __name__ == "__main__":
    main()
