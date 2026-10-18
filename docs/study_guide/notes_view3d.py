FILE = "view_3d_a1_2.py"
TITLE = "3D route viewer (view_3d_a1_2.py)"
INTRO = """<p>An interactive 3D window built with <b>VTK</b> (the visualisation library behind
ParaView, also used by RViz-like tools). It shows the hangar as voxels and the planned route as a
glowing green tube with a travelling light. It is a visual aid; the numbers come from
<code>plot_a1_2.py</code>.</p>"""

NOTES = [
("#!/usr/bin/env python3", "Docstring with controls",
 """<p>Usage, mouse and keyboard controls.</p>"""),
("import argparse", "Imports, shared loaders, constants",
 """<p>Imports the loaders of <code>plot_a1_2.py</code> as <code>P</code>, so both tools read the
 logs in exactly the same way (the script folder is put on <code>sys.path</code> first).
 Constants: voxel size 0.15 m (same as the planner), roof height 5 m, speed of the travelling
 light, the green colour.</p>"""),
("def load_pcd_binary_xyz(path):", "load_pcd_binary_xyz()",
 """<p>Reads the planner's inflated occupancy file: parse the text header up to
 <code>DATA binary</code>, then the rest is <code>POINTS × 3</code> 32-bit floats, read in one go
 with <code>np.frombuffer</code>.</p>"""),
("def voxelize(points, size=VOXEL):", "voxelize()",
 """<p><code>floor(p / 0.15)</code> = integer voxel index of each point; <code>np.unique</code>
 keeps one per voxel; <code>(index + 0.5) · size</code> = voxel centre. This reproduces the
 planner's voxel map.</p>"""),
("def box_solid_points(box, step=VOXEL):", "box_solid_points()",
 """<p>A 3D grid of points filling a rotated rack box: solid, like the planner's
 <code>addBox()</code>.</p>"""),
("def route_polyline(data, legs, mission):", "route_polyline(): the whole flight as one line",
 """<p>Joins the planned legs in order and adds the vertical parts that are not A* routes: from the
 floor at takeoff, down and up at a <code>landtakeoff</code>, down at the final <code>land</code>.
 Duplicate consecutive points are removed (they would give zero-length segments).</p>"""),
("def main():", "main(): options and data",
 """<p>Options: run folder (default newest), map, YAML, occupancy file, and
 <code>--screenshot</code> (render one PNG and exit, used to test the viewer). Loads the logs,
 voxelises the map (floor excluded) and fills the racks.</p>"""),
("    renderer = vtk.vtkRenderer()", "Renderer",
 """<p>A <b>renderer</b> draws a scene of <b>actors</b> into a window. Gradient dark background.</p>"""),
("    # ------------------------------------------------------------------ voxels", "Voxels with GPU instancing",
 """<p><code>voxel_poly</code>: the centres become a VTK point set with an RGB colour per point
 (<code>numpy_to_vtk</code> copies numpy arrays into VTK).</p>
 <p><code>voxel_actor</code>: <code>vtkGlyph3DMapper</code> draws one small cube <b>at every
 point</b> on the GPU (instancing). That is how 60 000+ voxels stay interactive. The cubes are
 92 % of a voxel, so gaps show the grid.</p>"""),
("    def height_colors(c, base, top):", "Colours and wall height cut",
 """<p>Colour blends from dark (floor) to light (5 m), so height is readable. Walls are one actor
 whose input is swapped by <code>apply_cut()</code>: only voxels below the cut height are shown.
 Keys <code>+</code>/<code>-</code>/<code>c</code> change it. Racks are a separate, orange
 actor.</p>"""),
("    floor = vtk.vtkPlaneSource()", "Floor grid",
 """<p>A flat plane at z = 0 with a 1 m grid drawn as edges, instead of thousands of floor
 voxels.</p>"""),
("    occ_actor = None", "Planner occupancy (key i)",
 """<p>The inflated occupancy from the planner (the space A* considers blocked), as small faint red
 cubes, hidden by default.</p>"""),
("    # ------------------------------------------------------------------ lines", "Tubes",
 """<p><code>tube_actor</code>: builds a polyline from points and wraps it in a
 <code>vtkTubeFilter</code> (a 3D tube of given radius). <code>glow=True</code>: ambient light
 only, so it looks self-lit.</p>"""),
("    route = route_polyline(data, legs, mission)", "Route, corridor, flown track",
 """<p>Bright green tube (4.5 cm) = planned route. A translucent tube with radius = safety radius
 around it = the corridor A* kept free. Thin blue tube = flown track (every 4th airborne
 sample).</p>"""),
("    # ------------------------------------------------------------------ waypoints", "Waypoints and labels",
 """<p>A second renderer on <b>layer 1</b> shares the camera and draws only the labels, so they
 are never hidden behind walls. Hard waypoints are cubes, soft ones spheres; labels are billboards
 (always facing the camera).</p>"""),
("    # ------------------------------------------------------------------ travelling green light", "The travelling green light",
 """<p>A bright core sphere, a translucent halo and a <b>positional VTK light</b> (green,
 attenuated with distance) all at the same point. The light really illuminates the voxels near
 it, which makes the route easy to follow in 3D. A headlight lights the rest.</p>"""),
("    if len(route) >= 2:", "Moving the light along the route",
 """<p>Cumulative segment lengths <code>cum</code> turn "distance travelled s" into a segment
 index (<code>searchsorted</code>) and a fraction <code>u</code>; the position is interpolated.
 <code>s</code> wraps around (<code>% total</code>), so the light loops.</p>"""),
("    # ------------------------------------------------------------------ overlay text", "Help text",
 """<p>A 2D text actor in the corner: run name, route length, legend, current cut height and the
 keys. It is updated after every key press.</p>"""),
("    # ------------------------------------------------------------------ window", "Window and camera",
 """<p>Two layers (scene + labels), 1400×900, anti-aliasing. <code>reset_camera</code> looks at
 the hangar centre from the south-east, high up, z axis up.</p>"""),
("    if args.screenshot:", "Screenshot mode",
 """<p>Renders off-screen once, with the light placed at 30 % of the route, and writes a PNG
 (<code>vtkWindowToImageFilter</code> → <code>vtkPNGWriter</code>).</p>"""),
("    interactor = vtk.vtkRenderWindowInteractor()", "Interaction: mouse, keys, timer",
 """<ul><li>Trackball camera style: drag to rotate, wheel to zoom, middle button to pan.</li>
 <li>An observer on the style's <code>CharEvent</code> <b>replaces</b> VTK's built-in keys
 (otherwise <code>w</code> = wireframe, <code>f</code> = fly-to...) with ours.</li>
 <li>A repeating 40 ms timer moves the light by speed × 0.04 s and re-renders.</li>
 <li><code>Start()</code> runs the event loop until the window is closed or <code>q</code>.</li></ul>"""),
("if __name__ == \"__main__\":", "Entry point", """<p>Runs <code>main()</code> when executed.</p>"""),
]
