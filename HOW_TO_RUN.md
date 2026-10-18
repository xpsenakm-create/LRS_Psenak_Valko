# How to run everything

All commands are run from the repository folder:

```bash
cd ~/Zadania/LRS_Psenak_Valko
```

## A1.1 — planner only (no simulator)

| What | Command |
|---|---|
| Planner + RViz + one demo route | `./run_a1_1.sh` |
| Custom route / safety radius | `./run_a1_1.sh --start 12 4 2 --goal 16 8 2 --safety-radius 0.75` |
| Planner without RViz | `./run_a1_1.sh --no-rviz` |
| Re-measure all A1.1 numbers (saved to `docs/A1_1/data/`) | `source install/setup.bash && python3 tools/a1_1_benchmark.py` |

While the planner runs, more routes can be requested without a restart:

```bash
ros2 topic pub --once /plan_request geometry_msgs/msg/PoseArray \
  "{header: {frame_id: map}, poses: [{position: {x: 13, y: 7, z: 1.5}, orientation: {w: 1}}, {position: {x: 4.5, y: 9.1, z: 1.5}, orientation: {w: 1}}]}"
```

## A1.2 — indoor mission (branch `A1_2`)

| What | Command |
|---|---|
| Full mission (opens Gazebo, SITL, MAVROS itself) | `./run_a1_2.sh` |
| Any mission file (e.g. the one from the defence) | `./run_a1_2.sh /path/to/mission.csv` |
| Close the simulator at the end, skip the rebuild | `A1_2_STOP_SIM=1 A1_2_SKIP_BUILD=1 ./run_a1_2.sh` |
| Also open RViz | `A1_2_RVIZ=1 ./run_a1_2.sh` |
| Gazebo, SITL, MAVROS already started by hand | `A1_2_START_SIM=0 ./run_a1_2.sh` |
| Re-open the plots of a run | `python3 plot_a1_2.py log/a1_2_runs/<run>` |
| Re-open the 3D view of a run | `python3 view_3d_a1_2.py log/a1_2_runs/<run>` |

- When the mission ends, three windows open: top view, overview, 3D view.
- Every run is saved in `log/a1_2_runs/<date>_<time>_<mission>/` (logs, rosbag, plots; not in git).
- The submitted recording is `results/hangar_example_20261018/`.
- Do not click in the Gazebo window during a flight (the ⏸ button freezes ArduPilot).
- All options: see the top of `run_a1_2.sh`.

Test missions: `src/lrs_psenak_valko/missions/test_east_aisle.csv`, `test_over_racks.csv`.

## A1.3 — outdoor mission (branch `A1_3`)

```bash
git switch A1_3
./run_a1_3.sh --record        # after landing press Ctrl+C: bag + plots go to results/A1_3/
git switch A1_2               # back
```

## Documents

| Document | Where | How to open |
|---|---|---|
| A1.1 (SK) | `docs/A1_1/A1_1_mapa_a_planovanie.md` / `.pdf` | VS Code + `Ctrl+Shift+V`, any PDF viewer, or GitHub |
| A1.2 (SK) | `docs/A1_2/A1_2_indoor_misia.md` / `.pdf` | same |
| A1.2 detailed (EN) | `docs/A1_2/A1_2_detailed_EN.md` | same |
| A1.3 (SK) | `docs/A1_3/` on branch `A1_3` | same |
| Study guide A1.1–A1.3 | `docs/A1_study_guide.html` | `xdg-open docs/A1_study_guide.html` (browser, offline) |

Study guide: search box at the top, "Quiz mode" hides the explanations, flashcards with examiner
questions near the end.

## After changing code or documents

| What | Command |
|---|---|
| Rebuild the study guide (also fetches the newest A1.3) | `git fetch && python3 docs/study_guide/build_study_guide.py` |
| Regenerate the figures in `docs/A1_1/img`, `docs/A1_2/img` | `python3 tools/doc_figures.py` |
| Re-render the diagrams | `for f in docs/A1_*/diagrams/*.dot; do dot -Tpng -Gdpi=200 "$f" -o "$(dirname "$(dirname "$f")")/img/$(basename "$f" .dot).png"; done` |
| Regenerate the PDFs | `python3 tools/md_to_pdf.py docs/A1_1/A1_1_mapa_a_planovanie.md docs/A1_2/A1_2_indoor_misia.md` |

If the study-guide build stops with "anchor not found", the code changed where an explanation is
attached: update the anchor text in the matching `docs/study_guide/notes_*.py` file.
