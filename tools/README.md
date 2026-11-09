# Keypoint recorder (ROS 2)

Build and source the workspace, then run:

```bash
ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml
```

The generated file is a ROS 2 parameter file containing the flat
`fsm.waypoints: [x0, y0, z0, ...]` array. Pass it to `run.launch.py` with
`keypoints_file:=/absolute/path/to/keypoints.yaml` when `navi_mode:=2`.

# Synthetic two-floor map + tour (basic-logic smoke test)

`gen_multi_floor_pcd.py` procedurally builds a **clean, noise-free** two-floor
scene — two floor slabs joined by a single straight staircase, plus outer walls —
and a matching `fsm.waypoints` tour. It exists to exercise SCAN-Planner's basic
floor-to-floor logic (2.5D A\* + B-spline + `PRESET_TARGET` FSM) deterministically,
without the difficulty and noise of the real GoSLAM indoor scan.

It writes two files (both already wired into the dedicated preset below):

- `src/simulator/map_generator/maps/synthetic_two_floor_0.05m.pcd` — binary PCD,
  ~517k points. Floor-1 top at `z=0` (== `grid_map.ground_height`), floor-2 top at
  `z=2.5`, staircase `x in [-2, 2]` ascending `+Y` with 20 risers (rise 0.125,
  run 0.35, ~19.6 deg slope).
- `src/simulator/map_generator/configs/keypoints.synthetic.yaml` — a 9-point tour:
  floor 1 -> a stair stop every 4 treads -> floor 2 west end (goal).

## Generate

```bash
python3 tools/gen_multi_floor_pcd.py     # rewrites both files in place
```

The PCD and YAML are single-sourced from the geometry constants at the top of the
script (`FLOOR*_Z`, `N_RISERS`, `RISE`/`RUN`, `BODY_OFFSET`, `WP_TREAD_STEP`), so
edit those and re-run to reshape the scene. With `--symlink-install` the regenerated
files are picked up in place — **no rebuild needed**.

## Run

```bash
# headless replay of the generated tour (default navi_mode:=2)
ros2 launch scan_planner run_synthetic_multi.launch.py

# or drive it manually with RViz "2D Nav Goal"
ros2 launch scan_planner run_synthetic_multi.launch.py navi_mode:=1
```

`run_synthetic_multi.launch.py` wraps `run.launch.py` with
`controller_mode:=open_loop` (the quadruped kinematic sim cannot climb stairs;
`open_loop_controller` republishes the planned trajectory as odometry so floor
changes work), `use_pcd_map:=true`, and its own `config/controllers.synthetic.yaml`
spawn (floor-1 entry `(0, -4.8, 0.55)` facing `+Y`) via the `controllers_file`
override — so the xiaohao preset's `controllers.yaml` stays untouched. A passing run
plans all 9 waypoints and ends on the FSM transition `EXEC_TRAJ -> WAIT_TARGET` with
**0** emergency stops.

## Tuning knobs (why the numbers are what they are)

- **`BODY_OFFSET = 0.55`** — waypoint Z = surface Z + this. `navi_mode:=2` consumes
  waypoint Z verbatim, so it must lift the whole **double-cylinder** body footprint
  (two 0.25 m-radius cylinders at `pos +/- 0.18 m` along the heading) clear of the
  inflated occupied band `[surface - z_down(0.4), surface + z_up(0.1)]`. The binding
  case is the *front* cylinder reaching the next riser up, giving a static minimum of
  `RISE + z_up = 0.125 + 0.10 ~= 0.23 m`. The runtime sensor-rendered map inflates a
  little *higher* than the ideal surfaces (raycast discretization + probabilistic hit
  accumulation on tread edges), so 0.35 — fine on paper — still got engulfed on the
  climb; 0.55 keeps a ~0.32 m margin. It is **driven by `RISE`, not `RUN`** —
  widening the treads does not relax it.
- **Waypoint spacing >= ~1.2 m** — SCAN-Planner sizes the global min-snap duration by
  a heuristic that yields a short, high-accel profile for hops below
  `max_vel^2/max_acc = 1.125 m`, which then fails the dynamic-feasibility check. Every
  4 treads (`WP_TREAD_STEP`) keeps stops ~1.4 m apart while the near-linear ramp
  between them still clears each intermediate tread band.

## Caveat: run a single isolated launch

Concurrent launches on the same `ROS_DOMAIN_ID` collide on the shared topics this
stack uses (`/quad_0/body_pose`, `/map_generator/global_cloud`,
`/grid_map/occupancy_inflate`) and silently contaminate each other. The symptom is the
robot stalling mid-climb with "terminal point is in an obstacle" while a probe of those
topics shows *another* map's coordinates. Orphaned launches get reparented to
`systemd --user` and persist across sessions, so before trusting a run:

```bash
pgrep -af 'scan_planner|map_pub|open_loop|pcl_render'   # expect exactly one of each
ROS_DOMAIN_ID=7 ros2 launch scan_planner run_synthetic_multi.launch.py   # isolate
```

# Waypoint Studio (browser point-cloud viewer)

`waypoint_studio.html` is a self-contained, browser-based tool for loading a
`.pcd` map and marking waypoints by flying through it freely (inspired by
[superspl.at](https://superspl.at/)). It exports the exact same
`fsm.waypoints` YAML that `keypoint_recorder.py` produces, so it can be used to
hand-pick waypoints directly on a static map instead of driving the robot.

It is written in plain WebGL with **no external dependencies or CDN** — just
double-click the file and it runs offline in any modern browser.

## Run

Open `tools/waypoint_studio.html` in a browser. There are three ways to load a
file (a `.pcd` cloud **or** a `.yaml`/`.csv` waypoint set):

1. **🗂 Browse files…** button — an in-page file browser that lists the folders
   and loadable files served by the local http server; click a folder to open it,
   or a file to load it. This works even inside sandboxed/embedded preview
   browsers (needs http, not `file://`).
2. **Drag & drop** a `.pcd` (or `.yaml`/`.csv`) file anywhere onto the page.
3. **Path/URL field** — type a server-relative path (e.g. `/map.pcd` or
   `/src/planner/plan_manage/config/keypoints.multi.yaml`) and press **Load** or
   Enter. The loader picks cloud vs. waypoints by file extension.

> **Embedded/preview browsers:** some IDE webviews and sandboxed iframes block
> the *native* OS file-picker dialog. This tool therefore uses the in-page
> **🗂 Browse files…** browser (a same-origin fetch of the http server's directory
> listing) instead of a native picker, so loading works there too. If you open
> the page from `file://` (no server), use drag & drop.

Optionally serve the repo so the path field and the `?pcd=` auto-load shortcut
work. Prefer the bundled **no-cache** server — plain `http.server` sends no cache
headers, so the browser can keep serving a **stale** page after edits:

```bash
cd <repo root>
python3 tools/serve_nocache.py 8765     # sends Cache-Control: no-store
# (or plain `python3 -m http.server 8765`, but hard-reload after edits)
# then open:
#   http://127.0.0.1:8765/tools/waypoint_studio.html
#   http://127.0.0.1:8765/tools/waypoint_studio.html?pcd=/map.pcd
#   http://127.0.0.1:8765/tools/waypoint_studio.html?pcd=/src/simulator/map_generator/maps/goslam_xiaohao_hq_indoor_single_0.1m.pcd
```

The `?pcd=` and path-field values are relative to the server root and only work
over http; drag-and-drop works from `file://` too. Large clouds (e.g. the
2M-point `map.pcd`) parse in a couple of seconds — a "Parsing …" toast shows
while it works.

## Controls

| Input | Action |
| --- | --- |
| `W` / `S` | Fly forward / backward along the view direction |
| `A` / `D` | Strafe left / right (horizontal translation) |
| `Q` / `E` | Turn left / right (yaw, in place) |
| `Shift` | Speed boost |
| Pitch view (right-drag) + `W`/`S` | Ascend / descend (fly along the pitched view) |
| Right-drag | Look around freely (unlimited 360° yaw and pitch, rolls over past straight up/down) |
| Mouse wheel | Zoom in / out (dolly along the view direction) |
| Left-click | Mark a waypoint (snaps to the nearest cloud point) |
| `P` | Drop a waypoint at the crosshair |
| `Z` | Undo last waypoint |
| `L` | Fly to the selected waypoint |

Points are colored by height (Z) to help distinguish floors. Each waypoint is
editable in the left panel (per-axis number fields, ⌖ fly-to, 🗑 delete).

A small **axis gizmo** in the bottom-right corner shows the current world
orientation — red = **X**, green = **Y**, blue = **Z** — and updates live as you
turn/pitch the camera. An axis pointing toward you is drawn bright and on top;
one pointing away is dimmed.

## Model orientation

Some `.pcd` maps are stored with the wrong axis convention (e.g. Y-up instead of
the Z-up this tool and ROS assume). The **Model orientation** panel rotates the
cloud about the origin so you can fix it:

- **Rot X / Y / Z (degrees)** — rotation applied in the order X→Y→Z
  (`R = Rz·Ry·Rx`). Changes apply on commit (Enter / blur / spinner arrows).
- **Y-up→Z-up** — shortcut that sets `X = +90°`, turning a Y-up cloud upright.
- **Reset** — back to `0, 0, 0` (original PCD frame).
- **⟳ Re-frame** — re-center the camera on the (possibly rotated) cloud.

The rotation is baked into the point coordinates, so picking and the height
colormap follow it, and already-marked waypoints rotate together with the model.
Exported `fsm.waypoints` are therefore in the corrected frame.

## Load waypoints

**⬆ Load waypoints…** (in the Waypoints panel) opens the file browser filtered to
`.yaml`/`.csv` files, so you can re-open a previously exported set — or a
`keypoints.*.yaml` recorded with `keypoint_recorder.py` — and keep editing it:

- **YAML** — reads the flat `fsm.waypoints: [x, y, z, ...]` array (comment lines
  are ignored); this is exactly the recorder / `planner.yaml` format.
- **CSV** — reads `index,x,y,z` rows (a leading index/header column is skipped).

Loading **replaces** the current waypoint list. You can also drag & drop a
waypoints file, or type its path into the path/URL field.

## Export

- **⬇ keypoints.yaml** — `scan_planner_node → ros__parameters → fsm.waypoints`
  flat `[x, y, z, ...]` array in the map/world frame, identical to the recorder
  output. Drop it straight into `run.launch.py` with `keypoints_file:=...` when
  `navi_mode:=2`.
- **⬇ CSV** — `index,x,y,z` rows for spreadsheets or scripting.
- **Body-height Z offset** (checkbox + value, default **+0.30 m**) — because
  `navi_mode:=2` (PRESET_TARGET) consumes waypoint Z **verbatim** with no
  body-height offset, a Z snapped to the *floor surface* lands inside the
  inflated occupied band (`grid_map.obstacles_inflation_z_up/down`) and A\*
  rejects it as a goal → `EMERGENCY_STOP`. The offset lifts every **exported** Z
  to roughly body height so goals sit in free space; it affects exports only
  (the on-screen markers/list keep the clicked point). Uncheck to write raw Z.
  All exported numbers are forced to floats (`x.0`) so the `fsm.waypoints`
  sequence stays type-homogeneous — a lone integer among floats makes `rcl` fail
  to parse the file and `scan_planner_node` aborts at startup.

> The exported coordinates are in the PCD's own frame. For real-robot replay the
> map frame must match the runtime odometry frame — see the multi-floor waypoint
> notes in the top-level `README.md`.
