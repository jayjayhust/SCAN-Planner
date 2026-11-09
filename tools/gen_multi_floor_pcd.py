#!/usr/bin/env python3
"""Procedurally generate a clean two-floor PCD map + matching waypoint tour.

This builds a deterministic, noise-free multi-floor scene for exercising
SCAN-Planner's basic floor-to-floor logic (A* + B-spline + PRESET_TARGET FSM)
without the difficulty of the real GoSLAM indoor scan.

Scene (world frame, Z up):
  * Floor 1 slab top ......... z = 0.00  (matches grid_map.ground_height)
  * Floor 2 slab top ......... z = 2.50  (slab occupies z in [2.30, 2.50])
  * Roof underside ........... z = 4.80
  * Footprint ................ x in [-8, 8], y in [-8, 8]
  * Straight staircase ....... x in [-2, 2], ascending +Y from y=-2.5 (z=0) to
                               y=+4.5 (z=2.5), 20 risers (rise 0.125, run 0.35).
                               A gentle ~19.6 deg slope keeps every consecutive
                               waypoint pair a short, locally-feasible ramp for
                               SCAN-Planner's 2.5D A* front-end.
  * Stairwell opening ........ the same x/y box is cut out of the floor-2 slab
                               so the stairs connect the two levels; a railing
                               guards the opening's south edge on floor 2.

Waypoints follow the project's PRESET_TARGET convention: the goal Z must sit at
tread/floor surface + BODY_OFFSET so the whole double-cylinder body footprint
(two 0.25 m-radius cylinders at pos +/- 0.18 m along the heading) lands in free
space above the inflated occupied band [surface - 0.4, surface + 0.1] instead of
inside it. The binding case is the FRONT cylinder reaching the next riser up, so
the static minimum offset is RISE + z_up = 0.125 + 0.10 ~= 0.23 m (found by
replicating the inflation + double-cylinder check against this very cloud). The runtime sensor-rendered
map inflates a little higher than the ideal surfaces (raycast discretization +
probabilistic hit accumulation on tread edges), which is why a 0.35 offset that
looked fine statically still got engulfed on the climb; BODY_OFFSET = 0.55 keeps
a ~0.32 m margin. Consecutive waypoints are kept >= ~1.2 m apart: SCAN-Planner
sizes the global min-snap duration by a heuristic (planner_manager.cpp) that
yields a short, high-accel profile for hops below max_vel^2/max_acc = 1.125 m,
which then fails the dynamic-feasibility check (acc limit = max_acc +
acc_tolerance). Stair stops are placed every 4 treads: far enough to be feasible,
close enough that the near-linear ramp between them still clears every
intermediate tread band.

Outputs (relative to the repo root):
  src/simulator/map_generator/maps/synthetic_two_floor_0.05m.pcd     (binary PCD)
  src/simulator/map_generator/configs/keypoints.synthetic.yaml       (fsm.waypoints)
"""

import os
import struct
import sys

import numpy as np

# --------------------------------------------------------------------------- #
# Geometry constants (single source of truth for both the PCD and waypoints)
# --------------------------------------------------------------------------- #
X_MIN, X_MAX = -8.0, 8.0
Y_MIN, Y_MAX = -8.0, 8.0

FLOOR1_Z = 0.0            # floor-1 top surface
FLOOR2_Z = 2.5            # floor-2 top surface
SLAB_UNDER = 2.30         # floor-2 slab underside == floor-1 ceiling
ROOF_Z = 4.80             # roof underside (floor-2 ceiling)
WALL_TOP = 5.00

STAIR_X0, STAIR_X1 = -2.0, 2.0   # stairwell / stair width
STAIR_Y0, STAIR_Y1 = -2.5, 4.5   # stair run (south -> north), 7 m
N_RISERS = 20
RISE = (FLOOR2_Z - FLOOR1_Z) / N_RISERS   # 0.125
RUN = (STAIR_Y1 - STAIR_Y0) / N_RISERS    # 0.35

BODY_OFFSET = 0.55        # waypoint Z = surface Z + BODY_OFFSET (see module docstring)
WP_TREAD_STEP = 4         # one stair waypoint every N treads (>= ~1.2 m apart)
APPROACH_CLEAR = 0.8      # floor-1 entry waypoint this far south of the riser

H_SPACING = 0.05          # horizontal surface sampling
V_SPACING = 0.06          # vertical surface sampling

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCD_OUT = os.path.join(
    REPO_ROOT,
    "src/simulator/map_generator/maps/synthetic_two_floor_0.05m.pcd",
)
YAML_OUT = os.path.join(
    REPO_ROOT, "src/simulator/map_generator/configs/keypoints.synthetic.yaml"
)


# --------------------------------------------------------------------------- #
# Point-sampling helpers
# --------------------------------------------------------------------------- #
def _linspace(a, b, step):
    n = max(int(round((b - a) / step)) + 1, 2)
    return np.linspace(a, b, n)


def horizontal(x0, x1, y0, y1, z, spacing=H_SPACING):
    """Grid of points on the plane Z = z spanning [x0,x1] x [y0,y1]."""
    xs = _linspace(x0, x1, spacing)
    ys = _linspace(y0, y1, spacing)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    gz = np.full_like(gx, z)
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


def wall_x(x, y0, y1, z0, z1, spacing=V_SPACING):
    """Vertical plane at constant X."""
    ys = _linspace(y0, y1, spacing)
    zs = _linspace(z0, z1, spacing)
    gy, gz = np.meshgrid(ys, zs, indexing="ij")
    gx = np.full_like(gy, x)
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


def wall_y(y, x0, x1, z0, z1, spacing=V_SPACING):
    """Vertical plane at constant Y."""
    xs = _linspace(x0, x1, spacing)
    zs = _linspace(z0, z1, spacing)
    gx, gz = np.meshgrid(xs, zs, indexing="ij")
    gy = np.full_like(gx, y)
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


def in_stairwell(x, y):
    return (STAIR_X0 <= x <= STAIR_X1) and (STAIR_Y0 <= y <= STAIR_Y1)


def build_cloud():
    parts = []

    # Floor 1 (top surface only; ground_height == 0).
    parts.append(horizontal(X_MIN, X_MAX, Y_MIN, Y_MAX, FLOOR1_Z))

    # Floor 2 slab: top + underside, with the stairwell opening cut out.
    f2_top = horizontal(X_MIN, X_MAX, Y_MIN, Y_MAX, FLOOR2_Z)
    f2_und = horizontal(X_MIN, X_MAX, Y_MIN, Y_MAX, SLAB_UNDER)
    for slab in (f2_top, f2_und):
        keep = np.array(
            [not in_stairwell(p[0], p[1]) for p in slab], dtype=bool
        )
        parts.append(slab[keep])

    # Roof underside (floor-2 ceiling).
    parts.append(horizontal(X_MIN, X_MAX, Y_MIN, Y_MAX, ROOF_Z))

    # Outer perimeter walls (full height, both floors).
    parts.append(wall_x(X_MIN, Y_MIN, Y_MAX, FLOOR1_Z, WALL_TOP))
    parts.append(wall_x(X_MAX, Y_MIN, Y_MAX, FLOOR1_Z, WALL_TOP))
    parts.append(wall_y(Y_MIN, X_MIN, X_MAX, FLOOR1_Z, WALL_TOP))
    parts.append(wall_y(Y_MAX, X_MIN, X_MAX, FLOOR1_Z, WALL_TOP))

    # Stairwell side walls (also act as floor-2 railings along the opening).
    parts.append(wall_x(STAIR_X0, STAIR_Y0, STAIR_Y1, FLOOR1_Z, ROOF_Z))
    parts.append(wall_x(STAIR_X1, STAIR_Y0, STAIR_Y1, FLOOR1_Z, ROOF_Z))
    # Railing across the opening's south edge on floor 2 (north edge is the exit).
    parts.append(wall_y(STAIR_Y0, STAIR_X0, STAIR_X1, FLOOR2_Z, FLOOR2_Z + 1.0))

    # Staircase: tread tops (horizontal) + risers (vertical, facing south).
    for k in range(1, N_RISERS + 1):
        z_top = FLOOR1_Z + k * RISE
        y_s = STAIR_Y0 + (k - 1) * RUN
        y_n = STAIR_Y0 + k * RUN
        parts.append(horizontal(STAIR_X0, STAIR_X1, y_s, y_n, z_top, spacing=H_SPACING))
        parts.append(wall_y(y_s, STAIR_X0, STAIR_X1, z_top - RISE, z_top))

    cloud = np.concatenate(parts, axis=0).astype(np.float32)
    return cloud


def write_pcd_binary(path, pts):
    n = pts.shape[0]
    header = (
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.astype("<f4").tobytes())


# --------------------------------------------------------------------------- #
# Waypoint tour (derived from the same stair geometry)
# --------------------------------------------------------------------------- #
def tread_waypoint(k, x=0.0):
    """Waypoint over tread k: y at tread centre, z = tread top + BODY_OFFSET."""
    y = STAIR_Y0 + (k - 0.5) * RUN
    z = FLOOR1_Z + k * RISE + BODY_OFFSET
    return (x, y, z)


def build_waypoints():
    floor1_z = FLOOR1_Z + BODY_OFFSET
    floor2_z = FLOOR2_Z + BODY_OFFSET
    # Floor-1 approach, then a stair stop every WP_TREAD_STEP treads so each
    # segment is a short near-linear ramp that clears every intermediate tread.
    wps = [(0.0, STAIR_Y0 - APPROACH_CLEAR, floor1_z)]   # floor 1, clear of the riser
    for k in range(WP_TREAD_STEP, N_RISERS + 1, WP_TREAD_STEP):
        wps.append(tread_waypoint(k))
    if (N_RISERS % WP_TREAD_STEP) != 0:        # ensure the top tread is included
        wps.append(tread_waypoint(N_RISERS))
    wps.extend([
        (0.0, STAIR_Y1 + 1.0, floor2_z),       # floor 2, north of the stairwell
        (-2.0, STAIR_Y1 + 1.0, floor2_z),
        (-4.0, STAIR_Y1 + 1.0, floor2_z),      # floor 2 west end (goal)
    ])
    return wps


SPAWN = (0.0, -4.8, FLOOR1_Z + BODY_OFFSET, 1.5708)  # x, y, z, yaw (facing +Y)


def write_yaml(path, wps):
    flat = []
    for x, y, z in wps:
        flat.extend([f"{float(x):.3f}", f"{float(y):.3f}", f"{float(z):.3f}"])
    body = ", ".join(flat)
    text = (
        "# Synthetic two-floor tour for navi_mode:=2 (PRESET_TARGET).\n"
        "#\n"
        "# Auto-generated by tools/gen_multi_floor_pcd.py against\n"
        f"# maps/synthetic_two_floor_0.05m.pcd. Waypoint Z = surface + {BODY_OFFSET} m so\n"
        "# each goal sits in free space above the inflated occupied band\n"
        "# [surface - 0.4, surface + 0.1]. Flat [x, y, z, ...] in the world frame.\n"
        "#\n"
        f"# Route: floor 1 -> straight staircase (a stop every {WP_TREAD_STEP} treads) -> floor 2 west end.\n"
        f"# Spawn (controllers.synthetic.yaml open_loop init): {SPAWN}\n"
        "scan_planner_node:\n"
        "  ros__parameters:\n"
        f"    fsm.waypoints: [{body}]\n"
    )
    with open(path, "w") as f:
        f.write(text)


def main():
    cloud = build_cloud()
    os.makedirs(os.path.dirname(PCD_OUT), exist_ok=True)
    write_pcd_binary(PCD_OUT, cloud)

    wps = build_waypoints()
    os.makedirs(os.path.dirname(YAML_OUT), exist_ok=True)
    write_yaml(YAML_OUT, wps)

    zmin, zmax = float(cloud[:, 2].min()), float(cloud[:, 2].max())
    print(f"PCD  : {PCD_OUT}")
    print(f"       {cloud.shape[0]} points, z in [{zmin:.2f}, {zmax:.2f}]")
    print(f"YAML : {YAML_OUT}")
    print(f"       {len(wps)} waypoints:")
    for i, (x, y, z) in enumerate(wps):
        print(f"         W{i}: ({x:6.2f}, {y:6.2f}, {z:5.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
