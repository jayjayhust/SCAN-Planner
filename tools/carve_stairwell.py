#!/usr/bin/env python3
"""Repair the xiaohao stairwell reconstruction by carving a descent corridor.

WHY: the GoSLAM scan goslam_xiaohao_hq_indoor_multi_0.1m.pcd physically compresses the
stairwell -- under SCAN-Planner's exact body-inflation criterion (double_cylinder_radius 0.25,
obstacles_inflation_z_up 0.1, z_down 0.4) the free corridor is only 0.22-0.40 m at W22-W24 and
BLOCKED at W33-W35, versus the 0.50 m the real 0.31 m Go2 body needs. Radius-outlier-removal does
NOT help (0-58 points removed): the narrowness is real, vertically-connected geometry, not noise.
The real-world remedy is to re-slam the stairwell; for simulation we instead CARVE a plausible
~1.1 m corridor so the planner + the 42-waypoint tour can be exercised at the REAL footprint.

WHAT: delete every point inside a tube around the W22->W36 descent centreline. Tube = lateral XY
radius R_LAT, and z within [path_z - Z_BELOW, path_z + Z_ABOVE] (>= the inflation band
[wz - 0.1, wz + 0.4] so the body is guaranteed free). Non-destructive: the source PCD is never
modified; only --write emits maps/goslam_xiaohao_hq_indoor_multi_0.1m_carved.pcd (the asset that
run_goslam_multi.launch.py loads). Without --write it just prints the before/after free-corridor
report so you can re-tune R_LAT / the centreline.

The freed corridor is verified with the SAME exact-inflation criterion the grid map uses, at the
real radius, so a reported width >= 0.50 m means a free body cell actually exists there.

Usage:  python3 tools/carve_stairwell.py            # dry run, report only
        python3 tools/carve_stairwell.py --write     # emit the repaired ..._carved.pcd
"""
import sys
import numpy as np

SRC = ("/home/jay/workspace/robotics/unitree/SCAN-Planner/src/simulator/"
       "map_generator/maps/goslam_xiaohao_hq_indoor_multi_0.1m.pcd")
DST = ("/home/jay/workspace/robotics/unitree/SCAN-Planner/src/simulator/"
       "map_generator/maps/goslam_xiaohao_hq_indoor_multi_0.1m_carved.pcd")

R_LAT, Z_BELOW, Z_ABOVE = 0.55, 0.22, 0.48
RB, Z_UP, Z_DOWN = 0.25, 0.1, 0.4   # REAL Go2 config used for verification

# descent centreline W22->W36 (W22-W24 use corridor-centred x; rest committed)
CENTERLINE = np.array([
    [-13.320, -9.689, 0.000], [-13.290, -10.306, -0.200], [-13.310, -10.923, -0.470],
    [-13.462, -11.540, -0.750], [-13.517, -12.579, -1.150], [-13.544, -13.107, -1.300],
    [-13.572, -13.635, -1.400], [-13.600, -14.163, -1.500], [-13.700, -14.100, -1.500],
    [-14.100, -14.100, -1.500], [-14.214, -14.176, -1.500], [-14.413, -13.158, -1.865],
    [-14.529, -12.566, -2.076], [-14.645, -11.975, -2.288], [-14.761, -11.383, -2.500],
])


def read_pcd(path):
    with open(path, "rb") as f:
        head = []
        while True:
            line = f.readline(); head.append(line)
            if line.decode("ascii", "replace").strip().upper().startswith("DATA"):
                break
        hdr = {}
        for line in head:
            t = line.decode("ascii", "replace").strip()
            if t and not t.startswith("#") and not t.upper().startswith(("VERSION", "DATA", "VIEWPOINT")):
                p = t.split(); hdr[p[0].upper()] = p[1:]
        npts = int(hdr["POINTS"][0])
        step = sum(int(s) * int(c) for s, c in zip(hdr["SIZE"], hdr.get("COUNT", [1] * len(hdr["SIZE"]))))
        buf = np.frombuffer(f.read(step * npts), dtype=np.uint8).reshape(npts, step)
        xyz = buf[:, 0:12].copy().view("<f4").reshape(npts, 3).astype(np.float64)
        return head, npts, step, buf, xyz


def free_slot(pts, wx, wy, wz, rb=RB, z_up=Z_UP, z_down=Z_DOWN):
    """Contiguous free x-run around wx under the exact inflation criterion (body half-width rb)."""
    X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]
    band = (np.abs(Y - wy) <= rb) & (Z >= wz - z_up) & (Z <= wz + z_down)
    ox = X[band]
    xs = np.arange(wx - 1.0, wx + 1.0 + 1e-9, 0.02)
    blocked = (np.any(np.abs(xs[:, None] - ox[None, :]) <= rb, axis=1)
               if ox.size else np.zeros(len(xs), bool))
    best, i = None, 0
    while i < len(xs):
        if not blocked[i]:
            j = i
            while j < len(xs) and not blocked[j]:
                j += 1
            lo, hi = xs[i], xs[j - 1]
            if lo - 0.30 <= wx <= hi + 0.30 and (best is None or (hi - lo) > (best[1] - best[0])):
                best = (lo, hi)
            i = j
        else:
            i += 1
    return best


def carve_mask(xyz):
    A, B = CENTERLINE[:-1], CENTERLINE[1:]
    d = B - A
    L2 = (d[:, 0] ** 2 + d[:, 1] ** 2)[:, None]
    P = xyz[:, None, :]                                   # K,1,3
    t = np.clip(((P[:, :, :2] - A[None, :, :2]) * d[None, :, :2]).sum(2) / np.where(L2 == 0, 1, L2).T, 0, 1)  # K,M
    proj_x = A[None, :, 0] + t * d[None, :, 0]
    proj_y = A[None, :, 1] + t * d[None, :, 1]
    proj_z = A[None, :, 2] + t * (B[None, :, 2] - A[None, :, 2])
    lat = np.hypot(P[:, :, 0] - proj_x, P[:, :, 1] - proj_y)          # K,M
    pz = xyz[:, 2][:, None]
    in_tube = (lat <= R_LAT) & (pz >= proj_z - Z_BELOW) & (pz <= proj_z + Z_ABOVE)
    return in_tube.any(axis=1)


CHECK = [("W22", -13.320, -9.689, 0.00), ("W23", -13.290, -10.306, -0.20),
         ("W24", -13.310, -10.923, -0.47), ("W25", -13.462, -11.540, -0.75),
         ("W30", -13.700, -14.100, -1.50), ("W33", -14.413, -13.158, -1.865),
         ("W34", -14.529, -12.566, -2.076), ("W35", -14.645, -11.975, -2.288)]


def report(tag, pts):
    print(f"--- {tag} ---")
    for name, wx, wy, wz in CHECK:
        s = free_slot(pts, wx, wy, wz)
        txt = f"[{s[0]:+.2f},{s[1]:+.2f}] w={s[1]-s[0]:.2f}" if s else "BLOCKED"
        ok = "OK" if (s and (s[1] - s[0]) >= 0.50) else "narrow"
        print(f"  {name} z={wz:+.2f}: {txt:30s} {ok}")


def main():
    head, npts, step, buf, xyz = read_pcd(SRC)
    mask = carve_mask(xyz)
    print(f"loaded {npts} pts; carving tube R_LAT={R_LAT} z[-{Z_BELOW},+{Z_ABOVE}] around W22->W36")
    print(f"removing {int(mask.sum())} pts ({100*mask.sum()/npts:.2f}% of map)\n")
    report("BEFORE (raw PCD)", xyz)
    print()
    report("AFTER carve", xyz[~mask])

    if "--write" in sys.argv:
        keep = ~mask
        new_head = []
        for line in head:
            t = line.decode("ascii", "replace").strip()
            if t.upper().startswith("WIDTH"):
                new_head.append(f"WIDTH {int(keep.sum())}\n".encode())
            elif t.upper().startswith("POINTS"):
                new_head.append(f"POINTS {int(keep.sum())}\n".encode())
            else:
                new_head.append(line if line.endswith(b"\n") else line + b"\n")
        with open(DST, "wb") as f:
            f.write(b"".join(new_head)); f.write(buf[keep].tobytes())
        print(f"\nwrote {DST} ({int(keep.sum())} pts)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
