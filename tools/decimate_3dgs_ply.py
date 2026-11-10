#!/usr/bin/env python3
"""Decimate a 3D Gaussian Splatting .ply by keeping every Nth splat.

The xiaohao full-resolution scan (``..._0.02m_3dgs.ply``, ~10 M splats, 2.30 GiB)
is too big for tools/waypoint_studio.html: the page fetches a map into a single
ArrayBuffer and browsers refuse one that large (RangeError). This tool streams
the source file and copies every Nth vertex record VERBATIM — all 62 properties
(centers, SH coefficients, scale, rotation, opacity) untouched — into a new .ply
whose header is identical except for the rewritten ``element vertex`` count.
The result stays a valid 3DGS model in the exact same world frame, loadable both
by the waypoint studio and by external splat viewers (e.g. SuperSplat).

Usage:
    python3 tools/decimate_3dgs_ply.py INPUT.ply STRIDE [-o OUTPUT.ply] [--force]

Examples:
    # ~10 M -> ~1.25 M splats, 2.30 GiB -> ~295 MiB (loads in the studio)
    python3 tools/decimate_3dgs_ply.py src/simulator/3dgs_map/GoSLAM_*_0.02m_3dgs.ply 8

Only the vertex element is decimated; any bytes after it (extra elements such as
faces) are copied through unchanged. Vertex elements containing a
``property list`` (variable stride) and elements declared before ``vertex`` are
rejected, since neither allows fixed-stride record copying.
"""
import argparse
import shutil
import sys
from pathlib import Path

# PLY scalar type -> byte size (vertex records are fixed-stride; lists rejected)
SIZE = {
    "char": 1, "uchar": 1, "int8": 1, "uint8": 1,
    "short": 2, "ushort": 2, "int16": 2, "uint16": 2,
    "int": 4, "uint": 4, "int32": 4, "uint32": 4, "float": 4, "float32": 4,
    "double": 8, "float64": 8,
}
CHUNK_BYTES = 1 << 26  # read the vertex block 64 MiB of records at a time


class Header:
    __slots__ = ("lines", "fmt", "count", "stride", "vertex_line", "data_off")


def parse_header(fh):
    """Read lines through end_header; return Header with vertex count/stride."""
    h = Header()
    h.lines, h.fmt, h.count, h.stride = [], None, None, 0
    h.vertex_line = None
    in_vertex = False
    while True:
        line = fh.readline()
        if not line:
            raise ValueError("unexpected EOF: no end_header in PLY header")
        h.lines.append(line)
        parts = line.decode("latin-1").split()
        if not parts:
            continue
        key = parts[0]
        if key == "ply" and len(h.lines) == 1:
            continue
        if key == "format":
            h.fmt = parts[1]
        elif key == "element":
            if parts[1] == "vertex":
                if h.count is not None:
                    raise ValueError("multiple vertex elements are not supported")
                in_vertex = True
                h.count = int(parts[2])
                h.vertex_line = len(h.lines) - 1
            else:
                if h.count is None:
                    raise ValueError("element %r declared before vertex" % parts[1])
                in_vertex = False
        elif key == "property" and in_vertex:
            if parts[1] == "list":
                raise ValueError('vertex "property list" has no fixed stride; cannot decimate')
            if parts[1] not in SIZE:
                raise ValueError("unknown vertex property type %r" % parts[1])
            h.stride += SIZE[parts[1]]
        elif key == "end_header":
            break
    if h.count is None:
        raise ValueError("header declares no vertex element")
    if h.fmt not in ("ascii", "binary_little_endian", "binary_big_endian"):
        raise ValueError("unsupported PLY format %r" % h.fmt)
    h.data_off = sum(len(l) for l in h.lines)
    return h


def rewrite_vertex_count(h, kept):
    """Replace the 'element vertex N' line, preserving its original line ending."""
    old = h.lines[h.vertex_line]
    nl = old[len(old.rstrip(b"\r\n")):]
    h.lines[h.vertex_line] = ("element vertex %d" % kept).encode("latin-1") + nl


def decimate(src, dst, k):
    with open(src, "rb") as fh, open(dst, "wb") as out:
        h = parse_header(fh)
        kept = (h.count + k - 1) // k
        rewrite_vertex_count(h, kept)
        out.writelines(h.lines)

        if h.fmt == "ascii":
            for i in range(h.count):
                line = fh.readline()
                if not line:
                    raise ValueError("ascii payload truncated at vertex %d/%d" % (i, h.count))
                if i % k == 0:
                    out.write(line)
        else:
            s = h.stride
            chunk_recs = max(1, CHUNK_BYTES // s)
            idx = 0
            last_pct = -1
            while idx < h.count:
                n = min(chunk_recs, h.count - idx)
                buf = fh.read(n * s)
                if len(buf) != n * s:
                    raise ValueError("binary payload truncated at vertex %d/%d" % (idx, h.count))
                mv = memoryview(buf)
                j0 = (k - idx % k) % k  # first record in this chunk whose global index % k == 0
                out.write(b"".join(mv[j * s:(j + 1) * s] for j in range(j0, n, k)))
                idx += n
                pct = int(100 * idx / h.count)
                if pct // 10 != last_pct // 10:
                    last_pct = pct
                    print("\r  copying splats %3d%% (%d/%d)" % (pct, idx, h.count),
                          end="", flush=True)
            print()
        shutil.copyfileobj(fh, out, 1 << 24)  # any non-vertex elements, verbatim
        return h, kept


def main():
    ap = argparse.ArgumentParser(
        description="Keep every Nth splat of a 3DGS .ply, verbatim, into a smaller .ply.")
    ap.add_argument("input", type=Path, help="source .ply (3DGS or plain colored)")
    ap.add_argument("stride", type=int, help="keep one splat every N (8 -> 1/8 of the splats)")
    ap.add_argument("-o", "--output", type=Path, default=None,
                    help="output path (default: <input stem>_stride<N>.ply next to the input)")
    ap.add_argument("--force", action="store_true", help="overwrite an existing output file")
    args = ap.parse_args()

    if args.stride < 1:
        ap.error("stride must be >= 1")
    src = args.input
    if not src.is_file():
        ap.error("input not found: %s" % src)
    dst = args.output or src.with_name("%s_stride%d%s" % (src.stem, args.stride, src.suffix))
    if dst.exists() and not args.force:
        ap.error("output exists (pass --force to overwrite): %s" % dst)

    print("decimating %s (stride %d) -> %s" % (src, args.stride, dst))
    h, kept = decimate(src, dst, args.stride)
    nbytes = dst.stat().st_size
    print("  kept %d of %d splats (stride %d, %s, %d-byte records)"
          % (kept, h.count, args.stride, h.fmt, h.stride))
    print("  wrote %s (%.2f GiB)" % (dst, nbytes / 2 ** 30))
    if nbytes > 1.5 * 2 ** 30:
        print("  WARNING: still above ~1.5 GiB; the browser studio may refuse to",
              "allocate it — use a larger stride.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
