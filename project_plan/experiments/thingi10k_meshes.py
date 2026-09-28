#!/usr/bin/env python3
"""Curates a set of Thingi10K meshes for the "clutter of objects in the
wild" experiment (E3e).

Deterministic pipeline: given a seed, a target count, and face-count bounds,
select well-behaved meshes from the Thingi10K metadata, download their STLs
from HuggingFace, convert them to welded/normalized OBJs (Drake collision
only supports .obj/.vtk), and validate each against the compliant
hydroelastic (barrier) pipeline via the mesh_preflight binary. Meshes that
fail download/conversion/preflight are skipped and the draw continues, so
the final set is deterministic given (seed, metadata, preflight behavior).

Usage:
    bazel build //examples/multibody/clutter:mesh_preflight
    python3 project_plan/experiments/thingi10k_meshes.py --seed 0 --count 10

Outputs (default --out project_plan/data/thingi10k_meshes):
    <out>/<file_id>.obj   welded, centered, scaled to --target-size
    <out>/manifest.json   selection parameters + per-mesh provenance
    <out>/raw/<id>.stl    download cache (not for commit)
"""

import argparse
import json
import random
import re
import struct
import subprocess
import sys
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE_URL = "https://huggingface.co/datasets/Thingi10K/Thingi10K/resolve/main"
METADATA_URL = f"{BASE_URL}/metadata/geometry_data.csv"


def download(url, dest):
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        return
    print(f"  downloading {url}")
    tmp = dest.with_suffix(dest.suffix + ".part")
    urllib.request.urlretrieve(url, tmp)
    tmp.rename(dest)


def load_metadata(out_dir):
    """Returns the well-behaved subset of the Thingi10K geometry metadata."""
    import csv
    csv_path = out_dir / "raw" / "geometry_data.csv"
    download(METADATA_URL, csv_path)
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def well_behaved(row):
    """Strict filter: watertight, manifold, oriented, solid, clean."""
    try:
        return (row["vertex_manifold"] == "1"
                and row["edge_manifold"] == "1"
                and row["oriented"] == "1"
                and row["PWN"] == "1"
                and row["solid"] == "1"
                and row["num_connected_components"] == "1"
                and row["num_boundary_edges"] == "0"
                and row["num_self_intersections"] == "0"
                and row["num_geometrical_degenerated_faces"] == "0"
                and row["num_duplicated_faces"] == "0")
    except KeyError:
        return False


# ----------------------------------------------------------------------------
# STL parsing (binary and ASCII) and OBJ output.
# ----------------------------------------------------------------------------

def read_stl(path):
    """Returns an (n, 3, 3) float64 array of facet vertices."""
    data = Path(path).read_bytes()
    # Binary detection: header + count must match the file size.
    if len(data) >= 84:
        (n,) = struct.unpack_from("<I", data, 80)
        if 84 + 50 * n == len(data):
            tri = np.frombuffer(data, dtype=np.uint8, count=50 * n, offset=84)
            tri = tri.reshape(n, 50)[:, 12:48].copy()  # skip normal, attrs
            return tri.view("<f4").reshape(n, 3, 3).astype(np.float64)
    # ASCII fallback.
    text = data.decode("ascii", errors="replace")
    verts = re.findall(
        r"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", text)
    arr = np.array(verts, dtype=np.float64)
    if arr.size == 0 or arr.shape[0] % 3 != 0:
        raise ValueError(f"cannot parse STL: {path}")
    return arr.reshape(-1, 3, 3)


def stl_to_obj(stl_path, obj_path, target_size):
    """Welds, centers, scales, and writes an OBJ.

    Returns (num_vertices, num_faces, bbox_extents_after_scale).
    Vertex order within each facet is preserved so the outward winding of the
    source STL survives.
    """
    tri = read_stl(stl_path)  # (n, 3, 3)
    flat = tri.reshape(-1, 3)

    # Weld exactly-coincident vertices (STL repeats vertices per facet).
    # Quantize to a tiny fraction of the bbox diagonal to absorb float32
    # noise from the binary format.
    lo, hi = flat.min(axis=0), flat.max(axis=0)
    diag = float(np.linalg.norm(hi - lo))
    if diag == 0:
        raise ValueError("degenerate STL: zero bounding box")
    quant = np.round(flat / (1e-9 * diag)).astype(np.int64)
    _, first_index, inverse = np.unique(
        quant, axis=0, return_index=True, return_inverse=True)
    vertices = flat[first_index]
    faces = inverse.reshape(-1, 3)

    # Drop degenerate faces (any repeated vertex index after welding).
    ok = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
          & (faces[:, 0] != faces[:, 2]))
    faces = faces[ok]

    # Center at the AABB center; uniform scale to the target max extent.
    lo, hi = vertices.min(axis=0), vertices.max(axis=0)
    vertices = vertices - (lo + hi) / 2
    scale = target_size / float((hi - lo).max())
    vertices = vertices * scale
    extents = (hi - lo) * scale

    with open(obj_path, "w") as f:
        f.write(f"# Converted from Thingi10K {Path(stl_path).name}\n")
        f.write(f"# welded vertices: {len(vertices)}, faces: {len(faces)}\n")
        for v in vertices:
            f.write(f"v {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n")
        for a, b, c in faces + 1:
            f.write(f"f {a} {b} {c}\n")
    return len(vertices), len(faces), extents.tolist()


# ----------------------------------------------------------------------------
# Selection
# ----------------------------------------------------------------------------

def draw_order(candidates, seed):
    """Deterministic round-robin draw across 4 face-count quartile buckets,
    so the selection spans the complexity range."""
    candidates = sorted(candidates, key=lambda r: int(r["file_id"]))
    faces = np.array([int(r["num_faces"]) for r in candidates])
    q = np.quantile(faces, [0.25, 0.5, 0.75])
    buckets = [[], [], [], []]
    for r, nf in zip(candidates, faces):
        buckets[int(np.searchsorted(q, nf))].append(r)
    rng = random.Random(seed)
    for b in buckets:
        rng.shuffle(b)
    order = []
    for i in range(max(len(b) for b in buckets)):
        for b in buckets:
            if i < len(b):
                order.append(b[i])
    return order


def preflight_ok(preflight, obj_path):
    if preflight is None:
        return True, "preflight skipped"
    out = subprocess.run([str(preflight), str(obj_path)],
                         capture_output=True, text=True, timeout=300)
    for line in out.stdout.splitlines():
        parts = line.split("\t")
        if len(parts) >= 2 and parts[1] == str(obj_path):
            return parts[0] == "OK", (parts[4] if len(parts) > 4 else "")
    return False, f"unparseable preflight output: {out.stdout!r}"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path,
                   default=ROOT / "data" / "thingi10k_meshes")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--count", type=int, default=10)
    p.add_argument("--min-faces", type=int, default=200)
    p.add_argument("--max-faces", type=int, default=5000)
    p.add_argument("--ids", type=str, default="",
                   help="Comma-separated file_ids; bypasses the random draw.")
    p.add_argument("--target-size", type=float, default=0.10,
                   help="Max AABB extent of the normalized mesh [m].")
    p.add_argument("--min-extent", type=float, default=0.0,
                   help="Reject meshes whose smallest normalized AABB extent "
                        "is below this [m]. Guards against degenerately thin "
                        "sheets that are guaranteed to pierce the barrier "
                        "layer on impact (thinness itself is desirable; only "
                        "sub-millimeter degeneracy is not).")
    p.add_argument("--preflight", type=Path,
                   default=ROOT.parent /
                   "bazel-bin/examples/multibody/clutter/mesh_preflight")
    args = p.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    preflight = args.preflight if args.preflight.exists() else None
    if preflight is None:
        print(f"WARNING: preflight binary not found at {args.preflight}; "
              "meshes will NOT be validated against the inflation pipeline!",
              file=sys.stderr)

    rows = load_metadata(args.out)
    by_id = {r["file_id"]: r for r in rows}
    if args.ids:
        order = [by_id[i.strip()] for i in args.ids.split(",")]
    else:
        candidates = [r for r in rows if well_behaved(r)
                      and args.min_faces <= int(r["num_faces"])
                      <= args.max_faces]
        print(f"{len(candidates)} well-behaved meshes with "
              f"{args.min_faces}..{args.max_faces} faces")
        order = draw_order(candidates, args.seed)

    selected, skipped = [], []
    for row in order:
        if len(selected) >= args.count:
            break
        fid = row["file_id"]
        stl = args.out / "raw" / f"{fid}.stl"
        obj = args.out / f"{fid}.obj"
        try:
            download(f"{BASE_URL}/raw_meshes/{fid}.stl", stl)
            nv, nf, extents = stl_to_obj(stl, obj, args.target_size)
            if min(extents) < args.min_extent:
                raise RuntimeError(
                    f"min extent {min(extents):.4f} m < {args.min_extent} m")
            ok, msg = preflight_ok(preflight, obj)
            if not ok:
                raise RuntimeError(f"preflight: {msg}")
        except Exception as e:  # noqa: BLE001 - skip-and-continue by design
            print(f"  SKIP {fid}: {e}")
            skipped.append({"file_id": fid, "reason": str(e)})
            obj.unlink(missing_ok=True)
            continue
        print(f"  OK   {fid}: {nv} verts, {nf} faces")
        selected.append({
            "file_id": fid,
            "obj": obj.name,
            "source_url": f"{BASE_URL}/raw_meshes/{fid}.stl",
            "num_vertices": nv,
            "num_faces": nf,
            "bbox_extents_m": [round(e, 6) for e in extents],
            "metadata_num_faces": int(row["num_faces"]),
        })

    if len(selected) < args.count:
        print(f"ERROR: only {len(selected)}/{args.count} meshes passed.",
              file=sys.stderr)
        return 1

    manifest = {
        "seed": args.seed,
        "count": args.count,
        "min_faces": args.min_faces,
        "max_faces": args.max_faces,
        "target_size_m": args.target_size,
        "min_extent_m": args.min_extent,
        "fixed_ids": args.ids or None,
        "dataset": "Thingi10K (https://ten-thousand-models.appspot.com/)",
        "well_behaved_filter": ("vertex_manifold & edge_manifold & oriented "
                                "& PWN & solid & 1 component & no boundary "
                                "edges & no self-intersections & no "
                                "degenerate/duplicated faces"),
        "preflight": str(args.preflight) if preflight else None,
        "meshes": selected,
        "skipped": skipped,
    }
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"Wrote {args.out / 'manifest.json'} ({len(selected)} meshes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
