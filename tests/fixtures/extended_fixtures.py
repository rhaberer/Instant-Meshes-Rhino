"""Generate local Instant Meshes examples and a manifest of test settings."""

import argparse
from collections import Counter
from hashlib import sha256
import json
from math import cos, pi, sin
from pathlib import Path


def torus(major_segments, minor_segments):
    positions, normals, faces = [], [], []
    for i in range(major_segments):
        u = 2 * pi * i / major_segments
        for j in range(minor_segments):
            v = 2 * pi * j / minor_segments
            radius = 1.1 + 0.32 * cos(v)
            positions.append((radius * cos(u), radius * sin(u), 0.32 * sin(v)))
            normals.append((cos(u) * cos(v), sin(u) * cos(v), sin(v)))

            a = i * minor_segments + j
            b = ((i + 1) % major_segments) * minor_segments + j
            c = i * minor_segments + (j + 1) % minor_segments
            d = ((i + 1) % major_segments) * minor_segments + (j + 1) % minor_segments
            if (i + j) % 2:
                faces.extend(((a, b, d), (a, d, c)))
            else:
                faces.extend(((a, b, c), (b, d, c)))
    return positions, faces, normals


def angle_bracket():
    # A solid L with a concave inside corner and a triangulated flat exterior.
    occupied = {(x, y, z) for x in range(12) for y in range(8) for z in range(10)
                if x < 3 or z < 3}
    positions, faces, vertices = [], [], {}
    for cell in sorted(occupied):
        for axis in range(3):
            u, v = (axis + 1) % 3, (axis + 2) % 3
            for direction in (-1, 1):
                neighbor = list(cell)
                neighbor[axis] += direction
                if tuple(neighbor) in occupied:
                    continue
                corners = []
                for du, dv in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    point = list(cell)
                    point[axis] += direction > 0
                    point[u] += du
                    point[v] += dv
                    key = tuple(point)
                    if key not in vertices:
                        vertices[key] = len(positions)
                        positions.append(tuple(component / 12 for component in point))
                    corners.append(vertices[key])
                a, b, c, d = corners
                if direction < 0:
                    b, d = d, b
                faces.extend(((a, b, c), (a, c, d)))
    return positions, faces


def nearby_sheets(columns=28, rows=18):
    # Two disconnected surfaces with overlapping footprints and a small gap.
    positions, faces = [], []
    for sheet in range(2):
        start = len(positions)
        for j in range(rows + 1):
            for i in range(columns + 1):
                u, v = (i / columns) ** 1.4, j / rows
                z = 0.16 * sin(3 * u + v) + sheet * 0.014
                positions.append((u, v, z))
        for j in range(rows):
            for i in range(columns):
                a = start + j * (columns + 1) + i
                b = a + 1
                c = a + columns + 1
                faces.extend(((a, b, c), (b, c + 1, c)))
    return positions, faces


def write_obj(path, positions, faces):
    with path.open("w", encoding="ascii") as out:
        for x, y, z in positions:
            out.write(f"v {x:.12g} {y:.12g} {z:.12g}\n")
        for a, b, c in faces:
            out.write(f"f {a + 1} {b + 1} {c + 1}\n")


def write_cloud(path, positions, normals):
    with path.open("w", encoding="ascii") as out:
        out.write(f"ply\nformat ascii 1.0\nelement vertex {len(positions)}\n")
        for name in ("x", "y", "z", "nx", "ny", "nz"):
            out.write(f"property float {name}\n")
        out.write("end_header\n")
        for position, normal in zip(positions, normals):
            out.write(" ".join(f"{value:.12g}" for value in (*position, *normal)) + "\n")


def mesh_entry(path, positions, faces, cases, purpose, boundary_edges=None):
    directed = Counter((a, b) for x, y, z in faces for a, b in ((x, y), (y, z), (z, x)))
    if any(count != 1 for count in directed.values()):
        raise ValueError(f"{path.name} contains duplicated directed edges.")
    boundaries = sum((b, a) not in directed for a, b in directed)
    if boundary_edges is not None and boundaries != boundary_edges:
        raise ValueError(f"{path.name} has unexpected boundary edges.")
    write_obj(path, positions, faces)
    return {
        "kind": "triangle-mesh", "vertices": len(positions), "triangles": len(faces),
        "boundary_edges": boundaries, "purpose": purpose, "cases": cases,
        "sha256": sha256(path.read_bytes()).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output
    positions, faces, normals = torus(48, 20)
    bracket_positions, bracket_faces = angle_bracket()
    sheets_positions, sheets_faces = nearby_sheets()
    dense_positions, dense_faces, _ = torus(240, 144)

    manifest = {
        "torus.obj": mesh_entry(output / "torus.obj", positions, faces, [
            {"name": "pure-quads", "target_vertices": 240, "flags": "-r 4 -p 4 -S 2"},
            {"name": "triangles", "target_vertices": 240, "flags": "-r 6 -p 6 -S 2"},
        ], "Closed genus-one surface with inner and outer curvature", 0),
        "angle-bracket.obj": mesh_entry(output / "angle-bracket.obj", bracket_positions, bracket_faces, [
            {"name": "crease", "target_vertices": 240, "flags": "-c 35 -D -S 2"},
        ], "Convex edges and a concave right-angle corner for crease preservation", 0),
        "nearby-sheets.obj": mesh_entry(output / "nearby-sheets.obj", sheets_positions, sheets_faces, [
            {"name": "open-boundaries", "target_vertices": 220, "flags": "-b -D -S 2"},
            {"name": "without-reprojection", "target_vertices": 220, "flags": "-b -D -S 0"},
        ], "Two nearby disconnected surfaces, uneven sampling and open boundaries", 184),
        "torus-dense.obj": mesh_entry(output / "torus-dense.obj", dense_positions, dense_faces, [
            {"name": "performance", "target_vertices": 2000, "flags": "-r 4 -p 4 -S 2"},
        ], "69,120 triangles for measuring solver time and memory as mesh size increases", 0),
    }
    path = output / "torus-points.ply"
    write_cloud(path, positions, normals)
    manifest[path.name] = {
        "kind": "oriented-point-cloud", "points": len(positions),
        "purpose": "Normal-aware nearest-neighbor and point-cloud reconstruction path",
        "cases": [{"name": "point-cloud", "target_vertices": 160, "flags": "-k 10 -D -S 2"}],
        "sha256": sha256(path.read_bytes()).hexdigest(),
    }
    (output / "fixtures.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    for name, entry in manifest.items():
        print(f"{name}: {entry['purpose']}")


if __name__ == "__main__":
    main()
