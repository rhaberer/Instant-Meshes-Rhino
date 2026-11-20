"""Convert three downloaded research models to shared triangle-only OBJ fixtures."""

import argparse
import gzip
from hashlib import sha256
import json
from pathlib import Path
import struct
import tarfile
import zipfile

STANFORD = "https://graphics.stanford.edu/data/3Dscanrep/"
CMU = "https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/"


def vertex(out, x, y, z):
    out.write(f"v {float(x):.12g} {float(y):.12g} {float(z):.12g}\n")


def triangle(out, a, b, c):
    out.write(f"f {int(a) + 1} {int(b) + 1} {int(c) + 1}\n")


def spot(source, out):
    n_vertices = n_faces = 0
    with zipfile.ZipFile(source) as archive:
        with archive.open("spot/spot_triangulated.obj") as mesh:
            for line in mesh:
                fields = line.split()
                if not fields:
                    continue
                if fields[0] == b"v":
                    vertex(out, *fields[1:4])
                    n_vertices += 1
                elif fields[0] == b"f":
                    assert len(fields) == 4
                    triangle(out, *(int(token.split(b"/")[0]) - 1 for token in fields[1:]))
                    n_faces += 1
    return n_vertices, n_faces


def ply_header(stream):
    assert stream.readline().strip() == b"ply"
    header = {"elements": {}, "properties": {}}
    name = None
    for line in stream:
        fields = line.split()
        if fields[0] == b"end_header":
            return header
        if fields[0] == b"format":
            header["format"] = fields[1]
        elif fields[0] == b"element":
            name = fields[1]
            header["elements"][name] = int(fields[2])
            header["properties"][name] = []
        elif fields[0] == b"property":
            header["properties"][name].append(tuple(fields[1:]))
    raise ValueError("Incomplete PLY header.")


def bunny(source, out):
    with tarfile.open(source, "r:gz") as archive:
        # Read this known member without extracting any filesystem paths from the archive.
        with archive.extractfile("bunny/reconstruction/bun_zipper.ply") as mesh:
            header = ply_header(mesh)
            assert header["format"] == b"ascii"
            n_vertices, n_faces = (header["elements"][name] for name in (b"vertex", b"face"))
            for _ in range(n_vertices):
                vertex(out, *mesh.readline().split()[:3])
            for _ in range(n_faces):
                arity, *indices = mesh.readline().split()
                assert arity == b"3"
                triangle(out, *indices)
    return n_vertices, n_faces


def armadillo(source, out):
    with gzip.open(source, "rb") as mesh:
        header = ply_header(mesh)
        assert header["format"] == b"binary_big_endian"
        assert header["properties"][b"vertex"] == [(b"float", axis) for axis in (b"x", b"y", b"z")]
        assert header["properties"][b"face"] == [(b"uchar", b"intensity"),
                                                 (b"list", b"uchar", b"int", b"vertex_indices")]
        n_vertices, n_faces = (header["elements"][name] for name in (b"vertex", b"face"))
        for _ in range(n_vertices):
            vertex(out, *struct.unpack(">fff", mesh.read(12)))
        for _ in range(n_faces):
            _, arity, a, b, c = struct.unpack(">BBiii", mesh.read(14))
            assert arity == 3
            triangle(out, a, b, c)
    return n_vertices, n_faces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archives", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {}
    models = (
        ("spot", "spot.zip", CMU + "spot.zip", CMU, "Keenan Crane", "CC0 1.0",
         "Clean organic geometry with small features", 500, spot),
        ("stanford-bunny", "bunny.tar.gz", "https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz",
         STANFORD, "Stanford Computer Graphics Laboratory, Greg Turk and Marc Levoy",
         "Stanford repository terms. Research use with attribution. Commercial use requires permission.",
         "Scanned surface with five open boundaries", 2500, bunny),
        ("stanford-armadillo", "Armadillo.ply.gz",
         "https://graphics.stanford.edu/pub/3Dscanrep/armadillo/Armadillo.ply.gz",
         STANFORD, "Stanford Computer Graphics Laboratory",
         "Stanford repository terms. Research use with attribution. Commercial use requires permission.",
         "High-density scan with limbs and fine surface detail", 7000, armadillo),
    )
    for name, archive, source_url, provider, attribution, terms, purpose, target, convert in models:
        source = args.archives / archive
        target_path = args.output / (name + ".obj")
        with target_path.open("w", encoding="ascii") as out:
            n_vertices, n_faces = convert(source, out)
        manifest[target_path.name] = {
            "source": source_url, "provider": provider, "attribution": attribution, "terms": terms,
            "source_sha256": sha256(source.read_bytes()).hexdigest(),
            "sha256": sha256(target_path.read_bytes()).hexdigest(),
            "kind": "triangle-mesh", "vertices": n_vertices, "triangles": n_faces, "purpose": purpose,
            "cases": [{"name": "pure-quads", "target_vertices": target, "flags": "-r 4 -p 4 -S 2"}],
        }
        print(f"{name}: {n_vertices:,} vertices and {n_faces:,} triangles")
    (args.output / "fixtures.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
