"""Measure the released solver across the repository's benchmark fixtures."""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import platform
import shutil
import sys
from tempfile import TemporaryDirectory

from benchmark import execute_case
from run import (Configuration, PORT_REVISION, ROOT, creased_patch, grid,
                 instrument_cli, run, sphere, write_obj, write_oriented_points)

sys.path.insert(0, str(ROOT / "tests" / "fixtures"))
from extended_fixtures import torus, write_obj as write_fixture_obj


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def benchmark_cases(folder, providers, dense_patch):
    fixtures = ROOT / "tests" / "fixtures" / "generated"
    for name, geometry, target in (
        ("grid", grid(), 220), ("sphere", sphere(), 180),
        ("creased-patch", creased_patch(), 96),
    ):
        path = folder / (name + ".obj")
        write_obj(path, geometry)
        yield name, path, Configuration("pure-quads", target)

    for name, path, config in (
        ("torus-quads", "torus.obj", Configuration("pure-quads", 240)),
        ("torus-triangles", "torus.obj", Configuration("triangles", 240, rosy=6, posy=3)),
        ("angle-bracket", "angle-bracket.obj", Configuration("crease", 240, quad=0, crease="35")),
        ("nearby-sheets", "nearby-sheets.obj", Configuration("boundaries", 220, quad=0, boundary=1)),
        ("nearby-sheets-unsmoothed", "nearby-sheets.obj", Configuration("boundaries", 220, quad=0, smooth=0, boundary=1)),
        ("dense-torus", "torus-dense.obj", Configuration("pure-quads", 2000)),
    ):
        yield name, fixtures / path, config

    if dense_patch:
        yield "dense-creased-patch", dense_patch, Configuration("pure-quads", 1000)
        yield "dense-creased-patch-dominant", dense_patch, Configuration("quad-dominant", 1000, quad=0)

    large_torus = folder / "large-torus.obj"
    positions, faces, _ = torus(524, 360)
    write_fixture_obj(large_torus, positions, faces)
    yield "large-torus", large_torus, Configuration("pure-quads", 1000)

    torus_points = folder / "torus-points.obj"
    ply = (fixtures / "torus-points.ply").read_text().partition("end_header\n")[2]
    with torus_points.open("w", encoding="ascii") as output:
        for point in ply.splitlines():
            x, y, z, nx, ny, nz = point.split()
            output.write(f"v {x} {y} {z}\nvn {nx} {ny} {nz}\n")
    yield "torus-point-cloud", torus_points, Configuration("point-cloud", 160, quad=0)

    wavy_points = folder / "wavy-points.obj"
    write_oriented_points(wavy_points)
    yield "wavy-point-cloud", wavy_points, Configuration("point-cloud", 160, quad=0)

    if providers:
        manifest = json.loads((ROOT / "tests" / "fixtures" / "providers" / "fixtures.json").read_text())
        for name, entry in manifest.items():
            path = providers / name
            if digest(path) != entry["sha256"]:
                raise ValueError(f"Unexpected provider fixture checksum for {name}")
            yield path.stem, path, Configuration("pure-quads", entry["cases"][0]["target_vertices"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    native = parser.add_mutually_exclusive_group(required=True)
    native.add_argument("--port", type=Path, help="Clean checkout of the pinned C++ port")
    native.add_argument("--native", type=Path, help="Existing parity runner instrumented with three warm-ups")
    parser.add_argument("--providers", type=Path, help="Prepared provider OBJs from prepare_provider_fixtures.py")
    parser.add_argument("--dense-patch", type=Path, help="Dense creased OBJ from the historical source snapshot")
    parser.add_argument("--case", help="Measure only the named benchmark case")
    parser.add_argument("--release", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    managed_project = ROOT / "tests" / "InstantMeshes.Parity"
    run("dotnet", "build", managed_project, "-c", "Release", "--disable-build-servers", "-p:NuGetAudit=false")
    managed = managed_project / "bin/Release/net8.0/InstantMeshes.Parity.dll"
    with TemporaryDirectory(prefix="instant-meshes-release-benchmark-") as directory:
        folder = Path(directory)
        native_runner = args.native.resolve() if args.native else folder / "build/native/parity"
        if args.port:
            if (run("git", "-C", args.port, "rev-parse", "HEAD").strip() != PORT_REVISION
                    or run("git", "-C", args.port, "status", "--porcelain").strip()):
                parser.error("Use a clean checkout of the pinned native port.")
            shutil.copy2(args.port / "Makefile", folder)
            for name in ("native", "vendor"):
                shutil.copytree(args.port / name, folder / name)
            instrument_cli(folder, warmups=3)
            run("make", "-j4", "parity", "CXX=clang++", cwd=folder)

        report = {
            "release": args.release,
            "scope": "Warm solver time, excluding input I/O and process startup",
            "os": platform.platform(), "architecture": platform.machine(),
            "managedRuntimes": [line.split(" [", 1)[0] for line in run("dotnet", "--list-runtimes").splitlines()],
            "nativeCompiler": run("clang++", "--version").splitlines()[0],
            "nativeFlags": ["-O2", "-ffp-contract=off", "-fno-fast-math", "-DEIGEN_DONT_VECTORIZE", "-DIM_PORT_DIAGNOSTICS"],
            "nativeRevision": PORT_REVISION,
            "nativeRunnerSha256": digest(native_runner),
            "workers": 1, "processPairs": 2, "warmupsPerProcess": 3, "samplesPerProcess": 3,
            "cases": {},
        }
        for name, obj, config in benchmark_cases(folder, args.providers, args.dense_patch):
            if args.case and name != args.case:
                continue
            with obj.open("rb") as mesh:
                elements = {"triangles": 0, "vertices": 0}
                for line in mesh:
                    if line.startswith(b"f "):
                        elements["triangles"] += 1
                    elif line.startswith(b"v "):
                        elements["vertices"] += 1
            report["cases"][name] = {"input": obj.name, **elements, "modes": {}}
            for mode, flag in (("ordinary", "0"), ("stable", "1")):
                case = execute_case(native_runner, managed, None, obj, name, config, flag, folder, 3)
                report["cases"][name]["modes"][mode] = case
                print(name, mode, case["summary"], flush=True)

        if not report["cases"]:
            parser.error(f"No benchmark case named {args.case!r}.")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
