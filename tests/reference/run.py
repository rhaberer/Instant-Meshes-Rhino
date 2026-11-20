"""Build the pinned port locally and compare managed preprocessing to native."""
import argparse
from dataclasses import dataclass
from hashlib import sha256
import json
import os
from math import cos, hypot, sin
from pathlib import Path
import shutil

import subprocess
from tempfile import TemporaryDirectory

from diagnostics import instrument_point_cloud, instrument_raw_scores, raw_score_evidence

PORT_REVISION = "cc463c0c509e8c50238a648c6cd92df8f3ad3c97"
UPSTREAM_REVISION = "7b3160864a2e1025af498c84cfed91cbfb613698"
ROOT = Path(__file__).resolve().parents[2]


def run(*args, cwd=None, env=None):
    result = subprocess.run(args, cwd=cwd, env=env, text=True, stdout=subprocess.PIPE)
    if result.returncode:
        print(result.stdout, end="", flush=True)
        result.check_returncode()
    return result.stdout


def grid(n=24):
    positions = [(x / n, sin(3 * x / n) * 0.15, y / n)
                 for y in range(n + 1) for x in range(n + 1)]
    triangles = []
    for y in range(n):
        for x in range(n):
            a = y * (n + 1) + x
            b = a + n + 1
            triangles.extend([(a, b, a + 1), (a + 1, b, b + 1)])
    return positions, triangles


def creased_patch():
    # Two different slopes meet at x=6. A rectangular interior cell is omitted.
    n = 12
    positions = [
        (x / n + 0.007 * sin(y + 2 * x),
         0.14 * abs(x / n - 0.5) + 0.012 * sin(y),
         y / n + 0.005 * sin(x + y))
        for y in range(n + 1) for x in range(n + 1)
    ]
    triangles = []
    for y in range(n):
        for x in range(n):
            if (x, y) == (3, 7):
                continue
            a = y * (n + 1) + x
            b = a + n + 1
            if (x + y) % 2:
                triangles.extend([(a, b, b + 1), (a, b + 1, a + 1)])
            else:
                triangles.extend([(a, b, a + 1), (a + 1, b, b + 1)])
    return positions, triangles


def sphere():
    vertices = [(0, 1, 0), (0, -1, 0), (1, 0, 0), (0, 0, 1), (0, 0, -1), (-1, 0, 0)]
    faces = [(0, 2, 3), (0, 3, 5), (0, 5, 4), (0, 4, 2),
             (1, 3, 2), (1, 5, 3), (1, 4, 5), (1, 2, 4)]
    for _ in range(3):
        midpoints = {}

        def midpoint(a, b):
            key = tuple(sorted((a, b)))
            if key not in midpoints:
                point = [(x + y) / 2 for x, y in zip(vertices[a], vertices[b])]
                length = hypot(*point)
                midpoints[key] = len(vertices)
                vertices.append(tuple(x / length for x in point))
            return midpoints[key]

        next_faces = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            next_faces.extend([(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)])
        faces = next_faces
    return vertices, faces


def write_obj(path, geometry):
    positions, triangles = geometry
    with path.open("w", encoding="ascii") as target:
        for position in positions:
            target.write("v " + " ".join(map(str, position)) + "\n")
        for face in triangles:
            target.write("f " + " ".join(str(i + 1) for i in face) + "\n")


@dataclass(frozen=True)
class Configuration:
    label: str
    target: int
    rosy: int = 4
    posy: int = 4
    quad: int = 1
    smooth: int = 2
    boundary: int = 0
    extrinsic: int = 1
    crease: str = "-"

    def arguments(self):
        return tuple(map(str, (self.target, self.rosy, self.posy, self.quad, self.smooth)))


def write_oriented_points(path, n=25):
    with path.open("w", encoding="ascii") as target:
        for y in range(n):
            for x in range(n):
                u = (x + 0.06 * sin(x + 2 * y)) / (n - 1)
                v = (y + 0.08 * sin(3 * x + y)) / (n - 1)
                target.write(f"v {u} {0.15 * sin(3 * u)} {v}\n")
                # Supply already unit-length oriented normals to both solvers.
                slope = -0.45 * cos(3 * u)
                length = hypot(slope, 1)
                target.write(f"vn {slope / length} {1 / length} 0\n")


def instrument_cli(scratch, warmups=1):
    path = scratch / "native/parity_main.cpp"
    source = path.read_text()
    source = '#include <chrono>\n#include <sys/resource.h>\n' + source
    source = source.replace("    im_result *result =",
        """
    if (const char *crease = std::getenv("IM_CREASE_ANGLE"))
        options.crease_angle = std::strtof(crease, nullptr);

    std::vector<float> normals;
    if (mesh.indices.empty()) {
        std::ifstream in(argv[1]);
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream stream(line);
            std::string tag;
            float x, y, z;
            if (stream >> tag && tag == "vn" && stream >> x >> y >> z) {
                normals.push_back(x);
                normals.push_back(y);
                normals.push_back(z);
            }
        }
        if (normals.size() != mesh.positions.size()) {
            std::fprintf(stderr, "Point cloud requires one oriented normal per point\\n");
            return 2;
        }
    }
    auto solve = [&] { return
""").replace("mesh.indices.data(), (uint32_t) mesh.indices.size(), nullptr, &options);",
        """mesh.indices.data(), (uint32_t) mesh.indices.size(),
                  normals.empty() ? nullptr : normals.data(), &options); };
    auto start = std::chrono::steady_clock::now();
    im_result *result = solve();
    if (const char *iterations = std::getenv("IM_TIMED_RUNS")) {
        auto seconds = [](auto since) {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
        };
        std::printf("coldSeconds=%.9f\\n", seconds(start));
        for (int i = 0; i < std::atoi(iterations); ++i) {
            start = std::chrono::steady_clock::now();
            im_result *sample = solve();
            std::printf("sampleSeconds=%.9f\\n", seconds(start));
            if (!im_result_ok(sample))
                return 1;
            im_result_free(sample);
        }
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
#if defined(__APPLE__)
        std::printf("peakBytes=%ld\\n", usage.ru_maxrss);
#else
        std::printf("peakBytes=%ld\\n", usage.ru_maxrss * 1024);
#endif
    }""")
    # The cold solve above counts as the first warm-up.
    if warmups > 1:
        source = source.replace(
            '        for (int i = 0; i < std::atoi(iterations); ++i) {',
            f'''        for (int i = 1; i < {warmups}; ++i) {{
            im_result *sample = solve();
            if (!im_result_ok(sample))
                return 1;
            im_result_free(sample);
        }}
        for (int i = 0; i < std::atoi(iterations); ++i) {{''')
    path.write_text(source)


def compare_result(native, managed, output, name, obj, config, stable, samples=0, point_stage=False):
    dump = output / f"{name}-{config.label}-{stable}.bin"
    env = os.environ.copy()
    if config.crease != "-":
        env["IM_CREASE_ANGLE"] = config.crease
    if samples:
        env["IM_TIMED_RUNS"] = str(samples)
    point_paths = []
    if point_stage:
        for variable, label in (("IM_POINT_CANDIDATES_BIN", "directed"),
                                ("IM_POINT_NEIGHBORS_BIN", "adjacency"),
                                ("IM_POINT_RADIUS_BIN", "radius"), ("IM_FIELD_BIN", "fields")):
            path = output / f"{name}-{stable}-{label}.bin"
            path.unlink(missing_ok=True)
            env[variable] = str(path)
            point_paths.append(path)
    native_output = run(native, obj, *config.arguments(), dump, stable,
                        str(config.boundary), str(config.extrinsic), env=env).strip()
    report = json.loads(run("dotnet", managed, "--result", obj, dump, *config.arguments(), stable,
                            str(config.boundary), str(config.extrinsic), config.crease, str(samples)))
    if samples:
        timings = [line.split("=", 1) for line in native_output.splitlines()[:-1]]
        report["nativeColdSeconds"] = float(timings[0][1])
        report["nativeSamplesSeconds"] = [float(value) for key, value in timings if key == "sampleSeconds"]
        report["nativePeakBytes"] = int(timings[-1][1])
    if point_stage:
        report["preprocessing"] = json.loads(run("dotnet", managed, "--point-stage", obj, *point_paths,
                                                 stable, str(config.target), str(config.quad)))
    print(f"{name} / {config.label} / stable={stable} {report['comparison']}", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts" / "parity")
    parser.add_argument("--original", type=Path, help="Optional original Instant Meshes executable")
    parser.add_argument("--scan-obj", type=Path,
                        help="Optional local scan as a triangle-only OBJ with bare positive face indices")
    parser.add_argument("--model-obj", type=Path,
                        help="Optional local OBJ for the native port's ten stable model configurations")
    parser.add_argument("--benchmark", type=Path,
                        help="Optional local OBJ for three warm managed and native timing samples")
    parser.add_argument("--trace-scan", action="store_true", help="Record the scan's first strict stage mismatch")
    parser.add_argument("--reference-score-policy", action="store_true",
                        help="Apply the diagnostic score policy only to the scratch native reference")
    parser.add_argument("--scan-target", type=int, default=2500)
    args = parser.parse_args()
    if args.scan_target <= 0:
        parser.error("--scan-target must be positive.")
    port = args.port.resolve()
    revision = run("git", "-C", port, "rev-parse", "HEAD").strip()
    if revision != PORT_REVISION or run("git", "-C", port, "status", "--porcelain").strip():
        parser.error("Use the clean pinned port checkout listed in NOTICE.md.")
    if UPSTREAM_REVISION not in (port / "vendor" / "UPSTREAM").read_text():
        parser.error("The vendored sources must match the upstream pin in NOTICE.md.")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    runner = ROOT / "tests" / "InstantMeshes.Parity"
    print(run("dotnet", "build", runner, "-c", "Release", "-p:NuGetAudit=false", "--disable-build-servers", "-m:1"), end="", flush=True)
    managed = runner / "bin" / "Release" / "net8.0" / "InstantMeshes.Parity.dll"

    with TemporaryDirectory(prefix="instant-meshes-reference-") as directory:
        scratch = Path(directory)
        shutil.copy2(port / "Makefile", scratch)
        for name in ("native", "vendor"):
            shutil.copytree(port / name, scratch / name)
        hierarchy = scratch / "vendor/instant-meshes/src/hierarchy.cpp"
        source = hierarchy.read_text()
        marker = '    bool dump_collapse = std::getenv'
        assert source.count(marker) == 1
        hierarchy.write_text(source.replace(marker, """
    if (const char *path = std::getenv("IM_CANDIDATES_BIN")) {
        std::ofstream stream(path, std::ios::binary | std::ios::app);
        stream.write(reinterpret_cast<const char*>(&nLinks), sizeof(nLinks));
        for (uint32_t i = 0; i < nLinks; ++i) {
            stream.write(reinterpret_cast<const char*>(&entries[i].i), sizeof(uint32_t));
            stream.write(reinterpret_cast<const char*>(&entries[i].j), sizeof(uint32_t));
            stream.write(reinterpret_cast<const char*>(&entries[i].order), sizeof(float));
        }
    }

""" + marker))
        optimizer = scratch / "native/synchronous_optimizer.h"
        source = optimizer.read_text()
        for matrix, kind in (("Q", "0"), ("O", "1")):
            marker = f"            canonicalize_field(mRes.{matrix}(level));"
            assert source.count(marker) == 1
            source = source.replace(marker, marker + f"""
            if (const char *path = std::getenv("IM_FIELD_BIN")) {{
                std::ofstream stream(path, std::ios::binary | std::ios::app);
                int32_t header[] = {{{kind}, level, levelIteration - 1, (int32_t)mRes.size(level)}};
                stream.write(reinterpret_cast<const char*>(header), sizeof(header));
                stream.write(reinterpret_cast<const char*>(mRes.{matrix}(level).data()),
                             3 * mRes.size(level) * sizeof(float));
            }}
""")
        optimizer.write_text('#include <fstream>\n' + source)
        instrument_cli(scratch)
        instrument_point_cloud(scratch)
        if args.reference_score_policy:
            run("patch", "-p1", "-i", Path(__file__).with_name("stable-score-policy.patch"), cwd=scratch)
        instrument_raw_scores(hierarchy)
        print(run("make", "-j4", "parity", "CXX=clang++", cwd=scratch), end="", flush=True)
        native = scratch / "build" / "native"
        stage_runner = scratch / "preprocessing"
        run("clang++", "-std=c++14", "-O2", "-ffp-contract=off", "-fno-fast-math",
            "-DEIGEN_DONT_VECTORIZE", "-DIM_PORT_DIAGNOSTICS",
            "-Inative", "-Ivendor/instant-meshes/src", "-Ivendor/instant-meshes/ext/dset",
            "-Ivendor/instant-meshes/ext/pcg32", "-Ivendor/eigen", "-Ivendor/tbb-shim",
            "-Ivendor/pss", "-Ivendor/port-shims", ROOT / "tests/reference/preprocessing.cpp",
            native / "libinstantmeshes.a", "-o", stage_runner, cwd=scratch)

        fixtures = {}
        cases = []
        for name, geometry, target in (("grid", grid(), 220), ("sphere", sphere(), 180),
                                       ("creased-patch", creased_patch(), 96)):
            obj = output / f"{name}.obj"
            write_obj(obj, geometry)
            cases.append((name, obj, target))
        for name, obj, target in cases:
            results = {}
            for mode, stable in (("ordinary", "0"), ("stable", "1")):
                stage_dump = output / f"{name}-{mode}-preprocessing.bin"
                final_dump = output / f"{name}-{mode}-remesh.bin"
                run(stage_runner, obj, stage_dump, stable)
                result = run("dotnet", managed, obj, stage_dump, mode).strip()
                print(result, flush=True)
                field_dump = output / f"{name}-{mode}-field.bin"
                field_dump.unlink(missing_ok=True)
                native_summary = run(native / "parity", obj, str(target), "4", "4",
                                     "1", "2", final_dump, stable, "0", "1",
                                     env={**os.environ, "IM_FIELD_BIN": str(field_dump)}).strip()
                details = {"preprocessing": result, "nativeRemesh": native_summary}
                if name != "local-scan":
                    limit = {"grid": "0.05", "sphere": "0.20", "creased-patch": "0.10"}[name]
                    subdivided = output / f"{name}-{mode}-subdivision.bin"
                    run(stage_runner, obj, subdivided, stable, limit)
                    result = run("dotnet", managed, obj, subdivided, mode, limit).strip()
                    print(f"After subdivision: {result}", flush=True)
                    details["subdivision"] = result
                hierarchy_dump = output / f"{name}-{mode}-hierarchy.bin"
                candidates = output / f"{name}-{mode}-candidates.bin"
                candidates.unlink(missing_ok=True)
                run(stage_runner, obj, hierarchy_dump, stable, "hierarchy", str(target),
                    env={**os.environ, "IM_CANDIDATES_BIN": str(candidates)})
                result = run("dotnet", managed, obj, hierarchy_dump, mode, str(target), candidates, field_dump, final_dump).strip()
                print(f"Hierarchy: {result}", flush=True)
                details["hierarchy"] = result
                results[mode] = details
            if args.original:
                original_obj = output / f"{name}-upstream.obj"
                run(args.original.resolve(), "-d", "-t", "1", "-v", str(target),
                    "-o", original_obj, obj)
                lines = original_obj.read_text().splitlines()
                results["upstream"] = {"vertices": sum(line.startswith("v ") for line in lines),
                                       "faces": sum(line.startswith("f ") for line in lines)}
            fixtures[name] = {"inputSha256": sha256(obj.read_bytes()).hexdigest(), "cases": results}

        variants = (
            ("grid", Configuration("quads-no-smoothing", 220, smooth=0)),
            ("grid", Configuration("quad-dominant", 220, quad=0)),
            ("grid", Configuration("triangles", 220, rosy=6, posy=3)),
            ("grid", Configuration("rosy-2", 220, rosy=2)),
            ("grid", Configuration("boundary-aligned", 220, boundary=1)),
            ("grid", Configuration("intrinsic", 220, extrinsic=0)),
            ("sphere", Configuration("triangles", 180, rosy=6, posy=3)),
            ("creased-patch", Configuration("crease-angle", 96, crease="12")),
        )
        alternative = {}
        for name, config in variants:
            key = f"{name}/{config.label}"
            alternative[key] = {mode: compare_result(native / "parity", managed, output, name,
                                                      output / f"{name}.obj", config, stable)
                                for mode, stable in (("ordinary", "0"), ("stable", "1"))}

        cloud = output / "oriented-wavy-points.obj"
        write_oriented_points(cloud)
        point_config = Configuration("point-cloud", 160)
        points = {mode: compare_result(native / "parity", managed, output, "points",
                                        cloud, point_config, stable, point_stage=True)
                  for mode, stable in (("ordinary", "0"), ("stable", "1"))}
        ply = ROOT / "tests/fixtures/generated/torus-points.ply"
        torus = output / "oriented-torus-points.obj"
        with torus.open("w", encoding="ascii") as file:
            rows = ply.read_text().split("end_header\n", 1)[1].splitlines()
            for row in rows:
                values = row.split()
                file.write("v " + " ".join(values[:3]) + "\nvn " + " ".join(values[3:]) + "\n")
        torus_points = {mode: compare_result(native / "parity", managed, output, "torus-points",
                                              torus, Configuration("point-cloud", 160, quad=0),
                                              stable, point_stage=True)
                        for mode, stable in (("ordinary", "0"), ("stable", "1"))}
        run("dotnet", managed, "--smoke", cloud, output / "grid.obj")

        model = None
        if args.model_obj:
            options = (
                Configuration("v300-quads-s0", 300, smooth=0),
                Configuration("v300-quads-s2", 300),
                Configuration("v1000-quads-s0", 1000, smooth=0),
                Configuration("v1000-quads-s2", 1000),
                Configuration("v2000-quads-s2", 2000),
                Configuration("v1000-quad-dominant", 1000, quad=0),
                Configuration("v1000-triangles", 1000, rosy=6, posy=3),
                Configuration("v1000-rosy-2", 1000, rosy=2),
                Configuration("v1000-intrinsic", 1000, extrinsic=0),
                Configuration("v1000-boundary-aligned", 1000, boundary=1),
            )
            model = {"inputSha256": sha256(args.model_obj.read_bytes()).hexdigest(),
                     "cases": {option.label: compare_result(native / "parity", managed, output, "model",
                                                            args.model_obj.resolve(), option, "1")
                               for option in options}}
        scan = None
        if args.scan_obj:
            path = args.scan_obj.resolve()
            scan = {"inputSha256": sha256(path.read_bytes()).hexdigest(),
                    "cases": {mode: compare_result(native / "parity", managed, output, "local-scan", path,
                                                    Configuration("quads", args.scan_target), stable)
                              for mode, stable in (("ordinary", "0"), ("stable", "1"))}}
        if args.scan_obj and args.trace_scan:
            for mode, stable in (("ordinary", "0"), ("stable", "1")):
                trace = output / f"local-scan-{mode}-stage"
                details = scan["cases"][mode]
                hierarchy_dump, candidates, field_dump = (
                    trace.with_suffix(f".{name}.bin") for name in ("hierarchy", "candidates", "fields"))
                candidates.unlink(missing_ok=True)
                field_dump.unlink(missing_ok=True)
                env = {**os.environ, "IM_CANDIDATES_BIN": str(candidates)}
                raw_scores = trace.with_suffix(".scores.bin")
                raw_scores.unlink(missing_ok=True)
                if stable == "1":
                    env["IM_RAW_SCORES_BIN"] = str(raw_scores)
                run(stage_runner, path, hierarchy_dump, stable, "hierarchy", str(args.scan_target), env=env)
                if stable == "1":
                    details["initialScoreTrace"] = raw_score_evidence(raw_scores, candidates)
                final_dump = output / f"local-scan-quads-{stable}.bin"
                run(native / "parity", path, str(args.scan_target), "4", "4", "1", "2", final_dump,
                    stable, env={**os.environ, "IM_FIELD_BIN": str(field_dump)})
                trace_run = subprocess.run(
                    ("dotnet", managed, path, hierarchy_dump, mode, str(args.scan_target), candidates,
                     field_dump, final_dump), capture_output=True, text=True)
                details["stageTrace"] = (trace_run.stdout or trace_run.stderr).splitlines()[0]
                print(f"Scan stage trace, {mode}: {details['stageTrace']}", flush=True)
        benchmark = {
            "grid": compare_result(native / "parity", managed, output, "grid",
                                   output / "grid.obj", Configuration("benchmark", 220), "1", 3)
        }
        if args.benchmark:
            benchmark["scan"] = compare_result(native / "parity", managed, output, "scan",
                args.benchmark.resolve(), Configuration("benchmark", args.scan_target), "1", 3)

    report = {"portCommit": PORT_REVISION, "upstreamCommit": UPSTREAM_REVISION,
              "referenceScorePolicy": args.reference_score_policy,
              "compiler": run("clang++", "--version").splitlines()[0], "fixtures": fixtures,
              "variants": alternative, "orientedPoints": points, "orientedTorus": torus_points,
              "model": model, "scan": scan, "benchmark": benchmark}
    (output / "reference-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Report saved to {output / 'reference-report.json'}", flush=True)
    for name, modes in alternative.items():
        for mode, outcome in modes.items():
            comparison = outcome["comparison"]
            if not comparison["ordered"] or comparison["maxPositionDelta"] > 5e-5:
                raise SystemExit(f"Synthetic output differs in {name} ({mode})")
    for name, cases in (("wavy", points), ("torus", torus_points)):
        for mode, outcome in cases.items():
            stage = outcome["preprocessing"]
            comparison = outcome["comparison"]
            if (not comparison["ordered"] or comparison["maxAbsoluteCoordinateDelta"] != 0
                    or stage["nativeRadius"] != stage["managedRadius"] or stage["nativeArea"] != stage["managedArea"]
                    or stage["directedOrderingDifferences"] or stage["orderingDifferences"]
                    or stage["iterations"] == 0 or stage["fieldDifference"]):
                raise SystemExit(f"Point-cloud parity differs for {name} ({mode})")


if __name__ == "__main__":
    main()
