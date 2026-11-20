"""Measure Release C#, the pinned serial port, and the original CLI locally."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import re
import shutil
from statistics import median
from subprocess import CalledProcessError, Popen, STDOUT
from tempfile import TemporaryDirectory, TemporaryFile
from time import perf_counter
import os
import platform

from run import (Configuration, PORT_REVISION, ROOT, UPSTREAM_REVISION,
                 creased_patch, grid, instrument_cli, run, sphere, write_obj)


def measurements(values):
    ordered = sorted(values)
    return {"median": median(values), "p95": ordered[(95 * len(ordered) + 99) // 100 - 1]}


def original_sample(executable, obj, config, output):
    command = [executable, "-d", "-t", "1", "-v", str(config.target), "-r", str(config.rosy),
               "-p", "6" if config.posy == 3 else "4", "-S", str(config.smooth),
               "-o", output, obj]
    if not config.quad:
        command.insert(1, "-D")
    with TemporaryFile(mode="w+b") as log_file:
        begin = perf_counter()
        with Popen(command, stdout=log_file, stderr=STDOUT) as process:
            _, status, resource = os.wait4(process.pid, 0)
            process.returncode = os.waitstatus_to_exitcode(status)
        wall = perf_counter() - begin
        log_file.seek(0)
        log = log_file.read().decode()
    if process.returncode:
        raise CalledProcessError(process.returncode, command, output=log)
    memory = resource.ru_maxrss * (1 if platform.system() == "Darwin" else 1024)
    original_time = r"([0-9.]+)\s*(ms|s)\b"

    def time_from(expression):
        match = re.search(expression + original_time, log, re.MULTILINE)
        return None if match is None else float(match[1]) * (0.001 if match[2] == "ms" else 1.0)

    stages = {
        "input": time_from(r"Loading [^\n]*?took "),
        "preprocessing": time_from(r"total time excluding file I/O: "),
        "hierarchy": time_from(r"Hierarchy construction took "),
        "orientation": time_from(r"Optimizing orientation field[^\n]*\n[^\n]*?took "),
        "position": time_from(r"Optimizing position field[^\n]*?took "),
        "extraction": time_from(r"Extraction is done\. \(total time: "),
        "output": time_from(r"Writing [^\n]*?took "),
    }
    compute = [stages[key] for key in ("preprocessing", "orientation", "position", "extraction")]
    counts = [list(map(int, match)) for match in re.findall(r"\(V=(\d+), F=(\d+)", log)]
    return {"wallSeconds": wall, "seconds": sum(compute) if all(value is not None for value in compute) else None,
            "peakBytes": memory, "inputCounts": counts[0] if counts else None,
            "outputCounts": counts[-1] if len(counts) > 1 else None, "log": log, "stages": stages}


def execute_case(native, managed, original, obj, label, config, stable, folder, repetitions):
    base = folder / f"{label}-{config.label}-{stable}"
    native_dump, managed_dump, original_obj = (base.with_suffix(suffix) for suffix in
                                               (".native.bin", ".managed.bin", ".original.obj"))
    env = {**os.environ, "IM_TIMED_RUNS": str(repetitions)}
    if config.crease != "-":
        env["IM_CREASE_ANGLE"] = config.crease
    native_command = [native, obj, *config.arguments(), native_dump, stable,
                      str(config.boundary), str(config.extrinsic)]
    managed_command = ["dotnet", managed, "--result", obj, native_dump, *config.arguments(), stable,
                       str(config.boundary), str(config.extrinsic), config.crease, str(repetitions), managed_dump]
    raw = {"native": [], "managed": [], "original": []}
    implementations = ("native", "managed", "original") if stable == "0" and original else ("native", "managed")
    for order in (implementations, implementations[::-1]):
        for implementation in order:
            if implementation == "native":
                output = run(*native_command, env=env)
                measures = [line.split("=", 1) for line in output.splitlines()[:-1]]
                raw["native"].append({
                    "coldSeconds": float(measures[0][1]),
                    "samplesSeconds": [float(value) for key, value in measures if key == "sampleSeconds"],
                    "peakBytes": int(measures[-1][1]), "output": output.splitlines()[-1]})
            elif implementation == "managed":
                raw["managed"].append(json.loads(run(*managed_command)))
            else:
                cold = original_sample(original, obj, config, original_obj)
                samples = [original_sample(original, obj, config, original_obj) for _ in range(repetitions)]
                raw["original"].append({"cold": cold, "samples": samples})
    native = [value for entry in raw["native"] for value in entry["samplesSeconds"]]
    managed = [sample["seconds"] for entry in raw["managed"] for sample in entry["managedSamples"]]
    original = [sample for burst in raw["original"] for sample in burst["samples"]]
    summary = {"native": measurements(native), "managed": measurements(managed)}
    if original:
        summary["originalWall"] = measurements([sample["wallSeconds"] for sample in original])
        compute = [sample["seconds"] for sample in original if sample["seconds"] is not None]
        if compute:
            summary["originalCompute"] = measurements(compute)
    return {"fixtureSha256": sha256(obj.read_bytes()).hexdigest(), "configuration": config.__dict__,
            "stable": stable == "1", "summary": summary, "samples": raw}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for argument in ("port", "original", "dense", "large"):
        parser.add_argument("--" + argument, type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/performance")
    args = parser.parse_args()
    port = args.port.resolve()
    if run("git", "-C", port, "rev-parse", "HEAD").strip() != PORT_REVISION:
        parser.error("Use the pinned native port.")
    if run("git", "-C", port, "status", "--porcelain").strip():
        parser.error("Use a clean native port checkout.")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for name, geometry in (("grid", grid()), ("sphere", sphere()), ("creased-patch", creased_patch())):
        write_obj(output / (name + ".obj"), geometry)
    managed_project = ROOT / "tests" / "InstantMeshes.Parity"
    run("dotnet", "build", managed_project, "-c", "Release", "--disable-build-servers", "-m:1", "-p:NuGetAudit=false")
    managed = managed_project / "bin/Release/net8.0/InstantMeshes.Parity.dll"
    report = {"runtime": run("dotnet", "--list-runtimes").splitlines(), "os": platform.platform(),
              "architecture": platform.machine(), "compiler": run("clang++", "--version").splitlines()[0],
              "nativeFlags": ["-std=c++14", "-O2", "-g", "-ffp-contract=off", "-fno-fast-math", "-DEIGEN_DONT_VECTORIZE", "-DIM_PORT_DIAGNOSTICS"],
              "portCommit": PORT_REVISION, "originalCommit": UPSTREAM_REVISION,
              "originalSha256": sha256(args.original.read_bytes()).hexdigest(), "runs": {}}
    with TemporaryDirectory(prefix="instant-meshes-benchmark-") as temporary:
        scratch = Path(temporary)
        shutil.copy2(port / "Makefile", scratch)
        for name in ("native", "vendor"):
            shutil.copytree(port / name, scratch / name)
        instrument_cli(scratch)
        run("make", "-j4", "parity", "CXX=clang++", cwd=scratch)
        native = scratch / "build/native/parity"
        for label, obj, target, repeats in (
            ("grid", output / "grid.obj", 220, 5),
            ("sphere", output / "sphere.obj", 180, 5),
            ("creased-patch", output / "creased-patch.obj", 96, 5),
            ("dense", args.dense.resolve(), 1000, 4),
            ("large", args.large.resolve(), 1000, 3),
        ):
            configs = [Configuration("pure-quads", target)]
            if label in ("grid", "dense"):
                configs.append(Configuration("quad-dominant", target, quad=0))
            for config in configs:
                for stable in ("0", "1"):
                    name = f"{label}/{config.label}/stable={stable}"
                    report["runs"][name] = execute_case(native, managed, args.original.resolve(), obj, label,
                                                         config, stable, output, repeats)
                    print(name, flush=True)
                    (output / "performance-samples.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
