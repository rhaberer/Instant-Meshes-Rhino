"""Smoke test the newly added benchmark fixtures through both in-process solvers."""
import argparse
from array import array
import json
from pathlib import Path
import shlex
import struct
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MANAGED = HERE / "managed/bin/Release/net8.0/Check.dll"
OUT = HERE / "results"


def dump(path):
    data = path.read_bytes()
    nv, nf, arity = struct.unpack_from("<III", data)
    assert len(data) == 12 + nv * 12 + nf * arity * 4
    return (nv, nf, arity), array("f", data[12:12 + nv * 12]), data[12 + nv * 12:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", type=Path, required=True, help="Path to the compiled native fixture runner")
    parser.add_argument("--generated-only", action="store_true", help="Skip optional provider meshes")
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    results = {}
    sources = ("generated",) if args.generated_only else ("generated", "providers")
    for source in sources:
        directory = ROOT / source
        for name, fixture in json.loads((directory / "fixtures.json").read_text()).items():
            for case in fixture["cases"]:
                flags = shlex.split(case["flags"])

                def option(name, default):
                    return flags[flags.index(name) + 1] if name in flags else default

                face_mode = "Triangles" if option("-p", "4") == "6" else (
                    "QuadDominant" if "-D" in flags else "PureQuads")
                label = f"{Path(name).stem}-{case['name']}"
                native = OUT / f"{label}-native.bin"
                managed = OUT / f"{label}-managed.bin"
                parameters = [directory / name, str(case["target_vertices"]), "1", option("-r", "4"),
                        face_mode, option("-S", "2"), "1" if "-b" in flags else "0",
                        option("-c", "-"), option("-k", "10")]
                row = {}
                for side, command, output in (("native", [args.native.resolve()], native),
                                              ("managed", ["dotnet", MANAGED], managed)):
                    try:
                        result = subprocess.run([*command, *parameters, output], capture_output=True,
                                                text=True, timeout=120)
                    except subprocess.TimeoutExpired:
                        row[side] = {"error": "Solver exceeded 120 seconds."}
                        continue
                    if result.returncode == 0:
                        counts, _, _ = dump(output)
                        row[side] = {"vertices": counts[0], "faces": counts[1], "arity": counts[2]}
                    else:
                        row[side] = {"error": (result.stderr or result.stdout or f"Exit {result.returncode}").strip().splitlines()[0]}
                if "vertices" in row["managed"] and "vertices" in row["native"]:
                    n_counts, n_positions, n_indices = dump(native)
                    m_counts, m_positions, m_indices = dump(managed)
                    exact = n_counts == m_counts and n_indices == m_indices
                    row["ordered_connectivity_matches"] = exact
                    if exact:
                        row["max_absolute_coordinate_delta"] = max(
                            (abs(a - b) for a, b in zip(n_positions, m_positions)), default=0)
                results[label] = row
                print(label, json.dumps(row), flush=True)
    (OUT / "fixture-smoke.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
