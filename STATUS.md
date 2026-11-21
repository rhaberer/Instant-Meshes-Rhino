# Parity and compatibility

For **1.0.0**, see the [downloads and installation steps](README.md#download-and-install) and [benchmark results](benchmarks/1.0.0.md). The parity and qualification results below are from the **0.1.3** engineering candidate. Its raw reports are preserved in the [historical source snapshot](https://github.com/rhaberer/Instant-Meshes-C-/blob/defc829e13d6694608c68e151fd82943c57d1cd0/InstantMeshesRhino-0.1.3.zip).

## Comparison results

The tests run C# and the pinned C++ port on the same inputs and options. Exact output means the same vertex and face counts, the same ordered indices, and zero coordinate delta.

| Suite | Result |
| --- | --- |
| 22 synthetic mesh configurations | Exact against the pinned port |
| 10 stable configurations of a private 377,280-triangle mesh | Exact against the pinned port |
| Two oriented point-cloud fixtures in ordinary and stable modes | Exact against the pinned port |
| 10 extended stable fixtures | 9 exact against the pinned port and all 10 exact against the shared score policy |
| 10 solves with hardware intrinsics disabled | Exact against their respective references |

The synthetic cases cover pure quads, quad-dominant output, triangles, orientation symmetry 2/4/6, intrinsic and extrinsic modes, smoothing, boundaries, and creases. The extended set adds Spot, Stanford Bunny, Stanford Armadillo, and generated stress cases. The historical snapshot includes settings, counts, and deltas under `artifacts/parity/readability-0.1.3`.

### Stable-mode Stanford Bunny

At a 2,500-vertex target, ordinary arithmetic matches the pinned port exactly at **10,012 vertices and 9,941 quads**. In stable mode many prepared vertex areas round to zero. Comparing rational collapse scores with zero denominators creates an intransitive ordering in the pinned comparator. Its recorded output has **7,450 vertices and 7,394 quads**.

The C# solver assigns one area quantum to affected linked zero-area vertices when the comparisons are ambiguous. A scratch C++ build using the same policy matches the C# result exactly at **10,001 vertices and 9,927 quads**. The historical snapshot includes both comparisons and the score cycle in `artifacts/bunny/reference-report.json`.

## Host qualification

Release builds and option tests passed. The archive passed integrity, binary identity, and isolated Yak install/removal checks. Yak reports differences between the plugin assembly's default name and version and the package metadata. Local search, installation and removal passed. An out-of-process probe using the installed Rhino 8 SR37 (8.37.26307.15002, arm64) assemblies exercised mesh and point-cloud conversion, Grasshopper inputs, and the example document round-trip.

Version 1.0.0 has Yak distributions for Windows and macOS with the `rh8_32` compatibility tag. The code targets .NET 8 with the Rhino 8.32 reference SDK. SR37 on macOS is the SDK qualification host. Interactive loading and workflows in Rhino and Grasshopper, including on Windows, are still unverified. Grasshopper 2 requires a separate Rhino 9 integration.

The 0.1.3 core method definitions and IL match all 694 in the timed 0.1.2 build. All 163 reflected public core API entries match. The historical snapshot includes the package SHA-256 and IL comparison.

## Reproduce

Use Python 3, .NET, Make, and Clang. Supply a clean local InstantMeshesPort checkout at the revision in [NOTICE.md](NOTICE.md), with its pinned vendor sources populated. If the vendor directory is missing, run `./scripts/vendor.sh` from the native checkout. It fetches the versions pinned by that checkout. Run the commands below from the C# project root. The runner builds the native reference in a temporary copy.

~~~sh
python3 tests/reference/run.py --port ../InstantMeshesPort
python3 tests/reference/run.py --port ../InstantMeshesPort \
  --scan-obj tests/fixtures/providers/stanford-bunny.obj --trace-scan \
  --reference-score-policy --output /tmp/instant-meshes-policy
~~~

Omit **--reference-score-policy** and use a separate output folder to compare against the unmodified port. Pass **--model-obj** with a local indexed triangle OBJ for the larger matrix. See the [fixture guide](tests/fixtures/FIXTURES.txt) for obtaining provider meshes locally.
