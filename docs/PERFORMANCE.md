# C# versus native performance

## Version 1.0.0

The release benchmark covers 17 configurations in both ordinary and stable arithmetic, including creases, disconnected sheets, oriented point clouds, the 345,944-triangle Stanford Armadillo, and a generated 377,280-triangle torus. The comparison uses one worker on Apple Silicon. Warm solver times exclude file I/O and process launch. Ordered indices and all coordinates matched the pinned C++ port in 33 of 34 comparisons. Stable Stanford Bunny reproduced the known topology difference. [Results and fixture attribution](../benchmarks/1.0.0.md) · [Raw samples](../benchmarks/1.0.0.json) · [Stable Bunny analysis](STATUS.md#stable-mode-stanford-bunny)

The managed core ran in Release on .NET 10.0.9. The arm64 Clang build of the pinned port used O2 with floating-point contraction, fast math, and Eigen vectorization disabled. Each implementation ran three warm-up solves and three timed solves per process across two alternating process pairs. The reported p95 is the slowest of the six samples. The JSON includes input hashes, options, output comparisons, allocations, and process peak memory. Live Rhino and Grasshopper workflow timing is pending.

### Reproduce the release benchmark

Use a clean checkout of the C++ port at the revision in [NOTICE.md](../NOTICE.md), with its vendor sources populated, and run the benchmark from the C# repository root. The benchmark generates the 377,280-triangle torus in a temporary directory. The optional provider and dense-patch arguments include the downloaded scans and the archived 68,310-triangle creased patch. [Provider preparation](../tests/fixtures/FIXTURES.txt) · [Historical snapshot containing the dense patch](https://github.com/rhaberer/Instant-Meshes-C-/blob/defc829e13d6694608c68e151fd82943c57d1cd0/InstantMeshesRhino-0.1.3.zip)

~~~sh
python3 tests/reference/benchmark_release.py \
  --port ../InstantMeshesPort \
  --providers /path/to/prepared/providers \
  --dense-patch /path/to/dense-creased-patch.obj \
  --release 1.0.0 \
  --output artifacts/performance/1.0.0.json
~~~

## Historical benchmarks

The earlier controlled benchmark used a private 377,280-triangle input. The 0.1.2 managed core and pinned C++ port matched ordered output on that input. C# took **1.783 s** versus **1.804 s** in ordinary mode and **2.302 s** versus **2.775 s** in stable mode. The 0.1.3 core had matching method definitions and IL. Its timing screen was too variable for a fresh speed claim. [Historical performance report and raw data](https://github.com/rhaberer/Instant-Meshes-C-/blob/defc829e13d6694608c68e151fd82943c57d1cd0/InstantMeshesRhino-0.1.3.zip) · [Qualification history](STATUS.md)

### Original Instant Meshes executable

A separate historical run built the upstream executable as arm64 Release with O3 and ran each sample in a fresh process with deterministic settings and one worker. The upstream executable has ordinary arithmetic. Compute time excludes OBJ I/O. Wall time measures the full command.

| Input | Original compute median | Original CLI wall median |
| --- | ---: | ---: |
| Grid, 1,152 triangles | 17 ms | 28.4 ms |
| Creased patch, 68,310 triangles | 290 ms | 358.0 ms |
| Private mesh, 377,280 triangles | 1.696 s | 1.950 s |

The original executable used different build settings and produced different ordered output from the earlier C# build on the private mesh. The release benchmark above uses the pinned port for controlled comparisons. [Historical original CLI captures](https://github.com/rhaberer/Instant-Meshes-C-/blob/defc829e13d6694608c68e151fd82943c57d1cd0/InstantMeshesRhino-0.1.3.zip)
