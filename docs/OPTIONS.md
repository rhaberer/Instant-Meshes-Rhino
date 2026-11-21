# Grasshopper remeshing options

**Instant Meshes** takes geometry, an optional vertex target, and an Options string. **Instant Meshes (Typed)** takes geometry and the settings below as separate inputs. Both return the remeshed geometry, actual vertex and face counts, and solve status.

## Wire a Grasshopper definition

1. Connect a Rhino mesh with faces or a Rhino point cloud with at least three points and one consistently oriented normal per point to **Geometry**. For a scripted input, pass a RhinoCommon `Rhino.Geometry.PointCloud` with normals through a generic output. Mesh quads are split into triangles before solving.
2. For the compact component, put the flags in a Panel wired to **Options**. For the typed component, connect numbers and toggles to the corresponding inputs. Leave density inputs empty to use Auto.
3. Preview or bake **Mesh**. Use **Vertices** and **Faces** to check the actual counts. **Status** reports pending work and, on completion, the resolved edge length and timings. Review component runtime messages for invalid inputs or ignored flags. The output normals are recomputed and input UVs, vertex colors, and custom attributes are omitted.

| Options text | Result |
| --- | --- |
| `-v 1000 -D -b` | About 1,000 vertices with quad-dominant output and open mesh boundary alignment. |
| `-s 0.5 -p 6 -S 0` | Triangles with a target edge length of 0.5 model units and zero smoothing passes. |
| `-f 2000 -k 16` | About 2,000 faces, with up to 16 neighbors per sample for a point cloud. |

## Flag reference

| Typed input | Options flag | Meaning and default |
| --- | --- | --- |
| Edge Length | `-s`, `--scale` | Target edge length in document units. |
| Target Faces | `-f`, `--faces` | Approximate face count. |
| Target Vertices | `-v`, `--vertices` | Approximate vertex count. |
| Orientation Symmetry | `-r`, `--rosy` | 2, 4, or 6. Default 4. |
| Position Symmetry | `-p`, `--posy` | 4 for quads, 6 for triangles. Default 4. Both components accept 3 as an alias for 6. |
| Quad Dominant | `-D`, `--dominant` | Mix quads and triangles when position symmetry is 4. Default false gives pure quads with final subdivision. |
| Intrinsic | `-i`, `--intrinsic` | Use intrinsic alignment. Default false uses extrinsic alignment. |
| Align Boundaries | `-b`, `--boundaries` | Align to open boundaries. Default false. |
| Crease Angle | `-c`, `--crease` | Dihedral threshold in degrees. Default −1 disables crease detection. |
| Smoothing Passes | `-S`, `--smooth` | Smoothing iterations for both input types. Mesh results are also reprojected to the input surface. Default 2. Use 0 to skip. |
| Point Neighbors | `-k`, `--knn` | Maximum neighbors per oriented point. Default 10. |
| Stable Topology | `--stable-topology`, `--ordinary` | Stable arithmetic is on by default. If both flags are supplied, the last one wins. |
| Worker Limit | `-t`, `--threads` | The solver uses one worker. Requests for more produce a warning. |
| Deterministic | `-d`, `--deterministic` | Execution is deterministic. Disabling it on the typed component produces a warning. |

Choose a single density target, or leave all targets empty for Auto. In the compact component, a density flag in Options overrides the Target Vertices input. Repeated flags for the same density use the last value. Flags are case-sensitive and decimal values use a dot.

Grasshopper supplies geometry and receives the mesh. One positional path, `-o` or `--output`, `-C` or `--compat`, `-F` or `--fullscreen`, `-h` or `--help`, and the macOS Finder `-psn…` marker are ignored with warnings. Unknown arguments, conflicting density modes, and malformed values produce errors. Export a result from the Mesh output.

## Source references

- [Grasshopper flag parser](../src/InstantMeshes.Grasshopper/ConsoleOptions.cs) and [managed solver settings](../src/InstantMeshes.Core/RemeshOptions.cs)
- [Original Instant Meshes source](https://github.com/wjakob/instant-meshes)
- [Instant Field-Aligned Meshes paper](https://igl.ethz.ch/projects/instant-meshes/)
