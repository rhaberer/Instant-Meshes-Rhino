# Instant Meshes for Rhino 8

This package provides a C# rewrite of the Instant Meshes solver for field-aligned remeshing of meshes and oriented point clouds. Thanks to Wenzel Jakob, Marco Tarini, Daniele Panozzo, and Olga Sorkine-Hornung for sharing the work that inspired this project. [Original Instant Meshes on GitHub](https://github.com/wjakob/instant-meshes)

- In Rhino, run **InstantMeshesRemesh** and select the input geometry.
- In the Grasshopper bundled with Rhino 8, find **Instant Meshes** and **Instant Meshes (Typed)** under **Mesh → IM**. The first accepts an optional target vertex count and an Options string. The second exposes the options as individual inputs. Right-click either component and choose **Help** for input requirements, examples, and source links.
- Try **Instant Meshes - Options.gh** and **Instant Meshes - Typed.gh** in the examples folder. Both definitions include the input bunny mesh.

The C# solver runs in the Rhino process. Both entry points pass geometry in memory and share the same solver, settings, and cancellation support. Package Manager provides separate Windows and macOS distributions for Rhino 8 SR32 or later. Both require .NET 8. On Windows, use `SetDotNetRuntime` to select .NET Core if Rhino is currently set to .NET Framework, then restart Rhino. The installed macOS SR37 SDK checks passed. Interactive host qualification remains open.

Original project contributions use the MIT License. See `NOTICE.md` and the accompanying licenses for the terms of adapted code and artwork. The corresponding Eigen-derived source is included under `source/`.
