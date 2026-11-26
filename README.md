# Instant Meshes for Rhino and Grasshopper

<img src="docs/assets/instant-meshes-icon.png" alt="Instant Meshes Wild bunny icon" width="152" align="right">

A C# rewrite of the Instant Meshes solver for Rhino 8 and its bundled Grasshopper. It remeshes meshes and oriented point clouds into pure quads, quad-dominant meshes, or triangles.

This project grew out of the original Instant Meshes by Wenzel Jakob, Marco Tarini, Daniele Panozzo, and Olga Sorkine-Hornung. Thanks to the team for sharing their research and source code. [Original Instant Meshes on GitHub](https://github.com/wjakob/instant-meshes)

## Why a C# rewrite?

The remeshing algorithm runs entirely in C# inside Rhino’s .NET process. The Rhino command and Grasshopper components share the same solver, pass geometry in memory, report progress, and support cancellation during a solve. [Source map](docs/ARCHITECTURE.md)

# Download and install

On Windows, Rhino must run on .NET 8. If it’s configured to use .NET Framework, run **`_SetDotNetRuntime`**, choose **.NET Core**, and restart Rhino. [McNeel’s runtime guide](https://developer.rhino3d.com/en/guides/rhinocommon/moving-to-dotnet-core/#choosing-the-net-runtime-on-windows)
For an offline install, download the `.yak` matching your platform from [GitHub Releases](https://github.com/rhaberer/Instant-Meshes-Rhino/releases). For a Grasshopper-only install, download `InstantMeshes.Grasshopper.gha` and the accompanying license notices. Copy the `.gha` to **File → Special Folders → Components Folder** in Grasshopper, then restart Rhino. Use one installation method to avoid duplicate components.

## Food4Rhino

You can download the component [here](https://www.food4rhino.com/en/app/instant-meshes-rhino-and-grasshopper-full-c).

## Rhino Package Manager

In Rhino 8 SR32 or later on Windows or macOS, open **`_PackageManager`**, search for `instant-meshes-rhino`, click **Install**, and restart Rhino. This installs the Rhino command, both Grasshopper components, and the two examples. [Rhino Package Manager help](https://docs.mcneel.com/rhino/8/help/en-us/commands/packagemanager.htm)

- In Rhino, run **InstantMeshesRemesh** and select a mesh or oriented point cloud. The command adds a new mesh alongside the original.
- In Grasshopper, look under **Mesh → IM**. **Instant Meshes** has an optional target vertex count and **Options** text input. **Instant Meshes (Typed)** exposes the settings individually. Right-click either component and select **Help** for setup, examples, and source links.
- The two example definitions embed the same original bunny mesh. Open either one to try the workflow. [Options example](examples/Instant%20Meshes%20-%20Options.gh) · [Typed example](examples/Instant%20Meshes%20-%20Typed.gh)

### Install a local Yak package on macOS

Quit Rhino, open Terminal in the folder containing the downloaded package, and run the command below. Use the filename shown on the release page if you downloaded a different version.

~~~sh
"/Applications/Rhino 8.app/Contents/Resources/bin/yak" install \
  ./instant-meshes-rhino-1.0.0-rh8_32-mac.yak
~~~

Restart Rhino after installation. In Rhino's command line, use `_PackageManager` to open the package browser. The underscore selects the English command name on localized installations. McNeel documents `_-PackageManager _Search` for pre-filling the browser search, while the Terminal command above installs an archive directly. [Rhino 8 Mac Package Manager](https://docs.mcneel.com/rhino/8mac/help/en-us/commands/packagemanager.htm) · [Yak install reference](https://developer.rhino3d.com/guides/yak/yak-cli-reference/)

### Install a local Yak package on Windows

Quit Rhino and run the following in PowerShell from the folder containing the downloaded package. Then start Rhino again.

~~~powershell
& "$env:ProgramFiles\Rhino 8\System\Yak.exe" install `
  .\instant-meshes-rhino-1.0.0-rh8_32-win.yak
~~~

Choose one target among edge length, faces, and vertices, or leave density on Auto. Edge length uses document units. Targets are approximate, especially when pure-quad output subdivides the mesh. Enter **-v 1000 -D -b -S 2** in Options to request 1,000 vertices, quad-dominant output, boundary alignment, and two smoothing passes. Recognized standalone-only switches produce warnings. The output carries geometry and normals. Input UVs, vertex colors, and custom attributes are outside the conversion. [Grasshopper options](docs/OPTIONS.md)

## Performance

Version **1.0.0** was benchmarked on 17 configurations against the pinned C++ port on Apple Silicon. These are median solver times in stable mode, Grasshopper’s default. Both solvers used one worker. [Full benchmark, both arithmetic modes](benchmarks/1.0.0.md)

| Input | C++ port | C# 1.0.0 | C# solve time |
| --- | ---: | ---: | ---: |
| Grid, 1,152 triangles | 34 ms | 50 ms | 46% longer |
| Dense torus, 69,120 triangles | 508 ms | 403 ms | 21% lower |
| Stanford Armadillo, 345,944 triangles | 2.949 s | 2.536 s | 14% lower |
| Large torus, 377,280 triangles | 2.771 s | 2.364 s | 15% lower |

Ordered faces and vertex coordinates matched in 33 of 34 comparisons. Stable Stanford Bunny reproduced the known topology difference. Timings exclude file I/O and process launch. [Method and historical upstream CLI comparison](docs/PERFORMANCE.md) · [Parity and compatibility](docs/STATUS.md)

## Build and test locally

Run the commands from the repository root with the .NET 8 SDK or newer. The projects target `net8.0` and restore the pinned Rhino and Grasshopper SDK packages from NuGet. Use Rhino 8 with a modern .NET runtime to load the assembly.

~~~sh
dotnet build src/InstantMeshes.Rhino -c Release
dotnet build src/InstantMeshes.Grasshopper -c Release
dotnet run --project tests/InstantMeshes.ConsoleOptions.Tests -c Release
~~~

### Copy the Grasshopper build

Copy `src/InstantMeshes.Grasshopper/bin/Release/net8.0/InstantMeshes.Grasshopper.gha` into the destination folder. The build compiles the solver and Rhino geometry adapter into the `.gha`.

For a manual Grasshopper install, locate the destination with **File → Special Folders → Components Folder**. Quit Rhino before replacing binaries, then restart Rhino and open Grasshopper. Keep a single installed copy of the assembly to avoid duplicate components. Rhino supplies the Grasshopper and RhinoCommon SDK assemblies. [Grasshopper assembly folders](https://developer.rhino3d.com/api/grasshopper/html/P_Grasshopper_Folders_AssemblyFolders.htm)

### Build a Yak package

For a Grasshopper-only Yak package, stage the `.gha` beside `manifest.yml` and include the notices, licenses, and corresponding Eigen-derived source listed in [NOTICE.md](NOTICE.md). This project's full package also includes the Rhino command, its companion DLLs, and examples. The same .NET assemblies support both platform distributions. On macOS with Rhino 8 installed, run the following to build both packages. Set `YAK_TOOL` to use the standalone Yak CLI on other build hosts. [Yak package layout](https://developer.rhino3d.com/en/guides/yak/the-anatomy-of-a-package/)

~~~sh
bash packaging/build-yak.sh "$PWD/artifacts/dev-release" mac win
~~~

Run `bash packaging/build-release.sh v1.0.0` to build both Yak packages, the standalone `.gha`, and the license notices into `artifacts/release/v1.0.0/`.

See [parity and compatibility](docs/STATUS.md#reproduce) for commands to reproduce the native comparisons and [notices](NOTICE.md) for the original work and pinned revisions.

## License

Original project contributions use the MIT License. Adapted code and artwork retain the licenses listed in the project notices. [MIT License](LICENSE) · [Third-party notices](NOTICE.md)
