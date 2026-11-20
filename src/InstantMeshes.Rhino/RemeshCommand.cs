using System.Diagnostics;
using System.Runtime.InteropServices;
using InstantMeshes.Core;
using InstantMeshes.RhinoGeometry;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.PlugIns;

[assembly: Guid("66bc7203-5962-4178-adb7-7be264b5da02")]

namespace InstantMeshes.RhinoHost;

public sealed class InstantMeshesPlugin : PlugIn;

public sealed class RemeshCommand : Command
{
    public override string EnglishName => "InstantMeshesRemesh";

    protected override Result RunCommand(RhinoDoc doc, RunMode mode)
    {
        try
        {
            using var objectSelector = new GetObject();
            objectSelector.SetCommandPrompt("Select a mesh or oriented point cloud");
            objectSelector.GeometryFilter = ObjectType.Mesh | ObjectType.PointSet;
            objectSelector.SubObjectSelect = false;

            var options = AddOptions(objectSelector);
            GetResult result;
            do
            {
                result = objectSelector.Get();
                if (result == GetResult.Option)
                {
                    options.ChangeSelection(objectSelector.Option());
                }
            } while (result == GetResult.Option);
            if (objectSelector.CommandResult() != Result.Success)
            {
                return objectSelector.CommandResult();
            }

            using ObjRef selected = objectSelector.Object(0);
            var conversion = Stopwatch.StartNew();
            GeometrySnapshot snapshot = selected.Geometry() switch
            {
                Mesh mesh => MeshAdapter.Snapshot(mesh),
                PointCloud cloud => MeshAdapter.Snapshot(cloud),
                _ => throw new ArgumentException("Select a mesh or point cloud.")
            };
            TimeSpan inputTime = conversion.Elapsed;
            RemeshOptions settings = options.Value;
            Save(settings);
            return Solve(doc, snapshot, settings, inputTime);
        }
        catch (OperationCanceledException)
        {
            return Result.Cancel;
        }
        catch (Exception ex) when (ex is ArgumentException or InvalidOperationException or OverflowException)
        {
            RhinoApp.WriteLine(ex.Message);
            return Result.Failure;
        }
    }

    private static Result Solve(RhinoDoc document, GeometrySnapshot source, RemeshOptions options, TimeSpan inputTime)
    {
        var elapsed = Stopwatch.StartNew();
        uint documentSerial = document.RuntimeSerialNumber;
        using var cancellation = new CancellationTokenSource();
        RhinoApp.EscapeKeyPressed += Cancel;
        RhinoDoc.CloseDocument += DocumentClosed;
        try
        {
            RhinoApp.WriteLine("Instant Meshes is remeshing. Press Escape to cancel.");
            Task<RemeshResult> solveTask = Task.Run(() => source.Solve(options, cancellation.Token), cancellation.Token);
            // Pump Rhino events so Escape and document-close cancellation can be observed.
            while (!solveTask.IsCompleted)
            {
                RhinoApp.Wait();
                Thread.Sleep(12);
            }
            RemeshResult result = solveTask.GetAwaiter().GetResult();
            cancellation.Token.ThrowIfCancellationRequested();
            // The original document may have closed while the worker was running.
            RhinoDoc? target = RhinoDoc.FromRuntimeSerialNumber(documentSerial);
            if (target is null || RhinoApp.IsClosing)
            {
                return Result.Cancel;
            }

            var conversion = Stopwatch.StartNew();
            using Mesh output = MeshAdapter.ToMesh(result, source.Origin);
            TimeSpan outputTime = conversion.Elapsed;
            if (target.Objects.AddMesh(output) == Guid.Empty)
            {
                throw new InvalidOperationException("Rhino could not add the remeshed geometry.");
            }

            target.Views.Redraw();
            RhinoApp.WriteLine($"Instant Meshes added {result.VertexCount} vertices and {result.FaceCount} faces.");
            RhinoApp.WriteLine(FormattableString.Invariant(
                $"Input {inputTime.TotalMilliseconds:F1} ms, solve {result.Diagnostics.Elapsed.TotalMilliseconds:F1} ms, output {outputTime.TotalMilliseconds:F1} ms, total {(elapsed.Elapsed + inputTime).TotalMilliseconds:F1} ms."));
            return Result.Success;
        }
        finally
        {
            // Event handlers must not outlive this command or its cancellation source.
            RhinoDoc.CloseDocument -= DocumentClosed;
            RhinoApp.EscapeKeyPressed -= Cancel;
        }

        void Cancel(object? sender, EventArgs e) => cancellation.Cancel();

        void DocumentClosed(object? sender, DocumentEventArgs e)
        {
            if (e.DocumentSerialNumber == documentSerial)
            {
                cancellation.Cancel();
            }
        }
    }

    private CommandOptions AddOptions(GetObject objectSelector)
    {
        var current = new CommandOptions(objectSelector,
            (DensityMode)Math.Clamp(Settings.GetInteger("Density", 0), 0, 3),
            (FaceMode)Math.Clamp(Settings.GetInteger("Faces", 0), 0, 2),
            Settings.GetInteger("Orientation", 4),
            Settings.GetDouble("Target", 1),
            Settings.GetDouble("Crease", -1),
            Settings.GetInteger("Smoothing", 2),
            Settings.GetInteger("Neighbors", 10),
            Settings.GetBool("Extrinsic", true),
            Settings.GetBool("Boundary", false),
            Settings.GetBool("Stable", true));
        return current;
    }

    private void Save(RemeshOptions options)
    {
        Settings.SetInteger("Density", (int)options.Density);
        Settings.SetInteger("Faces", (int)options.Faces);
        Settings.SetInteger("Orientation", options.OrientationSymmetry);
        Settings.SetDouble("Target", options.Target);
        Settings.SetDouble("Crease", options.CreaseAngle ?? -1);
        Settings.SetInteger("Smoothing", options.SmoothingPasses);
        Settings.SetInteger("Neighbors", options.PointNeighbors);
        Settings.SetBool("Extrinsic", options.Extrinsic);
        Settings.SetBool("Boundary", options.AlignToBoundaries);
        Settings.SetBool("Stable", options.StableTopology);
    }

    private sealed class CommandOptions
    {
        private static readonly int[] Orientations = [2, 4, 6];
        private readonly int densityOption, faceOption, orientationOption;
        private OptionDouble target, crease;
        private OptionInteger smoothing, neighbors;
        private OptionToggle extrinsic, boundary, stable;
        private DensityMode density;
        private FaceMode faces;
        private int orientation;

        public CommandOptions(GetObject objectSelector, DensityMode density, FaceMode faces, int orientation,
            double target, double crease, int smoothing, int neighbors, bool extrinsic, bool boundary, bool stable)
        {
            this.density = density;
            this.faces = faces;
            int index = Array.IndexOf(Orientations, orientation);
            this.orientation = index >= 0 ? index : 1;
            this.target = new(target, 0, double.MaxValue);
            this.crease = new(crease, -1, 180);
            this.smoothing = new(smoothing, 0, int.MaxValue);
            this.neighbors = new(neighbors, 1, int.MaxValue);
            this.extrinsic = new(extrinsic, "Intrinsic", "Extrinsic");
            this.boundary = new(boundary, "No", "Yes");
            this.stable = new(stable, "Off", "On");
            densityOption = objectSelector.AddOptionList("Density", Enum.GetNames<DensityMode>(), (int)density);
            faceOption = objectSelector.AddOptionList("Faces", Enum.GetNames<FaceMode>(), (int)faces);
            orientationOption = objectSelector.AddOptionList("Orientation", ["Two", "Four", "Six"], this.orientation);
            objectSelector.AddOptionDouble("Target", ref this.target);
            objectSelector.AddOptionDouble("CreaseAngle", ref this.crease);
            objectSelector.AddOptionInteger("Smoothing", ref this.smoothing);
            objectSelector.AddOptionInteger("PointNeighbors", ref this.neighbors);
            objectSelector.AddOptionToggle("Field", ref this.extrinsic);
            objectSelector.AddOptionToggle("AlignBoundaries", ref this.boundary);
            objectSelector.AddOptionToggle("StableTopology", ref this.stable);
        }

        public void ChangeSelection(CommandLineOption option)
        {
            if (option.Index == densityOption)
            {
                density = (DensityMode)option.CurrentListOptionIndex;
            }
            else if (option.Index == faceOption)
            {
                faces = (FaceMode)option.CurrentListOptionIndex;
            }
            else if (option.Index == orientationOption)
            {
                orientation = option.CurrentListOptionIndex;
            }
        }

        public RemeshOptions Value => new()
        {
            Density = density,
            Target = target.CurrentValue,
            Faces = faces,
            OrientationSymmetry = Orientations[orientation],
            Extrinsic = extrinsic.CurrentValue,
            AlignToBoundaries = boundary.CurrentValue,
            CreaseAngle = crease.CurrentValue >= 0 ? (float)crease.CurrentValue : null,
            SmoothingPasses = smoothing.CurrentValue,
            PointNeighbors = neighbors.CurrentValue,
            StableTopology = stable.CurrentValue
        };
    }
}
