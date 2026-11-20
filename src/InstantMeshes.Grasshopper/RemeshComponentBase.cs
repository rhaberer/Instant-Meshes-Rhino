using System.Diagnostics;
using System.Drawing;
using System.Runtime.CompilerServices;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using InstantMeshes.Core;
using InstantMeshes.RhinoGeometry;
using Rhino;
using Rhino.Geometry;

namespace InstantMeshes.GrasshopperHost;

public sealed class InstantMeshesAssembly : GH_AssemblyInfo
{
    public override string Name => "Instant Meshes";
    public override string Description => "Field-aligned remeshing with Instant Meshes, by Jakob, Tarini, Panozzo and Sorkine-Hornung.";
    public override string AuthorName => "Ronny Haberer";
    public override Guid Id => new("f06145fb-0e43-4529-8de6-fde87fca50b8");
}

public abstract class RemeshComponentBase : GH_Component
{
    // Serialize solves across components while their memory budget is unqualified.
    private static readonly SemaphoreSlim SolverSlot = new(1);

    private readonly Dictionary<int, Input> pendingInputs = [];
    private Batch? activeBatch;

    private sealed record Input(GeometrySnapshot Geometry, RemeshOptions Options, TimeSpan Conversion)
    {
        public bool Matches(Input other) => Options == other.Options && Geometry.HasSameGeometry(other.Geometry);
    }

    private sealed record Outcome(RemeshResult? Mesh, string? Error);

    private sealed class Batch
    {
        public Stopwatch WallTime { get; } = Stopwatch.StartNew();
        private readonly CancellationTokenSource cancellation = new();
        private readonly object lifetime = new();
        private bool disposed;

        public Dictionary<int, Input> Inputs { get; }
        public Dictionary<int, Outcome>? Results { get; set; }
        public CancellationToken Token { get; }

        public Batch(Dictionary<int, Input> inputs) =>
            (Inputs, Token) = (inputs, cancellation.Token);

        // Cancellation and disposal may arrive on different threads.
        public void Cancel()
        {
            lock (lifetime)
            {
                if (!disposed)
                {
                    cancellation.Cancel();
                }
            }
        }

        public void Dispose()
        {
            lock (lifetime)
            {
                disposed = true;
                cancellation.Dispose();
            }
        }
    }

    protected RemeshComponentBase(string name, string nickname, string description)
        : base(name, nickname, description, "Mesh", "IM")
    {
    }

    protected override Bitmap Icon
    {
        get
        {
            using Stream stream = typeof(RemeshComponentBase).Assembly.GetManifestResourceStream(
                "InstantMeshes.Grasshopper.ComponentIcon.png")!;
            using var bitmap = new Bitmap(stream);
            return new Bitmap(bitmap);
        }
    }

    protected override string HelpDescription => """
        <p>Connect a Rhino mesh with faces or a Rhino point cloud with at least three points and a
        consistently oriented normal per point. Mesh quads are triangulated before solving.
        The output mesh has recomputed normals. UVs, vertex colors, and custom attributes are omitted.</p>
        <p>The solve runs in the background. Status shows <em>Remeshing…</em> while work is pending.
        When the result is ready, Grasshopper updates Mesh, Vertices, Faces, and Status.
        Check the component's runtime messages for warnings or errors.</p>
        """;

    protected const string HelpSources = """
        <h3>Sources</h3>
        <ul>
          <li><a href="https://github.com/rhaberer/Instant-Meshes-Rhino">Rhino and Grasshopper project</a></li>
          <li><a href="https://igl.ethz.ch/projects/instant-meshes/">Instant Field-Aligned Meshes paper</a></li>
          <li><a href="https://github.com/wjakob/instant-meshes">Original Instant Meshes source</a></li>
        </ul>
        """;

    protected sealed override void RegisterInputParams(GH_InputParamManager parameters)
    {
        parameters.AddGenericParameter("Geometry", "G",
            "Rhino mesh with faces, or a Rhino point cloud with at least three points and an oriented normal per point.",
            GH_ParamAccess.item);
        RegisterSettings(parameters);
    }

    protected abstract void RegisterSettings(GH_InputParamManager parameters);
    protected abstract RemeshOptions ReadOptions(IGH_DataAccess data);

    protected override void RegisterOutputParams(GH_OutputParamManager parameters)
    {
        parameters.AddMeshParameter("Mesh", "M", "Remeshed geometry", GH_ParamAccess.item);
        parameters.AddIntegerParameter("Vertices", "V", "Actual vertex count", GH_ParamAccess.item);
        parameters.AddIntegerParameter("Faces", "F", "Actual face count", GH_ParamAccess.item);
        parameters.AddTextParameter("Status", "S",
            "Shows Remeshing… while work is pending, then the resolved edge length and timings or an error message.",
            GH_ParamAccess.item);
    }

    protected override void BeforeSolveInstance() => pendingInputs.Clear();

    protected override void SolveInstance(IGH_DataAccess data)
    {
        IGH_Goo? geometry = null;
        if (!data.GetData(0, ref geometry) || geometry is null)
        {
            return;
        }

        try
        {
            var conversionTimer = Stopwatch.StartNew();
            GeometrySnapshot snapshot = geometry.ScriptVariable() switch
            {
                Mesh mesh => MeshAdapter.Snapshot(mesh),
                PointCloud cloud => MeshAdapter.Snapshot(cloud),
                _ => throw new ArgumentException("Connect a mesh or an oriented point cloud.")
            };
            var item = new Input(snapshot, ReadOptions(data), conversionTimer.Elapsed);
            int index = data.Iteration;
            pendingInputs[index] = item;
            if (activeBatch?.Results is not { } completedResults ||
                !activeBatch.Inputs.TryGetValue(index, out Input? previous) || !item.Matches(previous))
            {
                data.SetData(3, "Remeshing…");
                return;
            }
            if (!completedResults.TryGetValue(index, out Outcome? outcome))
            {
                return;
            }

            if (outcome.Error is { } error)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
                data.SetData(3, error);
                return;
            }

            RemeshResult result = outcome.Mesh!;
            conversionTimer.Restart();
            data.SetData(0, MeshAdapter.ToMesh(result, snapshot.Origin));
            TimeSpan outputTime = conversionTimer.Elapsed;
            data.SetData(1, result.VertexCount);
            data.SetData(2, result.FaceCount);
            data.SetData(3, FormattableString.Invariant(FormattableStringFactory.Create(
                "Complete · edge length {0:G6} · input {1:F1} ms · solve {2:F1} ms · output {3:F1} ms · total {4:F1} ms",
                result.Diagnostics.EdgeLength,
                previous.Conversion.TotalMilliseconds,
                result.Diagnostics.Elapsed.TotalMilliseconds,
                outputTime.TotalMilliseconds,
                (activeBatch.WallTime.Elapsed + previous.Conversion + outputTime).TotalMilliseconds)));
        }
        catch (Exception ex) when (ex is ArgumentException or InvalidOperationException or OverflowException)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
            data.SetData(3, ex.Message);
        }
    }

    protected override void AfterSolveInstance()
    {
        if (activeBatch is not null && activeBatch.Inputs.Count == pendingInputs.Count &&
            pendingInputs.All(item => activeBatch.Inputs.TryGetValue(item.Key, out Input? previous) && item.Value.Matches(previous)))
        {
            pendingInputs.Clear();
            return;
        }
        // A changed input supersedes the previous batch. Its completion must not publish.
        Cancel();
        GH_Document? document = OnPingDocument();
        if (pendingInputs.Count > 0 && document is not null)
        {
            var batch = new Batch(new(pendingInputs));
            activeBatch = batch;
            _ = Run(document, batch);
        }
        pendingInputs.Clear();
    }

    public override void RemovedFromDocument(GH_Document document)
    {
        Cancel();
        base.RemovedFromDocument(document);
    }

    public override void DocumentContextChanged(GH_Document document, GH_DocumentContext context)
    {
        if (context is GH_DocumentContext.Close or GH_DocumentContext.Unloaded)
        {
            Cancel();
        }

        base.DocumentContextChanged(document, context);
    }

    private void Cancel()
    {
        activeBatch?.Cancel();
        activeBatch = null;
    }

    private async Task Run(GH_Document document, Batch batch)
    {
        var outcomes = new Dictionary<int, Outcome>(batch.Inputs.Count);
        CancellationToken cancellation = batch.Token;
        try
        {
            await SolverSlot.WaitAsync(cancellation).ConfigureAwait(false);
            try
            {
                await Task.Run(() =>
                {
                    foreach ((int index, Input input) in batch.Inputs)
                    {
                        cancellation.ThrowIfCancellationRequested();
                        try
                        {
                            outcomes[index] = new(input.Geometry.Solve(input.Options, cancellation), null);
                        }
                        catch (Exception ex) when (ex is not OperationCanceledException)
                        {
                            outcomes[index] = new(null, ex.Message);
                        }
                    }
                }, cancellation).ConfigureAwait(false);
            }
            finally
            {
                SolverSlot.Release();
            }

            if (!cancellation.IsCancellationRequested && !RhinoApp.IsClosing)
            {
                // Grasshopper state and solution scheduling belong to the UI thread.
                RhinoApp.InvokeOnUiThread((Action)(() =>
                {
                    // Ignore results from a superseded batch or a detached component.
                    if (activeBatch != batch || OnPingDocument() != document)
                    {
                        return;
                    }

                    batch.WallTime.Stop();
                    batch.Results = outcomes;
                    if (document.SolutionDepth == 0)
                    {
                        ExpireSolution(false);
                        document.ScheduleSolution(1);
                    }
                    else
                    {
                        document.ScheduleSolution(1, _ =>
                        {
                            if (activeBatch == batch && OnPingDocument() == document)
                            {
                                ExpireSolution(false);
                            }
                        });
                    }
                }));
            }
        }
        catch (OperationCanceledException) when (cancellation.IsCancellationRequested)
        {
        }
        catch (Exception ex) when (RhinoApp.IsClosing)
        {
            System.Diagnostics.Trace.WriteLine(ex);
        }
        finally
        {
            batch.Dispose();
        }
    }

}
