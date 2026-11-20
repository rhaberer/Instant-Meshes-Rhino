using System.Diagnostics;
using System.Numerics;

namespace InstantMeshes.Core;

public enum RemeshStage
{
    Preprocessing,
    Hierarchy,
    Fields,
    Extraction,
    Complete
}

public sealed record RemeshTimings(TimeSpan Preprocessing, TimeSpan Hierarchy,
    TimeSpan Orientation, TimeSpan Position, TimeSpan GraphExtraction, TimeSpan FaceExtraction,
    TimeSpan Reprojection);

public sealed record RemeshDiagnostics(int InputVertices, int PreparedVertices, int HierarchyLevels,
    int TargetFaces, int TargetVertices, float EdgeLength, int FilteredPointCount,
    int RegularizedAreaCount, TimeSpan Elapsed, RemeshTimings Timings);

public sealed record RemeshResult(Vector3[] Positions, Vector3[] VertexNormals, int[] Indices,
    Vector3[] FaceNormals, int FaceArity, RemeshDiagnostics Diagnostics)
{
    public int VertexCount => Positions.Length;
    public int FaceCount => Indices.Length / FaceArity;
}

public static class Remesher
{
    public static RemeshResult RemeshTriangles(ReadOnlySpan<Vector3> positions, ReadOnlySpan<int> indices,
        RemeshOptions? options = null, CancellationToken cancellationToken = default,
        IProgress<RemeshStage>? progress = null)
    {
        var watch = Stopwatch.StartNew();
        options ??= new();
        Validate(options);
        cancellationToken.ThrowIfCancellationRequested();
        progress?.Report(RemeshStage.Preprocessing);

        bool stable = options.StableTopology;
        // The solver owns these copies; subdivision and canonicalization must not mutate caller data.
        Vector3[] vertices = Preprocessing.CopyPositions(positions, stable, cancellationToken);
        int[] faces = indices.ToArray();
        MeshStatistics statistics = MeshStatistics.Measure(vertices, faces, cancellationToken);
        TargetDensity density = TargetDensity.Resolve(options, statistics.SurfaceArea, vertices.Length);
        PreparedMesh mesh = Preprocessing.PrepareOwned(vertices, faces, stable, cancellationToken,
            statistics.SubdivisionLimit(density.Scale), options.CreaseAngle);

        TimeSpan preprocessing = watch.Elapsed;
        progress?.Report(RemeshStage.Hierarchy);
        Hierarchy hierarchy = Hierarchy.Build(mesh, density.Scale, stable, cancellationToken);
        return Finish(hierarchy, mesh, options, density, positions.Length, 0,
            watch, preprocessing, cancellationToken, progress);
    }

    public static RemeshResult RemeshPoints(ReadOnlySpan<Vector3> positions, ReadOnlySpan<Vector3> normals,
        RemeshOptions? options = null, CancellationToken cancellationToken = default,
        IProgress<RemeshStage>? progress = null)
    {
        var watch = Stopwatch.StartNew();
        options ??= new();
        Validate(options);
        cancellationToken.ThrowIfCancellationRequested();
        progress?.Report(RemeshStage.Preprocessing);
        bool stable = options.StableTopology;
        Vector3[] vertices = Preprocessing.CopyPositions(positions, stable, cancellationToken);
        (HierarchyLevel level, double surfaceArea, int filteredPoints) =
            PointCloudPreprocessing.Prepare(vertices, normals, options, cancellationToken);
        TargetDensity density = TargetDensity.Resolve(options, surfaceArea, vertices.Length);
        TimeSpan preprocessing = watch.Elapsed;
        progress?.Report(RemeshStage.Hierarchy);
        Hierarchy hierarchy = Hierarchy.Build(level, density.Scale, stable, cancellationToken);
        return Finish(hierarchy, null, options, density, positions.Length, filteredPoints,
            watch, preprocessing, cancellationToken, progress);
    }

    private static RemeshResult Finish(Hierarchy hierarchy, PreparedMesh? mesh, RemeshOptions options,
        TargetDensity density, int inputCount, int filtered, Stopwatch watch, TimeSpan preprocessing,
        CancellationToken token,
        IProgress<RemeshStage>? progress)
    {
        // Both mesh and point inputs share field solving and extraction from this point on.
        TimeSpan hierarchyTime = watch.Elapsed - preprocessing;
        progress?.Report(RemeshStage.Fields);
        (TimeSpan orientationTime, TimeSpan positionTime) = FieldSolver.Solve(hierarchy, mesh, options, token);
        progress?.Report(RemeshStage.Extraction);
        var timer = Stopwatch.StartNew();
        ExtractedGraph graph = ExtractedGraph.Build(hierarchy, mesh, options, token);
        TimeSpan graphTime = timer.Elapsed;
        timer.Restart();
        ExtractedMesh result = FaceExtraction.Extract(graph, density.Scale, options, mesh, token);
        token.ThrowIfCancellationRequested();
        var timings = new RemeshTimings(preprocessing, hierarchyTime, orientationTime, positionTime,
            graphTime, timer.Elapsed - result.Reprojection, result.Reprojection);
        var diagnostics = new RemeshDiagnostics(inputCount, hierarchy.Levels[0].Positions.Length,
            hierarchy.Levels.Count, density.Faces, density.Vertices, density.Scale, filtered,
            hierarchy.RegularizedAreas, watch.Elapsed, timings);
        progress?.Report(RemeshStage.Complete);
        return new(result.Positions, result.Normals, result.Indices, result.FaceNormals, result.Arity, diagnostics);
    }

    private static void Validate(RemeshOptions options)
    {
        if (options.OrientationSymmetry is not (2 or 4 or 6))
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Orientation symmetry must be 2, 4 or 6.");
        }

        if (options.CreaseAngle is float angle && (!float.IsFinite(angle) || angle < 0 || angle > 180))
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Crease angle must be between 0 and 180 degrees.");
        }

        ArgumentOutOfRangeException.ThrowIfNegative(options.SmoothingPasses);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(options.PointNeighbors);
    }
}
