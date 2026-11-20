using System.Globalization;
using System.Numerics;
using InstantMeshes.Core;

if (args.Length > 0 && args[0] == "--result")
{
    return ResultParity.Run(args);
}

if (args.Length == 3 && args[0] == "--smoke")
{
    return Smoke.Run(args[1], args[2]);
}

if (args.Length == 9 && args[0] == "--point-stage")
{
    return PointParity.Run(args);
}

if (args.Length is not (3 or 4 or 7))
{
    Console.Error.WriteLine("Usage: InstantMeshes.Parity input.obj reference.bin stable|ordinary [max-edge-length | target-vertices candidates.bin field.bin final.bin]");
    return 2;
}

(Vector3[] vertices, int[] indices, _) = ResultParity.ReadObj(args[0]);

bool stable = args[2] switch
{
    "stable" => true,
    "ordinary" => false,
    _ => throw new ArgumentException("Expected stable or ordinary arithmetic.")
};
float? maxEdgeLength = args.Length == 4 ? ParseFloat(args[3]) : null;
bool compareHierarchy = args.Length == 7;
Vector3[] positions = Preprocessing.CopyPositions(vertices, stable, default);
MeshStatistics statistics = default;
TargetDensity density = default;
if (compareHierarchy)
{
    var options = new RemeshOptions { Density = DensityMode.Vertices, Target = ParseFloat(args[3]) };
    statistics = MeshStatistics.Measure(positions, indices, default);
    density = TargetDensity.Resolve(options, statistics.SurfaceArea, positions.Length);
    maxEdgeLength = statistics.SubdivisionLimit(density.Scale);
}
PreparedMesh mesh = Preprocessing.PrepareOwned(positions, indices, stable, default, maxEdgeLength);
using var reader = new BinaryReader(File.OpenRead(args[1]));
Require(reader.ReadInt32() == mesh.Positions.Length, "vertex count");
Require(reader.ReadInt32() == mesh.Triangles.Length, "index count");
CompareVectors(mesh.Positions, "input positions", 0f);
CompareIntegers(mesh.Triangles, "triangle index");
CompareIntegers(mesh.VertexEdges, "vertex edge");
CompareIntegers(mesh.OppositeEdges, "opposite edge");
for (int i = 0; i < mesh.Positions.Length; i++)
{
    Require(reader.ReadInt32() == (mesh.Boundary[i] ? 1 : 0), $"boundary at {i}");
    Require(reader.ReadInt32() == (mesh.NonManifold[i] ? 1 : 0), $"nonmanifold at {i}");
}
float maxNormalDifference = CompareVectors(mesh.Normals, "vertex normals", stable ? 0f : 2e-6f);
float maxAreaDifference = 0;
for (int i = 0; i < mesh.Areas.Length; i++)
{
    float difference = MathF.Abs(reader.ReadSingle() - mesh.Areas[i]);
    maxAreaDifference = MathF.Max(maxAreaDifference, difference);
    Require(difference <= (stable ? 0f : 1e-7f), $"vertex area at {i} (delta {difference:G9})");
}
CompareIntegers(mesh.NeighborOffsets, "neighbor offset");
CompareIntegers(mesh.Neighbors, "neighbor index");
string hierarchySummary = "";
if (compareHierarchy)
{
    Require(reader.ReadDouble() == statistics.SurfaceArea, "surface area");
    Require(reader.ReadDouble() == statistics.AverageEdgeLength, "average edge length");
    Require(reader.ReadDouble() == statistics.MaximumEdgeLength, "maximum edge length");
    CompareVectors([statistics.Center, statistics.Minimum, statistics.Maximum], "statistics", 0f);
    Require(reader.ReadSingle() == density.Scale, "target scale");

    using var candidateReader = new BinaryReader(File.OpenRead(args[4]));
    Hierarchy hierarchy = Hierarchy.Build(mesh, density.Scale, stable, default, (depth, candidates) =>
    {
        Require(candidateReader.ReadInt32() == candidates.Length, $"level {depth} candidate count");
        for (int i = 0; i < candidates.Length; i++)
        {
            int a = candidateReader.ReadInt32(), b = candidateReader.ReadInt32();
            float score = candidateReader.ReadSingle();
            EdgeCandidate actual = candidates[i];
            Require(a == actual.A && b == actual.B,
                $"level {depth} candidate {i}, expected ({a}, {b}), got ({actual.A}, {actual.B})");
            Require(score.Equals(actual.Score) || MathF.Abs(score - actual.Score) <= 2e-6f * MathF.Max(1, MathF.Abs(score)),
                $"level {depth} candidate {i} score, expected {score:G9}, got {actual.Score:G9}");
        }
    });
    Require(candidateReader.BaseStream.Position == candidateReader.BaseStream.Length, "end of native candidates");
    Require(reader.ReadInt32() == hierarchy.Levels.Count, "hierarchy level count");
    float maxOrientation = 0, maxOrigin = 0;
    for (int depth = 0; depth < hierarchy.Levels.Count; depth++)
    {
        HierarchyLevel level = hierarchy.Levels[depth];
        string label = $"level {depth}";
        Require(reader.ReadInt32() == level.Positions.Length, $"{label} vertex count");
        CompareVectors(level.Positions, $"{label} positions", 0);
        CompareVectors(level.Normals, $"{label} normals", 0);
        for (int i = 0; i < level.Areas.Length; i++)
        {
            Require(reader.ReadSingle() == level.Areas[i], $"{label} area at {i}");
        }

        CompareIntegers(level.Offsets, $"{label} neighbor offset");
        for (int i = 0; i < level.Links.Length; i++)
        {
            Require(reader.ReadInt32() == level.Links[i].Vertex, $"{label} neighbor vertex at {i}");
            Require(reader.ReadSingle() == level.Links[i].Weight, $"{label} neighbor weight at {i}");
        }
        Require(reader.ReadInt32() == level.Phases.Length, $"{label} color count");
        foreach (int[] phase in level.Phases)
        {
            Require(reader.ReadInt32() == phase.Length, $"{label} color size");
            CompareIntegers(phase, $"{label} color vertex");
        }
        maxOrientation = MathF.Max(maxOrientation, CompareVectors(level.Orientations, $"{label} initialized directions", 3e-7f));
        maxOrigin = MathF.Max(maxOrigin, CompareVectors(level.Origins, $"{label} initialized offsets", 0));
        if (depth + 1 < hierarchy.Levels.Count)
        {
            Require(reader.ReadInt32() == level.Children.Length, $"{label} child count");
            CompareIntegers(level.Children, $"{label} fine map");
            CompareIntegers(level.Parents, $"{label} coarse map");
        }
    }
    using var fieldReader = new BinaryReader(File.OpenRead(args[5]));
    float maxFieldDirection = 0, maxFieldOrigin = 0;
    var solveOptions = new RemeshOptions { StableTopology = stable };
    FieldSolver.Solve(hierarchy, mesh, solveOptions, default, (kind, depth, iteration, values) =>
    {
        string label = $"{kind} level {depth} iteration {iteration + 1}";
        Require(fieldReader.ReadInt32() == (int)kind, $"{label} kind");
        Require(fieldReader.ReadInt32() == depth, $"{label} level");
        Require(fieldReader.ReadInt32() == iteration, $"{label} iteration");
        Require(fieldReader.ReadInt32() == values.Length, $"{label} count");
        float max = CompareVectors(values, label, 3e-6f, fieldReader);
        if (kind == FieldKind.Orientation)
        {
            maxFieldDirection = MathF.Max(maxFieldDirection, max);
        }
        else
        {
            maxFieldOrigin = MathF.Max(maxFieldOrigin, max);
        }
    });
    Require(fieldReader.BaseStream.Position == fieldReader.BaseStream.Length, "end of field trace");
    ExtractedGraph graph = ExtractedGraph.Build(hierarchy, mesh, solveOptions, default);
    ExtractedMesh result = FaceExtraction.Extract(graph, density.Scale, solveOptions, mesh, default);
    using var final = new BinaryReader(File.OpenRead(args[6]));
    int expectedVertices = final.ReadInt32(), expectedFaces = final.ReadInt32(), expectedArity = final.ReadInt32();
    Require(expectedVertices == result.Positions.Length && expectedFaces == result.Indices.Length / result.Arity &&
        expectedArity == result.Arity, $"final counts, expected {expectedVertices}/{expectedFaces}/{expectedArity}, " +
        $"got {result.Positions.Length}/{result.Indices.Length / result.Arity}/{result.Arity}");
    long positionsStart = final.BaseStream.Position;
    final.BaseStream.Seek(expectedVertices * 12L, SeekOrigin.Current);
    CompareIntegers(result.Indices, "final ordered index", final);
    Require(final.BaseStream.Position == final.BaseStream.Length, "end of final native dump");
    final.BaseStream.Position = positionsStart;
    float maxPosition = CompareVectors(result.Positions, "output positions", 5e-5f, final);
    hierarchySummary = $", {hierarchy.Levels.Count} hierarchy levels, exact collapse/maps/colors, " +
        $"max initialized direction delta {maxOrientation:G9}, max initialized origin delta {maxOrigin:G9}, " +
        $"max solved direction delta {maxFieldDirection:G9}, max solved origin delta {maxFieldOrigin:G9}, " +
        $"exact ordered output {expectedVertices} vertices/{expectedFaces} faces, max position delta {maxPosition:G9}";
}
Require(reader.BaseStream.Position == reader.BaseStream.Length, "end of native dump");

Console.WriteLine($"{Path.GetFileName(args[0])} {args[2]}: {mesh.Positions.Length} vertices, {mesh.Triangles.Length / 3} triangles, " +
    $"exact adjacency, max normal delta {maxNormalDifference:G9}, max area delta {maxAreaDifference:G9}{hierarchySummary}");
return 0;

static float ParseFloat(string text) => float.Parse(text, CultureInfo.InvariantCulture);

static void Require(bool success, string label)
{
    if (!success)
    {
        throw new InvalidDataException($"Parity disagrees at {label}.");
    }
}

void CompareIntegers(int[] values, string label, BinaryReader? source = null)
{
    source ??= reader;
    for (int i = 0; i < values.Length; i++)
    {
        Require(source.ReadInt32() == values[i], $"{label} at {i}");
    }
}

float CompareVectors(Vector3[] values, string label, float tolerance, BinaryReader? source = null)
{
    source ??= reader;
    float maximum = 0;
    for (int i = 0; i < values.Length; i++)
    {
        var expected = new Vector3(source.ReadSingle(), source.ReadSingle(), source.ReadSingle());
        Vector3 delta = Vector3.Abs(values[i] - expected);
        float difference = MathF.Max(delta.X, MathF.Max(delta.Y, delta.Z));
        maximum = MathF.Max(maximum, difference);
        Require(difference <= tolerance, $"{label} at {i} (delta {difference:G9}, expected {expected}, got {values[i]})");
    }
    return maximum;
}
