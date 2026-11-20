using System.Numerics;
using System.Text.Json;
using InstantMeshes.Core;

internal static class PointParity
{
    public static int Run(string[] args)
    {
        (Vector3[] positions, _, Vector3[] normals) = ResultParity.ReadObj(args[1]);
        bool stable = args[6] == "1";
        var options = new RemeshOptions
        {
            StableTopology = stable,
            Density = DensityMode.Vertices,
            Target = int.Parse(args[7]),
            Faces = args[8] == "1" ? FaceMode.PureQuads : FaceMode.QuadDominant
        };
        positions = Preprocessing.CopyPositions(positions, stable, default);
        using var directed = new BinaryReader(File.OpenRead(args[2]));
        int directedOrdering = 0, directedMembership = 0, distanceTieRows = 0;
        object? firstDirected = null;
        var (level, area, filtered) = PointCloudPreprocessing.Prepare(positions, normals, options,
            default, (vertex, actual) =>
            {
                int[] expected = ReadIntegers(directed, directed.ReadInt32());
                if (expected.GroupBy(j => Preprocessing.Dot(positions[j] - positions[vertex],
                        positions[j] - positions[vertex])).Any(group => group.Count() > 1))
                {
                    distanceTieRows++;
                }

                if (expected.AsSpan().SequenceEqual(actual))
                {
                    return;
                }

                firstDirected ??= new { vertex, expected, actual };
                directedOrdering++;
                if (!expected.Order().SequenceEqual(actual.Order()))
                {
                    directedMembership++;
                }
            });
        RequireEnd(directed);
        using var adjacency = new BinaryReader(File.OpenRead(args[3]));
        double nativeArea = adjacency.ReadDouble(), nativeRadius = adjacency.ReadDouble();
        int[] offsets = ReadIntegers(adjacency, positions.Length + 1);
        int[] links = ReadIntegers(adjacency, offsets[^1]);
        int orderingDifferences = 0, membershipDifferences = 0;
        object? firstDifference = null;
        for (int i = 0; i < positions.Length; i++)
        {
            int[] expected = links[offsets[i]..offsets[i + 1]];
            int[] actual = level.Neighbors(i).ToArray().Select(link => link.Vertex).ToArray();
            if (expected.AsSpan().SequenceEqual(actual))
            {
                continue;
            }

            firstDifference ??= new { vertex = i, expected, actual };
            orderingDifferences++;
            if (!expected.Order().SequenceEqual(actual.Order()))
            {
                membershipDifferences++;
            }
        }
        float maxPositionComponentDelta = CompareVectors(adjacency, level.Positions);
        float maxNormalComponentDelta = CompareVectors(adjacency, level.Normals);
        RequireEnd(adjacency);
        using var radius = new BinaryReader(File.OpenRead(args[4]));
        var radiusChunks = new List<object>();
        while (radius.BaseStream.Position < radius.BaseStream.Length)
        {
            int first = radius.ReadInt32(), end = radius.ReadInt32();
            double nativePreciseSum = radius.ReadDouble();
            radiusChunks.Add(new { first, end, nativePreciseSum, truncated = (long)nativePreciseSum });
        }

        double managedRadius = Math.Sqrt(area / (0.5 * Math.PI * positions.Length));
        int iterations = 0;
        float maxDirectionComponentDelta = 0, maxOriginComponentDelta = 0;
        string? fieldDifference = null;
        var density = TargetDensity.Resolve(options, area, positions.Length);
        using var field = new BinaryReader(File.OpenRead(args[5]));
        if (orderingDifferences == 0 && maxPositionComponentDelta == 0 && maxNormalComponentDelta == 0)
        {
            Hierarchy hierarchy = Hierarchy.Build(level, density.Scale, stable, default);
            try
            {
                FieldSolver.Solve(hierarchy, null, options, default, (kind, depth, iteration, values) =>
                {
                    if (field.ReadInt32() != (int)kind || field.ReadInt32() != depth ||
                        field.ReadInt32() != iteration || field.ReadInt32() != values.Length)
                    {
                        throw new InvalidDataException($"Field trace header differs at {kind}, level {depth}, iteration {iteration + 1}.");
                    }

                    float delta = CompareVectors(field, values);
                    if (delta > 3e-6f)
                    {
                        fieldDifference ??= $"{kind}, level {depth}, iteration {iteration + 1}, delta {delta:G9}";
                    }

                    if (kind == FieldKind.Orientation)
                    {
                        maxDirectionComponentDelta = MathF.Max(maxDirectionComponentDelta, delta);
                    }
                    else
                    {
                        maxOriginComponentDelta = MathF.Max(maxOriginComponentDelta, delta);
                    }

                    iterations++;
                });
                RequireEnd(field);
            }
            catch (Exception exception) when (exception is EndOfStreamException or InvalidDataException)
            {
                fieldDifference = exception.Message;
            }
        }

        Console.WriteLine(JsonSerializer.Serialize(new
        {
            nativeArea,
            managedArea = area,
            nativeRadius,
            managedRadius,
            radiusChunks,
            directedOrderingDifferences = directedOrdering,
            directedMembershipDifferences = directedMembership,
            distanceTieRows,
            firstDirected,
            nativeLinks = links.Length,
            managedLinks = level.Links.Length,
            orderingDifferences,
            membershipDifferences,
            firstDifference,
            filtered,
            maxPositionComponentDelta,
            maxNormalComponentDelta,
            iterations,
            maxDirectionComponentDelta,
            maxOriginComponentDelta,
            fieldDifference
        }));
        return 0;
    }

    private static int[] ReadIntegers(BinaryReader reader, int count)
    {
        var values = new int[count];
        for (int i = 0; i < values.Length; i++)
        {
            values[i] = reader.ReadInt32();
        }

        return values;
    }

    private static float CompareVectors(BinaryReader reader, Vector3[] values)
    {
        float max = 0;
        foreach (Vector3 actual in values)
        {
            var expected = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            Vector3 delta = Vector3.Abs(actual - expected);
            max = MathF.Max(max, MathF.Max(delta.X, MathF.Max(delta.Y, delta.Z)));
        }
        return max;
    }

    private static void RequireEnd(BinaryReader reader)
    {
        if (reader.BaseStream.Position != reader.BaseStream.Length)
        {
            throw new InvalidDataException("Unexpected data after the native trace.");
        }
    }
}
