using System.Numerics;
using InstantMeshes.Core;

internal static class Smoke
{
    public static int Run(string cloudPath, string gridPath)
    {
        (Vector3[] cloud, _, Vector3[] normals) = ResultParity.ReadObj(cloudPath);
        (Vector3[] grid, int[] faces, _) = ResultParity.ReadObj(gridPath);

        Expect<ArgumentException>(() => Remesher.RemeshPoints(cloud, []));
        Vector3 normal = normals[0];
        normals[0] = Vector3.Zero;
        Expect<ArgumentException>(() => Remesher.RemeshPoints(cloud, normals));
        normals[0] = normal;
        Expect<ArgumentException>(() => Remesher.RemeshPoints(new Vector3[3], normals[..3]));
        Expect<ArgumentException>(() => Remesher.RemeshPoints(cloud[..3], [Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ]));
        Vector3[] tinyCloud = cloud.Select(point => point / 1024f).ToArray();
        RemeshResult tiny = Remesher.RemeshPoints(tinyCloud, normals,
            new RemeshOptions { Density = DensityMode.Vertices, Target = 160, StableTopology = false });
        if (tiny.FaceCount == 0 || !(tiny.Diagnostics.EdgeLength > 0))
        {
            throw new InvalidDataException("The point cloud lost usable spacing when the native sum truncates to zero.");
        }

        Expect<ArgumentException>(() => Remesher.RemeshTriangles([Vector3.Zero], [0, 0, 0]));
        Expect<ArgumentOutOfRangeException>(() => Remesher.RemeshTriangles(grid, faces,
            new RemeshOptions { Density = DensityMode.EdgeLength, Target = 0 }));

        foreach (RemeshStage stage in Enum.GetValues<RemeshStage>().Where(stage => stage != RemeshStage.Complete))
        {
            using var cancellation = new CancellationTokenSource();
            var progress = new CancelAt(stage, cancellation);
            Expect<OperationCanceledException>(() => Remesher.RemeshTriangles(grid, faces,
                cancellationToken: cancellation.Token, progress: progress));
        }

        RemeshResult edge = Remesher.RemeshTriangles(grid, faces,
            new RemeshOptions { Density = DensityMode.EdgeLength, Target = 0.1 });
        RemeshResult count = Remesher.RemeshTriangles(grid, faces,
            new RemeshOptions
            {
                Density = DensityMode.Faces,
                Target = 100,
                Faces = FaceMode.Triangles,
                OrientationSymmetry = 6,
                Extrinsic = false
            });
        if (edge.FaceCount == 0 || edge.Diagnostics.EdgeLength != 0.1f ||
            count.FaceCount == 0 || count.Diagnostics.TargetFaces != 100 || count.FaceArity != 3)
        {
            throw new InvalidDataException("Explicit density validation failed.");
        }

        Vector3[] disconnected = grid.Concat(grid.Select(p => p + Vector3.UnitX * 2)).ToArray();
        int[] disconnectedFaces = faces.Concat(faces.Select(v => v + grid.Length)).ToArray();
        RemeshResult separate = Remesher.RemeshTriangles(disconnected, disconnectedFaces,
            new RemeshOptions { Density = DensityMode.Vertices, Target = 400 });
        for (int i = 0; i < separate.Indices.Length; i += separate.FaceArity)
        {
            bool left = separate.Positions[separate.Indices[i]].X < 1.5f;
            for (int j = 1; j < separate.FaceArity; j++)
            {
                if ((separate.Positions[separate.Indices[i + j]].X < 1.5f) != left)
                {
                    throw new InvalidDataException("A face connects disjoint input components.");
                }
            }
        }

        const int copies = 64;
        Vector3[] largeCloud = Enumerable.Range(0, copies)
            .SelectMany(copy => cloud.Select(point => point + Vector3.UnitX * (2 * copy))).ToArray();
        Vector3[] largeNormals = Enumerable.Range(0, copies).SelectMany(_ => normals).ToArray();
        using (var cancellation = new CancellationTokenSource())
        {
            var progress = new CancelAt(RemeshStage.Preprocessing, cancellation, delayMilliseconds: 5);
            Expect<OperationCanceledException>(() =>
                Remesher.RemeshPoints(largeCloud, largeNormals, cancellationToken: cancellation.Token, progress: progress));
            if (progress.LastStage != RemeshStage.Preprocessing)
            {
                throw new InvalidDataException("The point query proceeded after cancellation during preprocessing.");
            }
        }

        VerifyCanonicalVectors();
        VerifyCornerTies();
        VerifyRadiusReduction();
        VerifyZeroAreaScores();
        Vector3[] large = Enumerable.Range(0, copies)
            .SelectMany(copy => grid.Select(point => point + Vector3.UnitX * (2 * copy))).ToArray();
        int[] largeFaces = Enumerable.Range(0, copies)
            .SelectMany(copy => faces.Select(vertex => vertex + grid.Length * copy)).ToArray();
        using (var cancellation = new CancellationTokenSource())
        {
            var progress = new CancelAt(RemeshStage.Fields, cancellation, delayMilliseconds: 5);
            Expect<OperationCanceledException>(() =>
                Remesher.RemeshTriangles(large, largeFaces, cancellationToken: cancellation.Token, progress: progress));
        }

        if (Preprocessing.FixedUnits(0.0078125f) != 7813 || Preprocessing.FixedUnits(-0.0078125f) != -7813)
        {
            throw new InvalidDataException("Fixed-point midpoint rounding disagrees.");
        }

        Expect<OverflowException>(() => Preprocessing.FixedUnits(float.MaxValue));
        Console.WriteLine("Density, cancellation, disconnected input, fixed arithmetic, zero-area scores and invalid-input checks passed.");
        return 0;
    }

    private static void VerifyCanonicalVectors()
    {
        const float limit = (float)(9223372036854775808.0 / 1_000_000.0);
        float[] values = [-0f, 0f, float.Epsilon, -float.Epsilon, 0.0078125f, -0.0078125f,
            MathF.BitDecrement(0.0078125f), MathF.BitIncrement(0.0078125f),
            8_388_607f, MathF.BitDecrement(limit), -MathF.BitDecrement(limit)];
        for (int i = 0; i < values.Length; i++)
        {
            Vector3 input = new(values[i], values[(i + 1) % values.Length], values[(i + 2) % values.Length]);
            Vector3 quantized = Preprocessing.Canonical(input, true);
            for (int lane = 0; lane < 3; lane++)
            {
                float expected = Preprocessing.Canonical(input[lane], true);
                if (BitConverter.SingleToInt32Bits(quantized[lane]) != BitConverter.SingleToInt32Bits(expected))
                {
                    throw new InvalidDataException("Canonical vector component differs from checked scalar rounding.");
                }
            }
        }
        foreach (float invalid in new[] { MathF.BitIncrement(limit), -MathF.BitIncrement(limit),
            float.PositiveInfinity, float.NaN })
        {
            for (int lane = 0; lane < 3; lane++)
            {
                Vector3 value = Vector3.One;
                value[lane] = invalid;
                Expect<OverflowException>(() => Preprocessing.Canonical(value, true));
            }
        }
    }

    private static void VerifyCornerTies()
    {
        // The first zero cost is (A0, B1). Column-first tie breaking would choose (A1, B0).
        Vector3[] a = [Vector3.Zero, Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ];
        Vector3[] b = [Vector3.UnitX, Vector3.Zero, Vector3.UnitZ, Vector3.UnitY];
        Check();
        var random = new Random(422);
        for (int trial = 0; trial < 2000; trial++)
        {
            for (int i = 0; i < 4; i++)
            {
                a[i] = Point();
                b[i] = Point();
            }
            Check();
        }

        Vector3 Point() => new(random.Next(5), random.Next(5), random.Next(5));

        void Check()
        {
            var expected = (a[0], b[0]);
            float best = float.PositiveInfinity;
            foreach (Vector3 left in a)
            {
                foreach (Vector3 right in b)
                {
                    Vector3 delta = left - right;
                    float cost = Preprocessing.Dot(delta, delta);
                    if (cost < best)
                    {
                        (expected, best) = ((left, right), cost);
                    }
                }
            }

            if (FieldGeometry.ClosestPositionCorners(a, b) != expected)
            {
                throw new InvalidDataException("Corner search differs from the ordered scalar search.");
            }
        }
    }

    private static void VerifyRadiusReduction()
    {
        const int count = 1538;
        Vector3[] points = Enumerable.Range(0, count)
            .Select(i => new Vector3(i % 38, 0, i / 38) / 512f).ToArray();
        Vector3[] normals = Enumerable.Repeat(Vector3.UnitY, count).ToArray();
        var (_, area, _) = PointCloudPreprocessing.Prepare(points, normals, new(), default);
        // Each of two 769-point chunks sums to 769/512 and truncates to one.
        float radius = 3f * (float)(2d / count);
        if (area != Math.PI * radius * radius * 0.5 * count)
        {
            throw new InvalidDataException("Point spacing must truncate each native reduction chunk before summing.");
        }
    }

    private static void VerifyZeroAreaScores()
    {
        Vector3[] points = [Vector3.Zero, Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ,
            Vector3.One, new Vector3(2)];
        Vector3[] normals = [Vector3.UnitX, Vector3.UnitX, Vector3.UnitX, -Vector3.UnitX,
            Vector3.UnitY, Vector3.UnitY];
        float[] areas = [0, 0, 1, 1, 1, 1];
        var level = new HierarchyLevel(points, normals, areas, [0, 4, 4, 4, 4, 5, 5],
            [new(1), new(2), new(3), new(4), new(5)]);
        int[] expected = [2, 1, 5, 4, 3];
        Hierarchy hierarchy = Hierarchy.Build(level, 1, true, default, (depth, candidates) =>
        {
            if (depth == 0 && !expected.SequenceEqual(candidates.Select(edge => edge.B)))
            {
                throw new InvalidDataException("Regularized scores differ from their defined descending order.");
            }
        });
        if (hierarchy.RegularizedAreas != 2 || hierarchy.Levels[0].Areas[0] != 1e-6f ||
            hierarchy.Levels[0].Areas[1] != 1e-6f || areas[0] != 0 || areas[1] != 0)
        {
            throw new InvalidDataException("Ambiguous links must regularize only the hierarchy copy of the zero areas.");
        }

        var unambiguous = new HierarchyLevel(points[..3], [Vector3.UnitX, Vector3.UnitX, Vector3.UnitX],
            [0, 1, 2], [0, 2, 3, 3], [new(1), new(2), new(2)]);
        hierarchy = Hierarchy.Build(unambiguous, 1, true, default, (depth, candidates) =>
        {
            if (depth == 0 && !candidates.Select(edge => edge.B).SequenceEqual([1, 2, 2]))
            {
                throw new InvalidDataException("Equal positive infinities must retain endpoint order.");
            }
        });
        if (hierarchy.RegularizedAreas != 0)
        {
            throw new InvalidDataException("A consistently ordered infinity must not change geometric areas.");
        }
    }

    private static void Expect<T>(Action run) where T : Exception
    {
        try
        {
            run();
        }
        catch (T)
        {
            return;
        }
        throw new InvalidDataException($"Expected {typeof(T).Name}.");
    }

    private sealed class CancelAt(RemeshStage stage, CancellationTokenSource token, int delayMilliseconds = 0) : IProgress<RemeshStage>
    {
        public RemeshStage LastStage { get; private set; }

        public void Report(RemeshStage value)
        {
            LastStage = value;
            if (stage != value)
            {
                return;
            }

            if (delayMilliseconds > 0)
            {
                token.CancelAfter(delayMilliseconds);
            }
            else
            {
                token.Cancel();
            }
        }
    }
}
