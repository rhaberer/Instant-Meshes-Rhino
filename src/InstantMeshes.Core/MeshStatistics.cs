// C# adaptation of Instant Meshes meshstats.cpp and the InstantMeshesPort driver. See LICENSE-instant-meshes.
using System.Numerics;

namespace InstantMeshes.Core;

internal readonly record struct MeshStatistics(Vector3 Minimum, Vector3 Maximum, Vector3 Center,
    double SurfaceArea, double AverageEdgeLength, double MaximumEdgeLength)
{
    public static MeshStatistics Measure(Vector3[] vertices, int[] triangles, CancellationToken token)
    {
        if (vertices.Length == 0 || triangles.Length == 0 || triangles.Length % 3 != 0)
        {
            throw new ArgumentException("A mesh requires vertices and complete triangles.");
        }

        MeshStatistics totals = Reduce(0, triangles.Length / 3);
        if (!(totals.SurfaceArea > 0) || !double.IsFinite(totals.SurfaceArea))
        {
            throw new ArgumentException("The mesh must have finite, positive surface area.");
        }

        return totals with
        {
            Center = totals.Center / (float)totals.SurfaceArea,
            AverageEdgeLength = totals.AverageEdgeLength / triangles.Length
        };

        MeshStatistics Reduce(int first, int end)
        {
            token.ThrowIfCancellationRequested();
            int count = end - first;
            // Reproduce the native serial shim's blocked reduction order.
            if (count > 1024)
            {
                int middle = first + count / 2;
                MeshStatistics left = Reduce(first, middle), right = Reduce(middle, end);
                return new(Vector3.Min(left.Minimum, right.Minimum), Vector3.Max(left.Maximum, right.Maximum),
                    left.Center + right.Center, left.SurfaceArea + right.SurfaceArea,
                    left.AverageEdgeLength + right.AverageEdgeLength,
                    Math.Max(left.MaximumEdgeLength, right.MaximumEdgeLength));
            }

            Vector3 minimum = new(float.PositiveInfinity), maximum = new(float.NegativeInfinity), center = default;
            double area = 0, totalEdgeLength = 0, longestEdge = 0;
            for (int face = first; face < end; face++)
            {
                int index = face * 3;
                Vector3 a = GetVertex(index), b = GetVertex(index + 1), c = GetVertex(index + 2);
                AccumulateEdge(a, b);
                AccumulateEdge(b, c);
                AccumulateEdge(c, a);
                float faceArea = 0.5f * Preprocessing.Norm(Vector3.Cross(b - a, c - a));
                area += faceArea;
                center += ((a + b + c) * (1f / 3f)) * faceArea;
            }
            return new(minimum, maximum, center, area, totalEdgeLength, longestEdge);

            Vector3 GetVertex(int index)
            {
                int vertex = triangles[index];
                if ((uint)vertex >= vertices.Length)
                {
                    throw new ArgumentException($"Triangle {index / 3} references an invalid vertex.");
                }

                return vertices[vertex];
            }

            void AccumulateEdge(Vector3 from, Vector3 to)
            {
                float edge = Preprocessing.Norm(from - to);
                totalEdgeLength += edge;
                longestEdge = Math.Max(longestEdge, edge);
                minimum = Vector3.Min(minimum, from);
                maximum = Vector3.Max(maximum, from);
            }
        }
    }

    public float? SubdivisionLimit(float scale) =>
        MaximumEdgeLength * 2 > scale || MaximumEdgeLength > AverageEdgeLength * 2
            ? MathF.Min(scale / 2, (float)AverageEdgeLength * 2)
            : null;
}

internal readonly record struct TargetDensity(float Scale, int Faces, int Vertices)
{
    public static TargetDensity Resolve(RemeshOptions options, double surfaceArea, int vertexCount)
    {
        if (!Enum.IsDefined(options.Density) || !Enum.IsDefined(options.Faces))
        {
            throw new ArgumentException("Choose a supported density and face mode.", nameof(options));
        }

        double target = options.Target;
        if (options.Density != DensityMode.Auto && (!double.IsFinite(target) || target <= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(options), "An explicit density target must be positive.");
        }

        bool triangular = options.PositionSymmetry == 3;
        if (options.Density == DensityMode.EdgeLength)
        {
            float scale = (float)target;
            float faceArea = triangular ? MathF.Sqrt(3f) / 4f * scale * scale : scale * scale;
            if (!float.IsFinite(scale) || !float.IsFinite(faceArea) || faceArea <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(options), "Edge length exceeds the solver's numeric range.");
            }

            int estimated = checked((int)(surfaceArea / faceArea));
            return new(scale, estimated, triangular ? estimated / 2 : estimated);
        }

        // Convert face or vertex targets to the area of an ideal output face.
        bool faceTarget = options.Density == DensityMode.Faces;
        if (options.Density != DensityMode.Auto && (target != Math.Truncate(target) || target > int.MaxValue))
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Face and vertex targets must be whole numbers in range.");
        }

        // A tiny input still needs a nonzero scale.
        int count = options.Density == DensityMode.Auto ? Math.Max(vertexCount / 16, 1) : (int)target;
        int faces = faceTarget || !triangular ? count : checked(count * 2);
        float area = (float)(surfaceArea / faces);
        float derivedScale = triangular ? 2 * MathF.Sqrt(area * MathF.Sqrt(1f / 3f)) : MathF.Sqrt(area);
        if (!(derivedScale > 0f) || !float.IsFinite(derivedScale))
        {
            throw new ArgumentException("Surface area and target density do not produce a usable edge length.");
        }

        return new(derivedScale, faces, faceTarget && triangular ? faces / 2 : count);
    }
}
