// Adapted to C# from Instant Field-Aligned Meshes (Jakob, Tarini, Panozzo,
// Sorkine-Hornung) and the InstantMeshesPort canonical arithmetic patches.
// See LICENSE-instant-meshes. Reference files: dedge.cpp, meshstats.cpp, normal.cpp, adjacency.cpp.

using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;

namespace InstantMeshes.Core;

internal sealed record PreparedMesh(
    Vector3[] Positions, int[] Triangles, int[] VertexEdges, int[] OppositeEdges,
    bool[] Boundary, bool[] NonManifold, Vector3[] Normals, float[] Areas,
    int[] NeighborOffsets, int[] Neighbors, HashSet<int>? Creases = null);

internal sealed record DirectedEdges(int[] VertexEdges, int[] OppositeEdges, bool[] Boundary, bool[] NonManifold);

internal static class Preprocessing
{
    internal const int MissingEdge = -1;
    internal const float ReciprocalOverflow = 2.93873587705571876e-39f;

    public static PreparedMesh Prepare(
        ReadOnlySpan<Vector3> positions, ReadOnlySpan<int> triangles, bool stable,
        CancellationToken cancellationToken = default, float? subdivisionLimit = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (positions.IsEmpty || triangles.IsEmpty || triangles.Length % 3 != 0)
        {
            throw new ArgumentException("A mesh requires vertices and complete triangles.");
        }

        Vector3[] vertices = CopyPositions(positions, stable, cancellationToken);
        return PrepareOwned(vertices, triangles.ToArray(), stable, cancellationToken, subdivisionLimit);
    }

    internal static Vector3[] CopyPositions(ReadOnlySpan<Vector3> positions, bool stable, CancellationToken token)
    {
        Vector3[] vertices = positions.ToArray();
        for (int i = 0; i < vertices.Length; i++)
        {
            CheckCancellation(i, token);
            Vector3 position = vertices[i];
            if (!float.IsFinite(position.X) || !float.IsFinite(position.Y) || !float.IsFinite(position.Z))
            {
                throw new ArgumentException($"Vertex {i} has non-finite coordinates.", nameof(positions));
            }

            vertices[i] = Canonical(position, stable);
        }
        return vertices;
    }

    internal static PreparedMesh PrepareOwned(Vector3[] vertices, int[] faces, bool stable,
        CancellationToken cancellationToken, float? subdivisionLimit, float? creaseAngle = null)
    {
        // Subdivision replaces the owned arrays, so rebuild directed edges afterwards.
        DirectedEdges topology = BuildEdges(vertices.Length, faces, cancellationToken);
        if (subdivisionLimit is float limit)
        {
            (vertices, faces) = Subdivision.Split(vertices, faces, topology, limit, cancellationToken);
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = Canonical(vertices[i], stable);
            }

            topology = BuildEdges(vertices.Length, faces, cancellationToken);
        }
        (int[] vertexEdges, int[] opposite, bool[] boundary, bool[] nonManifold) = topology;

        Vector3[] faceNormals = new Vector3[faces.Length / 3];
        for (int edge = 0; edge < faces.Length; edge += 3)
        {
            CheckCancellation(edge, cancellationToken);
            Vector3 origin = vertices[faces[edge]];
            Vector3 normal = Vector3.Cross(vertices[faces[edge + 1]] - origin, vertices[faces[edge + 2]] - origin);
            faceNormals[edge / 3] = Normalize(normal, stable && creaseAngle is null);
        }

        var creases = creaseAngle.HasValue ? new HashSet<int>() : null;
        float threshold = creaseAngle is float angleThreshold
            ? (float)Math.Cos(angleThreshold * Math.PI / 180)
            : -2;

        Vector3[] normals = new Vector3[vertices.Length];
        float[] areas = new float[vertices.Length];
        // Neighbor rows are contiguous: offsets[i] through offsets[i + 1] delimit vertex i.
        int[] offsets = new int[vertices.Length + 1];
        var neighbors = new List<int>(faces.Length);
        for (int i = 0; i < vertices.Length; i++)
        {
            CheckCancellation(i, cancellationToken);
            Vector3 normal = Vector3.Zero;
            float area = 0;
            int start = vertexEdges[i];
            int edge = start;
            if (edge != MissingEdge)
            {
                neighbors.Add(faces[Previous(edge)]);
                int visited = 0;
                do
                {
                    Vector3 origin = vertices[i];
                    Vector3 previous = vertices[faces[Previous(edge)]];
                    Vector3 next = vertices[faces[Next(edge)]];
                    Vector3 d0 = next - origin;
                    Vector3 d1 = previous - origin;
                    float angle = FastAcos(Dot(d0, d1) / MathF.Sqrt(Dot(d0, d0) * Dot(d1, d1)));
                    if (creases is null && float.IsFinite(angle))
                    {
                        normal = Canonical(normal + faceNormals[edge / 3] * Canonical(angle, stable), stable);
                    }

                    // Sum the two barycentric wedges incident to this vertex.
                    Vector3 center = Canonical((origin + previous + next) * (1f / 3f), stable);
                    Vector3 midpointPrevious = Canonical((origin + previous) * 0.5f, stable);
                    Vector3 midpointNext = Canonical((origin + next) * 0.5f, stable);
                    float contribution = 0.5f * (
                        Norm(Vector3.Cross(origin - midpointPrevious, origin - center)) +
                        Norm(Vector3.Cross(origin - midpointNext, origin - center)));
                    area = Canonical(area + Canonical(contribution, stable), stable);

                    int reverse = opposite[edge];
                    if (reverse == MissingEdge || Next(reverse) != start)
                    {
                        neighbors.Add(faces[Next(edge)]);
                    }

                    if (reverse == MissingEdge)
                    {
                        break;
                    }

                    edge = Next(reverse);
                    if (++visited > faces.Length || faces[edge] != i)
                    {
                        throw new InvalidOperationException("Inconsistent directed edge ring.");
                    }
                } while (edge != start);
            }
            if (creases is not null && start != MissingEdge)
            {
                normal = CreaseNormal(i, start);
            }

            normals[i] = Canonical(Normalize(normal, stable && creases is null), stable);
            areas[i] = Canonical(area, stable);
            offsets[i + 1] = neighbors.Count;
        }

        return new PreparedMesh(vertices, faces, vertexEdges, opposite, boundary, nonManifold,
            normals, areas, offsets, neighbors.ToArray(), creases);

        Vector3 CreaseNormal(int vertex, int start)
        {
            int edge = start;
            if (!boundary[vertex])
            {
                int lastCrease = -1;
                do
                {
                    int next = Next(opposite[edge]);
                    if (Dot(faceNormals[edge / 3], faceNormals[next / 3]) < threshold)
                    {
                        lastCrease = Math.Max(lastCrease, next);
                    }

                    edge = next;
                } while (edge != start);
                if (lastCrease >= 0)
                {
                    start = lastCrease;
                }
            }

            Vector3 normal = default;
            edge = start;
            do
            {
                Vector3 d0 = vertices[faces[Next(edge)]] - vertices[vertex];
                Vector3 d1 = vertices[faces[Previous(edge)]] - vertices[vertex];
                float angle = FastAcos(Dot(d0, d1) / MathF.Sqrt(Dot(d0, d0) * Dot(d1, d1)));
                if (float.IsFinite(angle))
                {
                    normal += faceNormals[edge / 3] * angle;
                }

                if (opposite[edge] == MissingEdge)
                {
                    break;
                }

                int next = Next(opposite[edge]);
                if (Dot(faceNormals[edge / 3], faceNormals[next / 3]) < threshold)
                {
                    creases!.Add(vertex);
                    break;
                }
                edge = next;
            } while (edge != start);
            return normal;
        }
    }

    private static DirectedEdges BuildEdges(int vertexCount, int[] faces, CancellationToken cancellationToken)
    {
        // Entries stay in face order, including duplicate edges from inconsistent winding.
        int[] vertexEdges = Missing(vertexCount);
        int[] lastEdges = Missing(vertexCount);
        int[] nextOutgoing = Missing(faces.Length);
        for (int edge = 0; edge < faces.Length; edge++)
        {
            CheckCancellation(edge, cancellationToken);
            int from = faces[edge];
            int to = faces[Next(edge)];
            if ((uint)from >= vertexCount || (uint)to >= vertexCount)
            {
                throw new ArgumentException($"Triangle {edge / 3} references an invalid vertex.", nameof(faces));
            }

            if (from == to)
            {
                continue;
            }

            if (lastEdges[from] == MissingEdge)
            {
                vertexEdges[from] = edge;
            }
            else
            {
                nextOutgoing[lastEdges[from]] = edge;
            }

            lastEdges[from] = edge;
        }

        int[] opposite = Missing(faces.Length);
        bool[] nonManifold = new bool[vertexCount];
        for (int edge = 0; edge < faces.Length; edge++)
        {
            CheckCancellation(edge, cancellationToken);
            int from = faces[edge];
            int to = faces[Next(edge)];
            if (from == to)
            {
                continue;
            }

            int reverse = MissingEdge;
            for (int other = vertexEdges[to]; other != MissingEdge; other = nextOutgoing[other])
            {
                if (faces[Next(other)] != from)
                {
                    continue;
                }

                if (reverse != MissingEdge)
                {
                    nonManifold[from] = nonManifold[to] = true;
                    reverse = MissingEdge;
                    break;
                }
                reverse = other;
            }
            if (reverse != MissingEdge && edge < reverse)
            {
                opposite[edge] = reverse;
                opposite[reverse] = edge;
            }
        }

        bool[] boundary = new bool[vertexCount];
        for (int i = 0; i < vertexCount; i++)
        {
            CheckCancellation(i, cancellationToken);
            int start = vertexEdges[i];
            if (nonManifold[i] || start == MissingEdge)
            {
                vertexEdges[i] = MissingEdge;
                continue;
            }
            int edge = start;
            int lowest = start;
            int visited = 0;
            do
            {
                lowest = Math.Min(lowest, edge);
                int previous = opposite[Previous(edge)];
                if (previous == MissingEdge)
                {
                    boundary[i] = true;
                    lowest = edge;
                    break;
                }
                edge = previous;
                if (++visited > faces.Length || faces[edge] != i)
                {
                    throw new InvalidOperationException("Inconsistent directed edge ring.");
                }
            } while (edge != start);
            vertexEdges[i] = lowest;
        }

        return new DirectedEdges(vertexEdges, opposite, boundary, nonManifold);
    }

    internal static int Next(int edge) => edge % 3 == 2 ? edge - 2 : edge + 1;
    internal static int Previous(int edge) => edge % 3 == 0 ? edge + 2 : edge - 1;

    internal static float Canonical(float value, bool stable)
    {
        if (!stable)
        {
            return value;
        }

        return (float)(FixedUnits(value) / 1_000_000.0);
    }

    internal static long FixedUnits(float value) =>
        checked((long)Math.Round(value * 1_000_000.0, MidpointRounding.AwayFromZero));

    // The port rounds hierarchy and field storage in single precision.
    internal static float RoundStorage(float value) =>
        MathF.Round(value * 1_000_000f, MidpointRounding.AwayFromZero) / 1_000_000f;

    internal static Vector3 RoundStorage(Vector3 value) =>
        new(RoundStorage(value.X), RoundStorage(value.Y), RoundStorage(value.Z));

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    internal static Vector3 Canonical(Vector3 value, bool stable)
    {
        if (!stable)
        {
            return value;
        }

        if (AdvSimd.Arm64.IsSupported)
        {
            Vector128<float> lanes = value.AsVector128();
            // The SIMD conversion saturates. Leave boundary and nonfinite inputs
            // to FixedUnits so the checked scalar overflow behavior is preserved.
            const float limit = (float)(9223372036854775808.0 / 1_000_000.0);
            if (Vector128.LessThanAll(Vector128.Abs(lanes), Vector128.Create(limit)))
            {
                return Vector128.Narrow(Quantize(Vector128.WidenLower(lanes)),
                    Quantize(Vector128.WidenUpper(lanes))).AsVector3();
            }
        }
        return new(Canonical(value.X, true), Canonical(value.Y, true), Canonical(value.Z, true));

        static Vector128<double> Quantize(Vector128<double> pair)
        {
            var scale = Vector128.Create(1_000_000.0);
            var units = AdvSimd.Arm64.ConvertToInt64RoundAwayFromZero(pair * scale);
            return Vector128.ConvertToDouble(units) / scale;
        }
    }

    // Match Eigen's scalar three-element redux grouping.
    internal static float Dot(Vector3 a, Vector3 b) => a.X * b.X + (a.Y * b.Y + a.Z * b.Z);
    internal static float Norm(Vector3 value) => MathF.Sqrt(Dot(value, value));

    internal static Vector3 Normalize(Vector3 value, bool stable)
    {
        float length = Norm(value);
        return length > ReciprocalOverflow ? Canonical(value / length, stable) : Vector3.UnitX;
    }

    private static float FastAcos(float value)
    {
        float negate = value < 0f ? 1f : 0f;
        float x = MathF.Abs(value);
        float result = -0.0187293f;
        result = result * x + 0.0742610f;
        result = result * x - 0.2121144f;
        result = result * x + 1.5707288f;
        result *= MathF.Sqrt(1f - x);
        result -= 2f * negate * result;
        return negate * MathF.PI + result;
    }

    private static int[] Missing(int count)
    {
        int[] edges = new int[count];
        Array.Fill(edges, MissingEdge);
        return edges;
    }

    internal static void CheckCancellation(int index, CancellationToken token)
    {
        if ((index & 1023) == 0)
        {
            token.ThrowIfCancellationRequested();
        }
    }
}
