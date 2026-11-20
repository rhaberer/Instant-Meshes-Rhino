// Smoothing adapted from Instant Meshes extract.cpp. Ray tests follow bvh.cpp. See LICENSE-instant-meshes.
using System.Numerics;
using System.Runtime.CompilerServices;
using static InstantMeshes.Core.Preprocessing;

namespace InstantMeshes.Core;

internal static class SurfaceProjection
{
    public static (Vector3[], Vector3[]) Smooth(PreparedMesh? mesh, Vector3[] positions,
        Vector3[] normals, List<int[]> faces, HashSet<int> creases, float scale,
        RemeshOptions options, CancellationToken token)
    {
        if (options.SmoothingPasses == 0)
        {
            return (positions, normals);
        }

        var adjacent = new SortedSet<int>[positions.Length];
        for (int i = 0; i < adjacent.Length; i++)
        {
            adjacent[i] = [];
        }

        for (int i = 0; i < faces.Count; i++)
        {
            CheckCancellation(i, token);
            int[] face = faces[i];
            bool irregular = face.Length == 4 && face[2] == face[3];
            int edges = irregular ? 1 : face.Length;
            for (int j = 0; j < edges; j++)
            {
                int a = face[j], b = face[(j + 1) % face.Length];
                if (a == b)
                {
                    continue;
                }

                adjacent[a].Add(b);
                adjacent[b].Add(a);
            }
        }
        int[][] neighbors = Array.ConvertAll(adjacent, set => set.ToArray());
        TriangleProjection? surface = mesh is null ? null : new(mesh, token);
        // Double buffers keep each smoothing pass independent of its traversal order.
        var nextPositions = new Vector3[positions.Length];
        var nextNormals = new Vector3[normals.Length];
        for (int pass = 0; pass < options.SmoothingPasses; pass++)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                CheckCancellation(i, token);
                if (neighbors[i].Length == 0 || creases.Contains(i))
                {
                    nextPositions[i] = positions[i];
                    nextNormals[i] = normals[i];
                    continue;
                }

                (Vector3 p, Vector3 n) = Fit(positions, normals, neighbors[i], normals[i], options.StableTopology);
                if (surface is not null)
                {
                    float forward = surface.Intersect(p, n, scale / 2);
                    float backward = surface.Intersect(p, -n, scale / 2);
                    if (MathF.Min(forward, backward) < scale * 0.5f)
                    {
                        p = forward < backward ? p + n * forward : p - n * backward;
                    }
                }
                nextPositions[i] = Canonical(p, options.StableTopology);
                nextNormals[i] = Canonical(n, options.StableTopology);
            }
            (positions, nextPositions) = (nextPositions, positions);
            (normals, nextNormals) = (nextNormals, normals);
        }
        return (positions, normals);
    }

    public static Vector3 FitNormal(Vector3[] positions, Vector3[] normals, int[] indices, bool stable) =>
        Canonical(Fit(positions, normals, indices, Vector3.Zero, stable).Normal, stable);

    private static (Vector3 Center, Vector3 Normal) Fit(Vector3[] positions, Vector3[] normals,
        int[] neighbors, Vector3 ownNormal, bool stable)
    {
        Vector3 center = default, average = default;
        foreach (int i in neighbors)
        {
            center += positions[i];
            average += normals[i];
        }
        center = Canonical(center, stable) / neighbors.Length;
        average = Canonical(average + ownNormal, stable);
        // The smallest covariance direction is the fitted plane normal.
        float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        foreach (int i in neighbors)
        {
            Vector3 delta = positions[i] - center;
            xx += delta.X * delta.X; xy += delta.X * delta.Y; xz += delta.X * delta.Z;
            yy += delta.Y * delta.Y; yz += delta.Y * delta.Z; zz += delta.Z * delta.Z;
        }
        Vector3 normal = ProjectionSvd.SmallestDirection(
            Canonical(xx, stable), Canonical(xy, stable), Canonical(xz, stable),
            Canonical(yy, stable), Canonical(yz, stable), Canonical(zz, stable));
        normal = Canonical(normal, stable);
        return (center, normal * MathF.CopySign(1, Dot(average, normal)));
    }
}

internal sealed class TriangleProjection
{
    private readonly record struct Triangle(Vector3 Point, Vector3 A, Vector3 B,
        Vector3 Minimum, Vector3 Maximum, Vector3 Center, int Index);
    private readonly record struct Node(Vector3 Minimum, Vector3 Maximum, int First, int End, int Right);

    private sealed class CenterOrder(int axis) : IComparer<Triangle>
    {
        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public int Compare(Triangle a, Triangle b)
        {
            int order = a.Center[axis].CompareTo(b.Center[axis]);
            return order == 0 ? a.Index.CompareTo(b.Index) : order;
        }
    }

    private static readonly CenterOrder[] CenterOrders = [new(0), new(1), new(2)];
    private readonly Triangle[] triangles;
    private readonly List<Node> nodes = [];

    public TriangleProjection(PreparedMesh mesh, CancellationToken token)
    {
        triangles = new Triangle[mesh.Triangles.Length / 3];
        for (int i = 0; i < triangles.Length; i++)
        {
            CheckCancellation(i, token);
            Vector3 a = mesh.Positions[mesh.Triangles[3 * i]],
                b = mesh.Positions[mesh.Triangles[3 * i + 1]],
                c = mesh.Positions[mesh.Triangles[3 * i + 2]];
            triangles[i] = new(a, b - a, c - a, Vector3.Min(a, Vector3.Min(b, c)),
                Vector3.Max(a, Vector3.Max(b, c)), (a + b + c) * (1f / 3f), i);
        }
        Build(0, triangles.Length, token);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private int Build(int first, int end, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        Vector3 minimum = new(float.PositiveInfinity), maximum = new(float.NegativeInfinity);
        for (int i = first; i < end; i++)
        {
            minimum = Vector3.Min(minimum, triangles[i].Minimum);
            maximum = Vector3.Max(maximum, triangles[i].Maximum);
        }
        int index = nodes.Count;
        nodes.Add(new(minimum, maximum, first, end, -1));
        if (end - first > 12)
        {
            int axis = SpatialAxis.Longest(maximum - minimum);
            int split = first + (end - first) / 2;
            PartitionAtMedian(first, end, split, CenterOrders[axis]);
            Build(first, split, token);
            int right = Build(split, end, token);
            nodes[index] = nodes[index] with { Right = right };
        }
        return index;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void PartitionAtMedian(int first, int end, int split, CenterOrder order)
    {
        // A child only needs the same set of triangles as the sorted half.
        // Fully sort small nodes so terminal leaves keep their original traversal order.
        int count = end - first;
        if (count <= 32)
        {
            Array.Sort(triangles, first, count, order);
            return;
        }

        long budget = 2L * count;
        int lo = first, hi = end - 1;
        while (lo < hi)
        {
            budget -= hi - lo + 1;
            if (budget < 0)
            {
                // Bound work on adversarial center orderings.
                Array.Sort(triangles, lo, hi - lo + 1, order);
                return;
            }
            Triangle pivot = triangles[lo + (hi - lo) / 2];
            int i = lo, j = hi;
            while (i <= j)
            {
                while (order.Compare(triangles[i], pivot) < 0)
                {
                    i++;
                }

                while (order.Compare(triangles[j], pivot) > 0)
                {
                    j--;
                }

                if (i <= j)
                {
                    (triangles[i], triangles[j]) = (triangles[j], triangles[i]);
                    i++;
                    j--;
                }
            }
            if (split <= j)
            {
                hi = j;
            }
            else if (split >= i)
            {
                lo = i;
            }
            else
            {
                return;
            }
        }
    }

    public float Intersect(Vector3 origin, Vector3 direction, float maximum)
    {
        float closest = maximum;
        bool hit = false;
        Find(0);
        return hit ? closest : float.PositiveInfinity;

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        void Find(int index)
        {
            Node node = nodes[index];
            float near = 0, far = closest;
            for (int axis = 0; axis < 3; axis++)
            {
                float component = direction[axis];
                if (component == 0)
                {
                    if (origin[axis] < node.Minimum[axis] || origin[axis] > node.Maximum[axis])
                    {
                        return;
                    }

                    continue;
                }
                float inverse = 1f / component;
                float a = (node.Minimum[axis] - origin[axis]) * inverse;
                float b = (node.Maximum[axis] - origin[axis]) * inverse;
                near = MathF.Max(near, MathF.Min(a, b));
                far = MathF.Min(far, MathF.Max(a, b));
                if (near > far)
                {
                    return;
                }
            }
            if (node.Right >= 0)
            {
                Find(index + 1);
                Find(node.Right);
                return;
            }
            for (int i = node.First; i < node.End; i++)
            {
                Triangle triangle = triangles[i];
                Vector3 p = Vector3.Cross(direction, triangle.B);
                float determinant = Dot(triangle.A, p);
                if (determinant == 0)
                {
                    continue;
                }

                float inverse = 1f / determinant;
                Vector3 t = origin - triangle.Point;
                float u = Dot(t, p) * inverse;
                if (u < 0 || u > 1)
                {
                    continue;
                }

                Vector3 q = Vector3.Cross(t, triangle.A);
                float v = Dot(direction, q) * inverse;
                if (v < 0 || u + v > 1)
                {
                    continue;
                }

                float distance = Dot(triangle.B, q) * inverse;
                if (distance >= 0 && distance <= closest)
                {
                    closest = distance;
                    hit = true;
                }
            }
        }
    }
}

internal static class SpatialAxis
{
    public static int Longest(Vector3 extent) =>
        extent.X >= extent.Y && extent.X >= extent.Z ? 0 : extent.Y >= extent.Z ? 1 : 2;
}
