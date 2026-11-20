// C# adaptation of Instant Meshes subdivide.cpp. See LICENSE-instant-meshes.
using System.Numerics;

namespace InstantMeshes.Core;

internal static class Subdivision
{
    public static (Vector3[] Positions, int[] Triangles) Split(
        Vector3[] positions, int[] triangles, DirectedEdges topology, float maximumLength,
        CancellationToken cancellationToken)
    {
        float maximumSquared = maximumLength * maximumLength;
        if (!float.IsFinite(maximumSquared) || maximumSquared <= 0f)
        {
            throw new ArgumentOutOfRangeException(nameof(maximumLength));
        }

        var vertices = new List<Vector3>(positions);
        var faces = new List<int>(triangles);
        var opposites = new List<int>(topology.OppositeEdges);
        var queue = new EdgeQueue();
        const int missing = Preprocessing.MissingEdge;

        for (int edge = 0; edge < triangles.Length; edge++)
        {
            if ((edge & 1023) == 0)
            {
                cancellationToken.ThrowIfCancellationRequested();
            }

            int a = triangles[edge];
            int b = triangles[Preprocessing.Next(edge)];
            if (topology.NonManifold[a] || topology.NonManifold[b])
            {
                continue;
            }

            int opposite = opposites[edge];
            if (opposite == missing || opposite > edge)
            {
                Schedule(edge);
            }
        }

        while (queue.TryPop(out Edge item))
        {
            cancellationToken.ThrowIfCancellationRequested();
            int e0 = item.Index;
            // A previous split can leave stale queue entries for an altered edge.
            if (LengthSquared(e0) != item.SquaredLength)
            {
                continue;
            }

            int e1 = opposites[e0];
            bool boundary = e1 == missing;
            int f0 = e0 - e0 % 3;
            int f1 = boundary ? missing : e1 - e1 % 3;
            int v0 = faces[e0];
            int v1 = faces[Preprocessing.Next(e0)];
            int v0Previous = faces[Preprocessing.Previous(e0)];
            int v1Previous = boundary ? missing : faces[Preprocessing.Previous(e1)];
            int vn = vertices.Count;
            Vector3 midpoint = (vertices[v0] + vertices[v1]) * 0.5f;
            if (midpoint == vertices[v0] || midpoint == vertices[v1])
            {
                throw new InvalidOperationException("Input precision is too low for the requested edge length.");
            }

            vertices.Add(midpoint);

            int f2 = boundary ? missing : AppendFace(vn, v1Previous, v1);
            int f3 = AppendFace(vn, v1, v0Previous);
            SetFace(f0, vn, v0Previous, v0);
            if (!boundary)
            {
                SetFace(f1, vn, v0, v1Previous);
            }

            // Reconnect the new triangles before scheduling their edges.
            int e0Previous = opposites[Preprocessing.Previous(e0)];
            int e0Next = opposites[Preprocessing.Next(e0)];
            Connect(f0, f3 + 2);
            Connect(f0 + 1, e0Previous);
            Connect(f3 + 1, e0Next);
            if (boundary)
            {
                Connect(f0 + 2, missing);
                Connect(f3, missing);
            }
            else
            {
                int e1Previous = opposites[Preprocessing.Previous(e1)];
                int e1Next = opposites[Preprocessing.Next(e1)];
                Connect(f0 + 2, f1);
                Connect(f1 + 1, e1Next);
                Connect(f1 + 2, f2);
                Connect(f2 + 1, e1Previous);
                Connect(f2 + 2, f3);
            }

            ScheduleFace(f0);
            if (!boundary)
            {
                ScheduleFace(f2);
                ScheduleFace(f1);
            }
            ScheduleFace(f3);
        }

        return (vertices.ToArray(), faces.ToArray());

        float LengthSquared(int edge)
        {
            Vector3 delta = vertices[faces[edge]] - vertices[faces[Preprocessing.Next(edge)]];
            // The reference reduces dynamic Eigen columns in component order.
            return (delta.X * delta.X + delta.Y * delta.Y) + delta.Z * delta.Z;
        }

        int AppendFace(int a, int b, int c)
        {
            int index = faces.Count;
            faces.Add(a);
            faces.Add(b);
            faces.Add(c);
            opposites.Add(missing);
            opposites.Add(missing);
            opposites.Add(missing);
            return index;
        }

        void SetFace(int face, int a, int b, int c)
        {
            faces[face] = a;
            faces[face + 1] = b;
            faces[face + 2] = c;
        }

        void Connect(int a, int b)
        {
            opposites[a] = b;
            if (b != missing)
            {
                opposites[b] = a;
            }
        }

        void Schedule(int edge)
        {
            float length = LengthSquared(edge);
            if (length > maximumSquared)
            {
                queue.Push(new Edge(edge, length));
            }
        }

        void ScheduleFace(int face)
        {
            Schedule(face);
            Schedule(face + 1);
            Schedule(face + 2);
        }
    }

    private readonly record struct Edge(int Index, float SquaredLength);

    // A binary max heap with left-child tie handling and leaf-first removal matches
    // the pinned native reference on the development Mac. Equal lengths are common.
    private sealed class EdgeQueue
    {
        private readonly List<Edge> items = new();

        public void Push(Edge item)
        {
            items.Add(item);
            PlaceFromLeaf(items.Count - 1, item);
        }

        public bool TryPop(out Edge result)
        {
            result = default;
            if (items.Count == 0)
            {
                return false;
            }

            result = items[0];
            int last = items.Count - 1;
            int hole = 0;
            while (hole <= (last - 1) / 2 && last > 0)
            {
                int child = 2 * hole + 1;
                if (child < last && items[child].SquaredLength < items[child + 1].SquaredLength)
                {
                    child++;
                }

                items[hole] = items[child];
                hole = child;
            }
            if (hole != last)
            {
                PlaceFromLeaf(hole, items[last]);
            }

            items.RemoveAt(last);
            return true;
        }

        private void PlaceFromLeaf(int hole, Edge item)
        {
            while (hole > 0)
            {
                int parent = (hole - 1) / 2;
                if (items[parent].SquaredLength >= item.SquaredLength)
                {
                    break;
                }

                items[hole] = items[parent];
                hole = parent;
            }
            items[hole] = item;
        }
    }
}
