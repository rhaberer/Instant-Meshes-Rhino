// C# adaptation of the graph stages in Instant Meshes extract.cpp. See LICENSE-instant-meshes.
using System.Numerics;
using static InstantMeshes.Core.FieldGeometry;
using static InstantMeshes.Core.Preprocessing;

namespace InstantMeshes.Core;

internal sealed record ExtractedGraph(Vector3[] Positions, Vector3[] Normals,
    List<int>[] Adjacency, HashSet<int> Creases)
{
    private readonly record struct Collapse(int A, int B, float Cost, int Order);
    private readonly record struct Snap(int A, int B, int C, float Height, int Order);

    public static ExtractedGraph Build(Hierarchy hierarchy, PreparedMesh? mesh, RemeshOptions options,
        CancellationToken token)
    {
        HierarchyLevel level = hierarchy.Levels[0];
        int count = level.Positions.Length;
        var sets = new DisjointSets(count);
        var adjacency = new List<int>[count];
        var collapses = new List<Collapse>();
        var collapsed = new ushort[count];
        bool triangular = options.PositionSymmetry == 3, stable = hierarchy.Stable;
        float scale = hierarchy.Scale, inverse = 1f / scale;
        for (int i = 0; i < count; i++)
        {
            adjacency[i] = [];
        }

        // Equal lattice sites become collapse candidates; neighboring sites become edges.
        for (int i = 0; i < count; i++)
        {
            CheckCancellation(i, token);
            foreach (Link link in level.Neighbors(i))
            {
                int j = link.Vertex;
                if (j <= i)
                {
                    continue;
                }

                (Vector3 qa, Vector3 qb) = Orient(level.Orientations[i], level.Normals[i],
                    level.Orientations[j], level.Normals[j], options.OrientationSymmetry, true);
                (LatticeIndex a, LatticeIndex b, float cost) = MatchLatticeIndices(
                    new(level.Positions[i], level.Normals[i], qa, level.Origins[i]),
                    new(level.Positions[j], level.Normals[j], qb, level.Origins[j]),
                    scale, inverse, triangular, stable);
                long u = Math.Abs((long)a.U - b.U), v = Math.Abs((long)a.V - b.V);
                if (u > 1 || v > 1 || (u == 1 && v == 1 && !triangular))
                {
                    continue;
                }

                if (u + v == 0)
                {
                    collapses.Add(new(i, j, stable ? RoundStorage(cost) : cost, collapses.Count));
                }
                else
                {
                    adjacency[i].Add(j);
                    adjacency[j].Add(i);
                }
            }
        }

        collapses.Sort((a, b) =>
        {
            int cost = a.Cost.CompareTo(b.Cost);
            if (cost != 0)
            {
                return cost;
            }

            if (!stable)
            {
                return a.Order.CompareTo(b.Order);
            }

            int left = a.A.CompareTo(b.A);
            return left == 0 ? a.B.CompareTo(b.B) : left;
        });

        var connected = new List<int>();
        var seen = new int[count];
        for (int i = 0; i < collapses.Count; i++)
        {
            CheckCancellation(i, token);
            Collapse entry = collapses[i];
            int a = sets.Find(entry.A), b = sets.Find(entry.B);
            if (a == b)
            {
                continue;
            }

            if (a > b)
            {
                (a, b) = (b, a);
            }

            connected.Clear();
            CollectRoots(adjacency[a], i + 1);
            if (seen[b] == i + 1)
            {
                continue;
            }

            CollectRoots(adjacency[b], i + 1);
            connected.Sort();
            int root = sets.Unite(a, b);
            adjacency[a].Clear();
            adjacency[b].Clear();
            adjacency[root].AddRange(connected);
            collapsed[root] = unchecked((ushort)(collapsed[a] + collapsed[b] + 1));
        }

        // Compact surviving roots and translate their links into the new index space.
        var map = new int[count];
        Array.Fill(map, -1);
        var compact = new List<List<int>>();
        var compactCollapses = new List<ushort>();
        float averageCollapses = 0;
        for (int i = 0; i < count; i++)
        {
            if (adjacency[i].Count == 0)
            {
                continue;
            }

            map[i] = compact.Count;
            compact.Add(adjacency[i]);
            compactCollapses.Add(collapsed[i]);
            averageCollapses += collapsed[i];
        }
        averageCollapses /= compact.Count;
        Array.Clear(seen);
        for (int i = 0; i < compact.Count; i++)
        {
            connected.Clear();
            foreach (int j in compact[i])
            {
                int destination = map[sets.Find(j)];
                if (destination >= 0 && destination != i && seen[destination] != i + 1)
                {
                    seen[destination] = i + 1;
                    connected.Add(destination);
                }
            }
            connected.Sort();
            compact[i] = new(connected);
        }
        for (int i = 0; i < compact.Count; i++)
        {
            if (compactCollapses[i] > averageCollapses / 10)
            {
                continue;
            }

            foreach (int j in compact[i])
            {
                compact[j].RemoveAll(k => k == i);
            }

            compact[i].Clear();
        }

        var graph = new ExtractedGraph(new Vector3[compact.Count], new Vector3[compact.Count],
            compact.ToArray(), []);
        var weights = new float[compact.Count];
        for (int i = 0; i < count; i++)
        {
            CheckCancellation(i, token);
            int j = map[sets.Find(i)];
            if (j < 0)
            {
                continue;
            }

            Vector3 delta = level.Origins[i] - level.Positions[i];
            float squared = (delta.X * delta.X + delta.Y * delta.Y) + delta.Z * delta.Z;
            float weight = MathF.Exp(-squared * inverse * inverse * 9);
            graph.Positions[j] += level.Origins[i] * weight;
            graph.Normals[j] += level.Normals[i] * weight;
            weights[j] += weight;
            if (mesh is not null && (mesh.Creases?.Contains(i) == true ||
                (options.AlignToBoundaries && mesh.Boundary[i])))
            {
                graph.Creases.Add(j);
            }
        }
        for (int i = 0; i < weights.Length; i++)
        {
            if (weights[i] == 0)
            {
                continue;
            }

            graph.Positions[i] /= weights[i];
            Vector3 n = graph.Normals[i];
            float length = MathF.Sqrt((n.X * n.X + n.Y * n.Y) + n.Z * n.Z);
            graph.Normals[i] = length > 0 ? n / length : n;
        }

        graph.Clean(scale, triangular, token);
        // Face tracing expects each neighbor ring in angular order around its normal.
        for (int i = 0; i < graph.Adjacency.Length; i++)
        {
            CheckCancellation(i, token);
            Vector3 p = graph.Positions[i];
            (Vector3 s, Vector3 t) = Hierarchy.TangentFrame(graph.Normals[i]);
            var ordered = graph.Adjacency[i]
                .OrderByDescending(j => MathF.Atan2(Dot(t, graph.Positions[j] - p), Dot(s, graph.Positions[j] - p)))
                .ToArray();
            graph.Adjacency[i].Clear();
            graph.Adjacency[i].AddRange(ordered);
        }
        for (int i = 0; i < graph.Positions.Length && stable; i++)
        {
            graph.Positions[i] = Canonical(graph.Positions[i], true);
            graph.Normals[i] = Canonical(graph.Normals[i], true);
        }
        return graph;

        void CollectRoots(List<int> adjacent, int visit)
        {
            foreach (int vertex in adjacent)
            {
                int neighbor = sets.Find(vertex);
                if (seen[neighbor] == visit)
                {
                    continue;
                }

                seen[neighbor] = visit;
                connected.Add(neighbor);
            }
        }
    }

    private void Clean(float scale, bool triangular, CancellationToken token)
    {
        float threshold = 0.3f * scale;
        var snaps = new List<Snap>();
        var diagonals = new List<Collapse>();
        bool changed;
        do
        {
            changed = false;
            while (SnapVertices())
            {
                changed = true;
            }

            if (!triangular && RemoveDiagonals())
            {
                changed = true;
            }
        } while (changed);

        // Repair nearly collinear two-edge paths before removing quad diagonals.
        bool SnapVertices()
        {
            snaps.Clear();
            for (int i = 0; i < Adjacency.Length; i++)
            {
                CheckCancellation(i, token);
                foreach (int j in Adjacency[i])
                {
                    foreach (int k in Adjacency[j])
                    {
                        if (k == i)
                        {
                            continue;
                        }

                        (float a, float b, float c) = Lengths(i, j, k);
                        if (a > MathF.Max(b, c))
                        {
                            float height = Height(a, b, c);
                            if (height < threshold)
                            {
                                snaps.Add(new(i, j, k, height, snaps.Count));
                            }
                        }
                    }
                }
            }
            snaps.Sort((a, b) => a.Height == b.Height ? a.Order.CompareTo(b.Order) : a.Height.CompareTo(b.Height));
            bool snapped = false;
            foreach ((int i, int j, int k, float height, _) in snaps)
            {
                if (!Adjacency[i].Contains(j) || !Adjacency[j].Contains(k))
                {
                    continue;
                }

                (float a, float b, float c) = Lengths(i, j, k);
                if (Height(a, b, c) != height)
                {
                    continue;
                }

                if (b < threshold || c < threshold)
                {
                    Merge(i, b < threshold ? j : k);
                }
                else
                {
                    Positions[i] = (Positions[j] + Positions[k]) * 0.5f;
                    Normals[i] = Unit(Normals[j] + Normals[k]);
                    if (Creases.Contains(j) && Creases.Contains(k))
                    {
                        Creases.Add(i);
                    }

                    Disconnect(j, k);
                    if (!Adjacency[k].Contains(i))
                    {
                        Adjacency[i].Add(k);
                        Adjacency[k].Add(i);
                    }
                }
                snapped = true;
            }
            return snapped;
        }

        bool RemoveDiagonals()
        {
            diagonals.Clear();
            for (int i = 0; i < Adjacency.Length; i++)
            {
                CheckCancellation(i, token);
                foreach (int j in Adjacency[i])
                {
                    int count = 0;
                    float length = 0;
                    foreach (int k in Adjacency[i])
                    {
                        if (k == j || !Adjacency[j].Contains(k))
                        {
                            continue;
                        }

                        count++;
                        length += Norm(Positions[k] - Positions[i]) + Norm(Positions[k] - Positions[j]);
                    }
                    if (count != 2)
                    {
                        continue;
                    }

                    float expected = length / 4 * MathF.Sqrt(2f);
                    float actual = Norm(Positions[i] - Positions[j]);
                    float score = MathF.Abs((actual - expected) / MathF.Min(actual, expected));
                    diagonals.Add(new(i, j, score, diagonals.Count));
                }
            }
            diagonals.Sort((a, b) => a.Cost == b.Cost ? a.Order.CompareTo(b.Order) : a.Cost.CompareTo(b.Cost));
            bool removed = false;
            foreach ((int i, int j, _, _) in diagonals)
            {
                int count = 0;
                foreach (int k in Adjacency[i])
                {
                    if (Adjacency[j].Contains(k))
                    {
                        count++;
                    }
                }

                if (count == 2)
                {
                    Disconnect(i, j);
                    removed = true;
                }
            }
            return removed;
        }

        (float, float, float) Lengths(int i, int j, int k) =>
            (Norm(Positions[j] - Positions[k]), Norm(Positions[i] - Positions[j]), Norm(Positions[i] - Positions[k]));

        static float Height(float a, float b, float c)
        {
            float s = 0.5f * (a + b + c);
            return 2 * MathF.Sqrt(s * (s - a) * (s - b) * (s - c)) / a;
        }

        void Disconnect(int i, int j)
        {
            Adjacency[i].RemoveAll(k => k == j);
            Adjacency[j].RemoveAll(k => k == i);
        }

        void Merge(int vertex, int other)
        {
            Positions[vertex] = (Positions[vertex] + Positions[other]) * 0.5f;
            Normals[vertex] = (Normals[vertex] + Normals[other]) * 0.5f;
            var neighbors = new SortedSet<int>(Adjacency[vertex]);
            foreach (int k in Adjacency[other])
            {
                if (k == vertex)
                {
                    continue;
                }

                neighbors.Add(k);
                List<int> references = Adjacency[k];
                for (int l = 0; l < references.Count; l++)
                {
                    if (references[l] == other)
                    {
                        references[l] = vertex;
                    }
                }
            }
            neighbors.Remove(vertex);
            neighbors.Remove(other);
            Adjacency[other].Clear();
            Adjacency[vertex].Clear();
            Adjacency[vertex].AddRange(neighbors);
            if (Creases.Remove(other))
            {
                Creases.Add(vertex);
            }
        }
    }
}
