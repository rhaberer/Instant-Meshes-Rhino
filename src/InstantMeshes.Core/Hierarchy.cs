// C# adaptation of Instant Meshes hierarchy.cpp and InstantMeshesPort patches. See LICENSE-instant-meshes.
using System.Numerics;
using static InstantMeshes.Core.Preprocessing;

namespace InstantMeshes.Core;

internal readonly record struct Link(int Vertex, float Weight = 1);
internal readonly record struct EdgeCandidate(int A, int B, int Order, float Score,
    Int128 Numerator, long Denominator);

internal sealed class HierarchyLevel(Vector3[] positions, Vector3[] normals, float[] areas,
    int[] offsets, Link[] links)
{
    public Vector3[] Positions { get; } = positions;
    public Vector3[] Normals { get; } = normals;
    public float[] Areas { get; } = areas;
    public int[] Offsets { get; } = offsets;
    public Link[] Links { get; } = links;
    public int[][] Phases { get; set; } = [];
    public int[] Parents { get; set; } = [];
    public int[] Children { get; set; } = [];
    public Vector3[] Orientations { get; } = new Vector3[positions.Length];
    public Vector3[] Origins { get; } = new Vector3[positions.Length];

    // Compressed rows preserve the order in which neighboring links were assembled.
    public ReadOnlySpan<Link> Neighbors(int vertex) =>
        Links.AsSpan(Offsets[vertex], Offsets[vertex + 1] - Offsets[vertex]);
}

internal sealed class Hierarchy
{
    private const int MaxDepth = 25;
    public List<HierarchyLevel> Levels { get; } = [];
    public float Scale { get; }
    public bool Stable { get; }
    public int RegularizedAreas { get; private set; }

    private Hierarchy(float scale, bool stable) => (Scale, Stable) = (scale, stable);

    public static Hierarchy Build(PreparedMesh mesh, float scale, bool stable, CancellationToken token,
        Action<int, EdgeCandidate[]>? observeCandidates = null)
    {
        var links = new Link[mesh.Neighbors.Length];
        for (int index = 0; index < links.Length; index++)
        {
            links[index] = new Link(mesh.Neighbors[index]);
        }

        return Build(new HierarchyLevel(mesh.Positions, mesh.Normals, mesh.Areas, mesh.NeighborOffsets, links),
            scale, stable, token, observeCandidates);
    }

    public static Hierarchy Build(HierarchyLevel level, float scale, bool stable, CancellationToken token,
        Action<int, EdgeCandidate[]>? observeCandidates = null)
    {
        var hierarchy = new Hierarchy(scale, stable);
        for (int depth = 0; ; depth++)
        {
            if (stable && HasAmbiguousAreaScores(level, token))
            {
                level = hierarchy.RegularizeAreas(level, token);
            }

            level.Phases = Color(level, token);
            Initialize(level, scale, token);
            hierarchy.Levels.Add(level);
            if (depth == MaxDepth || (depth > 0 && level.Positions.Length == 1))
            {
                return hierarchy;
            }

            level = Collapse(level, stable, token, depth, observeCandidates);
        }
    }

    private static bool HasAmbiguousAreaScores(HierarchyLevel level, CancellationToken token)
    {
        // A zero denominator consistently sorts as an infinity when all such
        // scores have the same nonzero sign. Zero numerators and mixed signs
        // make the native cross-product comparator lose its strict ordering.
        int firstSign = 0;
        for (int i = 0; i < level.Positions.Length; i++)
        {
            CheckCancellation(i, token);
            foreach (Link link in level.Neighbors(i))
            {
                int j = link.Vertex;
                if (level.Areas[i] > 0 && level.Areas[j] > 0)
                {
                    continue;
                }

                if (level.Areas[i] == 0 && level.Areas[j] == 0)
                {
                    return true;
                }

                int sign = Int128.Sign(FixedNormalDot(level.Normals[i], level.Normals[j]));
                if (sign == 0 || (firstSign != 0 && sign != firstSign))
                {
                    return true;
                }

                firstSign = sign;
            }
        }
        return false;
    }

    private HierarchyLevel RegularizeAreas(HierarchyLevel level, CancellationToken token)
    {
        float[] areas = (float[])level.Areas.Clone();
        for (int i = 0; i < level.Positions.Length; i++)
        {
            CheckCancellation(i, token);
            if (level.Offsets[i] == level.Offsets[i + 1])
            {
                continue;
            }

            SetMinimum(i);
            foreach (Link link in level.Neighbors(i))
            {
                SetMinimum(link.Vertex);
            }
        }
        return new(level.Positions, level.Normals, areas, level.Offsets, level.Links);

        void SetMinimum(int vertex)
        {
            if (areas[vertex] > 0)
            {
                return;
            }

            areas[vertex] = 1e-6f;
            RegularizedAreas++;
        }
    }

    private static Int128 FixedNormalDot(Vector3 a, Vector3 b) =>
        (Int128)FixedUnits(a.X) * FixedUnits(b.X) +
        (Int128)FixedUnits(a.Y) * FixedUnits(b.Y) +
        (Int128)FixedUnits(a.Z) * FixedUnits(b.Z);

    private static HierarchyLevel Collapse(HierarchyLevel level, bool stable, CancellationToken token,
        int depth, Action<int, EdgeCandidate[]>? observeCandidates)
    {
        int count = level.Positions.Length;
        var candidates = new EdgeCandidate[level.Links.Length];
        for (int i = 0; i < count; i++)
        {
            CheckCancellation(i, token);
            for (int e = level.Offsets[i]; e < level.Offsets[i + 1]; e++)
            {
                int neighbor = level.Links[e].Vertex;
                float area = level.Areas[i], neighborArea = level.Areas[neighbor];
                Vector3 normal = level.Normals[i], neighborNormal = level.Normals[neighbor];
                // Eigen's dynamic column dot groups the first two products.
                float score = ((normal.X * neighborNormal.X + normal.Y * neighborNormal.Y) +
                    normal.Z * neighborNormal.Z) * (area > neighborArea ? area / neighborArea : neighborArea / area);
                if (stable)
                {
                    long areaA = FixedUnits(area), areaB = FixedUnits(neighborArea);
                    Int128 dot = FixedNormalDot(normal, neighborNormal);
                    candidates[e] = new(i, neighbor, e, RoundStorage(score),
                        checked(dot * Math.Max(areaA, areaB)), Math.Min(areaA, areaB));
                }
                else
                {
                    candidates[e] = new(i, neighbor, e, score, 0, 1);
                }
            }
        }

        // Greedy matching below accepts at most one edge per fine vertex.
        // The comparator also fixes the order of equal scores.
        Array.Sort(candidates, stable ? CompareStable : CompareOrdinary);
        token.ThrowIfCancellationRequested();
        observeCandidates?.Invoke(depth, candidates);

        int[] parents = new int[count];
        Array.Fill(parents, -1);
        int merged = 0;
        foreach (EdgeCandidate candidate in candidates)
        {
            if (parents[candidate.A] >= 0 || parents[candidate.B] >= 0)
            {
                continue;
            }

            parents[candidate.A] = parents[candidate.B] = merged;
            candidates[merged++] = candidate;
        }

        int coarseCount = count - merged;
        var positions = new Vector3[coarseCount];
        var normals = new Vector3[coarseCount];
        var areas = new float[coarseCount];
        var children = new int[coarseCount * 2];
        for (int i = 0; i < merged; i++)
        {
            CheckCancellation(i, token);
            EdgeCandidate edge = candidates[i];
            float a = level.Areas[edge.A], b = level.Areas[edge.B], sum = a + b;
            Vector3 p = level.Positions[edge.A], q = level.Positions[edge.B];
            Store(i, edge.A, edge.B, sum > ReciprocalOverflow ? (p * a + q * b) / sum : (p + q) * 0.5f,
                Normalize(level.Normals[edge.A] * a + level.Normals[edge.B] * b, false), sum);
        }

        int next = merged;
        for (int i = 0; i < count; i++)
        {
            CheckCancellation(i, token);
            if (parents[i] >= 0)
            {
                continue;
            }

            parents[i] = next;
            Store(next++, i, -1, level.Positions[i], level.Normals[i], level.Areas[i]);
        }
        // Each coarse vertex owns two child slots; -1 denotes an unmatched vertex.
        level.Parents = parents;
        level.Children = children;

        var offsets = new int[coarseCount + 1];
        var links = new List<Link>();
        // Accumulate in child/link order, then emit the touched parents in vertex order.
        var destinations = new List<int>();
        var seen = new int[coarseCount];
        var weights = new float[coarseCount];
        for (int i = 0; i < coarseCount; i++)
        {
            CheckCancellation(i, token);
            destinations.Clear();
            Collect(children[2 * i], i);
            int second = children[2 * i + 1];
            if (second >= 0)
            {
                Collect(second, i);
            }

            destinations.Sort();
            foreach (int neighbor in destinations)
            {
                links.Add(new Link(neighbor, weights[neighbor]));
            }

            offsets[i + 1] = links.Count;
        }

        return new(positions, normals, areas, offsets, links.ToArray());

        void Store(int index, int first, int second, Vector3 position, Vector3 normal, float area)
        {
            positions[index] = stable ? RoundStorage(position) : position;
            normals[index] = stable ? RoundStorage(normal) : normal;
            areas[index] = stable ? RoundStorage(area) : area;
            children[2 * index] = first;
            children[2 * index + 1] = second;
        }

        void Collect(int child, int source)
        {
            foreach (Link link in level.Neighbors(child))
            {
                int destination = parents[link.Vertex];
                if (source == destination)
                {
                    continue;
                }

                if (seen[destination] != source + 1)
                {
                    seen[destination] = source + 1;
                    weights[destination] = 0;
                    destinations.Add(destination);
                }
                weights[destination] += link.Weight;
            }
        }
    }

    private static int CompareOrdinary(EdgeCandidate a, EdgeCandidate b)
    {
        int score = b.Score.CompareTo(a.Score);
        return score == 0 ? a.Order.CompareTo(b.Order) : score;
    }

    private static int CompareStable(EdgeCandidate a, EdgeCandidate b)
    {
        int score = checked(b.Numerator * a.Denominator).CompareTo(checked(a.Numerator * b.Denominator));
        if (score != 0)
        {
            return score;
        }

        int first = a.A.CompareTo(b.A);
        return first == 0 ? a.B.CompareTo(b.B) : first;
    }

    private static int[][] Color(HierarchyLevel level, CancellationToken token)
    {
        int count = level.Positions.Length;
        var permutation = new int[count];
        var colors = new int[count];
        Array.Fill(colors, -1);
        for (int i = 0; i < count; i++)
        {
            permutation[i] = i;
        }

        var rng = new Pcg32();
        for (int i = count - 1; i > 0; i--)
        {
            int j = rng.NextIndex(i + 1);
            (permutation[i], permutation[j]) = (permutation[j], permutation[i]);
        }

        var used = new List<int>();
        for (int i = 0; i < count; i++)
        {
            CheckCancellation(i, token);
            int vertex = permutation[i];
            foreach (Link link in level.Neighbors(vertex))
            {
                int color = colors[link.Vertex];
                if (color >= 0)
                {
                    used[color] = i;
                }
            }
            int available = 0;
            while (available < used.Count && used[available] == i)
            {
                available++;
            }

            if (available == used.Count)
            {
                used.Add(i);
            }

            colors[vertex] = available;
        }

        var phases = new List<int>[used.Count];
        for (int i = 0; i < phases.Length; i++)
        {
            phases[i] = [];
        }

        for (int i = 0; i < colors.Length; i++)
        {
            phases[colors[i]].Add(i);
        }

        return Array.ConvertAll(phases, phase => phase.ToArray());
    }

    internal static (Vector3 S, Vector3 T) TangentFrame(Vector3 n)
    {
        Vector3 t;
        if (MathF.Abs(n.X) > MathF.Abs(n.Y))
        {
            float inverse = 1f / MathF.Sqrt(n.X * n.X + n.Z * n.Z);
            t = new(n.Z * inverse, 0, -n.X * inverse);
        }
        else
        {
            float inverse = 1f / MathF.Sqrt(n.Y * n.Y + n.Z * n.Z);
            t = new(0, n.Z * inverse, -n.Y * inverse);
        }
        return (Vector3.Cross(t, n), t);
    }

    private static void Initialize(HierarchyLevel level, float scale, CancellationToken token)
    {
        var orientationRandom = new Pcg32();
        var positionRandom = new Pcg32();
        for (int i = 0; i < level.Positions.Length; i++)
        {
            CheckCancellation(i, token);
            (Vector3 s, Vector3 t) = TangentFrame(level.Normals[i]);
            float angle = (float)(orientationRandom.NextFloat() * 2 * Math.PI);
            level.Orientations[i] = s * MathF.Cos(angle) + t * MathF.Sin(angle);
            float x = positionRandom.NextFloat() * 2f - 1f;
            float y = positionRandom.NextFloat() * 2f - 1f;
            level.Origins[i] = level.Positions[i] + (s * x + t * y) * scale;
        }
    }
}
