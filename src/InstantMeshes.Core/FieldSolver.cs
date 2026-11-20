// C# adaptation of Instant Meshes field.cpp, hierarchy.cpp and the port's synchronous optimizer. See LICENSE-instant-meshes.
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static InstantMeshes.Core.FieldGeometry;
using static InstantMeshes.Core.Preprocessing;

namespace InstantMeshes.Core;

internal enum FieldKind
{
    Orientation,
    Position
}

internal static class FieldSolver
{
    private const int Iterations = 6;
    private readonly record struct Constraint(Vector3 Direction, Vector3 Origin, bool Active);
    private readonly record struct PositionBasis(Vector3 Direction, Vector3 Bitangent, Vector3 CanonicalBitangent);

    public static (TimeSpan Orientation, TimeSpan Position) Solve(Hierarchy hierarchy, PreparedMesh? input, RemeshOptions options,
        CancellationToken token, Action<FieldKind, int, int, Vector3[]>? observe = null)
    {
        if (options.OrientationSymmetry is not (2 or 4 or 6))
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Orientation symmetry must be 2, 4 or 6.");
        }

        var timer = Stopwatch.StartNew();
        List<HierarchyLevel> levels = hierarchy.Levels;
        bool triangular = options.PositionSymmetry == 3;
        float scale = hierarchy.Scale, inverse = 1f / scale;
        Constraint[][]? constraints = input is not null && options.AlignToBoundaries
            ? BuildConstraints(levels, input, options.OrientationSymmetry, scale, inverse,
                triangular, hierarchy.Stable, token)
            : null;

        // Solve coarse to fine, prolonging the result before visiting the next level.
        for (int depth = levels.Count - 1; depth >= 0; depth--)
        {
            HierarchyLevel level = levels[depth];
            Constraint[]? local = constraints?[depth];
            for (int iteration = 0; iteration < Iterations; iteration++)
            {
                IterateOrientations(level, local, options, hierarchy.Stable, token);
                observe?.Invoke(FieldKind.Orientation, depth, iteration, level.Orientations);
            }
            if (depth > 0)
            {
                Prolong(level.Orientations, levels[depth - 1], false, hierarchy.Stable, token);
            }
        }

        // The position solve uses the base orientation solution restricted back to all coarser levels.
        for (int depth = 1; depth < levels.Count; depth++)
        {
            HierarchyLevel coarse = levels[depth], fine = levels[depth - 1];
            for (int i = 0; i < coarse.Positions.Length; i++)
            {
                CheckCancellation(i, token);
                int a = fine.Children[2 * i], b = fine.Children[2 * i + 1];
                Vector3 q = fine.Orientations[a], n = coarse.Normals[i];
                if (b >= 0)
                {
                    (Vector3 first, Vector3 second) = Orient(q, fine.Normals[a],
                        fine.Orientations[b], fine.Normals[b], options.OrientationSymmetry, true);
                    q = first + second;
                }
                q -= n * Dot(n, q);
                coarse.Orientations[i] = Dot(q, q) > ReciprocalOverflow ? Unit(q) : q;
            }
        }

        TimeSpan orientationTime = timer.Elapsed;
        timer.Restart();
        // Directions and bitangents stay fixed during the position solve.
        // Reuse one buffer across levels while the lattice origins evolve.
        var basis = new PositionBasis[levels[0].Orientations.Length];
        bool extrinsic = options.Extrinsic;
        Vector3[] midpoints = extrinsic ? new Vector3[levels.Max(level => level.Links.Length)] : [];
        for (int depth = levels.Count - 1; depth >= 0; depth--)
        {
            HierarchyLevel level = levels[depth];
            Constraint[]? local = constraints?[depth];
            for (int i = 0; i < level.Orientations.Length; i++)
            {
                CheckCancellation(i, token);
                Vector3 direction = Unit(level.Orientations[i]);
                Vector3 bitangent = extrinsic ? Bitangent(direction, level.Normals[i], triangular) : default;
                basis[i] = new PositionBasis(direction, bitangent, Canonical(bitangent, extrinsic && hierarchy.Stable));
            }

            if (extrinsic)
            {
                FillMidpoints(level, midpoints, !triangular && hierarchy.Stable, token);
            }

            for (int iteration = 0; iteration < Iterations; iteration++)
            {
                IteratePositions(level, local, basis, midpoints, scale, inverse, triangular, extrinsic,
                    hierarchy.Stable, token);
                observe?.Invoke(FieldKind.Position, depth, iteration, level.Origins);
            }
            if (depth > 0)
            {
                Prolong(level.Origins, levels[depth - 1], true, hierarchy.Stable, token);
            }
        }
        return (orientationTime, timer.Elapsed);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static void IterateOrientations(HierarchyLevel level, Constraint[]? constraints,
        RemeshOptions options, bool stable, CancellationToken token)
    {
        // Colors group nonadjacent vertices; phases and neighbor updates remain ordered.
        foreach (int[] phase in level.Phases)
        {
            for (int index = 0; index < phase.Length; index++)
            {
                CheckCancellation(index, token);
                int i = phase[index];
                Vector3 n = level.Normals[i], sum = level.Orientations[i];
                float weight = 0;
                foreach (Link link in level.Neighbors(i))
                {
                    if (link.Weight == 0)
                    {
                        continue;
                    }

                    int j = link.Vertex;
                    (Vector3 a, Vector3 b) = Orient(sum, n, level.Orientations[j], level.Normals[j],
                        options.OrientationSymmetry, options.Extrinsic);
                    sum = NormalizeDirection(a * weight + b * link.Weight, n, stable);
                    weight += link.Weight;
                }

                if (constraints is not null && constraints[i].Active)
                {
                    (_, Vector3 b) = Orient(sum, n, constraints[i].Direction, n,
                        options.OrientationSymmetry, options.Extrinsic);
                    sum = NormalizeDirection(b, n, stable);
                }
                if (weight > 0)
                {
                    level.Orientations[i] = Canonical(sum, stable);
                }
            }
        }
        if (stable)
        {
            RoundField(level.Orientations, token);
        }
    }

    private static Vector3 NormalizeDirection(Vector3 sum, Vector3 n, bool stable)
    {
        sum = Canonical(sum - n * Dot(n, sum), stable);
        float norm = Norm(sum);
        return norm > ReciprocalOverflow ? Canonical(sum / norm, stable) : sum;
    }

    private static void FillMidpoints(HierarchyLevel level, Vector3[] midpoints, bool stable,
        CancellationToken token)
    {
        // Links retain their order. Only the endpoints and normals determine these midpoints.
        for (int vertex = 0; vertex < level.Positions.Length; vertex++)
        {
            CheckCancellation(vertex, token);
            Vector3 position = level.Positions[vertex], normal = level.Normals[vertex];
            for (int index = level.Offsets[vertex]; index < level.Offsets[vertex + 1]; index++)
            {
                Link link = level.Links[index];
                if (link.Weight == 0)
                {
                    continue;
                }

                int neighbor = link.Vertex;
                midpoints[index] = Canonical(
                    Midpoint(position, normal, level.Positions[neighbor], level.Normals[neighbor]), stable);
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static void IteratePositions(HierarchyLevel level, Constraint[]? constraints, PositionBasis[] basis,
        Vector3[] midpoints, float scale,
        float inverse, bool triangular, bool extrinsic, bool stable, CancellationToken token)
    {
        foreach (int[] phase in level.Phases)
        {
            for (int index = 0; index < phase.Length; index++)
            {
                CheckCancellation(index, token);
                int i = phase[index];
                Vector3 n = level.Normals[i], p = level.Positions[i], sum = level.Origins[i];
                PositionBasis currentBasis = basis[i];
                Vector3 q = currentBasis.Direction;
                float weight = 0;
                for (int linkIndex = level.Offsets[i]; linkIndex < level.Offsets[i + 1]; linkIndex++)
                {
                    Link link = level.Links[linkIndex];
                    if (link.Weight == 0)
                    {
                        continue;
                    }

                    int j = link.Vertex;
                    var a = new FieldSample(p, n, q, sum);
                    PositionBasis neighborBasis = basis[j];
                    var b = new FieldSample(level.Positions[j], level.Normals[j], neighborBasis.Direction,
                        level.Origins[j]);
                    (Vector3 first, Vector3 second) = extrinsic
                        ? MatchExtrinsicPositions(a, b, currentBasis.Bitangent, neighborBasis.Bitangent,
                            currentBasis.CanonicalBitangent, neighborBasis.CanonicalBitangent, midpoints[linkIndex],
                            scale, inverse, triangular)
                        : MatchIntrinsicPositions(a, b, scale, inverse, triangular);
                    sum = first * weight + second * link.Weight;
                    weight += link.Weight;
                    if (weight > ReciprocalOverflow)
                    {
                        sum /= weight;
                    }

                    sum -= n * Dot(n, sum - p);
                }

                if (constraints is not null && constraints[i].Active)
                {
                    Constraint constraint = constraints[i];
                    Vector3 d = constraint.Origin - sum;
                    d -= constraint.Direction * Dot(constraint.Direction, d);
                    sum += d;
                    sum -= n * Dot(n, sum - p);
                }
                if (weight > 0)
                {
                    level.Origins[i] = Canonical(RoundToLattice(new(sum, n, q, sum), p, scale, inverse, triangular), stable);
                }
            }
        }
        if (stable)
        {
            RoundField(level.Origins, token);
        }
    }

    private static void Prolong(Vector3[] source, HierarchyLevel fine, bool positions, bool stable,
        CancellationToken token)
    {
        Vector3[] result = positions ? fine.Origins : fine.Orientations;
        for (int i = 0; i < result.Length; i++)
        {
            CheckCancellation(i, token);
            Vector3 n = fine.Normals[i], value = source[fine.Parents[i]];
            Vector3 delta = positions ? value - fine.Positions[i] : value;
            value -= n * Dot(n, delta);
            result[i] = stable ? RoundStorage(value) : value;
        }
    }

    private static void RoundField(Vector3[] field, CancellationToken token)
    {
        for (int i = 0; i < field.Length; i++)
        {
            CheckCancellation(i, token);
            field[i] = RoundStorage(field[i]);
        }
    }

    private static Constraint[][] BuildConstraints(List<HierarchyLevel> levels, PreparedMesh mesh,
        int symmetry, float scale, float inverse, bool triangular, bool stable, CancellationToken token)
    {
        var all = new Constraint[levels.Count][];
        var basis = new Constraint[mesh.Positions.Length];
        all[0] = basis;
        for (int edge = 0; edge < mesh.OppositeEdges.Length; edge++)
        {
            CheckCancellation(edge, token);
            if (mesh.OppositeEdges[edge] != MissingEdge)
            {
                continue;
            }

            int a = mesh.Triangles[edge], b = mesh.Triangles[Next(edge)];
            Vector3 d = mesh.Positions[b] - mesh.Positions[a];
            if (Dot(d, d) <= 0)
            {
                continue;
            }

            Vector3 q = Unit(d);
            basis[a] = new(q, mesh.Positions[a], true);
            basis[b] = new(q, mesh.Positions[b], true);
        }

        for (int depth = 1; depth < levels.Count; depth++)
        {
            HierarchyLevel fine = levels[depth - 1], coarse = levels[depth];
            Constraint[] previous = all[depth - 1], next = new Constraint[coarse.Positions.Length];
            all[depth] = next;
            for (int i = 0; i < next.Length; i++)
            {
                CheckCancellation(i, token);
                int a = fine.Children[2 * i], b = fine.Children[2 * i + 1];
                Constraint first = previous[a], second = b >= 0 ? previous[b] : default;
                if (!first.Active && !second.Active)
                {
                    continue;
                }

                Constraint merged = first.Active ? first : second;
                if (first.Active && second.Active)
                {
                    (Vector3 qa, Vector3 qb) = Orient(first.Direction, fine.Normals[a],
                        second.Direction, fine.Normals[b], symmetry, true);
                    (Vector3 oa, Vector3 ob) = MatchExtrinsicPositions(
                        new(fine.Positions[a], fine.Normals[a], first.Direction, first.Origin),
                        new(fine.Positions[b], fine.Normals[b], second.Direction, second.Origin),
                        scale, inverse, triangular, stable);
                    merged = new(qa + qb, (oa + ob) / 2f, true);
                }

                Vector3 n = coarse.Normals[i], q = merged.Direction, o = merged.Origin;
                if (q != Vector3.Zero)
                {
                    q -= n * Dot(n, q);
                    if (Dot(q, q) > ReciprocalOverflow)
                    {
                        q = Unit(q);
                    }
                }
                // The pinned implementation projects the origin using the constraint direction here.
                if (o != Vector3.Zero)
                {
                    o -= n * Dot(n, q - coarse.Positions[i]);
                }

                next[i] = new(q, o, true);
            }
        }
        return all;
    }
}
