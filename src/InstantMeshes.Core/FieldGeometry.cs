// C# adaptation of Instant Meshes field.cpp and InstantMeshesPort patches. See LICENSE-instant-meshes.
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using static InstantMeshes.Core.Preprocessing;

namespace InstantMeshes.Core;

internal readonly record struct LatticeIndex(int U, int V);
internal readonly record struct FieldSample(Vector3 Position, Vector3 Normal, Vector3 Direction, Vector3 Origin);

internal static class FieldGeometry
{
    private const float SqrtThreeOverTwo = 0.866025403784439f;
    private const long LatticeDenominator = 1_000_000_000_000_000_000L;

    public static Vector3 Rotate60(Vector3 d, Vector3 n) =>
        SqrtThreeOverTwo * Vector3.Cross(n, d) + 0.5f * (d + n * Dot(n, d));

    public static Vector3 Unit(Vector3 vector)
    {
        float squared = Dot(vector, vector);
        return squared > 0 ? vector / MathF.Sqrt(squared) : vector;
    }

    private static Vector3 Transport(Vector3 value, Vector3 axis, float cosine, float factor) =>
        value * cosine + Vector3.Cross(axis, value) + axis * (Dot(axis, value) * factor);

    public static (Vector3 A, Vector3 B) Orient(Vector3 a, Vector3 n, Vector3 b, Vector3 m,
        int symmetry, bool extrinsic)
    {
        if (!extrinsic)
        {
            float cosine = Dot(m, n);
            if (cosine < 0.9999f)
            {
                Vector3 axis = Vector3.Cross(m, n);
                b = b * cosine + Vector3.Cross(axis, b) +
                    axis * (Dot(axis, b) * (1f - cosine) / Dot(axis, axis));
            }
        }

        if (symmetry == 2)
        {
            return (a, b * MathF.CopySign(1, Dot(a, b)));
        }

        if (symmetry == 4 && extrinsic)
        {
            return OrientFour(a, n, b, m);
        }

        Span<Vector3> left = stackalloc Vector3[3];
        Span<Vector3> right = stackalloc Vector3[3];
        int rightCount = FillDirections(right, b, extrinsic ? m : n, symmetry);
        if (!extrinsic)
        {
            Vector3 selected = right[0];
            float best = Dot(selected, a);
            for (int i = 1; i < rightCount; i++)
            {
                float score = Dot(right[i], a);
                if (MathF.Abs(score) > MathF.Abs(best) ||
                    (symmetry == 4 && MathF.Abs(score) == MathF.Abs(best)))
                {
                    (selected, best) = (right[i], score);
                }
            }
            return (a, selected * MathF.CopySign(1, best));
        }

        int leftCount = FillDirections(left, a, n, symmetry);
        float bestScore = float.NegativeInfinity;
        int first = 0, second = 0;
        for (int i = 0; i < leftCount; i++)
        {
            for (int j = 0; j < rightCount; j++)
            {
                float score = MathF.Abs(Dot(left[i], right[j]));
                if (score > bestScore)
                {
                    (first, second, bestScore) = (i, j, score);
                }
            }
        }

        return (left[first], right[second] * MathF.CopySign(1, Dot(left[first], right[second])));
    }

    // Compile the repeatedly visited kernels at full optimization on first use.
    // Small interactive solves otherwise spend several runs in quick-JIT code.
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static (Vector3 A, Vector3 B) OrientFour(Vector3 a, Vector3 n, Vector3 b, Vector3 m)
    {
        Vector3 perpendicularA = Vector3.Cross(n, a), perpendicularB = Vector3.Cross(m, b);
        Vector3 left = a, right = b;
        float signed = Dot(a, b);
        float best = float.IsNaN(signed) ? float.NegativeInfinity : MathF.Abs(signed);

        // Scan in the same row-major order as the general symmetry search.
        float score = Dot(a, perpendicularB);
        if (MathF.Abs(score) > best)
        {
            (right, signed, best) = (perpendicularB, score, MathF.Abs(score));
        }

        score = Dot(perpendicularA, b);
        if (MathF.Abs(score) > best)
        {
            (left, right, signed, best) = (perpendicularA, b, score, MathF.Abs(score));
        }

        score = Dot(perpendicularA, perpendicularB);
        if (MathF.Abs(score) > best)
        {
            (left, right, signed) = (perpendicularA, perpendicularB, score);
        }

        return (left, right * MathF.CopySign(1, signed));
    }

    private static int FillDirections(Span<Vector3> result, Vector3 direction, Vector3 normal, int symmetry)
    {
        if (symmetry == 4)
        {
            result[0] = direction;
            result[1] = Vector3.Cross(normal, direction);
            return 2;
        }
        result[0] = Rotate60(direction, -normal);
        result[1] = direction;
        result[2] = Rotate60(direction, normal);
        return 3;
    }

    private static Vector3 Midpoint(FieldSample a, FieldSample b) =>
        Midpoint(a.Position, a.Normal, b.Position, b.Normal);

    internal static Vector3 Midpoint(Vector3 p, Vector3 n, Vector3 q, Vector3 m)
    {
        float np = Dot(n, p), nq = Dot(n, q), mp = Dot(m, p), mq = Dot(m, q);
        float cosine = Dot(n, m);
        float inverse = 1f / (1f - cosine * cosine + 1e-4f);
        float u = 2f * (nq - np - cosine * (mp - mq)) * inverse;
        float v = 2f * (mp - mq - cosine * (nq - np)) * inverse;
        return 0.5f * (p + q) - 0.25f * (n * u + m * v);
    }

    private static Vector3 Bitangent(FieldSample sample, bool triangular) =>
        Bitangent(sample.Direction, sample.Normal, triangular);

    internal static Vector3 Bitangent(Vector3 direction, Vector3 normal, bool triangular) =>
        triangular ? Rotate60(direction, normal) : Vector3.Cross(normal, direction);

    private static (float U, float V) Coordinates(Vector3 q, Vector3 t, Vector3 delta, float inverse,
        bool triangular)
    {
        float u = Dot(q, delta), v = Dot(t, delta);
        return triangular
            ? ((4 * u - 2 * v) * (1f / 3f) * inverse, (-2 * u + 4 * v) * (1f / 3f) * inverse)
            : (u * inverse, v * inverse);
    }

    public static Vector3 RoundToLattice(FieldSample sample, Vector3 point, float scale,
        float inverse, bool triangular)
    {
        Vector3 t = Bitangent(sample, triangular);
        (float u, float v) = Coordinates(sample.Direction, t, point - sample.Origin, inverse, triangular);
        if (!triangular)
        {
            return sample.Origin + sample.Direction * MathF.Round(u, MidpointRounding.AwayFromZero) * scale
                + t * MathF.Round(v, MidpointRounding.AwayFromZero) * scale;
        }

        u = MathF.Floor(u);
        v = MathF.Floor(v);
        Vector3 best = default;
        float bestCost = float.PositiveInfinity;
        for (int i = 0; i < 4; i++)
        {
            Vector3 candidate = At(sample.Origin, sample.Direction, t, u + (i & 1), v + (i >> 1), scale);
            Vector3 delta = candidate - point;
            float cost = Dot(delta, delta);
            if (cost < bestCost)
            {
                (best, bestCost) = (candidate, cost);
            }
        }
        return best;
    }

    private static Vector3 FloorToLattice(FieldSample sample, Vector3 t, Vector3 point, float scale,
        float inverse, bool triangular)
    {
        Vector3 q = sample.Direction;
        (float u, float v) = Coordinates(q, t, point - sample.Origin, inverse, triangular);
        u = MathF.Floor(u);
        v = MathF.Floor(v);
        return triangular ? At(sample.Origin, q, t, u, v, scale)
            : sample.Origin + q * u * scale + t * v * scale;
    }

    public static (Vector3 A, Vector3 B) MatchExtrinsicPositions(FieldSample a, FieldSample b, float scale,
        float inverse, bool triangular, bool stable)
    {
        Vector3 aBitangent = Bitangent(a, triangular);
        Vector3 bBitangent = Bitangent(b, triangular);
        Vector3 midpoint = Midpoint(a, b);
        if (!triangular)
        {
            midpoint = Canonical(midpoint, stable);
        }

        return MatchExtrinsicPositions(a, b, aBitangent, bBitangent, Canonical(aBitangent, stable),
            Canonical(bBitangent, stable), midpoint, scale, inverse, triangular);
    }

    internal static (Vector3 A, Vector3 B) MatchIntrinsicPositions(FieldSample a, FieldSample b, float scale,
        float inverse, bool triangular)
    {
        float cosine = Dot(b.Normal, a.Normal);
        if (cosine < 0.9999f)
        {
            Vector3 axis = Vector3.Cross(b.Normal, a.Normal), middle = Midpoint(a, b);
            float factor = (1f - cosine) / Dot(axis, axis);
            b = b with
            {
                Direction = Transport(b.Direction, axis, cosine, factor),
                Origin = Transport(b.Origin - middle, axis, cosine, factor) + middle
            };
        }
        return (a.Origin, RoundToLattice(b with { Normal = a.Normal }, a.Origin, scale, inverse, triangular));
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    internal static (Vector3 A, Vector3 B) MatchExtrinsicPositions(FieldSample a, FieldSample b,
        Vector3 aBitangent, Vector3 bBitangent, Vector3 aCanonicalBitangent, Vector3 bCanonicalBitangent,
        Vector3 midpoint, float scale, float inverse, bool triangular)
    {
        // The lattice cell uses the unrounded bitangent. Its corners use the canonical bitangent.
        Vector3 oa = FloorToLattice(a, aBitangent, midpoint, scale, inverse, triangular);
        Vector3 ob = FloorToLattice(b, bBitangent, midpoint, scale, inverse, triangular);
        Span<Vector3> ca = stackalloc Vector3[4];
        Span<Vector3> cb = stackalloc Vector3[4];
        FillCorners(ca, oa, a.Direction, aCanonicalBitangent, 0, 0, scale, false);
        FillCorners(cb, ob, b.Direction, bCanonicalBitangent, 0, 0, scale, false);

        return ClosestPositionCorners(ca, cb);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    internal static (Vector3 A, Vector3 B) ClosestPositionCorners(ReadOnlySpan<Vector3> a,
        ReadOnlySpan<Vector3> b)
    {
        if (!Vector128.IsHardwareAccelerated)
        {
            (int i, int j, _) = ClosestPair(a, b, false);
            return (a[i], b[j]);
        }

        // Each lane follows one A corner through the original sequence of B corners.
        Vector128<float> ax = Vector128.Create(a[0].X, a[1].X, a[2].X, a[3].X);
        Vector128<float> ay = Vector128.Create(a[0].Y, a[1].Y, a[2].Y, a[3].Y);
        Vector128<float> az = Vector128.Create(a[0].Z, a[1].Z, a[2].Z, a[3].Z);
        Vector128<float> minima = Vector128.Create(float.PositiveInfinity);
        Vector128<int> indices = Vector128<int>.Zero;
        for (int j = 0; j < b.Length; j++)
        {
            Vector128<float> dx = ax - Vector128.Create(b[j].X);
            Vector128<float> dy = ay - Vector128.Create(b[j].Y);
            Vector128<float> dz = az - Vector128.Create(b[j].Z);
            Vector128<float> costs = dx * dx + (dy * dy + dz * dz);
            Vector128<float> better = Vector128.LessThan(costs, minima);
            minima = Vector128.ConditionalSelect(better, costs, minima);
            indices = Vector128.ConditionalSelect(better.AsInt32(), Vector128.Create(j), indices);
        }

        // Strict comparisons retain the first corner on a tie, as in the scalar scan.
        int bestA = 0;
        float best = minima.GetElement(0);
        for (int i = 1; i < 4; i++)
        {
            float cost = minima.GetElement(i);
            if (cost < best)
            {
                (bestA, best) = (i, cost);
            }
        }
        int bestB = indices.GetElement(bestA);
        return (a[bestA], b[bestB]);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public static (LatticeIndex A, LatticeIndex B, float Error) MatchLatticeIndices(
        FieldSample a, FieldSample b, float scale, float inverse, bool triangular, bool stable)
    {
        Vector3 midpoint = Midpoint(a, b);
        Vector3 ta = Bitangent(a, triangular), tb = Bitangent(b, triangular);
        if (!triangular)
        {
            midpoint = Canonical(midpoint, stable);
            ta = Canonical(ta, stable);
            tb = Canonical(tb, stable);
        }

        LatticeIndex oa = FloorIndex(a, ta, midpoint, inverse, triangular, stable);
        LatticeIndex ob = FloorIndex(b, tb, midpoint, inverse, triangular, stable);
        Span<Vector3> ca = stackalloc Vector3[4];
        Span<Vector3> cb = stackalloc Vector3[4];
        FillCorners(ca, a.Origin, a.Direction, ta, oa.U, oa.V, scale, stable);
        FillCorners(cb, b.Origin, b.Direction, tb, ob.U, ob.V, scale, stable);
        (int first, int second, float cost) = ClosestPair(ca, cb, stable);
        return (new(oa.U + (first & 1), oa.V + (first >> 1)),
            new(ob.U + (second & 1), ob.V + (second >> 1)), cost);
    }

    private static LatticeIndex FloorIndex(FieldSample sample, Vector3 t, Vector3 point, float inverse,
        bool triangular, bool stable)
    {
        Vector3 delta = point - sample.Origin;
        if (!stable)
        {
            (float u, float v) = Coordinates(sample.Direction, t, delta, inverse, triangular);
            return new(checked((int)MathF.Floor(u)), checked((int)MathF.Floor(v)));
        }

        // Fixed units make the floor decision independent of float ties.
        delta = Canonical(delta, true);
        Int128 qd = FixedDot(sample.Direction, delta), td = FixedDot(Canonical(t, true), delta);
        long inv = FixedUnits(inverse);
        return triangular
            ? new(FloorDivide(checked((4 * qd - 2 * td) * inv), 3 * LatticeDenominator),
                FloorDivide(checked((-2 * qd + 4 * td) * inv), 3 * LatticeDenominator))
            : new(FloorDivide(checked(qd * inv), LatticeDenominator),
                FloorDivide(checked(td * inv), LatticeDenominator));
    }

    private static Int128 FixedDot(Vector3 a, Vector3 b) => checked(
        (Int128)FixedUnits(a.X) * FixedUnits(b.X) +
        (Int128)FixedUnits(a.Y) * FixedUnits(b.Y) +
        (Int128)FixedUnits(a.Z) * FixedUnits(b.Z));

    private static int FloorDivide(Int128 numerator, long denominator)
    {
        // Integer division truncates toward zero; lattice coordinates require floor.
        Int128 quotient = numerator / denominator;
        if (numerator < 0 && numerator % denominator != 0)
        {
            quotient--;
        }

        return checked((int)quotient);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static void FillCorners(Span<Vector3> corners, Vector3 origin, Vector3 q, Vector3 t,
        int u, int v, float scale, bool stable)
    {
        for (int i = 0; i < corners.Length; i++)
        {
            corners[i] = Canonical(At(origin, q, t, u + (i & 1), v + (i >> 1), scale), stable);
        }
    }

    private static Vector3 At(Vector3 origin, Vector3 q, Vector3 t, float u, float v, float scale) =>
        origin + (q * u + t * v) * scale;

    private static (int A, int B, float Cost) ClosestPair(ReadOnlySpan<Vector3> a, ReadOnlySpan<Vector3> b, bool stable)
    {
        int first = 0, second = 0;
        float best = float.PositiveInfinity;
        Int128 bestKey = 0;
        for (int i = 0; i < a.Length; i++)
        {
            for (int j = 0; j < b.Length; j++)
            {
                Vector3 delta = a[i] - b[j];
                float cost = Canonical(Dot(delta, delta), stable);
                if (cost > best || (!stable && cost == best))
                {
                    continue;
                }

                Int128 key = stable ? DistanceKey(a[i], b[j]) : 0;
                if (cost < best || (cost == best && key < bestKey))
                {
                    (first, second, best, bestKey) = (i, j, cost, key);
                }
            }
        }

        return (first, second, best);
    }

    private static Int128 DistanceKey(Vector3 a, Vector3 b)
    {
        Int128 x = (Int128)FixedUnits(a.X) - FixedUnits(b.X);
        Int128 y = (Int128)FixedUnits(a.Y) - FixedUnits(b.Y);
        Int128 z = (Int128)FixedUnits(a.Z) - FixedUnits(b.Z);
        return checked(x * x + y * y + z * z);
    }
}
