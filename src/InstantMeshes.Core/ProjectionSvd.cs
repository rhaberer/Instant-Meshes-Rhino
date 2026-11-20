// Modified scalar 3-by-3 C# adaptation of Eigen's JacobiSVD.h, RealSvd2x2.h and Jacobi.h.
// Copyright (C) 2009-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2009, 2013-2016 Gael Guennebaud <gael.guennebaud@inria.fr>
// SPDX-License-Identifier: MPL-2.0
// See LICENSE-eigen.
using System.Numerics;

namespace InstantMeshes.Core;

internal static class ProjectionSvd
{
    private const float MinimumNormal = 1.17549435e-38f;
    private const float TwiceEpsilon = 2.38418579e-7f;

    public static Vector3 SmallestDirection(float xx, float xy, float xz, float yy, float yz, float zz)
    {
        // Row-major symmetric covariance and accumulated left rotations.
        Span<float> a = stackalloc float[9] { xx, xy, xz, xy, yy, yz, xz, yz, zz };
        Span<float> left = stackalloc float[9] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        float scale = 0;
        foreach (float value in a)
        {
            scale = MathF.Max(scale, MathF.Abs(value));
        }

        if (scale == 0)
        {
            return Vector3.UnitZ;
        }

        for (int i = 0; i < a.Length; i++)
        {
            a[i] /= scale;
        }

        float maximum = MathF.Max(MathF.Abs(a[0]), MathF.Max(MathF.Abs(a[4]), MathF.Abs(a[8])));
        bool changed;
        do
        {
            changed = false;
            for (int p = 1; p < 3; p++)
            {
                for (int q = 0; q < p; q++)
                {
                    float threshold = MathF.Max(MinimumNormal, TwiceEpsilon * maximum);
                    if (!(MathF.Abs(a[p * 3 + q]) > threshold || MathF.Abs(a[q * 3 + p]) > threshold))
                    {
                        continue;
                    }

                    changed = true;
                    RotateBlock(a, left, p, q);
                    maximum = MathF.Max(maximum, MathF.Max(MathF.Abs(a[p * 4]), MathF.Abs(a[q * 4])));
                }
            }
        } while (changed);

        // Keep Eigen’s singular-value ordering and its tie behavior.
        Span<int> order = stackalloc int[3] { 0, 1, 2 };
        for (int i = 0; i < 2; i++)
        {
            int best = i;
            for (int j = i + 1; j < 3; j++)
            {
                if (MathF.Abs(a[order[j] * 4]) * scale > MathF.Abs(a[order[best] * 4]) * scale)
                {
                    best = j;
                }
            }

            if (a[order[best] * 4] * scale == 0)
            {
                break;
            }

            (order[i], order[best]) = (order[best], order[i]);
        }
        int smallest = order[2];
        Vector3 normal = new(left[smallest], left[3 + smallest], left[6 + smallest]);
        if (a[smallest * 4] < 0)
        {
            normal = -normal;
        }

        return FieldGeometry.Unit(normal);
    }

    private static void RotateBlock(Span<float> a, Span<float> left, int p, int q)
    {
        float pp = a[p * 4], pq = a[p * 3 + q], qp = a[q * 3 + p], qq = a[q * 4];
        float difference = qp - pq;
        float c0 = 1, s0 = 0;
        if (MathF.Abs(difference) >= MinimumNormal)
        {
            float ratio = (pp + qq) / difference;
            float length = MathF.Sqrt(1f + ratio * ratio);
            c0 = ratio / length;
            s0 = 1f / length;
        }

        float x = c0 * pp + s0 * qp;
        float y = c0 * pq + s0 * qq;
        float z = -s0 * pq + c0 * qq;
        float c1 = 1, s1 = 0;
        float denominator = 2 * MathF.Abs(y);
        if (denominator >= MinimumNormal)
        {
            float tau = (x - z) / denominator;
            float length = MathF.Sqrt(tau * tau + 1f);
            float tangent = 1f / (tau > 0 ? tau + length : tau - length);
            float sign = tangent > 0 ? 1f : -1f;
            c1 = 1f / MathF.Sqrt(tangent * tangent + 1f);
            s1 = -sign * (y / MathF.Abs(y)) * MathF.Abs(tangent) * c1;
        }

        float c = c0 * c1 - s0 * -s1;
        float s = c0 * -s1 + s0 * c1;
        for (int i = 0; i < 3; i++)
        {
            Apply(a, p * 3 + i, q * 3 + i, c, s);
            Apply(left, i * 3 + p, i * 3 + q, c, s);
        }
        for (int i = 0; i < 3; i++)
        {
            Apply(a, i * 3 + p, i * 3 + q, c1, -s1);
        }
    }

    private static void Apply(Span<float> a, int i, int j, float c, float s)
    {
        float x = a[i], y = a[j];
        a[i] = c * x + s * y;
        a[j] = -s * x + c * y;
    }
}
