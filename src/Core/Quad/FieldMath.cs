// Ported from Instant Meshes (src/field.cpp, src/common.h).
// Copyright (c) 2015 Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung.
// BSD 3-Clause License — see THIRD_PARTY.md.
using System;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>Extrinsic 4-RoSy / 4-PoSy compatibility and lattice rounding (Jakob et al. 2015).</summary>
public static class FieldMath
{
    /// <summary>Two unit tangents s, t with (s, t, a) orthonormal. Port of coordinate_system.</summary>
    public static void CoordinateSystem(Vector3 a, out Vector3 b, out Vector3 c)
    {
        if (MathF.Abs(a.X) > MathF.Abs(a.Y))
        {
            float inv = 1f / MathF.Sqrt(a.X * a.X + a.Z * a.Z);
            c = new Vector3(a.Z * inv, 0f, -a.X * inv);
        }
        else
        {
            float inv = 1f / MathF.Sqrt(a.Y * a.Y + a.Z * a.Z);
            c = new Vector3(0f, a.Z * inv, -a.Y * inv);
        }
        b = c.Cross(a);
    }

    /// <summary>Best-matching representatives of two 4-RoSy fields. Port of compat_orientation_extrinsic_4.</summary>
    public static (Vector3, Vector3) CompatOrient4(Vector3 q0, Vector3 n0, Vector3 q1, Vector3 n1)
    {
        Vector3 a0 = q0, a1 = n0.Cross(q0);
        Vector3 b0 = q1, b1 = n1.Cross(q1);
        float best = float.NegativeInfinity;
        Vector3 ba = a0, bb = b0;
        for (int i = 0; i < 2; i++)
        {
            var A = i == 0 ? a0 : a1;
            for (int j = 0; j < 2; j++)
            {
                var B = j == 0 ? b0 : b1;
                float s = MathF.Abs(A.Dot(B));
                if (s > best) { best = s; ba = A; bb = B; }
            }
        }
        float dp = ba.Dot(bb);
        return (ba, bb * (dp < 0 ? -1f : 1f));
    }

    /// <summary>Port of compat_orientation_extrinsic_index_4 (rotation indices of the matched representatives).</summary>
    public static (int, int) CompatOrientIndex4(Vector3 q0, Vector3 n0, Vector3 q1, Vector3 n1)
    {
        Vector3 a0 = q0, a1 = n0.Cross(q0);
        Vector3 b0 = q1, b1 = n1.Cross(q1);
        float best = float.NegativeInfinity;
        int ba = 0, bb = 0;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                float sc = MathF.Abs((i == 0 ? a0 : a1).Dot(j == 0 ? b0 : b1));
                if (sc > best) { best = sc; ba = i; bb = j; }
            }
        if ((ba == 0 ? a0 : a1).Dot(bb == 0 ? b0 : b1) < 0) bb += 2;
        return (ba, bb);
    }

    /// <summary>Point minimizing distance to p0, p1 while lying on both tangent planes. Port of middle_point.</summary>
    public static Vector3 MiddlePoint(Vector3 p0, Vector3 n0, Vector3 p1, Vector3 n1)
    {
        float n0p0 = n0.Dot(p0), n0p1 = n0.Dot(p1),
              n1p0 = n1.Dot(p0), n1p1 = n1.Dot(p1),
              n0n1 = n0.Dot(n1),
              denom = 1f / (1f - n0n1 * n0n1 + 1e-4f),
              l0 = 2f * (n0p1 - n0p0 - n0n1 * (n1p0 - n1p1)) * denom,
              l1 = 2f * (n1p0 - n1p1 - n0n1 * (n0p1 - n0p0)) * denom;
        return 0.5f * (p0 + p1) - 0.25f * (n0 * l0 + n1 * l1);
    }

    public static Vector3 PositionFloor4(Vector3 o, Vector3 q, Vector3 n, Vector3 p, float scale, float invScale)
    {
        var t = n.Cross(q);
        var d = p - o;
        return o + q * (MathF.Floor(q.Dot(d) * invScale) * scale) + t * (MathF.Floor(t.Dot(d) * invScale) * scale);
    }

    static Vector2I PositionFloorIndex4(Vector3 o, Vector3 q, Vector3 n, Vector3 p, float invScale)
    {
        var t = n.Cross(q);
        var d = p - o;
        return new Vector2I((int)MathF.Floor(q.Dot(d) * invScale), (int)MathF.Floor(t.Dot(d) * invScale));
    }

    /// <summary>Lattice point (origin o, axes q / n×q, spacing scale) nearest to p.</summary>
    public static Vector3 PositionRound4(Vector3 o, Vector3 q, Vector3 n, Vector3 p, float scale, float invScale)
    {
        var t = n.Cross(q);
        var d = p - o;
        return o + q * (MathF.Round(q.Dot(d) * invScale) * scale) + t * (MathF.Round(t.Dot(d) * invScale) * scale);
    }

    /// <summary>Port of compat_position_extrinsic_4: nearest pair of lattice points of the two local lattices.</summary>
    public static (Vector3, Vector3) CompatPosition4(
        Vector3 p0, Vector3 n0, Vector3 q0, Vector3 o0,
        Vector3 p1, Vector3 n1, Vector3 q1, Vector3 o1, float scale, float invScale)
    {
        Vector3 t0 = n0.Cross(q0), t1 = n1.Cross(q1);
        var middle = MiddlePoint(p0, n0, p1, n1);
        var o0p = PositionFloor4(o0, q0, n0, middle, scale, invScale);
        var o1p = PositionFloor4(o1, q1, n1, middle, scale, invScale);
        float best = float.PositiveInfinity;
        int bi = 0, bj = 0;
        for (int i = 0; i < 4; i++)
        {
            var o0t = o0p + (q0 * (i & 1) + t0 * ((i & 2) >> 1)) * scale;
            for (int j = 0; j < 4; j++)
            {
                var o1t = o1p + (q1 * (j & 1) + t1 * ((j & 2) >> 1)) * scale;
                float cost = (o0t - o1t).LengthSquared();
                if (cost < best) { best = cost; bi = i; bj = j; }
            }
        }
        return (o0p + (q0 * (bi & 1) + t0 * ((bi & 2) >> 1)) * scale,
                o1p + (q1 * (bj & 1) + t1 * ((bj & 2) >> 1)) * scale);
    }

    /// <summary>Port of compat_position_extrinsic_index_4: integer lattice coordinates of the matched pair.</summary>
    public static (Vector2I, Vector2I) CompatPositionIndex4(
        Vector3 p0, Vector3 n0, Vector3 q0, Vector3 o0,
        Vector3 p1, Vector3 n1, Vector3 q1, Vector3 o1, float scale, float invScale, out float error)
    {
        Vector3 t0 = n0.Cross(q0), t1 = n1.Cross(q1);
        var middle = MiddlePoint(p0, n0, p1, n1);
        var o0p = PositionFloorIndex4(o0, q0, n0, middle, invScale);
        var o1p = PositionFloorIndex4(o1, q1, n1, middle, invScale);
        float best = float.PositiveInfinity;
        int bi = 0, bj = 0;
        for (int i = 0; i < 4; i++)
        {
            var o0t = o0 + (q0 * ((i & 1) + o0p.X) + t0 * (((i & 2) >> 1) + o0p.Y)) * scale;
            for (int j = 0; j < 4; j++)
            {
                var o1t = o1 + (q1 * ((j & 1) + o1p.X) + t1 * (((j & 2) >> 1) + o1p.Y)) * scale;
                float cost = (o0t - o1t).LengthSquared();
                if (cost < best) { best = cost; bi = i; bj = j; }
            }
        }
        error = best;
        return (new Vector2I((bi & 1) + o0p.X, ((bi & 2) >> 1) + o0p.Y),
                new Vector2I((bj & 1) + o1p.X, ((bj & 2) >> 1) + o1p.Y));
    }
}
