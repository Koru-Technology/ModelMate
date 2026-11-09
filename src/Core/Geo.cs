using System;
using System.Threading;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Hash for 64-bit edge keys ((a &lt;&lt; 32) | b). The default long.GetHashCode() is lo ^ hi = a ^ b, which
/// collides massively for nearby vertex indices (made a 148k-triangle edge count take seconds).
/// </summary>
public sealed class EdgeKeyComparer : System.Collections.Generic.IEqualityComparer<long>
{
    public static readonly EdgeKeyComparer Instance = new();
    public bool Equals(long x, long y) => x == y;
    public int GetHashCode(long k)
    {
        ulong h = (ulong)k * 0x9E3779B97F4A7C15UL;
        return (int)(h >> 32) ^ (int)h;
    }
}

public static class Geo
{
    /// <summary>Closest point on triangle abc to p (Ericson). u,v,w are barycentric weights of a,b,c.</summary>
    public static Vector3 ClosestPtTri(Vector3 p, Vector3 a, Vector3 b, Vector3 c, out float u, out float v, out float w)
    {
        Vector3 ab = b - a, ac = c - a, ap = p - a;
        float d1 = ab.Dot(ap), d2 = ac.Dot(ap);
        if (d1 <= 0 && d2 <= 0) { u = 1; v = 0; w = 0; return a; }
        Vector3 bp = p - b;
        float d3 = ab.Dot(bp), d4 = ac.Dot(bp);
        if (d3 >= 0 && d4 <= d3) { u = 0; v = 1; w = 0; return b; }
        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            float t = d1 / (d1 - d3);
            u = 1 - t; v = t; w = 0;
            return a + ab * t;
        }
        Vector3 cp = p - c;
        float d5 = ab.Dot(cp), d6 = ac.Dot(cp);
        if (d6 >= 0 && d5 <= d6) { u = 0; v = 0; w = 1; return c; }
        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            float t = d2 / (d2 - d6);
            u = 1 - t; v = 0; w = t;
            return a + ac * t;
        }
        float va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        {
            float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            u = 0; v = 1 - t; w = t;
            return b + (c - b) * t;
        }
        float denom = 1f / (va + vb + vc);
        float vv = vb * denom, ww = vc * denom;
        u = 1 - vv - ww; v = vv; w = ww;
        return a + ab * vv + ac * ww;
    }

    /// <summary>Möller–Trumbore. Returns t (or -1) and barycentrics (u for a, v for b, w for c).</summary>
    public static float RayTri(Vector3 o, Vector3 d, Vector3 a, Vector3 b, Vector3 c, out float u, out float v, out float w)
    {
        u = v = w = 0;
        Vector3 e1 = b - a, e2 = c - a;
        Vector3 pv = d.Cross(e2);
        float det = e1.Dot(pv);
        if (MathF.Abs(det) < 1e-20f) return -1;
        float inv = 1f / det;
        Vector3 tv = o - a;
        float bu = tv.Dot(pv) * inv;
        if (bu < 0 || bu > 1) return -1;
        Vector3 qv = tv.Cross(e1);
        float bv = d.Dot(qv) * inv;
        if (bv < 0 || bu + bv > 1) return -1;
        float t = e2.Dot(qv) * inv;
        u = 1 - bu - bv; v = bu; w = bv;
        return t;
    }

    public static float DistPointSegment(Vector3 p, Vector3 a, Vector3 b)
    {
        Vector3 ab = b - a;
        float l2 = ab.LengthSquared();
        float t = l2 > 0 ? Math.Clamp((p - a).Dot(ab) / l2, 0f, 1f) : 0f;
        return (a + ab * t - p).Length();
    }

    public static void AtomicMin(float[] arr, int i, float value)
    {
        float cur = Volatile.Read(ref arr[i]);
        while (value < cur)
        {
            float prev = Interlocked.CompareExchange(ref arr[i], value, cur);
            if (prev == cur) break;
            cur = prev;
        }
    }

    /// <summary>Orthonormal basis with +Y along dir and +Z as close as possible to refZ.</summary>
    public static Basis BasisYAlong(Vector3 dir, Vector3 refZ)
    {
        var y = dir.Normalized();
        var z = refZ - y * refZ.Dot(y);
        if (z.LengthSquared() < 1e-8f)
        {
            var alt = MathF.Abs(y.X) < 0.9f ? Vector3.Right : Vector3.Forward;
            z = alt - y * alt.Dot(y);
        }
        z = z.Normalized();
        var x = y.Cross(z);
        return new Basis(x, y, z);
    }
}
