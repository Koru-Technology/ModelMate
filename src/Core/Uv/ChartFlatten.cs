using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core.Uv;

/// <summary>
/// Per-triangle data for flattening: area and the gradients of the three barycentric (hat) functions, expressed in
/// the triangle's own orthonormal 2D frame. For a map with per-vertex coords uv_j, its Jacobian on the triangle is
/// J = Σ_j uv_j ⊗ g_j, which is all the solvers and the distortion measure need.
/// </summary>
public sealed class TriGeom
{
    public readonly float[] Area;
    public readonly Vector2[] Grad;   // 3 per triangle
    public readonly double TotalArea;
    /// <summary>Degenerate (zero-area) triangles: weights α with Σ α_j·p_j = 0 in 3D (the middle vertex sits on the
    /// segment of the other two, or two vertices coincide). They have no conformal energy, so the solvers hold them
    /// to the same relation in UV; otherwise such a triangle could fold over its neighbours.</summary>
    public readonly Dictionary<int, Vector3> Degenerate = new();

    public TriGeom(MeshData m)
    {
        int nt = m.TriCount;
        Area = new float[nt];
        Grad = new Vector2[3 * nt];
        double total = 0;
        for (int t = 0; t < nt; t++)
        {
            var p0 = m.Pos[m.Tris[3 * t]];
            var p1 = m.Pos[m.Tris[3 * t + 1]];
            var p2 = m.Pos[m.Tris[3 * t + 2]];
            var e1 = p1 - p0; var e2 = p2 - p0;
            var n = e1.Cross(e2);
            float a2 = n.Length();
            float l01 = e1.LengthSquared(), l02 = e2.LengthSquared(), l12 = (p2 - p1).LengthSquared();
            float lmax = MathF.Max(l01, MathF.Max(l02, l12));
            if (a2 <= 1e-9f * lmax || lmax <= 0)
            {
                // degenerate: no energy, no distortion; keep the middle vertex on the segment of the other two
                if (lmax <= 0) continue;
                Vector3 al;
                if (l12 >= l01 && l12 >= l02) { float u = Math.Clamp((p0 - p1).Dot(p2 - p1) / l12, 0, 1); al = new Vector3(1, -(1 - u), -u); }
                else if (l02 >= l01) { float u = Math.Clamp((p1 - p0).Dot(p2 - p0) / l02, 0, 1); al = new Vector3(-(1 - u), 1, -u); }
                else { float u = Math.Clamp((p2 - p0).Dot(p1 - p0) / l01, 0, 1); al = new Vector3(-(1 - u), -u, 1); }
                Degenerate[t] = al;
                continue;
            }
            Area[t] = 0.5f * a2;
            total += Area[t];
            // frame: x along e1, y = n × x (counter-clockwise faces stay counter-clockwise)
            var x = e1.Normalized();
            var y = (n / a2).Cross(x);
            var q0 = Vector2.Zero;
            var q1 = new Vector2(e1.Dot(x), e1.Dot(y));
            var q2 = new Vector2(e2.Dot(x), e2.Dot(y));
            // ∇φ_j = rot90(e_j) / (2A), e_j = edge opposite vertex j
            Grad[3 * t] = Rot(q2 - q1) / a2;
            Grad[3 * t + 1] = Rot(q0 - q2) / a2;
            Grad[3 * t + 2] = Rot(q1 - q0) / a2;
        }
        TotalArea = total;
    }

    static Vector2 Rot(Vector2 e) => new(-e.Y, e.X);

    /// <summary>Singular values (s1 ≥ s2) and determinant of the map's Jacobian on triangle t.</summary>
    public (float s1, float s2, float det) Sv(int t, Vector2 a, Vector2 b, Vector2 c)
    {
        var g0 = Grad[3 * t]; var g1 = Grad[3 * t + 1]; var g2 = Grad[3 * t + 2];
        // J = [[∂u/∂x, ∂u/∂y], [∂v/∂x, ∂v/∂y]]
        float j00 = a.X * g0.X + b.X * g1.X + c.X * g2.X;
        float j01 = a.X * g0.Y + b.X * g1.Y + c.X * g2.Y;
        float j10 = a.Y * g0.X + b.Y * g1.X + c.Y * g2.X;
        float j11 = a.Y * g0.Y + b.Y * g1.Y + c.Y * g2.Y;
        float e = 0.5f * (j00 + j11), f = 0.5f * (j00 - j11), g = 0.5f * (j10 + j01), h = 0.5f * (j10 - j01);
        float q = MathF.Sqrt(e * e + h * h), r = MathF.Sqrt(f * f + g * g);
        return (q + r, MathF.Abs(q - r), j00 * j11 - j01 * j10);
    }
}

/// <summary>Distortion of one flattened chart, measured after scaling it so its UV area equals its 3D area.</summary>
public struct ChartQuality
{
    public bool Flipped;
    public float AnisoP95;     // area-weighted 95th percentile of s1/s2 (1 = conformal)
    public float AreaStretch;  // area-weighted p95 of max(a, 1/a), a = area scale s1·s2 over the chart's mean (1 = uniform texel density)
    public float AreaStretchWide; // same, p99

    public override string ToString() => Flipped ? "flipped" : $"aniso p95 {AnisoP95:F2}, area stretch p95 {AreaStretch:F2}, p99 {AreaStretchWide:F2}";
}

/// <summary>
/// Flattens a disk-shaped set of triangles into the plane.
///
/// LSCM (Lévy et al. 2002): least-squares conformal map, two pinned vertices, solved as the normal equations of the
/// Cauchy-Riemann residuals. ARAP (Liu et al. 2008): local/global iterations that pull every triangle's Jacobian
/// toward its closest rotation, evening out the area scale a conformal map leaves on curved charts. Both systems
/// are sparse, symmetric positive definite once pinned, and solved by Jacobi-preconditioned conjugate gradient,
/// warm-started from the caller's layout.
/// </summary>
public static class ChartFlatten
{
    /// <summary>Chart-local system: vertex indices, and block-CSR adjacency (diagonal first in every row).</summary>
    sealed class Sys
    {
        public int N;
        public int[] Start, Col;       // CSR
        public double[] D, C;          // entry block [[D, -C], [C, D]]
        public int[] TriLocal;         // 3 local indices per chart triangle
    }

    static Sys Build(TriGeom g, List<int> tris, int[] tl, int n, bool conformal)
    {
        var s = new Sys { N = n, TriLocal = tl };
        var nb = new List<int>[n];
        for (int i = 0; i < n; i++) nb[i] = new List<int>(8) { i };
        for (int i = 0; i < tris.Count; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                int a = s.TriLocal[3 * i + k], b = s.TriLocal[3 * i + (k + 1) % 3];
                if (!nb[a].Contains(b)) nb[a].Add(b);
                if (!nb[b].Contains(a)) nb[b].Add(a);
            }
        }
        s.Start = new int[n + 1];
        for (int i = 0; i < n; i++) s.Start[i + 1] = s.Start[i] + nb[i].Count;
        s.Col = new int[s.Start[n]];
        for (int i = 0; i < n; i++) nb[i].CopyTo(s.Col, s.Start[i]);
        s.D = new double[s.Col.Length];
        s.C = new double[s.Col.Length];
        for (int i = 0; i < tris.Count; i++)
        {
            int t = tris[i];
            double A = g.Area[t];
            if (A <= 0)
            {
                if (g.Degenerate.Count == 0 || !g.Degenerate.TryGetValue(t, out var al)) continue;
                // stiff penalty w·|Σ α_j uv_j|² (isotropic: D part only); the energy terms are dimensionless, O(1) per vertex
                const double w = 100;
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                        s.D[Entry(s, s.TriLocal[3 * i + a], s.TriLocal[3 * i + b])] += w * al[a] * al[b];
                continue;
            }
            for (int a = 0; a < 3; a++)
            {
                var ga = g.Grad[3 * t + a];
                int ra = s.TriLocal[3 * i + a];
                for (int b = 0; b < 3; b++)
                {
                    var gb = g.Grad[3 * t + b];
                    int e = Entry(s, ra, s.TriLocal[3 * i + b]);
                    s.D[e] += A * (ga.X * (double)gb.X + ga.Y * (double)gb.Y);
                    if (conformal) s.C[e] += A * (ga.X * (double)gb.Y - ga.Y * (double)gb.X);
                }
            }
        }
        return s;
    }

    static int Entry(Sys s, int r, int c)
    {
        for (int e = s.Start[r]; e < s.Start[r + 1]; e++) if (s.Col[e] == c) return e;
        throw new InvalidOperationException("missing CSR entry");
    }

    /// <summary>y = H x on the interleaved (u, v) vector; pinned rows are zeroed.</summary>
    static void Mul(Sys s, double[] x, double[] y, bool[] pin, double reg)
    {
        for (int i = 0; i < s.N; i++)
        {
            double yu = 0, yv = 0;
            for (int e = s.Start[i]; e < s.Start[i + 1]; e++)
            {
                int j = s.Col[e];
                double d = s.D[e], c = s.C[e], xu = x[2 * j], xv = x[2 * j + 1];
                yu += d * xu - c * xv;
                yv += c * xu + d * xv;
            }
            if (pin[i]) { y[2 * i] = 0; y[2 * i + 1] = 0; }
            else { y[2 * i] = yu + reg * x[2 * i]; y[2 * i + 1] = yv + reg * x[2 * i + 1]; }
        }
    }

    /// <summary>Projected, Jacobi-preconditioned CG for H x = b with pinned vertices held at their values in x.</summary>
    static int Solve(Sys s, double[] x, double[] b, bool[] pin, double reg, int maxIt, double tol)
    {
        int n2 = 2 * s.N;
        var r = new double[n2]; var z = new double[n2]; var p = new double[n2]; var q = new double[n2];
        var dinv = new double[s.N];
        for (int i = 0; i < s.N; i++) dinv[i] = 1.0 / Math.Max(s.D[s.Start[i]] + reg, 1e-30);
        // scale of the problem, independent of the warm start: the residual with every free vertex at 0 (what the
        // pinned vertices and b ask of the free ones)
        var x0 = new double[n2];
        for (int i = 0; i < n2; i++) x0[i] = pin[i >> 1] ? x[i] : 0;
        Mul(s, x0, q, pin, reg);
        double rref = 0;
        for (int i = 0; i < n2; i++) if (!pin[i >> 1]) { double d = b[i] - q[i]; rref += d * d; }
        Mul(s, x, q, pin, reg);
        double rr0 = 0;
        for (int i = 0; i < n2; i++) { r[i] = pin[i >> 1] ? 0 : b[i] - q[i]; rr0 += r[i] * r[i]; }
        double stop = Math.Max(tol * tol * (rref > 0 ? rref : rr0), 1e-40);
        double rz = 0;
        for (int i = 0; i < n2; i++) { z[i] = r[i] * dinv[i >> 1]; p[i] = z[i]; rz += r[i] * z[i]; }
        int it = 0;
        for (; it < maxIt; it++)
        {
            double rr = 0;
            for (int i = 0; i < n2; i++) rr += r[i] * r[i];
            if (rr <= stop) break;
            Mul(s, p, q, pin, reg);
            double pq = 0;
            for (int i = 0; i < n2; i++) pq += p[i] * q[i];
            if (pq <= 0) break;
            double alpha = rz / pq;
            for (int i = 0; i < n2; i++) { x[i] += alpha * p[i]; r[i] -= alpha * q[i]; }
            double rz2 = 0;
            for (int i = 0; i < n2; i++) { z[i] = r[i] * dinv[i >> 1]; rz2 += r[i] * z[i]; }
            double beta = rz2 / Math.Max(rz, 1e-300);
            rz = rz2;
            for (int i = 0; i < n2; i++) p[i] = z[i] + beta * p[i];
        }
        return it;
    }

    public sealed class Result
    {
        public Vector2[] Uv;           // per local vertex, world units (UV area = 3D area)
        public ChartQuality Quality;
        public int Iterations;
    }

    /// <summary>
    /// LSCM from a warm start (per local vertex, any similarity of a good layout). The two pins are the warm-start
    /// vertices farthest apart, held where the warm start puts them, so the solution keeps its placement.
    /// </summary>
    /// <param name="fixedMask">optional: vertices held at their warm-start positions (then no other pins, and no
    /// rescaling, so the result stays in the caller's frame)</param>
    public static Result Lscm(TriGeom g, List<int> tris, int[] tl, Vector2[] warm, bool[] fixedMask = null, int maxIt = 4000, bool measure = true)
    {
        var s = Build(g, tris, tl, warm.Length, true);
        int n = s.N;
        var (x, cen) = Centered(warm);
        bool[] pin;
        if (fixedMask != null) pin = (bool[])fixedMask.Clone();
        else
        {
            pin = new bool[n];
            var (p0, p1) = FarPair(warm);
            pin[p0] = true; pin[p1] = true;
        }
        double reg = 1e-9 * Diag(s);
        var b = new double[2 * n];
        for (int i = 0; i < n; i++) { b[2 * i] = reg * x[2 * i]; b[2 * i + 1] = reg * x[2 * i + 1]; }
        int it = Solve(s, x, b, pin, reg, maxIt, 1e-6);
        var uv = new Vector2[n];
        for (int i = 0; i < n; i++) uv[i] = new Vector2((float)x[2 * i], (float)x[2 * i + 1]) + cen;
        SnapDegenerate(g, tris, s.TriLocal, uv, fixedMask);
        if (fixedMask == null) Normalize(g, tris, s.TriLocal, uv);
        return new Result { Uv = uv, Quality = measure ? Measure(g, tris, s.TriLocal, uv) : default, Iterations = it };
    }

    /// <summary>ARAP local/global iterations starting from a valid layout (normally the LSCM one).</summary>
    public static Result Arap(TriGeom g, List<int> tris, int[] tl, Vector2[] start, bool[] fixedMask = null, int iterations = 5, bool measure = true)
    {
        var s = Build(g, tris, tl, start.Length, false);
        int n = s.N;
        var (x, cen) = Centered(start);
        bool[] pin;
        if (fixedMask != null) pin = (bool[])fixedMask.Clone();
        else { pin = new bool[n]; pin[0] = true; }
        double reg = 1e-9 * Diag(s);
        var b = new double[2 * n];
        int total = 0;
        for (int iter = 0; iter < iterations; iter++)
        {
            // local step: closest rotation to each triangle's Jacobian; global step: H x = Σ A g_i·R
            Array.Clear(b);
            for (int i = 0; i < tris.Count; i++)
            {
                int t = tris[i];
                double A = g.Area[t];
                if (A <= 0) continue;
                int a = s.TriLocal[3 * i], bb = s.TriLocal[3 * i + 1], c = s.TriLocal[3 * i + 2];
                var g0 = g.Grad[3 * t]; var g1 = g.Grad[3 * t + 1]; var g2 = g.Grad[3 * t + 2];
                double j00 = x[2 * a] * g0.X + x[2 * bb] * g1.X + x[2 * c] * g2.X;
                double j01 = x[2 * a] * g0.Y + x[2 * bb] * g1.Y + x[2 * c] * g2.Y;
                double j10 = x[2 * a + 1] * g0.X + x[2 * bb + 1] * g1.X + x[2 * c + 1] * g2.X;
                double j11 = x[2 * a + 1] * g0.Y + x[2 * bb + 1] * g1.Y + x[2 * c + 1] * g2.Y;
                double th = Math.Atan2(j10 - j01, j00 + j11);
                double co = Math.Cos(th), si = Math.Sin(th);
                // R = [[co, -si], [si, co]]; rows are the target gradients of u and v
                for (int k = 0; k < 3; k++)
                {
                    var gk = g.Grad[3 * t + k];
                    int r = s.TriLocal[3 * i + k];
                    b[2 * r] += A * (gk.X * co - gk.Y * si);
                    b[2 * r + 1] += A * (gk.X * si + gk.Y * co);
                }
            }
            for (int i = 0; i < n; i++) { b[2 * i] += reg * x[2 * i]; b[2 * i + 1] += reg * x[2 * i + 1]; }
            total += Solve(s, x, b, pin, reg, 2000, 1e-3);
        }
        var uv = new Vector2[n];
        for (int i = 0; i < n; i++) uv[i] = new Vector2((float)x[2 * i], (float)x[2 * i + 1]) + cen;
        SnapDegenerate(g, tris, s.TriLocal, uv, fixedMask);
        if (fixedMask == null) Normalize(g, tris, s.TriLocal, uv);
        return new Result { Uv = uv, Quality = measure ? Measure(g, tris, s.TriLocal, uv) : default, Iterations = total };
    }

    /// <summary>Puts the middle vertex of each degenerate triangle exactly on its segment (the penalty leaves it close).</summary>
    static void SnapDegenerate(TriGeom g, List<int> tris, int[] tl, Vector2[] uv, bool[] fixedMask)
    {
        if (g.Degenerate.Count == 0) return;
        for (int pass = 0; pass < 3; pass++)
            for (int i = 0; i < tris.Count; i++)
            {
                if (g.Area[tris[i]] > 0 || !g.Degenerate.TryGetValue(tris[i], out var al)) continue;
                int mid = al.X == 1 ? 0 : al.Y == 1 ? 1 : 2;
                int lm = tl[3 * i + mid];
                if (fixedMask != null && fixedMask[lm]) continue;
                var p = Vector2.Zero;
                for (int k = 0; k < 3; k++) if (k != mid) p -= al[k] * uv[tl[3 * i + k]];
                uv[lm] = p;
            }
    }

    static (double[] x, Vector2 cen) Centered(Vector2[] p)
    {
        double cx = 0, cy = 0;
        foreach (var q in p) { cx += q.X; cy += q.Y; }
        cx /= Math.Max(1, p.Length); cy /= Math.Max(1, p.Length);
        var x = new double[2 * p.Length];
        for (int i = 0; i < p.Length; i++) { x[2 * i] = p[i].X - cx; x[2 * i + 1] = p[i].Y - cy; }
        return (x, new Vector2((float)cx, (float)cy));
    }

    static double Diag(Sys s)
    {
        double d = 0;
        for (int i = 0; i < s.N; i++) d += s.D[s.Start[i]];
        return d / Math.Max(1, s.N);
    }

    static (int, int) FarPair(Vector2[] p)
    {
        int a = 0;
        for (int i = 1; i < p.Length; i++) if (p[i].X < p[a].X) a = i;
        for (int pass = 0; pass < 2; pass++)
        {
            int b = a; float best = -1;
            for (int i = 0; i < p.Length; i++) { float d = (p[i] - p[a]).LengthSquared(); if (d > best) { best = d; b = i; } }
            if (pass == 0) a = b; else return (a, b == a ? (a + 1) % p.Length : b);
        }
        return (0, 1);
    }

    /// <summary>Scales about the centroid so the chart's UV area equals its 3D area.</summary>
    public static void Normalize(TriGeom g, List<int> tris, int[] tl, Vector2[] uv)
    {
        double a3 = 0, auv = 0;
        for (int i = 0; i < tris.Count; i++)
        {
            a3 += g.Area[tris[i]];
            var a = uv[tl[3 * i]]; var b = uv[tl[3 * i + 1]]; var c = uv[tl[3 * i + 2]];
            auv += 0.5 * ((b.X - a.X) * (double)(c.Y - a.Y) - (b.Y - a.Y) * (double)(c.X - a.X));
        }
        if (auv <= 1e-30 || a3 <= 0) return;
        float k = (float)Math.Sqrt(a3 / auv);
        var cen = Vector2.Zero;
        foreach (var p in uv) cen += p;
        cen /= uv.Length;
        for (int i = 0; i < uv.Length; i++) uv[i] = cen + (uv[i] - cen) * k;
    }

    /// <summary>Distortion of a layout given per-local-vertex coords (assumed normalized to UV area = 3D area).</summary>
    public static ChartQuality Measure(TriGeom g, List<int> tris, int[] tl, Vector2[] uv)
    {
        var q = new ChartQuality();
        int n = tris.Count;
        var anK = new float[n]; var anW = new float[n];
        var arK = new float[n]; var arW = new float[n];
        int k = 0;
        double maxA = 0;
        for (int i = 0; i < n; i++) maxA = Math.Max(maxA, g.Area[tris[i]]);
        for (int i = 0; i < n; i++)
        {
            int t = tris[i];
            float A = g.Area[t];
            if (A <= 0) continue;
            var (s1, s2, det) = g.Sv(t, uv[tl[3 * i]], uv[tl[3 * i + 1]], uv[tl[3 * i + 2]]);
            // a flip on a sliver much smaller than its neighbours covers no texel; anything else invalidates
            if (det <= 0 && A > 1e-4 * maxA) q.Flipped = true;
            anK[k] = s2 > 1e-12f ? s1 / s2 : 1e6f; anW[k] = A;
            arK[k] = MathF.Max(det, 1e-12f); arW[k] = A;
            k++;
        }
        if (k == 0) { q.AnisoP95 = 1; q.AreaStretch = 1; q.AreaStretchWide = 1; return q; }
        // the same measure the layout check applies to the whole atlas, relative to this chart's mean density
        double num = 0, den = 0;
        for (int i = 0; i < k; i++) { num += arK[i] * (double)arW[i]; den += arW[i]; }
        float mean = (float)(num / Math.Max(den, 1e-30));
        for (int i = 0; i < k; i++) { float a = arK[i] / mean; arK[i] = MathF.Max(a, 1 / a); }
        Array.Sort(anK, anW, 0, k);
        Array.Sort(arK, arW, 0, k);
        q.AnisoP95 = WQuantile(anK, anW, k, 0.95f);
        q.AreaStretch = WQuantile(arK, arW, k, 0.95f);
        q.AreaStretchWide = WQuantile(arK, arW, k, 0.99f);
        return q;
    }

    /// <summary>Weighted quantile of values sorted ascending.</summary>
    static float WQuantile(float[] v, float[] w, int n, float q)
    {
        double total = 0;
        for (int i = 0; i < n; i++) total += w[i];
        double acc = 0, target = q * total;
        for (int i = 0; i < n; i++) { acc += w[i]; if (acc >= target) return v[i]; }
        return v[n - 1];
    }

    /// <summary>
    /// The layout overlaps itself: a vertex's fan of triangles wraps around it more than once, or some pair of
    /// boundary edges (chart border, hole borders and both sides of cuts) cross. With no flipped triangles, a
    /// layout that passes both is injective.
    /// </summary>
    /// <param name="bnd">boundary corners: corner c = 3·i + k means the edge from corner k to k + 1 of triangle i</param>
    public static bool Overlaps(int[] tl, int[] bnd, Vector2[] uv)
    {
        // local folds that keep every triangle's orientation: a fan wrapping more than once around its vertex
        // (an interior vertex whose angles add up to 4π, or a boundary vertex past 2π) overlaps itself without any
        // boundary edges crossing
        var ang = new double[uv.Length];
        for (int c = 0; c < tl.Length; c++)
        {
            var a = uv[tl[c]]; var b = uv[tl[c - c % 3 + (c % 3 + 1) % 3]]; var d = uv[tl[c - c % 3 + (c % 3 + 2) % 3]];
            var u = b - a; var v = d - a;
            ang[tl[c]] += Math.Atan2(u.X * (double)v.Y - u.Y * (double)v.X, u.X * (double)v.X + u.Y * (double)v.Y);
        }
        var onB = new bool[uv.Length];
        foreach (int c in bnd) onB[tl[c]] = true;
        for (int i = 0; i < uv.Length; i++)
            if (ang[i] > (onB[i] ? 2 * Math.PI - 1e-4 : 2 * Math.PI + 1e-3)) return true;

        var edges = new List<(int a, int b, float x0, float x1)>(bnd.Length);
        foreach (int c in bnd)
        {
            int a = tl[c], b = tl[c - c % 3 + (c % 3 + 1) % 3];
            edges.Add((a, b, MathF.Min(uv[a].X, uv[b].X), MathF.Max(uv[a].X, uv[b].X)));
        }
        edges.Sort((p, q) => p.x0.CompareTo(q.x0));
        for (int i = 0; i < edges.Count; i++)
        {
            var e = edges[i];
            var p0 = uv[e.a]; var p1 = uv[e.b];
            float y0 = MathF.Min(p0.Y, p1.Y), y1 = MathF.Max(p0.Y, p1.Y);
            for (int j = i + 1; j < edges.Count && edges[j].x0 <= e.x1; j++)
            {
                var f = edges[j];
                if (f.a == e.a || f.a == e.b || f.b == e.a || f.b == e.b) continue;
                var q0 = uv[f.a]; var q1 = uv[f.b];
                if (MathF.Max(q0.Y, q1.Y) < y0 || MathF.Min(q0.Y, q1.Y) > y1) continue;
                if (SegmentsCross(p0, p1, q0, q1)) return true;
            }
        }
        return false;
    }

    static bool SegmentsCross(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float d1 = Cross(b - a, c - a), d2 = Cross(b - a, d - a), d3 = Cross(d - c, a - c), d4 = Cross(d - c, b - c);
        return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
            || (d1 == 0 && d2 == 0 && d3 == 0 && d4 == 0 && Collinear(a, b, c, d));
    }

    static bool Collinear(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        // collinear segments overlap when their projections on the common line overlap with positive length
        var u = b - a;
        float l = u.LengthSquared();
        if (l <= 0) return false;
        float t0 = (c - a).Dot(u) / l, t1 = (d - a).Dot(u) / l;
        return MathF.Max(t0, t1) > 1e-6f && MathF.Min(t0, t1) < 1 - 1e-6f;
    }

    public static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

    /// <summary>Marks cell centres strictly inside a counter-clockwise triangle; true if any was already marked.</summary>
    public static bool RasterMark(Vector2 a, Vector2 b, Vector2 c, float area, int gw, int gh, bool[] grid)
    {
        int x0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X))));
        int x1 = Math.Min(gw - 1, (int)MathF.Ceiling(MathF.Max(a.X, MathF.Max(b.X, c.X))));
        int y0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.Y, MathF.Min(b.Y, c.Y))));
        int y1 = Math.Min(gh - 1, (int)MathF.Ceiling(MathF.Max(a.Y, MathF.Max(b.Y, c.Y))));
        float eps = 1e-4f * area;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
            {
                var q = new Vector2(x + 0.5f, y + 0.5f);
                if (Cross(b - a, q - a) <= eps || Cross(c - b, q - b) <= eps || Cross(a - c, q - c) <= eps) continue;
                int i = y * gw + x;
                if (grid[i]) return true;
                grid[i] = true;
            }
        return false;
    }
}
