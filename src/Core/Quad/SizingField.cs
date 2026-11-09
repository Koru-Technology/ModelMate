using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>
/// Target edge length per base-mesh vertex for adaptive quad remeshing.
///
/// Spacing follows the local thickness of the body part (a ray cast inward to the far side), so every
/// cross-section, whether torso, arm or horn, gets a similar number of sides, as in hand-made low-poly models.
///   relative size r = (thickness / median thickness)^adaptivity     (adaptivity 0 = uniform, 1 = fully proportional)
/// then: ratio between finest and coarsest spacing capped; growth limited (s_v ≤ s_u + gradient·|u−v|) so density
/// changes gradually (abrupt changes create extra triangle/pentagon pairs); floor at what the base mesh can
/// resolve; rescaled so Σ area / s² meets the quad budget.
/// </summary>
public static class SizingField
{
    public sealed class Result
    {
        public float[] Size;          // target edge length per base vertex
        public float[] Thickness;     // smoothed local thickness (diameter estimate)
        public float Min, Median, Max;
        public bool FloorHit;         // some sizes were raised to the base-mesh resolution floor
    }

    /// <param name="curvature">Weight of the equal-error curvature term (spacing ∝ κ^-½, the rule for constant chord
    /// error); 0 = thickness only. The finer of the thickness and curvature requirements wins at each point.</param>
    public static Result Build(MeshData m, int targetQuads, float adaptivity, float floorSize,
        float maxRatio = 10f, float gradient = 0.3f, Action<string> log = null, float curvature = 0f)
    {
        int n = m.VertexCount;
        if (m.Nrm == null) m.ComputeNormals();
        var bvh = new Bvh(m.Pos, m.Tris);
        var box = m.Bounds();
        float diag = box.Size.Length();

        // 1) thickness: ray inward along -normal to the first back-facing (exit) triangle
        var thick = new float[n];
        Parallel.For(0, n, v =>
        {
            var d = -m.Nrm[v];
            var o = m.Pos[v] + d * (1e-4f * diag);
            thick[v] = bvh.Raycast(o, d, diag, out var hit, t => bvh.TriNormal[t].Dot(d) > 0) ? hit.Dist : diag;
        });
        // smooth in log space (removes ray noise at creases and tips)
        var (start, nbr) = m.BuildAdjacency();
        var lg = new float[n];
        for (int v = 0; v < n; v++) lg[v] = MathF.Log(MathF.Max(thick[v], 1e-6f));
        for (int pass = 0; pass < 8; pass++)
        {
            var next = new float[n];
            Parallel.For(0, n, v =>
            {
                float s = 0; int c = 0;
                for (int k = start[v]; k < start[v + 1]; k++) { s += lg[nbr[k]]; c++; }
                next[v] = c > 0 ? 0.5f * lg[v] + 0.5f * s / c : lg[v];
            });
            lg = next;
        }
        var sorted = (float[])lg.Clone();
        Array.Sort(sorted);
        float medLog = sorted[n / 2];

        // 1b) curvature: mean normal change per unit length to the neighbours, smoothed in log space over a wide area
        float[] lk = null; float medK = 0;
        if (curvature > 0)
        {
            lk = new float[n];
            Parallel.For(0, n, v =>
            {
                float s = 0; int c = 0;
                for (int k = start[v]; k < start[v + 1]; k++)
                {
                    int u = nbr[k];
                    float d = (m.Pos[u] - m.Pos[v]).Length();
                    if (d > 1e-9f) { s += (m.Nrm[u] - m.Nrm[v]).Length() / d; c++; }
                }
                lk[v] = MathF.Log(MathF.Max(c > 0 ? s / c : 0, 1e-3f / diag));
            });
            for (int pass = 0; pass < 30; pass++)
            {
                var next = new float[n];
                var cur = lk;
                Parallel.For(0, n, v =>
                {
                    float s = 0; int c = 0;
                    for (int k = start[v]; k < start[v + 1]; k++) { s += cur[nbr[k]]; c++; }
                    next[v] = c > 0 ? 0.5f * cur[v] + 0.5f * s / c : cur[v];
                });
                lk = next;
            }
            var sk = (float[])lk.Clone();
            Array.Sort(sk);
            medK = sk[n / 2];
        }

        // 2) relative size (finer of the thickness and curvature requirements), ratio-capped around the median
        float halfSpan = MathF.Log(MathF.Sqrt(maxRatio));
        var r = new float[n];
        for (int v = 0; v < n; v++)
        {
            float x = adaptivity * (lg[v] - medLog);
            if (lk != null) x = MathF.Min(x, -0.5f * curvature * (lk[v] - medK));
            r[v] = MathF.Exp(Math.Clamp(x, -halfSpan, halfSpan));
            thick[v] = MathF.Exp(lg[v]);
        }

        // dual areas
        var area = new float[n];
        for (int t = 0; t < m.TriCount; t++)
        {
            float a = m.TriNormal(t).Length() / 6f;
            area[m.Tris[3 * t]] += a; area[m.Tris[3 * t + 1]] += a; area[m.Tris[3 * t + 2]] += a;
        }

        // 3) scale to budget, limit growth, apply floor; iterate (the limits change the count)
        var size = new float[n];
        double Count(float[] s) { double c = 0; for (int v = 0; v < n; v++) c += area[v] / ((double)s[v] * s[v]); return c; }
        double k = Math.Sqrt(Count(r) / targetQuads);
        for (int v = 0; v < n; v++) size[v] = (float)(k * r[v]);
        bool floorHit = false;
        for (int it = 0; it < 6; it++)
        {
            LimitGrowth(m, start, nbr, size, gradient);
            for (int v = 0; v < n; v++) if (size[v] < floorSize) { size[v] = floorSize; floorHit = true; }
            double f = Math.Sqrt(Count(size) / targetQuads);
            for (int v = 0; v < n; v++) size[v] = (float)(size[v] * f);
        }
        LimitGrowth(m, start, nbr, size, gradient);
        for (int v = 0; v < n; v++) size[v] = MathF.Max(size[v], floorSize);

        var ss = (float[])size.Clone();
        Array.Sort(ss);
        var res = new Result { Size = size, Thickness = thick, Min = ss[0], Median = ss[n / 2], Max = ss[^1], FloorHit = floorHit };
        log?.Invoke($"Sizing field: adaptivity {adaptivity}, curvature {curvature}, edge length {res.Min:0.####}–{res.Max:0.####} (median {res.Median:0.####}), " +
                    $"≈{Count(size):F0} quads for a target of {targetQuads}{(floorHit ? "; finest sizes limited by base-mesh resolution" : "")}");
        return res;
    }

    /// <summary>Dijkstra-style: s_v ≤ s_u + g·|p_u − p_v| for all neighbours, so size grows by at most g per unit length.</summary>
    static void LimitGrowth(MeshData m, int[] start, int[] nbr, float[] s, float g)
    {
        var pq = new PriorityQueue<int, float>();
        for (int v = 0; v < s.Length; v++) pq.Enqueue(v, s[v]);
        var done = new bool[s.Length];
        while (pq.TryDequeue(out int u, out float su))
        {
            if (done[u] || su > s[u]) continue;
            done[u] = true;
            for (int k = start[u]; k < start[u + 1]; k++)
            {
                int v = nbr[k];
                float lim = s[u] + g * (m.Pos[u] - m.Pos[v]).Length();
                if (lim < s[v]) { s[v] = lim; pq.Enqueue(v, lim); }
            }
        }
    }
}
