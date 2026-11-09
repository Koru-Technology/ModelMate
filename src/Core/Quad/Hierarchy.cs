// Ported from Instant Meshes (src/hierarchy.cpp).
// Copyright (c) 2015 Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung.
// BSD 3-Clause License — see THIRD_PARTY.md.
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>One level of the multiresolution graph.</summary>
public sealed class Level
{
    public Vector3[] V, N;
    public float[] A;
    public int[] AdjStart, AdjId;   // CSR adjacency
    public float[] AdjW;
    public int[][] Phases;          // graph colouring: vertices of one colour can be updated in parallel
    // Links to the next finer level (level l-1); only set for l > 0.
    public int[] Up0, Up1;          // fine children of each coarse vertex (Up1 = -1 if none)
    // Solution and constraints.
    public Vector3[] Q, O, CQ;
    public float[] CQw;
    public float[] S;               // target edge length per vertex (uniform unless a sizing field is given)
    public Vector3[] CO;            // position constraint: a lattice point should lie here (when COw > 0)
    public float[] COw;

    public int Count => V.Length;
}

/// <summary>
/// Multiresolution hierarchy built by repeated greedy pairwise collapse (Jakob et al. 2015, §5).
/// Level 0 is the input mesh; each coarser level has roughly half the vertices.
/// </summary>
public sealed class Hierarchy
{
    public readonly List<Level> Levels = new();
    public float Scale;             // median target edge length (global reference)
    public Level Fine => Levels[0];

    const int MaxDepth = 25;

    /// <param name="sizing">Optional per-vertex target edge length (same length as m.Pos); null = uniform `scale`.
    /// Deviation from the reference, which only supports a global scale: compat and rounding use local scales,
    /// and coarse levels take the area-weighted mean of their children.</param>
    public static Hierarchy Build(MeshData m, float scale, int seed, float[] sizing = null)
    {
        var h = new Hierarchy { Scale = scale };
        if (m.Nrm == null) m.ComputeNormals();
        var (start, nbr) = m.BuildAdjacency();
        var l0 = new Level
        {
            V = (Vector3[])m.Pos.Clone(), N = (Vector3[])m.Nrm.Clone(),
            A = DualAreas(m), AdjStart = start, AdjId = nbr, AdjW = new float[nbr.Length],
        };
        Array.Fill(l0.AdjW, 1f); // uniform weights, as generate_adjacency_matrix_uniform
        l0.Phases = Coloring(l0, seed);
        h.Levels.Add(l0);
        for (int d = 0; d < MaxDepth; d++)
        {
            var c = Downsample(h.Levels[^1]);
            if (c == null) break;
            c.Phases = Coloring(c, seed + d + 1);
            h.Levels.Add(c);
            if (c.Count <= 1) break;
        }
        foreach (var l in h.Levels)
        {
            l.CQ = new Vector3[l.Count];
            l.CQw = new float[l.Count];
        }
        // scale per level: fine from sizing, coarse = area-weighted mean of children
        h.Levels[0].S = new float[h.Levels[0].Count];
        for (int i = 0; i < h.Levels[0].Count; i++) h.Levels[0].S[i] = sizing != null ? sizing[i] : scale;
        for (int li = 1; li < h.Levels.Count; li++)
        {
            var f = h.Levels[li - 1]; var c = h.Levels[li];
            c.S = new float[c.Count];
            for (int i = 0; i < c.Count; i++)
            {
                int u0 = c.Up0[i], u1 = c.Up1[i];
                if (u1 < 0) { c.S[i] = f.S[u0]; continue; }
                float a0 = f.A[u0], a1 = f.A[u1], sa = a0 + a1;
                c.S[i] = sa > 1e-30f ? (f.S[u0] * a0 + f.S[u1] * a1) / sa : 0.5f * (f.S[u0] + f.S[u1]);
            }
        }
        return h;
    }

    static float[] DualAreas(MeshData m)
    {
        var a = new float[m.VertexCount];
        for (int t = 0; t < m.TriCount; t++)
        {
            float ar = m.TriNormal(t).Length() * 0.5f / 3f;
            a[m.Tris[3 * t]] += ar; a[m.Tris[3 * t + 1]] += ar; a[m.Tris[3 * t + 2]] += ar;
        }
        return a;
    }

    /// <summary>Port of downsample_graph: greedy matching ordered by normal agreement × area ratio.</summary>
    static Level Downsample(Level f)
    {
        int n = f.Count;
        int nl = f.AdjId.Length;
        var order = new float[nl];
        var ei = new int[nl];
        var idx = new int[nl];
        for (int i = 0; i < n; i++)
            for (int k = f.AdjStart[i]; k < f.AdjStart[i + 1]; k++)
            {
                int j = f.AdjId[k];
                float dp = f.N[i].Dot(f.N[j]);
                float ratio = f.A[i] > f.A[j] ? f.A[i] / f.A[j] : f.A[j] / f.A[i];
                order[k] = -(dp * ratio);   // sort descending by dp*ratio
                ei[k] = i;
                idx[k] = k;
            }
        Array.Sort(order, idx);            // deterministic: ties broken by input order not guaranteed, so re-sort stably below
        StableTies(order, idx);

        var merged = new bool[n];
        var pairs = new List<(int, int)>();
        foreach (var k in idx)
        {
            int i = ei[k], j = f.AdjId[k];
            if (merged[i] || merged[j]) continue;
            merged[i] = merged[j] = true;
            pairs.Add((i, j));
        }
        if (pairs.Count == 0) return null;

        int cn = n - pairs.Count;
        var c = new Level { V = new Vector3[cn], N = new Vector3[cn], A = new float[cn], Up0 = new int[cn], Up1 = new int[cn] };
        var toLower = new int[n];
        int ci = 0;
        foreach (var (i, j) in pairs)
        {
            float a1 = f.A[i], a2 = f.A[j], s = a1 + a2;
            c.V[ci] = s > 1e-30f ? (f.V[i] * a1 + f.V[j] * a2) / s : (f.V[i] + f.V[j]) * 0.5f;
            var nn = f.N[i] * a1 + f.N[j] * a2;
            float len = nn.Length();
            c.N[ci] = len > 1e-30f ? nn / len : Vector3.Right;
            c.A[ci] = s;
            c.Up0[ci] = i; c.Up1[ci] = j;
            toLower[i] = toLower[j] = ci;
            ci++;
        }
        for (int i = 0; i < n; i++)
        {
            if (merged[i]) continue;
            c.V[ci] = f.V[i]; c.N[ci] = f.N[i]; c.A[ci] = f.A[i];
            c.Up0[ci] = i; c.Up1[ci] = -1;
            toLower[i] = ci;
            ci++;
        }

        // coarse adjacency: union of children's links, mapped down, weights summed, self-links dropped
        var lists = new List<(int id, float w)>[cn];
        int total = 0;
        Parallel.For(0, cn, i =>
        {
            var scratch = new SortedDictionary<int, float>();
            for (int u = 0; u < 2; u++)
            {
                int up = u == 0 ? c.Up0[i] : c.Up1[i];
                if (up < 0) continue;
                for (int k = f.AdjStart[up]; k < f.AdjStart[up + 1]; k++)
                {
                    int lo = toLower[f.AdjId[k]];
                    if (lo == i) continue;
                    scratch[lo] = scratch.TryGetValue(lo, out float w) ? w + f.AdjW[k] : f.AdjW[k];
                }
            }
            var l = new List<(int, float)>(scratch.Count);
            foreach (var kv in scratch) l.Add((kv.Key, kv.Value));
            lists[i] = l;
        });
        c.AdjStart = new int[cn + 1];
        for (int i = 0; i < cn; i++) { c.AdjStart[i + 1] = c.AdjStart[i] + lists[i].Count; }
        total = c.AdjStart[cn];
        c.AdjId = new int[total];
        c.AdjW = new float[total];
        for (int i = 0; i < cn; i++)
        {
            int k = c.AdjStart[i];
            foreach (var (id, w) in lists[i]) { c.AdjId[k] = id; c.AdjW[k] = w; k++; }
        }
        return c;
    }

    /// <summary>Within runs of equal keys, order by index so the result is deterministic.</summary>
    static void StableTies(float[] keys, int[] idx)
    {
        int i = 0;
        while (i < keys.Length)
        {
            int j = i + 1;
            while (j < keys.Length && keys[j] == keys[i]) j++;
            if (j - i > 1) Array.Sort(idx, i, j - i);
            i = j;
        }
    }

    /// <summary>Port of generate_graph_coloring_deterministic (greedy colouring in seeded random order).</summary>
    static int[][] Coloring(Level l, int seed)
    {
        int n = l.Count;
        var perm = new int[n];
        for (int i = 0; i < n; i++) perm[i] = i;
        var rng = new Random(seed);
        for (int i = n - 1; i > 0; i--) { int j = rng.Next(i + 1); (perm[i], perm[j]) = (perm[j], perm[i]); }
        var color = new int[n];
        Array.Fill(color, -1);
        var sizes = new List<int>();
        var possible = new List<bool>();
        foreach (var ip in perm)
        {
            for (int c = 0; c < possible.Count; c++) possible[c] = true;
            for (int k = l.AdjStart[ip]; k < l.AdjStart[ip + 1]; k++)
            {
                int c = color[l.AdjId[k]];
                if (c >= 0) possible[c] = false;
            }
            int chosen = possible.IndexOf(true);
            if (chosen < 0) { chosen = possible.Count; possible.Add(true); sizes.Add(0); }
            color[ip] = chosen;
            sizes[chosen]++;
        }
        var phases = new int[sizes.Count][];
        var fill = new int[sizes.Count];
        for (int c = 0; c < sizes.Count; c++) phases[c] = new int[sizes[c]];
        for (int i = 0; i < n; i++) phases[color[i]][fill[color[i]]++] = i;
        return phases;
    }

    /// <summary>Seeded random tangent orientations and lattice offsets on every level (resetSolution).</summary>
    public void ResetSolution(int seed)
    {
        for (int li = 0; li < Levels.Count; li++)
        {
            var l = Levels[li];
            l.Q = new Vector3[l.Count];
            l.O = new Vector3[l.Count];
            var rng = new Random(seed * 7919 + li);
            for (int i = 0; i < l.Count; i++)
            {
                FieldMath.CoordinateSystem(l.N[i], out var s, out var t);
                float ang = (float)(rng.NextDouble() * 2 * Math.PI);
                l.Q[i] = s * MathF.Cos(ang) + t * MathF.Sin(ang);
                float x = (float)rng.NextDouble() * 2 - 1, y = (float)rng.NextDouble() * 2 - 1;
                l.O[i] = l.V[i] + (s * x + t * y) * l.S[i];
            }
        }
    }

    /// <summary>
    /// Port of propagateConstraints (orientation part): restrict CQ/CQw from fine to coarse.
    /// Deviation: the reference binarizes coarse weights to 1; we average them so soft guidance
    /// (weights in [0,1]) keeps its strength at every level.
    /// </summary>
    /// <summary>
    /// Restricts position constraints (CO/COw) from fine to coarse levels (propagateConstraints, position part).
    /// Point constraints only: a coarse vertex takes the constraint of a constrained child.
    /// </summary>
    public void PropagatePositionConstraints()
    {
        foreach (var l in Levels) { l.CO ??= new Vector3[l.Count]; l.COw ??= new float[l.Count]; }
        for (int li = 0; li < Levels.Count - 1; li++)
        {
            var f = Levels[li]; var c = Levels[li + 1];
            for (int i = 0; i < c.Count; i++)
            {
                int u0 = c.Up0[i], u1 = c.Up1[i];
                int src = f.COw[u0] > 0 ? u0 : (u1 >= 0 && f.COw[u1] > 0 ? u1 : -1);
                if (src < 0) { c.COw[i] = 0; continue; }
                c.CO[i] = f.CO[src];
                c.COw[i] = f.COw[src];
            }
        }
    }

    public void PropagateConstraints()
    {
        for (int li = 0; li < Levels.Count - 1; li++)
        {
            var f = Levels[li];
            var c = Levels[li + 1];
            Parallel.For(0, c.Count, i =>
            {
                int u0 = c.Up0[i], u1 = c.Up1[i];
                bool h0 = f.CQw[u0] != 0, h1 = u1 >= 0 && f.CQw[u1] != 0;
                Vector3 cq = Vector3.Zero;
                float cqw = 0;
                if (h0 && !h1) { cq = f.CQ[u0]; cqw = f.CQw[u0] * (u1 >= 0 ? 0.5f : 1f); }
                else if (h1 && !h0) { cq = f.CQ[u1]; cqw = f.CQw[u1] * 0.5f; }
                else if (h0 && h1)
                {
                    var (a, b) = FieldMath.CompatOrient4(f.CQ[u0], f.N[u0], f.CQ[u1], f.N[u1]);
                    cq = a * f.CQw[u0] + b * f.CQw[u1];
                    cqw = 0.5f * (f.CQw[u0] + f.CQw[u1]);
                }
                if (cq != Vector3.Zero)
                {
                    var n = c.N[i];
                    cq -= n * n.Dot(cq);
                    if (cq.LengthSquared() > 1e-30f) cq = cq.Normalized(); else cqw = 0;
                }
                c.CQ[i] = cq;
                c.CQw[i] = cqw;
            });
        }
    }
}
