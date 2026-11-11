using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>
/// Finds sharp convex tips (horn tips, noses, fingertips) on a dense surface: vertices that are the extreme point
/// along their own normal within a neighbourhood of one target edge length, around which the normal turns a lot
/// (cone-like). Only one tip per neighbourhood is kept (the sharpest).
/// </summary>
public static class TipDetector
{
    public static List<int> Find(MeshData m, float[] sizing, float rho, float minConeDeg = 35f)
    {
        if (m.Nrm == null) m.ComputeNormals();
        var (start, nbr) = m.BuildAdjacency();
        int n = m.VertexCount;
        var score = new float[n];
        var hood = new int[n][];   // neighbourhoods, kept only for tip candidates
        // Per-thread scratch (visit stamps + BFS queue) instead of per-vertex collections: the old version allocated
        // a HashSet, Queue and List for each of ~200k vertices, which was GC-bound (4x slower on iOS NativeAOT).
        System.Threading.Tasks.Parallel.For(0, n,
            () => (stamp: new int[n], queue: new int[256]),
            (v, _, s) =>
            {
                float r = sizing != null ? sizing[v] : rho;
                float r2 = r * r;
                var nv = m.Nrm[v]; var pv = m.Pos[v];
                int mark = v + 1;
                s.stamp[v] = mark;
                var queue = s.queue;
                int head = 0, tail = 0;
                queue[tail++] = v;
                float dev = 0; int cnt = 0; bool extreme = true;
                while (head < tail)
                {
                    int u = queue[head++];
                    for (int k = start[u]; k < start[u + 1]; k++)
                    {
                        int w = nbr[k];
                        if (s.stamp[w] == mark) continue;
                        s.stamp[w] = mark;
                        var d = m.Pos[w] - pv;
                        if (d.LengthSquared() > r2) continue;
                        if (tail == queue.Length) { Array.Resize(ref queue, queue.Length * 2); s.queue = queue; }
                        queue[tail++] = w;
                        if (d.Dot(nv) > 1e-6f * r) extreme = false;
                        dev += 1f - Math.Clamp(m.Nrm[w].Dot(nv), -1f, 1f); cnt++;
                    }
                }
                if (!extreme || cnt < 6) return s;
                float meanDeg = Mathf.RadToDeg(MathF.Acos(Math.Clamp(1f - dev / cnt, -1f, 1f)));
                if (meanDeg >= minConeDeg)
                {
                    score[v] = meanDeg;
                    hood[v] = queue[1..tail];   // neighbours (excluding v itself), same order as before
                }
                return s;
            },
            _ => { });
        var tips = new List<int>();
        for (int v = 0; v < n; v++)
        {
            if (score[v] <= 0) continue;
            bool best = true;
            foreach (var w in hood[v]) if (score[w] > score[v] || (score[w] == score[v] && w < v)) { best = false; break; }
            if (best) tips.Add(v);
        }
        return tips;
    }
}
