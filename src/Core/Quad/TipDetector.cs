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
        var hood = new List<int>[n];
        System.Threading.Tasks.Parallel.For(0, n, v =>
        {
            float r = sizing != null ? sizing[v] : rho;
            var nv = m.Nrm[v]; var pv = m.Pos[v];
            // breadth-first neighbourhood within Euclidean radius r
            var seen = new HashSet<int> { v };
            var q = new Queue<int>(); q.Enqueue(v);
            var list = new List<int>();
            float dev = 0; int cnt = 0; bool extreme = true;
            while (q.Count > 0)
            {
                int u = q.Dequeue();
                for (int k = start[u]; k < start[u + 1]; k++)
                {
                    int w = nbr[k];
                    if (!seen.Add(w)) continue;
                    if ((m.Pos[w] - pv).LengthSquared() > r * r) continue;
                    q.Enqueue(w); list.Add(w);
                    if ((m.Pos[w] - pv).Dot(nv) > 1e-6f * r) { extreme = false; }
                    dev += 1f - Math.Clamp(m.Nrm[w].Dot(nv), -1f, 1f); cnt++;
                }
            }
            hood[v] = list;
            if (!extreme || cnt < 6) return;
            float meanDeg = Mathf.RadToDeg(MathF.Acos(Math.Clamp(1f - dev / cnt, -1f, 1f)));
            if (meanDeg >= minConeDeg) score[v] = meanDeg;
        });
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
