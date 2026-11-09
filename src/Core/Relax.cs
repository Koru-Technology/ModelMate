using System;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core;

/// <summary>Tangential Laplacian smoothing with guarded reprojection onto the source surface.</summary>
public static class Relax
{
    public struct Stats { public int Snapped, Unsnapped, Reverted; }

    /// <param name="adjacency">Optional smoothing neighbourhood (e.g. real quad edges only, so diagonals don't skew quads).</param>
    public static Stats Run(MeshData m, Bvh src, float maxDist, int iterations, float lambda = 0.5f,
        (int[] start, int[] nbr)? adjacency = null)
    {
        var stats = new Stats();
        var (start, nbr) = adjacency ?? m.BuildAdjacency();
        int nv = m.VertexCount;
        var snappedFlag = new bool[nv];
        for (int it = 0; it < iterations; it++)
        {
            m.ComputeNormals();
            var nrm = m.Nrm;
            var old = (Vector3[])m.Pos.Clone();
            var next = new Vector3[nv];

            // 1) tangential smoothing
            Parallel.For(0, nv, v =>
            {
                int s = start[v], e = start[v + 1];
                if (e == s) { next[v] = old[v]; return; }
                Vector3 c = Vector3.Zero;
                for (int k = s; k < e; k++) c += old[nbr[k]];
                c /= e - s;
                var d = c - old[v];
                d -= nrm[v] * d.Dot(nrm[v]);
                next[v] = old[v] + d * lambda;
            });

            // 2) guarded snap: only onto source triangles facing the same way, within maxDist
            Parallel.For(0, nv, v =>
            {
                var n = nrm[v];
                if (src.Closest(next[v], maxDist, t => src.TriNormal[t].Dot(n) > 0.3f, out var hit))
                {
                    next[v] = hit.Point;
                    snappedFlag[v] = true;
                }
                else snappedFlag[v] = false;
            });
            m.Pos = next;

            // 3) undo moves that flipped a triangle (repeat until stable)
            int reverted = 0;
            for (int pass = 0; pass < 4; pass++)
            {
                int flips = 0;
                for (int t = 0; t < m.TriCount; t++)
                {
                    int a = m.Tris[3 * t], b = m.Tris[3 * t + 1], c = m.Tris[3 * t + 2];
                    var nNew = (m.Pos[b] - m.Pos[a]).Cross(m.Pos[c] - m.Pos[a]);
                    var nOld = (old[b] - old[a]).Cross(old[c] - old[a]);
                    if (nNew.Dot(nOld) <= 0)
                    {
                        if (m.Pos[a] != old[a]) { m.Pos[a] = old[a]; flips++; }
                        if (m.Pos[b] != old[b]) { m.Pos[b] = old[b]; flips++; }
                        if (m.Pos[c] != old[c]) { m.Pos[c] = old[c]; flips++; }
                    }
                }
                reverted += flips;
                if (flips == 0) break;
            }
            stats.Reverted += reverted;
        }
        foreach (var f in snappedFlag) { if (f) stats.Snapped++; else stats.Unsnapped++; }
        m.ComputeNormals();
        return stats;
    }
}
