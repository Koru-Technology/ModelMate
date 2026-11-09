using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

public struct CleanupReport
{
    public int DegenerateRemoved, DuplicateRemoved, NonManifoldEdges, VerticesSplit, BadEdgesAfter;
    public override string ToString() =>
        $"degenerate={DegenerateRemoved} duplicate={DuplicateRemoved} nonManifoldEdges={NonManifoldEdges} " +
        $"vertsSplit={VerticesSplit} edgesNot2Faces(after)={BadEdgesAfter}";
}

/// <summary>Removes degenerate/duplicate faces and splits non-manifold edges and vertices.</summary>
public static class MeshCleanup
{
    public static CleanupReport Clean(MeshData m)
    {
        var rep = new CleanupReport();
        RemoveDegenerate(m, ref rep);
        SplitNonManifold(m, ref rep);
        m.CompactVertices();
        rep.BadEdgesAfter = CountBadEdges(m);
        return rep;
    }

    /// <summary>
    /// Drops index-degenerate triangles and duplicates. Same-winding duplicates keep one copy; a pair with
    /// opposite winding is a zero-volume "pillow" (both sides of the same patch) and is removed entirely.
    /// </summary>
    static void RemoveDegenerate(MeshData m, ref CleanupReport rep)
    {
        var groups = new Dictionary<(int, int, int), List<int>>();
        var keep = new bool[m.TriCount];
        for (int t = 0; t < m.TriCount; t++)
        {
            int a = m.Tris[3 * t], b = m.Tris[3 * t + 1], c = m.Tris[3 * t + 2];
            if (a == b || b == c || a == c) { rep.DegenerateRemoved++; continue; }
            int x = Math.Min(a, Math.Min(b, c)), z = Math.Max(a, Math.Max(b, c)), y = a + b + c - x - z;
            if (!groups.TryGetValue((x, y, z), out var g)) groups[(x, y, z)] = g = new List<int>();
            g.Add(t);
            keep[t] = true;
        }
        foreach (var g in groups.Values)
        {
            if (g.Count < 2) continue;
            int pos = 0, neg = 0;
            int Orient(int t)
            {
                // canonical rotation starting at the smallest index; compare the next vertex
                int a = m.Tris[3 * t], b = m.Tris[3 * t + 1], c = m.Tris[3 * t + 2];
                if (a < b && a < c) return b < c ? 1 : -1;
                if (b < a && b < c) return c < a ? 1 : -1;
                return a < b ? 1 : -1;
            }
            foreach (var t in g) if (Orient(t) > 0) pos++; else neg++;
            int pairs = Math.Min(pos, neg);
            // remove opposite-winding pairs entirely, then collapse remaining same-winding copies to one
            int rp = pairs, rn = pairs;
            bool keptOne = false;
            foreach (var t in g)
            {
                int o = Orient(t);
                if (o > 0 && rp > 0) { keep[t] = false; rp--; rep.DuplicateRemoved++; continue; }
                if (o < 0 && rn > 0) { keep[t] = false; rn--; rep.DuplicateRemoved++; continue; }
                if (keptOne) { keep[t] = false; rep.DuplicateRemoved++; }
                else keptOne = true;
            }
        }
        var outTris = new List<int>(m.Tris.Length);
        var outFlags = m.TriFlags != null ? new List<byte>(m.TriCount) : null;
        for (int t = 0; t < m.TriCount; t++)
        {
            if (!keep[t]) continue;
            outTris.Add(m.Tris[3 * t]); outTris.Add(m.Tris[3 * t + 1]); outTris.Add(m.Tris[3 * t + 2]);
            outFlags?.Add(m.TriFlags[t]);
        }
        m.Tris = outTris.ToArray();
        if (outFlags != null) m.TriFlags = outFlags.ToArray();
    }

    static long EdgeKey(int a, int b) => a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

    /// <summary>
    /// Pairs half-edges (2-face edges directly; &gt;2-face edges by sorting faces around the edge axis),
    /// then splits every vertex into one copy per connected fan of paired faces.
    /// </summary>
    static void SplitNonManifold(MeshData m, ref CleanupReport rep)
    {
        int nh = m.Tris.Length;
        var keys = new long[nh];
        var hes = new int[nh];
        for (int h = 0; h < nh; h++)
        {
            int f = h / 3, s = h % 3;
            keys[h] = EdgeKey(m.Tris[3 * f + s], m.Tris[3 * f + (s + 1) % 3]);
            hes[h] = h;
        }
        Array.Sort(keys, hes);
        var partner = new int[nh];
        Array.Fill(partner, -1);

        int i = 0;
        var group = new List<int>();
        while (i < nh)
        {
            int j = i;
            while (j < nh && keys[j] == keys[i]) j++;
            int cnt = j - i;
            if (cnt == 2)
            {
                partner[hes[i]] = hes[i + 1];
                partner[hes[i + 1]] = hes[i];
            }
            else if (cnt > 2)
            {
                rep.NonManifoldEdges++;
                group.Clear();
                for (int k = i; k < j; k++) group.Add(hes[k]);
                PairAroundEdge(m, group, partner);
            }
            i = j;
        }

        // union corners (corner id == half-edge id: corner of vertex Tris[h])
        var uf = new int[nh];
        for (int h = 0; h < nh; h++) uf[h] = h;
        int Find(int x)
        {
            while (uf[x] != x) { uf[x] = uf[uf[x]]; x = uf[x]; }
            return x;
        }
        void Union(int a, int b)
        {
            a = Find(a); b = Find(b);
            if (a != b) uf[a] = b;
        }
        for (int h = 0; h < nh; h++)
        {
            int p = partner[h];
            if (p < h) continue;
            int f = h / 3, s = h % 3, g = p / 3, t = p % 3;
            int ha = 3 * f + s, hb = 3 * f + (s + 1) % 3;     // corners of a, b in f
            int pa = 3 * g + t, pb = 3 * g + (t + 1) % 3;     // corners of edge start/end in g
            if (m.Tris[pa] == m.Tris[ha]) { Union(ha, pa); Union(hb, pb); }
            else { Union(ha, pb); Union(hb, pa); }
        }

        var rootVert = new Dictionary<int, int>();
        var used = new bool[m.Pos.Length];
        var newPos = new List<Vector3>(m.Pos);
        var newTris = new int[nh];
        for (int h = 0; h < nh; h++)
        {
            int r = Find(h);
            if (!rootVert.TryGetValue(r, out int v))
            {
                int orig = m.Tris[h];
                if (!used[orig]) { used[orig] = true; v = orig; }
                else { v = newPos.Count; newPos.Add(m.Pos[orig]); rep.VerticesSplit++; }
                rootVert[r] = v;
            }
            newTris[h] = v;
        }
        m.Pos = newPos.ToArray();
        m.Tris = newTris;
        m.Nrm = null;
        m.Uv = null;
    }

    static void PairAroundEdge(MeshData m, List<int> group, int[] partner)
    {
        int h0 = group[0];
        int f0 = h0 / 3, s0 = h0 % 3;
        int a = m.Tris[3 * f0 + s0], b = m.Tris[3 * f0 + (s0 + 1) % 3];
        Vector3 pa = m.Pos[a], axis = (m.Pos[b] - pa).Normalized();
        // reference frame perpendicular to the edge
        var refv = MathF.Abs(axis.X) < 0.9f ? Vector3.Right : Vector3.Up;
        var u = (refv - axis * refv.Dot(axis)).Normalized();
        var w = axis.Cross(u);
        var items = new List<(float ang, int he, bool fwd)>();
        foreach (var h in group)
        {
            int f = h / 3, s = h % 3;
            int third = m.Tris[3 * f + (s + 2) % 3];
            var d = m.Pos[third] - pa;
            float ang = MathF.Atan2(d.Dot(w), d.Dot(u));
            items.Add((ang, h, m.Tris[h] == a));
        }
        items.Sort((x, y) => x.ang.CompareTo(y.ang));
        int n = items.Count;
        var done = new bool[n];
        // pair each a->b face with the next b->a face counter-clockwise around the edge
        for (int k = 0; k < n; k++)
        {
            if (done[k] || !items[k].fwd) continue;
            for (int step = 1; step < n; step++)
            {
                int j = (k + step) % n;
                if (done[j] || items[j].fwd) continue;
                partner[items[k].he] = items[j].he;
                partner[items[j].he] = items[k].he;
                done[k] = done[j] = true;
                break;
            }
        }
    }

    /// <summary>
    /// Closes boundary loops of up to maxEdges edges with a centroid fan (new vertex at the loop centroid).
    /// Returns (filled, skipped). Keeps TriFlags in sync (new triangles get no diagonal flags).
    /// </summary>
    public static (int filled, int skipped) FillHoles(MeshData m, int maxEdges)
    {
        // boundary half-edges: a→b present, b→a absent
        var half = new HashSet<long>(EdgeKeyComparer.Instance);
        for (int t = 0; t < m.TriCount; t++)
            for (int k = 0; k < 3; k++)
                half.Add(((long)m.Tris[3 * t + k] << 32) | (uint)m.Tris[3 * t + (k + 1) % 3]);
        var next = new Dictionary<int, List<int>>();   // boundary loop runs b→a (reverse of the face edge)
        for (int t = 0; t < m.TriCount; t++)
            for (int k = 0; k < 3; k++)
            {
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                if (half.Contains(((long)b << 32) | (uint)a)) continue;
                if (!next.TryGetValue(b, out var l)) next[b] = l = new List<int>();
                l.Add(a);
            }
        var pos = new List<Vector3>(m.Pos);
        var tris = new List<int>(m.Tris);
        var flags = m.TriFlags != null ? new List<byte>(m.TriFlags) : null;
        int filled = 0, skipped = 0;
        var visited = new HashSet<(int, int)>();
        foreach (var start in new List<int>(next.Keys))
        {
            foreach (var first in next[start])
            {
                if (visited.Contains((start, first))) continue;
                var loop = new List<int> { start };
                int cur = first, prev = start;
                visited.Add((start, first));
                bool closed = false;
                while (loop.Count <= maxEdges + 1)
                {
                    if (cur == start) { closed = true; break; }
                    loop.Add(cur);
                    if (!next.TryGetValue(cur, out var outs) || outs.Count == 0) break;
                    int nx = outs[0];
                    foreach (var o in outs) if (!visited.Contains((cur, o))) { nx = o; break; }
                    visited.Add((cur, nx));
                    prev = cur; cur = nx;
                }
                if (!closed || loop.Count > maxEdges || loop.Count < 3) { skipped++; continue; }
                var c = Vector3.Zero;
                foreach (var v in loop) c += m.Pos[v];
                int ci = pos.Count;
                pos.Add(c / loop.Count);
                // loop runs opposite to the existing faces' boundary edges, so (loop[i], loop[i+1], c) faces outward
                for (int i = 0; i < loop.Count; i++)
                {
                    tris.Add(loop[i]); tris.Add(loop[(i + 1) % loop.Count]); tris.Add(ci);
                    flags?.Add(0);
                }
                filled++;
            }
        }
        m.Pos = pos.ToArray();
        m.Tris = tris.ToArray();
        if (flags != null) m.TriFlags = flags.ToArray();
        m.Nrm = null; m.Uv = null;
        return (filled, skipped);
    }

    public static int CountBadEdges(MeshData m)
    {
        var counts = new Dictionary<long, int>(m.Tris.Length, EdgeKeyComparer.Instance);
        for (int t = 0; t < m.TriCount; t++)
            for (int s = 0; s < 3; s++)
            {
                long k = EdgeKey(m.Tris[3 * t + s], m.Tris[3 * t + (s + 1) % 3]);
                counts[k] = counts.TryGetValue(k, out int c) ? c + 1 : 1;
            }
        int bad = 0;
        foreach (var c in counts.Values) if (c != 2) bad++;
        return bad;
    }
}
