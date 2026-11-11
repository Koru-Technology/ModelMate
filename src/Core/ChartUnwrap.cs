using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Pure C# UV atlas, used where Godot's xatlas module is missing (it is compiled into the editor only, so exported
/// apps such as the iPad build lack <c>ArrayMesh.LightmapUnwrap</c>).
///
/// 1. Charts: region growing over triangle adjacency; a unit joins when its normal is within a cone of the chart's
///    running average normal. Both triangles of a quad (TriFlags diagonal) form one unit, so seams follow real edges.
/// 2. Each chart is projected onto the plane of its average normal (stretch ≤ 1/cos(cone)). Charts with flipped or
///    overlapping triangles are re-grown with a tighter cone, down to single units.
/// 3. Each chart is rotated to its minimum-area bounding rectangle and packed with a skyline packer at uniform texel
///    density with a gutter, shrinking the density until everything fits.
/// The texture baker samples the source per texel, so projection stretch only changes texel density, not correctness.
/// </summary>
public static class ChartUnwrap
{
    public struct Info { public int Charts, SplitVerts, Regrown; public float Fill, TexelsPerUnit; }

    static readonly float[] ConeDeg = { 50f, 35f, 20f };

    public static (MeshData split, int[] origin, Info info) Unwrap(MeshData m, int texSize, int padTexels = 4)
    {
        int nt = m.TriCount;
        var info = new Info();
        var fn = new Vector3[nt];
        var fa = new float[nt];
        double totalArea = 0;
        for (int t = 0; t < nt; t++)
        {
            var n = m.TriNormal(t);
            float l = n.Length();
            fa[t] = 0.5f * l;
            fn[t] = l > 1e-30f ? n / l : Vector3.Up;
            totalArea += fa[t];
        }

        // triangle adjacency across edges (manifold pairs only)
        var nbr = new int[3 * nt];
        Array.Fill(nbr, -1);
        var owner = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        for (int t = 0; t < nt; t++)
            for (int k = 0; k < 3; k++)
            {
                long key = Key(m.Tris[3 * t + k], m.Tris[3 * t + (k + 1) % 3]);
                if (!owner.TryGetValue(key, out int o)) owner[key] = 3 * t + k;
                else if (o >= 0) { nbr[o] = t; nbr[3 * t + k] = o / 3; owner[key] = -1; }
            }

        // units: quad pairs (shared flagged diagonal) or single triangles
        var unitOf = new int[nt];
        Array.Fill(unitOf, -1);
        var units = new List<int[]>();
        for (int t = 0; t < nt; t++)
        {
            if (unitOf[t] >= 0) continue;
            int partner = -1;
            if (m.TriFlags != null)
                for (int k = 0; k < 3; k++)
                    if ((m.TriFlags[t] & (1 << k)) != 0 && nbr[3 * t + k] is int o && o >= 0 && unitOf[o] < 0) { partner = o; break; }
            unitOf[t] = units.Count;
            if (partner >= 0) { unitOf[partner] = units.Count; units.Add(new[] { t, partner }); }
            else units.Add(new[] { t });
        }
        int nu = units.Count;
        var un = new Vector3[nu];
        var ua = new float[nu];
        for (int u = 0; u < nu; u++)
        {
            var ns = Vector3.Zero;
            foreach (int t in units[u]) { ns += fn[t] * fa[t]; ua[u] += fa[t]; }
            un[u] = ns.LengthSquared() > 1e-40f ? ns.Normalized() : fn[units[u][0]];
        }
        var unbr = new List<int>[nu];
        for (int u = 0; u < nu; u++)
        {
            var l = new List<int>(4);
            foreach (int t in units[u])
                for (int k = 0; k < 3; k++)
                {
                    int o = nbr[3 * t + k];
                    if (o >= 0 && unitOf[o] != u && !l.Contains(unitOf[o])) l.Add(unitOf[o]);
                }
            unbr[u] = l;
        }

        // rasterization density for the overlap test: ~2 samples per final texel
        float sEst = (float)Math.Sqrt(0.7 * texSize * (double)texSize / Math.Max(totalArea, 1e-30));

        // grow charts; invalid ones are re-grown tighter
        var charts = new List<(List<int> units, Vector3 n)>();
        var work = new Stack<(List<int> units, int level)>();
        var all = new List<int>(nu);
        for (int u = 0; u < nu; u++) all.Add(u);
        work.Push((all, 0));
        var chartOf = new int[nu];
        var allowed = new bool[nu];
        while (work.Count > 0)
        {
            var (set, level) = work.Pop();
            if (level >= ConeDeg.Length)
            {
                // last resort: single units, and single triangles when a folded quad still flips
                foreach (int u in set)
                {
                    var single = new List<int> { u };
                    if (Valid(m, units, single, un[u], fn, fa, 2 * sEst)) charts.Add((single, un[u]));
                    else foreach (int t in units[u]) charts.Add((new List<int> { -1 - t }, fn[t]));
                }
                continue;
            }
            foreach (int u in set) allowed[u] = true;
            var grown = Grow(set, allowed, chartOf, un, ua, unbr, MathF.Cos(Mathf.DegToRad(ConeDeg[level])));
            foreach (int u in set) allowed[u] = false;
            foreach (var (cu, cn) in grown)
            {
                if (cu.Count == 1 || Valid(m, units, cu, cn, fn, fa, 2 * sEst)) charts.Add((cu, cn));
                else { info.Regrown++; work.Push((cu, level + 1)); }
            }
        }

        // project, rotate to min-area rectangle, landscape orientation
        int nc = charts.Count;
        var chartTris = new List<int>[nc];
        var local = new Dictionary<int, Vector2>[nc];   // welded vertex -> local 2D coords (world units)
        var size = new Vector2[nc];
        for (int c = 0; c < nc; c++)
        {
            var tris = new List<int>();
            foreach (int u in charts[c].units)
                if (u < 0) tris.Add(-1 - u); else tris.AddRange(units[u]);
            chartTris[c] = tris;
            var (bu, bv) = Basis(charts[c].n);
            var pts = new Dictionary<int, Vector2>();
            foreach (int t in tris)
                for (int k = 0; k < 3; k++)
                {
                    int v = m.Tris[3 * t + k];
                    if (!pts.ContainsKey(v)) pts[v] = new Vector2(m.Pos[v].Dot(bu), m.Pos[v].Dot(bv));
                }
            float ang = MinAreaAngle(new List<Vector2>(pts.Values));
            float ca = MathF.Cos(ang), sa = MathF.Sin(ang);
            var mn = new Vector2(float.MaxValue, float.MaxValue);
            var mx = new Vector2(float.MinValue, float.MinValue);
            var rot = new Dictionary<int, Vector2>(pts.Count);
            foreach (var (v, p) in pts)
            {
                var q = new Vector2(ca * p.X - sa * p.Y, sa * p.X + ca * p.Y);
                rot[v] = q;
                mn = new Vector2(MathF.Min(mn.X, q.X), MathF.Min(mn.Y, q.Y));
                mx = new Vector2(MathF.Max(mx.X, q.X), MathF.Max(mx.Y, q.Y));
            }
            var sz = mx - mn;
            bool swap = sz.Y > sz.X;
            var fin = new Dictionary<int, Vector2>(rot.Count);
            foreach (var (v, q0) in rot)
            {
                var q = q0 - mn;
                // 90° rotation (keeps orientation): (x, y) -> (H - y, x)
                fin[v] = swap ? new Vector2(sz.Y - q.Y, q.X) : q;
            }
            local[c] = fin;
            size[c] = swap ? new Vector2(sz.Y, sz.X) : sz;
        }

        // pack: largest density that fits
        double rectArea = 0;
        for (int c = 0; c < nc; c++) rectArea += Math.Max(size[c].X * (double)size[c].Y, 1e-30);
        float s = (float)Math.Sqrt(0.85 * texSize * (double)texSize / rectArea);
        (int x, int y)[] place = null;
        for (int attempt = 0; attempt < 200 && (place = Pack(size, s, texSize, padTexels)) == null; attempt++) s *= 0.95f;
        if (place == null) throw new Exception($"ChartUnwrap: could not pack {nc} charts into {texSize}²");
        // grow back toward the largest density that still fits
        float lo = s, hi = s / 0.95f;
        for (int it = 0; it < 6; it++)
        {
            float mid = 0.5f * (lo + hi);
            var pm = Pack(size, mid, texSize, padTexels);
            if (pm != null) { lo = mid; place = pm; } else hi = mid;
        }
        s = lo;

        // emit split mesh (triangle order preserved)
        var triChart = new int[nt];
        for (int c = 0; c < nc; c++) foreach (int t in chartTris[c]) triChart[t] = c;
        var map = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        var pos = new List<Vector3>();
        var nrm = new List<Vector3>();
        var uv = new List<Vector2>();
        var origin = new List<int>();
        var outTris = new int[3 * nt];
        float half = padTexels * 0.5f;
        double uvArea = 0;
        for (int t = 0; t < nt; t++)
        {
            int c = triChart[t];
            for (int k = 0; k < 3; k++)
            {
                int v = m.Tris[3 * t + k];
                long key = ((long)c << 32) | (uint)v;
                if (!map.TryGetValue(key, out int idx))
                {
                    idx = pos.Count;
                    map[key] = idx;
                    pos.Add(m.Pos[v]);
                    nrm.Add(m.Nrm != null ? m.Nrm[v] : fn[t]);
                    var q = local[c][v];
                    uv.Add(new Vector2((place[c].x + half + q.X * s) / texSize, (place[c].y + half + q.Y * s) / texSize));
                    origin.Add(v);
                }
                outTris[3 * t + k] = idx;
            }
            uvArea += fa[t] * (double)s * s;
        }
        var split = new MeshData(pos.ToArray(), outTris) { Nrm = nrm.ToArray(), Uv = uv.ToArray() };
        info.Charts = nc;
        info.SplitVerts = pos.Count;
        info.TexelsPerUnit = s;
        info.Fill = (float)(uvArea / ((double)texSize * texSize));
        return (split, origin.ToArray(), info);
    }

    static long Key(int a, int b) => a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

    /// <summary>Tangent basis (u, v) with u × v = n, so counter-clockwise faces stay counter-clockwise in 2D.</summary>
    static (Vector3 u, Vector3 v) Basis(Vector3 n)
    {
        var a = MathF.Abs(n.X) < 0.9f ? Vector3.Right : Vector3.Up;
        var u = a.Cross(n).Normalized();
        u = n.Cross(u).Normalized();   // any unit vector perpendicular to n
        var v = n.Cross(u);
        return (u, v);
    }

    static List<(List<int> units, Vector3 n)> Grow(List<int> set, bool[] allowed, int[] chartOf, Vector3[] un, float[] ua,
        List<int>[] unbr, float cosCone)
    {
        foreach (int u in set) chartOf[u] = -1;
        var result = new List<(List<int>, Vector3)>();
        var pq = new PriorityQueue<int, float>();
        // seeds: largest units first, so charts start on broad faces
        var order = new List<int>(set);
        order.Sort((a, b) => ua[b].CompareTo(ua[a]));
        foreach (int seed in order)
        {
            if (chartOf[seed] >= 0) continue;
            int id = result.Count;
            var members = new List<int> { seed };
            chartOf[seed] = id;
            var sum = un[seed] * ua[seed];
            var avg = un[seed];
            pq.Clear();
            foreach (int o in unbr[seed]) if (allowed[o] && chartOf[o] < 0) pq.Enqueue(o, 1 - un[o].Dot(avg));
            while (pq.TryDequeue(out int u, out _))
            {
                if (chartOf[u] >= 0) continue;
                if (un[u].Dot(avg) < cosCone) continue;
                chartOf[u] = id;
                members.Add(u);
                sum += un[u] * ua[u];
                if (sum.LengthSquared() > 1e-40f) avg = sum.Normalized();
                foreach (int o in unbr[u]) if (allowed[o] && chartOf[o] < 0) pq.Enqueue(o, 1 - un[o].Dot(avg));
            }
            result.Add((members, avg));
        }
        return result;
    }

    /// <summary>Chart projects without flipped or overlapping triangles (rasterized at the given density).</summary>
    static bool Valid(MeshData m, List<int[]> units, List<int> cu, Vector3 n, Vector3[] fn, float[] fa, float density)
    {
        var (bu, bv) = Basis(n);
        var tris = new List<int>();
        foreach (int u in cu) tris.AddRange(units[u]);
        var p = new Vector2[3 * tris.Count];
        var mn = new Vector2(float.MaxValue, float.MaxValue);
        var mx = new Vector2(float.MinValue, float.MinValue);
        for (int i = 0; i < tris.Count; i++)
        {
            int t = tris[i];
            if (fn[t].Dot(n) <= 0.05f && fa[t] > 0) return false;   // nearly edge-on or back-facing
            for (int k = 0; k < 3; k++)
            {
                var w = m.Pos[m.Tris[3 * t + k]];
                var q = new Vector2(w.Dot(bu), w.Dot(bv));
                p[3 * i + k] = q;
                mn = new Vector2(MathF.Min(mn.X, q.X), MathF.Min(mn.Y, q.Y));
                mx = new Vector2(MathF.Max(mx.X, q.X), MathF.Max(mx.Y, q.Y));
            }
        }
        var ext = mx - mn;
        float d = density;
        float maxDim = MathF.Max(ext.X, ext.Y) * d;
        if (maxDim > 1024) d *= 1024 / maxDim;
        int gw = Math.Max(1, (int)MathF.Ceiling(ext.X * d) + 1), gh = Math.Max(1, (int)MathF.Ceiling(ext.Y * d) + 1);
        var grid = new bool[gw * gh];
        for (int i = 0; i < tris.Count; i++)
        {
            var a = (p[3 * i] - mn) * d; var b = (p[3 * i + 1] - mn) * d; var c = (p[3 * i + 2] - mn) * d;
            float area = Cross(b - a, c - a);
            if (MathF.Abs(area) < 1e-12f) continue;
            if (area < 0) return false;
            if (Raster(a, b, c, area, gw, gh, grid)) return false;
        }
        return true;
    }

    static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

    /// <summary>Marks cell centres strictly inside a counter-clockwise triangle; true if any was already marked.</summary>
    static bool Raster(Vector2 a, Vector2 b, Vector2 c, float area, int gw, int gh, bool[] grid)
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

    /// <summary>Rotation angle that minimizes the axis-aligned bounding box of the points (rotating calipers).</summary>
    static float MinAreaAngle(List<Vector2> pts)
    {
        var hull = ConvexHull(pts);
        if (hull.Count < 3) return 0;
        float best = float.MaxValue, bestAng = 0;
        for (int i = 0; i < hull.Count; i++)
        {
            var e = hull[(i + 1) % hull.Count] - hull[i];
            if (e.LengthSquared() < 1e-30f) continue;
            float ang = -MathF.Atan2(e.Y, e.X);   // rotate edge onto +X
            float ca = MathF.Cos(ang), sa = MathF.Sin(ang);
            float mnx = float.MaxValue, mny = float.MaxValue, mxx = float.MinValue, mxy = float.MinValue;
            foreach (var p in hull)
            {
                float x = ca * p.X - sa * p.Y, y = sa * p.X + ca * p.Y;
                mnx = MathF.Min(mnx, x); mxx = MathF.Max(mxx, x); mny = MathF.Min(mny, y); mxy = MathF.Max(mxy, y);
            }
            float a = (mxx - mnx) * (mxy - mny);
            if (a < best) { best = a; bestAng = ang; }
        }
        return bestAng;
    }

    static List<Vector2> ConvexHull(List<Vector2> pts)
    {
        pts.Sort((a, b) => a.X != b.X ? a.X.CompareTo(b.X) : a.Y.CompareTo(b.Y));
        var h = new List<Vector2>();
        for (int pass = 0; pass < 2; pass++)
        {
            int start = h.Count;
            for (int i = 0; i < pts.Count; i++)
            {
                var p = pass == 0 ? pts[i] : pts[pts.Count - 1 - i];
                while (h.Count >= start + 2 && Cross(h[^1] - h[^2], p - h[^2]) <= 0) h.RemoveAt(h.Count - 1);
                h.Add(p);
            }
            h.RemoveAt(h.Count - 1);
        }
        return h;
    }

    /// <summary>Skyline bottom-left packing of the chart rectangles at density s; null if they don't fit.</summary>
    static (int x, int y)[] Pack(Vector2[] size, float s, int T, int pad)
    {
        int n = size.Length;
        var w = new int[n]; var h = new int[n];
        var order = new int[n];
        for (int i = 0; i < n; i++)
        {
            w[i] = Math.Max(1, (int)MathF.Ceiling(size[i].X * s)) + pad;
            h[i] = Math.Max(1, (int)MathF.Ceiling(size[i].Y * s)) + pad;
            if (w[i] > T || h[i] > T) return null;
            order[i] = i;
        }
        Array.Sort(order, (a, b) => h[a] != h[b] ? h[b].CompareTo(h[a]) : w[b].CompareTo(w[a]));
        var segs = new List<(int x, int y, int w)> { (0, 0, T) };
        var res = new (int, int)[n];
        foreach (int r in order)
        {
            int bestI = -1, bestY = int.MaxValue, bestW = int.MaxValue;
            for (int i = 0; i < segs.Count; i++)
            {
                int x = segs[i].x;
                if (x + w[r] > T) break;
                int y = 0, rem = w[r];
                for (int j = i; rem > 0; j++) { y = Math.Max(y, segs[j].y); rem -= segs[j].w; }
                if (y + h[r] > T) continue;
                if (y < bestY || (y == bestY && segs[i].w < bestW)) { bestI = i; bestY = y; bestW = segs[i].w; }
            }
            if (bestI < 0) return null;
            int bx = segs[bestI].x;
            res[r] = (bx, bestY);
            // replace covered span [bx, bx + w) with the new segment
            int end = bx + w[r];
            var ns = new List<(int x, int y, int w)>(segs.Count + 2);
            for (int i = 0; i < segs.Count; i++)
            {
                var sg = segs[i];
                int se = sg.x + sg.w;
                if (se <= bx || sg.x >= end) { ns.Add(sg); continue; }
                if (sg.x < bx) ns.Add((sg.x, sg.y, bx - sg.x));
                if (sg.x <= bx) ns.Add((bx, bestY + h[r], w[r]));
                if (se > end) ns.Add((end, sg.y, se - end));
            }
            // merge equal-height neighbours
            segs = new List<(int x, int y, int w)>(ns.Count);
            foreach (var sg in ns)
                if (segs.Count > 0 && segs[^1].y == sg.y) segs[^1] = (segs[^1].x, sg.y, segs[^1].w + sg.w);
                else segs.Add(sg);
        }
        return res;
    }

    // ------------------------------------------------------------------ layout checks

    /// <summary>
    /// Checks any UV layout: inside [0,1], no texel claimed by two triangles, no triangle flipped against its chart,
    /// gutter of at least 2 texels between charts (charts = UV-connected components), and area stretch.
    /// </summary>
    public static void CheckLayout(CheckList cl, string label, MeshData split, int texSize, bool strict)
    {
        int nt = split.TriCount;
        // charts: union-find over shared split vertices
        var parent = new int[split.VertexCount];
        for (int i = 0; i < parent.Length; i++) parent[i] = i;
        int Find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
        for (int t = 0; t < nt; t++)
        {
            int a = Find(split.Tris[3 * t]);
            for (int k = 1; k < 3; k++) { int b = Find(split.Tris[3 * t + k]); if (a != b) parent[b] = a; }
        }
        var chartId = new int[nt];
        var chartSign = new Dictionary<int, double>();
        var uvA = new float[nt];
        for (int t = 0; t < nt; t++)
        {
            chartId[t] = Find(split.Tris[3 * t]);
            var a = split.Uv[split.Tris[3 * t]] * texSize; var b = split.Uv[split.Tris[3 * t + 1]] * texSize; var c = split.Uv[split.Tris[3 * t + 2]] * texSize;
            uvA[t] = 0.5f * Cross(b - a, c - a);
            chartSign[chartId[t]] = (chartSign.TryGetValue(chartId[t], out var s0) ? s0 : 0) + uvA[t];
        }
        int flipped = 0;
        for (int t = 0; t < nt; t++)
            if (MathF.Abs(uvA[t]) > 1e-6f && Math.Sign(uvA[t]) != Math.Sign(chartSign[chartId[t]])) flipped++;

        // rasterize chart ids at texture resolution
        var owner = new int[texSize * texSize];
        Array.Fill(owner, -1);
        int overlap = 0;
        for (int t = 0; t < nt; t++)
        {
            var a = split.Uv[split.Tris[3 * t]] * texSize; var b = split.Uv[split.Tris[3 * t + 1]] * texSize; var c = split.Uv[split.Tris[3 * t + 2]] * texSize;
            if (uvA[t] < 0) (b, c) = (c, b);
            float area = MathF.Abs(2 * uvA[t]);
            if (area < 1e-9f) continue;
            float eps = 1e-4f * area;
            int x0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X))));
            int x1 = Math.Min(texSize - 1, (int)MathF.Ceiling(MathF.Max(a.X, MathF.Max(b.X, c.X))));
            int y0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.Y, MathF.Min(b.Y, c.Y))));
            int y1 = Math.Min(texSize - 1, (int)MathF.Ceiling(MathF.Max(a.Y, MathF.Max(b.Y, c.Y))));
            for (int y = y0; y <= y1; y++)
                for (int x = x0; x <= x1; x++)
                {
                    var q = new Vector2(x + 0.5f, y + 0.5f);
                    if (Cross(b - a, q - a) <= eps || Cross(c - b, q - b) <= eps || Cross(a - c, q - c) <= eps) continue;
                    int i = y * texSize + x;
                    if (owner[i] >= 0) overlap++;
                    owner[i] = chartId[t];
                }
        }
        int touching = 0, covered = 0;
        for (int y = 0; y < texSize; y++)
            for (int x = 0; x < texSize; x++)
            {
                int o = owner[y * texSize + x];
                if (o < 0) continue;
                covered++;
                bool touch = false;
                for (int dy = -1; dy <= 1 && !touch; dy++)
                    for (int dx = -1; dx <= 1 && !touch; dx++)
                    {
                        int xx = x + dx, yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= texSize || yy >= texSize) continue;
                        int p = owner[yy * texSize + xx];
                        if (p >= 0 && p != o) touch = true;
                    }
                if (touch) touching++;
            }

        // area stretch relative to the global texel density
        double sum3 = 0, sumUv = 0;
        var area3 = new float[nt];
        for (int t = 0; t < nt; t++) { area3[t] = 0.5f * split.TriNormal(t).Length(); sum3 += area3[t]; sumUv += MathF.Abs(uvA[t]); }
        double g = sumUv / Math.Max(sum3, 1e-30);
        var st = new List<float>();
        for (int t = 0; t < nt; t++)
        {
            if (area3[t] <= 0) continue;
            float r = (float)(MathF.Abs(uvA[t]) / (area3[t] * g));
            st.Add(r > 1e-6f ? MathF.Max(r, 1 / r) : 1e6f);
        }
        st.Sort();
        float p95 = st.Count > 0 ? st[(int)(0.95f * (st.Count - 1))] : 1;

        cl.Check(flipped == 0, $"{label}: no UV triangles flipped against their chart ({flipped})");
        if (strict)
        {
            cl.Check(overlap == 0, $"{label}: no texel claimed by two triangles at {texSize}² ({overlap})");
            cl.Check(touching == 0, $"{label}: charts at least 2 texels apart ({touching} touching texels)");
            cl.Check(p95 <= 1.6f, $"{label}: UV area stretch p95 {p95:F2} <= 1.6");
        }
        // xatlas layouts (editor builds) are reported, not gated: it occasionally packs a few texels of overlap
        else cl.Metric($"{label}: {overlap} texels claimed twice, {touching} texels touching another chart, area stretch p95 {p95:F2}");
        cl.Metric($"{label}: {chartSign.Count} charts, {split.VertexCount} split verts, {covered / (float)(texSize * texSize):P0} of texture covered");
    }
}
