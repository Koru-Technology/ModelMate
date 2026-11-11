using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>
/// Sharp edges and corners of a dense base mesh, for hard-surface remeshing.
///
/// Per vertex, the normal voting tensor T = Σ A_f n_f n_fᵀ over the faces within radius r (Page et al. 2002) says
/// what the surface looks like at that scale: one dominant eigenvalue on a flat or gently curved patch, two along a
/// crease (the crease runs along the eigenvector of the smallest one), three at a corner. Measuring at a radius of a
/// few voxels lets edges that a sculpt or the voxel base mesh rounded over a few triangles still count as edges.
/// Creases are thinned to the vertices where the normal turn peaks across them (ridges). Ridge points and corners are
/// then sharpened: planes fitted to the source surface on each side are intersected, which restores the edge the
/// base mesh (and the sculpt) rounded off.
/// </summary>
public sealed class Features
{
    public const byte None = 0, Band = 1, Ridge = 2, Corner = 3;

    public byte[] Kind;        // per base vertex
    public Vector3[] Dir;      // crease direction (Band, Ridge)
    public Vector3[] Point;    // sharpened position (Ridge, Corner)
    public int Ridges, Corners, Bands, Sharpened;
    public double FlatArea, TotalArea;   // area of vertices with one dominant normal (no sharp edge nearby)
    public float CreaseLength;           // ≈ ridge count · mean edge length

    /// <param name="r">tensor radius (model units)</param>
    /// <param name="angleDeg">edges whose faces turn by more than this are sharp</param>
    /// <param name="maxShift">sharpening may move a point at most this far</param>
    public static Features Find(MeshData m, Bvh source, float r, float angleDeg, float maxShift)
    {
        int nv = m.VertexCount, nt = m.TriCount;
        if (m.Nrm == null) m.ComputeNormals();
        var (start, nbr) = m.BuildAdjacency();
        var fn = new Vector3[nt];
        var fa = new float[nt];
        var vt = new List<int>[nv];
        for (int v = 0; v < nv; v++) vt[v] = new List<int>(7);
        double total = 0;
        for (int t = 0; t < nt; t++)
        {
            var n = m.TriNormal(t);
            float l = n.Length();
            fa[t] = 0.5f * l;
            fn[t] = l > 1e-30f ? n / l : Vector3.Zero;
            total += fa[t];
            for (int k = 0; k < 3; k++) vt[m.Tris[3 * t + k]].Add(t);
        }

        var f = new Features { Kind = new byte[nv], Dir = new Vector3[nv], Point = new Vector3[nv], TotalArea = total };
        // eigen-analysis per vertex: λ1 ≥ λ2 ≥ λ3 (normalized by the trace), e3 = crease direction
        var l2 = new float[nv];
        var l3 = new float[nv];
        // two planes at angle φ with equal weight give λ2 = (1 − cos φ)/2; unequal shares and the vertex's own
        // position inside the band lower it, hence the margin
        float tau = 0.3f * 0.5f * (1 - MathF.Cos(Mathf.DegToRad(angleDeg)));
        var flatArea = new double[nv];
        Parallel.For(0, nv, () => new Ball(nv), (v, _, ball) =>
        {
            ball.Collect(m, start, nbr, v, r);
            double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0, w = 0;
            foreach (int u in ball.Members)
                foreach (int t in vt[u])
                {
                    var n = fn[t]; double a = fa[t];
                    xx += a * n.X * n.X; xy += a * n.X * n.Y; xz += a * n.X * n.Z;
                    yy += a * n.Y * n.Y; yz += a * n.Y * n.Z; zz += a * n.Z * n.Z; w += a;
                }
            if (w <= 0) return ball;
            var (ev, e3) = Eigen.Symmetric3(xx / w, xy / w, xz / w, yy / w, yz / w, zz / w);
            l2[v] = (float)ev.Y; l3[v] = (float)ev.Z;
            f.Dir[v] = e3;
            if (ev.Y < tau)
            {
                double va = 0;
                foreach (int t in vt[v]) va += fa[t] / 3;
                flatArea[v] = va;
            }
            return ball;
        }, _ => { });
        foreach (var a in flatArea) f.FlatArea += a;

        // classify: band where λ2 is significant; ridge where it peaks across the crease; corner where λ3 peaks
        Parallel.For(0, nv, () => new Ball(nv), (v, _, ball) =>
        {
            bool corner = l3[v] > tau, crease = l2[v] > tau;
            if (!corner && !crease) return ball;
            ball.Collect(m, start, nbr, v, r);
            var p = m.Pos[v];
            bool peak = true;
            if (corner)
            {
                foreach (int u in ball.Members)
                    if (u != v && (l3[u] > l3[v] || (l3[u] == l3[v] && u < v))) { peak = false; break; }
                f.Kind[v] = peak ? Corner : Band;
                return ball;
            }
            var d = f.Dir[v];
            foreach (int u in ball.Members)
            {
                if (u == v) continue;
                var off = m.Pos[u] - p;
                float ol = off.Length();
                if (ol < 1e-12f || MathF.Abs(off.Dot(d)) > 0.5f * ol) continue;   // only neighbours across the crease
                if (l2[u] > l2[v] || (l2[u] == l2[v] && u < v)) { peak = false; break; }
            }
            f.Kind[v] = peak ? Ridge : Band;
            return ball;
        }, _ => { });

        // sharpen ridges and corners against the source
        int sharpened = 0;
        Parallel.For(0, nv, () => new Ball(nv), (v, _, ball) =>
        {
            byte k = f.Kind[v];
            if (k != Ridge && k != Corner) return ball;
            f.Point[v] = m.Pos[v];
            ball.Collect(m, start, nbr, v, 1.5f * r);
            var pts = new List<(Vector3 p, Vector3 n)>(ball.Members.Count);
            foreach (int u in ball.Members)
                if (source.Closest(m.Pos[u], 2 * r, null, out var hit)) pts.Add((hit.Point, source.TriNormal[hit.Tri]));
            if (pts.Count < 6) return ball;
            var x = k == Ridge ? SharpenEdge(pts, m.Pos[v], m.Nrm[v], f.Dir[v], angleDeg, out var dir) : SharpenCorner(pts, m.Pos[v], angleDeg, out dir);
            if (x is Vector3 q && (q - m.Pos[v]).Length() <= maxShift)
            {
                f.Point[v] = q;
                if (k == Ridge) f.Dir[v] = dir;
                System.Threading.Interlocked.Increment(ref sharpened);
            }
            // a corner is where three planes meet in a point; three normal groups that don't (the top, side and
            // bottom of a thin ring: two of them parallel) are only edges close together
            else if (k == Corner) f.Kind[v] = Band;
            return ball;
        }, _ => { });
        f.Sharpened = sharpened;

        double edge = 0; int ne = 0;
        for (int v = 0; v < nv; v++) for (int k = start[v]; k < start[v + 1]; k++) { edge += (m.Pos[v] - m.Pos[nbr[k]]).Length(); ne++; }
        float meanEdge = (float)(edge / Math.Max(1, ne));
        foreach (var k in f.Kind) { if (k == Ridge) f.Ridges++; else if (k == Corner) f.Corners++; else if (k == Band) f.Bands++; }
        f.CreaseLength = f.Ridges * meanEdge;
        return f;
    }

    /// <summary>
    /// Local feature size per vertex (∞ where there is none): at a sharp edge, the distance across to the nearest
    /// parallel sharp edge within `radius` (the height of a step, the width of a ledge, the thickness of a plate).
    /// Edges closer than `minSep` count as the same edge; edges that aren't parallel (meeting at a corner) don't count.
    /// </summary>
    public float[] LocalFeatureSize(MeshData m, float radius, float minSep)
    {
        int nv = m.VertexCount;
        var lfs = new float[nv];
        Array.Fill(lfs, float.PositiveInfinity);
        var ridge = new List<int>();
        for (int v = 0; v < nv; v++) if (Kind[v] == Ridge) ridge.Add(v);
        if (ridge.Count == 0) return lfs;
        // uniform grid of ridge points, cell = radius
        var grid = new Dictionary<(int, int, int), List<int>>();
        (int, int, int) Cell(Vector3 p) => ((int)MathF.Floor(p.X / radius), (int)MathF.Floor(p.Y / radius), (int)MathF.Floor(p.Z / radius));
        foreach (int v in ridge)
        {
            var c = Cell(Point[v]);
            if (!grid.TryGetValue(c, out var l)) grid[c] = l = new List<int>();
            l.Add(v);
        }
        Parallel.ForEach(ridge, v =>
        {
            var p = Point[v]; var d = Dir[v];
            var (cx, cy, cz) = Cell(p);
            float best = float.PositiveInfinity;
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (!grid.TryGetValue((cx + dx, cy + dy, cz + dz), out var l)) continue;
                        foreach (int u in l)
                        {
                            if (MathF.Abs(Dir[u].Dot(d)) < 0.8f) continue;
                            var off = Point[u] - p;
                            var perp = off - d * d.Dot(off);
                            float dp = perp.Length();
                            if (dp < minSep || off.Length() > radius) continue;
                            best = MathF.Min(best, dp);
                        }
                    }
            lfs[v] = best;
        });
        return lfs;
    }

    public override string ToString() =>
        $"{Ridges} crease vertices (≈{CreaseLength:0.##} units), {Corners} corners, {Bands} band, {Sharpened} sharpened; flat {FlatArea / Math.Max(TotalArea, 1e-30):P0} of area";

    /// <summary>Two planes, one per side of the crease, fitted to source samples; their intersection line.</summary>
    static Vector3? SharpenEdge(List<(Vector3 p, Vector3 n)> pts, Vector3 at, Vector3 vn, Vector3 crease, float angleDeg, out Vector3 dir)
    {
        dir = crease;
        var perp = crease.Cross(vn);
        if (perp.LengthSquared() < 1e-12f) return null;
        perp = perp.Normalized();
        Vector3 n1 = default, n2 = default; float d1 = 0, d2 = 0; int c1 = 0, c2 = 0;
        foreach (var (p, n) in pts)
        {
            float s = n.Dot(perp);
            if (MathF.Abs(s) < 0.15f) continue;   // on the rounded part of the edge: belongs to neither plane
            if (s > 0) { n1 += n; d1 += n.Dot(p); c1++; } else { n2 += n; d2 += n.Dot(p); c2++; }
        }
        if (c1 < 3 || c2 < 3) return null;
        float l1 = n1.Length(), l2 = n2.Length();
        n1 /= l1; n2 /= l2; d1 = d1 / c1 / (l1 / c1); d2 = d2 / c2 / (l2 / c2);
        if (n1.Dot(n2) > MathF.Cos(Mathf.DegToRad(0.5f * angleDeg))) return null;
        var L = n1.Cross(n2);
        if (L.LengthSquared() < 1e-12f) return null;
        L = L.Normalized();
        dir = L.Dot(crease) < 0 ? -L : L;
        return Solve3(n1, d1, n2, d2, L, L.Dot(at));
    }

    /// <summary>Three planes from the source samples' normal clusters (farthest-point seeds); their intersection.</summary>
    static Vector3? SharpenCorner(List<(Vector3 p, Vector3 n)> pts, Vector3 at, float angleDeg, out Vector3 dir)
    {
        dir = Vector3.Zero;
        var s0 = pts[0].n;
        var s1 = s0; float best = 2;
        foreach (var (_, n) in pts) { float d = n.Dot(s0); if (d < best) { best = d; s1 = n; } }
        var s2 = s0; best = 2;
        foreach (var (_, n) in pts) { float d = MathF.Max(n.Dot(s0), n.Dot(s1)); if (d < best) { best = d; s2 = n; } }
        var seeds = new[] { s0, s1, s2 };
        var nn = new Vector3[3]; var dd = new float[3]; var cc = new int[3];
        for (int it = 0; it < 3; it++)
        {
            Array.Clear(nn); Array.Clear(dd); Array.Clear(cc);
            foreach (var (p, n) in pts)
            {
                int b = 0;
                for (int k = 1; k < 3; k++) if (n.Dot(seeds[k]) > n.Dot(seeds[b])) b = k;
                if (n.Dot(seeds[b]) < 0.9f) continue;   // rounded part: in no plane
                nn[b] += n; dd[b] += n.Dot(p); cc[b]++;
            }
            for (int k = 0; k < 3; k++) if (cc[k] > 0) seeds[k] = nn[k].Normalized();
        }
        float cosMin = MathF.Cos(Mathf.DegToRad(0.5f * angleDeg));
        for (int k = 0; k < 3; k++) if (cc[k] < 3) return null;
        for (int a = 0; a < 3; a++) for (int b = a + 1; b < 3; b++) if (seeds[a].Dot(seeds[b]) > cosMin) return null;
        for (int k = 0; k < 3; k++) dd[k] = dd[k] / cc[k] / (nn[k].Length() / cc[k]);
        return Solve3(seeds[0], dd[0], seeds[1], dd[1], seeds[2], dd[2]);
    }

    /// <summary>x with a·x = da, b·x = db, c·x = dc (Cramer); null when nearly singular.</summary>
    static Vector3? Solve3(Vector3 a, float da, Vector3 b, float db, Vector3 c, float dc)
    {
        float det = a.Dot(b.Cross(c));
        if (MathF.Abs(det) < 1e-4f) return null;
        return (da * b.Cross(c) + db * c.Cross(a) + dc * a.Cross(b)) / det;
    }

    /// <summary>Vertices within a Euclidean radius, grown over mesh edges (an approximate geodesic ball).</summary>
    sealed class Ball
    {
        public readonly List<int> Members = new(64);
        readonly int[] _stamp;
        int _cur;
        readonly Queue<int> _q = new();
        public Ball(int n) { _stamp = new int[n]; }

        public void Collect(MeshData m, int[] start, int[] nbr, int v, float r)
        {
            Members.Clear(); _q.Clear();
            _cur++;
            var c = m.Pos[v];
            float r2 = r * r;
            _stamp[v] = _cur; _q.Enqueue(v);
            while (_q.Count > 0)
            {
                int u = _q.Dequeue();
                Members.Add(u);
                for (int k = start[u]; k < start[u + 1]; k++)
                {
                    int w = nbr[k];
                    if (_stamp[w] == _cur) continue;
                    _stamp[w] = _cur;
                    if ((m.Pos[w] - c).LengthSquared() <= r2) _q.Enqueue(w);
                }
            }
        }
    }
}

/// <summary>Eigen-decomposition of a symmetric 3×3 matrix (cyclic Jacobi).</summary>
public static class Eigen
{
    /// <summary>Eigenvalues sorted descending, and the eigenvector of the smallest one.</summary>
    public static (Vector3 values, Vector3 smallest) Symmetric3(double a00, double a01, double a02, double a11, double a12, double a22)
    {
        var A = new double[3, 3] { { a00, a01, a02 }, { a01, a11, a12 }, { a02, a12, a22 } };
        var V = new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        for (int sweep = 0; sweep < 12; sweep++)
        {
            double off = A[0, 1] * A[0, 1] + A[0, 2] * A[0, 2] + A[1, 2] * A[1, 2];
            if (off < 1e-20) break;
            for (int p = 0; p < 2; p++)
                for (int q = p + 1; q < 3; q++)
                {
                    if (Math.Abs(A[p, q]) < 1e-30) continue;
                    double theta = 0.5 * (A[q, q] - A[p, p]) / A[p, q];
                    double t = Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                    if (theta == 0) t = 1;
                    double c = 1 / Math.Sqrt(t * t + 1), s = t * c;
                    for (int k = 0; k < 3; k++)
                    {
                        double akp = A[k, p], akq = A[k, q];
                        A[k, p] = c * akp - s * akq; A[k, q] = s * akp + c * akq;
                    }
                    for (int k = 0; k < 3; k++)
                    {
                        double apk = A[p, k], aqk = A[q, k];
                        A[p, k] = c * apk - s * aqk; A[q, k] = s * apk + c * aqk;
                    }
                    for (int k = 0; k < 3; k++)
                    {
                        double vkp = V[k, p], vkq = V[k, q];
                        V[k, p] = c * vkp - s * vkq; V[k, q] = s * vkp + c * vkq;
                    }
                }
        }
        var idx = new[] { 0, 1, 2 };
        Array.Sort(idx, (i, j) => A[j, j].CompareTo(A[i, i]));
        var vals = new Vector3((float)A[idx[0], idx[0]], (float)A[idx[1], idx[1]], (float)A[idx[2], idx[2]]);
        int sm = idx[2];
        var e = new Vector3((float)V[0, sm], (float)V[1, sm], (float)V[2, sm]);
        return (vals, e.LengthSquared() > 0 ? e.Normalized() : Vector3.Right);
    }
}
