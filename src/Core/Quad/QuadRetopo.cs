using System;
using System.Collections.Generic;
using System.Diagnostics;
using Godot;

namespace ModelMate.Core.Quad;

public sealed class QuadResult
{
    public MeshData Mesh;            // triangulated quad-dominant mesh with TriFlags (diagonals)
    public QuadStats Stats;
    public Extraction.Report Extract;
    public CleanupReport Cleanup;
    public (int filled, int skipped) Holes;
    public Relax.Stats Reproject;
    public float Rho;
    public int OrientationSingularities;
    public float[] Sizing;           // per base vertex target edge length (null = uniform Rho)
    public int Tips;                 // sharp tips pinned
    public int Locked;               // hard surface: output vertices held on corners and sharp edges
    public int CreaseSplits;         // hard surface: quads split along a sharp edge
    public Extraction.Lock?[] Locks; // per output vertex (hard surface), or null
    public string Failure;           // null when the result passes the structural gates
}

/// <summary>
/// Field-aligned quad remeshing (Instant Meshes style) of a clean, dense base mesh, with optional
/// soft orientation guidance (per base-vertex direction + weight in [0,1]).
/// </summary>
public static class QuadRetopo
{
    /// <summary>Port of compute_orientation_singularities_impl (rosy = 4) on the base mesh faces.</summary>
    public static int OrientationSingularities(MeshData m, Level l)
    {
        int count = 0;
        for (int f = 0; f < m.TriCount; f++)
        {
            int index = 0;
            for (int k = 0; k < 3; k++)
            {
                int i = m.Tris[3 * f + k], j = m.Tris[3 * f + (k + 1) % 3];
                var (a, b) = FieldMath.CompatOrientIndex4(l.Q[i], l.N[i], l.Q[j], l.N[j]);
                index += b - a;
            }
            index = ((index % 4) + 4) % 4;
            if (index == 1 || index == 3) count++;
        }
        return count;
    }

    static void DebugEdges(MeshData m, string stage, Action<string> log)
    {
        var cnt = new System.Collections.Generic.Dictionary<long, int>(EdgeKeyComparer.Instance);
        for (int t = 0; t < m.TriCount; t++)
            for (int k = 0; k < 3; k++)
            {
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                cnt[key] = cnt.TryGetValue(key, out int c) ? c + 1 : 1;
            }
        int shown = 0;
        foreach (var kv in cnt)
            if (kv.Value != 2 && shown++ < 12)
            {
                int a = (int)(kv.Key >> 32), b = (int)(kv.Key & 0xffffffff);
                log?.Invoke($"    [{stage}] edge {a}-{b} faces={kv.Value} at {(m.Pos[a] + m.Pos[b]) * 0.5f}");
            }
    }

    /// <summary>99th-percentile distance from the dense surface to a remesh (sampled), in model units.</summary>
    public static float SurfaceP99(MeshData dense, MeshData remesh)
    {
        var bvh = new Bvh(remesh.Pos, remesh.Tris);
        int stride = Math.Max(1, dense.VertexCount / 20000);
        int n = (dense.VertexCount + stride - 1) / stride;
        var d = new float[n];
        System.Threading.Tasks.Parallel.For(0, n, k =>
        {
            var p = dense.Pos[k * stride];
            d[k] = bvh.Closest(p, float.MaxValue, null, out var hit) ? hit.Dist : 0;
        });
        Array.Sort(d);
        return d[Math.Min(n - 1, (int)(0.99 * n))];
    }

    /// <summary>Least-squares fit of the remesh to the dense surface after reprojection (see SurfaceFit).</summary>
    public static bool FitToSurface = true;
    /// <summary>Pin a lattice point onto each detected sharp tip (horn tips, noses) so they aren't cut short.</summary>
    public static bool PinTips = true;

    /// <param name="features">hard-surface mode: sharp edges and corners the field aligns to and the extraction keeps</param>
    /// <summary>
    /// A quad whose two opposite corners both sit on the same sharp edge must bend along that diagonal: split it
    /// there (two triangles, the crease a real edge), whichever diagonal the triangulation picked.
    /// </summary>
    static int SplitAlongCreases(MeshData m, Extraction.Lock?[] locks, float rho)
    {
        if (m.TriFlags == null) return 0;
        bool OnSameEdge(int a, int b)
        {
            if (locks[a] is not Extraction.Lock la || locks[b] is not Extraction.Lock lb) return false;
            var ab = m.Pos[b] - m.Pos[a];
            float l = ab.Length();
            if (l < 1e-12f) return false;
            ab /= l;
            // each end on a sharp edge (or a corner) and the diagonal along that edge's direction
            bool alongA = la.Dir == Vector3.Zero || MathF.Abs(ab.Dot(la.Dir)) > 0.9f;
            bool alongB = lb.Dir == Vector3.Zero || MathF.Abs(ab.Dot(lb.Dir)) > 0.9f;
            return alongA && alongB && !(la.Dir == Vector3.Zero && lb.Dir == Vector3.Zero);
        }
        var owner = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        int nt = m.TriCount, splits = 0;
        var edges = new HashSet<long>(EdgeKeyComparer.Instance);
        long Key(int x, int y) => x < y ? ((long)x << 32) | (uint)y : ((long)y << 32) | (uint)x;
        for (int t = 0; t < nt; t++)
            for (int k = 0; k < 3; k++) edges.Add(Key(m.Tris[3 * t + k], m.Tris[3 * t + (k + 1) % 3]));
        for (int t = 0; t < nt; t++)
            for (int k = 0; k < 3; k++)
            {
                if ((m.TriFlags[t] & (1 << k)) == 0) continue;
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                if (!owner.TryGetValue(key, out int t1)) { owner[key] = t; continue; }
                // t1 and t form a quad split along a–b; c and d are the other two corners
                int c = -1, d = -1;
                for (int q = 0; q < 3; q++) { int x = m.Tris[3 * t1 + q]; if (x != a && x != b) c = x; }
                for (int q = 0; q < 3; q++) { int x = m.Tris[3 * t + q]; if (x != a && x != b) d = x; }
                if (c < 0 || d < 0) continue;
                if (OnSameEdge(a, b))
                {
                    // the crease already runs along the split: make it a real edge
                    m.TriFlags[t] = 0; m.TriFlags[t1] = 0; splits++;
                }
                else if (OnSameEdge(c, d) && !edges.Contains(Key(c, d)))   // an existing c–d edge would become non-manifold
                {
                    // re-split along c–d, keeping the winding: quad loop is the union of both triangles' edges
                    int[] T1 = { m.Tris[3 * t1], m.Tris[3 * t1 + 1], m.Tris[3 * t1 + 2] };
                    // rotate T1 so it reads (u, v, c) with u→v the shared edge in t1's orientation
                    while (T1[2] != c) (T1[0], T1[1], T1[2]) = (T1[1], T1[2], T1[0]);
                    int u = T1[0], v = T1[1];   // t1 = (u, v, c); t = (v, u, d)
                    m.Tris[3 * t1] = u; m.Tris[3 * t1 + 1] = d; m.Tris[3 * t1 + 2] = c;
                    m.Tris[3 * t] = d; m.Tris[3 * t + 1] = v; m.Tris[3 * t + 2] = c;
                    m.TriFlags[t] = 0; m.TriFlags[t1] = 0; splits++;
                    edges.Remove(key); edges.Add(Key(c, d));
                }
            }
        return splits;
    }

    public static QuadResult Run(MeshData baseMesh, Bvh source, float rho, Vector3[] guideDir, float[] guideW,
        int seed, Action<string> log, float[] sizing = null, List<int> tips = null, Features features = null)
    {
        var r = new QuadResult { Rho = rho, Sizing = sizing };
        var sw = Stopwatch.StartNew();
        void T(string what) { log?.Invoke($"  [quad:{what}] {sw.ElapsedMilliseconds} ms"); sw.Restart(); }

        if (baseMesh.Nrm == null) baseMesh.ComputeNormals();
        var h = Hierarchy.Build(baseMesh, rho, seed, sizing);
        T($"hierarchy ({h.Levels.Count} levels, {h.Fine.Count} verts)");
        h.ResetSolution(seed);
        if (PinTips)
        {
            tips ??= TipDetector.Find(baseMesh, sizing, rho);
            if (tips.Count > 0)
            {
                var f0 = h.Fine;
                f0.CO = new Vector3[f0.Count]; f0.COw = new float[f0.Count];
                foreach (var t in tips) { f0.CO[t] = baseMesh.Pos[t]; f0.COw[t] = 1f; }
                h.PropagatePositionConstraints();
            }
            r.Tips = tips.Count;
            log?.Invoke($"  Tips pinned: {tips.Count}");
        }
        if (guideDir != null && guideW != null)
        {
            var f = h.Fine;
            for (int i = 0; i < f.Count; i++)
            {
                var n = f.N[i];
                var d = guideDir[i] - n * n.Dot(guideDir[i]);
                float w = guideW[i];
                if (d.LengthSquared() < 1e-12f || w <= 0) { f.CQ[i] = Vector3.Zero; f.CQw[i] = 0; continue; }
                f.CQ[i] = d.Normalized();
                f.CQw[i] = Math.Clamp(w, 0f, 1f);
            }
            h.PropagateConstraints();
        }
        if (features != null)
        {
            // edges: the field runs along them (orientation) and a lattice line lies on them (line constraint);
            // corners: a lattice point sits on them (point constraint, like the tips)
            var f0 = h.Fine;
            f0.CO ??= new Vector3[f0.Count]; f0.COw ??= new float[f0.Count]; f0.CD ??= new Vector3[f0.Count];
            for (int i = 0; i < f0.Count; i++)
            {
                byte k = features.Kind[i];
                if (k == Features.Band || k == Features.Ridge)
                {
                    var n = f0.N[i];
                    var d = features.Dir[i] - n * n.Dot(features.Dir[i]);
                    if (d.LengthSquared() > 1e-12f) { f0.CQ[i] = d.Normalized(); f0.CQw[i] = 1; }
                }
                if (k == Features.Ridge) { f0.CO[i] = features.Point[i]; f0.COw[i] = 1; f0.CD[i] = features.Dir[i]; }
                else if (k == Features.Corner) { f0.CO[i] = features.Point[i]; f0.COw[i] = 1; f0.CD[i] = Vector3.Zero; }
            }
            h.PropagateConstraints();
            h.PropagatePositionConstraints();
            f0.Features = features;
        }
        FieldSolver.SolveOrientations(h);
        r.OrientationSingularities = OrientationSingularities(baseMesh, h.Fine);
        T($"orientation field ({r.OrientationSingularities} singularities)");
        FieldSolver.SolvePositions(h);
        T("position field");

        var pm = Extraction.Extract(h, out r.Extract);
        T("extraction");
        log?.Invoke($"  Extraction: {r.Extract}");

        var m = pm.Triangulate();
        r.Cleanup = MeshCleanup.Clean(m);
        r.Holes = MeshCleanup.FillHoles(m, 12);
        if (r.Holes.filled > 0)
        {
            var c2 = MeshCleanup.Clean(m);
            r.Cleanup.NonManifoldEdges += c2.NonManifoldEdges;
            r.Cleanup.VerticesSplit += c2.VerticesSplit;
            r.Cleanup.BadEdgesAfter = c2.BadEdgesAfter;
        }
        m.ComputeNormals();
        Extraction.Lock?[] locks = null;
        bool[] locked = null;
        if (pm.Locks != null && pm.Locks.Count > 0)
        {
            locks = new Extraction.Lock?[m.VertexCount];
            locked = new bool[m.VertexCount];
            for (int v = 0; v < m.VertexCount; v++)
                if (pm.Locks.TryGetValue(m.Pos[v], out var lk)) { locks[v] = lk; locked[v] = true; r.Locked++; }
        }
        r.Locks = locks;
        if (locks != null) r.CreaseSplits = SplitAlongCreases(m, locks, rho);
        float snap = rho;
        if (sizing != null) foreach (var x in sizing) snap = MathF.Max(snap, x);
        r.Reproject = Relax.Run(m, source, snap, 3, 0.1f, QuadStats.RealAdjacency(m), locks);
        T("cleanup+reproject");
        if (FitToSurface)
        {
            var fs = SurfaceFit.Fit(m, baseMesh, locked: locked);
            T($"surface fit (mean shift {fs.MeanShift:0.#####}, max {fs.MaxShift:0.#####}, reverted {fs.Reverted})");
        }
        r.Mesh = m;
        r.Stats = QuadStats.Compute(m);
        log?.Invoke($"  Quad mesh: {r.Stats}");
        if (r.Locks != null) log?.Invoke($"  Hard surface: {r.Locked} vertices held on corners and sharp edges, {r.CreaseSplits} quads split along a sharp edge");

        if (r.Cleanup.BadEdgesAfter > 0 || r.Stats.BoundaryEdges > 0)
            r.Failure = $"not closed/manifold (bad edges {r.Cleanup.BadEdgesAfter}, boundary {r.Stats.BoundaryEdges}, holes skipped {r.Holes.skipped})";
        else if (r.Stats.QuadFraction < 0.70f)
            r.Failure = $"only {r.Stats.QuadFraction:P0} quads";
        if (r.Failure != null) DebugEdges(m, "defect", log);
        return r;
    }
}
