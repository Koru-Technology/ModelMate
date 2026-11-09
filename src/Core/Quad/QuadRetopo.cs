using System;
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

    public static QuadResult Run(MeshData baseMesh, Bvh source, float rho, Vector3[] guideDir, float[] guideW,
        int seed, Action<string> log, float[] sizing = null)
    {
        var r = new QuadResult { Rho = rho, Sizing = sizing };
        var sw = Stopwatch.StartNew();
        void T(string what) { log?.Invoke($"  [quad:{what}] {sw.ElapsedMilliseconds} ms"); sw.Restart(); }

        if (baseMesh.Nrm == null) baseMesh.ComputeNormals();
        var h = Hierarchy.Build(baseMesh, rho, seed, sizing);
        T($"hierarchy ({h.Levels.Count} levels, {h.Fine.Count} verts)");
        h.ResetSolution(seed);
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
        float snap = rho;
        if (sizing != null) foreach (var x in sizing) snap = MathF.Max(snap, x);
        r.Reproject = Relax.Run(m, source, snap, 3, 0.1f, QuadStats.RealAdjacency(m));
        T("cleanup+reproject");
        r.Mesh = m;
        r.Stats = QuadStats.Compute(m);
        log?.Invoke($"  Quad mesh: {r.Stats}");

        if (r.Cleanup.BadEdgesAfter > 0 || r.Stats.BoundaryEdges > 0)
            r.Failure = $"not closed/manifold (bad edges {r.Cleanup.BadEdgesAfter}, boundary {r.Stats.BoundaryEdges}, holes skipped {r.Holes.skipped})";
        else if (r.Stats.QuadFraction < 0.70f)
            r.Failure = $"only {r.Stats.QuadFraction:P0} quads";
        if (r.Failure != null) DebugEdges(m, "defect", log);
        return r;
    }
}
