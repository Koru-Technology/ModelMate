using System;
using System.Collections.Generic;
using System.Diagnostics;
using Godot;
using ModelMate.Core.Quad;

namespace ModelMate.Core;

public enum RetopoMode { Quad, Voxel }

public sealed class PipelineSettings
{
    public RetopoMode Mode = RetopoMode.Quad;
    /// <summary>Output triangle budget for both modes (quad mode targets half as many quads). In voxel mode 0 = no decimation.</summary>
    public int TargetTris = 8000;
    /// <summary>Quad mode needs a positive budget; 0 (meaning "no decimation" in voxel mode) falls back to the default.</summary>
    public int TargetQuads => TargetTris > 0 ? Math.Max(1, TargetTris / 2) : 4000;
    /// <summary>Minimum base-mesh resolution (cells on the longest axis) in quad mode, so low budgets still get a fine
    /// surface for the normal-map bake and the fidelity reference.</summary>
    public int DetailRes = 256;
    public int Seed = 1;            // field initialization seed (deterministic results)
    public float FlowStrength = 0;  // quad mode: skeleton guidance of edge flow (0 = off; experimental, see README)
    /// <summary>Quad-mode spacing: "auto" (default: build uniform and adaptive, keep the better surface match),
    /// "uniform" or "adaptive".</summary>
    public string Sizing = "auto";
    public float Adaptivity = 0;      // sizing field: thickness term (0 = off, 1 = spacing ∝ local thickness)
    public float CurvatureSizing = 1; // sizing field: curvature term (equal-error spacing ∝ κ^-½)
    public float SizingGradient = 0.2f; // max growth of edge length per unit distance (smaller = gentler density changes)
    public float SizingMaxRatio = 6f;   // max ratio between coarsest and finest edge length
    public bool FitVoxel = true;        // also least-squares fit the decimated voxel mesh to the dense surface
    public int VoxelRes = 256;      // voxel mode: cells on longest axis of the base mesh (same default detail as quad mode)
    public int Close = 1;           // gap-closing voxels
    public int RelaxIters = 4;
    public int WeightRes = 80;      // cells on longest axis for weights
    public float Falloff = 3f;      // weight = 1/d^falloff
    public float SmoothRadius = 0.02f;  // weight smoothing radius, fraction of model height (world-space, budget-independent)
    public bool SymmetricWeights = true; // mirror-average weights when the model and rig are both symmetric
    public int TexSize = 1024;
    public string Rig = "humanoid"; // preset name (humanoid, single, blob_biped, …) or a path to a preset .json
    public float MaxEdgeFrac = 0.04f; // decimation max edge length, fraction of bbox diagonal
    public bool JointAware = true;    // keep more triangles near joints when decimating
    public bool NormalMap = true;     // bake a tangent-space normal map from the source
    public float RotateY = 0;         // degrees applied to the source on import (e.g. 180 for models facing -Z)
}

/// <summary>
/// Stateful pipeline: import → retopo → skeleton → weights (welded) → unwrap → transfer → bake → build.
/// Steps marked [main] touch Godot objects and must run on the main thread; others are thread-safe.
/// </summary>
public sealed class Pipeline
{
    public PipelineSettings S = new();
    public Action<string> Log = s => GD.Print(s);

    public SourceMesh Source;
    public Bvh SourceBvh;
    public VoxelGrid Grid;
    public MeshData Welded;
    public RigSkeleton Rig;
    public RigPreset.PresetDto RigPresetData;   // JSON preset the rig came from (null for code presets); carries named poses
    public VoxelGrid WeightGrid;         // built once per import (rebuilt if gap closing changes)
    int _weightGridClose = -1;
    public MeshData WeightSurface;      // dense surface the weights were solved on (the voxel base mesh)
    public int[] SurfBones;             // top-4 weights on WeightSurface (the reference for every output mesh)
    public float[] SurfWeights;
    public bool WeightsSymmetric;
    public int[] WeldedBones;
    public float[] WeldedWeights;
    public MeshData Split;
    public int[] SplitOrigin;
    public int[] SplitBones;
    public float[] SplitWeights;
    public byte[] AlbedoRgba;
    public byte[] NormalRgba;
    public CleanupReport Cleanup;
    public MeshData BaseMesh;         // quad mode: dense voxel base mesh the fields were solved on
    public QuadResult Quad;           // quad mode result (null in voxel mode)
    public bool UsedFallback;
    public string FallbackReason;
    public string SizingChosen;       // quad mode: which spacing was used ("uniform" / "adaptive")
    /// <summary>Triangle budget that actually applied (voxel mode or voxel fallback); 0 when none.</summary>
    public int AppliedTriBudget => S.Mode == RetopoMode.Voxel || UsedFallback ? S.TargetTris : 0;

    readonly object _logLock = new();
    void L(string s) { lock (_logLock) Log(s); }

    T Timed<T>(string name, Func<T> f)
    {
        var sw = Stopwatch.StartNew();
        var r = f();
        L($"[{name}] {sw.ElapsedMilliseconds} ms");
        return r;
    }

    /// <summary>[main]</summary>
    public void Import(string path) => Import(path, false, out _);

    /// <summary>[main]</summary>
    public void Import(string path, bool keepScene, out Node scene)
    {
        Node sc = null;
        Source = Timed("import", () => Importer.Load(path, keepScene, out sc));
        WeightGrid = null; _weightGridClose = -1;
        scene = sc;
        if (S.RotateY != 0)
        {
            var rb = new Basis(Vector3.Up, Mathf.DegToRad(S.RotateY));
            var m = Source.Mesh;
            for (int i = 0; i < m.Pos.Length; i++) m.Pos[i] = rb * m.Pos[i];
            if (m.Nrm != null) for (int i = 0; i < m.Nrm.Length; i++) m.Nrm[i] = rb * m.Nrm[i];
            if (scene is Node3D n3) n3.Basis = rb * n3.Basis;
            L($"Rotated source {S.RotateY}° about Y");
        }
        SourceBvh = Timed("source BVH", () => new Bvh(Source.Mesh.Pos, Source.Mesh.Tris));
        var b = Source.Mesh.Bounds();
        L($"Source: {Source.Mesh.VertexCount} verts, {Source.Mesh.TriCount} tris, {Source.Materials.Count} materials, size {b.Size}");
    }

    /// <summary>Voxel remesh of the source: voxelize → surface nets → cleanup → relax/reproject (no decimation).</summary>
    public static MeshData BuildVoxelBase(MeshData src, Bvh bvh, int res, int close, int relaxIters,
        Action<string> log, out VoxelGrid grid, out CleanupReport cleanup)
    {
        var sw = Stopwatch.StartNew();
        grid = VoxelGrid.Build(src.Pos, src.Tris, bvh, res, close, log);
        log?.Invoke($"Voxel grid {grid.Nx}x{grid.Ny}x{grid.Nz}, voxel size {grid.H:0.#####} units, inside test: {grid.Method} [{sw.ElapsedMilliseconds} ms]");
        sw.Restart();
        var m = SurfaceNets.Extract(grid);
        long tNets = sw.ElapsedMilliseconds; sw.Restart();
        cleanup = MeshCleanup.Clean(m);
        long tClean = sw.ElapsedMilliseconds; sw.Restart();
        var rs = Relax.Run(m, bvh, 2f * grid.H, relaxIters);
        long tRelax = sw.ElapsedMilliseconds;
        log?.Invoke($"Base mesh: {m.VertexCount} verts, {m.TriCount} tris; cleanup {cleanup}; relax snapped={rs.Snapped} unsnapped={rs.Unsnapped} [nets {tNets} ms, cleanup {tClean} ms, relax {tRelax} ms]");
        m.ComputeNormals();
        return m;
    }

    public static double Area(MeshData m)
    {
        double a = 0;
        for (int t = 0; t < m.TriCount; t++) a += m.TriNormal(t).Length() * 0.5;
        return a;
    }

    public void Retopo()
    {
        Quad = null; BaseMesh = null; UsedFallback = false; FallbackReason = null; SizingChosen = null;
        if (S.Mode == RetopoMode.Quad)
        {
            string reason;
            try { reason = RetopoQuad(); }
            catch (Exception e) { reason = "exception: " + e.Message; GD.PrintErr(e.ToString()); }
            if (reason == null) return;
            UsedFallback = true;
            FallbackReason = reason;
            L($"WARNING: quad remesh failed ({reason}); falling back to voxel remesh.");
            RetopoVoxel(S.TargetTris);
            return;
        }
        RetopoVoxel(S.TargetTris);
    }

    /// <returns>null on success, otherwise the failure reason.</returns>
    string RetopoQuad()
    {
        var sm = Source.Mesh;
        var ext = sm.Bounds().Size;
        float longest = MathF.Max(ext.X, MathF.Max(ext.Y, ext.Z));
        // spacing from the (over-estimated, because of internal faces) source area; refined below
        float rho = MathF.Sqrt((float)(Area(sm) / S.TargetQuads));
        int res = Math.Max((int)MathF.Ceiling(longest / (rho / 3f)), S.DetailRes);
        bool clamped = false;
        if (res > 512)
        {
            res = 512;
            float hh = longest / 512f;
            if (hh > rho / 2f)
            {
                rho = 2f * hh;
                clamped = true;
                L($"Target of {S.TargetQuads} quads is too fine for this model at the 512-cell voxel cap; spacing clamped to {rho:0.####}.");
            }
        }
        res = Math.Max(res, 64);
        float rhoFloor = rho;   // when clamped, never go finer than the voxel cap allows
        Grid = null;
        VoxelGrid g = null;
        BaseMesh = Timed("voxel base mesh", () => BuildVoxelBase(sm, SourceBvh, res, S.Close, S.RelaxIters, L, out g, out Cleanup));
        Grid = g;
        // refine spacing from the clean base-mesh area
        rho = MathF.Sqrt((float)(Area(BaseMesh) / S.TargetQuads));
        if (clamped) rho = MathF.Max(rho, rhoFloor);
        L($"Quad spacing ρ = {rho:0.####} units (voxel H = {Grid.H:0.####}, H/ρ = {Grid.H / rho:0.00})");

        Vector3[] gd = null; float[] gw = null;
        if (S.FlowStrength > 0 && Rig != null && Rig.Bones.Count > 1)
        {
            var (b4, w4, _, _) = SolveWeights(BaseMesh);
            float r = rho;
            (gd, gw) = Timed("flow guidance", () => FlowGuidance.Build(BaseMesh, Rig, b4, w4, r, S.FlowStrength));
            L($"Flow guidance: strength {S.FlowStrength}");
        }

        // Spacing: uniform, adaptive (sizing field), or auto = run both on the same base mesh and keep the one that
        // matches the surface better (lower p99 deviation), so adaptivity never makes the shape worse.
        // sharp tips are a property of the surface: detect once (uniform radius) and reuse for every remesh pass
        float rt = rho;
        var tips = QuadRetopo.PinTips ? Timed("tip detection", () => TipDetector.Find(BaseMesh, null, rt)) : null;
        QuadResult uni = null, ada = null;
        if (S.Sizing != "adaptive")
        {
            float rq = rho;
            uni = Timed("quad remesh (uniform)", () => QuadRetopo.Run(BaseMesh, SourceBvh, rq, gd, gw, S.Seed, L, null, tips));
        }
        if (S.Sizing != "uniform") ada = RunAdaptive(gd, gw, tips);
        if (uni != null && ada != null)
        {
            float pu = uni.Failure == null ? QuadRetopo.SurfaceP99(BaseMesh, uni.Mesh) : float.MaxValue;
            float pa = ada.Failure == null ? QuadRetopo.SurfaceP99(BaseMesh, ada.Mesh) : float.MaxValue;
            Quad = pa < pu ? ada : uni;
            SizingChosen = pa < pu ? "adaptive" : "uniform";
            L($"Sizing auto: surface p99 uniform {pu:0.#####} vs adaptive {pa:0.#####} → {SizingChosen}");
        }
        else { Quad = uni ?? ada; SizingChosen = uni != null ? "uniform" : "adaptive"; }
        if (Quad.Failure != null) return Quad.Failure;
        Welded = Quad.Mesh;
        Welded.ComputeNormals();
        return null;
    }

    /// <summary>Quad remesh with the curvature/thickness sizing field, rescaled once if it misses the budget by &gt;4%.</summary>
    QuadResult RunAdaptive(Vector3[] gd, float[] gw, List<int> tips)
    {
        var sf = Timed("sizing field", () => SizingField.Build(BaseMesh, S.TargetQuads, S.Adaptivity, 2.5f * Grid.H,
            S.SizingMaxRatio, S.SizingGradient, L, S.CurvatureSizing));
        var sizing = sf.Size;
        float rhoQ = sf.Median;
        var q = Timed("quad remesh (adaptive)", () => QuadRetopo.Run(BaseMesh, SourceBvh, rhoQ, gd, gw, S.Seed, L, sizing, tips));
        // extraction yields more faces where density changes than Σ area/size² predicts
        if (q.Failure == null)
        {
            float faceEq = q.Stats.Quads + 0.5f * q.Stats.Triangles + q.Stats.Other;
            float ratio = faceEq / S.TargetQuads;
            if (MathF.Abs(ratio - 1) > 0.04f)
            {
                float f = MathF.Sqrt(ratio);
                var s2 = new float[sizing.Length];
                for (int i = 0; i < s2.Length; i++) s2[i] = MathF.Max(sizing[i] * f, 2.5f * Grid.H);
                L($"Budget calibration: {faceEq:F0} quad-equivalents for a target of {S.TargetQuads}; rescaling the sizing field by {f:F3}");
                float r2 = rhoQ * f;
                q = Timed("quad remesh (adaptive, calibrated)", () => QuadRetopo.Run(BaseMesh, SourceBvh, r2, gd, gw, S.Seed, L, s2, tips));
            }
        }
        return q;
    }

    void RetopoVoxel(int targetTris)
    {
        var sm = Source.Mesh;
        VoxelGrid g = null;
        var m = Timed("voxel remesh", () => BuildVoxelBase(sm, SourceBvh, S.VoxelRes, S.Close, S.RelaxIters, L, out g, out Cleanup));
        Grid = g;
        BaseMesh = m;   // decimation below returns a new mesh, so this keeps the dense surface for normal baking
        if (targetTris > 0 && m.TriCount > targetTris)
        {
            float[] vw = null;
            if (S.JointAware && (Rig != null || S.Rig == "humanoid"))
            {
                var rig = Rig ?? HumanoidTemplate.Fit(m.Bounds(), SourceBvh, null); // provisional if none yet
                vw = JointWeights(m, rig);
            }
            float maxEdge = S.MaxEdgeFrac * m.Bounds().Size.Length();
            var mm = m;
            m = Timed("decimate", () => QemDecimator.Decimate(mm, targetTris, vw, maxEdge));
            L($"Decimated to {m.TriCount} tris");
            if (QuadRetopo.FitToSurface && S.FitVoxel)
            {
                var dm = m;
                var fs = Timed("surface fit", () => SurfaceFit.Fit(dm, BaseMesh));
                L($"Surface fit: mean shift {fs.MeanShift:0.#####}, max {fs.MaxShift:0.#####}");
            }
        }
        m.ComputeNormals();
        Welded = m;
    }

    /// <summary>Per-vertex decimation weight: up to 6x error cost near bending joints.</summary>
    static float[] JointWeights(MeshData m, RigSkeleton rig)
    {
        float h = m.Bounds().Size.Y;
        float r = 0.06f * h;
        var w = new float[m.VertexCount];
        for (int v = 0; v < w.Length; v++)
        {
            float best = 0;
            foreach (var b in rig.Bones)
            {
                if (!b.Deform || b.Parent < 0 || b.Name == "Hips") continue;
                float d = (m.Pos[v] - b.Head).Length() / r;
                best = MathF.Max(best, MathF.Exp(-d * d));
            }
            w[v] = 1f + 5f * best;
        }
        return w;
    }

    public void FitSkeleton()
    {
        var box = Source.Mesh.Bounds();   // source bounds: the skeleton can be fitted before retopology
        Rig = RigPreset.Fit(S.Rig, box, SourceBvh, L, out RigPresetData);
        L($"Skeleton: {Rig.Bones.Count} bones");
    }

    /// <summary>
    /// Weights are solved once on the dense base surface, smoothed over a world-space radius, optionally
    /// mirror-averaged, pruned to 4 influences, then transferred to the output mesh by closest point. The result
    /// therefore doesn't depend on the triangle budget or the remesh topology.
    /// </summary>
    public void ComputeWeights()
    {
        var surf = BaseMesh ?? Welded;
        WeightSurface = surf;
        Bvh surfBvh;
        (SurfBones, SurfWeights, WeightsSymmetric, surfBvh) = SolveWeights(surf);
        if (surf == Welded) (WeldedBones, WeldedWeights) = (SurfBones, SurfWeights);
        else
            (WeldedBones, WeldedWeights) = Timed("transfer weights", () =>
                Symmetry.TransferWeights(surf, surfBvh, SurfBones, SurfWeights, Welded));
    }

    /// <summary>Geodesic weights on a dense surface: smooth (world radius), mirror if symmetric, top-4.</summary>
    public (int[] bones, float[] weights, bool symmetric, Bvh surfBvh) SolveWeights(MeshData surf)
    {
        var sm = Source.Mesh;
        if (WeightGrid == null || _weightGridClose != S.Close)
        {
            WeightGrid = Timed("weight voxels", () => VoxelGrid.Build(sm.Pos, sm.Tris, SourceBvh, S.WeightRes, S.Close, L));
            _weightGridClose = S.Close;
        }
        int nb = Rig.Bones.Count;
        var dense = Timed("geodesic weights", () => VoxelGeodesicWeights.Compute(surf, Rig, WeightGrid, SourceBvh, S.Falloff, L));

        // passes for a fixed world radius: ½-averaging is a lazy random walk, spread ≈ edge·√(passes/2), so (radius/edge)² passes ≈ 0.7·radius
        double edgeSum = 0; int edgeN = 0;
        for (int t = 0; t < surf.TriCount; t += 7)
        {
            edgeSum += (surf.Pos[surf.Tris[3 * t]] - surf.Pos[surf.Tris[3 * t + 1]]).Length(); edgeN++;
        }
        float edge = MathF.Max((float)(edgeSum / Math.Max(1, edgeN)), 1e-9f);
        float radius = S.SmoothRadius * sm.Bounds().Size.Y;
        int passes = Math.Clamp((int)MathF.Ceiling(radius / edge * (radius / edge)), 1, 200);
        dense = Timed($"smooth weights ({passes} passes)", () => WeightSmoother.Smooth(surf, dense, nb, passes));

        bool symmetric = false;
        var surfBvh = new Bvh(surf.Pos, surf.Tris);
        if (S.SymmetricWeights)
        {
            var box = surf.Bounds();
            float cx = box.GetCenter().X;
            float voxel = Grid?.H ?? box.Size.Length() / 256f;
            var (meshSym, mean, p99, max) = Symmetry.DetectMesh(surf, surfBvh, cx, voxel);
            bool rigSym = Symmetry.RigSymmetric(Rig, cx, 0.01f * box.Size.Y, out var why);
            if (meshSym && rigSym)
            {
                dense = Timed("symmetric weights", () => Symmetry.SymmetrizeDense(surf, surfBvh, dense, nb, Symmetry.BoneMirror(Rig), cx));
                symmetric = true;
                L("Weights: mirror-averaged (model and rig are symmetric)");
            }
            else L($"Weights: not mirrored ({(meshSym ? "rig: " + why : $"model isn't mirror-symmetric (mean {mean / voxel:F2}, p99 {p99 / voxel:F2}, max {max / voxel:F1} voxels)")})");
        }
        var (bones, weights) = WeightSmoother.Prune(dense, surf.VertexCount, nb);
        return (bones, weights, symmetric, surfBvh);
    }

    /// <summary>[main] UV unwrap + transfer of welded attributes onto split vertices.</summary>
    public void Unwrap()
    {
        (Split, SplitOrigin) = Timed("uv unwrap", () => UvUnwrap.Unwrap(Welded, S.TexSize, L));
        int n = Split.VertexCount;
        SplitBones = new int[n * 4];
        SplitWeights = new float[n * 4];
        for (int v = 0; v < n; v++)
        {
            int o = SplitOrigin[v];
            Array.Copy(WeldedBones, o * 4, SplitBones, v * 4, 4);
            Array.Copy(WeldedWeights, o * 4, SplitWeights, v * 4, 4);
        }
        L($"Unwrap: {Welded.VertexCount} welded -> {n} split verts");
        if (S.NormalMap) Split.Tan = UvUnwrap.GenerateTangents(Split);
    }

    public void Bake()
    {
        var bvhBase = BaseMesh != null ? new Bvh(BaseMesh.Pos, BaseMesh.Tris) : null;
        (AlbedoRgba, NormalRgba) = Timed("bake maps", () =>
            TextureBaker.Bake(Split, Source, SourceBvh, S.TexSize, 2f * Grid.H, BaseMesh, bvhBase));
    }

    /// <summary>[main]</summary>
    public Node3D BuildScene(string name)
    {
        var img = AlbedoRgba != null ? Image.CreateFromData(S.TexSize, S.TexSize, false, Image.Format.Rgba8, AlbedoRgba) : null;
        var nimg = NormalRgba != null ? Image.CreateFromData(S.TexSize, S.TexSize, false, Image.Format.Rgba8, NormalRgba) : null;
        return RigBuilder.Build(name, Split, SplitBones, SplitWeights, Rig, img, nimg);
    }
}
