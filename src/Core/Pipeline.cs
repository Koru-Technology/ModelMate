using System;
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
    public int VoxelRes = 192;      // cells on longest axis for retopo
    public int Close = 1;           // gap-closing voxels
    public int RelaxIters = 4;
    public int WeightRes = 80;      // cells on longest axis for weights
    public float Falloff = 3f;      // weight = 1/d^falloff
    public int SmoothPasses = 3;
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
        Quad = null; BaseMesh = null; UsedFallback = false; FallbackReason = null;
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

        Vector3[] gd = null; float[] gw = null;   // Q2: skeleton guidance
        Quad = Timed("quad remesh", () => QuadRetopo.Run(BaseMesh, SourceBvh, rho, gd, gw, S.Seed, L));
        if (Quad.Failure != null) return Quad.Failure;
        Welded = Quad.Mesh;
        Welded.ComputeNormals();
        return null;
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

    public void ComputeWeights()
    {
        var sm = Source.Mesh;
        var wg = Timed("weight voxels", () => VoxelGrid.Build(sm.Pos, sm.Tris, SourceBvh, S.WeightRes, S.Close, L));
        var dense = Timed("geodesic weights", () => VoxelGeodesicWeights.Compute(Welded, Rig, wg, SourceBvh, S.Falloff, L));
        (WeldedBones, WeldedWeights) = Timed("smooth+prune", () =>
            WeightSmoother.SmoothAndPrune(Welded, dense, Rig.Bones.Count, S.SmoothPasses));
    }

    /// <summary>[main] UV unwrap + transfer of welded attributes onto split vertices.</summary>
    public void Unwrap()
    {
        (Split, SplitOrigin) = Timed("uv unwrap", () => UvUnwrap.Unwrap(Welded, S.TexSize));
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
