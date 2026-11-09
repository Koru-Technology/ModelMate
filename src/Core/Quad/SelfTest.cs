using System;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>Synthetic field/extraction tests. Base meshes go through the real voxel path (staircase bias shows up).</summary>
public static class SelfTest
{
    public static void Run(CheckList cl, Action<string> log)
    {
        Sphere(cl, log);
        Capsule(cl, log);
    }

    static MeshData FromPrimitive(PrimitiveMesh pm, Basis b)
    {
        var a = pm.GetMeshArrays();
        var v = a[(int)Mesh.ArrayType.Vertex].AsVector3Array();
        for (int i = 0; i < v.Length; i++) v[i] = b * v[i];
        var idx = UvUnwrap.FromGodotWinding(a[(int)Mesh.ArrayType.Index].AsInt32Array());
        return new MeshData(v, idx);
    }

    static (MeshData baseMesh, Bvh bvh, float rho) Prepare(MeshData src, int targetQuads, Action<string> log)
    {
        var bvh = new Bvh(src.Pos, src.Tris);
        var size = src.Bounds().Size;
        float longest = MathF.Max(size.X, MathF.Max(size.Y, size.Z));
        float rho = MathF.Sqrt((float)(Pipeline.Area(src) / targetQuads));
        int res = Math.Clamp((int)MathF.Ceiling(longest / (rho / 3f)), 64, 512);
        var bm = Pipeline.BuildVoxelBase(src, bvh, res, 1, 4, log, out _, out _);
        rho = MathF.Sqrt((float)(Pipeline.Area(bm) / targetQuads));
        return (bm, bvh, rho);
    }

    static void Sphere(CheckList cl, Action<string> log)
    {
        log("Selftest 1a: unconstrained sphere");
        var src = FromPrimitive(new SphereMesh { Radius = 0.5f, Height = 1f, RadialSegments = 128, Rings = 64 }, Basis.Identity);
        var (bm, bvh, rho) = Prepare(src, 1500, log);
        var r = QuadRetopo.Run(bm, bvh, rho, null, null, 1, log);
        var s = r.Stats;
        cl.Check(r.Failure == null, $"1a sphere: structural gates ({r.Failure ?? "ok"})");
        cl.Check(s.EulerChar == 2, $"1a sphere: closed genus-0 (χ = {s.EulerChar})");
        // A smooth 4-RoSy field on a genus-0 surface has exactly 8 index-1/4 singularities.
        cl.Check(r.OrientationSingularities == 8, $"1a sphere: orientation field has exactly 8 singularities ({r.OrientationSingularities})");
        // Quad-dominant extraction adds position-field singularities (triangle/pentagon pairs); bound them loosely.
        cl.Check(s.Irregular <= 0.05f * s.Vertices, $"1a sphere: irregular vertices {s.Irregular} <= 5% of {s.Vertices}");
        cl.Check(s.QuadFraction > 0.90f, $"1a sphere: quads {s.QuadFraction:P1} > 90%");
        cl.Check(Math.Abs(s.Quads - 1500) <= 300, $"1a sphere: quad count {s.Quads} within ±20% of 1500");
    }

    static void Capsule(CheckList cl, Action<string> log)
    {
        log("Selftest 1b: capsule off the grid axes, analytic guidance along its axis");
        var axis = new Vector3(1f, 0.6f, 0.3f).Normalized();
        var basis = Geo.BasisYAlong(axis, Vector3.Up);   // maps local +Y (capsule axis) to `axis`
        var src = FromPrimitive(new CapsuleMesh { Radius = 0.15f, Height = 1.2f, RadialSegments = 128, Rings = 64 }, basis);
        var (bm, bvh, rho) = Prepare(src, 1500, log);
        var dir = new Vector3[bm.VertexCount];
        var w = new float[bm.VertexCount];
        for (int i = 0; i < dir.Length; i++)
        {
            dir[i] = axis;
            var n = bm.Nrm[i];
            w[i] = (axis - n * n.Dot(axis)).Length();   // fades to 0 on the caps
        }
        var r = QuadRetopo.Run(bm, bvh, rho, dir, w, 1, log);
        var m = r.Mesh;
        var s = r.Stats;
        cl.Check(r.Failure == null, $"1b capsule: structural gates ({r.Failure ?? "ok"})");
        cl.Check(s.EulerChar == 2, $"1b capsule: closed genus-0 (χ = {s.EulerChar})");
        cl.Check(r.OrientationSingularities == 8, $"1b capsule: orientation field has exactly 8 singularities ({r.OrientationSingularities})");
        bool OnBarrel(int v) => MathF.Abs(m.Pos[v].Dot(axis)) < 0.30f;   // cylinder part is |s| < 0.45
        int barrel = 0, reg = 0;
        for (int v = 0; v < m.VertexCount; v++)
            if (OnBarrel(v)) { barrel++; if (s.Valence[v] == 4) reg++; }
        float regFrac = barrel > 0 ? reg / (float)barrel : 0;
        cl.Check(regFrac > 0.95f, $"1b capsule: barrel valence-4 {regFrac:P1} > 95% (n={barrel})");
        float al = QuadStats.Alignment(m, v => OnBarrel(v) ? axis : null, 10f, out int samples);
        cl.Check(al > 0.90f, $"1b capsule: barrel edges within 10° of axis/perpendicular {al:P1} > 90% (n={samples})");
    }
}
