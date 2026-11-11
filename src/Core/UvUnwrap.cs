using System;
using System.Collections.Generic;
using Godot;
using ModelMate.Core.Uv;

namespace ModelMate.Core;

/// <summary>
/// UV unwrap. Splits seam vertices; returns the split mesh plus, for each split vertex, the index of the welded
/// vertex it came from (to transfer weights/normals). Uses the built-in <see cref="ChartUnwrap"/> on every platform;
/// xatlas (ArrayMesh.LightmapUnwrap, editor builds only) remains available on request. Main thread only for xatlas.
/// </summary>
public static class UvUnwrap
{
    /// <summary>"auto" or "charts" (built-in, merged charts), "cone" (built-in without merging: the old layout,
    /// kept for comparison) or "xatlas" (editor builds only).</summary>
    public static string Mode = "auto";
    /// <summary>Which unwrapper produced the last layout: "charts", "cone" or "xatlas".</summary>
    public static string LastUsed { get; private set; }

    public static (MeshData split, int[] origin) Unwrap(MeshData welded, int texSize, Action<string> log = null, SeamHints hints = null)
    {
        if (Mode == "xatlas")
        {
            var r = Xatlas(welded, texSize);
            LastUsed = "xatlas";
            return r;
        }
        bool merge = Mode != "cone";
        var (split, origin, info) = ChartUnwrap.Unwrap(welded, texSize, -1, hints, merge);
        LastUsed = merge ? "charts" : "cone";
        if (merge)
            log?.Invoke($"UV unwrap (charts): {info.ConeCharts} cone charts -> {info.Charts} charts ({info.Merged} merges, {info.Cut} rings cut open, " +
                $"{info.Refined} seams moved); seams {info.SeamLength:0.#}× model height, visible-seam cost {info.SeamCostShare:P0} of the cone charts'; " +
                $"worst chart: aniso p95 {info.AnisoP95:F2}, area stretch p95 {info.AreaStretchMax:F2}; fill {info.Fill:P0}, {info.TexelsPerUnit:0.#} texels/unit [{info.Detail}]");
        else
            log?.Invoke($"UV unwrap (cone): {info.Charts} charts ({info.Regrown} re-grown), fill {info.Fill:P0}, {info.TexelsPerUnit:0.#} texels/unit");
        return (split, origin);
    }

    static (MeshData split, int[] origin) Xatlas(MeshData welded, int texSize)
    {
        // Encode welded vertex index in UV.x so the mapping survives exactly.
        var uvId = new Vector2[welded.VertexCount];
        for (int i = 0; i < uvId.Length; i++) uvId[i] = new Vector2(i, 0);

        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = welded.Pos;
        arrays[(int)Mesh.ArrayType.Normal] = welded.Nrm;
        arrays[(int)Mesh.ArrayType.TexUV] = uvId;
        arrays[(int)Mesh.ArrayType.Index] = ToGodotWinding(welded.Tris);
        var am = new ArrayMesh();
        am.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);

        double area = 0;
        for (int t = 0; t < welded.TriCount; t++) area += welded.TriNormal(t).Length() * 0.5;
        float texel = (float)Math.Sqrt(area / (0.6 * texSize * (double)texSize));
        var err = am.LightmapUnwrap(Transform3D.Identity, texel);
        if (err == Error.Unconfigured || err == Error.Unavailable)
            throw new Exception($"LightmapUnwrap: {err} (xatlas is compiled into Godot editor builds only; use --unwrap charts)");
        if (err != Error.Ok) throw new Exception($"LightmapUnwrap failed: {err}");

        var outArr = am.SurfaceGetArrays(0);
        var pos = outArr[(int)Mesh.ArrayType.Vertex].AsVector3Array();
        var uv1 = outArr[(int)Mesh.ArrayType.TexUV].AsVector2Array();
        var uv2 = outArr[(int)Mesh.ArrayType.TexUV2].AsVector2Array();
        var idx = outArr[(int)Mesh.ArrayType.Index].VariantType == Variant.Type.Nil
            ? Sequential(pos.Length) : outArr[(int)Mesh.ArrayType.Index].AsInt32Array();

        var origin = new int[pos.Length];
        Dictionary<Vector3, int> byPos = null;
        int fallback = 0;
        for (int i = 0; i < pos.Length; i++)
        {
            int o = uv1 != null && i < uv1.Length ? (int)MathF.Round(uv1[i].X) : -1;
            if (o < 0 || o >= welded.VertexCount || (welded.Pos[o] - pos[i]).LengthSquared() > 1e-10f)
            {
                byPos ??= BuildPosMap(welded.Pos);
                if (!byPos.TryGetValue(pos[i], out o))
                    throw new Exception($"Unwrap: split vertex {i} has no welded origin");
                fallback++;
            }
            origin[i] = o;
        }
        if (fallback > 0) GD.Print($"Unwrap: {fallback} vertices mapped by position fallback");

        var m = new MeshData(pos, FromGodotWinding(idx)) { Uv = uv2 };
        m.Nrm = new Vector3[pos.Length];
        for (int i = 0; i < pos.Length; i++) m.Nrm[i] = welded.Nrm[origin[i]];
        return (m, origin);
    }

    static Dictionary<Vector3, int> BuildPosMap(Vector3[] pos)
    {
        var d = new Dictionary<Vector3, int>();
        for (int i = 0; i < pos.Length; i++) d.TryAdd(pos[i], i);
        return d;
    }

    static int[] Sequential(int n) { var a = new int[n]; for (int i = 0; i < n; i++) a[i] = i; return a; }

    /// <summary>MeshData is counter-clockwise; Godot arrays use clockwise front faces.</summary>
    public static int[] ToGodotWinding(int[] tris)
    {
        var o = (int[])tris.Clone();
        for (int i = 0; i < o.Length; i += 3) (o[i + 1], o[i + 2]) = (o[i + 2], o[i + 1]);
        return o;
    }

    public static int[] FromGodotWinding(int[] tris) => ToGodotWinding(tris);
}
