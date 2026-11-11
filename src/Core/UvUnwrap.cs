using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// UV unwrap. Splits seam vertices; returns the split mesh plus, for each split vertex, the index of the welded
/// vertex it came from (to transfer weights/normals). Uses ArrayMesh.LightmapUnwrap (xatlas) when this Godot build
/// has it (the editor does; exported apps such as the iPad build don't) and <see cref="ChartUnwrap"/> otherwise.
/// Main thread only.
/// </summary>
public static class UvUnwrap
{
    /// <summary>"auto" (xatlas when available, else charts), "xatlas" or "charts".</summary>
    public static string Mode = "auto";
    /// <summary>Which unwrapper produced the last layout: "xatlas" or "charts".</summary>
    public static string LastUsed { get; private set; }
    static bool _xatlasMissing;

    public static (MeshData split, int[] origin) Unwrap(MeshData welded, int texSize, Action<string> log = null)
    {
        if (Mode == "charts" || (Mode == "auto" && _xatlasMissing)) return Charts(welded, texSize, log);
        try
        {
            var r = Xatlas(welded, texSize);
            LastUsed = "xatlas";
            return r;
        }
        catch (XatlasUnavailable) when (Mode == "auto")
        {
            _xatlasMissing = true;
            log?.Invoke("UV unwrap: xatlas is not in this Godot build; using the built-in chart unwrapper");
            return Charts(welded, texSize, log);
        }
    }

    sealed class XatlasUnavailable : Exception { public XatlasUnavailable(string m) : base(m) { } }

    static (MeshData, int[]) Charts(MeshData welded, int texSize, Action<string> log)
    {
        var (split, origin, info) = ChartUnwrap.Unwrap(welded, texSize);
        LastUsed = "charts";
        log?.Invoke($"UV unwrap (charts): {info.Charts} charts ({info.Regrown} re-grown), fill {info.Fill:P0}, {info.TexelsPerUnit:0.#} texels/unit");
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
        if (err == Error.Unconfigured || err == Error.Unavailable) throw new XatlasUnavailable($"LightmapUnwrap: {err}");
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

    /// <summary>[main] MikkTSpace tangents via SurfaceTool (same generator Godot uses on import).</summary>
    public static float[] GenerateTangents(MeshData m)
    {
        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = m.Pos;
        arrays[(int)Mesh.ArrayType.Normal] = m.Nrm;
        arrays[(int)Mesh.ArrayType.TexUV] = m.Uv;
        arrays[(int)Mesh.ArrayType.Index] = ToGodotWinding(m.Tris);
        var st = new SurfaceTool();
        st.CreateFromArrays(arrays);
        st.GenerateTangents();
        var outArr = st.CommitToArrays();
        var v = outArr[(int)Mesh.ArrayType.Vertex].AsVector3Array();
        if (v.Length != m.VertexCount) throw new Exception($"GenerateTangents changed vertex count {m.VertexCount} -> {v.Length}");
        for (int i = 0; i < v.Length; i++)
            if ((v[i] - m.Pos[i]).LengthSquared() > 1e-12f) throw new Exception("GenerateTangents reordered vertices");
        return outArr[(int)Mesh.ArrayType.Tangent].AsFloat32Array();
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
