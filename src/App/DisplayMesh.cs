using System;
using Godot;
using ModelMate.Core;

namespace ModelMate.App;

/// <summary>
/// Viewport meshes, drawn like Blender's default cube: plain grey matte (base colour 0.8, roughness 0.5) with an
/// optional wireframe on top, in one shader. Vertex colours carry only the weight heatmap; otherwise every model
/// is plain grey. Each triangle gets barycentric coordinates (UV.xy, UV2.x); a quad's diagonal is hidden by
/// pinning the coordinate opposite it to 1, so remeshed models show their real quad edges. Lines are about one
/// pixel wide and fade out where triangles are only a few pixels across (dense sculpts would otherwise turn black).
/// </summary>
public static class DisplayMesh
{
    const string ShaderCode = @"
shader_type spatial;
render_mode cull_disabled;
uniform bool wire = true;
uniform vec3 wire_color : source_color = vec3(0.04, 0.04, 0.05);
uniform float wire_strength = 0.8;
varying vec3 bary;
void vertex() { bary = vec3(UV, UV2.x); }
void fragment() {
    ALBEDO = COLOR.rgb;
    ROUGHNESS = 0.5;
    if (wire) {
        vec3 d = fwidth(bary);
        vec3 a = clamp(bary / max(d * 1.2, vec3(1e-6)), 0.0, 1.0);
        float edge = 1.0 - min(min(a.x, a.y), a.z);
        float density = max(max(d.x, d.y), d.z);              // ~1 / triangle size in pixels
        float fade = 1.0 - smoothstep(0.12, 0.35, density);
        ALBEDO = mix(ALBEDO, wire_color, edge * wire_strength * fade);
    }
}";

    /// <summary>Blender's default material base colour (linear 0.8 grey).</summary>
    public static readonly Color Grey = new(0.8f, 0.8f, 0.8f);

    static ShaderMaterial _material;

    /// <summary>One material shared by every display mesh, so the wireframe toggle applies everywhere.</summary>
    public static ShaderMaterial Material => _material ??= new ShaderMaterial { Shader = new Shader { Code = ShaderCode } };

    public static void SetWire(bool on) => Material.SetShaderParameter("wire", on);

    /// <summary>The imported model, plain grey.</summary>
    public static ArrayMesh ForSource(SourceMesh s) => Build(s.Mesh, null);

    /// <summary>
    /// Un-indexed triangle mesh with barycentrics for the wireframe. <paramref name="vertexColor"/> (per vertex of
    /// <paramref name="m"/>) is only used for the weight heatmap; null means plain grey. Bones/weights (4 per vertex)
    /// make it skinnable.
    /// </summary>
    public static ArrayMesh Build(MeshData m, Color[] vertexColor, int[] bones = null, float[] weights = null)
    {
        int nt = m.TriCount, nc = 3 * nt;
        var pos = new Vector3[nc];
        var nrm = new Vector3[nc];
        var col = new Color[nc];
        var uv = new Vector2[nc];
        var uv2 = new Vector2[nc];
        var bo = bones != null ? new int[4 * nc] : null;
        var we = bones != null ? new float[4 * nc] : null;
        if (m.Nrm == null) m.ComputeNormals();
        Span<Vector3> bc = stackalloc Vector3[3];
        for (int t = 0; t < nt; t++)
        {
            bc[0] = new Vector3(1, 0, 0); bc[1] = new Vector3(0, 1, 0); bc[2] = new Vector3(0, 0, 1);
            byte f = m.TriFlags != null ? m.TriFlags[t] : (byte)0;
            for (int k = 0; k < 3; k++)
                if ((f & (1 << k)) != 0)
                {
                    // edge k (corner k -> k+1) lies where the coordinate of the opposite corner is 0: pin it to 1
                    int opp = (k + 2) % 3;
                    for (int c = 0; c < 3; c++) bc[c][opp] = 1;
                }
            // Godot draws clockwise front faces: emit corners 0, 2, 1
            for (int j = 0; j < 3; j++)
            {
                int k = j == 0 ? 0 : 3 - j;
                int v = m.Tris[3 * t + k];
                int o = 3 * t + j;
                pos[o] = m.Pos[v];
                nrm[o] = m.Nrm[v];
                col[o] = vertexColor != null ? vertexColor[v] : Grey;
                uv[o] = new Vector2(bc[k].X, bc[k].Y);
                uv2[o] = new Vector2(bc[k].Z, 0);
                if (bo != null)
                    for (int q = 0; q < 4; q++) { bo[4 * o + q] = bones[4 * v + q]; we[4 * o + q] = weights[4 * v + q]; }
            }
        }
        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = pos;
        arrays[(int)Mesh.ArrayType.Normal] = nrm;
        arrays[(int)Mesh.ArrayType.Color] = col;
        arrays[(int)Mesh.ArrayType.TexUV] = uv;
        arrays[(int)Mesh.ArrayType.TexUV2] = uv2;
        if (bo != null) { arrays[(int)Mesh.ArrayType.Bones] = bo; arrays[(int)Mesh.ArrayType.Weights] = we; }
        var am = new ArrayMesh();
        am.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);
        return am;
    }
}
