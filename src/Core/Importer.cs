using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>Loads a glTF/glb and merges all mesh surfaces into one world-space SourceMesh. Main thread only.</summary>
public static class Importer
{
    public static SourceMesh Load(string path) => Load(path, false, out _);

    /// <param name="keepScene">If true, the generated scene is returned (caller owns it) for display.</param>
    public static SourceMesh Load(string path, bool keepScene, out Node scene)
    {
        scene = null;
        var doc = new GltfDocument();
        var state = new GltfState();
        var err = doc.AppendFromFile(path, state);
        if (err != Error.Ok) throw new Exception($"glTF load failed ({err}): {path}");
        var root = doc.GenerateScene(state);
        if (root == null) throw new Exception("glTF produced no scene");
        try { return FromScene(root); }
        finally { if (keepScene) scene = root; else root.Free(); }
    }

    public static SourceMesh FromScene(Node root)
    {
        var pos = new List<Vector3>();
        var nrm = new List<Vector3>();
        var uv = new List<Vector2>();
        var col = new List<Color>();
        bool anyCol = false;
        var tris = new List<int>();
        var triMat = new List<int>();
        var src = new SourceMesh();
        var matCache = new Dictionary<Material, int>();

        void Walk(Node n, Transform3D parentXf)
        {
            var xf = parentXf;
            if (n is Node3D n3) xf = parentXf * n3.Transform;
            if (n is MeshInstance3D mi && mi.Mesh != null)
            {
                var mesh = mi.Mesh;
                var nb = xf.Basis.Inverse().Transposed();
                for (int s = 0; s < mesh.GetSurfaceCount(); s++)
                {
                    if (mesh is ArrayMesh am && am.SurfaceGetPrimitiveType(s) != Mesh.PrimitiveType.Triangles) continue;
                    var arr = mesh.SurfaceGetArrays(s);
                    var v = arr[(int)Mesh.ArrayType.Vertex].AsVector3Array();
                    if (v == null || v.Length == 0) continue;
                    var nArr = arr[(int)Mesh.ArrayType.Normal].VariantType == Variant.Type.Nil ? null : arr[(int)Mesh.ArrayType.Normal].AsVector3Array();
                    var uvArr = arr[(int)Mesh.ArrayType.TexUV].VariantType == Variant.Type.Nil ? null : arr[(int)Mesh.ArrayType.TexUV].AsVector2Array();
                    var cArr = arr[(int)Mesh.ArrayType.Color].VariantType == Variant.Type.Nil ? null : arr[(int)Mesh.ArrayType.Color].AsColorArray();
                    var idx = arr[(int)Mesh.ArrayType.Index].VariantType == Variant.Type.Nil ? null : arr[(int)Mesh.ArrayType.Index].AsInt32Array();

                    var mat = mi.GetActiveMaterial(s);
                    int matIdx = MaterialIndex(src, matCache, mat);

                    int baseV = pos.Count;
                    for (int i = 0; i < v.Length; i++)
                    {
                        pos.Add(xf * v[i]);
                        nrm.Add(nArr != null && i < nArr.Length ? (nb * nArr[i]).Normalized() : Vector3.Up);
                        uv.Add(uvArr != null && i < uvArr.Length ? uvArr[i] : Vector2.Zero);
                        if (cArr != null && i < cArr.Length) { col.Add(cArr[i]); anyCol = true; }
                        else col.Add(Colors.White);
                    }
                    int triCount = idx != null ? idx.Length / 3 : v.Length / 3;
                    bool flip = xf.Basis.Determinant() < 0;
                    for (int t = 0; t < triCount; t++)
                    {
                        int a = idx != null ? idx[3 * t] : 3 * t;
                        int b = idx != null ? idx[3 * t + 1] : 3 * t + 1;
                        int c = idx != null ? idx[3 * t + 2] : 3 * t + 2;
                        // Godot stores clockwise front faces; convert to counter-clockwise (normal = (b-a)x(c-a)).
                        if (!flip) (b, c) = (c, b);
                        tris.Add(baseV + a); tris.Add(baseV + b); tris.Add(baseV + c);
                        triMat.Add(matIdx);
                    }
                }
            }
            foreach (var c in n.GetChildren()) Walk(c, xf);
        }

        Walk(root, Transform3D.Identity);
        if (tris.Count == 0) throw new Exception("No triangle meshes found in file");

        var md = new MeshData(pos.ToArray(), tris.ToArray()) { Nrm = nrm.ToArray(), Uv = uv.ToArray() };
        src.Mesh = md;
        src.TriMat = triMat.ToArray();
        if (anyCol) src.Col = col.ToArray();
        if (src.Materials.Count == 0) src.Materials.Add(new SourceMaterial());
        return src;
    }

    static int MaterialIndex(SourceMesh src, Dictionary<Material, int> cache, Material mat)
    {
        if (mat == null)
        {
            if (!cache.TryGetValue(FallbackKey, out int fi))
            {
                fi = src.Materials.Count;
                src.Materials.Add(new SourceMaterial());
                cache[FallbackKey] = fi;
            }
            return fi;
        }
        if (cache.TryGetValue(mat, out int i)) return i;
        var sm = new SourceMaterial();
        if (mat is BaseMaterial3D bm)
        {
            sm.Albedo = bm.AlbedoColor;
            sm.Transparent = bm.Transparency != BaseMaterial3D.TransparencyEnum.Disabled || bm.AlbedoColor.A < 0.99f;
            if (bm.AlbedoTexture != null)
            {
                var img = bm.AlbedoTexture.GetImage();
                if (img != null)
                {
                    img = (Image)img.Duplicate();
                    if (img.IsCompressed()) img.Decompress();
                    img.Convert(Image.Format.Rgba8);
                    sm.Tex = new SourceTexture { Rgba = img.GetData(), W = img.GetWidth(), H = img.GetHeight() };
                }
            }
        }
        i = src.Materials.Count;
        src.Materials.Add(sm);
        cache[mat] = i;
        return i;
    }

    static readonly Material FallbackKey = new StandardMaterial3D();
}
