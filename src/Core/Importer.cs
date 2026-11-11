using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Loads a glTF/glb and merges all mesh surfaces into one world-space SourceMesh: geometry only (materials,
/// textures, vertex colours, lights and cameras in the file are ignored). Main thread only.
/// </summary>
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
        var tris = new List<int>();
        bool missingNormals = false;

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
                    if (nArr == null || nArr.Length < v.Length) missingNormals = true;
                    var idx = arr[(int)Mesh.ArrayType.Index].VariantType == Variant.Type.Nil ? null : arr[(int)Mesh.ArrayType.Index].AsInt32Array();

                    int baseV = pos.Count;
                    for (int i = 0; i < v.Length; i++)
                    {
                        pos.Add(xf * v[i]);
                        nrm.Add(nArr != null && i < nArr.Length ? (nb * nArr[i]).Normalized() : Vector3.Up);
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
                    }
                }
            }
            foreach (var c in n.GetChildren()) Walk(c, xf);
        }

        Walk(root, Transform3D.Identity);
        if (tris.Count == 0) throw new Exception("No triangle meshes found in file");
        var md = new MeshData(pos.ToArray(), tris.ToArray()) { Nrm = nrm.ToArray() };
        if (missingNormals) md.ComputeNormals();   // e.g. Nomad exports positions only
        return new SourceMesh { Mesh = md };
    }
}
