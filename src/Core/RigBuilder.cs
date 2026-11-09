using System;
using Godot;

namespace ModelMate.Core;

/// <summary>Builds the Godot node tree: Node3D root → Skeleton3D → skinned MeshInstance3D. Main thread only.</summary>
public static class RigBuilder
{
    public static Node3D Build(string name, MeshData m, int[] bones4, float[] weights4, RigSkeleton rig, Image albedo, Image normal = null)
    {
        var root = new Node3D { Name = name };
        var skel = new Skeleton3D { Name = "Skeleton3D" };
        root.AddChild(skel);
        skel.Owner = root;

        var globals = new Transform3D[rig.Bones.Count];
        for (int i = 0; i < rig.Bones.Count; i++)
        {
            var b = rig.Bones[i];
            int idx = skel.AddBone(b.Name);
            if (idx != i) throw new Exception("bone index mismatch");
            globals[i] = rig.GlobalRest(i);
        }
        for (int i = 0; i < rig.Bones.Count; i++)
        {
            var b = rig.Bones[i];
            if (b.Parent >= 0) skel.SetBoneParent(i, b.Parent);
            var local = b.Parent >= 0 ? globals[b.Parent].AffineInverse() * globals[i] : globals[i];
            skel.SetBoneRest(i, local);
        }
        skel.ResetBonePoses();

        var skin = new Skin();
        for (int i = 0; i < rig.Bones.Count; i++)
            skin.AddNamedBind(rig.Bones[i].Name, globals[i].AffineInverse());

        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = m.Pos;
        arrays[(int)Mesh.ArrayType.Normal] = m.Nrm;
        arrays[(int)Mesh.ArrayType.TexUV] = m.Uv;
        if (m.Tan != null) arrays[(int)Mesh.ArrayType.Tangent] = m.Tan;
        arrays[(int)Mesh.ArrayType.Bones] = bones4;
        arrays[(int)Mesh.ArrayType.Weights] = weights4;
        arrays[(int)Mesh.ArrayType.Index] = UvUnwrap.ToGodotWinding(m.Tris);
        var mesh = new ArrayMesh();
        mesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);

        var mat = new StandardMaterial3D { Roughness = 0.9f };
        if (albedo != null)
        {
            albedo.GenerateMipmaps();
            mat.AlbedoTexture = ImageTexture.CreateFromImage(albedo);
        }
        if (normal != null && m.Tan != null)
        {
            normal.GenerateMipmaps();
            mat.NormalEnabled = true;
            mat.NormalTexture = ImageTexture.CreateFromImage(normal);
        }
        mesh.SurfaceSetMaterial(0, mat);
        mesh.ResourceName = name;

        var mi = new MeshInstance3D { Name = name + "_Mesh", Mesh = mesh, Skin = skin };
        skel.AddChild(mi);
        mi.Owner = root;
        mi.Skeleton = new NodePath("..");
        return root;
    }
}

public static class Exporter
{
    public static void ExportGlb(Node3D root, string path)
    {
        var doc = new GltfDocument();
        var state = new GltfState();
        var err = doc.AppendFromScene(root, state);
        if (err != Error.Ok) throw new Exception($"glTF AppendFromScene failed: {err}");
        err = doc.WriteToFilesystem(state, path);
        if (err != Error.Ok) throw new Exception($"glTF write failed: {err}");
    }
}
