using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>Procedural unrigged T-pose humanoid (overlapping primitives, checker texture). +Z forward, left = +X.</summary>
public static class TestAsset
{
    sealed class Part
    {
        public PrimitiveMesh Mesh;
        public Transform3D Xf;
        public Transform3D PreXf;   // before the A-pose arm rotation (T-pose space)
        public string Tag;
    }

    public static void Write(string dir)
    {
        DirAccess.MakeDirRecursiveAbsolute(dir);
        WriteVariant(System.IO.Path.Combine(dir, "humanoid.glb"), open: false);
        WriteVariant(System.IO.Path.Combine(dir, "humanoid_open.glb"), open: true);
        WriteVariant(System.IO.Path.Combine(dir, "humanoid_apose.glb"), open: false, apose: true);
        WriteVariant(System.IO.Path.Combine(dir, "humanoid_lumpy.glb"), open: false, apose: true, lumpy: true);
    }

    /// <summary>
    /// Organic-looking displacement (T-pose space): muscle-like bulges plus ridges running obliquely to the limbs,
    /// so principal curvature no longer follows the bones. Not mirror-symmetric on purpose.
    /// </summary>
    static float Lumps(Vector3 p)
    {
        var d1 = new Vector3(0.6f, 0.5f, 0.6f).Normalized();
        var d2 = new Vector3(-0.4f, 0.7f, 0.5f).Normalized();
        var d3 = new Vector3(0.3f, -0.2f, 0.9f).Normalized();
        float f = 0.0045f * MathF.Sin(38f * p.Dot(d1)) + 0.0035f * MathF.Sin(52f * p.Dot(d2) + 1.3f)
                + 0.0025f * MathF.Sin(71f * p.Dot(d3) + 0.4f);
        (Vector3 c, float a, float r)[] bulges =
        {
            (new(0.36f, 1.48f, 0.03f), 0.014f, 0.06f), (new(-0.36f, 1.48f, 0.03f), 0.012f, 0.06f),   // biceps
            (new(0.62f, 1.45f, 0.03f), 0.009f, 0.05f), (new(-0.62f, 1.45f, 0.03f), 0.009f, 0.05f),   // forearms
            (new(0.10f, 0.66f, 0.07f), 0.012f, 0.08f), (new(-0.10f, 0.66f, 0.07f), 0.012f, 0.08f),   // thighs
            (new(0.10f, 0.31f, -0.05f), 0.014f, 0.06f), (new(-0.10f, 0.31f, -0.05f), 0.014f, 0.06f), // calves
            (new(0.09f, 1.30f, 0.11f), 0.016f, 0.08f), (new(-0.09f, 1.30f, 0.11f), 0.016f, 0.08f),   // chest
            (new(0f, 1.05f, 0.10f), 0.013f, 0.10f), (new(0.05f, 1.40f, -0.11f), 0.012f, 0.09f),       // belly, back
        };
        foreach (var (c, a, r) in bulges) f += a * MathF.Exp(-(p - c).LengthSquared() / (r * r));
        return f;
    }

    /// <summary>~1M-triangle variant for timing (very high-segment primitives).</summary>
    public static void WriteDense(string dir)
    {
        DirAccess.MakeDirRecursiveAbsolute(dir);
        WriteVariant(System.IO.Path.Combine(dir, "humanoid_dense.glb"), open: false, dense: true);
    }

    static List<Part> Parts(bool apose, bool dense, bool lumpy = false)
    {
        var parts = new List<Part>();
        void Add(PrimitiveMesh m, Vector3 at, Basis b = default, string tag = "")
        {
            if (lumpy)
            {   // enough resolution for the displacement to show
                if (m is CapsuleMesh c) { c.RadialSegments = 96; c.Rings = 48; }
                if (m is SphereMesh sp) { sp.RadialSegments = 96; sp.Rings = 48; }
                if (m is BoxMesh bx) { bx.SubdivideWidth = 24; bx.SubdivideHeight = 24; bx.SubdivideDepth = 24; }
            }
            if (dense)
            {
                if (m is CapsuleMesh c) { c.RadialSegments = 320; c.Rings = 160; }
                if (m is SphereMesh sp) { sp.RadialSegments = 320; sp.Rings = 160; }
                if (m is BoxMesh bx) { bx.SubdivideWidth = 60; bx.SubdivideHeight = 60; bx.SubdivideDepth = 60; }
            }
            var xf = new Transform3D(b == default ? Basis.Identity : b, at);
            var pre = xf;
            // A-pose: rotate arm parts 45° down about the shoulder pivot
            if (apose && MathF.Abs(at.X) > 0.25f && MathF.Abs(at.Y - 1.43f) < 0.01f)
            {
                float sx = MathF.Sign(at.X);
                var pivot = new Vector3(sx * 0.19f, 1.43f, 0);
                var rot = new Transform3D(new Basis(Vector3.Back, -sx * Mathf.Pi / 4), Vector3.Zero);
                xf = new Transform3D(Basis.Identity, pivot) * rot * new Transform3D(Basis.Identity, -pivot) * xf;
            }
            parts.Add(new Part { Mesh = m, Xf = xf, PreXf = pre, Tag = tag });
        }
        var toX = new Basis(Vector3.Back, Mathf.Pi / 2); // capsule axis Y -> X

        Add(new BoxMesh { Size = new Vector3(0.36f, 0.55f, 0.22f) }, new Vector3(0, 1.20f, 0), tag: "torso");
        Add(new BoxMesh { Size = new Vector3(0.32f, 0.18f, 0.20f) }, new Vector3(0, 0.92f, 0));
        Add(new CapsuleMesh { Radius = 0.06f, Height = 0.20f }, new Vector3(0, 1.52f, 0));
        Add(new SphereMesh { Radius = 0.11f, Height = 0.22f }, new Vector3(0, 1.66f, 0));
        Add(new SphereMesh { Radius = 0.025f, Height = 0.05f }, new Vector3(0, 1.66f, 0.11f)); // nose (+Z forward)
        foreach (var sx in new[] { 1f, -1f })
        {
            Add(new CapsuleMesh { Radius = 0.05f, Height = 0.34f }, new Vector3(sx * 0.36f, 1.43f, 0), toX);
            Add(new CapsuleMesh { Radius = 0.045f, Height = 0.30f }, new Vector3(sx * 0.66f, 1.43f, 0), toX);
            Add(new CapsuleMesh { Radius = 0.035f, Height = 0.18f }, new Vector3(sx * 0.88f, 1.43f, 0), toX, sx > 0 ? "lhand" : "");
            Add(new CapsuleMesh { Radius = 0.075f, Height = 0.48f }, new Vector3(sx * 0.10f, 0.62f, 0));
            Add(new CapsuleMesh { Radius = 0.06f, Height = 0.44f }, new Vector3(sx * 0.10f, 0.27f, 0));
            Add(new BoxMesh { Size = new Vector3(0.10f, 0.07f, 0.24f) }, new Vector3(sx * 0.10f, 0.035f, 0.05f));
        }
        return parts;
    }

    static void WriteVariant(string path, bool open, bool apose = false, bool dense = false, bool lumpy = false)
    {
        var pos = new List<Vector3>(); var nrm = new List<Vector3>(); var uv = new List<Vector2>(); var idx = new List<int>();
        foreach (var part in Parts(apose, dense, lumpy))
        {
            var a = part.Mesh.GetMeshArrays();
            var v = a[(int)Mesh.ArrayType.Vertex].AsVector3Array();
            var n = a[(int)Mesh.ArrayType.Normal].AsVector3Array();
            var t = a[(int)Mesh.ArrayType.TexUV].AsVector2Array();
            var ii = a[(int)Mesh.ArrayType.Index].AsInt32Array();
            int baseV = pos.Count;
            for (int i = 0; i < v.Length; i++)
            {
                if (lumpy)
                {
                    // displace in T-pose space, then apply the A-pose rotation, so overlapping parts agree
                    var pT = part.PreXf * v[i];
                    var nT = (part.PreXf.Basis * n[i]).Normalized();
                    if (part.Mesh is BoxMesh bx)
                    {
                        // per-face box normals would tear the corners open; use a position-only (ellipsoid) direction
                        var h = bx.Size * 0.5f;
                        var d = pT - part.PreXf.Origin;
                        nT = new Vector3(d.X / (h.X * h.X), d.Y / (h.Y * h.Y), d.Z / (h.Z * h.Z)).Normalized();
                    }
                    pos.Add(part.Xf * part.PreXf.AffineInverse() * (pT + nT * Lumps(pT)));
                }
                else pos.Add(part.Xf * v[i]);
                nrm.Add((part.Xf.Basis * n[i]).Normalized());
                uv.Add(t[i]);
            }
            for (int k = 0; k < ii.Length; k += 3)
            {
                Vector3 p0 = part.Xf * v[ii[k]], p1 = part.Xf * v[ii[k + 1]], p2 = part.Xf * v[ii[k + 2]];
                if (open && part.Tag == "torso" && p0.Y > 1.47f && p1.Y > 1.47f && p2.Y > 1.47f) continue; // hole at neck
                if (open && part.Tag == "lhand" && p0.X > 0.94f && p1.X > 0.94f && p2.X > 0.94f) continue; // hole at fingertip
                idx.Add(baseV + ii[k]); idx.Add(baseV + ii[k + 1]); idx.Add(baseV + ii[k + 2]);
            }
        }
        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = pos.ToArray();
        arrays[(int)Mesh.ArrayType.Normal] = nrm.ToArray();
        arrays[(int)Mesh.ArrayType.TexUV] = uv.ToArray();
        arrays[(int)Mesh.ArrayType.Index] = idx.ToArray();
        var mesh = new ArrayMesh();
        mesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);

        var img = Image.CreateEmpty(256, 256, false, Image.Format.Rgba8);
        for (int y = 0; y < 256; y++)
            for (int x = 0; x < 256; x++)
            {
                bool c = ((x / 32) + (y / 32)) % 2 == 0;
                img.SetPixel(x, y, c ? new Color(0.9f, 0.35f, 0.2f) : new Color(0.2f, 0.5f, 0.9f));
            }
        var mat = new StandardMaterial3D { AlbedoTexture = ImageTexture.CreateFromImage(img) };
        mesh.SurfaceSetMaterial(0, mat);

        var root = new Node3D { Name = System.IO.Path.GetFileNameWithoutExtension(path) };
        var mi = new MeshInstance3D { Name = "Body", Mesh = mesh };
        root.AddChild(mi);
        mi.Owner = root;
        Exporter.ExportGlb(root, path);
        root.Free();
    }
}
