using System.Linq;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.App;

/// <summary>
/// Windowed (not headless) snapshot: -- --render model.glb out.png [--pose] [--wire]
/// Loads a glb, optionally applies a test pose, renders front and side views side by side.
/// </summary>
public static class Render
{
    public static async Task<int> Run(Node host, string inPath, string outPath, bool pose, bool wire, string views = "front,side",
        string posePreset = null, string poseName = null)
    {
        Node3D scene;
        if (inPath.EndsWith(".obj", System.StringComparison.OrdinalIgnoreCase))
        {
            scene = LoadObjWithEdges(ProjectSettings.GlobalizePath(inPath));
            wire = false; // edges are drawn as real polygon edges instead
        }
        else
        {
            var doc = new GltfDocument();
            var state = new GltfState();
            if (doc.AppendFromFile(ProjectSettings.GlobalizePath(inPath), state) != Error.Ok) return 1;
            scene = (Node3D)doc.GenerateScene(state);
        }
        host.AddChild(scene);

        var skel = scene.FindChildren("*", "Skeleton3D", true, false).OfType<Skeleton3D>().FirstOrDefault();
        if (pose && skel != null) ApplyTestPose(skel);
        if (posePreset != null && skel != null)
        {
            var rots = ModelMate.Core.RigPreset.PoseRotations(ModelMate.Core.RigPreset.Load(ProjectSettings.GlobalizePath(posePreset)), poseName);
            foreach (var (bone, rot) in rots)
            {
                int i = skel.FindBone(bone);
                if (i < 0) { GD.PrintErr($"pose: no bone {bone}"); continue; }
                skel.SetBonePoseRotation(i, (skel.GetBoneRest(i).Basis * rot).GetRotationQuaternion());
            }
        }

        var aabb = new Aabb();
        bool first = true;
        foreach (var mi in scene.FindChildren("*", "MeshInstance3D", true, false).OfType<MeshInstance3D>())
        {
            var b = mi.GlobalTransform * mi.GetAabb();
            aabb = first ? b : aabb.Merge(b);
            first = false;
        }

        var env = new WorldEnvironment { Environment = new Environment { BackgroundMode = Environment.BGMode.Color, BackgroundColor = new Color(0.16f, 0.17f, 0.2f), AmbientLightSource = Environment.AmbientSource.Color, AmbientLightColor = new Color(0.5f, 0.5f, 0.55f) } };
        host.AddChild(env);
        var sun = new DirectionalLight3D { RotationDegrees = new Vector3(-40, 30, 0) };
        host.AddChild(sun);

        if (wire) host.GetViewport().DebugDraw = Viewport.DebugDrawEnum.Wireframe;

        var center = aabb.GetCenter();
        float r = aabb.Size.Length() * 0.62f;
        var named = new System.Collections.Generic.Dictionary<string, Vector3>
        {
            ["front"] = Vector3.Back, ["side"] = Vector3.Right, ["back"] = Vector3.Forward,
            ["left"] = Vector3.Left, ["top"] = Vector3.Up, ["bottom"] = Vector3.Down,
            ["three"] = new Vector3(0.7f, 0.35f, 0.7f).Normalized(),
        };
        var dirs = views.Split(',').Select(x => named[x.Trim()]).ToArray();
        var images = new Image[dirs.Length];
        var cam = new Camera3D { Fov = 40 };
        host.AddChild(cam);
        cam.MakeCurrent();
        for (int v = 0; v < dirs.Length; v++)
        {
            cam.Position = center + dirs[v] * r * 1.6f;
            var up = System.MathF.Abs(dirs[v].Y) > 0.9f ? Vector3.Back : Vector3.Up;
            cam.LookAt(center, up);
            for (int f = 0; f < 4; f++) await host.ToSignal(RenderingServer.Singleton, RenderingServer.SignalName.FramePostDraw);
            images[v] = host.GetViewport().GetTexture().GetImage();
        }
        int w = images[0].GetWidth(), h = images[0].GetHeight();
        var outImg = Image.CreateEmpty(w * images.Length, h, false, images[0].GetFormat());
        for (int v = 0; v < images.Length; v++) outImg.BlitRect(images[v], new Rect2I(0, 0, w, h), new Vector2I(w * v, 0));
        outImg.SavePng(ProjectSettings.GlobalizePath(outPath));
        GD.Print($"Rendered {outPath}");
        return 0;
    }

    /// <summary>Polygon OBJ → shaded surface + black polygon-edge overlay (shows quad flow, not triangle diagonals).</summary>
    static Node3D LoadObjWithEdges(string path)
    {
        var pos = new System.Collections.Generic.List<Vector3>();
        var faces = new System.Collections.Generic.List<int[]>();
        var ci = System.Globalization.CultureInfo.InvariantCulture;
        foreach (var line in System.IO.File.ReadLines(path))
        {
            var p = line.Split(' ', System.StringSplitOptions.RemoveEmptyEntries);
            if (p.Length == 0) continue;
            if (p[0] == "v") pos.Add(new Vector3(float.Parse(p[1], ci), float.Parse(p[2], ci), float.Parse(p[3], ci)));
            else if (p[0] == "f") faces.Add(p.Skip(1).Select(x => int.Parse(x.Split('/')[0]) - 1).ToArray());
        }
        var st = new SurfaceTool();
        st.Begin(Mesh.PrimitiveType.Triangles);
        foreach (var f in faces)
            for (int i = 1; i + 1 < f.Length; i++)
            {   // MeshData is CCW; Godot front faces are CW
                st.AddVertex(pos[f[0]]); st.AddVertex(pos[f[i + 1]]); st.AddVertex(pos[f[i]]);
            }
        st.GenerateNormals();
        var root = new Node3D();
        root.AddChild(new MeshInstance3D
        {
            Mesh = st.Commit(),
            MaterialOverride = new StandardMaterial3D { AlbedoColor = new Color(0.78f, 0.79f, 0.82f), Roughness = 0.8f },
        });
        var im = new ImmediateMesh();
        im.SurfaceBegin(Mesh.PrimitiveType.Lines);
        foreach (var f in faces)
            for (int i = 0; i < f.Length; i++) { im.SurfaceAddVertex(pos[f[i]]); im.SurfaceAddVertex(pos[f[(i + 1) % f.Length]]); }
        im.SurfaceEnd();
        root.AddChild(new MeshInstance3D
        {
            Mesh = im,
            MaterialOverride = new StandardMaterial3D { AlbedoColor = new Color(0.05f, 0.05f, 0.08f), ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded },
        });
        return root;
    }

    /// <summary>Arms down/forward, one knee bent, head turned: exercises shoulders, elbows, hips, knees.</summary>
    public static void ApplyTestPose(Skeleton3D s)
    {
        void Rot(string bone, Vector3 axis, float deg)
        {
            int i = s.FindBone(bone);
            if (i < 0) return;
            var rest = s.GetBoneRest(i);
            s.SetBonePoseRotation(i, (rest.Basis * new Basis(axis, Mathf.DegToRad(deg))).GetRotationQuaternion());
        }
        // arm bones: local +Y along arm, +Z down → rotate about local X to swing down/forward
        Rot("LeftUpperArm", Vector3.Right, 50);
        Rot("RightUpperArm", Vector3.Right, 50);
        Rot("LeftLowerArm", Vector3.Back, -60);
        Rot("RightLowerArm", Vector3.Back, 60);
        Rot("LeftUpperLeg", Vector3.Right, -40);
        Rot("LeftLowerLeg", Vector3.Right, 70);
        Rot("Head", Vector3.Up, 30);
        Rot("Spine", Vector3.Right, 15);
    }
}
