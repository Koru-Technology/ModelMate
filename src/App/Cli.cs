using System;
using System.Collections.Generic;
using System.Linq;
using Godot;
using ModelMate.Core;

namespace ModelMate.App;

/// <summary>
/// Headless entry: Godot --headless --path . -- --in a.glb --out b.glb [--voxel 192] [--close 1] [--tris 0]
///   [--tex 1024] [--relax 4] [--falloff 3] [--rig humanoid|single|blob_biped|preset.json] [--save-rig out.json]
/// or: -- --make-test-asset test/
/// </summary>
public static class Cli
{
    public static int Run(string[] args)
    {
        var a = Parse(args);
        try
        {
            if (a.ContainsKey("selftest-fields"))
            {
                var st = new CheckList();
                ModelMate.Core.Quad.SelfTest.Run(st, s => GD.Print(s));
                ModelMate.Core.Quad.SelfTest.Rig(st, s => GD.Print(s));
                GD.Print("Selftest:");
                st.Print(s => GD.Print(s));
                return st.AllOk ? 0 : 1;
            }
            if (a.TryGetValue("make-test-asset", out var dir))
            {
                TestAsset.Write(ProjectSettings.GlobalizePath(dir));
                if (a.ContainsKey("dense")) TestAsset.WriteDense(ProjectSettings.GlobalizePath(dir));
                GD.Print($"Wrote test assets to {dir}");
                return 0;
            }
            a.TryGetValue("out", out var output);
            if (!a.TryGetValue("in", out var input) || (output == null && !a.ContainsKey("deform-report")))
            {
                GD.PrintErr("usage: -- --in model.glb (--out out.glb | --deform-report dir [--budgets 1000,2000] [--modes quad,voxel]) [--voxel N] [--close N] [--tris N] [--tex N] [--relax N] [--falloff F] [--rig humanoid|single|blob_biped|preset.json] [--save-rig out.json] [--rotate-y DEG] [--retopo quad|voxel] [--quads N] [--dump-quads out.obj]");
                return 2;
            }
            var p = new Pipeline();
            if (a.TryGetValue("voxel", out var v)) p.S.VoxelRes = int.Parse(v);
            if (a.TryGetValue("close", out v)) p.S.Close = int.Parse(v);
            if (a.TryGetValue("tris", out v)) p.S.TargetTris = int.Parse(v);
            if (a.TryGetValue("tex", out v)) p.S.TexSize = int.Parse(v);
            if (a.TryGetValue("relax", out v)) p.S.RelaxIters = int.Parse(v);
            if (a.TryGetValue("falloff", out v)) p.S.Falloff = float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.TryGetValue("rig", out v)) p.S.Rig = v;
            if (a.ContainsKey("no-normal")) p.S.NormalMap = false;
            if (a.TryGetValue("retopo", out v)) p.S.Mode = v == "voxel" ? RetopoMode.Voxel : RetopoMode.Quad;
            if (a.TryGetValue("quads", out v)) p.S.TargetTris = 2 * int.Parse(v);   // alias: quads = triangles / 2
            if (a.TryGetValue("detail", out v)) p.S.DetailRes = int.Parse(v);
            if (a.TryGetValue("seed", out v)) p.S.Seed = int.Parse(v);
            if (a.TryGetValue("sizing-grad", out v)) p.S.SizingGradient = float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.TryGetValue("sizing-ratio", out v)) p.S.SizingMaxRatio = float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.TryGetValue("sizing", out v)) p.S.Sizing = v;
            if (a.ContainsKey("no-fit")) ModelMate.Core.Quad.QuadRetopo.FitToSurface = false;
            if (a.ContainsKey("no-pin")) ModelMate.Core.Quad.QuadRetopo.PinTips = false;
            if (a.TryGetValue("unwrap", out v)) UvUnwrap.Mode = v;

            if (a.TryGetValue("curv", out v)) p.S.CurvatureSizing = float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.TryGetValue("adapt", out v)) p.S.Adaptivity = float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.TryGetValue("flow", out v)) p.S.FlowStrength = float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.TryGetValue("rotate-y", out v)) p.S.RotateY = float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.ContainsKey("no-joint-aware")) p.S.JointAware = false;

            input = ProjectSettings.GlobalizePath(input);
            if (output != null) output = ProjectSettings.GlobalizePath(output);
            var total = System.Diagnostics.Stopwatch.StartNew();
            p.Import(input);
            FitAndMaybeSave(p, a);
            if (a.TryGetValue("deform-report", out var rd))
            {
                var budgets = (a.TryGetValue("budgets", out var bs) ? bs : "1000,2000,4000,8000,16000")
                    .Split(',').Select(x => int.Parse(x.Trim())).ToArray();
                var modes = (a.TryGetValue("modes", out var ms) ? ms : "quad,voxel").Split(',').Select(x => x.Trim()).ToArray();
                return DeformReport.Run(p, ProjectSettings.GlobalizePath(rd), budgets, modes, s => GD.Print(s));
            }
            p.Retopo();
            if (a.TryGetValue("dump-quads", out var dq) && p.Welded?.TriFlags != null)
            {
                ModelMate.Core.Quad.PolyMesh.FromTriangles(p.Welded).WriteObj(ProjectSettings.GlobalizePath(dq));
                GD.Print($"Wrote quad layout to {dq}");
            }
            p.ComputeWeights();
            if (a.ContainsKey("probe-weights")) ProbeWeights(p);
            p.Unwrap();
            p.Bake();

            var cl = new CheckList();
            Verify.PreExport(cl, p.Welded, p.Split, p.SplitOrigin, p.SplitBones, p.SplitWeights, p.AppliedTriBudget);
            ChartUnwrap.CheckLayout(cl, $"UV layout ({UvUnwrap.LastUsed})", p.Split, p.S.TexSize, UvUnwrap.LastUsed == "charts");
            Verify.BakedTexture(cl, p.AlbedoRgba);
            Verify.QuadMesh(cl, p);
            Verify.LimbSides(cl, p);

            var name = System.IO.Path.GetFileNameWithoutExtension(output);
            var root = p.BuildScene(name);
            Exporter.ExportGlb(root, output);
            root.Free();
            GD.Print($"Exported {output} ({p.Split.TriCount} tris, {p.Rig.Bones.Count} bones) in {total.ElapsedMilliseconds} ms");

            Verify.RoundTrip(cl, output, Verify.IsHumanoid(p.Rig), p.S.NormalMap);
            GD.Print("Verification:");
            cl.Print(s => GD.Print(s));
            return cl.AllOk ? 0 : 1;
        }
        catch (Exception e)
        {
            GD.PrintErr("ERROR: " + e);
            return 1;
        }
    }

    static void FitAndMaybeSave(Pipeline p, Dictionary<string, string> a)
    {
        p.FitSkeleton();
        if (a.TryGetValue("save-rig", out var sr))
        {
            var path = RigPreset.Save(p.Rig, p.Source.Mesh.Bounds(), System.IO.Path.GetFileNameWithoutExtension(sr),
                ProjectSettings.GlobalizePath(sr), p.RigPresetData?.Description, p.RigPresetData?.Poses);
            GD.Print($"Saved rig to {path}");
        }
    }

    /// <summary>Prints top weights at named probe points (for the procedural test humanoid).</summary>
    static void ProbeWeights(Pipeline p)
    {
        var probes = new (string, Vector3)[]
        {
            ("chest front", new(0, 1.30f, 0.11f)), ("torso top L corner", new(0.17f, 1.47f, 0)),
            ("torso side L", new(0.18f, 1.25f, 0)), ("upper arm mid", new(0.36f, 1.48f, 0)),
            ("elbow", new(0.52f, 1.43f, 0.045f)), ("thigh", new(0.10f, 0.62f, 0.075f)),
            ("crotch front", new(0, 0.85f, 0.1f)), ("neck", new(0, 1.55f, 0.06f)), ("knee", new(0.1f, 0.44f, 0.07f)),
        };
        foreach (var b in p.Rig.Bones) GD.Print($"  joint {b.Name,-14} {b.Head}");
        foreach (var (name, pt) in probes)
        {
            int best = 0; float bd = float.MaxValue;
            for (int v = 0; v < p.Welded.VertexCount; v++)
            {
                float d = (p.Welded.Pos[v] - pt).LengthSquared();
                if (d < bd) { bd = d; best = v; }
            }
            var parts = new List<string>();
            for (int k = 0; k < 4; k++)
            {
                float w = p.WeldedWeights[4 * best + k];
                if (w > 0.005f) parts.Add($"{p.Rig.Bones[p.WeldedBones[4 * best + k]].Name}={w:F2}");
            }
            GD.Print($"  probe {name,-20} {string.Join(" ", parts)}");
        }
    }

    static Dictionary<string, string> Parse(string[] args)
    {
        var d = new Dictionary<string, string>();
        for (int i = 0; i < args.Length; i++)
        {
            if (!args[i].StartsWith("--")) continue;
            var k = args[i][2..];
            string val = i + 1 < args.Length && !args[i + 1].StartsWith("--") ? args[++i] : "";
            d[k] = val;
        }
        return d;
    }
}
