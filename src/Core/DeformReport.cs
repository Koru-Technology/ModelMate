using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Deformation report: how well each remesh variant (mode × triangle budget) follows a dense reference when posed.
///
/// Every variant gets the SAME weights, transferred from the dense reference surface, so differences come from
/// tessellation and edge layout only. Per pose and variant it measures, in % of model height:
///   posed deviation (reference samples → posed variant surface), and its excess over the rest-pose deviation at
///   the same samples; normal error (degrees); triangles flipped or shrunk below 20% area near the posed bones.
/// </summary>
public static class DeformReport
{
    sealed class Variant
    {
        public string Mode; public int Budget; public bool Fallback; public string FallbackReason;
        public MeshData Mesh; public int[] Bones; public float[] Weights; public float[] RestDist;
        public string Name => $"{Mode}{Budget}";
        // aggregates
        public readonly List<Row> Rows = new();
    }

    sealed class Pose { public string Name; public bool Named; public Dictionary<int, Basis> Rot = new(); }

    sealed class Row
    {
        public string Pose; public bool Named;
        public float DevMean, DevP99, RestP99, ExcessP99, NrmMean, NrmP95;
        public int Flipped, Shrunk, RegionTris, Samples;
    }

    static readonly Dictionary<string, (string bone, Vector3 axis, float deg)[]> HumanoidPoses = new()
    {
        ["elbows_90"] = new[] { ("LeftLowerArm", Vector3.Back, 90f), ("RightLowerArm", Vector3.Back, -90f) },
        ["knees_90"] = new[] { ("LeftLowerLeg", Vector3.Right, 90f), ("RightLowerLeg", Vector3.Right, 90f) },
        ["arms_down"] = new[] { ("LeftUpperArm", Vector3.Right, 60f), ("RightUpperArm", Vector3.Right, 60f) },
        ["stride"] = new[] { ("LeftUpperLeg", Vector3.Right, -45f), ("LeftLowerLeg", Vector3.Right, 60f), ("RightUpperLeg", Vector3.Right, 30f) },
    };

    public static int Run(Pipeline p, string dir, int[] budgets, string[] modes, Action<string> log)
    {
        Directory.CreateDirectory(dir);
        var src = p.Source.Mesh;
        float H = src.Bounds().Size.Y;
        var rig = p.Rig;
        int nb = rig.Bones.Count;

        // ---- reference: dense base surface + weights (shared by every variant)
        int refRes = Math.Max(p.S.DetailRes, 256);
        var refSurf = Pipeline.BuildVoxelBase(src, p.SourceBvh, refRes, p.S.Close, p.S.RelaxIters, log, out var refGrid, out _);
        p.Grid = refGrid;
        var (refB, refW, sym, refBvh) = p.SolveWeights(refSurf);
        log($"Report reference: {refSurf.VertexCount} verts (res {refRes}), weights {(sym ? "mirrored" : "not mirrored")}");

        // ---- variants
        var variants = new List<Variant>();
        foreach (var mode in modes)
            foreach (var b in budgets)
            {
                p.S.Mode = mode == "voxel" ? RetopoMode.Voxel : RetopoMode.Quad;
                p.S.FlowStrength = mode == "quadflow" ? 0.7f : 0f;   // quadflow = quad mode with skeleton guidance
                // quad = uniform spacing, quadadapt = sizing field, quadauto = the default (better of the two)
                p.S.Sizing = mode == "quadadapt" ? "adaptive" : mode == "quadauto" ? "auto" : "uniform";
                p.S.TargetTris = b;
                // Same starting surface for both modes: voxel mode would otherwise remesh from VoxelRes (192 by
                // default), which can bridge narrow gaps (armpits) and confound the comparison.
                p.S.VoxelRes = refRes;
                p.Retopo();
                var v = new Variant { Mode = mode, Budget = b, Mesh = p.Welded, Fallback = p.UsedFallback, FallbackReason = p.FallbackReason };
                (v.Bones, v.Weights) = Symmetry.TransferWeights(refSurf, refBvh, refB, refW, v.Mesh);
                var vb = new Bvh(v.Mesh.Pos, v.Mesh.Tris);
                v.RestDist = new float[refSurf.VertexCount];
                Parallel.For(0, refSurf.VertexCount, i =>
                    v.RestDist[i] = vb.Closest(refSurf.Pos[i], 0.25f * H, null, out var h) ? h.Dist : 0.25f * H);
                variants.Add(v);
                log($"Report variant {v.Name}: {v.Mesh.TriCount} tris{(v.Fallback ? " (FALLBACK: " + v.FallbackReason + ")" : "")}");
            }

        // ---- poses
        var poses = new List<Pose>();
        for (int i = 0; i < nb; i++)
        {
            if (!rig.Bones[i].Deform || rig.Bones[i].Parent < 0) continue;
            foreach (var (axis, an) in new[] { (Vector3.Right, "X"), (Vector3.Back, "Z") })
                foreach (var deg in new[] { 45f, -45f })
                {
                    var ps = new Pose { Name = $"{rig.Bones[i].Name} {an}{deg:+0;-0}" };
                    ps.Rot[i] = new Basis(axis, Mathf.DegToRad(deg));
                    poses.Add(ps);
                }
        }
        IEnumerable<(string, Dictionary<int, Basis>)> Named()
        {
            if (p.RigPresetData?.Poses != null)
                foreach (var name in p.RigPresetData.Poses.Keys)
                {
                    var rots = RigPreset.PoseRotations(p.RigPresetData, name);
                    yield return (name, rots.Where(kv => rig.Find(kv.Key) >= 0).ToDictionary(kv => rig.Find(kv.Key), kv => kv.Value));
                }
            else if (Verify.IsHumanoid(rig))
                foreach (var (name, rots) in HumanoidPoses)
                    yield return (name, rots.Where(r => rig.Find(r.bone) >= 0)
                        .ToDictionary(r => rig.Find(r.bone), r => new Basis(r.axis, Mathf.DegToRad(r.deg))));
        }
        foreach (var (name, rot) in Named()) poses.Add(new Pose { Name = name, Named = true, Rot = rot });
        log($"Report: {variants.Count} variants × {poses.Count} poses");

        // rest transforms
        var restG = new Transform3D[nb];
        for (int i = 0; i < nb; i++) restG[i] = rig.GlobalRest(i);
        var localRest = new Transform3D[nb];
        for (int i = 0; i < nb; i++)
            localRest[i] = rig.Bones[i].Parent >= 0 ? restG[rig.Bones[i].Parent].AffineInverse() * restG[i] : restG[i];

        var worst = new Dictionary<Variant, (float excess, Pose pose)>();
        foreach (var pose in poses)
        {
            var g = Skinning.Globals(nb, i => rig.Bones[i].Parent,
                i => pose.Rot.TryGetValue(i, out var r) ? localRest[i] * new Transform3D(r, Vector3.Zero) : localRest[i]);
            var bindXf = new Transform3D[nb];
            for (int i = 0; i < nb; i++) bindXf[i] = g[i] * restG[i].AffineInverse();

            // region = posed bones and their descendants
            var inRegion = new bool[nb];
            for (int i = 0; i < nb; i++)
                for (int q = i; q >= 0; q = rig.Bones[q].Parent)
                    if (pose.Rot.ContainsKey(q)) { inRegion[i] = true; break; }
            float RegionW(int[] bones, float[] w, int v)
            {
                float s = 0;
                for (int k = 0; k < 4; k++) if (inRegion[bones[4 * v + k]]) s += w[4 * v + k];
                return s;
            }

            // samples on the reference
            var cand = Enumerable.Range(0, refSurf.VertexCount).Where(v => pose.Named || RegionW(refB, refW, v) > 0.05f).ToList();
            int cap = pose.Named ? 20000 : 8000;
            int stride = Math.Max(1, cand.Count / cap);
            var samples = cand.Where((_, k) => k % stride == 0).ToArray();
            if (samples.Length == 0) continue;

            var refPosed = new MeshData(Skinning.Skin(refSurf.Pos, refB, refW, 4, bindXf), refSurf.Tris);
            refPosed.ComputeNormals();

            foreach (var v in variants)
            {
                var vp = new MeshData(Skinning.Skin(v.Mesh.Pos, v.Bones, v.Weights, 4, bindXf), v.Mesh.Tris);
                vp.ComputeNormals();
                var vb = new Bvh(vp.Pos, vp.Tris);
                var dev = new float[samples.Length];
                var exc = new float[samples.Length];
                var nerr = new float[samples.Length];
                Parallel.For(0, samples.Length, k =>
                {
                    int s = samples[k];
                    if (!vb.Closest(refPosed.Pos[s], 0.25f * H, null, out var hit)) { dev[k] = 0.25f * H; exc[k] = dev[k]; nerr[k] = 90; return; }
                    dev[k] = hit.Dist;
                    exc[k] = hit.Dist - v.RestDist[s];
                    int a = vp.Tris[3 * hit.Tri], b = vp.Tris[3 * hit.Tri + 1], c = vp.Tris[3 * hit.Tri + 2];
                    var n = (vp.Nrm[a] * hit.U + vp.Nrm[b] * hit.V + vp.Nrm[c] * hit.W).Normalized();
                    nerr[k] = Mathf.RadToDeg(MathF.Acos(Math.Clamp(n.Dot(refPosed.Nrm[s]), -1f, 1f)));
                });
                // collapsed triangles near the posed bones
                int flipped = 0, shrunk = 0, regionTris = 0;
                for (int t = 0; t < v.Mesh.TriCount; t++)
                {
                    int a = v.Mesh.Tris[3 * t], b = v.Mesh.Tris[3 * t + 1], c = v.Mesh.Tris[3 * t + 2];
                    if (RegionW(v.Bones, v.Weights, a) < 0.3f && RegionW(v.Bones, v.Weights, b) < 0.3f && RegionW(v.Bones, v.Weights, c) < 0.3f) continue;
                    regionTris++;
                    var n0 = (v.Mesh.Pos[b] - v.Mesh.Pos[a]).Cross(v.Mesh.Pos[c] - v.Mesh.Pos[a]);
                    var n1 = (vp.Pos[b] - vp.Pos[a]).Cross(vp.Pos[c] - vp.Pos[a]);
                    if (n0.Dot(n1) < 0) flipped++;
                    else if (n1.Length() < 0.2f * n0.Length()) shrunk++;
                }
                var restSamp = samples.Select(s => v.RestDist[s]).ToArray();
                var row = new Row
                {
                    Pose = pose.Name, Named = pose.Named, Samples = samples.Length,
                    DevMean = Pct(dev.Average(), H), DevP99 = Pct(P(dev, 0.99f), H), RestP99 = Pct(P(restSamp, 0.99f), H),
                    ExcessP99 = Pct(P(exc, 0.99f), H), NrmMean = nerr.Average(), NrmP95 = P(nerr, 0.95f),
                    Flipped = flipped, Shrunk = shrunk, RegionTris = regionTris,
                };
                v.Rows.Add(row);
                if (!worst.TryGetValue(v, out var w) || row.ExcessP99 > w.excess) worst[v] = (row.ExcessP99, pose);
                if (pose.Named) WriteObj(Path.Combine(dir, $"{v.Name}_{Safe(pose.Name)}.obj"), vp, v.Mesh.TriFlags);
            }
        }

        // worst pose per variant as OBJ (re-skin)
        foreach (var (v, (_, pose)) in worst)
        {
            var g = Skinning.Globals(nb, i => rig.Bones[i].Parent,
                i => pose.Rot.TryGetValue(i, out var r) ? localRest[i] * new Transform3D(r, Vector3.Zero) : localRest[i]);
            var bindXf = new Transform3D[nb];
            for (int i = 0; i < nb; i++) bindXf[i] = g[i] * restG[i].AffineInverse();
            var vp = new MeshData(Skinning.Skin(v.Mesh.Pos, v.Bones, v.Weights, 4, bindXf), v.Mesh.Tris);
            WriteObj(Path.Combine(dir, $"{v.Name}_worst_{Safe(pose.Name)}.obj"), vp, v.Mesh.TriFlags);
        }

        WriteCsv(Path.Combine(dir, "report.csv"), variants);
        var md = Markdown(p, variants, worst, H);
        File.WriteAllText(Path.Combine(dir, "report.md"), md);
        log(md);
        return variants.Any(v => v.Fallback) ? 1 : 0;
    }

    static float Pct(float v, float h) => 100f * v / h;
    static float P(float[] a, float q)
    {
        var c = (float[])a.Clone();
        Array.Sort(c);
        return c.Length == 0 ? 0 : c[Math.Min(c.Length - 1, (int)(q * c.Length))];
    }
    static string Safe(string s) => new string(s.Select(ch => char.IsLetterOrDigit(ch) || ch == '-' ? ch : '_').ToArray());

    static void WriteObj(string path, MeshData posed, byte[] triFlags)
    {
        var m = new MeshData(posed.Pos, posed.Tris) { TriFlags = triFlags };
        Quad.PolyMesh.FromTriangles(m).WriteObj(path);
    }

    static void WriteCsv(string path, List<Variant> variants)
    {
        var ci = CultureInfo.InvariantCulture;
        var sb = new StringBuilder("mode,budget,tris,fallback,pose,named,samples,dev_mean_pct,dev_p99_pct,rest_p99_pct,excess_p99_pct,normal_mean_deg,normal_p95_deg,flipped,shrunk,region_tris\n");
        foreach (var v in variants)
            foreach (var r in v.Rows)
                sb.AppendLine(string.Join(",", v.Mode, v.Budget, v.Mesh.TriCount, v.Fallback, $"\"{r.Pose}\"", r.Named, r.Samples,
                    r.DevMean.ToString("0.###", ci), r.DevP99.ToString("0.###", ci), r.RestP99.ToString("0.###", ci), r.ExcessP99.ToString("0.###", ci),
                    r.NrmMean.ToString("0.##", ci), r.NrmP95.ToString("0.##", ci), r.Flipped, r.Shrunk, r.RegionTris));
        File.WriteAllText(path, sb.ToString());
    }

    static string Markdown(Pipeline p, List<Variant> variants, Dictionary<Variant, (float excess, Pose pose)> worst, float H)
    {
        var ci = CultureInfo.InvariantCulture;
        string F(float x, string fmt = "0.00") => x.ToString(fmt, ci);
        var sb = new StringBuilder();
        sb.AppendLine($"# Deformation report");
        sb.AppendLine();
        sb.AppendLine($"Model height {F(H, "0.###")} units. Distances are % of height. Same weights for every variant (transferred from the dense reference).");
        sb.AppendLine();
        sb.AppendLine("- **rest p99**: how far the remesh is from the dense surface before posing (shape fidelity).");
        sb.AppendLine("- **excess p99**: how much *further* it gets when posed, at the same points (how cleanly it bends).");
        sb.AppendLine("- **collapsed**: triangles near the posed bones that flip or shrink below 20% area, per 1,000 triangles there.");
        sb.AppendLine();
        sb.AppendLine("**Bone poses** (each deform bone ±45° about its local X and Z):");
        sb.AppendLine();
        sb.AppendLine("| variant | tris | rest p99 | posed p99 (avg) | excess p99 (avg) | excess p99 (worst, pose) | normal err avg° | collapsed / 1k region tris |");
        sb.AppendLine("|---|---|---|---|---|---|---|---|");
        foreach (var v in variants.OrderBy(v => v.Budget).ThenBy(v => v.Mode))
        {
            var rs = v.Rows.Where(r => !r.Named).ToList();
            if (rs.Count == 0) continue;
            var w = rs.OrderByDescending(r => r.ExcessP99).First();
            float rate = 1000f * rs.Sum(r => r.Flipped + r.Shrunk) / Math.Max(1, rs.Sum(r => r.RegionTris));
            sb.AppendLine($"| {v.Name}{(v.Fallback ? " ⚠fallback" : "")} | {v.Mesh.TriCount} | {F(rs.Average(r => r.RestP99))} | {F(rs.Average(r => r.DevP99))} | {F(rs.Average(r => r.ExcessP99))} | {F(w.ExcessP99)} ({w.Pose}) | {F(rs.Average(r => r.NrmMean), "0.0")} | {F(rate, "0.0")} |");
        }
        var named = variants.SelectMany(v => v.Rows.Where(r => r.Named).Select(r => r.Pose)).Distinct().ToList();
        if (named.Count > 0)
        {
            sb.AppendLine();
            sb.AppendLine("**Named poses**, excess p99 % (normal err avg°, flipped+shrunk):");
            sb.AppendLine();
            sb.AppendLine("| variant | " + string.Join(" | ", named) + " |");
            sb.AppendLine("|---|" + string.Concat(named.Select(_ => "---|")));
            foreach (var v in variants.OrderBy(v => v.Budget).ThenBy(v => v.Mode))
            {
                sb.Append($"| {v.Name} |");
                foreach (var n in named)
                {
                    var r = v.Rows.FirstOrDefault(x => x.Named && x.Pose == n);
                    sb.Append(r == null ? " – |" : $" {F(r.ExcessP99)} ({F(r.NrmMean, "0.0")}°, {r.Flipped + r.Shrunk}) |");
                }
                sb.AppendLine();
            }
        }
        return sb.ToString();
    }
}
