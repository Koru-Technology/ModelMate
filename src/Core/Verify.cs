using System;
using System.Collections.Generic;
using System.Linq;
using Godot;

namespace ModelMate.Core;

public sealed class CheckList
{
    public readonly List<(bool ok, string msg)> Items = new();
    public void Check(bool ok, string msg) => Items.Add((ok, msg));
    public bool AllOk => Items.All(i => i.ok);
    public void Print(Action<string> log)
    {
        foreach (var (ok, msg) in Items) log((ok ? "  [PASS] " : "  [FAIL] ") + msg);
    }
}

public static class Verify
{
    /// <summary>Checks on the final in-memory data before export.</summary>
    public static void PreExport(CheckList cl, MeshData welded, MeshData split, int[] origin, int[] bones4, float[] w4, int targetTris)
    {
        int bad = MeshCleanup.CountBadEdges(welded);
        cl.Check(bad == 0, $"welded mesh manifold (edges with !=2 faces: {bad})");
        if (targetTris > 0)
            cl.Check(welded.TriCount <= targetTris * 1.02f, $"triangle budget {welded.TriCount} <= {targetTris}");

        bool uvOk = split.Uv.All(u => u.X >= -1e-4f && u.Y >= -1e-4f && u.X <= 1.0001f && u.Y <= 1.0001f);
        cl.Check(uvOk, "UVs within [0,1]");

        int badSum = 0, nonFinite = 0;
        for (int v = 0; v < split.VertexCount; v++)
        {
            float s = 0;
            for (int k = 0; k < 4; k++) { float x = w4[4 * v + k]; if (!float.IsFinite(x)) nonFinite++; s += x; }
            if (MathF.Abs(s - 1) > 1e-3f) badSum++;
        }
        cl.Check(badSum == 0 && nonFinite == 0, $"weights sum to 1, max 4 influences (bad={badSum}, nonfinite={nonFinite})");

        // seam consistency: all split copies of one welded vertex carry identical attributes
        var first = new Dictionary<int, int>();
        int seamBad = 0, seamPairs = 0;
        for (int v = 0; v < split.VertexCount; v++)
        {
            if (!first.TryGetValue(origin[v], out int f)) { first[origin[v]] = v; continue; }
            seamPairs++;
            bool same = split.Pos[v] == split.Pos[f] && split.Nrm[v] == split.Nrm[f];
            for (int k = 0; k < 4 && same; k++)
                same = bones4[4 * v + k] == bones4[4 * f + k] && w4[4 * v + k] == w4[4 * f + k];
            if (!same) seamBad++;
        }
        cl.Check(seamBad == 0, $"seam consistency ({seamPairs} split copies, {seamBad} mismatched)");
    }

    /// <summary>Quad-mode structure and fidelity checks (no-op in voxel mode unless a fallback happened).</summary>
    public static void QuadMesh(CheckList cl, Pipeline p)
    {
        if (p.S.Mode != RetopoMode.Quad) return;
        cl.Check(!p.UsedFallback, $"retopo mode = quad (fallback: {p.FallbackReason ?? "none"})");
        if (p.Quad == null || p.UsedFallback) return;
        var s = p.Quad.Stats;
        float rho = p.Quad.Rho;
        cl.Check(s.QuadFraction >= 0.90f, $"quads {s.QuadFraction:P1} >= 90% ({s})");
        cl.Check(s.Valence4Fraction >= 0.85f, $"valence-4 vertices {s.Valence4Fraction:P1} >= 85%");
        cl.Check(s.BoundaryEdges == 0, $"no boundary edges ({s.BoundaryEdges})");
        int baseChi = EulerChar(p.BaseMesh);
        cl.Check(s.EulerChar == baseChi, $"topology preserved (χ quad {s.EulerChar} = χ base {baseChi})");
        int target = p.S.TargetQuads;
        float faceEq = s.Quads + 0.5f * s.Triangles;
        cl.Check(MathF.Abs(faceEq - target) <= 0.2f * target, $"face count {faceEq:F0} quad-equivalents within ±20% of {target}");
        // Fidelity, measured base-surface → quad mesh. (Quad vertices are snapped onto the source, so measuring
        // vertex → source would always read ~0 and miss chords that cut across curved or sharp regions.)
        var m = p.Welded;
        var qb = new Bvh(m.Pos, m.Tris);
        var bm = p.BaseMesh;
        double sum = 0; float max = 0;
        for (int v = 0; v < bm.VertexCount; v++)
        {
            float d = qb.Closest(bm.Pos[v], 10 * rho, null, out var hit) ? hit.Dist : 10 * rho;
            sum += d; max = MathF.Max(max, d);
        }
        float mean = (float)(sum / bm.VertexCount);
        cl.Check(mean < 0.15f * rho && max < 0.75f * rho, $"surface deviation (base → quad): mean {mean / rho:F3}ρ < 0.15ρ, max {max / rho:F3}ρ < 0.75ρ");
    }

    static int EulerChar(MeshData m)
    {
        var edges = new HashSet<long>(EdgeKeyComparer.Instance);
        for (int t = 0; t < m.TriCount; t++)
            for (int k = 0; k < 3; k++)
            {
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                edges.Add(a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a);
            }
        return m.VertexCount - edges.Count + m.TriCount;
    }

    public static void BakedTexture(CheckList cl, byte[] rgba)
    {
        long sum = 0; int distinct = 0; var seen = new HashSet<int>();
        for (int i = 0; i < rgba.Length; i += 4)
        {
            sum += rgba[i] + rgba[i + 1] + rgba[i + 2];
            if (seen.Count < 64 && seen.Add(rgba[i] << 16 | rgba[i + 1] << 8 | rgba[i + 2])) distinct++;
        }
        cl.Check(sum > 0 && distinct > 1, $"baked texture non-empty ({distinct}+ distinct colors)");
    }

    /// <summary>Re-imports the exported glb, poses LeftUpperArm, CPU-skins and checks deformation.</summary>
    public static void RoundTrip(CheckList cl, string path, bool humanoid, bool expectNormalMap = false)
    {
        var doc = new GltfDocument();
        var state = new GltfState();
        var err = doc.AppendFromFile(path, state);
        cl.Check(err == Error.Ok, $"re-import {path} ({err})");
        if (err != Error.Ok) return;
        var root = doc.GenerateScene(state);
        try
        {
            var skel = FindFirst<Skeleton3D>(root);
            var mi = FindFirst<MeshInstance3D>(root);
            cl.Check(skel != null && mi != null && mi.Skin != null, "re-imported scene has Skeleton3D + skinned MeshInstance3D");
            if (skel == null || mi?.Skin == null) return;

            var mat = mi.Mesh.SurfaceGetMaterial(0) as BaseMaterial3D ?? mi.GetActiveMaterial(0) as BaseMaterial3D;
            cl.Check(mat?.AlbedoTexture != null, "re-imported material has albedo texture");
            if (expectNormalMap)
                cl.Check(mat != null && mat.NormalEnabled && mat.NormalTexture != null, "re-imported material has normal map");

            int nb = skel.GetBoneCount();
            var names = Enumerable.Range(0, nb).Select(skel.GetBoneName).ToList();
            if (humanoid)
            {
                string[] required = { "Hips", "Spine", "Chest", "UpperChest", "Neck", "Head", "LeftUpperArm", "LeftLowerArm",
                    "LeftHand", "RightUpperArm", "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "RightFoot" };
                var miss = required.Where(r => !names.Contains(r)).ToList();
                cl.Check(miss.Count == 0, $"humanoid bone names present (missing: {string.Join(",", miss)})");
                if (miss.Count > 0) return;

                // every bone must exist in SkeletonProfileHumanoid with a matching parent (for BoneMap retargeting)
                var profile = new SkeletonProfileHumanoid();
                var bad = new List<string>();
                for (int i = 0; i < nb; i++)
                {
                    int pi = profile.FindBone(names[i]);
                    if (pi < 0) { bad.Add(names[i] + "(unknown)"); continue; }
                    int sp = skel.GetBoneParent(i);
                    string skelParent = sp >= 0 ? names[sp] : "";
                    string profParent = profile.GetBoneParent(pi).ToString();
                    if (skelParent != profParent) bad.Add($"{names[i]}(parent {skelParent} != {profParent})");
                }
                cl.Check(bad.Count == 0, $"matches SkeletonProfileHumanoid hierarchy ({string.Join(", ", bad)})");
            }

            var rest = new Transform3D[nb];
            for (int i = 0; i < nb; i++)
            {
                int p = skel.GetBoneParent(i);
                rest[i] = (p >= 0 ? rest[p] : Transform3D.Identity) * skel.GetBoneRest(i);
            }

            if (humanoid)
            {
                int lua = names.IndexOf("LeftUpperArm"), lla = names.IndexOf("LeftLowerArm");
                cl.Check(rest[lua].Origin.X > 0, $"LeftUpperArm on +X side (x={rest[lua].Origin.X:F3})");
                int checkedBones = 0, badDir = 0;
                for (int i = 0; i < nb; i++)
                {
                    var kids = Enumerable.Range(0, nb).Where(j => skel.GetBoneParent(j) == i).ToList();
                    if (kids.Count != 1) continue;
                    var d = rest[kids[0]].Origin - rest[i].Origin;
                    if (d.Length() < 1e-5f) continue;
                    checkedBones++;
                    if (rest[i].Basis.Y.Normalized().Dot(d.Normalized()) < 0.99f) badDir++;
                }
                cl.Check(badDir == 0, $"bone local +Y points at child ({checkedBones} checked, {badDir} bad)");
            }

            // CPU skinning
            var arr = mi.Mesh.SurfaceGetArrays(0);
            var verts = arr[(int)Mesh.ArrayType.Vertex].AsVector3Array();
            var vb = arr[(int)Mesh.ArrayType.Bones].AsInt32Array();
            var vw = arr[(int)Mesh.ArrayType.Weights].AsFloat32Array();
            int wpv = vb.Length / verts.Length;
            var skin = mi.Skin;
            var bindBone = new int[skin.GetBindCount()];
            for (int b = 0; b < bindBone.Length; b++)
            {
                var bn = skin.GetBindName(b);
                bindBone[b] = bn != "" ? names.IndexOf(bn) : skin.GetBindBone(b);
            }

            Vector3[] Skin(Transform3D[] global)
            {
                var o = new Vector3[verts.Length];
                for (int v = 0; v < verts.Length; v++)
                {
                    Vector3 acc = Vector3.Zero;
                    for (int k = 0; k < wpv; k++)
                    {
                        float w = vw[v * wpv + k];
                        if (w == 0) continue;
                        int bind = vb[v * wpv + k];
                        acc += (global[bindBone[bind]] * skin.GetBindPose(bind) * verts[v]) * w;
                    }
                    o[v] = acc;
                }
                return o;
            }

            var restSkinned = Skin(rest);
            float maxRestErr = 0;
            for (int v = 0; v < verts.Length; v++) maxRestErr = MathF.Max(maxRestErr, (restSkinned[v] - verts[v]).Length());
            var aabb = new Aabb(verts[0], Vector3.Zero);
            foreach (var p in verts) aabb = aabb.Expand(p);
            float H = aabb.Size.Y;
            cl.Check(maxRestErr < 1e-3f * H, $"rest pose reproduces mesh (max err {maxRestErr:E2})");
            if (!humanoid) return;

            // pose: rotate LeftUpperArm 60° about its local Z
            int target = names.IndexOf("LeftUpperArm");
            var posed = new Transform3D[nb];
            for (int i = 0; i < nb; i++)
            {
                int p = skel.GetBoneParent(i);
                var local = skel.GetBoneRest(i);
                if (i == target) local = local * new Transform3D(new Basis(Vector3.Back, Mathf.DegToRad(60)), Vector3.Zero);
                posed[i] = (p >= 0 ? posed[p] : Transform3D.Identity) * local;
            }
            var posedSkinned = Skin(posed);

            float elbowX = rest[names.IndexOf("LeftLowerArm")].Origin.X;
            float rElbowX = rest[names.IndexOf("RightLowerArm")].Origin.X;
            var leftArm = new List<float>(); float rightMax = 0, feetMax = 0;
            for (int v = 0; v < verts.Length; v++)
            {
                float d = (posedSkinned[v] - restSkinned[v]).Length();
                if (verts[v].X > elbowX) leftArm.Add(d);
                if (verts[v].X < rElbowX) rightMax = MathF.Max(rightMax, d);
                if (verts[v].Y < aabb.Position.Y + 0.1f * H) feetMax = MathF.Max(feetMax, d);
            }
            leftArm.Sort();
            float med = leftArm.Count > 0 ? leftArm[leftArm.Count / 2] : 0;
            cl.Check(med > 0.05f * H, $"posed: left forearm/hand moves (median {med:F3}, n={leftArm.Count})");
            cl.Check(rightMax < 0.005f * H, $"posed: right arm stays put (max {rightMax:E2})");
            cl.Check(feetMax < 0.005f * H, $"posed: feet stay put (max {feetMax:E2})");

            // seam check: vertices sharing a position must stay together when posed
            var byPos = new Dictionary<Vector3, int>();
            float seamMax = 0; int seamPairs = 0;
            for (int v = 0; v < verts.Length; v++)
            {
                if (byPos.TryGetValue(verts[v], out int f))
                {
                    seamPairs++;
                    seamMax = MathF.Max(seamMax, (posedSkinned[v] - posedSkinned[f]).Length());
                }
                else byPos[verts[v]] = v;
            }
            cl.Check(seamMax < 1e-4f, $"posed: seams stay closed ({seamPairs} pairs, max gap {seamMax:E2})");
        }
        finally { root.Free(); }
    }

    static T FindFirst<T>(Node n) where T : Node
    {
        if (n is T t) return t;
        foreach (var c in n.GetChildren())
        {
            var r = FindFirst<T>(c);
            if (r != null) return r;
        }
        return null;
    }
}
