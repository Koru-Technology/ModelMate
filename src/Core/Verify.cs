using System;
using System.Collections.Generic;
using System.Linq;
using Godot;

namespace ModelMate.Core;

public sealed class CheckList
{
    public readonly List<(bool ok, string msg)> Items = new();
    public readonly List<string> Notes = new();   // metrics and warnings: printed, never fail the run
    public void Check(bool ok, string msg) => Items.Add((ok, msg));
    public void Metric(string msg) => Notes.Add("  [METRIC] " + msg);
    public void Warn(string msg) => Notes.Add("  [WARN] " + msg);
    public bool AllOk => Items.All(i => i.ok);
    public void Print(Action<string> log)
    {
        foreach (var (ok, msg) in Items) log((ok ? "  [PASS] " : "  [FAIL] ") + msg);
        foreach (var n in Notes) log(n);
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

        if (bones4 == null)
        {
            // static mesh (no skeleton yet): seams must still join up
            int bad0 = 0, pairs0 = 0;
            var first0 = new Dictionary<int, int>();
            for (int v = 0; v < split.VertexCount; v++)
            {
                if (!first0.TryGetValue(origin[v], out int f)) { first0[origin[v]] = v; continue; }
                pairs0++;
                if (split.Pos[v] != split.Pos[f] || split.Nrm[v] != split.Nrm[f]) bad0++;
            }
            cl.Check(bad0 == 0, $"seam consistency ({pairs0} split copies, {bad0} mismatched)");
            return;
        }

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
        // Gate: catches broken extraction (the fallback triggers below 70%). Uniform spacing is typically ≥93% quads;
        // adaptive spacing trades some quads for shape fidelity (the triangles sit where density changes).
        cl.Check(s.QuadFraction >= 0.80f, $"quads {s.QuadFraction:P1} >= 80% ({s})");
        cl.Metric($"spacing: {p.SizingChosen ?? "uniform"}, quad share {s.QuadFraction:P1}");
        // Quality metric, not a gate: at low counts the fixed singularities at limb junctions dominate the fraction.
        cl.Metric($"valence-4 vertices {s.Valence4Fraction:P1} ({s.Irregular} irregular of {s.Vertices})");
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
        // deviations are measured in units of the LOCAL target spacing (uniform ρ unless a sizing field was used)
        var sz = p.Quad.Sizing;
        var dev = new float[bm.VertexCount];
        double sum = 0; float max = 0; Vector3 at = default; float maxAbs = 0;
        for (int v = 0; v < bm.VertexCount; v++)
        {
            float local = sz != null ? sz[v] : rho;
            float d = qb.Closest(bm.Pos[v], 10 * local, null, out var hit) ? hit.Dist : 10 * local;
            dev[v] = d / local * rho;   // expressed in ρ-equivalents so the thresholds below stay in ρ
            sum += dev[v];
            if (dev[v] > max) { max = dev[v]; at = bm.Pos[v]; maxAbs = d; }
        }
        float mean = (float)(sum / bm.VertexCount);
        Array.Sort(dev);
        float p995 = dev[Math.Min(dev.Length - 1, (int)(0.995 * dev.Length))];
        // Gate on the bulk of the surface; a few points far off are thin tips cut short, reported by name below.
        cl.Check(mean < 0.15f * rho && p995 < 0.5f * rho,
            $"surface deviation (base → quad): mean {mean / rho:F3}ρ < 0.15ρ, p99.5 {p995 / rho:F3}ρ < 0.5ρ");
        if (max >= 0.75f * rho)
            cl.Warn($"feature shortened: surface up to {max / rho:F2}× local spacing ({maxAbs:0.####} units) from the remesh at {at} — likely a thin tip; raise the budget to keep it");
        else cl.Metric($"max surface deviation {max / rho:F3}ρ at {at}");
    }

    /// <summary>
    /// Metric: how many sides each limb has, from the edges crossing a plane perpendicular to the bone at its
    /// midpoint (vertices mostly owned by that bone only). Quad mode counts real quad edges; voxel mode counts all
    /// triangle edges (the cross-section polygon's segment count).
    /// </summary>
    public static void LimbSides(CheckList cl, Pipeline p)
    {
        if (p.Rig == null || p.WeldedBones == null || !IsHumanoid(p.Rig)) return;
        var m = p.Welded;
        var edges = Quad.QuadStats.RealEdges(m);   // all edges when TriFlags is null (voxel mode)
        float W(int v, int bone)
        {
            float w = 0;
            for (int k = 0; k < 4; k++) if (p.WeldedBones[4 * v + k] == bone) w += p.WeldedWeights[4 * v + k];
            return w;
        }
        var parts = new List<string>();
        int min = int.MaxValue;
        foreach (var name in new[] { "LeftUpperArm", "LeftLowerArm", "RightUpperArm", "RightLowerArm", "LeftUpperLeg", "LeftLowerLeg", "RightUpperLeg", "RightLowerLeg" })
        {
            int b = p.Rig.Find(name);
            if (b < 0) continue;
            var a = p.Rig.Bones[b].Head; var t = p.Rig.Tail(b);
            var mid = (a + t) * 0.5f; var ax = (t - a).Normalized();
            int n = 0;
            foreach (var (u, v) in edges)
            {
                float su = (m.Pos[u] - mid).Dot(ax), sv = (m.Pos[v] - mid).Dot(ax);
                if (su * sv >= 0) continue;
                if (W(u, b) < 0.5f || W(v, b) < 0.5f) continue;
                n++;
            }
            parts.Add($"{name} {n}");
            min = Math.Min(min, n);
        }
        if (parts.Count > 0) cl.Metric($"limb sides: min {min} ({string.Join(", ", parts)})");
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

    /// <summary>A rig counts as humanoid when it has the core SkeletonProfileHumanoid bones (extra bones allowed).</summary>
    public static bool IsHumanoid(RigSkeleton r) => r != null &&
        new[] { "Hips", "Spine", "Head", "LeftUpperArm", "LeftLowerArm", "RightUpperArm", "RightLowerArm", "LeftUpperLeg", "RightUpperLeg" }
            .All(n => r.Find(n) >= 0);

    /// <summary>Re-imports the exported glb, poses LeftUpperArm, CPU-skins and checks deformation.</summary>
    /// <param name="rigged">false for a static-mesh export: only the mesh and its UVs are checked</param>
    public static void RoundTrip(CheckList cl, string path, bool humanoid, bool rigged = true)
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
            if (!rigged)
            {
                cl.Check(mi != null, "re-imported scene has a MeshInstance3D");
                if (mi == null) return;
                var a0 = mi.Mesh.SurfaceGetArrays(0);
                var uv0 = a0[(int)Mesh.ArrayType.TexUV].VariantType == Variant.Type.Nil ? null : a0[(int)Mesh.ArrayType.TexUV].AsVector2Array();
                cl.Check(uv0 != null && uv0.Length == a0[(int)Mesh.ArrayType.Vertex].AsVector3Array().Length, "re-imported mesh has UVs");
                return;
            }
            cl.Check(skel != null && mi != null && mi.Skin != null, "re-imported scene has Skeleton3D + skinned MeshInstance3D");
            if (skel == null || mi?.Skin == null) return;

            var uvArr = mi.Mesh.SurfaceGetArrays(0);
            var uvs = uvArr[(int)Mesh.ArrayType.TexUV].VariantType == Variant.Type.Nil ? null : uvArr[(int)Mesh.ArrayType.TexUV].AsVector2Array();
            cl.Check(uvs != null && uvs.Length == uvArr[(int)Mesh.ArrayType.Vertex].AsVector3Array().Length, "re-imported mesh has UVs");

            int nb = skel.GetBoneCount();
            var names = Enumerable.Range(0, nb).Select(skel.GetBoneName).ToList();
            if (humanoid)
            {
                string[] required = { "Hips", "Spine", "Chest", "UpperChest", "Neck", "Head", "LeftUpperArm", "LeftLowerArm",
                    "LeftHand", "RightUpperArm", "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "RightFoot" };
                var miss = required.Where(r => !names.Contains(r)).ToList();
                if (miss.Count > 0) cl.Warn($"humanoid rig is missing standard bones ({string.Join(", ", miss)}); retargeted animations won't drive them");

                // every bone must exist in SkeletonProfileHumanoid with a matching parent (for BoneMap retargeting)
                var profile = new SkeletonProfileHumanoid();
                var bad = new List<string>();
                var extra = new List<string>();
                for (int i = 0; i < nb; i++)
                {
                    int pi = profile.FindBone(names[i]);
                    if (pi < 0) { extra.Add(names[i]); continue; }   // extra bones (e.g. a tail) don't break retargeting
                    int sp = skel.GetBoneParent(i);
                    string skelParent = sp >= 0 ? names[sp] : "";
                    string profParent = profile.GetBoneParent(pi).ToString();
                    if (skelParent != profParent) bad.Add($"{names[i]}(parent {skelParent} != {profParent})");
                }
                cl.Check(bad.Count == 0, $"matches SkeletonProfileHumanoid hierarchy ({string.Join(", ", bad)})");
                if (extra.Count > 0) cl.Metric($"bones outside SkeletonProfileHumanoid (ignored by retargeting): {string.Join(", ", extra)}");
            }

            var rest = new Transform3D[nb];
            for (int i = 0; i < nb; i++)
            {
                int p = skel.GetBoneParent(i);
                rest[i] = (p >= 0 ? rest[p] : Transform3D.Identity) * skel.GetBoneRest(i);
            }

            // Bone orientation convention (any rig): a bone with exactly one child points its local +Y at it.
            {
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
            // Left/right convention (any rig with Left*/Right* bones): Left is on +X relative to its Right partner.
            {
                var wrong = new List<string>();
                for (int i = 0; i < nb; i++)
                {
                    if (!names[i].StartsWith("Left")) continue;
                    int r = names.IndexOf("Right" + names[i][4..]);
                    if (r >= 0 && rest[i].Origin.X <= rest[r].Origin.X) wrong.Add(names[i]);
                }
                if (names.Any(n => n.StartsWith("Left")))
                    cl.Check(wrong.Count == 0, $"Left* bones are on +X of their Right* partners ({string.Join(", ", wrong)})");
            }

            // CPU skinning
            var arr = mi.Mesh.SurfaceGetArrays(0);
            var verts = arr[(int)Mesh.ArrayType.Vertex].AsVector3Array();
            var vb = arr[(int)Mesh.ArrayType.Bones].AsInt32Array();
            var vw = arr[(int)Mesh.ArrayType.Weights].AsFloat32Array();
            int wpv = vb.Length / verts.Length;
            var skin = mi.Skin;
            int nbind = skin.GetBindCount();
            var bindBone = new int[nbind];
            var bindPose = new Transform3D[nbind];
            for (int b = 0; b < nbind; b++)
            {
                var bn = skin.GetBindName(b);
                bindBone[b] = bn != "" ? names.IndexOf(bn) : skin.GetBindBone(b);
                bindPose[b] = skin.GetBindPose(b);
            }
            Vector3[] SkinWith(Transform3D[] global)
            {
                var x = new Transform3D[nbind];
                for (int b = 0; b < nbind; b++) x[b] = global[bindBone[b]] * bindPose[b];
                return Skinning.Skin(verts, vb, vw, wpv, x);
            }
            Transform3D[] Posed(Func<int, Basis> rot) =>
                Skinning.Globals(nb, skel.GetBoneParent, i => skel.GetBoneRest(i) * new Transform3D(rot(i), Vector3.Zero));

            var restSkinned = SkinWith(rest);
            float maxRestErr = 0;
            for (int v = 0; v < verts.Length; v++) maxRestErr = MathF.Max(maxRestErr, (restSkinned[v] - verts[v]).Length());
            var aabb = new Aabb(verts[0], Vector3.Zero);
            foreach (var p in verts) aabb = aabb.Expand(p);
            float H = aabb.Size.Y;
            cl.Check(maxRestErr < 1e-3f * H, $"rest pose reproduces mesh (max err {maxRestErr:E2})");

            // Generic per-bone test: rotating one bone moves part of the mesh, and nothing outside the region weighted to
            // that bone or its descendants.
            {
                var influenced = new bool[nbind];
                var problems = new List<string>();
                int tested = 0;
                for (int b = 0; b < nb; b++)
                {
                    int bind = Array.IndexOf(bindBone, b);
                    if (bind < 0) continue;
                    // region: vertices with weight on b or any descendant
                    var region = new bool[verts.Length];
                    var inSub = new bool[nb];
                    for (int i = 0; i < nb; i++) { int q = i; while (q >= 0 && q != b) q = skel.GetBoneParent(q); inSub[i] = q == b; }
                    int regionCount = 0;
                    for (int v = 0; v < verts.Length; v++)
                        for (int k = 0; k < wpv; k++)
                            if (vw[v * wpv + k] > 0 && inSub[bindBone[vb[v * wpv + k]]]) { region[v] = true; regionCount++; break; }
                    if (regionCount == 0) continue;   // e.g. non-deform root
                    tested++;
                    var posedSk = SkinWith(Posed(i => i == b ? new Basis(Vector3.Right, Mathf.DegToRad(30)) : Basis.Identity));
                    int moved = 0; float outside = 0;
                    for (int v = 0; v < verts.Length; v++)
                    {
                        float d = (posedSk[v] - restSkinned[v]).Length();
                        if (region[v]) { if (d > 1e-3f * H) moved++; }
                        else outside = MathF.Max(outside, d);
                    }
                    if (moved == 0) problems.Add($"{names[b]} moves nothing");
                    if (outside > 1e-5f * H) problems.Add($"{names[b]} moves vertices outside its region ({outside:E1})");
                }
                cl.Check(problems.Count == 0, $"posed: each of {tested} weighted bones moves only its own region ({string.Join("; ", problems)})");
                // seams under a pose that bends every bone at once
                var all = SkinWith(Posed(i => new Basis(Vector3.Right, Mathf.DegToRad(20))));
                var (pairs, gap) = Skinning.SeamGap(verts, all);
                cl.Check(gap < 1e-4f * MathF.Max(1f, H), $"posed: seams stay closed with every bone bent ({pairs} pairs, max gap {gap:E2})");
            }

            if (humanoid)
            {
                int lua = names.IndexOf("LeftUpperArm");
                cl.Check(rest[lua].Origin.X > 0, $"LeftUpperArm on +X side (x={rest[lua].Origin.X:F3})");
                // pose: rotate LeftUpperArm 60° about its local Z
                int target = lua;
                var posed = Skinning.Globals(nb, skel.GetBoneParent, i =>
                    i == target ? skel.GetBoneRest(i) * new Transform3D(new Basis(Vector3.Back, Mathf.DegToRad(60)), Vector3.Zero) : skel.GetBoneRest(i));
                var posedSkinned = SkinWith(posed);
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
            }
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
