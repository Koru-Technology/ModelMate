using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

public enum CenterMode { None, Spine, Limb, ZOnly, XOnly }

public sealed class RigBone
{
    public string Name;
    public int Parent = -1;
    public Vector3 Head;
    public int PrimaryChild = -1;   // tail = child's head when set
    public Vector3 TailEnd;         // used for leaf bones
    public bool Deform = true;
    public CenterMode Center = CenterMode.None;
}

/// <summary>Editable skeleton description (joint positions in model space).</summary>
public sealed class RigSkeleton
{
    public List<RigBone> Bones = new();

    public int Find(string name) => Bones.FindIndex(b => b.Name == name);

    public Vector3 Tail(int i)
    {
        var b = Bones[i];
        return b.PrimaryChild >= 0 ? Bones[b.PrimaryChild].Head : b.TailEnd;
    }

    /// <summary>Global rest: origin at head, +Y toward tail, +Z toward character forward (+Z) or down for arms.</summary>
    public Transform3D GlobalRest(int i)
    {
        var b = Bones[i];
        var dir = Tail(i) - b.Head;
        if (dir.LengthSquared() < 1e-12f) dir = Vector3.Up;
        bool arm = b.Name.Contains("Arm") || b.Name.Contains("Hand") || b.Name.Contains("Shoulder");
        var refZ = arm ? Vector3.Down : Vector3.Back; // Godot: Back = +Z
        return new Transform3D(Geo.BasisYAlong(dir, refZ), b.Head);
    }

    public RigSkeleton Clone()
    {
        var s = new RigSkeleton();
        foreach (var b in Bones)
            s.Bones.Add(new RigBone
            {
                Name = b.Name, Parent = b.Parent, Head = b.Head, PrimaryChild = b.PrimaryChild,
                TailEnd = b.TailEnd, Deform = b.Deform, Center = b.Center,
            });
        return s;
    }
}

/// <summary>Humanoid template using SkeletonProfileHumanoid bone names. +Y up, +Z forward, Left = +X.</summary>
public static class HumanoidTemplate
{
    public static RigSkeleton Single(Aabb box)
    {
        var s = new RigSkeleton();
        var c = box.GetCenter();
        s.Bones.Add(new RigBone
        {
            Name = "Root", Head = new Vector3(c.X, box.Position.Y, c.Z),
            TailEnd = new Vector3(c.X, box.End.Y, c.Z),
        });
        return s;
    }

    public static RigSkeleton Fit(Aabb box, Bvh bvh, Action<string> log)
    {
        float h = box.Size.Y;
        var c = box.GetCenter();
        float baseY = box.Position.Y;
        Vector3 P(float x, float y, float z) => new(c.X + x * h, baseY + y * h, c.Z + z * h);

        var s = new RigSkeleton();
        int Add(string name, int parent, Vector3 head, CenterMode cm, bool deform = true)
        {
            s.Bones.Add(new RigBone { Name = name, Parent = parent, Head = head, Center = cm, Deform = deform });
            return s.Bones.Count - 1;
        }

        // --- arm pose detection: horizontal reach from shoulder joint to AABB side ---
        float halfW = MathF.Max(box.End.X - c.X, c.X - box.Position.X);
        float shoulderX = 0.10f * h, armY = 0.80f;
        float L = 0.42f * h;
        float reach = halfW - shoulderX;
        float cos = Math.Clamp(reach / L, 0.2f, 1f);
        float armScale = 1f;
        if (reach >= L) { cos = 1f; armScale = reach / L; }     // T-pose: stretch chain to fingertips
        float sin = MathF.Sqrt(1 - cos * cos);
        string pose = cos > 0.95f ? "T-pose" : $"A-pose (~{Mathf.RadToDeg(MathF.Acos(cos)):F0}° down)";
        log?.Invoke($"Arm pose: {pose}");

        int root = Add("Root", -1, P(0, 0, 0), CenterMode.None, deform: false);
        int hips = Add("Hips", root, P(0, 0.53f, 0), CenterMode.Spine);
        int spine = Add("Spine", hips, P(0, 0.60f, 0), CenterMode.Spine);
        int chest = Add("Chest", spine, P(0, 0.67f, 0), CenterMode.Spine);
        int upper = Add("UpperChest", chest, P(0, 0.74f, 0), CenterMode.Spine);
        int neck = Add("Neck", upper, P(0, 0.83f, 0), CenterMode.Spine);
        int head = Add("Head", neck, P(0, 0.88f, 0), CenterMode.Spine);
        s.Bones[head].TailEnd = P(0, 1.0f, 0);

        foreach (var (side, sx) in new[] { ("Left", 1f), ("Right", -1f) })
        {
            var dir = new Vector3(sx * cos, -sin, 0);
            var shoulderPos = P(sx * 0.10f, armY, 0);
            int sh = Add(side + "Shoulder", upper, P(sx * 0.025f, armY - 0.01f, 0), CenterMode.ZOnly);
            int ua = Add(side + "UpperArm", sh, shoulderPos, CenterMode.ZOnly);
            int la = Add(side + "LowerArm", ua, shoulderPos + dir * (0.17f * h * armScale), CenterMode.Limb);
            int hand = Add(side + "Hand", la, shoulderPos + dir * (0.32f * h * armScale), CenterMode.Limb);
            s.Bones[hand].TailEnd = shoulderPos + dir * (0.42f * h * armScale);

            int ul = Add(side + "UpperLeg", hips, P(sx * 0.09f, 0.50f, 0), CenterMode.ZOnly);
            int ll = Add(side + "LowerLeg", ul, P(sx * 0.09f, 0.28f, 0), CenterMode.Limb);
            int ft = Add(side + "Foot", ll, P(sx * 0.09f, 0.05f, -0.01f), CenterMode.XOnly);
            int toe = Add(side + "Toes", ft, P(sx * 0.09f, 0.015f, 0.07f), CenterMode.XOnly);
            s.Bones[toe].TailEnd = P(sx * 0.09f, 0.015f, 0.12f);
        }

        // primary children
        void Chain(string a, string b) => s.Bones[s.Find(a)].PrimaryChild = s.Find(b);
        Chain("Root", "Hips"); Chain("Hips", "Spine"); Chain("Spine", "Chest"); Chain("Chest", "UpperChest");
        Chain("UpperChest", "Neck"); Chain("Neck", "Head");
        foreach (var side in new[] { "Left", "Right" })
        {
            Chain(side + "Shoulder", side + "UpperArm"); Chain(side + "UpperArm", side + "LowerArm");
            Chain(side + "LowerArm", side + "Hand");
            Chain(side + "UpperLeg", side + "LowerLeg"); Chain(side + "LowerLeg", side + "Foot");
            Chain(side + "Foot", side + "Toes");
        }

        if (bvh != null) CenterInVolume(s, bvh, h, log);
        return s;
    }

    /// <summary>Moves joints to the middle of the mesh cross-section along chosen axes.</summary>
    public static void CenterInVolume(RigSkeleton s, Bvh bvh, float height, Action<string> log)
    {
        float maxShift = 0.06f * height;
        float maxWidth = 0.30f * height;  // wider cross-sections are not this joint's limb
        var hits = new List<(float t, int sign)>();
        int moved = 0;

        // Hand/toe tail ends move with their joints so the leaf direction is preserved.
        for (int i = 0; i < s.Bones.Count; i++)
        {
            var b = s.Bones[i];
            if (b.Center == CenterMode.None) continue;
            Vector3 dir = (s.Tail(i) - b.Head).Normalized();
            var axes = new List<(Vector3 axis, float backBias)>();
            switch (b.Center)
            {
                case CenterMode.Spine: axes.Add((Vector3.Right, 0)); axes.Add((Vector3.Back, 0.1f)); break;
                case CenterMode.ZOnly: axes.Add((Vector3.Back, 0)); break;
                case CenterMode.XOnly: axes.Add((Vector3.Right, 0)); break;
                case CenterMode.Limb:
                {
                    var bas = Geo.BasisYAlong(dir, MathF.Abs(dir.Y) > 0.7f ? Vector3.Back : Vector3.Up);
                    axes.Add((bas.X, 0)); axes.Add((bas.Z, 0));
                    break;
                }
            }
            var p = b.Head;
            foreach (var (axis, bias) in axes)
            {
                if (Interval(bvh, p, axis, height, hits, out float lo, out float hi) &&
                    (b.Center == CenterMode.Spine || hi - lo < maxWidth))
                {
                    float mid = (lo + hi) * 0.5f - bias * (hi - lo);
                    float cur = p.Dot(axis);
                    float shift = Math.Clamp(mid - cur, -maxShift, maxShift);
                    p += axis * shift;
                }
            }
            var delta = p - b.Head;
            if (delta.LengthSquared() > 0) moved++;
            b.Head = p;
            if (b.PrimaryChild < 0) b.TailEnd += delta;
        }
        log?.Invoke($"Centered {moved} joints in volume");
    }

    /// <summary>
    /// Inside interval along the line through p in direction axis, nearest to p. Casts 3 slightly jittered
    /// parallel rays (a ray exactly through a shared edge can miss a crossing and merge two parts) and
    /// requires at least two to agree; returns the median.
    /// </summary>
    static bool Interval(Bvh bvh, Vector3 p, Vector3 axis, float R, List<(float t, int sign)> hits, out float lo, out float hi)
    {
        lo = hi = 0;
        var perp1 = Geo.BasisYAlong(axis, Vector3.Up).X;
        var perp2 = axis.Cross(perp1);
        float j = 1e-4f * R;
        var jitters = new[] { perp1 * (0.37f * j) + perp2 * (0.11f * j), perp1 * (-0.29f * j) + perp2 * (0.53f * j), perp1 * (0.07f * j) - perp2 * (0.61f * j) };
        var res = new List<(float lo, float hi)>();
        foreach (var jt in jitters)
            if (Interval1(bvh, p + jt, axis, R, hits, out float l, out float h)) res.Add((l, h));
        if (res.Count < 2) return false;
        res.Sort((a, b) => (a.lo + a.hi).CompareTo(b.lo + b.hi));
        var med = res[res.Count / 2];
        int agree = 0;
        foreach (var r in res)
            if (MathF.Abs((r.lo + r.hi) - (med.lo + med.hi)) < 0.02f * R && MathF.Abs((r.hi - r.lo) - (med.hi - med.lo)) < 0.02f * R) agree++;
        if (agree < 2) return false;
        (lo, hi) = med;
        return true;
    }

    static bool Interval1(Bvh bvh, Vector3 p, Vector3 axis, float R, List<(float t, int sign)> hits, out float lo, out float hi)
    {
        lo = hi = 0;
        var o = p - axis * R;
        bvh.RaycastAll(o, axis, 2 * R, hits);
        if (hits.Count < 2) return false;
        float tp = R, best = float.MaxValue;
        int wind = 0;
        float segStart = 0;
        bool found = false;
        for (int k = 0; k < hits.Count; k++)
        {
            int before = wind;
            wind += hits[k].sign;
            if (before <= 0 && wind > 0) segStart = hits[k].t;
            if (before > 0 && wind <= 0)
            {
                float segEnd = hits[k].t;
                float d = tp < segStart ? segStart - tp : (tp > segEnd ? tp - segEnd : 0);
                if (d < best) { best = d; lo = segStart; hi = segEnd; found = true; }
            }
        }
        if (!found) return false;
        float pd = p.Dot(axis);
        lo = pd - R + lo;
        hi = pd - R + hi;
        return true;
    }
}
