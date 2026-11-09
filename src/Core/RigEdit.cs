using System;
using System.Collections.Generic;
using System.Linq;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Structural edits on a RigSkeleton. Every edit leaves the skeleton valid: one root at index 0, parents
/// before children, unique names, PrimaryChild links pointing at real children.
/// </summary>
public static class RigEdit
{
    public static string UniqueName(RigSkeleton s, string baseName)
    {
        if (s.Find(baseName) < 0) return baseName;
        for (int i = 1; ; i++)
        {
            var n = $"{baseName}.{i:000}";
            if (s.Find(n) < 0) return n;
        }
    }

    /// <summary>Adds a child bone starting at the parent's tail, continuing in the parent's direction.</summary>
    public static int AddChild(RigSkeleton s, int parent, string name, float minLength)
    {
        var p = s.Bones[parent];
        var head = s.Tail(parent);
        var dir = head - p.Head;
        float len = MathF.Max(dir.Length() * 0.5f, minLength);
        dir = dir.LengthSquared() > 1e-12f ? dir.Normalized() : Vector3.Up;
        var b = new RigBone { Name = UniqueName(s, name), Parent = parent, Head = head, TailEnd = head + dir * len };
        s.Bones.Add(b);
        Normalize(s);
        return s.Find(b.Name);
    }

    /// <summary>Removes a bone; its children move to its parent. The root cannot be removed.</summary>
    public static void Remove(RigSkeleton s, int b)
    {
        if (b <= 0 || b >= s.Bones.Count) throw new InvalidOperationException("The root bone can't be removed.");
        var bone = s.Bones[b];
        var parent = s.Bones[bone.Parent];
        if (parent.PrimaryChild == b) { parent.PrimaryChild = -1; parent.TailEnd = bone.Head; }
        foreach (var c in s.Bones) if (c.Parent == b) c.Parent = bone.Parent;
        Reindex(s, b);
        Normalize(s);
    }

    public static void Rename(RigSkeleton s, int b, string name)
    {
        name = name.Trim();
        if (name.Length == 0) throw new InvalidOperationException("Bone names can't be empty.");
        int existing = s.Find(name);
        if (existing >= 0 && existing != b) throw new InvalidOperationException($"A bone named {name} already exists.");
        s.Bones[b].Name = name;
    }

    public static bool IsDescendant(RigSkeleton s, int b, int ofBone)
    {
        for (int p = b; p >= 0; p = s.Bones[p].Parent)
            if (p == ofBone) return true;
        return false;
    }

    public static void Reparent(RigSkeleton s, int b, int newParent)
    {
        if (b == 0) throw new InvalidOperationException("The root bone can't be reparented.");
        if (IsDescendant(s, newParent, b)) throw new InvalidOperationException("That would create a cycle.");
        var bone = s.Bones[b];
        var old = s.Bones[bone.Parent];
        if (old.PrimaryChild == b) { old.PrimaryChild = -1; old.TailEnd = bone.Head; }
        bone.Parent = newParent;
        Normalize(s);
    }

    public static string MirrorName(string n) =>
        n.StartsWith("Left") ? "Right" + n[4..] : n.StartsWith("Right") ? "Left" + n[5..] : null;

    /// <summary>
    /// Copies every Left* bone onto its Right* partner, mirrored across the plane x = centerX, creating missing
    /// partners. Returns the number of bones created.
    /// </summary>
    public static int Symmetrize(RigSkeleton s, float centerX)
    {
        Vector3 M(Vector3 p) => new(2 * centerX - p.X, p.Y, p.Z);
        int created = 0;
        // parents come first, so partners of parents exist before their children are mirrored
        for (int i = 0; i < s.Bones.Count; i++)
        {
            var b = s.Bones[i];
            if (!b.Name.StartsWith("Left")) continue;
            string pn = MirrorName(b.Name);
            int pi = s.Find(pn);
            int parent = b.Parent;
            string parentMirror = MirrorName(s.Bones[parent].Name);
            if (parentMirror != null && s.Find(parentMirror) >= 0) parent = s.Find(parentMirror);
            if (pi < 0)
            {
                s.Bones.Add(new RigBone { Name = pn, Parent = parent, Deform = b.Deform, Center = b.Center });
                pi = s.Bones.Count - 1;
                created++;
            }
            var r = s.Bones[pi];
            r.Head = M(b.Head);
            r.TailEnd = M(s.Tail(i));
            r.Deform = b.Deform;
            r.Center = b.Center;
            if (r.Parent != parent && !IsDescendant(s, parent, pi)) r.Parent = parent;
        }
        // mirror primary-child links
        foreach (var b in s.Bones.Where(x => x.Name.StartsWith("Left") && x.PrimaryChild >= 0).ToList())
        {
            int r = s.Find(MirrorName(b.Name));
            int rc = s.Find(MirrorName(s.Bones[b.PrimaryChild].Name) ?? "");
            if (r >= 0 && rc >= 0 && s.Bones[rc].Parent == r) s.Bones[r].PrimaryChild = rc;
        }
        Normalize(s);
        return created;
    }

    /// <summary>Validity problems (empty when valid).</summary>
    public static List<string> Validate(RigSkeleton s)
    {
        var errs = new List<string>();
        if (s.Bones.Count == 0) { errs.Add("no bones"); return errs; }
        if (s.Bones[0].Parent != -1) errs.Add("bone 0 is not the root");
        for (int i = 1; i < s.Bones.Count; i++)
        {
            var b = s.Bones[i];
            if (b.Parent < 0) errs.Add($"{b.Name}: second root");
            else if (b.Parent >= i) errs.Add($"{b.Name}: parent after child");
            if (b.PrimaryChild >= 0 && (b.PrimaryChild >= s.Bones.Count || s.Bones[b.PrimaryChild].Parent != i))
                errs.Add($"{b.Name}: tail link to a bone that isn't its child");
        }
        foreach (var g in s.Bones.GroupBy(b => b.Name).Where(g => g.Count() > 1)) errs.Add($"duplicate name {g.Key}");
        return errs;
    }

    /// <summary>Removes bone `gone` and rewrites indices.</summary>
    static void Reindex(RigSkeleton s, int gone)
    {
        int Map(int i) => i < 0 || i == gone ? -1 : (i > gone ? i - 1 : i);
        s.Bones.RemoveAt(gone);
        foreach (var b in s.Bones) { b.Parent = Map(b.Parent); b.PrimaryChild = Map(b.PrimaryChild); }
    }

    /// <summary>Topologically orders bones (parents first, root at 0) and fixes stale tail links.</summary>
    public static void Normalize(RigSkeleton s)
    {
        int n = s.Bones.Count;
        var tails = new Vector3[n];
        for (int i = 0; i < n; i++) tails[i] = s.Tail(i);   // keep tails where they are across the reorder
        var order = new List<int>();
        var placed = new bool[n];
        void Visit(int i)
        {
            if (placed[i]) return;
            int p = s.Bones[i].Parent;
            if (p >= 0) Visit(p);
            placed[i] = true;
            order.Add(i);
        }
        int root = s.Bones.FindIndex(b => b.Parent < 0);
        if (root >= 0) Visit(root);
        for (int i = 0; i < n; i++) Visit(i);
        var map = new int[n];
        for (int k = 0; k < n; k++) map[order[k]] = k;
        var nb = new List<RigBone>(n);
        foreach (var i in order)
        {
            var b = s.Bones[i];
            b.Parent = b.Parent >= 0 ? map[b.Parent] : -1;
            b.PrimaryChild = b.PrimaryChild >= 0 ? map[b.PrimaryChild] : -1;
            b.TailEnd = tails[i];
            nb.Add(b);
        }
        s.Bones = nb;
        for (int i = 0; i < n; i++)
        {
            var b = s.Bones[i];
            if (b.PrimaryChild >= 0 && s.Bones[b.PrimaryChild].Parent != i) b.PrimaryChild = -1;
        }
    }
}
