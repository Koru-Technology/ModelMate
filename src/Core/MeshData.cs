using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>Plain-array triangle mesh. Safe to use off the main thread (no Godot objects).</summary>
public sealed class MeshData
{
    public Vector3[] Pos;
    public Vector3[] Nrm;
    public Vector2[] Uv;
    public float[] Tan;   // optional tangents, 4 floats per vertex (xyz, sign)
    public int[] Tris;
    public byte[] TriFlags; // optional per triangle: bit k set = edge (corner k → k+1) is a quad diagonal
    public bool[] Hard;     // optional per half-edge (3 per triangle): a sharp edge; normals split across it

    public int VertexCount => Pos.Length;
    public int TriCount => Tris.Length / 3;

    public MeshData(Vector3[] pos, int[] tris)
    {
        Pos = pos;
        Tris = tris;
    }

    public Aabb Bounds()
    {
        if (Pos.Length == 0) return new Aabb();
        Vector3 mn = Pos[0], mx = Pos[0];
        foreach (var p in Pos)
        {
            mn = mn.Min(p);
            mx = mx.Max(p);
        }
        return new Aabb(mn, mx - mn);
    }

    public Vector3 TriNormal(int t)
    {
        var a = Pos[Tris[3 * t]];
        var b = Pos[Tris[3 * t + 1]];
        var c = Pos[Tris[3 * t + 2]];
        return (b - a).Cross(c - a); // not normalized (length = 2*area)
    }

    /// <summary>Area-weighted smooth vertex normals.</summary>
    public void ComputeNormals()
    {
        var n = new Vector3[Pos.Length];
        for (int t = 0; t < TriCount; t++)
        {
            var fn = TriNormal(t);
            n[Tris[3 * t]] += fn;
            n[Tris[3 * t + 1]] += fn;
            n[Tris[3 * t + 2]] += fn;
        }
        for (int i = 0; i < n.Length; i++)
        {
            float l = n[i].Length();
            n[i] = l > 1e-20f ? n[i] / l : Vector3.Up;
        }
        Nrm = n;
    }

    /// <summary>Marks edges whose two faces turn by more than the angle (quad diagonals never) as hard.</summary>
    public int MarkHardEdges(float angleDeg)
    {
        int nt = TriCount, count = 0;
        Hard = new bool[3 * nt];
        float cosA = MathF.Cos(Mathf.DegToRad(angleDeg));
        var fn = new Vector3[nt];
        for (int t = 0; t < nt; t++) { var n = TriNormal(t); float l = n.Length(); fn[t] = l > 1e-30f ? n / l : Vector3.Zero; }
        var owner = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        for (int t = 0; t < nt; t++)
            for (int k = 0; k < 3; k++)
            {
                int a = Tris[3 * t + k], b = Tris[3 * t + (k + 1) % 3];
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                if (!owner.TryGetValue(key, out int h)) { owner[key] = 3 * t + k; continue; }
                int o = h / 3;
                bool diagonal = TriFlags != null && ((TriFlags[t] & (1 << k)) != 0 || (TriFlags[o] & (1 << (h % 3))) != 0);
                if (diagonal || fn[t] == Vector3.Zero || fn[o] == Vector3.Zero || fn[t].Dot(fn[o]) > cosA) continue;
                Hard[3 * t + k] = Hard[h] = true;
                count++;
            }
        return count;
    }

    /// <summary>
    /// Per corner (3 per triangle): smoothing group id and its normal. Corners of a vertex share a group unless a
    /// hard edge separates them; the normal is the angle-weighted mean of the group's faces. Without hard edges every
    /// vertex is one group.
    /// </summary>
    public (int[] group, Vector3[] normal) CornerNormals()
    {
        int nt = TriCount;
        var parent = new int[3 * nt];
        for (int i = 0; i < parent.Length; i++) parent[i] = i;
        int Find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
        var owner = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        for (int t = 0; t < nt; t++)
            for (int k = 0; k < 3; k++)
            {
                int a = Tris[3 * t + k], b = Tris[3 * t + (k + 1) % 3];
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                if (!owner.TryGetValue(key, out int h)) { owner[key] = 3 * t + k; continue; }
                if (Hard != null && (Hard[3 * t + k] || Hard[h])) continue;
                int o = h / 3;
                // join the corners of a and of b on both sides
                for (int q = 0; q < 3; q++)
                {
                    int x = Tris[3 * o + q];
                    if (x == a) { int r1 = Find(3 * t + k), r2 = Find(3 * o + q); if (r1 != r2) parent[r2] = r1; }
                    if (x == b) { int r1 = Find(3 * t + (k + 1) % 3), r2 = Find(3 * o + q); if (r1 != r2) parent[r2] = r1; }
                }
            }
        var group = new int[3 * nt];
        var sum = new Dictionary<int, Vector3>();
        for (int t = 0; t < nt; t++)
        {
            var n = TriNormal(t);
            float l = n.Length();
            if (l > 1e-30f) n /= l;
            for (int k = 0; k < 3; k++)
            {
                var p = Pos[Tris[3 * t + k]];
                var e1 = Pos[Tris[3 * t + (k + 1) % 3]] - p; var e2 = Pos[Tris[3 * t + (k + 2) % 3]] - p;
                float ang = (e1.LengthSquared() > 0 && e2.LengthSquared() > 0) ? e1.AngleTo(e2) : 0;
                int r = Find(3 * t + k);
                group[3 * t + k] = r;
                sum[r] = (sum.TryGetValue(r, out var s) ? s : Vector3.Zero) + n * ang;
            }
        }
        var normal = new Vector3[3 * nt];
        for (int c = 0; c < normal.Length; c++)
        {
            var s = sum[group[c]];
            normal[c] = s.LengthSquared() > 1e-30f ? s.Normalized() : Vector3.Up;
        }
        return (group, normal);
    }

    /// <summary>Compressed-sparse-row vertex adjacency (unique neighbors).</summary>
    public (int[] start, int[] nbr) BuildAdjacency()
    {
        int v = Pos.Length;
        var sets = new HashSet<int>[v];
        for (int i = 0; i < v; i++) sets[i] = new HashSet<int>();
        for (int t = 0; t < TriCount; t++)
        {
            int a = Tris[3 * t], b = Tris[3 * t + 1], c = Tris[3 * t + 2];
            sets[a].Add(b); sets[a].Add(c);
            sets[b].Add(a); sets[b].Add(c);
            sets[c].Add(a); sets[c].Add(b);
        }
        var start = new int[v + 1];
        for (int i = 0; i < v; i++) start[i + 1] = start[i] + sets[i].Count;
        var nbr = new int[start[v]];
        for (int i = 0; i < v; i++)
        {
            int k = start[i];
            foreach (var j in sets[i]) nbr[k++] = j;
        }
        return (start, nbr);
    }

    /// <summary>Drops vertices not referenced by any triangle.</summary>
    public void CompactVertices()
    {
        var remap = new int[Pos.Length];
        Array.Fill(remap, -1);
        int n = 0;
        for (int i = 0; i < Tris.Length; i++)
        {
            int v = Tris[i];
            if (remap[v] < 0) remap[v] = n++;
            Tris[i] = remap[v];
        }
        var pos = new Vector3[n];
        Vector3[] nrm = Nrm != null ? new Vector3[n] : null;
        Vector2[] uv = Uv != null ? new Vector2[n] : null;
        for (int i = 0; i < remap.Length; i++)
        {
            int r = remap[i];
            if (r < 0) continue;
            pos[r] = Pos[i];
            if (nrm != null) nrm[r] = Nrm[i];
            if (uv != null) uv[r] = Uv[i];
        }
        Pos = pos; Nrm = nrm; Uv = uv;
    }
}

/// <summary>
/// Imported high-poly source: all meshes merged into one world-space triangle mesh. Geometry only: the file's
/// materials, textures and vertex colours are not read (ModelMate prepares meshes for a PBR painter).
/// </summary>
public sealed class SourceMesh
{
    public MeshData Mesh;   // Pos/Nrm/Tris
}
