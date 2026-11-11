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
