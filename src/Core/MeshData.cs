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

/// <summary>Raw RGBA8 texture, sampled on worker threads.</summary>
public sealed class SourceTexture
{
    public byte[] Rgba;
    public int W, H;

    Color Texel(int x, int y)
    {
        x = ((x % W) + W) % W;
        y = ((y % H) + H) % H;
        int i = 4 * (y * W + x);
        return new Color(Rgba[i] / 255f, Rgba[i + 1] / 255f, Rgba[i + 2] / 255f, Rgba[i + 3] / 255f);
    }

    public Color SampleBilinear(Vector2 uv)
    {
        float fx = uv.X * W - 0.5f, fy = uv.Y * H - 0.5f;
        int x0 = (int)MathF.Floor(fx), y0 = (int)MathF.Floor(fy);
        float tx = fx - x0, ty = fy - y0;
        var a = Texel(x0, y0).Lerp(Texel(x0 + 1, y0), tx);
        var b = Texel(x0, y0 + 1).Lerp(Texel(x0 + 1, y0 + 1), tx);
        return a.Lerp(b, ty);
    }
}

public sealed class SourceMaterial
{
    public Color Albedo = Colors.White;
    public SourceTexture Tex;
    public bool Transparent;   // alpha-blended overlay (e.g. a glossy duplicate shell): ignored when baking colour
}

/// <summary>Imported high-poly source: merged mesh in world space plus per-triangle material info.</summary>
public sealed class SourceMesh
{
    public MeshData Mesh;          // Pos/Nrm/Uv/Tris (Uv may be null)
    public Color[] Col;            // optional vertex colors
    public int[] TriMat;           // material index per triangle
    public List<SourceMaterial> Materials = new();

    public Color SampleColor(int tri, float u, float v, float w)
    {
        var m = Mesh;
        int a = m.Tris[3 * tri], b = m.Tris[3 * tri + 1], c = m.Tris[3 * tri + 2];
        var mat = Materials[TriMat[tri]];
        Color col = mat.Albedo;
        if (mat.Tex != null && m.Uv != null)
        {
            var uv = m.Uv[a] * u + m.Uv[b] * v + m.Uv[c] * w;
            col *= mat.Tex.SampleBilinear(uv);
        }
        if (Col != null)
        {
            col *= Col[a] * u + Col[b] * v + Col[c] * w;
        }
        return col;
    }
}
