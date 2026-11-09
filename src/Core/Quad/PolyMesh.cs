using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>Polygon mesh (mostly quads) with counter-clockwise faces (outward normals, same convention as MeshData).</summary>
public sealed class PolyMesh
{
    public Vector3[] Pos;
    public Vector3[] Nrm;   // optional
    public List<int[]> Faces = new();

    /// <summary>Newell normal of a polygon (length ~ 2·area).</summary>
    public Vector3 FaceNormal(int[] f)
    {
        var n = Vector3.Zero;
        for (int i = 0; i < f.Length; i++)
        {
            var a = Pos[f[i]]; var b = Pos[f[(i + 1) % f.Length]];
            n += a.Cross(b);
        }
        return n;
    }

    /// <summary>Faces are all traced in the same rotational sense; flip them all if they disagree with the vertex normals.</summary>
    public void OrientConsistently()
    {
        if (Nrm == null || Faces.Count == 0) return;
        int agree = 0, disagree = 0;
        foreach (var f in Faces)
        {
            var avg = Vector3.Zero;
            foreach (var v in f) avg += Nrm[v];
            if (FaceNormal(f).Dot(avg) >= 0) agree++; else disagree++;
        }
        if (disagree > agree) foreach (var f in Faces) Array.Reverse(f);
    }

    public void Compact()
    {
        var remap = new int[Pos.Length];
        Array.Fill(remap, -1);
        int n = 0;
        foreach (var f in Faces)
            for (int i = 0; i < f.Length; i++)
            {
                if (remap[f[i]] < 0) remap[f[i]] = n++;
                f[i] = remap[f[i]];
            }
        var p = new Vector3[n];
        var nr = Nrm != null ? new Vector3[n] : null;
        for (int i = 0; i < remap.Length; i++)
        {
            if (remap[i] < 0) continue;
            p[remap[i]] = Pos[i];
            if (nr != null) nr[remap[i]] = Nrm[i];
        }
        Pos = p; Nrm = nr;
    }

    /// <summary>
    /// Triangulates: quads along the shorter diagonal, triangles as-is, larger polygons by fan.
    /// Returned mesh has TriFlags marking the diagonal (non-quad) edges.
    /// </summary>
    public MeshData Triangulate()
    {
        var tris = new List<int>();
        var flags = new List<byte>();
        void Tri(int a, int b, int c, byte f) { tris.Add(a); tris.Add(b); tris.Add(c); flags.Add(f); }
        foreach (var f in Faces)
        {
            if (f.Length == 3) Tri(f[0], f[1], f[2], 0);
            else if (f.Length == 4)
            {
                int a = f[0], b = f[1], c = f[2], d = f[3];
                if ((Pos[a] - Pos[c]).LengthSquared() <= (Pos[b] - Pos[d]).LengthSquared())
                {
                    Tri(a, b, c, 1 << 2);   // edge c→a is the diagonal
                    Tri(a, c, d, 1 << 0);   // edge a→c
                }
                else
                {
                    Tri(a, b, d, 1 << 1);   // edge b→d
                    Tri(b, c, d, 1 << 2);   // edge d→b
                }
            }
            else
            {
                for (int i = 1; i + 1 < f.Length; i++)
                {
                    byte fl = 0;
                    if (i > 1) fl |= 1 << 0;              // edge f0→fi is internal
                    if (i + 2 < f.Length) fl |= 1 << 2;   // edge f(i+1)→f0 is internal
                    Tri(f[0], f[i], f[i + 1], fl);
                }
            }
        }
        var m = new MeshData((Vector3[])Pos.Clone(), tris.ToArray()) { TriFlags = flags.ToArray() };
        return m;
    }

    /// <summary>Rebuilds polygons from a triangle mesh with diagonal flags (pairs of triangles sharing a diagonal).</summary>
    public static PolyMesh FromTriangles(MeshData m)
    {
        var pm = new PolyMesh { Pos = m.Pos, Nrm = m.Nrm };
        var flags = m.TriFlags ?? new byte[m.TriCount];
        var diagOwner = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        var done = new bool[m.TriCount];
        for (int t = 0; t < m.TriCount; t++)
            for (int k = 0; k < 3; k++)
                if ((flags[t] & (1 << k)) != 0)
                {
                    long key = Key(m.Tris[3 * t + k], m.Tris[3 * t + (k + 1) % 3]);
                    if (diagOwner.TryGetValue(key, out int o) && !done[o] && !done[t] && CountBits(flags[t]) == 1 && CountBits(flags[o]) == 1)
                    {
                        // t has diagonal a→c at edge k; merge with o into a quad
                        int a = m.Tris[3 * t + k], c = m.Tris[3 * t + (k + 1) % 3], b = m.Tris[3 * t + (k + 2) % 3];
                        int d = -1;
                        for (int q = 0; q < 3; q++) { int x = m.Tris[3 * o + q]; if (x != a && x != c) d = x; }
                        pm.Faces.Add(new[] { c, b, a, d });
                        done[t] = done[o] = true;
                    }
                    else diagOwner[key] = t;
                }
        for (int t = 0; t < m.TriCount; t++)
            if (!done[t]) pm.Faces.Add(new[] { m.Tris[3 * t], m.Tris[3 * t + 1], m.Tris[3 * t + 2] });
        return pm;
    }

    static int CountBits(byte b) { int c = 0; while (b != 0) { c += b & 1; b >>= 1; } return c; }
    static long Key(int a, int b) => a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

    public void WriteObj(string path)
    {
        var sb = new StringBuilder();
        var ci = CultureInfo.InvariantCulture;
        foreach (var p in Pos) sb.Append("v ").Append(p.X.ToString(ci)).Append(' ').Append(p.Y.ToString(ci)).Append(' ').Append(p.Z.ToString(ci)).Append('\n');
        foreach (var f in Faces)
        {
            sb.Append('f');
            foreach (var v in f) sb.Append(' ').Append(v + 1);
            sb.Append('\n');
        }
        File.WriteAllText(path, sb.ToString());
    }
}

/// <summary>Topology/quality statistics of a quad-dominant mesh (computed from triangles + diagonal flags).</summary>
public sealed class QuadStats
{
    public int Vertices, Edges, Quads, Triangles, Other, Irregular, Valence4, BoundaryEdges;
    public int Faces => Quads + Triangles + Other;
    public int EulerChar => Vertices - Edges + Faces;
    public float QuadFraction => Faces > 0 ? Quads / (float)Faces : 0;
    public float Valence4Fraction => Vertices > 0 ? Valence4 / (float)Vertices : 0;
    public int[] Valence;   // per vertex, counting only real (non-diagonal) edges

    public static QuadStats Compute(MeshData m)
    {
        var pm = PolyMesh.FromTriangles(m);
        var s = new QuadStats { Vertices = m.VertexCount };
        foreach (var f in pm.Faces)
        {
            if (f.Length == 4) s.Quads++;
            else if (f.Length == 3) s.Triangles++;
            else s.Other++;
        }
        var edgeCount = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        foreach (var f in pm.Faces)
            for (int i = 0; i < f.Length; i++)
            {
                long k = Key(f[i], f[(i + 1) % f.Length]);
                edgeCount[k] = edgeCount.TryGetValue(k, out int c) ? c + 1 : 1;
            }
        s.Edges = edgeCount.Count;
        s.Valence = new int[m.VertexCount];
        foreach (var kv in edgeCount)
        {
            s.Valence[(int)(kv.Key >> 32)]++;
            s.Valence[(int)(kv.Key & 0xffffffff)]++;
            if (kv.Value == 1) s.BoundaryEdges++;
        }
        foreach (var v in s.Valence) { if (v == 4) s.Valence4++; else s.Irregular++; }
        return s;
    }

    static long Key(int a, int b) => a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

    /// <summary>Unique real (non-diagonal) edges.</summary>
    public static List<(int a, int b)> RealEdges(MeshData m)
    {
        var flags = m.TriFlags ?? new byte[m.TriCount];
        var set = new HashSet<long>(EdgeKeyComparer.Instance);
        var list = new List<(int, int)>();
        for (int t = 0; t < m.TriCount; t++)
            for (int k = 0; k < 3; k++)
            {
                if ((flags[t] & (1 << k)) != 0) continue;
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                if (set.Add(Key(a, b))) list.Add((a, b));
            }
        return list;
    }

    /// <summary>CSR adjacency over real (non-diagonal) edges, for quad-preserving smoothing.</summary>
    public static (int[] start, int[] nbr) RealAdjacency(MeshData m)
    {
        var edges = RealEdges(m);
        var deg = new int[m.VertexCount + 1];
        foreach (var (a, b) in edges) { deg[a + 1]++; deg[b + 1]++; }
        for (int i = 0; i < m.VertexCount; i++) deg[i + 1] += deg[i];
        var fill = (int[])deg.Clone();
        var nbr = new int[deg[m.VertexCount]];
        foreach (var (a, b) in edges) { nbr[fill[a]++] = b; nbr[fill[b]++] = a; }
        return (deg, nbr);
    }

    /// <summary>
    /// Edge-flow alignment: over real edges at qualifying vertices, the share whose tangent-plane angle to the
    /// given axis (folded modulo 90°) is within tolDeg. Random directions score ≈ 2·tol/90.
    /// </summary>
    public static float Alignment(MeshData m, Func<int, Vector3?> axisAt, float tolDeg, out int samples)
    {
        if (m.Nrm == null) m.ComputeNormals();
        int ok = 0; samples = 0;
        foreach (var (a, b) in RealEdges(m))
        {
            foreach (var v in new[] { a, b })
            {
                var ax = axisAt(v);
                if (ax == null) continue;
                var n = m.Nrm[v];
                var e = m.Pos[a == v ? b : a] - m.Pos[v];
                e -= n * n.Dot(e);
                var d = ax.Value - n * n.Dot(ax.Value);
                if (e.LengthSquared() < 1e-20f || d.LengthSquared() < 1e-6f) continue;
                float ang = Mathf.RadToDeg(MathF.Acos(Math.Clamp(MathF.Abs(e.Normalized().Dot(d.Normalized())), 0f, 1f))); // 0..90
                float folded = MathF.Min(ang, 90 - ang);
                samples++;
                if (folded <= tolDeg) ok++;
            }
        }
        return samples > 0 ? ok / (float)samples : 0;
    }

    public override string ToString() =>
        $"V={Vertices} F={Faces} (quads {Quads}, tris {Triangles}, other {Other}) quad%={QuadFraction:P1} " +
        $"valence4={Valence4Fraction:P1} irregular={Irregular} boundaryEdges={BoundaryEdges} χ={EulerChar}";
}
