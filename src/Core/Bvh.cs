using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

public struct BvhHit
{
    public int Tri;
    public Vector3 Point;
    public float U, V, W;   // barycentric weights of tri verts a,b,c
    public float Dist;      // distance (closest query) or t (ray query)
}

/// <summary>Triangle BVH: closest-point and ray queries. Immutable after build; thread-safe queries.</summary>
public sealed class Bvh
{
    struct Node
    {
        public Vector3 Min, Max;
        public int Left;    // child index, or -1 for leaf
        public int Start, Count;
    }

    readonly Vector3[] _p;
    readonly int[] _t;
    readonly int[] _order;
    readonly Node[] _nodes;
    int _nodeCount;
    public readonly Vector3[] TriNormal; // unit normals

    const int LeafSize = 4;

    public Bvh(Vector3[] pos, int[] tris)
    {
        _p = pos;
        _t = tris;
        int n = tris.Length / 3;
        _order = new int[n];
        TriNormal = new Vector3[n];
        var cen = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            _order[i] = i;
            var a = pos[tris[3 * i]]; var b = pos[tris[3 * i + 1]]; var c = pos[tris[3 * i + 2]];
            cen[i] = (a + b + c) / 3f;
            var fn = (b - a).Cross(c - a);
            float l = fn.Length();
            TriNormal[i] = l > 0 ? fn / l : Vector3.Up;
        }
        _nodes = new Node[Math.Max(1, 2 * n)];
        var keys = new float[n];
        _nodeCount = 1;
        BuildNode(0, 0, n, cen, keys);
    }

    void BuildNode(int ni, int start, int count, Vector3[] cen, float[] keys)
    {
        Vector3 mn = new(float.MaxValue, float.MaxValue, float.MaxValue), mx = -mn;
        Vector3 cmn = mn, cmx = mx;
        for (int i = start; i < start + count; i++)
        {
            int t = _order[i];
            var a = _p[_t[3 * t]]; var b = _p[_t[3 * t + 1]]; var c = _p[_t[3 * t + 2]];
            mn = mn.Min(a).Min(b).Min(c);
            mx = mx.Max(a).Max(b).Max(c);
            cmn = cmn.Min(cen[t]); cmx = cmx.Max(cen[t]);
        }
        _nodes[ni].Min = mn; _nodes[ni].Max = mx;
        _nodes[ni].Start = start; _nodes[ni].Count = count;
        var ext = cmx - cmn;
        if (count <= LeafSize || ext.LengthSquared() == 0)
        {
            _nodes[ni].Left = -1;
            return;
        }
        int axis = ext.X > ext.Y ? (ext.X > ext.Z ? 0 : 2) : (ext.Y > ext.Z ? 1 : 2);
        for (int i = start; i < start + count; i++) keys[i] = cen[_order[i]][axis];
        int half = count / 2;
        Select(keys, _order, start, start + count - 1, start + half);
        int l = System.Threading.Interlocked.Add(ref _nodeCount, 2) - 2;
        _nodes[ni].Left = l;
        if (count > 20000)
            System.Threading.Tasks.Parallel.Invoke(
                () => BuildNode(l, start, half, cen, keys),
                () => BuildNode(l + 1, start + half, count - half, cen, keys));
        else
        {
            BuildNode(l, start, half, cen, keys);
            BuildNode(l + 1, start + half, count - half, cen, keys);
        }
    }

    /// <summary>Quickselect: partially orders keys/items so index k holds the k-th smallest key.</summary>
    static void Select(float[] keys, int[] items, int lo, int hi, int k)
    {
        while (hi > lo)
        {
            float pivot = keys[(lo + hi) >> 1];
            int i = lo, j = hi;
            while (i <= j)
            {
                while (keys[i] < pivot) i++;
                while (keys[j] > pivot) j--;
                if (i <= j)
                {
                    (keys[i], keys[j]) = (keys[j], keys[i]);
                    (items[i], items[j]) = (items[j], items[i]);
                    i++; j--;
                }
            }
            if (k <= j) hi = j;
            else if (k >= i) lo = i;
            else return;
        }
    }

    static float BoxDistSq(in Node n, Vector3 p)
    {
        float dx = MathF.Max(MathF.Max(n.Min.X - p.X, 0), p.X - n.Max.X);
        float dy = MathF.Max(MathF.Max(n.Min.Y - p.Y, 0), p.Y - n.Max.Y);
        float dz = MathF.Max(MathF.Max(n.Min.Z - p.Z, 0), p.Z - n.Max.Z);
        return dx * dx + dy * dy + dz * dz;
    }

    /// <summary>Closest point on any triangle passing <paramref name="filter"/> within maxDist.</summary>
    public bool Closest(Vector3 p, float maxDist, Func<int, bool> filter, out BvhHit hit)
    {
        hit = default;
        float best = maxDist * maxDist;
        bool found = false;
        Span<int> stack = stackalloc int[128];
        int sp = 0;
        stack[sp++] = 0;
        while (sp > 0)
        {
            ref Node n = ref _nodes[stack[--sp]];
            if (BoxDistSq(n, p) > best) continue;
            if (n.Left < 0)
            {
                for (int i = n.Start; i < n.Start + n.Count; i++)
                {
                    int t = _order[i];
                    if (filter != null && !filter(t)) continue;
                    var q = Geo.ClosestPtTri(p, _p[_t[3 * t]], _p[_t[3 * t + 1]], _p[_t[3 * t + 2]], out float u, out float v, out float w);
                    float d2 = (q - p).LengthSquared();
                    if (d2 < best)
                    {
                        best = d2; found = true;
                        hit.Tri = t; hit.Point = q; hit.U = u; hit.V = v; hit.W = w;
                    }
                }
            }
            else
            {
                int l = n.Left, r = n.Left + 1;
                float dl = BoxDistSq(_nodes[l], p), dr = BoxDistSq(_nodes[r], p);
                if (dl < dr) { stack[sp++] = r; stack[sp++] = l; }
                else { stack[sp++] = l; stack[sp++] = r; }
            }
        }
        if (found) hit.Dist = MathF.Sqrt(best);
        return found;
    }

    static bool RayBox(in Node n, Vector3 o, Vector3 invD, float tmax)
    {
        float t0 = 0, t1 = tmax;
        for (int a = 0; a < 3; a++)
        {
            float ta = (n.Min[a] - o[a]) * invD[a];
            float tb = (n.Max[a] - o[a]) * invD[a];
            if (ta > tb) (ta, tb) = (tb, ta);
            if (float.IsNaN(ta) || float.IsNaN(tb)) continue;
            t0 = MathF.Max(t0, ta);
            t1 = MathF.Min(t1, tb);
            if (t0 > t1) return false;
        }
        return true;
    }

    /// <summary>Nearest ray hit with t in [0, tmax].</summary>
    public bool Raycast(Vector3 o, Vector3 d, float tmax, out BvhHit hit, Func<int, bool> filter = null)
    {
        hit = default;
        var invD = new Vector3(1f / d.X, 1f / d.Y, 1f / d.Z);
        float best = tmax;
        bool found = false;
        Span<int> stack = stackalloc int[128];
        int sp = 0;
        stack[sp++] = 0;
        while (sp > 0)
        {
            ref Node n = ref _nodes[stack[--sp]];
            if (!RayBox(n, o, invD, best)) continue;
            if (n.Left < 0)
            {
                for (int i = n.Start; i < n.Start + n.Count; i++)
                {
                    int t = _order[i];
                    if (filter != null && !filter(t)) continue;
                    float th = Geo.RayTri(o, d, _p[_t[3 * t]], _p[_t[3 * t + 1]], _p[_t[3 * t + 2]], out float u, out float v, out float w);
                    if (th >= 0 && th < best)
                    {
                        best = th; found = true;
                        hit.Tri = t; hit.U = u; hit.V = v; hit.W = w;
                    }
                }
            }
            else
            {
                stack[sp++] = n.Left;
                stack[sp++] = n.Left + 1;
            }
        }
        if (found) { hit.Dist = best; hit.Point = o + d * best; }
        return found;
    }

    /// <summary>All hits along the ray (t >= 0): (t, sign) where sign=+1 entering (d·n &lt; 0), -1 exiting.</summary>
    public void RaycastAll(Vector3 o, Vector3 d, float tmax, List<(float t, int sign)> hits)
    {
        hits.Clear();
        var invD = new Vector3(1f / d.X, 1f / d.Y, 1f / d.Z);
        Span<int> stack = stackalloc int[128];
        int sp = 0;
        stack[sp++] = 0;
        while (sp > 0)
        {
            ref Node n = ref _nodes[stack[--sp]];
            if (!RayBox(n, o, invD, tmax)) continue;
            if (n.Left < 0)
            {
                for (int i = n.Start; i < n.Start + n.Count; i++)
                {
                    int t = _order[i];
                    float th = Geo.RayTri(o, d, _p[_t[3 * t]], _p[_t[3 * t + 1]], _p[_t[3 * t + 2]], out _, out _, out _);
                    if (th >= 0 && th <= tmax)
                        hits.Add((th, d.Dot(TriNormal[t]) < 0 ? 1 : -1));
                }
            }
            else
            {
                stack[sp++] = n.Left;
                stack[sp++] = n.Left + 1;
            }
        }
        hits.Sort((a, b) => a.t.CompareTo(b.t));
    }
}
