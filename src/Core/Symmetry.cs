using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core;

/// <summary>Mirror symmetry across the plane x = centerX: detection on meshes and rigs, and symmetric weights.</summary>
public static class Symmetry
{
    public static Vector3 Mirror(Vector3 p, float cx) => new(2 * cx - p.X, p.Y, p.Z);

    /// <summary>
    /// Mirrors every vertex and measures its distance to the original surface. Symmetric when the mean is under
    /// 0.25 voxel and the 99th percentile under 1 voxel. (Not the max: small deliberate asymmetries such as eye
    /// highlights shouldn't veto an otherwise mirrored model.)
    /// </summary>
    public static (bool ok, float mean, float p99, float max) DetectMesh(MeshData m, Bvh bvh, float cx, float voxel)
    {
        var d = new float[m.VertexCount];
        float cap = 20 * voxel;
        Parallel.For(0, m.VertexCount, v =>
            d[v] = bvh.Closest(Mirror(m.Pos[v], cx), cap, null, out var hit) ? hit.Dist : cap);
        double sum = 0;
        foreach (var x in d) sum += x;
        float mean = (float)(sum / Math.Max(1, d.Length));
        Array.Sort(d);
        float p99 = d.Length > 0 ? d[Math.Min(d.Length - 1, (int)(0.99 * d.Length))] : 0;
        float max = d.Length > 0 ? d[^1] : 0;
        return (mean < 0.25f * voxel && p99 < 1f * voxel, mean, p99, max);
    }

    /// <summary>Index of each bone's mirror partner (Left*↔Right*), or the bone itself for centre bones.</summary>
    public static int[] BoneMirror(RigSkeleton r)
    {
        var map = new int[r.Bones.Count];
        for (int i = 0; i < map.Length; i++)
        {
            var other = RigEdit.MirrorName(r.Bones[i].Name);
            int j = other != null ? r.Find(other) : -1;
            map[i] = j >= 0 ? j : i;
        }
        return map;
    }

    /// <summary>Rig is symmetric when every Left*/Right* pair mirrors within tol and centre bones lie on the plane.</summary>
    public static bool RigSymmetric(RigSkeleton r, float cx, float tol, out string why)
    {
        var map = BoneMirror(r);
        for (int i = 0; i < map.Length; i++)
        {
            var b = r.Bones[i];
            bool sided = b.Name.StartsWith("Left") || b.Name.StartsWith("Right");
            if (sided && map[i] == i) { why = $"{b.Name} has no mirror partner"; return false; }
            int j = map[i];
            if ((Mirror(b.Head, cx) - r.Bones[j].Head).Length() > tol || (Mirror(r.Tail(i), cx) - r.Tail(j)).Length() > tol)
            {
                why = j == i ? $"{b.Name} is off the centre plane" : $"{b.Name} doesn't mirror {r.Bones[j].Name}";
                return false;
            }
        }
        why = null;
        return true;
    }

    /// <summary>
    /// Averages dense [vertex × bone] weights with their mirror image: weights sampled at each vertex's mirrored
    /// position (closest point on the surface, barycentric), with Left/Right bones swapped.
    /// </summary>
    public static float[] SymmetrizeDense(MeshData m, Bvh bvh, float[] dense, int nb, int[] boneMirror, float cx)
    {
        var o = new float[dense.Length];
        Parallel.For(0, m.VertexCount, v =>
        {
            int vb = v * nb;
            if (!bvh.Closest(Mirror(m.Pos[v], cx), float.MaxValue, null, out var hit))
            {
                for (int b = 0; b < nb; b++) o[vb + b] = dense[vb + b];
                return;
            }
            int a = m.Tris[3 * hit.Tri] * nb, bb = m.Tris[3 * hit.Tri + 1] * nb, c = m.Tris[3 * hit.Tri + 2] * nb;
            for (int b = 0; b < nb; b++)
            {
                float mirrored = dense[a + b] * hit.U + dense[bb + b] * hit.V + dense[c + b] * hit.W;
                o[vb + boneMirror[b]] += 0.5f * mirrored;
                o[vb + b] += 0.5f * dense[vb + b];
            }
        });
        return o;
    }

    /// <summary>
    /// Transfers top-4 weights from a surface to the vertices of another mesh lying on it: closest point,
    /// barycentric blend of the three corners' influences, then top-4 and normalize.
    /// </summary>
    public static (int[] bones, float[] weights) TransferWeights(MeshData from, Bvh fromBvh, int[] fb, float[] fw, MeshData to)
    {
        int n = to.VertexCount;
        var bones = new int[n * 4];
        var weights = new float[n * 4];
        Parallel.For(0, n, v =>
        {
            var acc = new Dictionary<int, float>(12);
            if (fromBvh.Closest(to.Pos[v], float.MaxValue, null, out var hit))
            {
                Span<float> bw = stackalloc float[3] { hit.U, hit.V, hit.W };
                for (int c = 0; c < 3; c++)
                {
                    int s = from.Tris[3 * hit.Tri + c];
                    for (int k = 0; k < 4; k++)
                    {
                        float w = fw[4 * s + k] * bw[c];
                        if (w <= 0) continue;
                        acc[fb[4 * s + k]] = acc.TryGetValue(fb[4 * s + k], out var prev) ? prev + w : w;
                    }
                }
            }
            int slot = 0; float sum = 0;
            foreach (var kv in TopK(acc, 4)) { bones[4 * v + slot] = kv.Key; weights[4 * v + slot] = kv.Value; sum += kv.Value; slot++; }
            if (sum <= 0) { bones[4 * v] = 0; weights[4 * v] = 1; return; }
            for (int k = 0; k < 4; k++) weights[4 * v + k] /= sum;
        });
        return (bones, weights);
    }

    static List<KeyValuePair<int, float>> TopK(Dictionary<int, float> d, int k)
    {
        var l = new List<KeyValuePair<int, float>>(d);
        l.Sort((a, b) => b.Value.CompareTo(a.Value));
        if (l.Count > k) l.RemoveRange(k, l.Count - k);
        return l;
    }
}
