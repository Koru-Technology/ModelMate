using System;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core.Uv;

/// <summary>What the unwrapper may use, beyond the mesh, to decide where seams show least. All optional.</summary>
public sealed class SeamHints
{
    public RigSkeleton Rig;
    public int[] Bones;      // 4 per welded vertex
    public float[] Weights;  // 4 per welded vertex
}

/// <summary>
/// How visible a seam along each mesh edge would be. Each vertex is "looked at" from 26 directions around the model
/// (a ray toward each direction must escape the mesh), weighted toward the front (+Z, the convention of the rig
/// templates), less toward the sides and top, least toward the back and underside. Occluded spots such as armpits,
/// inner legs and creases come out cheap. Vertices skinned to the head are weighted up further. The weights are
/// symmetric in X, so a mirrored model gets mirrored costs.
/// </summary>
public static class SeamCost
{
    /// <summary>Per-vertex visibility in [0, ~2].</summary>
    public static float[] Visibility(MeshData m, SeamHints hints)
    {
        var nrm = m.Nrm;
        if (nrm == null) { m.ComputeNormals(); nrm = m.Nrm; }
        var bvh = new Bvh(m.Pos, m.Tris);
        var box = m.Bounds();
        float diag = box.Size.Length();
        float eps = 1e-3f * diag;

        var dirs = new Vector3[26];
        var w = new float[26];
        int k = 0;
        for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
                for (int z = -1; z <= 1; z++)
                {
                    if (x == 0 && y == 0 && z == 0) continue;
                    var d = new Vector3(x, y, z).Normalized();
                    dirs[k] = d;
                    w[k] = 0.15f + 1.0f * MathF.Max(0, d.Z) + 0.25f * MathF.Max(0, d.Y) - 0.1f * MathF.Max(0, -d.Y);
                    k++;
                }
        // normalizer: an unoccluded vertex facing straight forward scores 1
        float norm = 0;
        for (int i = 0; i < 26; i++) norm += w[i] * MathF.Max(0, dirs[i].Z);

        float[] head = HeadWeight(m.VertexCount, hints);
        var vis = new float[m.VertexCount];
        Parallel.For(0, m.VertexCount, v =>
        {
            var n = nrm[v];
            var o = m.Pos[v] + n * eps;
            float s = 0;
            for (int i = 0; i < 26; i++)
            {
                float c = n.Dot(dirs[i]);
                if (c <= 0.05f) continue;
                if (bvh.Raycast(o, dirs[i], 2 * diag, out _)) continue;
                s += w[i] * c;
            }
            s /= norm;
            if (head != null) s *= 1 + 1.5f * head[v];
            vis[v] = s;
        });
        return vis;
    }

    /// <summary>Per half-edge (3 per triangle: corner k → k+1) cost of cutting there: length · (floor + visibility)².</summary>
    public static float[] EdgeCost(MeshData m, float[] vis)
    {
        int nt = m.TriCount;
        var c = new float[3 * nt];
        for (int t = 0; t < nt; t++)
            for (int k = 0; k < 3; k++)
            {
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                float len = (m.Pos[a] - m.Pos[b]).Length();
                float s = 0.05f + 0.5f * (vis[a] + vis[b]);
                c[3 * t + k] = len * s * s;
                // a hard edge already splits the normals: seams belong there (painters and bakers want them there)
                if (m.Hard != null && m.Hard[3 * t + k]) c[3 * t + k] *= 0.1f;
            }
        return c;
    }

    /// <summary>Skin weight on the head bone and its descendants (null without a rig that has a Head bone).</summary>
    static float[] HeadWeight(int nv, SeamHints h)
    {
        if (h?.Rig == null || h.Bones == null || h.Weights == null || h.Bones.Length < 4 * nv) return null;
        int head = h.Rig.Find("Head");
        if (head < 0) return null;
        var inHead = new bool[h.Rig.Bones.Count];
        for (int b = 0; b < inHead.Length; b++) inHead[b] = RigEdit.IsDescendant(h.Rig, b, head);
        var r = new float[nv];
        for (int v = 0; v < nv; v++)
            for (int k = 0; k < 4; k++)
            {
                int b = h.Bones[4 * v + k];
                if (b >= 0 && b < inHead.Length && inHead[b]) r[v] += h.Weights[4 * v + k];
            }
        return r;
    }
}
