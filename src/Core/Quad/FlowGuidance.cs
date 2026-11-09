using System;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>
/// Skeleton guidance for the orientation field: each base vertex is pulled toward the axis of the bone that owns
/// it, so edges run along and around limbs.
///   weight = strength × ownership × |tangential share of the bone axis|
/// It fades at limb tips (axis ≈ normal) and at junctions where ownership is split, and is doubled in each bending
/// joint's slab (within 2ρ of the joint along the bone, owned by the joint's two bones) to keep singularities out.
/// </summary>
public static class FlowGuidance
{
    public static (Vector3[] dir, float[] weight) Build(MeshData m, RigSkeleton rig, int[] bones4, float[] w4,
        float rho, float strength)
    {
        int n = m.VertexCount, nb = rig.Bones.Count;
        var axis = new Vector3[nb];
        for (int b = 0; b < nb; b++)
        {
            var d = rig.Tail(b) - rig.Bones[b].Head;
            axis[b] = d.LengthSquared() > 1e-12f ? d.Normalized() : Vector3.Up;
        }
        if (m.Nrm == null) m.ComputeNormals();
        var dir = new Vector3[n];
        var wt = new float[n];
        Parallel.For(0, n, v =>
        {
            int best = -1; float own = 0;
            for (int k = 0; k < 4; k++)
            {
                int b = bones4[4 * v + k];
                float w = w4[4 * v + k];
                if (w > own && rig.Bones[b].Deform) { own = w; best = b; }
            }
            if (best < 0) return;
            var nrm = m.Nrm[v];
            var a = axis[best];
            float tangential = (a - nrm * nrm.Dot(a)).Length();
            float w8 = strength * own * tangential;
            // joint slab boost
            for (int b = 0; b < nb && w8 > 0; b++)
            {
                int p = rig.Bones[b].Parent;
                if (p < 0 || !rig.Bones[b].Deform || !rig.Bones[p].Deform) continue;
                float pair = 0;
                for (int k = 0; k < 4; k++)
                {
                    int bb = bones4[4 * v + k];
                    if (bb == b || bb == p) pair += w4[4 * v + k];
                }
                if (pair > 0.5f && MathF.Abs((m.Pos[v] - rig.Bones[b].Head).Dot(axis[b])) < 2f * rho) { w8 *= 2f; break; }
            }
            dir[v] = a;
            wt[v] = MathF.Min(w8, 1f);
        });
        return (dir, wt);
    }
}
