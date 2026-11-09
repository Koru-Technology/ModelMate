// Ported from Instant Meshes (src/field.cpp: optimize_orientations_impl, optimize_positions_impl,
// Optimizer::run). Copyright (c) 2015 Wenzel Jakob, Daniele Panozzo, Marco Tarini, and
// Olga Sorkine-Hornung. BSD 3-Clause License — see THIRD_PARTY.md.
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>Hierarchical Gauss–Seidel solves for the 4-RoSy orientation field and 4-PoSy position field.</summary>
public static class FieldSolver
{
    const int LevelIterations = 6;

    /// <summary>Coarse-to-fine orientation solve. Constraints (CQ/CQw) must already be propagated.</summary>
    public static void SolveOrientations(Hierarchy h)
    {
        for (int level = h.Levels.Count - 1; level >= 0; level--)
        {
            for (int it = 0; it < LevelIterations; it++) OrientationSweep(h.Levels[level]);
            if (level > 0)
            {
                // prolong: copy coarse solution to both children, projected into their tangent planes
                var c = h.Levels[level];
                var f = h.Levels[level - 1];
                Parallel.For(0, c.Count, j =>
                {
                    for (int k = 0; k < 2; k++)
                    {
                        int dest = k == 0 ? c.Up0[j] : c.Up1[j];
                        if (dest < 0) continue;
                        var q = c.Q[j]; var n = f.N[dest];
                        f.Q[dest] = q - n * n.Dot(q);
                    }
                });
            }
        }
    }

    /// <summary>Coarse-to-fine position solve (uses each level's Q).</summary>
    public static void SolvePositions(Hierarchy h)
    {
        for (int level = h.Levels.Count - 1; level >= 0; level--)
        {
            for (int it = 0; it < LevelIterations; it++) PositionSweep(h.Levels[level]);
            if (level > 0)
            {
                var c = h.Levels[level];
                var f = h.Levels[level - 1];
                Parallel.For(0, c.Count, j =>
                {
                    for (int k = 0; k < 2; k++)
                    {
                        int dest = k == 0 ? c.Up0[j] : c.Up1[j];
                        if (dest < 0) continue;
                        Vector3 o = c.O[j], n = f.N[dest], v = f.V[dest];
                        f.O[dest] = o - n * n.Dot(o - v);
                    }
                });
            }
        }
    }

    static void OrientationSweep(Level l)
    {
        foreach (var phase in l.Phases)
        {
            Parallel.For(0, phase.Length, pi =>
            {
                int i = phase[pi];
                var ni = l.N[i];
                float wsum = 0;
                var sum = l.Q[i];
                for (int k = l.AdjStart[i]; k < l.AdjStart[i + 1]; k++)
                {
                    float w = l.AdjW[k];
                    if (w == 0) continue;
                    int j = l.AdjId[k];
                    var (a, b) = FieldMath.CompatOrient4(sum, ni, l.Q[j], l.N[j]);
                    sum = a * wsum + b * w;
                    sum -= ni * ni.Dot(sum);
                    wsum += w;
                    float norm = sum.Length();
                    if (norm > 1e-30f) sum /= norm;
                }
                float cw = l.CQw[i];
                if (cw != 0)
                {
                    var (a, b) = FieldMath.CompatOrient4(sum, ni, l.CQ[i], ni);
                    sum = a * (1 - cw) + b * cw;
                    sum -= ni * ni.Dot(sum);
                    float norm = sum.Length();
                    if (norm > 1e-30f) sum /= norm;
                }
                if (wsum > 0) l.Q[i] = sum;
            });
        }
    }

    static void PositionSweep(Level l)
    {
        foreach (var phase in l.Phases)
        {
            Parallel.For(0, phase.Length, pi =>
            {
                int i = phase[pi];
                Vector3 ni = l.N[i], vi = l.V[i];
                var qi = l.Q[i].Normalized();
                var sum = l.O[i];
                float wsum = 0;
                for (int k = l.AdjStart[i]; k < l.AdjStart[i + 1]; k++)
                {
                    float w = l.AdjW[k];
                    if (w == 0) continue;
                    int j = l.AdjId[k];
                    var qj = l.Q[j].Normalized();
                    float sij = 0.5f * (l.S[i] + l.S[j]);
                    var (a, b) = FieldMath.CompatPosition4(vi, ni, qi, sum, l.V[j], l.N[j], qj, l.O[j], sij, 1f / sij);
                    sum = a * wsum + b * w;
                    wsum += w;
                    if (wsum > 1e-30f) sum /= wsum;
                    sum -= ni * ni.Dot(sum - vi);
                }
                // point constraint (port of the CO/COw term): pull the lattice origin onto the constraint point
                if (l.COw != null && l.COw[i] > 0)
                {
                    sum += l.COw[i] * (l.CO[i] - sum);
                    sum -= ni * ni.Dot(sum - vi);
                }
                if (wsum > 0) l.O[i] = FieldMath.PositionRound4(sum, qi, ni, vi, l.S[i], 1f / l.S[i]);
            });
        }
    }
}
