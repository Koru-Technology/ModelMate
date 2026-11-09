using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>CPU linear-blend skinning on plain arrays (shared by the export checks and the deformation report).</summary>
public static class Skinning
{
    /// <summary>Global transforms from parent indices (parents before children) and per-bone local transforms.</summary>
    public static Transform3D[] Globals(int n, Func<int, int> parent, Func<int, Transform3D> local)
    {
        var g = new Transform3D[n];
        for (int i = 0; i < n; i++)
        {
            int p = parent(i);
            g[i] = (p >= 0 ? g[p] : Transform3D.Identity) * local(i);
        }
        return g;
    }

    /// <summary>Skins vertices. bindXf[k] is the full transform for influence index k (global pose × bind / inverse rest).</summary>
    public static Vector3[] Skin(Vector3[] verts, int[] bones, float[] weights, int perVertex, Transform3D[] bindXf)
    {
        var o = new Vector3[verts.Length];
        System.Threading.Tasks.Parallel.For(0, verts.Length, v =>
        {
            Vector3 acc = Vector3.Zero;
            for (int k = 0; k < perVertex; k++)
            {
                float w = weights[v * perVertex + k];
                if (w == 0) continue;
                acc += (bindXf[bones[v * perVertex + k]] * verts[v]) * w;
            }
            o[v] = acc;
        });
        return o;
    }

    /// <summary>Largest gap between vertices that share a rest position (UV seams) after posing.</summary>
    public static (int pairs, float maxGap) SeamGap(Vector3[] rest, Vector3[] posed)
    {
        var byPos = new Dictionary<Vector3, int>();
        int pairs = 0; float max = 0;
        for (int v = 0; v < rest.Length; v++)
        {
            if (byPos.TryGetValue(rest[v], out int f)) { pairs++; max = MathF.Max(max, (posed[v] - posed[f]).Length()); }
            else byPos[rest[v]] = v;
        }
        return (pairs, max);
    }
}
