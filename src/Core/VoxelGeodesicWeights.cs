using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Skin weights from geodesic distance through the mesh interior (voxel Dijkstra), so influence
/// cannot jump across empty space (e.g. arm to torso, leg to leg).
/// </summary>
public static class VoxelGeodesicWeights
{
    /// <returns>Dense weights [vertex * boneCount + bone].</returns>
    public static float[] Compute(MeshData mesh, RigSkeleton skel, VoxelGrid g, Bvh source, float falloff, Action<string> log)
    {
        int nb = skel.Bones.Count, nv = mesh.VertexCount;
        var w = new float[nv * nb];

        // per vertex: nearest inside lattice point + offset distance (no trilinear blending)
        var vIdx = new int[nv];
        var vOff = new float[nv];
        int missing = 0;
        Parallel.For(0, nv, v =>
        {
            vIdx[v] = NearestInside(g, mesh.Pos[v], 2, out vOff[v]);
            if (vIdx[v] < 0) vIdx[v] = NearestInside(g, mesh.Pos[v], 5, out vOff[v]);
        });
        foreach (var i in vIdx) if (i < 0) missing++;
        if (missing > 0) log?.Invoke($"Weights: {missing} vertices have no inside voxel nearby (assigned to nearest bone)");

        var deform = new List<int>();
        for (int b = 0; b < nb; b++) if (skel.Bones[b].Deform) deform.Add(b);
        var dists = new float[nb][];

        Parallel.ForEach(deform, b =>
        {
            var head = skel.Bones[b].Head;
            var tail = skel.Tail(b);
            dists[b] = Dijkstra(g, Seeds(g, head, tail, skel.Bones[b].Name, log));
        });

        // Envelope offset: subtract each bone's local thickness (distance from bone to the surface), so a
        // vertex is judged by how far it lies *beyond* the bone's own body part. Without this, wide parts
        // (torso) lose to nearby thin limbs whose bones sit close to the surface.
        var radius = new float[nb];
        foreach (var b in deform)
        {
            var head = skel.Bones[b].Head; var tail = skel.Tail(b);
            var rs = new List<float>();
            for (int k = 1; k <= 5; k++)
            {
                var pt = head + (tail - head) * (k / 6f);
                if (source.Closest(pt, float.MaxValue, null, out var hit)) rs.Add(hit.Dist);
            }
            rs.Sort();
            radius[b] = rs.Count > 0 ? 0.8f * rs[rs.Count / 2] : 0;
        }

        float eps = 0.5f * g.H;
        Parallel.For(0, nv, v =>
        {
            int gi = vIdx[v];
            float sum = 0;
            for (int k = 0; k < deform.Count; k++)
            {
                int b = deform[k];
                float d;
                if (gi >= 0 && !float.IsInfinity(dists[b][gi])) d = dists[b][gi] + vOff[v];
                else d = Geo.DistPointSegment(mesh.Pos[v], skel.Bones[b].Head, skel.Tail(b)) * 4f; // penalized fallback
                d -= radius[b];
                float wt = 1f / MathF.Pow(MathF.Max(d, eps), falloff);
                w[v * nb + b] = wt;
                sum += wt;
            }
            if (sum > 0) for (int b = 0; b < nb; b++) w[v * nb + b] /= sum;
        });
        return w;
    }

    static int NearestInside(VoxelGrid g, Vector3 p, int radius, out float dist)
    {
        var f = (p - g.Origin) / g.H;
        int cx = (int)MathF.Round(f.X), cy = (int)MathF.Round(f.Y), cz = (int)MathF.Round(f.Z);
        int best = -1;
        float bd = float.MaxValue;
        for (int z = cz - radius; z <= cz + radius; z++)
        for (int y = cy - radius; y <= cy + radius; y++)
        for (int x = cx - radius; x <= cx + radius; x++)
        {
            if (x < 0 || y < 0 || z < 0 || x >= g.Nx || y >= g.Ny || z >= g.Nz) continue;
            int i = g.Idx(x, y, z);
            if (!g.Inside[i]) continue;
            float d = (g.PointAt(x, y, z) - p).LengthSquared();
            if (d < bd) { bd = d; best = i; }
        }
        dist = best >= 0 ? MathF.Sqrt(bd) : 0;
        return best;
    }

    static List<(int idx, float d)> Seeds(VoxelGrid g, Vector3 a, Vector3 b, string name, Action<string> log)
    {
        var seeds = new Dictionary<int, float>();
        float len = (b - a).Length();
        int steps = Math.Max(2, (int)MathF.Ceiling(len / (g.H * 0.33f)));
        for (int s = 0; s <= steps; s++)
        {
            var p = a + (b - a) * (s / (float)steps);
            int i = g.NearestIndex(p);
            if (!g.Inside[i]) continue;
            g.Coords(i, out int x, out int y, out int z);
            float d = Geo.DistPointSegment(g.PointAt(x, y, z), a, b);
            if (!seeds.TryGetValue(i, out float old) || d < old) seeds[i] = d;
        }
        if (seeds.Count == 0)
        {
            // clamp: bone lies outside the mesh -> seed from nearest inside voxel to its midpoint
            var mid = (a + b) * 0.5f;
            int best = -1; float bd = float.MaxValue;
            for (int i = 0; i < g.Count; i++)
            {
                if (!g.Inside[i]) continue;
                g.Coords(i, out int x, out int y, out int z);
                float d = (g.PointAt(x, y, z) - mid).LengthSquared();
                if (d < bd) { bd = d; best = i; }
            }
            if (best >= 0) seeds[best] = MathF.Sqrt(bd);
            log?.Invoke($"Warning: bone {name} lies outside the mesh; seeded from nearest interior voxel ({MathF.Sqrt(bd):F3} away)");
        }
        var list = new List<(int, float)>();
        foreach (var kv in seeds) list.Add((kv.Key, kv.Value));
        return list;
    }

    static float[] Dijkstra(VoxelGrid g, List<(int idx, float d)> seeds)
    {
        var dist = new float[g.Count];
        Array.Fill(dist, float.PositiveInfinity);
        var pq = new PriorityQueue<int, float>();
        foreach (var (i, d) in seeds)
        {
            if (d < dist[i]) { dist[i] = d; pq.Enqueue(i, d); }
        }
        float h = g.H;
        Span<float> stepCost = stackalloc float[4] { 0, h, h * 1.41421356f, h * 1.7320508f };
        while (pq.TryDequeue(out int i, out float di))
        {
            if (di > dist[i]) continue;
            g.Coords(i, out int x, out int y, out int z);
            for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
                int k = Math.Abs(dx) + Math.Abs(dy) + Math.Abs(dz);
                if (k == 0) continue;
                int nx = x + dx, ny = y + dy, nz = z + dz;
                if (nx < 0 || ny < 0 || nz < 0 || nx >= g.Nx || ny >= g.Ny || nz >= g.Nz) continue;
                int j = g.Idx(nx, ny, nz);
                if (!g.Inside[j]) continue;
                float nd = di + stepCost[k];
                if (nd < dist[j]) { dist[j] = nd; pq.Enqueue(j, nd); }
            }
        }
        return dist;
    }
}

public static class WeightSmoother
{
    /// <summary>Laplacian smoothing on the welded mesh, then top-4 prune + normalize.</summary>
    public static (int[] bones, float[] weights) SmoothAndPrune(MeshData mesh, float[] dense, int nb, int passes)
    {
        int nv = mesh.VertexCount;
        var (start, nbr) = mesh.BuildAdjacency();
        var cur = dense;
        for (int p = 0; p < passes; p++)
        {
            var next = new float[cur.Length];
            Parallel.For(0, nv, v =>
            {
                int s = start[v], e = start[v + 1];
                for (int b = 0; b < nb; b++)
                {
                    float avg = 0;
                    for (int k = s; k < e; k++) avg += cur[nbr[k] * nb + b];
                    avg = e > s ? avg / (e - s) : cur[v * nb + b];
                    next[v * nb + b] = 0.5f * cur[v * nb + b] + 0.5f * avg;
                }
            });
            cur = next;
        }

        var bones = new int[nv * 4];
        var weights = new float[nv * 4];
        Parallel.For(0, nv, v =>
        {
            Span<int> bi = stackalloc int[4] { 0, 0, 0, 0 };
            Span<float> bw = stackalloc float[4] { 0, 0, 0, 0 };
            for (int b = 0; b < nb; b++)
            {
                float x = cur[v * nb + b];
                if (x <= bw[3]) continue;
                int k = 3;
                while (k > 0 && x > bw[k - 1]) { bw[k] = bw[k - 1]; bi[k] = bi[k - 1]; k--; }
                bw[k] = x; bi[k] = b;
            }
            float sum = bw[0] + bw[1] + bw[2] + bw[3];
            for (int k = 0; k < 4; k++)
            {
                bones[v * 4 + k] = bi[k];
                weights[v * 4 + k] = sum > 0 ? bw[k] / sum : (k == 0 ? 1 : 0);
            }
        });
        return (bones, weights);
    }
}
