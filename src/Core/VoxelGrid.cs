using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Lattice of sample points with narrow-band unsigned distance to the source surface and an
/// inside/outside classification (gap-closing flood fill, with ray-winding vote fallback).
/// </summary>
public sealed class VoxelGrid
{
    public int Nx, Ny, Nz;
    public Vector3 Origin;
    public float H;
    public float Band;
    public float[] Dist;   // unsigned, clamped to Band
    public bool[] Inside;
    public string Method = "";

    public int Count => Nx * Ny * Nz;
    public int Idx(int x, int y, int z) => x + Nx * (y + Ny * z);
    public Vector3 PointAt(int x, int y, int z) => Origin + new Vector3(x, y, z) * H;

    public void Coords(int i, out int x, out int y, out int z)
    {
        x = i % Nx;
        int r = i / Nx;
        y = r % Ny;
        z = r / Ny;
    }

    public static VoxelGrid Build(Vector3[] pos, int[] tris, Bvh bvh, int res, int close, Action<string> log)
    {
        var g = new VoxelGrid();
        Vector3 mn = pos[0], mx = pos[0];
        foreach (var p in pos) { mn = mn.Min(p); mx = mx.Max(p); }
        var size = mx - mn;
        float ext = MathF.Max(size.X, MathF.Max(size.Y, size.Z));
        g.H = ext / res;
        int pad = 3 + close;
        g.Origin = mn - Vector3.One * (pad * g.H);
        g.Nx = (int)MathF.Ceiling(size.X / g.H) + 1 + 2 * pad;
        g.Ny = (int)MathF.Ceiling(size.Y / g.H) + 1 + 2 * pad;
        g.Nz = (int)MathF.Ceiling(size.Z / g.H) + 1 + 2 * pad;
        g.Band = MathF.Max(3, close + 2) * g.H;

        // --- narrow-band unsigned distance ---
        int n = g.Count;
        g.Dist = new float[n];
        Array.Fill(g.Dist, g.Band);
        int triCount = tris.Length / 3;
        // Mark band cells via triangle AABBs (cheap), then one BVH distance query per marked cell.
        // Cost scales with surface area / H², not triangle count, so dense sculpts stay fast.
        var band = new bool[n];
        Parallel.For(0, triCount, t =>
        {
            var a = pos[tris[3 * t]]; var b = pos[tris[3 * t + 1]]; var c = pos[tris[3 * t + 2]];
            var tmn = a.Min(b).Min(c) - Vector3.One * g.Band;
            var tmx = a.Max(b).Max(c) + Vector3.One * g.Band;
            int x0 = Math.Max(0, (int)MathF.Ceiling((tmn.X - g.Origin.X) / g.H));
            int y0 = Math.Max(0, (int)MathF.Ceiling((tmn.Y - g.Origin.Y) / g.H));
            int z0 = Math.Max(0, (int)MathF.Ceiling((tmn.Z - g.Origin.Z) / g.H));
            int x1 = Math.Min(g.Nx - 1, (int)MathF.Floor((tmx.X - g.Origin.X) / g.H));
            int y1 = Math.Min(g.Ny - 1, (int)MathF.Floor((tmx.Y - g.Origin.Y) / g.H));
            int z1 = Math.Min(g.Nz - 1, (int)MathF.Floor((tmx.Z - g.Origin.Z) / g.H));
            for (int z = z0; z <= z1; z++)
            for (int y = y0; y <= y1; y++)
            {
                int row = g.Idx(0, y, z);
                for (int x = x0; x <= x1; x++) band[row + x] = true;
            }
        });
        Parallel.For(0, g.Nz, z =>
        {
            for (int y = 0; y < g.Ny; y++)
            for (int x = 0; x < g.Nx; x++)
            {
                int i = g.Idx(x, y, z);
                if (!band[i]) continue;
                if (bvh.Closest(g.PointAt(x, y, z), g.Band, null, out var hit)) g.Dist[i] = hit.Dist;
            }
        });

        // --- wall + gap closing flood fill ---
        // Any surface crossing of a lattice edge lies within H/2 of one endpoint, so this wall is
        // watertight against 6-connected flood fill (except across true holes in the mesh).
        float wallD = 0.5f * g.H * 1.001f;
        var blocked = new bool[n];
        int wallCount = 0;
        for (int i = 0; i < n; i++)
            if (g.Dist[i] <= wallD) { blocked[i] = true; wallCount++; }
        for (int k = 0; k < close; k++) blocked = Dilate6(g, blocked);

        var outside = new bool[n];
        var queue = new Queue<int>();
        for (int z = 0; z < g.Nz; z++)
        for (int y = 0; y < g.Ny; y++)
        for (int x = 0; x < g.Nx; x++)
        {
            if (x != 0 && y != 0 && z != 0 && x != g.Nx - 1 && y != g.Ny - 1 && z != g.Nz - 1) continue;
            int i = g.Idx(x, y, z);
            if (!blocked[i] && !outside[i]) { outside[i] = true; queue.Enqueue(i); }
        }
        Flood(g, queue, outside, blocked);
        // erode the closed solid back by `close` (grow outside into the dilated shell)
        for (int k = 0; k < close; k++) outside = Dilate6(g, outside);

        // Union of two inside tests:
        //  - flood fill: robust to inverted/inconsistent normals, but a hole floods the cavity it opens into;
        //  - ray-winding vote: robust to holes and overlapping parts, needs mostly outward normals.
        var vote = WindingVote(g, bvh);
        g.Inside = new bool[n];
        int floodOnly = 0, voteOnly = 0;
        for (int i = 0; i < n; i++)
        {
            bool f = !outside[i];
            g.Inside[i] = f || vote[i];
            if (f && !vote[i]) floodOnly++;
            if (!f && vote[i]) voteOnly++;
        }
        g.Method = $"flood(close={close}) | winding-vote";
        if (voteOnly > 0.01f * wallCount)
            log?.Invoke($"Inside test: flood fill leaked somewhere; winding vote recovered {voteOnly} interior points");
        return g;
    }

    static bool[] Dilate6(VoxelGrid g, bool[] src)
    {
        var dst = (bool[])src.Clone();
        Parallel.For(0, g.Nz, z =>
        {
            for (int y = 0; y < g.Ny; y++)
            for (int x = 0; x < g.Nx; x++)
            {
                int i = g.Idx(x, y, z);
                if (src[i]) continue;
                if ((x > 0 && src[i - 1]) || (x < g.Nx - 1 && src[i + 1]) ||
                    (y > 0 && src[i - g.Nx]) || (y < g.Ny - 1 && src[i + g.Nx]) ||
                    (z > 0 && src[i - g.Nx * g.Ny]) || (z < g.Nz - 1 && src[i + g.Nx * g.Ny]))
                    dst[i] = true;
            }
        });
        return dst;
    }

    static void Flood(VoxelGrid g, Queue<int> queue, bool[] mark, bool[] blocked)
    {
        int sxy = g.Nx * g.Ny;
        while (queue.Count > 0)
        {
            int i = queue.Dequeue();
            g.Coords(i, out int x, out int y, out int z);
            void Try(int j)
            {
                if (!mark[j] && !blocked[j]) { mark[j] = true; queue.Enqueue(j); }
            }
            if (x > 0) Try(i - 1);
            if (x < g.Nx - 1) Try(i + 1);
            if (y > 0) Try(i - g.Nx);
            if (y < g.Ny - 1) Try(i + g.Nx);
            if (z > 0) Try(i - sxy);
            if (z < g.Nz - 1) Try(i + sxy);
        }
    }

    /// <summary>
    /// Inside test by signed ray crossings (non-zero winding) along each axis, majority vote of 3.
    /// Tolerates overlapping/self-intersecting parts; a hole only corrupts the columns through it.
    /// </summary>
    static bool[] WindingVote(VoxelGrid g, Bvh bvh)
    {
        int n = g.Count;
        var votes = new byte[n];
        int[] dims = { g.Nx, g.Ny, g.Nz };
        for (int axis = 0; axis < 3; axis++)
        {
            int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
            var dir = Vector3.Zero; dir[axis] = 1;
            float len = (dims[axis] + 2) * g.H;
            Parallel.For(0, dims[a2], j2 =>
            {
                var hits = new List<(float t, int sign)>();
                var c = new int[3];
                for (int j1 = 0; j1 < dims[a1]; j1++)
                {
                    var o = g.Origin;
                    o[axis] -= g.H;
                    o[a1] += j1 * g.H + 1.37e-4f * g.H;  // jitter off exact vertices/edges
                    o[a2] += j2 * g.H + 2.91e-4f * g.H;
                    bvh.RaycastAll(o, dir, len, hits);
                    int h = 0, wind = 0;
                    c[a1] = j1; c[a2] = j2;
                    for (int k = 0; k < dims[axis]; k++)
                    {
                        float tk = (k + 1) * g.H;
                        while (h < hits.Count && hits[h].t < tk) wind += hits[h++].sign;
                        if (wind > 0)
                        {
                            c[axis] = k;
                            votes[g.Idx(c[0], c[1], c[2])]++;
                        }
                    }
                }
            });
        }
        var inside = new bool[n];
        for (int i = 0; i < n; i++) inside[i] = votes[i] >= 2;
        return inside;
    }

    /// <summary>Index of nearest lattice point to p (clamped).</summary>
    public int NearestIndex(Vector3 p)
    {
        var f = (p - Origin) / H;
        int x = Math.Clamp((int)MathF.Round(f.X), 0, Nx - 1);
        int y = Math.Clamp((int)MathF.Round(f.Y), 0, Ny - 1);
        int z = Math.Clamp((int)MathF.Round(f.Z), 0, Nz - 1);
        return Idx(x, y, z);
    }
}
