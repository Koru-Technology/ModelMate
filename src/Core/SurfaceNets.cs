using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>Naive surface nets over the voxel grid's inside/outside classification.</summary>
public static class SurfaceNets
{
    public static MeshData Extract(VoxelGrid g)
    {
        int cx = g.Nx - 1, cy = g.Ny - 1, cz = g.Nz - 1;
        var cellVert = new int[cx * cy * cz];
        Array.Fill(cellVert, -1);
        var pos = new List<Vector3>();
        int CellIdx(int x, int y, int z) => x + cx * (y + cy * z);

        // 12 cube edges as corner-offset pairs
        int[,] corner = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 0, 0, 1 }, { 1, 0, 1 }, { 0, 1, 1 }, { 1, 1, 1 } };
        int[,] edges = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } };
        Span<int> ci = stackalloc int[8];

        for (int z = 0; z < cz; z++)
        for (int y = 0; y < cy; y++)
        for (int x = 0; x < cx; x++)
        {
            int mask = 0;
            for (int k = 0; k < 8; k++)
            {
                ci[k] = g.Idx(x + corner[k, 0], y + corner[k, 1], z + corner[k, 2]);
                if (g.Inside[ci[k]]) mask |= 1 << k;
            }
            if (mask == 0 || mask == 255) continue;
            Vector3 sum = Vector3.Zero;
            int cnt = 0;
            for (int e = 0; e < 12; e++)
            {
                int a = edges[e, 0], b = edges[e, 1];
                bool ia = (mask & (1 << a)) != 0, ib = (mask & (1 << b)) != 0;
                if (ia == ib) continue;
                float da = g.Dist[ci[a]], db = g.Dist[ci[b]];
                float t = da + db > 1e-12f ? da / (da + db) : 0.5f;
                var pa = new Vector3(corner[a, 0], corner[a, 1], corner[a, 2]);
                var pb = new Vector3(corner[b, 0], corner[b, 1], corner[b, 2]);
                sum += pa + (pb - pa) * t;
                cnt++;
            }
            var local = sum / cnt;
            cellVert[CellIdx(x, y, z)] = pos.Count;
            pos.Add(g.Origin + (new Vector3(x, y, z) + local) * g.H);
        }

        var tris = new List<int>();
        void Quad(int c0, int c1, int c2, int c3, Vector3 outward)
        {
            int q0 = cellVert[c0], q1 = cellVert[c1], q2 = cellVert[c2], q3 = cellVert[c3];
            if (q0 < 0 || q1 < 0 || q2 < 0 || q3 < 0) return;
            Vector3 p0 = pos[q0], p1 = pos[q1], p2 = pos[q2], p3 = pos[q3];
            var nrm = (p2 - p0).Cross(p3 - p1);
            if (nrm.Dot(outward) < 0)
            {
                (q1, q3) = (q3, q1);
                (p1, p3) = (p3, p1);
            }
            if ((p0 - p2).LengthSquared() <= (p1 - p3).LengthSquared())
            {
                tris.Add(q0); tris.Add(q1); tris.Add(q2);
                tris.Add(q0); tris.Add(q2); tris.Add(q3);
            }
            else
            {
                tris.Add(q0); tris.Add(q1); tris.Add(q3);
                tris.Add(q1); tris.Add(q2); tris.Add(q3);
            }
        }

        for (int z = 1; z < g.Nz - 1; z++)
        for (int y = 1; y < g.Ny - 1; y++)
        for (int x = 1; x < g.Nx - 1; x++)
        {
            bool i0 = g.Inside[g.Idx(x, y, z)];
            // x-edge
            if (x < g.Nx - 1 && i0 != g.Inside[g.Idx(x + 1, y, z)])
                Quad(CellIdx(x, y - 1, z - 1), CellIdx(x, y, z - 1), CellIdx(x, y, z), CellIdx(x, y - 1, z),
                     i0 ? Vector3.Right : Vector3.Left);
            if (y < g.Ny - 1 && i0 != g.Inside[g.Idx(x, y + 1, z)])
                Quad(CellIdx(x - 1, y, z - 1), CellIdx(x, y, z - 1), CellIdx(x, y, z), CellIdx(x - 1, y, z),
                     i0 ? Vector3.Up : Vector3.Down);
            if (z < g.Nz - 1 && i0 != g.Inside[g.Idx(x, y, z + 1)])
                Quad(CellIdx(x - 1, y - 1, z), CellIdx(x, y - 1, z), CellIdx(x, y, z), CellIdx(x - 1, y, z),
                     i0 ? Vector3.Back : Vector3.Forward);
        }

        var m = new MeshData(pos.ToArray(), tris.ToArray());
        return m;
    }
}
