using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core.Uv;

/// <summary>
/// Packs flattened charts into a square texture at one uniform texel density.
///
/// Each chart is rasterized conservatively onto a coarse grid (cells of texSize/512 texels), dilated by half the
/// padding, and reduced to a bottom and top profile per column. Charts are dropped, largest first, onto a skyline
/// (Tetris-style: the chart rests where its bottom profile first touches the skyline), trying 16 turns starting
/// from its minimum-area orientation and every column, and keeping the placement that leaves the lowest peak and
/// the least hidden gap below it. Irregular charts interlock instead of each claiming its bounding rectangle. The
/// density is the largest one at which everything fits.
/// </summary>
public static class AtlasPack
{
    public struct Placement { public float Angle; public Vector2 Offset; }   // texel = R(Angle)·uv·s + Offset

    sealed class Shape
    {
        public int W, H;
        public int[] Bottom, Top;   // per column; Bottom = int.MaxValue when the column is empty
        public int TopMax;
        public Vector2 Offset;      // texel offset of the rotated, scaled chart within the shape's cell frame
        public float Angle;
    }

    public static (Placement[] place, float s) Pack(IReadOnlyList<FlatChart> charts, int texSize, int padTexels)
    {
        int n = charts.Count;
        var baseAngle = new float[n];
        double area = 0;
        var order = new int[n];
        var chartArea = new double[n];
        for (int c = 0; c < n; c++)
        {
            baseAngle[c] = ChartUnwrap.MinAreaAngle(new List<Vector2>(charts[c].Uv));
            var ch = charts[c];
            for (int i = 0; i < ch.Tris.Count; i++)
            {
                var a = ch.Uv[ch.TriLocal[3 * i]]; var b = ch.Uv[ch.TriLocal[3 * i + 1]]; var d = ch.Uv[ch.TriLocal[3 * i + 2]];
                chartArea[c] += MathF.Abs(ChartFlatten.Cross(b - a, d - a)) * 0.5;
            }
            area += chartArea[c];
            order[c] = c;
        }
        Array.Sort(order, (a, b) => chartArea[b].CompareTo(chartArea[a]));

        float s = (float)Math.Sqrt(0.7 * texSize * (double)texSize / Math.Max(area, 1e-30));
        Placement[] place = null;
        int attempt = 0;
        for (; attempt < 200 && (place = TryPack(charts, order, baseAngle, s, texSize, padTexels)) == null; attempt++) s *= 0.93f;
        if (place == null) throw new Exception($"AtlasPack: could not pack {n} charts into {texSize}²");
        // grow back toward the largest density that still fits
        float lo = s, hi = attempt == 0 ? s * 1.3f : s / 0.93f;
        for (int it = 0; it < 7; it++)
        {
            float mid = 0.5f * (lo + hi);
            var pm = TryPack(charts, order, baseAngle, mid, texSize, padTexels);
            if (pm != null) { lo = mid; place = pm; } else hi = mid;
        }
        return (place, lo);
    }

    const int Orientations = 16;   // turns tried per chart (any rotation keeps a layout valid; mirroring would not)
    static Placement[] TryPack(IReadOnlyList<FlatChart> charts, int[] order, float[] baseAngle, float s, int T, int pad)
    {
        int cell = Math.Max(1, T / 512);
        int R = T / cell;
        int dil = Math.Max(1, (int)MathF.Ceiling(pad / (2f * cell)));
        var sky = new int[R];
        var res = new Placement[charts.Count];
        foreach (int c in order)
        {
            int bestX = -1, bestY = 0; long bestScore = long.MaxValue; Shape best = null;
            for (int rot = 0; rot < Orientations; rot++)
            {
                var sh = Rasterize(charts[c], baseAngle[c] + rot * 2 * MathF.PI / Orientations, s / cell, dil);
                if (sh.W > R || sh.TopMax > R) continue;
                for (int x = 0; x + sh.W <= R; x++)
                {
                    int y = 0;
                    for (int i = 0; i < sh.W; i++)
                        if (sh.Bottom[i] != int.MaxValue) y = Math.Max(y, sky[x + i] - sh.Bottom[i]);
                    int peak = y + sh.TopMax;
                    if (peak > R) continue;
                    long waste = 0;
                    for (int i = 0; i < sh.W; i++)
                        if (sh.Bottom[i] != int.MaxValue) waste += y + sh.Bottom[i] - sky[x + i];
                    long score = (long)peak * R * 4 + waste;
                    if (score < bestScore) { bestScore = score; bestX = x; bestY = y; best = sh; }
                }
            }
            if (best == null) return null;
            for (int i = 0; i < best.W; i++)
                if (best.Bottom[i] != int.MaxValue) sky[bestX + i] = Math.Max(sky[bestX + i], bestY + best.Top[i]);
            res[c] = new Placement { Angle = best.Angle, Offset = (new Vector2(bestX, bestY) * cell) + best.Offset * cell };
        }
        return res;
    }

    /// <summary>Conservative raster of the chart rotated by angle and scaled to cells, dilated by dil cells.</summary>
    static Shape Rasterize(FlatChart ch, float angle, float sc, int dil)
    {
        float ca = MathF.Cos(angle), sa = MathF.Sin(angle);
        var q = new Vector2[ch.Uv.Length];
        var mn = new Vector2(float.MaxValue, float.MaxValue);
        var mx = new Vector2(float.MinValue, float.MinValue);
        for (int i = 0; i < q.Length; i++)
        {
            var p = ch.Uv[i];
            q[i] = new Vector2(ca * p.X - sa * p.Y, sa * p.X + ca * p.Y) * sc;
            mn = mn.Min(q[i]); mx = mx.Max(q[i]);
        }
        var off = new Vector2(dil, dil) - mn;
        for (int i = 0; i < q.Length; i++) q[i] += off;
        int w = (int)MathF.Floor(mx.X - mn.X) + 1 + 2 * dil, h = (int)MathF.Floor(mx.Y - mn.Y) + 1 + 2 * dil;
        // per-column extent of the cells the chart touches, then the box dilation applied to the profiles directly
        // (a column of the dilated mask spans the neighbouring columns' extents, widened by dil)
        // the lowest and highest point of the chart within a column always lie on its boundary, so the boundary
        // edges alone give the column extents
        var lo = new int[w]; var hi = new int[w];
        Array.Fill(lo, int.MaxValue); Array.Fill(hi, int.MinValue);
        foreach (int c in ch.Bnd)
            Segment(q[ch.TriLocal[c]], q[ch.TriLocal[c - c % 3 + (c % 3 + 1) % 3]], w, h, lo, hi);
        var sh = new Shape { W = w, H = h, Bottom = new int[w], Top = new int[w], Angle = angle, Offset = off };
        for (int x = 0; x < w; x++)
        {
            int b = int.MaxValue, top = int.MinValue;
            for (int dx = Math.Max(0, x - dil); dx <= Math.Min(w - 1, x + dil); dx++)
            {
                if (lo[dx] < b) b = lo[dx];
                if (hi[dx] > top) top = hi[dx];
            }
            if (b == int.MaxValue) { sh.Bottom[x] = int.MaxValue; sh.Top[x] = 0; continue; }
            sh.Bottom[x] = Math.Max(0, b - dil);
            sh.Top[x] = Math.Min(h, top + 1 + dil);
            sh.TopMax = Math.Max(sh.TopMax, sh.Top[x]);
        }
        return sh;
    }

    /// <summary>Widens each column's [lo, hi] to the cells the segment passes through.</summary>
    static void Segment(Vector2 a, Vector2 b, int w, int h, int[] lo, int[] hi)
    {
        if (a.X > b.X) (a, b) = (b, a);
        int x0 = Math.Clamp((int)MathF.Floor(a.X), 0, w - 1), x1 = Math.Clamp((int)MathF.Floor(b.X), 0, w - 1);
        float dx = b.X - a.X;
        for (int x = x0; x <= x1; x++)
        {
            // the segment's part within the column strip [x, x + 1]
            float ya = a.Y, yb = b.Y;
            if (dx > 1e-12f)
            {
                float t0 = Math.Clamp((x - a.X) / dx, 0, 1), t1 = Math.Clamp((x + 1 - a.X) / dx, 0, 1);
                ya = a.Y + (b.Y - a.Y) * t0; yb = a.Y + (b.Y - a.Y) * t1;
            }
            int c0 = Math.Clamp((int)MathF.Floor(MathF.Min(ya, yb)), 0, h - 1), c1 = Math.Clamp((int)MathF.Floor(MathF.Max(ya, yb)), 0, h - 1);
            if (c0 < lo[x]) lo[x] = c0;
            if (c1 > hi[x]) hi[x] = c1;
        }
    }
}
