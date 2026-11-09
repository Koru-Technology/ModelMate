using System;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core;

/// <summary>Bakes source albedo onto the low-poly mesh's UV layout. Thread-safe (raw arrays only).</summary>
public static class TextureBaker
{
    public static byte[] BakeAlbedo(MeshData low, SourceMesh src, Bvh srcBvh, int size, float maxDist, int dilate = 8)
        => Bake(low, src, srcBvh, size, maxDist, null, null, dilate).albedo;

    /// <summary>
    /// Bakes albedo and (if low.Tan is set) a tangent-space normal map (OpenGL / glTF convention:
    /// B = cross(N, T) * w, encoded n*0.5+0.5).
    /// </summary>
    /// <param name="normalSurf">Surface whose smooth normals are baked into the normal map (the clean voxel base mesh:
    /// the outer surface of the source union, without internal shells or duplicate overlay layers). Null = source.</param>
    public static (byte[] albedo, byte[] normal) Bake(MeshData low, SourceMesh src, Bvh srcBvh, int size, float maxDist,
        MeshData normalSurf, Bvh normalBvh, int dilate = 8)
    {
        if (normalSurf == null) { normalSurf = src.Mesh; normalBvh = srcBvh; }
        bool doNormal = low.Tan != null && normalSurf.Nrm != null;
        var srcTriN = srcBvh.TriNormal;
        var triMat = src.TriMat;
        var mats = src.Materials;
        bool Opaque(int t) => !mats[triMat[t]].Transparent;
        var nmap = doNormal ? new float[size * size * 4] : null;
        var color = new float[size * size * 4];
        var covered = new bool[size * size];

        Parallel.For(0, low.TriCount, t =>
        {
            int ia = low.Tris[3 * t], ib = low.Tris[3 * t + 1], ic = low.Tris[3 * t + 2];
            Vector2 ua = low.Uv[ia] * size, ub = low.Uv[ib] * size, uc = low.Uv[ic] * size;
            float area = (ub - ua).Cross(uc - ua);
            if (MathF.Abs(area) < 1e-12f) return;
            int x0 = Math.Max(0, (int)MathF.Floor(MathF.Min(ua.X, MathF.Min(ub.X, uc.X))) - 1);
            int x1 = Math.Min(size - 1, (int)MathF.Ceiling(MathF.Max(ua.X, MathF.Max(ub.X, uc.X))) + 1);
            int y0 = Math.Max(0, (int)MathF.Floor(MathF.Min(ua.Y, MathF.Min(ub.Y, uc.Y))) - 1);
            int y1 = Math.Min(size - 1, (int)MathF.Ceiling(MathF.Max(ua.Y, MathF.Max(ub.Y, uc.Y))) + 1);
            float pad = 0.5f / MathF.Max(1e-6f, MathF.Sqrt(MathF.Abs(area))); // ~half-texel conservative margin
            for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
            {
                var p = new Vector2(x + 0.5f, y + 0.5f);
                float wa = (ub - p).Cross(uc - p) / area;
                float wb = (uc - p).Cross(ua - p) / area;
                float wc = 1 - wa - wb;
                if (wa < -pad || wb < -pad || wc < -pad) continue;
                var pos = low.Pos[ia] * wa + low.Pos[ib] * wb + low.Pos[ic] * wc;
                var nrm = (low.Nrm[ia] * wa + low.Nrm[ib] * wb + low.Nrm[ic] * wc).Normalized();

                BvhHit hit;
                bool ok = false;
                // Colour: ray along -normal from outside (projects like a cage) onto opaque, outward-facing source
                // triangles only — skips transparent overlay shells and the backs of overlapping internal shells.
                var o = pos + nrm * maxDist;
                var nr = nrm;
                if (srcBvh.Raycast(o, -nrm, 2 * maxDist, out hit, t => Opaque(t) && srcTriN[t].Dot(nr) > 0)) ok = true;
                else ok = srcBvh.Closest(pos, maxDist * 4, t => Opaque(t) && srcTriN[t].Dot(nr) > 0, out hit)
                       || srcBvh.Closest(pos, maxDist * 4, null, out hit);
                if (!ok) continue;
                var c = src.SampleColor(hit.Tri, hit.U, hit.V, hit.W);
                int i = y * size + x;
                if (doNormal && normalBvh.Closest(pos, maxDist * 4, null, out var nh))
                {
                    var sn = normalSurf.Nrm;
                    var nt = normalSurf.Tris;
                    int sa = nt[3 * nh.Tri], sb = nt[3 * nh.Tri + 1], sc = nt[3 * nh.Tri + 2];
                    var hn = (sn[sa] * nh.U + sn[sb] * nh.V + sn[sc] * nh.W).Normalized();
                    var ta = new Vector3(low.Tan[4 * ia], low.Tan[4 * ia + 1], low.Tan[4 * ia + 2]);
                    var tb = new Vector3(low.Tan[4 * ib], low.Tan[4 * ib + 1], low.Tan[4 * ib + 2]);
                    var tc = new Vector3(low.Tan[4 * ic], low.Tan[4 * ic + 1], low.Tan[4 * ic + 2]);
                    var tn = ta * wa + tb * wb + tc * wc;
                    tn = (tn - nrm * tn.Dot(nrm)).Normalized();
                    float sign = low.Tan[4 * ia + 3] < 0 ? -1 : 1;
                    var bn = nrm.Cross(tn) * sign;
                    var ts = new Vector3(hn.Dot(tn), hn.Dot(bn), hn.Dot(nrm));
                    if (ts.Z < 0.05f) ts = new Vector3(0, 0, 1); // degenerate: keep flat
                    ts = ts.Normalized();
                    nmap[4 * i] = ts.X * 0.5f + 0.5f; nmap[4 * i + 1] = ts.Y * 0.5f + 0.5f; nmap[4 * i + 2] = ts.Z * 0.5f + 0.5f; nmap[4 * i + 3] = 1;
                }
                color[4 * i] = c.R; color[4 * i + 1] = c.G; color[4 * i + 2] = c.B; color[4 * i + 3] = c.A;
                covered[i] = true;
            }
        });

        var coveredA = (bool[])covered.Clone();
        var albedo = Finish(color, coveredA, size, dilate, new[] { 0f, 0f, 0f, 1f });
        byte[] normal = doNormal ? Finish(nmap, covered, size, dilate, new[] { 0.5f, 0.5f, 1f, 1f }) : null;
        return (albedo, normal);
    }

    static byte[] Finish(float[] color, bool[] covered, int size, int dilate, float[] empty)
    {
        // dilate to hide seams
        for (int pass = 0; pass < dilate; pass++)
        {
            var add = new bool[size * size];
            var nc = (float[])color.Clone();
            var src = color;
            Parallel.For(0, size, y =>
            {
                for (int x = 0; x < size; x++)
                {
                    int i = y * size + x;
                    if (covered[i]) continue;
                    float r = 0, g = 0, b = 0, a = 0; int n = 0;
                    for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int xx = x + dx, yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= size || yy >= size) continue;
                        int j = yy * size + xx;
                        if (!covered[j]) continue;
                        r += src[4 * j]; g += src[4 * j + 1]; b += src[4 * j + 2]; a += src[4 * j + 3]; n++;
                    }
                    if (n == 0) continue;
                    nc[4 * i] = r / n; nc[4 * i + 1] = g / n; nc[4 * i + 2] = b / n; nc[4 * i + 3] = a / n;
                    add[i] = true;
                }
            });
            color = nc;
            for (int i = 0; i < covered.Length; i++) covered[i] |= add[i];
        }
        for (int i = 0; i < covered.Length; i++)
            if (!covered[i]) for (int k = 0; k < 4; k++) color[4 * i + k] = empty[k];
        var bytes = new byte[size * size * 4];
        for (int i = 0; i < bytes.Length; i++)
            bytes[i] = (byte)Math.Clamp((int)MathF.Round(color[i] * 255f), 0, 255);
        return bytes;
    }
}
