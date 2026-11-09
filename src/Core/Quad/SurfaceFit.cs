using System;
using System.Threading.Tasks;
using Godot;

namespace ModelMate.Core.Quad;

/// <summary>
/// Moves the vertices of a coarse remesh along their normals so its faces fit the dense surface in a least-squares
/// sense, without changing topology.
///
/// A coarse mesh whose vertices all lie ON a curved surface has every face cutting inside it (on convex regions), a
/// systematic one-sided error. Placing vertices slightly off the surface balances that error, which is what quadric
/// decimation does implicitly. Each pass: every dense sample is matched to its closest remesh point; its signed
/// residual along the face normal is distributed to the face's corners by barycentric weight; each vertex moves by
/// the weighted mean residual (damped, capped at a fraction of its edge length, reverted if a face would flip).
/// </summary>
public static class SurfaceFit
{
    public struct Stats { public int Passes; public float MeanShift, MaxShift; public int Reverted; }

    public static Stats Fit(MeshData m, MeshData dense, int passes = 4, float damping = 0.8f, float maxShiftFrac = 0.3f)
    {
        var st = new Stats { Passes = passes };
        int nv = m.VertexCount;
        // per-vertex cap: a fraction of the mean incident edge length
        var cap = new float[nv];
        var cnt = new int[nv];
        for (int t = 0; t < m.TriCount; t++)
            for (int k = 0; k < 3; k++)
            {
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                float l = (m.Pos[a] - m.Pos[b]).Length();
                cap[a] += l; cnt[a]++; cap[b] += l; cnt[b]++;
            }
        for (int v = 0; v < nv; v++) cap[v] = cnt[v] > 0 ? maxShiftFrac * cap[v] / cnt[v] : 0;

        var origin = (Vector3[])m.Pos.Clone();
        int stride = Math.Max(1, dense.VertexCount / 60000);
        int ns = (dense.VertexCount + stride - 1) / stride;
        for (int pass = 0; pass < passes; pass++)
        {
            m.ComputeNormals();
            var bvh = new Bvh(m.Pos, m.Tris);
            var num = new double[nv];
            var den = new double[nv];
            var lk = new object[64];
            for (int i = 0; i < lk.Length; i++) lk[i] = new object();
            Parallel.For(0, ns, k =>
            {
                var p = dense.Pos[k * stride];
                if (!bvh.Closest(p, float.MaxValue, null, out var hit)) return;
                var nt = bvh.TriNormal[hit.Tri];
                float r = (p - hit.Point).Dot(nt);   // signed residual along the face normal (+ = surface is outside)
                Span<float> bw = stackalloc float[3] { hit.U, hit.V, hit.W };
                for (int c = 0; c < 3; c++)
                {
                    int v = m.Tris[3 * hit.Tri + c];
                    float w = bw[c] * MathF.Max(0f, nt.Dot(m.Nrm[v]));
                    if (w <= 0) continue;
                    lock (lk[v & 63]) { num[v] += w * r; den[v] += w; }
                }
            });
            var old = (Vector3[])m.Pos.Clone();
            for (int v = 0; v < nv; v++)
            {
                if (den[v] <= 0) continue;
                float d = damping * (float)(num[v] / den[v]);
                var np = m.Pos[v] + m.Nrm[v] * d;
                // cap the total displacement from the starting (on-surface) position
                var off = np - origin[v];
                float ol = off.Length();
                if (ol > cap[v]) np = origin[v] + off * (cap[v] / ol);
                m.Pos[v] = np;
            }
            // undo moves that flip a face
            for (int t = 0; t < m.TriCount; t++)
            {
                int a = m.Tris[3 * t], b = m.Tris[3 * t + 1], c = m.Tris[3 * t + 2];
                var n0 = (old[b] - old[a]).Cross(old[c] - old[a]);
                var n1 = (m.Pos[b] - m.Pos[a]).Cross(m.Pos[c] - m.Pos[a]);
                if (n0.Dot(n1) <= 0)
                {
                    m.Pos[a] = old[a]; m.Pos[b] = old[b]; m.Pos[c] = old[c];
                    st.Reverted++;
                }
            }
        }
        double sum = 0;
        for (int v = 0; v < nv; v++)
        {
            float s = (m.Pos[v] - origin[v]).Length();
            sum += s; st.MaxShift = MathF.Max(st.MaxShift, s);
        }
        st.MeanShift = (float)(sum / Math.Max(1, nv));
        m.ComputeNormals();
        return st;
    }
}
