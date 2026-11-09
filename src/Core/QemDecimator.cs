using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core;

/// <summary>Garland–Heckbert quadric edge collapse with link-condition and normal-flip guards.</summary>
public static class QemDecimator
{
    struct Quadric
    {
        // symmetric 4x4: a2 ab ac ad b2 bc bd c2 cd d2
        public double A2, AB, AC, AD, B2, BC, BD, C2, CD, D2;

        public static Quadric FromPlane(double a, double b, double c, double d, double w) => new()
        {
            A2 = w * a * a, AB = w * a * b, AC = w * a * c, AD = w * a * d,
            B2 = w * b * b, BC = w * b * c, BD = w * b * d,
            C2 = w * c * c, CD = w * c * d, D2 = w * d * d,
        };

        public void Add(in Quadric q)
        {
            A2 += q.A2; AB += q.AB; AC += q.AC; AD += q.AD; B2 += q.B2;
            BC += q.BC; BD += q.BD; C2 += q.C2; CD += q.CD; D2 += q.D2;
        }

        public double Eval(Vector3 v)
        {
            double x = v.X, y = v.Y, z = v.Z;
            return A2 * x * x + 2 * AB * x * y + 2 * AC * x * z + 2 * AD * x
                 + B2 * y * y + 2 * BC * y * z + 2 * BD * y
                 + C2 * z * z + 2 * CD * z + D2;
        }

        public bool Optimal(out Vector3 v)
        {
            double det = A2 * (B2 * C2 - BC * BC) - AB * (AB * C2 - BC * AC) + AC * (AB * BC - B2 * AC);
            v = default;
            if (Math.Abs(det) < 1e-12) return false;
            double inv = 1.0 / det;
            double x = -inv * (AD * (B2 * C2 - BC * BC) - AB * (BD * C2 - BC * CD) + AC * (BD * BC - B2 * CD));
            double y = -inv * (A2 * (BD * C2 - CD * BC) - AD * (AB * C2 - BC * AC) + AC * (AB * CD - BD * AC));
            double z = -inv * (A2 * (B2 * CD - BC * BD) - AB * (AB * CD - BD * AC) + AD * (AB * BC - B2 * AC));
            v = new Vector3((float)x, (float)y, (float)z);
            return true;
        }
    }

    /// <param name="vertexWeight">Optional per-vertex multiplier on quadric error (higher = keep detail).</param>
    /// <param name="maxEdge">Collapses that would create longer edges are rejected (keeps triangles small enough
    /// to skin well); relaxed by 1.5x per round only if the budget can't otherwise be met.</param>
    public static MeshData Decimate(MeshData src, int targetTris, float[] vertexWeight = null, float maxEdge = float.MaxValue)
    {
        var pos = (Vector3[])src.Pos.Clone();
        var tris = (int[])src.Tris.Clone();
        int nv = pos.Length, nt = tris.Length / 3;
        if (nt <= targetTris) return new MeshData(pos, tris);

        var q = new Quadric[nv];
        var vf = new List<int>[nv];
        for (int i = 0; i < nv; i++) vf[i] = new List<int>(8);
        var aliveT = new bool[nt];
        for (int t = 0; t < nt; t++)
        {
            aliveT[t] = true;
            int a = tris[3 * t], b = tris[3 * t + 1], c = tris[3 * t + 2];
            var n = (pos[b] - pos[a]).Cross(pos[c] - pos[a]);
            double area = n.Length() * 0.5;
            if (area > 0) n /= (float)(area * 2);
            var pl = Quadric.FromPlane(n.X, n.Y, n.Z, -n.Dot(pos[a]), area);
            q[a].Add(pl); q[b].Add(pl); q[c].Add(pl);
            vf[a].Add(t); vf[b].Add(t); vf[c].Add(t);
        }

        // boundary vertices are locked
        var locked = new bool[nv];
        var ec = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        for (int t = 0; t < nt; t++)
            for (int s = 0; s < 3; s++)
            {
                int a = tris[3 * t + s], b = tris[3 * t + (s + 1) % 3];
                long k = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                ec[k] = ec.TryGetValue(k, out int c) ? c + 1 : 1;
            }
        foreach (var kv in ec)
            if (kv.Value != 2) { locked[(int)(kv.Key >> 32)] = true; locked[(int)(kv.Key & 0xffffffff)] = true; }

        var version = new int[nv];
        var aliveV = new bool[nv];
        Array.Fill(aliveV, true);
        var heap = new PriorityQueue<(int a, int b, int va, int vb), double>();

        double W(int v) => vertexWeight != null ? vertexWeight[v] : 1.0;

        (double cost, Vector3 p) Cost(int a, int b)
        {
            var qq = q[a]; qq.Add(q[b]);
            Vector3 best;
            double bc;
            var mid = (pos[a] + pos[b]) * 0.5f;
            if (qq.Optimal(out var opt) && (opt - mid).Length() < 2 * (pos[a] - pos[b]).Length())
            { best = opt; bc = qq.Eval(opt); }
            else
            {
                best = mid; bc = qq.Eval(mid);
                double ca = qq.Eval(pos[a]), cb = qq.Eval(pos[b]);
                if (ca < bc) { bc = ca; best = pos[a]; }
                if (cb < bc) { bc = cb; best = pos[b]; }
            }
            return (Math.Max(bc, 0) * Math.Max(W(a), W(b)), best);
        }

        void Push(int a, int b)
        {
            if (locked[a] || locked[b]) return;
            var (c, _) = Cost(a, b);
            heap.Enqueue((a, b, version[a], version[b]), c);
        }

        var nbrA = new HashSet<int>();
        int alive = nt;
        float maxEdge2 = maxEdge * maxEdge;
        for (int round = 0; round < 12 && alive > targetTris; round++)
        {
        if (round > 0) { maxEdge2 *= 2.25f; heap.Clear(); }
        for (int t = 0; t < nt; t++)
        {
            if (!aliveT[t]) continue;
            for (int s = 0; s < 3; s++)
            {
                int x = tris[3 * t + s], y = tris[3 * t + (s + 1) % 3];
                if (x < y) Push(x, y);
            }
        }
        while (alive > targetTris && heap.Count > 0)
        {
            var (a, b, va, vb) = heap.Dequeue();
            if (!aliveV[a] || !aliveV[b] || version[a] != va || version[b] != vb) continue;
            var (_, target) = Cost(a, b);

            // link condition: exactly two shared neighbors
            nbrA.Clear();
            foreach (var t in vf[a]) for (int s = 0; s < 3; s++) nbrA.Add(tris[3 * t + s]);
            int shared = 0;
            var seenB = new HashSet<int>();
            foreach (var t in vf[b])
                for (int s = 0; s < 3; s++)
                {
                    int x = tris[3 * t + s];
                    if (x != a && x != b && nbrA.Contains(x) && seenB.Add(x)) shared++;
                }
            if (shared != 2) continue;

            // edge-length guard
            bool tooLong = false;
            foreach (var v in new[] { a, b })
            {
                foreach (var t in vf[v])
                    for (int s = 0; s < 3 && !tooLong; s++)
                    {
                        int x = tris[3 * t + s];
                        if (x != a && x != b && (pos[x] - target).LengthSquared() > maxEdge2) tooLong = true;
                    }
                if (tooLong) break;
            }
            if (tooLong) continue;

            // flip guard
            bool ok = true;
            foreach (var v in new[] { a, b })
            {
                foreach (var t in vf[v])
                {
                    int i0 = tris[3 * t], i1 = tris[3 * t + 1], i2 = tris[3 * t + 2];
                    bool hasA = i0 == a || i1 == a || i2 == a, hasB = i0 == b || i1 == b || i2 == b;
                    if (hasA && hasB) continue;
                    Vector3 p0 = pos[i0], p1 = pos[i1], p2 = pos[i2];
                    var nOld = (p1 - p0).Cross(p2 - p0);
                    if (i0 == v) p0 = target; else if (i1 == v) p1 = target; else p2 = target;
                    var nNew = (p1 - p0).Cross(p2 - p0);
                    if (nNew.LengthSquared() < 1e-14f * nOld.LengthSquared() ||
                        nNew.Normalized().Dot(nOld.Normalized()) < 0.2f) { ok = false; break; }
                }
                if (!ok) break;
            }
            if (!ok) continue;

            // collapse b into a
            pos[a] = target;
            q[a].Add(q[b]);
            foreach (var t in vf[b])
            {
                if (!aliveT[t]) continue;
                int i0 = tris[3 * t], i1 = tris[3 * t + 1], i2 = tris[3 * t + 2];
                if (i0 == a || i1 == a || i2 == a)
                {
                    aliveT[t] = false; alive--;
                    for (int s = 0; s < 3; s++) { int x = tris[3 * t + s]; if (x != b) vf[x].Remove(t); }
                }
                else
                {
                    for (int s = 0; s < 3; s++) if (tris[3 * t + s] == b) tris[3 * t + s] = a;
                    vf[a].Add(t);
                }
            }
            vf[b].Clear();
            aliveV[b] = false;
            version[a]++;
            nbrA.Clear();
            foreach (var t in vf[a]) for (int s = 0; s < 3; s++) nbrA.Add(tris[3 * t + s]);
            foreach (var x in nbrA) if (x != a) Push(a, x);
        }
        }

        var outTris = new List<int>(alive * 3);
        for (int t = 0; t < nt; t++)
            if (aliveT[t]) { outTris.Add(tris[3 * t]); outTris.Add(tris[3 * t + 1]); outTris.Add(tris[3 * t + 2]); }
        var m = new MeshData(pos, outTris.ToArray());
        m.CompactVertices();
        return m;
    }
}
