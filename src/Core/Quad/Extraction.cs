// Ported from Instant Meshes (src/extract.cpp: extract_graph, extract_faces).
// Copyright (c) 2015 Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung.
// BSD 3-Clause License — see THIRD_PARTY.md.
//
// Deviations from the reference (quad-dominant mode, posy = 4):
//  * sequential deterministic execution (the reference's "deterministic" mode);
//  * faces of degree 3 are kept as triangles and degree 5 is split into quad + triangle, instead of
//    fanning around a new centroid vertex (fewer irregular vertices; we never run pure-quad subdivision);
//  * the final smoothing/reprojection pass is done by ModelMate's guarded Relax instead.
using System;
using System.Collections.Generic;
using System.Linq;
using Godot;

namespace ModelMate.Core.Quad;

public static class Extraction
{
    public struct Report
    {
        public int Collapsed, Conflicts, Spurious, Snapped, EdgesRemoved, Faces, Holes, HolesSkipped;
        public override string ToString() =>
            $"collapsed={Collapsed} conflicts={Conflicts} spuriousRemoved={Spurious} snapped={Snapped} " +
            $"edgesRemoved={EdgesRemoved} faces={Faces} holesFilled={Holes} holesSkipped={HolesSkipped}";
    }

    sealed class DisjointSets
    {
        readonly int[] _p, _rank;
        public DisjointSets(int n) { _p = new int[n]; _rank = new int[n]; for (int i = 0; i < n; i++) _p[i] = i; }
        public int Find(int x)
        {
            while (_p[x] != x) { _p[x] = _p[_p[x]]; x = _p[x]; }
            return x;
        }
        /// <summary>Unites two roots; returns the new root.</summary>
        public int Unite(int a, int b)
        {
            if (_rank[a] < _rank[b]) (a, b) = (b, a);
            _p[b] = a;
            if (_rank[a] == _rank[b]) _rank[a]++;
            return a;
        }
    }

    public static PolyMesh Extract(Hierarchy h, out Report rep)
    {
        rep = default;
        var (adj, O, N) = ExtractGraph(h, ref rep);
        return ExtractFaces(adj, O, N, ref rep);
    }

    // ---------------------------------------------------------------- extract_graph

    static (List<int>[] adj, Vector3[] O, Vector3[] N) ExtractGraph(Hierarchy h, ref Report rep)
    {
        var l = h.Fine;
        int n = l.Count;
        var S = l.S;   // per-vertex target edge length
        var dset = new DisjointSets(n);
        var adjNew = new List<int>[n];
        for (int i = 0; i < n; i++) adjNew[i] = new List<int>();
        var collapse = new List<(int a, int b, float err)>();

        // Step 1: classify edges by integer lattice offset
        for (int i = 0; i < n; i++)
        {
            for (int k = l.AdjStart[i]; k < l.AdjStart[i + 1]; k++)
            {
                int j = l.AdjId[k];
                if (j < i) continue;
                var (qa, qb) = FieldMath.CompatOrient4(l.Q[i], l.N[i], l.Q[j], l.N[j]);
                float sij = 0.5f * (S[i] + S[j]);
                var (sa, sb) = FieldMath.CompatPositionIndex4(l.V[i], l.N[i], qa, l.O[i], l.V[j], l.N[j], qb, l.O[j], sij, 1f / sij, out float err);
                err /= sij * sij;   // compare collapse candidates relative to their local scale
                int dx = Math.Abs(sa.X - sb.X), dy = Math.Abs(sa.Y - sb.Y);
                if (Math.Max(dx, dy) > 1 || (dx == 1 && dy == 1)) continue; // long links and quad diagonals
                if (dx + dy == 0) collapse.Add((i, j, err));
                else { adjNew[i].Add(j); adjNew[j].Add(i); }
            }
        }

        // Step 2: collapse edges in order of increasing error
        collapse.Sort((x, y) => x.err != y.err ? x.err.CompareTo(y.err) : (x.a != y.a ? x.a.CompareTo(y.a) : x.b.CompareTo(y.b)));
        var nCollapses = new int[n];
        var temp = new HashSet<int>();
        foreach (var (a0, b0, _) in collapse)
        {
            int a = dset.Find(a0), b = dset.Find(b0);
            if (a == b) continue;
            if (a > b) (a, b) = (b, a);
            bool contained = false;
            foreach (var nb in adjNew[a]) if (dset.Find(nb) == b) { contained = true; break; }
            if (!contained) foreach (var nb in adjNew[b]) if (dset.Find(nb) == a) { contained = true; break; }
            if (contained) { rep.Conflicts++; continue; }
            temp.Clear();
            foreach (var nb in adjNew[a]) temp.Add(dset.Find(nb));
            foreach (var nb in adjNew[b]) temp.Add(dset.Find(nb));
            int target = dset.Unite(a, b);
            temp.Remove(target);
            int nc = nCollapses[a] + nCollapses[b] + 1;
            adjNew[a].Clear(); adjNew[b].Clear();
            adjNew[target].AddRange(temp);
            nCollapses[target] = nc;
            rep.Collapsed++;
        }

        // Step 3: assign vertices
        var vmap = new Dictionary<int, int>();
        var order = new List<int>();
        for (int i = 0; i < n; i++)
        {
            if (adjNew[i].Count == 0) continue;
            vmap[i] = order.Count;
            order.Add(i);
        }
        int nv = order.Count;
        var adj = new List<int>[nv];
        var col = new int[nv];
        double avgCollapses = 0;
        for (int vi = 0; vi < nv; vi++)
        {
            int i = order[vi];
            col[vi] = nCollapses[i];
            avgCollapses += col[vi];
            var set = new SortedSet<int>();
            foreach (var k in adjNew[i])
                if (vmap.TryGetValue(dset.Find(k), out int m) && m != vi) set.Add(m);
            adj[vi] = set.ToList();
        }
        // make symmetric (defensive; the reference relies on symmetry)
        for (int vi = 0; vi < nv; vi++)
            foreach (var m in adj[vi].ToArray())
                if (!adj[m].Contains(vi)) adj[m].Add(vi);
        avgCollapses /= Math.Max(1, nv);

        // Step 3a: remove spurious vertices (clusters that absorbed very few fine vertices)
        for (int i = 0; i < nv; i++)
        {
            if (col[i] > avgCollapses / 10) continue;
            foreach (var nb in adj[i]) adj[nb].Remove(i);
            adj[i].Clear();
            rep.Spurious++;
        }

        // Step 4: positions/normals as weighted averages of the fine lattice points
        var O = new Vector3[nv];
        var N = new Vector3[nv];
        var cw = new float[nv];
        var cs = new float[nv];   // cluster scale
        for (int i = 0; i < n; i++)
        {
            if (!vmap.TryGetValue(dset.Find(i), out int j)) continue;
            float inv = 1f / S[i];
            float w = MathF.Exp(-(l.O[i] - l.V[i]).LengthSquared() * inv * inv * 9);
            O[j] += l.O[i] * w;
            N[j] += l.N[i] * w;
            cs[j] += S[i] * w;
            cw[j] += w;
        }
        for (int i = 0; i < nv; i++)
        {
            if (cw[i] == 0) { cs[i] = h.Scale; continue; }
            O[i] /= cw[i];
            cs[i] /= cw[i];
            N[i] = N[i].Normalized();
        }

        // Step 5: snap vertices onto nearby edges / remove unnecessary diagonal edges
        RemoveUnnecessaryEdges(adj, O, N, cs, ref rep);

        // Step 6: orient edges (sort neighbors by angle in the tangent plane, descending)
        for (int i = 0; i < nv; i++)
        {
            var p = O[i];
            FieldMath.CoordinateSystem(N[i], out var s, out var t);
            adj[i].Sort((j0, j1) =>
            {
                Vector3 v0 = O[j0] - p, v1 = O[j1] - p;
                float a0 = MathF.Atan2(t.Dot(v0), s.Dot(v0)), a1 = MathF.Atan2(t.Dot(v1), s.Dot(v1));
                return a1.CompareTo(a0);
            });
        }
        return (adj, O, N);
    }

    static float Height(Vector3 pi, Vector3 pj, Vector3 pk)
    {
        float a = (pj - pk).Length(), b = (pi - pj).Length(), c = (pi - pk).Length();
        float s = 0.5f * (a + b + c);
        return 2 * MathF.Sqrt(MathF.Max(0, s * (s - a) * (s - b) * (s - c))) / a;
    }

    static void RemoveUnnecessaryEdges(List<int>[] adj, Vector3[] O, Vector3[] N, float[] scale, ref Report rep)
    {
        int guard = 0;
        bool changed;
        do
        {
            changed = false;
            bool changedInner;
            do
            {
                changedInner = false;
                var cands = new List<(float h, int i, int j, int k)>();
                for (int i = 0; i < adj.Length; i++)
                {
                    var pi = O[i];
                    float thresh = 0.3f * scale[i];
                    foreach (var j in adj[i])
                    {
                        var pj = O[j];
                        foreach (var k in adj[j])
                        {
                            if (k == i) continue;
                            var pk = O[k];
                            float a = (pj - pk).Length(), b = (pi - pj).Length(), c = (pi - pk).Length();
                            if (a > MathF.Max(b, c))
                            {
                                float hh = Height(pi, pj, pk);
                                if (hh < thresh) cands.Add((hh, i, j, k));
                            }
                        }
                    }
                }
                cands.Sort((x, y) => x.h.CompareTo(y.h));
                foreach (var (hh0, i, j, k) in cands)
                {
                    bool e1 = adj[i].Contains(j), e2 = adj[j].Contains(k), e3 = adj[k].Contains(i);
                    if (!e1 || !e2) continue;
                    var pi = O[i]; var pj = O[j]; var pk = O[k];
                    if (Height(pi, pj, pk) != hh0) continue;
                    float thresh = 0.3f * scale[i];
                    if ((pi - pj).Length() < thresh || (pi - pk).Length() < thresh)
                    {
                        int merge = (pi - pj).Length() < thresh ? j : k;
                        O[i] = (O[i] + O[merge]) * 0.5f;
                        N[i] = (N[i] + N[merge]) * 0.5f;
                        scale[i] = 0.5f * (scale[i] + scale[merge]);
                        var upd = new HashSet<int>();
                        foreach (var nb in adj[merge])
                        {
                            if (nb == i) continue;
                            upd.Add(nb);
                            var la = adj[nb];
                            for (int q = 0; q < la.Count; q++) if (la[q] == merge) la[q] = i;
                        }
                        foreach (var nb in adj[i]) upd.Add(nb);
                        upd.Remove(i); upd.Remove(merge);
                        adj[merge].Clear();
                        adj[i].Clear();
                        adj[i].AddRange(upd);
                        // dedupe lists that may now contain i twice
                        foreach (var nb in upd) { var la = adj[nb]; var d = la.Distinct().ToList(); la.Clear(); la.AddRange(d); }
                    }
                    else
                    {
                        O[i] = (pj + pk) * 0.5f;
                        N[i] = (N[j] + N[k]).Normalized();
                        adj[j].RemoveAll(x => x == k);
                        adj[k].RemoveAll(x => x == j);
                        if (!e3) { adj[i].Add(k); adj[k].Add(i); }
                    }
                    changed = changedInner = true;
                    rep.Snapped++;
                }
            } while (changedInner && ++guard < 100);

            // remove diagonals of quads that were extracted as two triangles
            var dc = new List<(float score, int i, int j)>();
            for (int i = 0; i < adj.Length; i++)
            {
                var pi = O[i];
                foreach (var j in adj[i])
                {
                    var pj = O[j];
                    int nTris = 0; float length = 0;
                    foreach (var k in adj[i])
                    {
                        if (k == j || !adj[j].Contains(k)) continue;
                        nTris++;
                        length += (O[k] - pi).Length() + (O[k] - pj).Length();
                    }
                    if (nTris == 2)
                    {
                        float exp = length / 4 * MathF.Sqrt(2f);
                        float diag = (pi - pj).Length();
                        float score = MathF.Abs((diag - exp) / MathF.Min(diag, exp));
                        dc.Add((score, i, j));
                    }
                }
            }
            dc.Sort((x, y) => x.score.CompareTo(y.score));
            foreach (var (_, i, j) in dc)
            {
                int nTris = 0;
                foreach (var k in adj[i]) if (adj[j].Contains(k)) nTris++;
                if (nTris == 2)
                {
                    adj[i].RemoveAll(x => x == j);
                    adj[j].RemoveAll(x => x == i);
                    changed = true;
                    rep.EdgesRemoved++;
                }
            }
        } while (changed && ++guard < 200);
    }

    // ---------------------------------------------------------------- extract_faces

    static PolyMesh ExtractFaces(List<int>[] adjL, Vector3[] O, Vector3[] N, ref Report rep)
    {
        int nv = adjL.Length;
        var adj = new int[nv][];
        var used = new byte[nv][];   // bit 1 = used, bit 2 = keep for hole filling
        for (int i = 0; i < nv; i++) { adj[i] = adjL[i].ToArray(); used[i] = new byte[adj[i].Length]; }
        var faces = new List<int[]>();
        var pos = new List<Vector3>(O);
        var nrm = new List<Vector3>(N);
        var result = new List<(int v, int e)>();

        bool ExtractFace(int cur, int curIdx, int targetSize)
        {
            int initial = cur;
            bool success = false;
            result.Clear();
            for (;;)
            {
                if ((used[cur][curIdx] & 1) != 0 || (targetSize > 0 && result.Count + 1 > targetSize)) break;
                result.Add((cur, curIdx));
                int next = adj[cur][curIdx], rank = adj[next].Length, idx = -1;
                for (int j = 0; j < rank; j++) if (adj[next][j] == cur) { idx = j; break; }
                if (idx == -1 || rank == 1) break;
                cur = next;
                curIdx = (idx + 1) % rank;
                if (cur == initial)
                {
                    success = targetSize == 0 || result.Count == targetSize;
                    break;
                }
            }
            if (success) foreach (var (v, e) in result) used[v][e] |= 1;
            else result.Clear();
            return success;
        }

        float QuadScore(List<int> verts, int start)
        {
            float score = 0;
            for (int k = 0; k < 4; k++)
            {
                var v0 = pos[verts[(start + k) % verts.Count]];
                var v1 = pos[verts[(start + k + 1) % verts.Count]];
                var v2 = pos[verts[(start + k + 2) % verts.Count]];
                var d0 = (v0 - v1).Normalized(); var d1 = (v2 - v1).Normalized();
                score += MathF.Abs(MathF.Acos(Math.Clamp(d0.Dot(d1), -1f, 1f)) * 180f / MathF.PI - 90f);
            }
            return score;
        }

        void FillFace()
        {
            var verts = result.Select(r => r.v).ToList();
            while (verts.Count > 4)
            {
                int best = 0; float bs = float.MaxValue;
                for (int i = 0; i < verts.Count; i++) { float s = QuadScore(verts, i); if (s < bs) { bs = s; best = i; } }
                var q = new int[4];
                for (int k = 0; k < 4; k++) q[k] = verts[(best + k) % verts.Count];
                faces.Add(q);
                // remove the two interior corners of the cut quad
                int r1 = (best + 1) % verts.Count, r2 = (best + 2) % verts.Count;
                var keep = new List<int>();
                for (int i = 0; i < verts.Count; i++) if (i != r1 && i != r2) keep.Add(verts[i]);
                verts = keep;
            }
            if (verts.Count >= 3) faces.Add(verts.ToArray());
        }

        for (int deg = 3; deg <= 8; deg++)
            for (int i = 0; i < nv; i++)
                for (int j = 0; j < adj[i].Length; j++)
                {
                    if (!ExtractFace(i, j, deg)) continue;
                    FillFace();
                    rep.Faces++;
                }

        // Step 8: fill holes — keep only unused half-edges whose twin is used (hole boundaries)
        for (int i = 0; i < nv; i++)
            for (int j = 0; j < adj[i].Length; j++)
            {
                if ((used[i][j] & 1) != 0) continue;
                int jid = adj[i][j];
                for (int k = 0; k < adj[jid].Length; k++)
                    if (adj[jid][k] == i)
                    {
                        if ((used[jid][k] & 1) != 0) { used[i][j] |= 2; used[jid][k] |= 2; }
                        break;
                    }
            }
        for (int i = 0; i < nv; i++)
        {
            var keepA = new List<int>(); var keepU = new List<byte>();
            for (int j = 0; j < adj[i].Length; j++)
                if ((used[i][j] & 2) != 0) { keepA.Add(adj[i][j]); keepU.Add(used[i][j]); }
            adj[i] = keepA.ToArray(); used[i] = keepU.ToArray();
        }
        for (int i = 0; i < nv; i++)
            for (int j = 0; j < adj[i].Length; j++)
            {
                if (!ExtractFace(i, j, 0)) continue;
                if (result.Count >= 7) { rep.HolesSkipped++; continue; }
                FillFace();
                rep.Holes++;
            }

        var pm = new PolyMesh { Pos = pos.ToArray(), Nrm = nrm.ToArray(), Faces = faces };
        pm.OrientConsistently();
        pm.Compact();
        return pm;
    }
}
