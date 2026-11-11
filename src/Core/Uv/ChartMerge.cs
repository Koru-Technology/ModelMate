using System;
using System.Collections.Generic;
using Godot;

namespace ModelMate.Core.Uv;

/// <summary>
/// A flattened chart. Its vertices are groups of triangle corners: corners of one welded vertex that are connected
/// around it without crossing a cut. A chart can therefore contain internal seams (<see cref="Cuts"/>), e.g. the
/// cut that opens a ring-shaped chart around a limb.
/// </summary>
public sealed class FlatChart
{
    public List<int> Tris;              // welded triangle ids
    public int[] TriLocal;              // 3 local vertex ids per triangle (corner -> local vertex)
    public int[] Verts;                 // local vertex -> welded vertex
    public Vector2[] Uv;                // per local vertex, world units
    public int[] Bnd;                   // boundary corners (3·i + k: edge k → k + 1 of triangle i), incl. both sides of cuts
    public HashSet<long> Cuts;          // welded edges (vertex-pair keys) cut open inside the chart
    public ChartQuality Quality;
    public string Method;               // "planar", "lscm" or "arap"
    internal int Ver;
    internal bool Alive = true;
}

/// <summary>
/// Bottom-up chart merging. Starts from small, always-valid charts and repeatedly tries to join the two adjacent
/// charts whose shared boundary is the most visible seam (<see cref="SeamCost"/>). If the union closes a ring, it is
/// first tried as an island with a hole; if that doesn't flatten, it is cut open along the cheapest paths between its
/// boundary loops, provided that cut costs well under the seam the join removes. A join is kept only if the result
/// flattens (LSCM, with ARAP as a rescue) without flips, overlaps or more than the allowed distortion. Afterwards
/// <see cref="Refine"/> moves the most visible remaining seams, and a final pass re-solves each chart globally.
/// </summary>
public static class ChartMerge
{
    public const float MaxAniso = 1.3f;        // area-weighted p95 of s1/s2
    public const float MaxStretch = 1.45f;     // area-weighted p95 of the area stretch (the layout check allows 1.6 overall)
    public const float MaxStretchWide = 2.2f;  // p99
    const float RescueStretch = 3f;            // LSCM results up to this stretch get an ARAP attempt
    const float RetryGrowth = 1.5f;            // a rejected pair is retried once the two charts grew this much
    const int MaxCutPaths = 6;
    const float CutGain = 0.6f;                // a merge that must be cut open may add at most this share of the seam it removes
    const float LocalRatio = 0.25f;            // smaller chart under this fraction of the larger: solve locally
    const int LocalRings = 6;                  // rings of the larger chart around the join that may move

    public struct Stats
    {
        public int Initial, Final, Trials, Merged, Cut, RejectedTopology, RejectedDistortion, RejectedOverlap, Rescued, Polished;
        public int RefineTried, Refined;
        public long MsRefine;
        public long MsLscm, MsArap, MsOverlap, ItLscm, ItArap, MsTopo, MsFinal, MsGlobal, MsLocal;
        public int NGlobal, NLocal, VertsGlobal;
    }
    /// <summary>Print every merge/refinement decision and per-chart distortion (CLI --uv-debug).</summary>
    public static bool Debug;

    public static bool Acceptable(ChartQuality q) =>
        !q.Flipped && q.AnisoP95 <= MaxAniso && q.AreaStretch <= MaxStretch && q.AreaStretchWide <= MaxStretchWide;

    static float Score(ChartQuality q) => q.Flipped ? float.MaxValue : q.AreaStretch * q.AreaStretch * q.AnisoP95 * q.AnisoP95;

    public static long Key(int a, int b) => a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

    sealed class Ctx
    {
        public MeshData M;
        public TriGeom G;
        public int[] Nbr;
        public float[] Cost;
        public int[] TriChart;   // chart slot of each triangle
        public int[] TriPos;     // index of each triangle in its chart's Tris
        public int[] Mark;       // scratch: stamp per triangle
        public int[] Idx;        // scratch: index of a triangle in the set being built
        public int Stamp;
    }

    public static List<FlatChart> Run(MeshData m, TriGeom g, int[] nbr, List<(List<int> tris, Dictionary<int, Vector2> planar)> initial,
        float[] edgeCost, out Stats st)
    {
        st = new Stats { Initial = initial.Count };
        int nt = m.TriCount;
        var x = new Ctx
        {
            M = m, G = g, Nbr = nbr, Cost = edgeCost,
            TriChart = new int[nt], TriPos = new int[nt], Mark = new int[nt], Idx = new int[nt],
        };
        var charts = new List<FlatChart>(initial.Count);

        // initial charts: conformal flattening from the planar projection, kept when at least as good and valid
        foreach (var (tris, planar) in initial)
        {
            var cuts = new HashSet<long>();
            var (tl, verts, _, _, bnd) = Topology(x, tris, cuts);
            var warm = new Vector2[verts.Length];
            for (int i = 0; i < verts.Length; i++) warm[i] = planar[verts[i]];
            var c = new FlatChart { Tris = tris, TriLocal = tl, Verts = verts, Uv = warm, Cuts = cuts, Bnd = bnd, Method = "planar" };
            c.Quality = ChartFlatten.Measure(g, tris, tl, warm);
            if (tris.Count > 1)
            {
                var r = ChartFlatten.Lscm(g, tris, tl, warm);
                if (!r.Quality.Flipped && Score(r.Quality) <= Score(c.Quality) * 1.0001f && !ChartFlatten.Overlaps(tl, bnd, r.Uv))
                { c.Uv = r.Uv; c.Quality = r.Quality; c.Method = "lscm"; }
            }
            Assign(x, c, charts.Count);
            charts.Add(c);
        }

        var pq = new PriorityQueue<(int a, int b, int va, int vb), float>();
        var rejectedAt = new Dictionary<long, int>();   // pair -> combined triangle count when it was last rejected
        for (int i = 0; i < charts.Count; i++)
            foreach (var (o, cost) in Neighbours(x, charts[i], i))
                if (o > i) pq.Enqueue((i, o, charts[i].Ver, charts[o].Ver), -cost);

        int maxTrials = 6 * initial.Count + 200;
        float prio = 0;
        while (st.Trials < maxTrials && pq.TryDequeue(out var e, out prio))
        {
            var A = charts[e.a]; var B = charts[e.b];
            if (!A.Alive || !B.Alive || A.Ver != e.va || B.Ver != e.vb) continue;
            long pk = Key(e.a, e.b);
            int size = A.Tris.Count + B.Tris.Count;
            if (rejectedAt.TryGetValue(pk, out int was) && size < RetryGrowth * was) continue;
            st.Trials++;
            var merged = TryMerge(x, A, B, -prio, ref st);
            if (merged == null) { rejectedAt[pk] = size; continue; }
            merged.Ver = A.Ver + B.Ver + 1;
            charts[e.a] = merged;
            B.Alive = false;
            Assign(x, merged, e.a);
            st.Merged++;
            foreach (var (o, cost) in Neighbours(x, merged, e.a))
                pq.Enqueue((e.a, o, merged.Ver, charts[o].Ver), -cost);
        }

        var swr = System.Diagnostics.Stopwatch.StartNew();
        Refine(x, charts, ref st);
        st.MsRefine = swr.ElapsedMilliseconds;

        var swf = System.Diagnostics.Stopwatch.StartNew();
        // final pass: a global solve of each chart (merges may have solved locally), then ARAP to even out the
        // area scale a conformal map leaves on curved charts; the best valid layout wins
        var result = new List<FlatChart>();
        foreach (var c in charts)
        {
            if (!c.Alive) continue;
            if (c.Tris.Count > 2)
            {
                var sw = System.Diagnostics.Stopwatch.StartNew();
                var rl = ChartFlatten.Lscm(g, c.Tris, c.TriLocal, c.Uv);
                st.MsLscm += sw.ElapsedMilliseconds; st.ItLscm += rl.Iterations;
                sw.Restart();
                bool fromLscm = !rl.Quality.Flipped && Score(rl.Quality) < Score(c.Quality);
                if (fromLscm && rl.Quality.AnisoP95 <= MaxAniso && !ChartFlatten.Overlaps(c.TriLocal, c.Bnd, rl.Uv))
                { c.Uv = rl.Uv; c.Quality = rl.Quality; c.Method = "lscm"; }
                // ARAP until it would make the chart overlap itself
                foreach (int its in new[] { 10, 4, 2 })
                {
                    var ra = ChartFlatten.Arap(g, c.Tris, c.TriLocal, c.Uv, null, its);
                    st.ItArap += ra.Iterations;
                    bool ok = !ra.Quality.Flipped && Score(ra.Quality) < Score(c.Quality) && ra.Quality.AnisoP95 <= MaxAniso
                        && !ChartFlatten.Overlaps(c.TriLocal, c.Bnd, ra.Uv);
                    if (Debug) GD.Print($"  final pass {c.Tris.Count} tris: {c.Method} {c.Quality}; lscm {rl.Quality}; arap×{its} {ra.Quality} -> {(ok ? "kept" : "not kept")}");
                    if (ok) { c.Uv = ra.Uv; c.Quality = ra.Quality; c.Method = "arap"; st.Polished++; break; }
                }
                st.MsArap += sw.ElapsedMilliseconds;
            }
            // local solves keep world units without rescaling; give every chart the same average texel density
            ChartFlatten.Normalize(g, c.Tris, c.TriLocal, c.Uv);
            result.Add(c);
            if (Debug) GD.Print($"  chart {result.Count - 1}: {c.Tris.Count} tris, {c.Method}, {c.Quality}, {c.Cuts.Count} cut edges");
        }
        st.Final = result.Count;
        st.MsFinal = swf.ElapsedMilliseconds;
        return result;
    }

    // ------------------------------------------------------------------ boundary refinement

    const float RefineGain = 0.97f;              // keep a new boundary only if it costs at most this much of the old
    // share of each side's depth that may change hands; negative: that many rings (a narrow band that only smooths)
    static readonly float[] RefineFreedom = { 0.6f, 0.25f, -3f };
    const int RefinePairs = 10;                  // the most visible seams are refined; the rest cost little anyway

    /// <summary>
    /// Moves the seam between each pair of adjacent charts to where it costs least. The two charts' triangles are
    /// split by a minimum cut (capacity of a triangle adjacency = seam cost of that edge; quads are never split)
    /// between the deepest parts of each chart, so the new seam takes the cheapest route within the band that may
    /// change hands, which also straightens the stair-stepped borders region growing leaves. The band is tried
    /// wide first; a split is kept when both charts still flatten validly and the seam cost drops.
    /// </summary>
    static void Refine(Ctx x, List<FlatChart> charts, ref Stats st)
    {
        var tried = new HashSet<(int, int, int, int)>();   // pairs already tried in their current state
        for (int round = 0; round < 2; round++)
        {
            var pairs = new List<(int a, int b, float cost)>();
            for (int i = 0; i < charts.Count; i++)
                if (charts[i].Alive)
                    foreach (var (o, cost) in Neighbours(x, charts[i], i))
                        if (o > i) pairs.Add((i, o, cost));
            pairs.Sort((p, q) => q.cost.CompareTo(p.cost));
            if (pairs.Count > RefinePairs) pairs.RemoveRange(RefinePairs, pairs.Count - RefinePairs);
            bool any = false;
            foreach (var (a, b, _) in pairs)
            {
                if (!tried.Add((a, b, charts[a].Ver, charts[b].Ver))) continue;
                foreach (float f in RefineFreedom)
                {
                    st.RefineTried++;
                    if (TryRefine(x, charts, a, b, f, ref st)) { st.Refined++; any = true; break; }
                }
            }
            if (!any) break;
        }
    }

    static bool TryRefine(Ctx x, List<FlatChart> charts, int sa, int sb, float freedom, ref Stats st)
    {
        var m = x.M;
        var A = charts[sa]; var B = charts[sb];
        var U = new List<int>(A.Tris.Count + B.Tris.Count);
        U.AddRange(A.Tris); U.AddRange(B.Tris);
        int n = U.Count;
        var idx = new Dictionary<int, int>(n);
        for (int i = 0; i < n; i++) idx[U[i]] = i;
        bool InA(int i) => i < A.Tris.Count;

        // depth of every triangle from the A–B border, within its own chart
        var depth = new int[n];
        Array.Fill(depth, -1);
        var q = new Queue<int>();
        for (int i = 0; i < n; i++)
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * U[i] + k];
                if (o >= 0 && idx.TryGetValue(o, out int j) && InA(j) != InA(i)) { depth[i] = 0; q.Enqueue(i); break; }
            }
        if (q.Count == 0) return false;
        while (q.Count > 0)
        {
            int i = q.Dequeue();
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * U[i] + k];
                if (o >= 0 && idx.TryGetValue(o, out int j) && InA(j) == InA(i) && depth[j] < 0) { depth[j] = depth[i] + 1; q.Enqueue(j); }
            }
        }
        int maxA = 0, maxB = 0;
        for (int i = 0; i < n; i++) if (InA(i)) maxA = Math.Max(maxA, depth[i]); else maxB = Math.Max(maxB, depth[i]);

        // min cut between the cores
        var cuts = new HashSet<long>(A.Cuts);
        cuts.UnionWith(B.Cuts);
        float EdgeCap(int t, int k)
        {
            int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
            if (m.TriFlags != null && (m.TriFlags[t] & (1 << k)) != 0) return 1e9f;   // quad diagonal: never cut
            return cuts.Contains(Key(a, b)) ? 0 : x.Cost[3 * t + k];
        }
        var mf = new MaxFlow(n + 2);
        int S = n, T = n + 1;
        for (int i = 0; i < n; i++)
        {
            int t = U[i];
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * t + k];
                if (o < 0 || !idx.TryGetValue(o, out int j) || j < i) continue;
                double c = EdgeCap(t, k);
                mf.Add(i, j, c, c);
            }
            if (depth[i] < 0) continue;
            // cores: the part of each chart deeper than the band that may change hands
            float coreA = freedom < 0 ? -freedom : freedom * maxA, coreB = freedom < 0 ? -freedom : freedom * maxB;
            if (InA(i) && depth[i] >= Math.Max(1, coreA)) mf.Add(S, i, 1e9, 0);
            else if (!InA(i) && depth[i] >= Math.Max(1, coreB)) mf.Add(i, T, 1e9, 0);
        }
        mf.Run(S, T);
        var side = mf.SourceSide(S);   // true: goes to A
        var toA = new bool[n];
        for (int i = 0; i < n; i++) toA[i] = side[i];
        // keep each side connected to its core: stray pieces join the side around them
        FixComponents(x, U, idx, toA, true, cuts);
        FixComponents(x, U, idx, toA, false, cuts);

        double oldCost = 0, newCost = 0;
        bool changed = false;
        for (int i = 0; i < n; i++)
        {
            if (toA[i] != InA(i)) changed = true;
            int t = U[i];
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * t + k];
                if (o < 0 || !idx.TryGetValue(o, out int j) || j < i) continue;
                bool cut = cuts.Contains(Key(m.Tris[3 * t + k], m.Tris[3 * t + (k + 1) % 3]));
                if (cut || InA(i) != InA(j)) oldCost += x.Cost[3 * t + k];
                if (cut || toA[i] != toA[j]) newCost += x.Cost[3 * t + k];
            }
        }
        if (!changed || newCost > RefineGain * oldCost) return false;

        // new charts: kept triangles first (their layout stays), gained ones after
        var aTris = new List<int>(); var bTris = new List<int>();
        for (int i = 0; i < n; i++) if (toA[i] && InA(i)) aTris.Add(U[i]); else if (!toA[i] && !InA(i)) bTris.Add(U[i]);
        int nA = aTris.Count, nB = bTris.Count;
        if (nA == 0 || nB == 0) return false;
        for (int i = 0; i < n; i++) if (toA[i] && !InA(i)) aTris.Add(U[i]); else if (!toA[i] && InA(i)) bTris.Add(U[i]);
        var cutsA = new HashSet<long>(cuts); var cutsB = new HashSet<long>(cuts);
        var nA2 = Make(x, A, B, aTris, nA, cutsA, ref st, out string whyA, true);
        if (nA2 == null) { if (Debug) GD.Print($"  refine {A.Tris.Count}|{B.Tris.Count} (freedom {freedom}): A side rejected: {whyA}"); return false; }
        var nB2 = Make(x, B, A, bTris, nB, cutsB, ref st, out string whyB, true);
        if (nB2 == null) { if (Debug) GD.Print($"  refine {A.Tris.Count}|{B.Tris.Count} (freedom {freedom}): B side rejected: {whyB}"); return false; }
        // cutting either side open may have added seams; recount
        double finalCost = 0;
        for (int i = 0; i < n; i++)
        {
            int t = U[i];
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * t + k];
                if (o < 0 || !idx.TryGetValue(o, out int j) || j < i) continue;
                long key = Key(m.Tris[3 * t + k], m.Tris[3 * t + (k + 1) % 3]);
                if (toA[i] != toA[j] || (toA[i] ? nA2.Cuts : nB2.Cuts).Contains(key)) finalCost += x.Cost[3 * t + k];
            }
        }
        if (Debug) GD.Print($"  refine {A.Tris.Count}|{B.Tris.Count} (freedom {freedom}): -> {nA2.Tris.Count}|{nB2.Tris.Count}, seam cost {oldCost:F3} -> {finalCost:F3}");
        if (finalCost > RefineGain * oldCost) return false;
        nA2.Ver = A.Ver + B.Ver + 1; nB2.Ver = nA2.Ver + 1;
        charts[sa] = nA2; charts[sb] = nB2;
        Assign(x, nA2, sa); Assign(x, nB2, sb);
        return true;
    }

    /// <summary>Triangles of one side not connected (within the side) to its deepest triangle move to the other side.</summary>
    static void FixComponents(Ctx x, List<int> U, Dictionary<int, int> idx, bool[] toA, bool sideA, HashSet<long> cuts)
    {
        var m = x.M;
        int n = U.Count;
        var comp = new int[n];
        Array.Fill(comp, -1);
        var sizes = new List<int>();
        for (int s0 = 0; s0 < n; s0++)
        {
            if (toA[s0] != sideA || comp[s0] >= 0) continue;
            int id = sizes.Count, size = 0;
            var st = new Stack<int>();
            st.Push(s0); comp[s0] = id;
            while (st.Count > 0)
            {
                int i = st.Pop(); size++;
                for (int k = 0; k < 3; k++)
                {
                    int o = x.Nbr[3 * U[i] + k];
                    if (o >= 0 && idx.TryGetValue(o, out int j) && toA[j] == sideA && comp[j] < 0
                        && !cuts.Contains(Key(m.Tris[3 * U[i] + k], m.Tris[3 * U[i] + (k + 1) % 3]))) { comp[j] = id; st.Push(j); }
                }
            }
            sizes.Add(size);
        }
        if (sizes.Count <= 1) return;
        int keep = 0;
        for (int c = 1; c < sizes.Count; c++) if (sizes[c] > sizes[keep]) keep = c;
        for (int i = 0; i < n; i++) if (toA[i] == sideA && comp[i] != keep) toA[i] = !sideA;
    }

    static void Assign(Ctx x, FlatChart c, int slot)
    {
        for (int i = 0; i < c.Tris.Count; i++) { x.TriChart[c.Tris[i]] = slot; x.TriPos[c.Tris[i]] = i; }
    }

    static Dictionary<int, float> Neighbours(Ctx x, FlatChart c, int id)
    {
        var d = new Dictionary<int, float>();
        foreach (int t in c.Tris)
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * t + k];
                if (o < 0) continue;
                int oc = x.TriChart[o];
                if (oc == id) continue;
                d[oc] = (d.TryGetValue(oc, out var v) ? v : 0) + x.Cost[3 * t + k];
            }
        return d;
    }

    /// <summary>
    /// Corner grouping of a triangle set with cuts: local vertex per corner, local -> welded, Euler characteristic
    /// of the cut-open surface and its number of boundary loops. Uses the Mark/Idx scratch arrays.
    /// </summary>
    static (int[] tl, int[] verts, int chi, int loops, int[] bnd) Topology(Ctx x, List<int> tris, HashSet<long> cuts)
    {
        var m = x.M;
        int f = tris.Count;
        int stamp = ++x.Stamp;
        for (int i = 0; i < f; i++) { x.Mark[tris[i]] = stamp; x.Idx[tris[i]] = i; }
        var parent = new int[3 * f];
        for (int i = 0; i < parent.Length; i++) parent[i] = i;
        int Find(int a) { while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; }
        void Union(int a, int b) { a = Find(a); b = Find(b); if (a != b) parent[b] = a; }
        int boundaryHalf = 0;
        for (int i = 0; i < f; i++)
        {
            int t = tris[i];
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * t + k];
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                if (o < 0 || x.Mark[o] != stamp || (cuts.Count > 0 && cuts.Contains(Key(a, b)))) { boundaryHalf++; continue; }
                int j = x.Idx[o];
                if (j < i) continue;   // each interior edge once
                for (int kk = 0; kk < 3; kk++)
                {
                    int v = m.Tris[3 * o + kk];
                    if (v == a) Union(3 * i + k, 3 * j + kk);
                    else if (v == b) Union(3 * i + (k + 1) % 3, 3 * j + kk);
                }
            }
        }
        var tl = new int[3 * f];
        var id = new Dictionary<int, int>();
        var verts = new List<int>();
        for (int c = 0; c < 3 * f; c++)
        {
            int r = Find(c);
            if (!id.TryGetValue(r, out int l)) { l = verts.Count; id[r] = l; verts.Add(m.Tris[3 * tris[c / 3] + c % 3]); }
            tl[c] = l;
        }
        // boundary loops: components of local vertices joined by boundary half-edges
        int nv = verts.Count;
        var lp = new int[nv];
        for (int i = 0; i < nv; i++) lp[i] = i;
        int LFind(int a) { while (lp[a] != a) { lp[a] = lp[lp[a]]; a = lp[a]; } return a; }
        var onB = new bool[nv];
        var bnd = new int[boundaryHalf];
        int nb = 0;
        for (int i = 0; i < f; i++)
        {
            int t = tris[i];
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * t + k];
                int a = m.Tris[3 * t + k], b = m.Tris[3 * t + (k + 1) % 3];
                if (!(o < 0 || x.Mark[o] != stamp || (cuts.Count > 0 && cuts.Contains(Key(a, b))))) continue;
                bnd[nb++] = 3 * i + k;
                int la = tl[3 * i + k], lb = tl[3 * i + (k + 1) % 3];
                onB[la] = onB[lb] = true;
                int ra = LFind(la), rb = LFind(lb);
                if (ra != rb) lp[rb] = ra;
            }
        }
        int loops = 0;
        for (int i = 0; i < nv; i++) if (onB[i] && LFind(i) == i) loops++;
        int chi = nv - (3 * f + boundaryHalf) / 2 + f;
        return (tl, verts.ToArray(), chi, loops, bnd);
    }

    /// <summary>
    /// Cuts a genus-0 surface with several boundary loops open into a disk: repeatedly adds the cheapest path
    /// (seam cost) between one loop and any other. Returns false if it is not a disk afterwards.
    /// </summary>
    static bool CutToDisk(Ctx x, List<int> tris, HashSet<long> cuts, ref int[] tl, ref int[] verts, ref int chi, ref int loops, ref int[] bnd, out float added)
    {
        var m = x.M;
        added = 0;
        for (int iter = 0; iter < MaxCutPaths && chi != 1; iter++)
        {
            if (loops < 2 || chi != 2 - loops) return false;   // closed, or has a handle: no single cut tree we try
            int f = tris.Count, nv = verts.Length;
            int stamp = ++x.Stamp;
            for (int i = 0; i < f; i++) { x.Mark[tris[i]] = stamp; x.Idx[tris[i]] = i; }
            bool IsB(int t, int k, int o) => o < 0 || x.Mark[o] != stamp || cuts.Contains(Key(m.Tris[3 * t + k], m.Tris[3 * t + (k + 1) % 3]));
            // loop label per boundary local vertex
            var lp = new int[nv];
            for (int i = 0; i < nv; i++) lp[i] = i;
            int LFind(int a) { while (lp[a] != a) { lp[a] = lp[lp[a]]; a = lp[a]; } return a; }
            var onB = new bool[nv];
            // interior adjacency between local vertices, with the half-edge it came from
            var adj = new List<(int to, float w, long key)>[nv];
            for (int i = 0; i < nv; i++) adj[i] = new List<(int, float, long)>(6);
            for (int i = 0; i < f; i++)
            {
                int t = tris[i];
                for (int k = 0; k < 3; k++)
                {
                    int o = x.Nbr[3 * t + k];
                    int la = tl[3 * i + k], lb = tl[3 * i + (k + 1) % 3];
                    if (IsB(t, k, o))
                    {
                        onB[la] = onB[lb] = true;
                        int ra = LFind(la), rb = LFind(lb);
                        if (ra != rb) lp[rb] = ra;
                        continue;
                    }
                    if (x.Idx[o] < i) continue;
                    if (m.TriFlags != null && (m.TriFlags[t] & (1 << k)) != 0) continue;   // never cut a quad in two
                    float w = MathF.Max(x.Cost[3 * t + k], 1e-9f);
                    long key = Key(m.Tris[3 * t + k], m.Tris[3 * t + (k + 1) % 3]);
                    adj[la].Add((lb, w, key));
                    adj[lb].Add((la, w, key));
                }
            }
            // source: the loop with the most vertices (usually the outer boundary); targets: every other loop
            var count = new Dictionary<int, int>();
            for (int i = 0; i < nv; i++) if (onB[i]) { int r = LFind(i); count[r] = (count.TryGetValue(r, out int c0) ? c0 : 0) + 1; }
            int src = -1, best = -1;
            foreach (var (r, c) in count) if (c > best) { best = c; src = r; }
            var dist = new float[nv];
            var prev = new int[nv];
            var prevKey = new long[nv];
            Array.Fill(dist, float.MaxValue);
            Array.Fill(prev, -1);
            var pq = new PriorityQueue<int, float>();
            for (int i = 0; i < nv; i++) if (onB[i] && LFind(i) == src) { dist[i] = 0; pq.Enqueue(i, 0); }
            int hit = -1;
            while (pq.TryDequeue(out int u, out float du))
            {
                if (du > dist[u]) continue;
                if (onB[u] && LFind(u) != src) { hit = u; break; }
                foreach (var (to, w, key) in adj[u])
                {
                    float nd = du + w;
                    if (nd < dist[to]) { dist[to] = nd; prev[to] = u; prevKey[to] = key; pq.Enqueue(to, nd); }
                }
            }
            if (hit < 0) return false;
            added += dist[hit];
            for (int v = hit; prev[v] >= 0; v = prev[v]) cuts.Add(prevKey[v]);
            (tl, verts, chi, loops, bnd) = Topology(x, tris, cuts);
        }
        return chi == 1;
    }

    static FlatChart TryMerge(Ctx x, FlatChart A, FlatChart B, float prio, ref Stats st)
    {
        var tris = new List<int>(A.Tris.Count + B.Tris.Count);
        tris.AddRange(A.Tris); tris.AddRange(B.Tris);
        var cuts = new HashSet<long>(A.Cuts);
        cuts.UnionWith(B.Cuts);
        var c = Make(x, A, B, tris, A.Tris.Count, cuts, ref st, out string why, maxCutCost: CutGain * prio);
        if (c == null) Dbg(A, B, prio, why);
        return c;
    }

    /// <summary>
    /// Builds and flattens a chart from triangles of two current charts: tris[0, nP) come from P, the rest from Q.
    /// Genus 0 is required; holes are fine (an island with a hole is still one island), so it is first flattened as
    /// it is, and cut open along the cheapest paths only if that fails. Null (with the reason) when rejected.
    /// </summary>
    /// <param name="keepP">P's triangles keep their layout and only Q's (plus a band) are solved, however many</param>
    /// <param name="maxCutCost">a cut that opens the chart into a disk must cost less than this (a merge must not add
    /// more visible seam than it removes); infinite when the caller weighs the costs itself</param>
    static FlatChart Make(Ctx x, FlatChart P, FlatChart Q, List<int> tris, int nP, HashSet<long> cuts, ref Stats st, out string why,
        bool keepP = false, float maxCutCost = float.MaxValue)
    {
        var swt = System.Diagnostics.Stopwatch.StartNew();
        var (tl, verts, chi, loops, bnd) = Topology(x, tris, cuts);
        st.MsTopo += swt.ElapsedMilliseconds;
        if (loops < 1 || chi != 2 - loops) { st.RejectedTopology++; why = $"topology χ={chi}, {loops} boundary loops"; return null; }
        if (loops > 1)
        {
            int holes = loops - 1;
            var r0 = Flatten(x, P, Q, tris, nP, tl, verts, bnd, ref st, out why, keepP);
            if (r0 != null) return new FlatChart { Tris = tris, TriLocal = tl, Verts = verts, Uv = r0.Uv, Cuts = cuts, Bnd = bnd, Quality = r0.Quality, Method = why };
            if (!CutToDisk(x, tris, cuts, ref tl, ref verts, ref chi, ref loops, ref bnd, out float added))
            { st.RejectedTopology++; why = $"with {holes} hole(s): {why}; could not cut open (χ={chi}, {loops} loops)"; return null; }
            if (Debug) GD.Print($"  cut open {tris.Count} tris ({nP}+{tris.Count - nP}): added cut cost {added:F3} vs limit {maxCutCost:F3}");
            if (added >= maxCutCost)
            { st.RejectedTopology++; why = $"with {holes} hole(s): {why}; cutting it open would cost {added:F3}, over the limit {maxCutCost:F3}"; return null; }
            st.Cut++;
        }
        var r = Flatten(x, P, Q, tris, nP, tl, verts, bnd, ref st, out why, keepP);
        if (r == null) return null;
        return new FlatChart { Tris = tris, TriLocal = tl, Verts = verts, Uv = r.Uv, Cuts = cuts, Bnd = bnd, Quality = r.Quality, Method = why };
    }

    /// <summary>
    /// Flattens a candidate chart warm-started from the current layouts of P and Q (Q placed onto P by the similarity
    /// that best matches the corners they share). When one part is much smaller than the other, only it and a band
    /// around it move. Null with the reason when rejected, else the method in why.
    /// </summary>
    static ChartFlatten.Result Flatten(Ctx x, FlatChart P, FlatChart Q, List<int> tris, int nP, int[] tl, int[] verts, int[] bnd, ref Stats st, out string why, bool keepP = false)
    {
        var (z, tr) = Similarity(x, P, Q);
        int pSlot = x.TriChart[P.Tris[0]];
        var sum = new Vector2[verts.Length];
        var cnt = new int[verts.Length];
        for (int i = 0; i < tris.Count; i++)
        {
            int t = tris[i], pos = x.TriPos[t];
            bool inP = x.TriChart[t] == pSlot;
            for (int k = 0; k < 3; k++)
            {
                var p = inP ? P.Uv[P.TriLocal[3 * pos + k]] : CMul(z, Q.Uv[Q.TriLocal[3 * pos + k]]) + tr;
                sum[tl[3 * i + k]] += p; cnt[tl[3 * i + k]]++;
            }
        }
        var warm = new Vector2[verts.Length];
        for (int i = 0; i < warm.Length; i++) warm[i] = sum[i] / Math.Max(1, cnt[i]);

        var sw = System.Diagnostics.Stopwatch.StartNew();
        int nQ = tris.Count - nP;
        if (keepP && nQ == 0)
        {
            // only lost triangles: what is left of a valid layout is valid
            why = P.Method;
            var kept = new ChartFlatten.Result { Uv = warm, Quality = ChartFlatten.Measure(x.G, tris, tl, warm) };
            if (ChartFlatten.Overlaps(tl, bnd, warm)) { why = "overlap"; return null; }
            return kept;
        }
        bool smallIsP = nP < nQ;
        Sub sub = keepP ? LocalRegion(tris, tl, verts.Length, nP, tris.Count, warm, 0.85f)
            : Math.Min(nP, nQ) < LocalRatio * Math.Max(nP, nQ)
            ? LocalRegion(tris, tl, verts.Length, smallIsP ? 0 : nP, smallIsP ? nP : tris.Count, warm) : null;
        var r = sub != null ? sub.Lscm(x.G, warm) : ChartFlatten.Lscm(x.G, tris, tl, warm);
        st.MsLscm += sw.ElapsedMilliseconds; st.ItLscm += r.Iterations;
        if (sub != null) { st.NLocal++; st.MsLocal += sw.ElapsedMilliseconds; } else { st.NGlobal++; st.MsGlobal += sw.ElapsedMilliseconds; st.VertsGlobal += verts.Length; }
        why = sub != null ? "lscm (local)" : "lscm";
        if (Debug && keepP) why += $" [P {nP}, Q {nQ}, free tris {(sub != null ? sub.Tris.Count : tris.Count)}, it {r.Iterations}, flips {FlipCount(x.G, tris, tl, r.Uv)}]";
        // local solves also get a local ARAP pass (cheap), so a big chart doesn't accumulate area distortion at
        // every place something was attached; global ones only when LSCM alone is too distorted
        if (sub != null || !Acceptable(r.Quality))
        {
            if (r.Quality.Flipped || r.Quality.AreaStretch > RescueStretch || r.Quality.AnisoP95 > 2 * MaxAniso)
            { st.RejectedDistortion++; why = why + " " + r.Quality; return null; }
            sw.Restart();
            var ra = sub != null ? sub.Arap(x.G, r.Uv) : ChartFlatten.Arap(x.G, tris, tl, r.Uv);
            st.MsArap += sw.ElapsedMilliseconds; st.ItArap += ra.Iterations;
            bool useArap = Acceptable(ra.Quality) && (!Acceptable(r.Quality) || Score(ra.Quality) < Score(r.Quality));
            bool arapOverlaps = useArap && ChartFlatten.Overlaps(tl, bnd, ra.Uv);
            if (arapOverlaps) useArap = false;
            if (useArap) { if (!Acceptable(r.Quality)) st.Rescued++; r = ra; why = sub != null ? "arap (local)" : "arap"; }
            else if (!Acceptable(r.Quality))
            {
                if (arapOverlaps) { st.RejectedOverlap++; why = "overlap after arap " + ra.Quality; }
                else { st.RejectedDistortion++; why = "arap " + ra.Quality; }
                return null;
            }
        }
        sw.Restart();
        bool ov = ChartFlatten.Overlaps(tl, bnd, r.Uv);
        st.MsOverlap += sw.ElapsedMilliseconds;
        if (ov) { st.RejectedOverlap++; why = "overlap after " + why + " " + r.Quality; return null; }
        return r;
    }

    /// <summary>A sub-problem: the triangles touching free vertices, with everything else held in place.</summary>
    sealed class Sub
    {
        public List<int> AllTris; public int[] AllTl;
        public List<int> Tris; public int[] Tl; public int[] ToFull; public bool[] Fixed;

        Vector2[] Extract(Vector2[] full)
        {
            var w = new Vector2[ToFull.Length];
            for (int i = 0; i < w.Length; i++) w[i] = full[ToFull[i]];
            return w;
        }

        ChartFlatten.Result Finish(TriGeom g, Vector2[] full, ChartFlatten.Result r)
        {
            var uv = (Vector2[])full.Clone();
            for (int i = 0; i < ToFull.Length; i++) uv[ToFull[i]] = r.Uv[i];
            return new ChartFlatten.Result { Uv = uv, Quality = ChartFlatten.Measure(g, AllTris, AllTl, uv), Iterations = r.Iterations };
        }

        public ChartFlatten.Result Lscm(TriGeom g, Vector2[] full) => Finish(g, full, ChartFlatten.Lscm(g, Tris, Tl, Extract(full), Fixed, measure: false));
        public ChartFlatten.Result Arap(TriGeom g, Vector2[] full) => Finish(g, full, ChartFlatten.Arap(g, Tris, Tl, Extract(full), Fixed, measure: false));
    }

    /// <summary>Free vertices: those of triangles [s0, s1) plus LocalRings rings around them; null if nearly all are free.</summary>
    static Sub LocalRegion(List<int> tris, int[] tl, int nv, int s0, int s1, Vector2[] warm, float maxFree = 0.6f)
    {
        var free = new bool[nv];
        for (int i = s0; i < s1; i++) for (int k = 0; k < 3; k++) free[tl[3 * i + k]] = true;
        for (int ring = 0; ring < LocalRings; ring++)
        {
            var next = (bool[])free.Clone();
            for (int i = 0; i < tris.Count; i++)
                if (free[tl[3 * i]] || free[tl[3 * i + 1]] || free[tl[3 * i + 2]])
                    for (int k = 0; k < 3; k++) next[tl[3 * i + k]] = true;
            free = next;
        }
        int nFree = 0;
        foreach (bool f in free) if (f) nFree++;
        if (nFree > maxFree * nv) return null;
        var sub = new Sub { AllTris = tris, AllTl = tl, Tris = new List<int>() };
        var subTl = new List<int>();
        var map = new Dictionary<int, int>();
        var toFull = new List<int>();
        for (int i = 0; i < tris.Count; i++)
        {
            if (!(free[tl[3 * i]] || free[tl[3 * i + 1]] || free[tl[3 * i + 2]])) continue;
            sub.Tris.Add(tris[i]);
            for (int k = 0; k < 3; k++)
            {
                int l = tl[3 * i + k];
                if (!map.TryGetValue(l, out int j)) { j = toFull.Count; map[l] = j; toFull.Add(l); }
                subTl.Add(j);
            }
        }
        sub.Tl = subTl.ToArray();
        sub.ToFull = toFull.ToArray();
        sub.Fixed = new bool[toFull.Count];
        int nFixed = 0;
        for (int j = 0; j < toFull.Count; j++) if (!free[toFull[j]]) { sub.Fixed[j] = true; nFixed++; }
        return nFixed >= 2 ? sub : null;
    }

    static int FlipCount(TriGeom g, List<int> tris, int[] tl, Vector2[] uv)
    {
        int n = 0;
        for (int i = 0; i < tris.Count; i++)
        {
            if (g.Area[tris[i]] <= 0) continue;
            if (g.Sv(tris[i], uv[tl[3 * i]], uv[tl[3 * i + 1]], uv[tl[3 * i + 2]]).det <= 0) n++;
        }
        return n;
    }

    static void Dbg(FlatChart A, FlatChart B, float prio, string why)
    {
        if (Debug) GD.Print($"  merge {A.Tris.Count}+{B.Tris.Count} tris (seam cost {prio:F3}) rejected: {why}");
    }

    static Vector2 CMul(Vector2 a, Vector2 b) => new(a.X * b.X - a.Y * b.Y, a.X * b.Y + a.Y * b.X);

    /// <summary>Complex least squares z·b + t ≈ a over corners on the A–B boundary (rotation and translation only).</summary>
    static (Vector2 z, Vector2 t) Similarity(Ctx x, FlatChart A, FlatChart B)
    {
        var m = x.M;
        var pa = new List<Vector2>(); var pb = new List<Vector2>();
        int aSlot = x.TriChart[A.Tris[0]];
        for (int i = 0; i < B.Tris.Count; i++)
        {
            int t = B.Tris[i];
            for (int k = 0; k < 3; k++)
            {
                int o = x.Nbr[3 * t + k];
                if (o < 0 || x.TriChart[o] != aSlot) continue;
                int ia = x.TriPos[o];
                for (int e = 0; e < 2; e++)
                {
                    int kb = (k + e) % 3, v = m.Tris[3 * t + kb];
                    for (int kk = 0; kk < 3; kk++)
                        if (m.Tris[3 * o + kk] == v) { pa.Add(A.Uv[A.TriLocal[3 * ia + kk]]); pb.Add(B.Uv[B.TriLocal[3 * i + kb]]); }
                }
            }
        }
        if (pa.Count < 2) return (new Vector2(1, 0), Vector2.Zero);
        Vector2 ca = Vector2.Zero, cb = Vector2.Zero;
        for (int i = 0; i < pa.Count; i++) { ca += pa[i]; cb += pb[i]; }
        ca /= pa.Count; cb /= pb.Count;
        double re = 0, im = 0, nn = 0;
        for (int i = 0; i < pa.Count; i++)
        {
            var a = pa[i] - ca; var b = pb[i] - cb;
            re += b.X * a.X + b.Y * a.Y;   // conj(b)·a
            im += b.X * a.Y - b.Y * a.X;
            nn += b.LengthSquared();
        }
        var z = new Vector2(1, 0);
        if (nn > 1e-30)
        {
            z = new Vector2((float)(re / nn), (float)(im / nn));
            float len = z.Length();
            z = len > 1e-12f ? z / len : new Vector2(1, 0);   // charts are at the same scale; keep only the rotation
        }
        return (z, ca - CMul(z, cb));
    }
}
