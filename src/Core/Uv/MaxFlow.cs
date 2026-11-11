using System;
using System.Collections.Generic;

namespace ModelMate.Core.Uv;

/// <summary>Dinic's max-flow on an undirected-capacity graph; afterwards <see cref="SourceSide"/> gives a minimum cut.</summary>
public sealed class MaxFlow
{
    readonly int _n;
    readonly List<int> _to = new(), _next = new();
    readonly List<double> _cap = new();
    readonly int[] _head, _level, _it;

    public MaxFlow(int n)
    {
        _n = n;
        _head = new int[n];
        Array.Fill(_head, -1);
        _level = new int[n];
        _it = new int[n];
    }

    /// <summary>Edge u→v with capacity c, and v→u with capacity rc.</summary>
    public void Add(int u, int v, double c, double rc)
    {
        _to.Add(v); _cap.Add(c); _next.Add(_head[u]); _head[u] = _to.Count - 1;
        _to.Add(u); _cap.Add(rc); _next.Add(_head[v]); _head[v] = _to.Count - 1;
    }

    bool Bfs(int s, int t)
    {
        Array.Fill(_level, -1);
        var q = new Queue<int>();
        _level[s] = 0; q.Enqueue(s);
        while (q.Count > 0)
        {
            int u = q.Dequeue();
            for (int e = _head[u]; e >= 0; e = _next[e])
                if (_cap[e] > 1e-12 && _level[_to[e]] < 0) { _level[_to[e]] = _level[u] + 1; q.Enqueue(_to[e]); }
        }
        return _level[t] >= 0;
    }

    double Dfs(int u, int t, double f)
    {
        if (u == t) return f;
        for (; _it[u] >= 0; _it[u] = _next[_it[u]])
        {
            int e = _it[u], v = _to[e];
            if (_cap[e] <= 1e-12 || _level[v] != _level[u] + 1) continue;
            double d = Dfs(v, t, Math.Min(f, _cap[e]));
            if (d > 0) { _cap[e] -= d; _cap[e ^ 1] += d; return d; }
        }
        return 0;
    }

    public double Run(int s, int t)
    {
        double flow = 0;
        while (Bfs(s, t))
        {
            Array.Copy(_head, _it, _n);
            double f;
            while ((f = Dfs(s, t, double.MaxValue)) > 0) flow += f;
        }
        return flow;
    }

    /// <summary>Nodes reachable from s in the residual graph (call after Run).</summary>
    public bool[] SourceSide(int s)
    {
        var seen = new bool[_n];
        var st = new Stack<int>();
        seen[s] = true; st.Push(s);
        while (st.Count > 0)
        {
            int u = st.Pop();
            for (int e = _head[u]; e >= 0; e = _next[e])
                if (_cap[e] > 1e-12 && !seen[_to[e]]) { seen[_to[e]] = true; st.Push(_to[e]); }
        }
        return seen;
    }
}
