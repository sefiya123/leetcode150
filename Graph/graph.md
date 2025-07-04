# 📌 Graph – DSA Cheat Sheet (C++ | Interview-Focused)

---

### 🔹 **Topic Name: Graphs**

---

### 🔹 **Key Concepts**

* A **graph** is a collection of nodes (vertices) and edges representing relationships.
* Graphs can be **directed/undirected**, **weighted/unweighted**, and **cyclic/acyclic**.
* Applications: networking, route maps, job scheduling, social connections, etc.

---

### 🔹 **Common Patterns**

* 🔁 **BFS / DFS Traversals**
* 🔄 **Cycle Detection (DFS/Union-Find)**
* 📚 **Topological Sort** (Kahn’s Algo, DFS)
* 🛣 **Shortest Paths** (Dijkstra, Bellman-Ford, Floyd-Warshall)
* 🔗 **Union-Find** (Disjoint Set)
* 🌈 **Graph Coloring / Bipartite Check**
* 🧩 **Grid to Graph Mapping** (multi-source BFS)

---

### 🔹 **Must-Know Subtopics**

* Graph Representations (Adjacency List/Matrix)
* DFS & BFS (Recursive & Iterative)
* Cycle Detection (DFS/Union-Find)
* Topological Sort (DFS/Kahn’s)
* Dijkstra’s Algorithm
* Bellman-Ford Algorithm
* Floyd-Warshall
* Minimum Spanning Tree (Prim’s/Kruskal’s)
* Union-Find (Path Compression + Rank)
* Bipartite Graph Check
* Tarjan’s Algorithm (SCC, Bridges)
* Kosaraju’s Algorithm
* Grid as Graph Problems (Islands, Fire Spread, Rotten Oranges)

---

### 🔹 **High-Yield Interview Problems**

| Problem                                                                 | Platform | Concept      |
| ----------------------------------------------------------------------- | -------- | ------------ |
| [Number of Islands](https://leetcode.com/problems/number-of-islands)    | Leetcode | DFS/BFS Grid |
| [Course Schedule](https://leetcode.com/problems/course-schedule)        | Leetcode | Topo Sort    |
| [Alien Dictionary](https://leetcode.com/problems/alien-dictionary/)     | Leetcode | Topo Sort    |
| [Clone Graph](https://leetcode.com/problems/clone-graph/)               | Leetcode | DFS/BFS      |
| [Network Delay Time](https://leetcode.com/problems/network-delay-time/) | Leetcode | Dijkstra     |

---

### 🔹 **C++ Implementation Tips**

* Use `unordered_map` or `vector<vector<int>>` for adjacency list
* Use `queue` for BFS, `stack` for DFS (iterative)
* Use `priority_queue<pair<int,int>, vector<>, greater<>>` for Dijkstra
* Mark visited using `vector<bool>` or `unordered_set`
* Use direction arrays for grid-based graph problems

---

### 🔹 **Time & Space Complexity Summary**

| Operation                     | Time Complexity | Space Complexity |
| ----------------------------- | --------------- | ---------------- |
| BFS/DFS                       | O(V + E)        | O(V)             |
| Dijkstra (min-heap)           | O(E log V)      | O(V + E)         |
| Bellman-Ford                  | O(V × E)        | O(V)             |
| Floyd-Warshall                | O(V³)           | O(V²)            |
| Kruskal's (MST)               | O(E log E)      | O(V)             |
| Union-Find (w/ optimizations) | O(α(N)) ≈ O(1)  | O(N)             |

---

### 🔹 **Code Snippets (C++)**

#### ✅ Graph Representation

```cpp
vector<vector<int>> graph(n);
for (auto& [u, v] : edges) {
    graph[u].push_back(v); // add v for directed, add both for undirected
}
```

#### ✅ DFS Traversal (Recursive)

```cpp
void dfs(int node, vector<bool>& visited, vector<vector<int>>& graph) {
    visited[node] = true;
    for (int nei : graph[node]) {
        if (!visited[nei]) dfs(nei, visited, graph);
    }
}
```

#### ✅ BFS Traversal

```cpp
#include <queue>

void bfs(int start, vector<vector<int>>& graph) {
    vector<bool> visited(graph.size(), false);
    queue<int> q;
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        int node = q.front(); q.pop();
        for (int nei : graph[node]) {
            if (!visited[nei]) {
                visited[nei] = true;
                q.push(nei);
            }
        }
    }
}
```

#### ✅ Cycle Detection (Undirected using DFS)

```cpp
bool hasCycle(int node, int parent, vector<bool>& visited, vector<vector<int>>& graph) {
    visited[node] = true;
    for (int nei : graph[node]) {
        if (!visited[nei]) {
            if (hasCycle(nei, node, visited, graph)) return true;
        } else if (nei != parent) {
            return true;
        }
    }
    return false;
}
```

#### ✅ Topological Sort (Kahn’s Algorithm - BFS)

```cpp
vector<int> topoSort(int V, vector<pair<int, int>>& edges) {
    vector<vector<int>> graph(V);
    vector<int> indegree(V, 0);
    for (auto [u, v] : edges) {
        graph[u].push_back(v);
        indegree[v]++;
    }

    queue<int> q;
    for (int i = 0; i < V; ++i)
        if (indegree[i] == 0) q.push(i);

    vector<int> topo;
    while (!q.empty()) {
        int node = q.front(); q.pop();
        topo.push_back(node);
        for (int nei : graph[node]) {
            if (--indegree[nei] == 0)
                q.push(nei);
        }
    }
    return topo;
}
```

#### ✅ Dijkstra’s Algorithm (Min-Heap)

```cpp
#include <queue>
#include <climits>

vector<int> dijkstra(int V, vector<vector<pair<int, int>>>& graph, int src) {
    vector<int> dist(V, INT_MAX);
    dist[src] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        for (auto [v, w] : graph[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}
```

#### ✅ Union-Find (Path Compression + Rank)

```cpp
class UnionFind {
    vector<int> parent, rank;

public:
    UnionFind(int n) {
        parent.resize(n);
        rank.assign(n, 0);
        iota(parent.begin(), parent.end(), 0);
    }

    int find(int x) {
        if (x != parent[x])
            parent[x] = find(parent[x]);
        return parent[x];
    }

    bool unite(int x, int y) {
        int xr = find(x), yr = find(y);
        if (xr == yr) return false;

        if (rank[xr] < rank[yr]) parent[xr] = yr;
        else if (rank[xr] > rank[yr]) parent[yr] = xr;
        else {
            parent[yr] = xr;
            rank[xr]++;
        }
        return true;
    }
};
```

#### ✅ Grid as Graph (Number of Islands)

```cpp
void dfs(int r, int c, vector<vector<char>>& grid, vector<vector<bool>>& visited) {
    int rows = grid.size(), cols = grid[0].size();
    if (r < 0 || r >= rows || c < 0 || c >= cols || grid[r][c] == '0' || visited[r][c])
        return;
    visited[r][c] = true;
    dfs(r + 1, c, grid, visited);
    dfs(r - 1, c, grid, visited);
    dfs(r, c + 1, grid, visited);
    dfs(r, c - 1, grid, visited);
}

int numIslands(vector<vector<char>>& grid) {
    int rows = grid.size(), cols = grid[0].size(), count = 0;
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (grid[r][c] == '1' && !visited[r][c]) {
                dfs(r, c, grid, visited);
                count++;
            }
        }
    }
    return count;
}
```


```cpp
// 📌 Graph – DSA Cheat Sheet (C++ | Interview-Focused)

// 🔹 Key Concepts
// - Graphs represent relationships via nodes and edges
// - Types: Directed/Undirected, Weighted/Unweighted, Cyclic/Acyclic
// - Real-World: Maps, Networks, Scheduling, Social Media

// 🔹 Common Patterns
// - BFS/DFS
// - Cycle Detection (DFS/Union-Find)
// - Topological Sort (DFS, Kahn’s)
// - Shortest Path (Dijkstra, Bellman-Ford, Floyd-Warshall)
// - Union-Find
// - Graph Coloring (Bipartite)
// - Grid-to-Graph Problems

// 🔹 Implementation Snippets

// Graph Representation (Adjacency List)
vector<vector<int>> graph(n);
for (auto& [u, v] : edges) graph[u].push_back(v);

// DFS
void dfs(int u, vector<bool>& vis, vector<vector<int>>& g) {
    vis[u] = true;
    for (int v : g[u]) if (!vis[v]) dfs(v, vis, g);
}

// BFS
void bfs(int start, vector<vector<int>>& g) {
    vector<bool> vis(g.size(), false);
    queue<int> q; q.push(start); vis[start] = true;
    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (int v : g[u]) if (!vis[v]) {
            vis[v] = true; q.push(v);
        }
    }
}

// Cycle Detection (Undirected)
bool hasCycle(int u, int parent, vector<vector<int>>& g, vector<bool>& vis) {
    vis[u] = true;
    for (int v : g[u]) {
        if (!vis[v]) {
            if (hasCycle(v, u, g, vis)) return true;
        } else if (v != parent) return true;
    }
    return false;
}

// Graph Coloring (Bipartite Check - BFS)
bool isBipartite(vector<vector<int>>& graph) {
    int n = graph.size();
    vector<int> color(n, -1);
    for (int i = 0; i < n; ++i) {
        if (color[i] == -1) {
            queue<int> q;
            q.push(i);
            color[i] = 0;
            while (!q.empty()) {
                int u = q.front(); q.pop();
                for (int v : graph[u]) {
                    if (color[v] == -1) {
                        color[v] = 1 - color[u];
                        q.push(v);
                    } else if (color[v] == color[u]) return false;
                }
            }
        }
    }
    return true;
}

// Dijkstra's Algorithm
vector<int> dijkstra(int n, vector<vector<pair<int, int>>>& graph, int src) {
    vector<int> dist(n, INT_MAX);
    dist[src] = 0;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        for (auto [v, w] : graph[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

// Bellman-Ford Algorithm
vector<int> bellmanFord(int n, vector<tuple<int, int, int>>& edges, int src) {
    vector<int> dist(n, INT_MAX);
    dist[src] = 0;
    for (int i = 1; i < n; ++i) {
        for (auto [u, v, w] : edges) {
            if (dist[u] != INT_MAX && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
            }
        }
    }
    return dist;
}

// Floyd-Warshall Algorithm
vector<vector<int>> floydWarshall(vector<vector<int>>& mat) {
    int n = mat.size();
    vector<vector<int>> dist = mat;
    for (int k = 0; k < n; ++k)
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (dist[i][k] < INT_MAX && dist[k][j] < INT_MAX)
                    dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j]);
    return dist;
}

// Prim’s MST
int primMST(int n, vector<vector<pair<int, int>>>& graph) {
    vector<bool> vis(n, false);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    pq.push({0, 0});
    int total = 0;
    while (!pq.empty()) {
        auto [w, u] = pq.top(); pq.pop();
        if (vis[u]) continue;
        vis[u] = true;
        total += w;
        for (auto [v, wt] : graph[u]) if (!vis[v]) pq.push({wt, v});
    }
    return total;
}

// Union-Find (Disjoint Set)
class UnionFind {
public:
    vector<int> parent, rank;
    UnionFind(int n) : parent(n), rank(n, 0) {
        iota(parent.begin(), parent.end(), 0);
    }
    int find(int x) {
        return parent[x] == x ? x : parent[x] = find(parent[x]);
    }
    bool unionSet(int x, int y) {
        int xr = find(x), yr = find(y);
        if (xr == yr) return false;
        if (rank[xr] < rank[yr]) parent[xr] = yr;
        else if (rank[xr] > rank[yr]) parent[yr] = xr;
        else parent[yr] = xr, rank[xr]++;
        return true;
    }
};

// Kruskal’s MST
int kruskalMST(int n, vector<tuple<int, int, int>>& edges) {
    sort(edges.begin(), edges.end(), [](auto& a, auto& b) {
        return get<2>(a) < get<2>(b);
    });
    UnionFind uf(n);
    int total = 0;
    for (auto& [u, v, w] : edges)
        if (uf.unionSet(u, v)) total += w;
    return total;
}

```
---

### ✅ Final Interview Readiness Checklist

* [ ] Can I traverse graphs using BFS & DFS on adjacency list/matrix?
* [ ] Can I detect cycles in directed/undirected graphs using DFS/Union-Find?
* [ ] Can I compute shortest paths using Dijkstra, Bellman-Ford, or Floyd-Warshall?
* [ ] Can I solve grid problems as graphs (BFS/DFS)?
* [ ] Can I implement Union-Find efficiently with path compression?
* [ ] Can I perform topological sort using Kahn’s and DFS?
