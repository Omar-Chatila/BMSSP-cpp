#include "BMSSP.h"

#include <cmath>
#include <functional>
#include <queue>
#include <ranges>

BMSSP::BMSSP(Graph &graph, Vertex *src) : graph_(graph), source_(src) {
    const size_t n = graph.get_vertices().size();
    k_ = static_cast<size_t>(std::pow(std::log(n), 1.0/3.0));
    t_ = static_cast<size_t>(std::pow(std::log(n), 2.0/3.0));
}

size_t dfs(const Vertex* u, std::unordered_map<const Vertex*, std::vector<const Vertex*>>& children,
    std::unordered_map<const Vertex*, int>& subtree_size) {
    size_t size = 1;
    for (const auto v : children[u])
        size += dfs(v, children, subtree_size);
    subtree_size[u] = size;
    return size;
}

std::pair<VertexSet, VertexSet> BMSSP::find_pivots(const VertexSet &S, double B) const {
    std::unordered_map<const Vertex*, double> dist;
    for (auto& [v_ptr, v_dist] : S)
        dist[v_ptr] = v_dist;

    VertexSet W = S;
    VertexSet W_prev = S;
    for (size_t i = 1; i <= k_; ++i) {
        VertexSet Wi;
        for (auto [u, d_u] : W_prev) {
            for (auto& [v_id, w_uv] : u->outgoing_edges_) {
                const Vertex *v = graph_.get_vertex(v_id);
                const double cand = d_u + w_uv;
                if (cand < B and (!dist.contains(v) or cand < dist[v])) {
                    dist[v] = cand;
                    Wi.push_back({v, cand});
                }
            }
        }
        if (Wi.empty()) break;

        W.insert(W.end(), Wi.begin(), Wi.end());
        W_prev = std::move(Wi);

        if (W.size() > k_ * S.size()) {
            return {S, W};
        }
    }

    // construct Forest F
    std::unordered_map<const Vertex*, std::vector<const Vertex*>> children;
    std::unordered_map<const Vertex*, const Vertex*> parent;

    for (auto& [u, du] : W) {
        for (auto& [v_id, w] : u->outgoing_edges_) {
            const Vertex* v = graph_.get_vertex(v_id);
            if (!dist.contains(v)) continue;

            if (dist[v] == du + w) {
                parent[v] = u;
                children[u].push_back(v);
            }
        }
    }
    std::unordered_map<const Vertex*, int> subtree_size;
    for (auto& [u, _] : S) {
        if (!parent.contains(u)) {
            dfs(u, children, subtree_size);
        }
    }

    VertexSet P;
    for (auto& [u, du] : S) {
        if (subtree_size[u] >= k_) {
            P.push_back({u, du});
        }
    }
    return {P, W};
}

std::pair<double, VertexSet> BMSSP::base_case(Pair& S, const double B) const {
    std::unordered_map<const Vertex*, double> dist;
    auto& [v_ptr, v_dist] = S;
    dist[v_ptr] = v_dist;

    std::priority_queue<Pair, VertexSet, std::function<bool(const Pair&, const Pair&)>> H(
        [](const Pair& a, const Pair& b) {
                return b < a;
            });
    H.push({v_ptr, v_dist});

    VertexSet U_0;
    double B_prime = B;

    while (!H.empty()) {
        auto [u, d_u] = H.top();
        H.pop();

        if (d_u != dist[u]) continue;
        if (d_u >= B) break;

        U_0.push_back({u, d_u});
        if (U_0.size() == k_ + 1) {
            B_prime = d_u;
            break;
        }

        for (auto& [v_id, w_uv] : u->outgoing_edges_) {
            const Vertex *v = graph_.get_vertex(v_id);
            const double cand = d_u + w_uv;
            if (cand < B and (!dist.contains(v) || cand < dist[v])) {
                dist[v] = cand;
                H.push({v, cand});
            }
        }
    }

    const size_t limit = std::min(k_, U_0.size());
    VertexSet U;
    for (size_t i = 0; i < limit; ++i) {
        U.push_back(U_0[i]);
    }
    return {B_prime, U};
}
