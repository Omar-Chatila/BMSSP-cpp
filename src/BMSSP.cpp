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

std::pair<double, VertexSet> BMSSP::bmssp(int l, double B, VertexSet &S) const {
    if (l == 0) {
        return base_case(S[0], B);
    }
    std::unordered_map<const Vertex*, double> dist;
    for (const auto& [vtx, v_dist] : S) {
        dist[vtx] = v_dist;
    }
    auto [P, W] = find_pivots(S, B);
    const size_t M = static_cast<size_t>(std::pow(2, (l - 1) * t_));
    DequeueBlocks D(M, B);
    double B0_prime = std::numeric_limits<double>::infinity();
    for (const auto& [vtx, dist_v] : P) {
        D.insert(vtx, dist_v);
        B0_prime = std::min(B0_prime, dist_v);
    }
    size_t i = 0;
    VertexSet U;
    double B_prime = B;
    while (U.size() < k_ * std::pow(2, l * t_) and !D.empty()) {
        ++i;
        auto [Si, Bi] = D.pull();
        auto [Bi_prime, Ui] = bmssp(l - 1, Bi, Si);
        B_prime = std::min(B_prime, Bi_prime);
        U.insert(U.end(), Ui.begin(), Ui.end());
        VertexSet K;
        for (auto [u, du] : Ui) {
            for (auto [v_id, w_uv] : u->outgoing_edges_) {
                const Vertex *v = graph_.get_vertex(v_id);
                const double cand = du + w_uv;
                if (!dist.contains(v) or cand < dist[v]) {
                    dist[v] = cand;
                    if (cand >= Bi and cand < B) {
                        D.insert(v, cand);
                    } else if (cand >= Bi_prime and cand < Bi) {
                        K.push_back({v, cand});
                    }
                }
            }
            for (auto [x, dx] : Si) {
                if (dx >= Bi_prime and dx < Bi) {
                    K.push_back({x, dx});
                }
            }
        }
        auto batch = std::list<Pair>(K.begin(), K.end());
        for (auto& [x, dx] : Si) {
            if (dx >= Bi_prime && dx < Bi) {
                batch.push_back({x, dx});
            }
        }
        D.batch_prepend(batch, Bi);
    }
    VertexSet result = U;
    for (auto& [vtx, dv] : W) {
        if (dist.contains(vtx) && dist[vtx] < B_prime) {
            result.push_back({vtx, dist[vtx]});
        }
    }
    return {B_prime, result};
}

std::unordered_map<const Vertex *, double> BMSSP::run() const {
    const int l = std::ceil(std::log(graph_.get_vertices().size()) / t_);
    VertexSet S;
    S.push_back({source_, 0.0});
    constexpr double B = std::numeric_limits<double>::infinity();
    auto [_, v_set] = bmssp(l, B, S);
 
    std::unordered_map<const Vertex*, double> result;
    for (const auto& [vtx, v_dist] : v_set) {
        result[vtx] = v_dist;
    }
    return result;
}
