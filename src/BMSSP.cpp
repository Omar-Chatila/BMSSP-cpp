#include "BMSSP.h"

#include <cmath>
#include <functional>
#include <queue>
#include <ranges>
#include <stack>

static constexpr double INF = std::numeric_limits<double>::infinity();

BMSSP::BMSSP(Graph &graph, const Vertex *src) : graph_(graph), source_(src) {
    const size_t n = graph.get_vertices().size();
    k_ = static_cast<size_t>(std::pow(std::log(n), 1.0/3.0));
    t_ = static_cast<size_t>(std::pow(std::log(n), 2.0/3.0));
    n_ = n;
}

BMSSP::BMSSP(Graph &graph, const Vertex *src, const size_t k, const size_t t) : graph_(graph), source_(src), n_(graph.size()), k_(k), t_(t) {
}

inline void dfs(const Vertex* root,
            const std::vector<std::vector<uint64_t>>& children,
            std::vector<size_t>& subtree_size) {
    std::vector<uint64_t> order;
    order.reserve(children.size());

    std::stack<uint64_t> stack;
    stack.push(root->id_);

    while (!stack.empty()) {
        const uint64_t u = stack.top();
        stack.pop();
        order.push_back(u);
        for (const auto v : children[u]) {
            stack.push(v);
        }
    }

    for (const auto u : std::ranges::reverse_view(order)) {
        size_t size = 1;
        for (const uint64_t child : children[u]) {
            size += subtree_size[child];
        }
        subtree_size[u] = size;
    }
}

std::pair<VertexSet, VertexSet> BMSSP::find_pivots(const VertexSet &S, double B) const {
    std::vector dist(n_, INF);
    std::vector exists(n_, false);
    for (const auto& [vtx, v_dist] : S) {
        dist[vtx->id_] = v_dist;
        exists[vtx->id_] = true;
    }

    VertexSet W = S;
    VertexSet W_prev = S;
    for (size_t i = 1; i <= k_; ++i) {
        VertexSet Wi;
        for (auto [u, d_u] : W_prev) {
            for (auto& [v_id, w_uv] : u->outgoing_edges_) {
                const Vertex *v = graph_.get_vertex(v_id);
                const double cand = d_u + w_uv;
                if (cand < B and (!exists[v->id_] or cand < dist[v->id_])) {
                    dist[v->id_] = cand;
                    Wi.emplace_back(v, cand);
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
    //std::unordered_map<const Vertex*, std::vector<const Vertex*>> children;
    std::vector<std::vector<uint64_t>> children;
    //std::unordered_map<const Vertex*, const Vertex*> parent;
    std::vector<uint64_t> parent;
    std::vector<bool> parent_exists;

    for (auto& [u, du] : W) {
        for (auto& [v_id, w] : u->outgoing_edges_) {
            const Vertex* v = graph_.get_vertex(v_id);
            if (!exists[v->id_]) continue;

            if (dist[v->id_] == du + w) {
                parent[v->id_] = u->id_;
                parent_exists[v_id] = true;
                children[u->id_].push_back(v->id_);
            }
        }
    }
    //std::unordered_map<const Vertex*, size_t> subtree_size;
    std::vector<size_t> subtree_size;
    for (auto& [u, _] : S) {
        if (!parent_exists[u->id_]) {
            dfs(u, children,  subtree_size);
        }
    }

    VertexSet P;
    for (auto& [u, du] : S) {
        if (subtree_size[u->id_] >= k_) {
            P.emplace_back(u, du);
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
    H.emplace(v_ptr, v_dist);

    VertexSet U;

    while (!H.empty()) {
        auto [u, d_u] = H.top();
        H.pop();

        if (d_u != dist[u]) continue;
        if (d_u >= B) break;

        U.emplace_back(u, d_u);

        for (auto& [v_id, w_uv] : u->outgoing_edges_) {
            const Vertex *v = graph_.get_vertex(v_id);
            const double cand = d_u + w_uv;
            if (cand < B && (!dist.contains(v) || cand < dist[v])) {
                dist[v] = cand;
                H.emplace(v, cand);
            }
        }
    }
    return {B, U};
}

std::pair<double, VertexSet> BMSSP::bmssp(int l, double B, VertexSet &S) const {
    if (l == 0) {
        return base_case(S[0], B);
    }
    std::vector dist(n_, INF);
    std::vector exists(n_, false);
    for (const auto& [vtx, v_dist] : S) {
        dist[vtx->id_] = v_dist;
        exists[vtx->id_] = true;
    }
    auto [P, W] = find_pivots(S, B);
    const auto M = static_cast<size_t>(std::pow(2, (l - 1) * t_));
    DequeueBlocks D(n_, M, B);
    double B0_prime = std::numeric_limits<double>::infinity();
    for (const auto& [vtx, dist_v] : P) {
        D.insert(vtx, dist_v);
        B0_prime = std::min(B0_prime, dist_v);
    }
    size_t i = 0;
    VertexSet U;
    double B_prime = B;
    while (U.size() < static_cast<size_t>(static_cast<double>(k_) * std::pow(2, l * t_)) and !D.empty()) {
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
                if (!exists[v->id_] or cand < dist[v->id_]) {
                    dist[v->id_] = cand;
                    if (cand >= Bi and cand < B) {
                        D.insert(v, cand);
                    } else if (cand >= Bi_prime and cand < Bi) {
                        K.emplace_back(v, cand);
                    }
                }
            }
            for (auto [x, dx] : Si) {
                if (dx >= Bi_prime and dx < Bi) {
                    K.emplace_back(x, dx);
                }
            }
        }
        auto batch = std::list<Pair>(K.begin(), K.end());
        for (auto& [x, dx] : Si) {
            if (dx >= Bi_prime && dx < Bi) {
                batch.emplace_back(x, dx);
            }
        }
        D.batch_prepend(batch, Bi);
    }
    VertexSet result = U;
    for (auto& [vtx, dv] : W) {
        if (exists[vtx->id_] && dist[vtx->id_] < B_prime) {
            result.emplace_back(vtx, dist[vtx->id_]);
        }
    }
    return {B_prime, result};
}

std::vector<double> BMSSP::run() const {
    const int l = std::ceil(std::log(graph_.get_vertices().size()) / static_cast<double>(t_));
    VertexSet S;
    S.emplace_back(source_, 0.0);
    constexpr double B = std::numeric_limits<double>::infinity();
    auto [_, v_set] = bmssp(l, B, S);
 
    std::vector<double> result(n_);
    for (const auto& [vtx, v_dist] : v_set) {
        result[vtx->id_] = v_dist;
    }
    return result;
}
