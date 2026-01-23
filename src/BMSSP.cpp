#include "BMSSP.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <ranges>

static constexpr double INF = std::numeric_limits<double>::infinity();

static bool output = false;

BMSSP::BMSSP(Graph &graph, const Vertex *src) : graph_(graph), source_(src) {
    const size_t n = graph.get_vertices().size();
    k_ = static_cast<size_t>(std::pow(std::log2(n), 1.0/3.0));
    t_ = static_cast<size_t>(std::pow(std::log2(n), 2.0/3.0));
    if (!output) {
        std::cout << "k: " << k_ << " t: " << t_ << std::endl;
        output = true;
    }
    n_ = n;
    pivot_dist_cache_.reserve(n_);
    pivot_root_cache_.reserve(n_);
    pivot_tree_sz_cache_.reserve(n_);
    base_dist_cache_.reserve(n_);
    execution_failed_ = false;
}

BMSSP::BMSSP(Graph &graph, const Vertex *src, const size_t k, const size_t t) : graph_(graph), source_(src), n_(graph.size()), k_(k), t_(t) {
    pivot_dist_cache_.reserve(n_);
    pivot_root_cache_.reserve(n_);
    pivot_tree_sz_cache_.reserve(n_);
    base_dist_cache_.reserve(n_);
    execution_failed_ = false;
}

std::pair<VertexSet, VertexSet> BMSSP::find_pivots(const VertexSet &S, const double B) const {
    // with the help of https://github.com/lcs147/bmssp/blob/master/bmssp.hpp
    pivot_dist_cache_.assign(n_, INF);
    for (const auto& [vtx, v_dist] : S) {
        pivot_dist_cache_[vtx->id_] = v_dist;
    }

    VertexSet W = S;
    VertexSet W_prev = S;

    pivot_root_cache_.assign(n_, 0);
    pivot_tree_sz_cache_.assign(n_, 0);

    for (auto& [u, du] : S) {
        pivot_root_cache_[u->id_] = u->id_;
    }

    for (size_t i = 1; i <= k_; ++i) {
        VertexSet Wi;
        for (auto [u, d_u] : W_prev) {
            for (auto& [v_id, w_uv] : u->outgoing_edges_) {
                const Vertex *v = graph_.get_vertex(v_id);
                const double cand = d_u + w_uv;
                if (cand < B and cand < pivot_dist_cache_[v->id_]) {
                    pivot_dist_cache_[v->id_] = cand;
                    pivot_root_cache_[v->id_] = pivot_root_cache_[u->id_];
                    Wi.emplace_back(v, cand);
                }
            }
        }
        if (Wi.empty()) break;

        W.insert(W.end(), Wi.begin(), Wi.end());
        W_prev = std::move(Wi);

        if (W.size() > k_ * S.size()) {
            return {S, std::move(W)};
        }
    }

    for (auto& [vtx, _] : W) {
        pivot_tree_sz_cache_[pivot_root_cache_[vtx->id_]]++;
    }

    VertexSet P;
    for (auto& [u, du] : S) {
        if (pivot_tree_sz_cache_[u->id_] >= k_) {
            P.emplace_back(u, du);
        }
    }
    return {std::move(P), std::move(W)};
}

std::pair<double, VertexSet> BMSSP::base_case(const Pair& S, const double B, const int l) const {
    base_dist_cache_.assign(n_, INF);
    auto& [v_ptr, v_dist] = S;
    base_dist_cache_[v_ptr->id_] = v_dist;

    std::priority_queue<Pair, VertexSet, std::function<bool(const Pair&, const Pair&)>> H(
        [](const Pair& a, const Pair& b) {
            return b < a;
        });
    H.emplace(v_ptr, v_dist);

    VertexSet U;
    U.reserve(static_cast<size_t>(static_cast<double>(k_) * std::pow(2, l * t_)));

    while (!H.empty()) {
        auto [u, d_u] = H.top();
        H.pop();

        if (d_u != base_dist_cache_[u->id_]) continue;
        if (d_u >= B) break;

        U.emplace_back(u, d_u);

        for (auto& [v_id, w_uv] : u->outgoing_edges_) {
            const Vertex *v = graph_.get_vertex(v_id);
            const double cand = d_u + w_uv;
            if (cand < B && cand < base_dist_cache_[v->id_]) {
                base_dist_cache_[v->id_] = cand;
                H.emplace(v, cand);
            }
        }
    }
    return {B, U};
}

static void printVertexSet(const VertexSet& V) {
    for (auto& [vtx, v_B] : V) {
        std::cout << "(" << vtx->id_ << ": d[v]=" << v_B << "), ";
    }
    std::cout << std::endl;
}

std::pair<double, VertexSet> BMSSP::bmssp(const int l, const double B, const VertexSet &S) {
    std::cout << "Recursion l: " << l << " B: " << B << std::endl;
    printVertexSet(S);
    if (l == 0) {
        std::cout << "l=" << l << ": Enter Base Case\n";
        if (S.empty()) {
            execution_failed_ = true;
            return {};
        }
        return base_case(S[0], B, l);
    }
    std::vector dist(n_, INF);
    for (const auto& [vtx, v_dist] : S) {
        dist[vtx->id_] = v_dist;
    }
    std::cout << "l=" << l << ": Enter find pivots\n";

    auto [P, W] = find_pivots(S, B);

    printVertexSet(P);
    std::cout  << "l=" << l << ": Complete vertices in W:\n";
    printVertexSet(W);

    const auto M = static_cast<size_t>(std::pow(2, (l - 1) * t_));

    std::cout << "l=" << l << ": Initialize DequeBlocks with M=" << M << ", B=" << B << std::endl;
    DequeueBlocks D(n_, M, B);
    double B0_prime = INF;
    for (const auto& [vtx, dist_v] : P) {
        D.insert(vtx, dist_v);
        B0_prime = std::min(B0_prime, dist_v);
    }
    size_t i = 0;
    VertexSet U;
    const size_t U_cap = k_ * static_cast<size_t>(std::pow(2, l * t_));
    U.reserve(U_cap);
    double B_prime = B;
    while (U.size() < U_cap and !D.empty()) {
        ++i;
        auto [Si, Bi] = D.pull();
        std::cout << "l=" << l << ": While loop i=" << i << std::endl;
        std::cout << "l=" << l << ": Call BMSSP with Bi " << Bi << " and Si= " << std::endl;
        printVertexSet(Si);

        auto [Bi_prime, Ui] = bmssp(l - 1, Bi, Si);
        std::cout << "l=" << l << ": BMSSP result: Bi = " << Bi << "Si= "<< std::endl;
        printVertexSet(Si);
        if (execution_failed_)
            break;
        B_prime = std::min(B_prime, Bi_prime);
        U.insert(U.end(), Ui.begin(), Ui.end());
        std::cout << "l=" << l << ": Bi' = " << Bi_prime << " U=" <<std::endl;
        printVertexSet(U);
        VertexSet K;
        for (const auto& [u, du] : Ui) {
            for (const auto& [v_id, w_uv] : u->outgoing_edges_) {
                const Vertex *v = graph_.get_vertex(v_id);
                const double cand = du + w_uv;
                if (cand < dist[v->id_]) {
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
        std::cout << "l=" << l << ": Batch K = " << std::endl;
        printVertexSet(K);

        auto batch = std::list(K.begin(), K.end());
        for (auto& [x, dx] : Si) {
            if (dx >= Bi_prime && dx < Bi) {
                batch.emplace_back(x, dx);
            }
        }
        std::cout << "l=" << l << ": Batch K after adding vertices from Si " << std::endl;
        printVertexSet(K);
        std::cout << "batch prepend with K and Bi=" << Bi << std::endl;
        D.batch_prepend(batch, Bi);
    }
    if (execution_failed_) {
        std::cerr << "Execution failed" << std::endl;
        return {};
    }

    std::cout << "l = " << l << "BMSSP result after adding vertices from W to U:\n";
    VertexSet& result = U;
    for (auto& [vtx, dv] : W) {
        if (dist[vtx->id_] < B_prime) {
            result.emplace_back(vtx, dist[vtx->id_]);
        }
    }
    return {B_prime, std::move(result)};
}

std::vector<double> BMSSP::run() {
    std::cout << "BASE CALL: " << std::endl;
    const int l = std::ceil(std::log(graph_.get_vertices().size()) / static_cast<double>(t_));
    VertexSet S;
    S.emplace_back(source_, 0.0);
    constexpr double B = INF;
    std::cout << "    S = {(v0, 0.0)}, B = INF, l = " << l << std::endl;
    auto [_, v_set] = bmssp(l, B, S);
    if (execution_failed_) return {};
 
    std::vector<double> result(n_);
    for (const auto& [vtx, v_dist] : v_set) {
        result[vtx->id_] = v_dist;
    }
    return result;
}

bool BMSSP::has_exec_failed() const {
    return execution_failed_;
}
