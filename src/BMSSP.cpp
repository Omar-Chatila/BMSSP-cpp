#include "BMSSP.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <ranges>

static constexpr double INF = std::numeric_limits<double>::infinity();


BMSSP::BMSSP(Graph &graph, const Vertex *src) : graph_(graph), source_(src) {
    n_ = graph.get_vertices().size();
    k_ = static_cast<size_t>(std::pow(std::log2(n_), 1.0/3.0));
    t_ = static_cast<size_t>(std::pow(std::log2(n_), 2.0/3.0));

    pivot_root_cache_.reserve(n_);
    pivot_tree_sz_cache_.reserve(n_);
    dist_cache_.assign(n_, INF);
    execution_failed_ = false;
}

BMSSP::BMSSP(Graph &graph, const Vertex *src, const size_t k, const size_t t) : graph_(graph), source_(src), n_(graph.size()), k_(k), t_(t) {
    pivot_root_cache_.reserve(n_);
    pivot_tree_sz_cache_.reserve(n_);
    dist_cache_.assign(n_, INF);
    execution_failed_ = false;
}

std::pair<VertexSet, VertexSet> BMSSP::find_pivots(const VertexSet &S, const double B) const {
    // with the help of https://github.com/lcs147/bmssp/blob/master/bmssp.hpp

    VertexSet W = S;
    VertexSet W_prev = S;

    pivot_root_cache_.assign(n_, 0);
    pivot_tree_sz_cache_.assign(n_, 0);

    for (const auto& [u, du] : S) {
        pivot_root_cache_[u->id_] = u->id_;
    }

    for (size_t i = 1; i <= k_; ++i) {
        VertexSet Wi;
        for (const auto& [u, d_u] : W_prev) {
            for (const auto& [v_id, w_uv] : u->outgoing_edges_) {
                const Vertex *v = graph_.get_vertex(v_id);
                const double cand = d_u + w_uv;
                if (cand < B and cand <= dist_cache_[v->id_]) {
                    dist_cache_[v->id_] = cand;
                    if (dist_cache_[v->id_] < B) {
                        pivot_root_cache_[v->id_] = pivot_root_cache_[u->id_];
                        Wi.emplace_back(v, cand);
                    }
                }
            }
        }
        if (Wi.empty()) break;

        W.insert(W.end(), Wi.begin(), Wi.end());
        W_prev = std::move(Wi);

        if (W.size() > k_ * S.size()) {
            std::cout << "Pivots: ";
            for (auto p : S) {
                std::cout << p.key_->id_ << ", ";
            }
            std::cout << std::endl;
            std::cout << "Visited: ";
            for (auto p : W) {
                std::cout << p.key_->id_ << ", ";
            }
            std::cout << std::endl;
            return {S, std::move(W)};
        }
    }

    for (const auto& [vtx, _] : W) {
        pivot_tree_sz_cache_[pivot_root_cache_[vtx->id_]]++;
    }

    VertexSet P;
    P.reserve(W.size() / k_);
    for (const auto& [u, du] : S) {
        if (pivot_tree_sz_cache_[u->id_] >= k_) {
            P.emplace_back(u, du);
        }
    }
    std::cout << "Pivots: ";
    for (auto p : S) {
        std::cout << p.key_->id_ << ", ";
    }
    std::cout << std::endl;
    std::cout << "Visited: ";
    for (auto p : W) {
        std::cout << p.key_->id_ << ", ";
    }
    std::cout << std::endl;
    return {P, W};
}

std::pair<double, VertexSet> BMSSP::base_case(const Pair &S, const double B) const {
    const auto& [v_ptr, v_dist] = S;

    std::priority_queue<Pair, VertexSet, std::function<bool(const Pair&, const Pair&)>> H(
        [](const Pair& a, const Pair& b) {
            return b < a;
        });
    H.emplace(v_ptr, v_dist);

    VertexSet U;
    U.reserve(k_ + 1);

    while (!H.empty() && U.size() < k_ + 1) {
        const auto [u, d_u] = H.top();
        H.pop();

        if (d_u > dist_cache_[u->id_]) continue;
        //if (d_u >= B) break;

        U.emplace_back(u, d_u);

        for (const auto& [v_id, w_uv] : u->outgoing_edges_) {
            const Vertex *v = graph_.get_vertex(v_id);
            const double cand = d_u + w_uv;
            if (cand < B && cand <= dist_cache_[v->id_]) {
                dist_cache_[v->id_] = cand;
                H.emplace(v, cand);
            }
        }
    }
    std::cout << "Base Case: ";
    if (U.size() <= k_) {
        std::cout << "B: " << B << " Vertices: ";
        for (auto p : U)
            std::cout << p.key_->id_ << ", ";
        std::cout << std::endl;
        return {B, U};
    }
    double B_new = dist_cache_[U.back().key_->id_];
    U.pop_back();
    std::cout << "B: " << B_new << " Vertices: ";
    for (auto p : U)
        std::cout << p.key_->id_ << ", ";
    std::cout << std::endl;
    return {B_new, U};
}

std::pair<double, VertexSet> BMSSP::bmssp(int l, double B, const VertexSet &S) {
    if (l == 0) {
        if (S.empty()) {
            execution_failed_ = true;
            return {};
        }
        return base_case(S[0], B);
    }

    auto [P, W] = find_pivots(S, B);
    const auto M = static_cast<size_t>(std::pow(2, (l - 1) * t_));
    DequeueBlocks D(n_, M, B);
    double B_prime = B;
    for (const auto& [vtx, dist_v] : P) {
        D.insert(vtx, dist_v);
        B_prime = std::min(B_prime, dist_v);
    }
    size_t i = 0;
    VertexSet U;
    const auto cap = static_cast<size_t>(std::pow(2, l * t_));
    U.reserve(W.size() + cap);

    while (U.size() < cap and !D.empty()) {
        ++i;
        auto [Si, Bi] = D.pull();

        std::cout << "Bi : " << Bi << std::endl;
        std::cout <<"Pulled Vertices: ";
        for (auto p : Si)
            std::cout << p.key_->id_ << ", ";
        std::cout << std::endl;

        auto [Bi_prime, Ui] = bmssp(l - 1, Bi, Si);

        std::cout << "Bi prime " << Bi_prime << " Complete v: ";
        for (auto p : Ui)
            std::cout << p.key_->id_ << ", ";
        std::cout << std::endl;

        if (execution_failed_)
            break;
        U.insert(U.end(), Ui.begin(), Ui.end());
        VertexSet K;
        for (const auto& [u, du] : Ui) {
            D.erase(u);
            for (const auto& [v_id, w_uv] : u->outgoing_edges_) {
                const Vertex *v = graph_.get_vertex(v_id);
                const double cand = du + w_uv;
                if (cand <= dist_cache_[v->id_]) {
                    dist_cache_[v->id_] = cand;
                    if (cand >= Bi and cand < B) {
                        D.insert(v, cand);
                    } else if (cand >= Bi_prime and cand < Bi) {
                        K.emplace_back(v, cand);
                    }
                }
            }
        }
        for (const auto& [x, dx] : Si) {
            if (dx >= Bi_prime and dx < Bi) {
                K.emplace_back(x, dx);
            }
        }
        D.batch_prepend(K, Bi);
        B_prime = Bi_prime;
    }
    if (execution_failed_) {
        std::cerr << "Execution failed" << std::endl;
        return {};
    }
    double resB;
    if (D.empty()) resB = B;
    else resB = B_prime;


    for (const auto& [vtx, dv] : W) {
        if (dist_cache_[vtx->id_] < resB) {
            U.emplace_back(vtx, dist_cache_[vtx->id_]);
        }
    }
    std::cout << "Returned B: " << resB << " Complete: " << std::endl;
    for (auto p : U)
        std::cout << p.key_->id_ << ", ";
    std::cout << std::endl;
    return {resB, U};
}

std::vector<double> BMSSP::run() {
    const int l = std::ceil(std::log2(n_) / static_cast<double>(t_));
    VertexSet S;
    S.emplace_back(source_, 0.0);
    constexpr double B = INF;
    dist_cache_[0] = 0;
    auto [_, v_set] = bmssp(l, B, S);
    if (execution_failed_) return {};
 
    std::vector<double> result(n_);
    for (int i = 0; i <  dist_cache_.size(); ++i) {
        result[i] = dist_cache_[i];
        std::cout << i << ": " << dist_cache_[i] << std::endl;
    }
    return result;
}

bool BMSSP::has_exec_failed() const {
    return execution_failed_;
}
