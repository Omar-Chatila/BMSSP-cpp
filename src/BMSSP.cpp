#include "BMSSP.h"

#include <cmath>
#include <functional>
#include <queue>

BMSSP::BMSSP(Graph &graph, Vertex *src) : graph_(graph), source_(src) {
    const size_t n = graph.get_vertices().size();
    k_ = static_cast<size_t>(std::pow(std::log(n), 1.0/3.0));
    t_ = static_cast<size_t>(std::pow(std::log(n), 2.0/3.0));
}

std::pair<VertexSet, VertexSet> BMSSP::find_pivots(const VertexSet &S, double B) {

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
