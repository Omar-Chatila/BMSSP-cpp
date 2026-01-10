#include "Dijkstra.h"
#include "FibHeap.h"

Dijkstra::Dijkstra(Graph& graph, const Vertex* src) : graph_(graph), source_(src) {}

std::vector<double> Dijkstra::run() const {
    const size_t n = graph_.size();
    auto& vertices = graph_.get_vertices();
    std::vector<DijkstraState> states_(n);
    std::vector<bool> state_exists(n, false);
    for (const auto& v : vertices) {
        const Vertex* vertex = &v;
        states_[vertex->id_] = DijkstraState{std::numeric_limits<double>::infinity(), false, nullptr};
        state_exists[vertex->id_] = true;
    }
    states_[source_->id_].dist_ = 0;
    FibHeap<HeapKey> priority_queue;
    states_[source_->id_].heap_node_ = priority_queue.insert({0, source_});

    while (!priority_queue.empty()) {
        auto [dist_u, u] = priority_queue.extract_min();

        if (states_[u->id_].finalized_ == true)
            continue;

        states_[u->id_].finalized_ = true;
        states_[u->id_].heap_node_ = nullptr;

        for (const auto& [v_id, weight] : u->outgoing_edges_) {
            const Vertex* v = graph_.get_vertex(v_id);
            if (states_[v->id_].finalized_)
                continue;

            const double new_weight = states_[u->id_].dist_ + weight;
            if (new_weight < states_[v->id_].dist_) {
                HeapKey v_key{new_weight, v};
                const auto v_node = states_[v->id_].heap_node_;
                if (v_node == nullptr) {
                    states_[v->id_].heap_node_ = priority_queue.insert(v_key);
                } else {
                    priority_queue.decrease_key(v_node, v_key);
                }
                states_[v->id_].dist_ = new_weight;
            }
        }
    }

    std::vector<double> result(n);
    for (size_t i = 0; i < states_.size(); ++i)
        result[i] = states_[i].dist_;

    return result;
}