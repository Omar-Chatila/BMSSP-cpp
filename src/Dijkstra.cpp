#include "Dijkstra.h"
#include "FibHeap.h"

Dijkstra::Dijkstra(Graph& graph, const Vertex* src) : graph_(graph), source_(src) {}

std::unordered_map<const Vertex *, double> Dijkstra::run() const {
    auto& vertices = graph_.get_vertices();
    std::unordered_map<const Vertex* , DijkstraState> states_;
    for (const auto& v : vertices) {
        const Vertex* vertex = &v;
        states_[vertex] = DijkstraState{std::numeric_limits<double>::infinity(), false, nullptr};
    }
    states_[source_].dist_ = 0;
    FibHeap<HeapKey> priority_queue;
    states_[source_].heap_node_ = priority_queue.insert({0, source_});

    while (!priority_queue.empty()) {
        auto [dist_u, u] = priority_queue.extract_min();

        if (states_[u].finalized_ == true)
            continue;

        states_[u].finalized_ = true;
        states_[u].heap_node_ = nullptr;

        for (const auto& [v_id, weight] : u->outgoing_edges_) {
            const Vertex* v = graph_.get_vertex(v_id);
            if (states_[v].finalized_)
                continue;

            const double new_weight = states_[u].dist_ + weight;
            if (new_weight < states_[v].dist_) {
                HeapKey v_key{new_weight, v};
                const auto v_node = states_[v].heap_node_;
                if (v_node == nullptr) {
                    states_[v].heap_node_ = priority_queue.insert(v_key);
                } else {
                    priority_queue.decrease_key(v_node, v_key);
                }
                states_[v].dist_ = new_weight;
            }
        }
    }

    std::unordered_map<const Vertex*, double> result;
    for (const auto& [vertex, node] : states_) {
        result[vertex] = node.dist_;
    }
    return result;
}


