#include "Graph.h"

Graph::Graph(const GraphType type) : type_(type) {}

void Graph::add_vertex(const uint64_t id) {
    if (id < id_map_.size())
        return;
    vertices_.emplace_back(id);
    id_map_.resize(id + 1);
    id_map_[id] = &vertices_.back();
}

void Graph::add_edge(const uint64_t from_id, const uint64_t to_id, const double weight) {
    Vertex* v = id_map_.at(from_id);
    if (type_ == GraphType::DIRECTED) {
        v->outgoing_edges_.emplace_back(to_id, weight);
    } else {
        Vertex* u = id_map_.at(to_id);
        v->outgoing_edges_.emplace_back(to_id, weight);
        u->outgoing_edges_.emplace_back(from_id, weight);
    }
    ++num_edges_;
}

const std::deque<Vertex> &Graph::get_vertices() const {
    return vertices_;
}

const Vertex *Graph::get_vertex(const uint64_t id) const {
    return id_map_[id];
}

bool Graph::empty() const {
    return vertices_.empty();
}

size_t Graph::size() const {
    return vertices_.size();
}

size_t Graph::edges_size() const {
    return num_edges_;
}