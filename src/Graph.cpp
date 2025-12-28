//
// Created by omar on 28.12.25.
//
#include "Graph.h"

Graph::Graph() {

}

void Graph::add_vertex(const uint64_t id) {
    vertices_.emplace_back(id);
    id_map_[id] = &vertices_.back();
}

void Graph::add_edge(const uint64_t from_id, const uint64_t to_id, const double weight) const {
    Vertex* v = id_map_.at(from_id);
    v->outgoing_edges_.emplace_back(to_id, weight);
}

const std::deque<Vertex> &Graph::get_vertices() const {
    return vertices_;
}

const Vertex *Graph::get_vertex(const uint64_t id) const {
    return id_map_.at(id);
}
