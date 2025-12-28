//
// Created by omar on 22.12.25.
//

#pragma once
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include "FibHeap.h"

struct Edge {
    uint64_t to_id_;
    double weight_;

    explicit Edge(const uint64_t to, const double weight) : to_id_(to), weight_(weight) {}
};

struct Vertex {
    uint64_t id_;
    std::vector<Edge> outgoing_edges_;

    explicit Vertex(const uint64_t id) : id_(id) {}
};



class Graph {
private:
    std::deque<Vertex> vertices_;
    std::unordered_map<uint64_t, Vertex*> id_map_;

public:
    explicit Graph();
    void add_vertex(uint64_t id);
    void add_edge(uint64_t from_id, uint64_t to_id, double weight) const;
    const std::deque<Vertex>& get_vertices() const;
    const Vertex* get_vertex(uint64_t id) const;
};
