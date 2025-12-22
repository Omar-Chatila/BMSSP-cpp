//
// Created by omar on 22.12.25.
//

#pragma once
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include "FibHeap.h"

struct Vertex;

struct Edge {
    uint64_t to_id_;
    double weight_;
};

struct Vertex {
    uint64_t id_;
    std::vector<Edge> outgoing_edges_;
};

struct DijkstraState {
    Vertex* v = nullptr;
    double dist_ = std::numeric_limits<double>::infinity();
    bool finalized_ = false;
    Node<DijkstraState>* heap_node_ = nullptr;
};

class Graph {
private:
    std::deque<Vertex> vertices_;
    std::unordered_map<uint64_t, Vertex*> id_map_;
public:
    explicit Graph();
    void add_vertex(uint64_t id);
    void add_edge(uint64_t from_id, uint64_t to_id, double weight);
};
