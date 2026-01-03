//
// Created by omar on 28.12.25.
//

#ifndef ALGO_SEMINAR_DIJKSTRA_H
#define ALGO_SEMINAR_DIJKSTRA_H

#include "Graph.h"
#include "FibHeap.h"

struct HeapKey {
    double dist;
    const Vertex* v;

    bool operator<(const HeapKey& key) const {
        return this->dist < key.dist;
    }
};

struct DijkstraState {
    double dist_ = std::numeric_limits<double>::infinity();
    bool finalized_ = false;
    Node<HeapKey>* heap_node_ = nullptr;

    bool operator<(const DijkstraState& ds) const {
        return this->dist_ < ds.dist_;
    }
    bool operator==(const DijkstraState& ds) = delete;
    bool operator>(const DijkstraState& ds) = delete;
};

class Dijkstra {
private:
    Graph& graph_;
    Vertex* source_;
    std::unordered_map<const Vertex* , DijkstraState> states_;
public:
    void reset_states();
    explicit Dijkstra(Graph& graph, Vertex* src);
    std::unordered_map<const Vertex* , double> run();
};


#endif //ALGO_SEMINAR_DIJKSTRA_H