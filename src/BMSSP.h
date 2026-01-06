#ifndef ALGO_SEMINAR_BMSSP_H
#define ALGO_SEMINAR_BMSSP_H
#include "BlockLinkedList.h"
#include "Graph.h"

using VertexSet = std::vector<Pair>;

class BMSSP {
private:
    Graph& graph_;
    Vertex* source_;

    size_t k_;
    size_t t_;
    std::pair<VertexSet, VertexSet> find_pivots(const VertexSet& S, double B) const;

    std::pair<double, VertexSet> base_case(Pair& S, const double B) const;

    std::pair<double, VertexSet> bmssp(int l, double B, VertexSet& S) const;
public:
    explicit BMSSP(Graph& graph, Vertex* src);

    std::unordered_map<const Vertex *, double> run() const;
};


#endif //ALGO_SEMINAR_BMSSP_H