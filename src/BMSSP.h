#ifndef ALGO_SEMINAR_BMSSP_H
#define ALGO_SEMINAR_BMSSP_H
#include "BlockLinkedList.h"
#include "Graph.h"

using VertexSet = std::vector<Pair>;

class BMSSP {
private:
    Graph& graph_;
    const Vertex* source_;

    size_t n_;
    size_t k_;
    size_t t_;

    [[nodiscard]]
    std::pair<VertexSet, VertexSet> find_pivots(const VertexSet& S, double B) const;

    std::pair<double, VertexSet> base_case(Pair& S, double B, int l) const;

    std::pair<double, VertexSet> bmssp(int l, double B, VertexSet& S) const;
public:
    BMSSP(Graph& graph, const Vertex* src);

    BMSSP(Graph& graph, const Vertex* src, size_t k, size_t t);

    [[nodiscard]]
    std::vector<double> run() const;
};


#endif //ALGO_SEMINAR_BMSSP_H