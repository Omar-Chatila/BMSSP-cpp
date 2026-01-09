#include <chrono>
#include <iostream>
#include <random>
#include <benchmark/benchmark.h>

#include "BlockLinkedList.h"
#include "BMSSP.h"
#include "Dijkstra.h"
#include "FibHeap.h"
#include "Graph.h"
#include "GraphFactory.h"

/*
size_t dfs(const Vertex* u, std::unordered_map<const Vertex*, std::vector<const Vertex*>>& children,
    std::unordered_map<const Vertex*, size_t>& subtree_size) {
    size_t size = 1;
    for (const auto v : children[u])
        size += dfs(v, children, subtree_size);
    subtree_size[u] = size;
    return size;
}
 */


void fib_heap_demo() {
    FibHeap<int> fib_heap;
    fib_heap.insert(8);
    fib_heap.insert(2);
    fib_heap.insert(1);
    fib_heap.insert(7);
    fib_heap.insert(2);

    const auto min = fib_heap.extract_min();
    std::cout << "Min: " << min << std::endl;
    auto min2 = fib_heap.extract_min();
    auto min3 = fib_heap.extract_min();
    std::cout << "Min 2: " << min2 << "\n";
    std::cout << "Min 3: " << min3 << "\n";

    FibHeap<int> fib_heap2;
    fib_heap2.insert(3);
    fib_heap2.insert(9);
    fib_heap2.insert(5);
    fib_heap.merge(fib_heap2);
    std::cout << "min merged" << fib_heap.min() << std::endl;
    while (!fib_heap.empty()) fib_heap.extract_min();
}

void block_list_demo() {
    constexpr int n = 1000;
    DequeueBlocks D(n, 100, 120);
    std::vector<Vertex*> vertices;
    vertices.reserve(n);

    // Define range
    int min = 1;
    int max = 200;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(min, max);

    for (int i = 0; i < 1000; ++i) {
        const int randomValue = distrib(gen);
        std::cout << randomValue << std::endl;
        vertices.push_back(new Vertex(i));
        D.insert(vertices[i], randomValue);
    }

    auto [pulled, bound] = D.pull();
    std::cout << "bound " << bound << "\n";
    for (const auto& [vert, val] : pulled) {
        std::cout << vert->id_ << " : " << val << "\n";
    }
    D.pull();
    auto [p2, b2] = D.pull();
    std::cout << "bound " << b2 << "\n";
    for (const auto& [vert, val] : p2) {
        std::cout << vert->id_ << " : " << val << "\n";
    }
}

auto get_directed_example() {
    /*
     * http://andrewd.ces.clemson.edu/courses/cpsc212/f06/labs/lab09.html
     * Expected Output:
     *   Shortest path from 1 to 1 is 0
     *   Shortest path from 1 to 2 is 3
     *   Shortest path from 1 to 3 is 1
     *   Shortest path from 1 to 4 is 5
     *   Shortest path from 1 to 5 is 3
     *   Shortest path from 1 to 6 is 4
    */

    Graph g(GraphType::DIRECTED);
    for (int i = 0; i < 6; ++i)
        g.add_vertex(i);
    const Vertex* src = g.get_vertex(0);

    g.add_edge(0, 1, 3);
    g.add_edge(0,2,1);
    g.add_edge(1,2,2);
    g.add_edge(1,3,3);
    g.add_edge(1,4,6);
    g.add_edge(2,4,2);
    g.add_edge(4, 5, 1);
    g.add_edge(5,3,1);

    return std::make_pair(std::move(g), src);
}

auto get_undirected_example() {
    /*
     * https://www.researchgate.net/figure/Example-Graph-for-dynamic-Dijkstra-algorithm_fig5_323578961
     * Expected Output:
     *  Shortest path from 0 to 0 is 0
     *  Shortest path from 0 to 1 is 2
     *  Shortest path from 0 to 2 is 4
     *  Shortest path from 0 to 3 is 4
     *  Shortest path from 0 to 4 is 8
     *  Shortest path from 0 to 5 is 7
     *  Shortest path from 0 to 6 is 14
     *  Shortest path from 0 to 7 is 13
    */

    Graph g(GraphType::UNDIRECTED);
    for (int i = 0; i < 8; ++i) {
        g.add_vertex(i);
    }
    const Vertex* src = g.get_vertex(0);
    g.add_edge(0, 1, 2);
    g.add_edge(0, 3, 4);
    g.add_edge(0, 2, 5);
    g.add_edge(2, 3, 1);
    g.add_edge(1, 2, 2);
    g.add_edge(3, 5, 4);
    g.add_edge(2, 5, 3);
    g.add_edge(2, 4, 4);
    g.add_edge(1, 4, 7);
    g.add_edge(1, 6, 12);
    g.add_edge(4, 5, 4);
    g.add_edge(4, 7, 5);
    g.add_edge(5, 7, 7);
    g.add_edge(6, 7, 3);

    return std::make_pair(std::move(g), src);
}

void dijkstra_vs_bmssp_demo(GraphType type) {
    auto [g, src] = type == GraphType::DIRECTED ? get_directed_example() : get_undirected_example();
    std::cout << "Dijkstra: " << "\n";
    constexpr int iterations = 1000;

    long dijkstra_time = 0;
    for (int i = 0; i < iterations; ++i) {
        auto begin = std::chrono::steady_clock::now();
        Dijkstra dijkstra(g, src);
        auto vertex_dists_dijkstra = dijkstra.run();
        auto end = std::chrono::steady_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
        dijkstra_time += time;
    }
    dijkstra_time /= iterations;

    const Dijkstra dijkstra(g, src);
    const auto vertex_dists_dijkstra = dijkstra.run();
    for (size_t i = 0; i < vertex_dists_dijkstra.size(); ++i) {
        std::cout << "Shortest path from " << src->id_ << " to " << i << " is " << vertex_dists_dijkstra[i] << "\n";
    }
    std::cout << "Dijkstra execution took " << dijkstra_time << " ns\n\n";

    std::cout << "BMSSP: " << "\n";
    long bmssp_time = 0;
    for (int i = 0; i < iterations; ++i) {
        auto begin = std::chrono::steady_clock::now();
        BMSSP bmssp(g, src);
        auto vertex_dists_bmssp = bmssp.run();
        auto end = std::chrono::steady_clock::now();
        const auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
        bmssp_time += time;
    }
    bmssp_time /= iterations;

    const BMSSP bmssp(g, src);
    const auto vertex_dists_bmssp = bmssp.run();
    for (size_t i = 0; i < vertex_dists_bmssp.size(); ++i) {
        std::cout << "Shortest path from " << src->id_ << " to " << i << " is " << vertex_dists_bmssp[i] << "\n";
    }
    std::cout << "BMSSP execution took " << bmssp_time << " ns\n";
}

void time_dijkstra(Graph& g, const std::vector<const Vertex*>& srcs) {
    long total = 0;
    for (const Vertex* src : srcs) {
        auto begin = std::chrono::steady_clock::now();
        Dijkstra dijkstra(g, src);
        auto vertex_dists_dijkstra = dijkstra.run();
        auto end = std::chrono::steady_clock::now();
        const auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        total += time;
    }
    const size_t n = srcs.size();
    std::cout << "Total run time avg over " << n << " runs: " << static_cast<double>(total) / n << " us\n\n";
}

void time_bmssp(Graph& g, const std::vector<const Vertex*>& srcs) {
    long total = 0;
    for (const Vertex* src : srcs) {
        auto begin = std::chrono::steady_clock::now();
        //BMSSP bmssp(g, src);
        BMSSP bmssp(g, src, 6, 7);

        auto vertex_dists_dijkstra = bmssp.run();
        auto end = std::chrono::steady_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        total += time;
    }
    const size_t n = srcs.size();
    std::cout << "Total run time avg over " << n << " runs: " << static_cast<double>(total) / n << " us";
}

int main() {
    auto graph = graph_from_csv("../resources/graph1000.csv", GraphType::DIRECTED);
    auto srcs = get_start_vertices(graph, 10);
    std::cout << srcs.size() << std::endl;
    //std::cout << "start dijkstra runs\n";
    time_dijkstra(graph, srcs);
    std::cout << "start bmssp runs\n";
    time_bmssp(graph, srcs);
    return 0;
}

