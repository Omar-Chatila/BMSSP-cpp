#include <chrono>
#include <iostream>
#include <random>

#include "BlockLinkedList.h"
#include "BMSSP.h"
#include "Dijkstra.h"
#include "FibHeap.h"
#include "Graph.h"

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
    DequeueBlocks D(100, 120);
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

int main() {
    /*
     * Expected Output:
     *   Shortest path from 1 to 1 is 0
     *   Shortest path from 1 to 2 is 3
     *   Shortest path from 1 to 3 is 1
     *   Shortest path from 1 to 4 is 5
     *   Shortest path from 1 to 5 is 3
     *   Shortest path from 1 to 6 is 4
    */

    Graph g;
    for (int i = 1; i <= 6; ++i)
        g.add_vertex(i);
    const Vertex* src = g.get_vertex(1);

    g.add_edge(1, 2, 3);
    g.add_edge(1,3,1);
    g.add_edge(2,3,2);
    g.add_edge(2,4,3);
    g.add_edge(2,5,6);
    g.add_edge(3,5,2);
    g.add_edge(5, 6, 1);
    g.add_edge(6,4,1);

    std::cout << "Dijkstra: " << "\n";
    auto begin = std::chrono::steady_clock::now();
    Dijkstra dijkstra(g, src);
    auto vertex_dists_dijkstra = dijkstra.run();
    auto end = std::chrono::steady_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

    for (const auto& [v, v_dist] : vertex_dists_dijkstra) {
        std::cout << "Shortest path from " << src->id_ << " to " << v->id_ << " is " << v_dist << "\n";
    }
    std::cout << "Dijkstra execution took " << time << "us\n\n";

    std::cout << "BMSSP: " << "\n";
    begin = std::chrono::steady_clock::now();
    BMSSP bmssp(g, src);
    auto vertex_dists_bmssp = bmssp.run();
    end = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    std::cout << "BMSSP execution took " << time << "us\n";

    for (const auto& [v, v_dist] : vertex_dists_dijkstra) {
        std::cout << "Shortest path from " << src->id_ << " to " << v->id_ << " is " << v_dist << "\n";
    }
}
