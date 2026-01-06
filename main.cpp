#include <iostream>
#include <random>

#include "BlockLinkedList.h"
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
    int n = 1000;
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
}

int main() {

}
