#include <iostream>

#include "src/FibHeap.h"
#include "src/Graph.h"

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

int main() {
    Graph g{};

}
