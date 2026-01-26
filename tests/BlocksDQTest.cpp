#include <assert.h>
#include <iostream>
#include <set>
#include <unordered_set>

#include "BlockLinkedList.h"
#include "tests.h"

void initialization_test() {
    size_t n = 10;
    size_t M = 4;
    double B = 20;
    DequeueBlocks dq(n, M, B);
    assert(!dq.empty());
}

void insert_test() {
    const size_t n = 100;
    const size_t M = 4;
    const int B = 20;
    DequeueBlocks dq(n, M, B);
    srand(0);
    std::vector<Pair> vertices;
    for (int i = 0; i < n; ++i) {
        const double rand_val = rand() % B;
        Vertex* v = new Vertex(i);
        vertices.emplace_back(v, rand_val);
        dq.insert(v, rand_val);
    }
    std::cout << "\npulled\n"<< std::endl;
    auto [pulled, bound] = dq.pull();
    for (int i = 0; i < pulled.size(); ++i) {
        std::cout << pulled[i].key_->id_ << ": " << pulled[i].value_ << ", " << std::endl;
    }
    std::cout << std::endl;
    std::ranges::sort(vertices, [](const Pair& a, const Pair& b){return a.value_ < b.value_;});

    std::set<Pair> smallest;
    for (int i = 0; i < M; ++i) {
        smallest.emplace(vertices[i]);
    }

    for (int i = 0; i < M; ++i) {
        assert(smallest.contains(pulled[i]));
    }

    for (const auto v : vertices) delete v.key_;
    std::cout << "Insert test passed!\n";
}

void batch_prepend_test() {
    std::cout << "=== Testing batch_prepend ===" << std::endl;

    // Test 1: Small batch (L ≤ M)
    {
        std::cout << "Test 1: Small batch (L ≤ M)" << std::endl;
        const size_t N = 1000;
        const size_t M = 8;
        const double B = 1000;

        DequeueBlocks dq(N, M, B);

        // Create a small batch
        std::vector<Pair> batch;
        std::vector<Vertex*> vertices;
        for (int i = 0; i < 5; ++i) {  // L = 5 ≤ M = 8
            Vertex* v = new Vertex(i);
            double value = i * 10.0;  // 0, 10, 20, 30, 40
            batch.emplace_back(v, value);
            vertices.push_back(v);
        }

        double batch_upper = 40.0;  // max value
        dq.batch_prepend(batch, batch_upper);

        // Pull should return all batch elements
        auto [pulled, bound] = dq.pull();

        std::cout << "Pulled " << pulled.size() << " elements" << std::endl;
        assert(pulled.size() == 5);

        // Verify all batch elements were returned
        std::set<std::pair<int, double>> pulled_set;
        for (const auto& p : pulled) {
            pulled_set.insert({p.key_->id_, p.value_});
        }

        for (int i = 0; i < 5; ++i) {
            assert(pulled_set.find({i, i * 10.0}) != pulled_set.end());
        }

        // Cleanup
        for (auto v : vertices) delete v;
        std::cout << "Test 1 passed!" << std::endl << std::endl;
    }

    // Test 2: Large batch (L > M) - should create multiple blocks
    {
        std::cout << "Test 2: Large batch (L > M)" << std::endl;
        const size_t N = 1000;
        const size_t M = 4;
        const double B = 1000;

        DequeueBlocks dq(N, M, B);

        // Create a large batch
        std::vector<Pair> batch;
        std::vector<Vertex*> vertices;
        const int L = 20;

        for (int i = 0; i < L; ++i) {
            Vertex* v = new Vertex(i);
            double value = (i * 17) % 100;
            batch.emplace_back(v, value);
            vertices.push_back(v);
        }

        // Save values BEFORE batch_prepend moves the batch
        std::vector<double> batch_values;
        for (const auto& p : batch) {
            batch_values.push_back(p.value_);
        }

        double batch_upper = 99.0;
        dq.batch_prepend(batch, batch_upper);

        // Pull should return M smallest elements
        auto [pulled, bound] = dq.pull();

        std::cout << "Pulled " << pulled.size() << " elements" << std::endl;
        assert(pulled.size() == M);

        // Now batch_values has the original values
        std::ranges::sort(batch_values);
        std::vector<double> expected_smallest(batch_values.begin(),
                                              batch_values.begin() + M);

        // Check each pulled element is among expected smallest
        for (const auto& p : pulled) {
            bool found = false;
            for (double expected : expected_smallest) {
                if (std::abs(p.value_ - expected) < 1e-9) {
                    found = true;
                    break;
                }
            }
            assert(found && "Pulled element not among M smallest");
        }

        // Cleanup
        for (auto v : vertices) delete v;
        std::cout << "Test 2 passed!" << std::endl << std::endl;
    }

    // Test 3: Batch prepend after some insertions
    {
        std::cout << "Test 3: Batch prepend with existing elements" << std::endl;
        const size_t N = 1000;
        const size_t M = 6;
        const double B = 1000;

        DequeueBlocks dq(N, M, B);

        // First insert some elements
        std::vector<Vertex*> vertices1;
        for (int i = 0; i < 3; ++i) {
            Vertex* v = new Vertex(i);
            dq.insert(v, 50.0 + i * 5.0);  // 50, 55, 60
            vertices1.push_back(v);
        }

        // Then batch prepend with smaller values
        std::vector<Pair> batch;
        std::vector<Vertex*> vertices2;
        for (int i = 0; i < 4; ++i) {
            Vertex* v = new Vertex(10 + i);  // Different IDs
            batch.emplace_back(v, i * 10.0);  // 0, 10, 20, 30
            vertices2.push_back(v);
        }

        dq.batch_prepend(batch, 30.0);

        // Pull - should get 0, 10, 20, 30, 50, 55 (M=6 smallest)
        auto [pulled, bound] = dq.pull();

        std::cout << "Pulled " << pulled.size() << " elements" << std::endl;
        assert(pulled.size() == M);

        // Sort pulled by value
        std::ranges::sort(pulled, [](const Pair& a, const Pair& b) {
            return a.value_ < b.value_;
        });

        // Verify order
        double expected[] = {0.0, 10.0, 20.0, 30.0, 50.0, 55.0};
        for (int i = 0; i < M; ++i) {
            assert(std::abs(pulled[i].value_ - expected[i]) < 1e-9);
        }

        // Cleanup
        for (auto v : vertices1) delete v;
        for (auto v : vertices2) delete v;
        std::cout << "Test 3 passed!" << std::endl << std::endl;
    }

    // Test 4: Batch prepend with duplicate values
    {
        std::cout << "Test 4: Batch prepend with duplicates" << std::endl;
        const size_t N = 1000;
        const size_t M = 4;
        const double B = 1000;

        DequeueBlocks dq(N, M, B);

        std::vector<Pair> batch;
        std::vector<Vertex*> vertices;

        // Create batch with duplicate values
        for (int i = 0; i < 8; ++i) {
            Vertex* v = new Vertex(i);
            double value = (i % 3) * 10.0;  // Values: 0, 10, 20, 0, 10, 20, 0, 10
            batch.emplace_back(v, value);
            vertices.push_back(v);
        }

        dq.batch_prepend(batch, 20.0);

        // Pull should return M=4 smallest values (0, 0, 0, 10)
        auto [pulled, bound] = dq.pull();
        assert(pulled.size() == M);

        // Count values
        int zero_count = 0, ten_count = 0;
        for (const auto& p : pulled) {
            if (std::abs(p.value_ - 0.0) < 1e-9) zero_count++;
            else if (std::abs(p.value_ - 10.0) < 1e-9) ten_count++;
        }

        // Should have 3 zeros and 1 ten (from 8 elements: 0,0,0,10,10,10,20,20)
        // M=4 smallest: 0,0,0,10
        assert(zero_count == 3);
        assert(ten_count == 1);

        // Cleanup
        for (auto v : vertices) delete v;
        std::cout << "Test 4 passed!" << std::endl << std::endl;
    }

    // Test 5: Multiple batch prepends
    {
        std::cout << "Test 5: Multiple batch prepends" << std::endl;
        const size_t N = 1000;
        const size_t M = 5;
        const double B = 1000;

        DequeueBlocks dq(N, M, B);

        // First batch
        std::vector<Pair> batch1;
        std::vector<Vertex*> vertices1;
        for (int i = 0; i < 3; ++i) {
            Vertex* v = new Vertex(i);
            batch1.emplace_back(v, 100.0 + i * 10.0);  // 100, 110, 120
            vertices1.push_back(v);
        }
        dq.batch_prepend(batch1, 120.0);

        // Second batch with smaller values
        std::vector<Pair> batch2;
        std::vector<Vertex*> vertices2;
        for (int i = 0; i < 4; ++i) {
            Vertex* v = new Vertex(10 + i);
            batch2.emplace_back(v, i * 5.0);  // 0, 5, 10, 15
            vertices2.push_back(v);
        }
        dq.batch_prepend(batch2, 15.0);

        // Pull should get: 0, 5, 10, 15, 100 (M=5 smallest)
        auto [pulled, bound] = dq.pull();
        assert(pulled.size() == M);

        // Sort and verify
        std::ranges::sort(pulled, [](const Pair& a, const Pair& b) {
            return a.value_ < b.value_;
        });

        double expected[] = {0.0, 5.0, 10.0, 15.0, 100.0};
        for (int i = 0; i < M; ++i) {
            assert(std::abs(pulled[i].value_ - expected[i]) < 1e-9);
        }

        // Cleanup
        for (auto v : vertices1) delete v;
        for (auto v : vertices2) delete v;
        std::cout << "Test 5 passed!" << std::endl << std::endl;
    }

    // Test 6: Edge case - empty batch
    {
        std::cout << "Test 6: Empty batch" << std::endl;
        const size_t N = 1000;
        const size_t M = 4;
        const double B = 1000;

        DequeueBlocks dq(N, M, B);

        std::vector<Pair> batch;  // Empty
        dq.batch_prepend(batch, 0.0);

        // Should still work - structure unchanged
        assert(!dq.empty());  // Still has initial D1 block

        std::cout << "Test 6 passed!" << std::endl << std::endl;
    }

    std::cout << "All batch_prepend tests passed!" << std::endl;
}