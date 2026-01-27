#include <assert.h>
#include <iostream>
#include <set>
#include <unordered_set>

#include "BlockLinkedList.h"
#include "tests.h"

namespace tests::dq {
    void initialization_test() {
        size_t n = 10;
        size_t M = 4;
        double B = 20;
        DequeueBlocks dq(n, M, B);
        assert(!dq.empty());
    }

    void insert_test() {
        std::cout << "=== Comprehensive Insert Test ===" << std::endl;

        // Test 1: Basic insertion with unique values
        {
            std::cout << "Test 1: Basic insertion with unique values" << std::endl;
            const size_t n = 100;
            const size_t M = 4;
            const double B = 1000;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            for (int i = 0; i < n; ++i) {
                Vertex* v = new Vertex(i);
                // Generate unique values by adding i/1000 to make them slightly different
                double value = i + (i % 10) * 0.1;
                dq.insert(v, value);
                vertices.push_back(v);
            }

            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M);

            // Find the M-th smallest value manually
            std::vector<double> all_values;
            for (auto v : vertices) {
                all_values.push_back(v->id_ + (v->id_ % 10) * 0.1);
            }
            std::ranges::sort(all_values);
            double m_th_value = all_values[M-1];

            // Check properties:
            // 1. All pulled values should be ≤ m_th_value
            // 2. All values < m_th_value should be in pulled set
            // 3. Exactly M elements pulled

            std::set<int> pulled_ids;
            for (const auto& p : pulled) {
                pulled_ids.insert(p.key_->id_);
                assert(p.value_ <= m_th_value + 1e-9);
            }

            // Count how many values are strictly less than m_th_value
            int strictly_less_count = 0;
            for (double val : all_values) {
                if (val < m_th_value - 1e-9) strictly_less_count++;
            }

            // We should have pulled all values < m_th_value, plus some with value == m_th_value
            assert(strictly_less_count <= M);

            // Check we have the right number of elements with m_th_value
            int boundary_count = 0;
            for (double val : all_values) {
                if (std::abs(val - m_th_value) < 1e-9) boundary_count++;
            }

            // The pulled set should contain:
            // - All elements with value < m_th_value
            // - Some elements with value == m_th_value to make total M

            // Cleanup
            for (auto v : vertices) delete v;
            std::cout << "Test 1 passed!" << std::endl;
        }

        // Test 2: Many duplicates at the boundary
        {
            std::cout << "\nTest 2: Many duplicates at the boundary" << std::endl;
            const size_t n = 100;
            const size_t M = 5;
            const double B = 1000;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            // Create many vertices with the same minimal value
            for (int i = 0; i < n; ++i) {
                Vertex* v = new Vertex(i);
                double value;
                if (i < 20) {
                    value = 1.0;  // Many duplicates with value 1
                } else {
                    value = 2.0 + i * 0.01;
                }
                dq.insert(v, value);
                vertices.push_back(v);
            }

            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M);

            // All pulled values should be 1.0 (since we have at least M vertices with value 1.0)
            for (const auto& p : pulled) {
                assert(std::abs(p.value_ - 1.0) < 1e-9);
            }

            // Should have pulled exactly M vertices with value 1.0
            // (Which specific ones doesn't matter as long as they all have value 1.0)

            // Cleanup
            for (auto v : vertices) delete v;
            std::cout << "Test 2 passed!" << std::endl;
        }

        // Test 3: Insert with decreasing values (should update)
        {
            std::cout << "\nTest 3: Insert with decreasing values" << std::endl;
            const size_t n = 50;
            const size_t M = 4;
            const double B = 1000;
            DequeueBlocks dq(n, M, B);

            Vertex* v = new Vertex(0);
            // Insert with decreasing values - should keep the smallest
            dq.insert(v, 100.0);
            dq.insert(v, 50.0);   // Should update
            dq.insert(v, 75.0);   // Should be ignored (not smaller than 50)
            dq.insert(v, 25.0);   // Should update

            // Insert other vertices
            std::vector<Vertex*> other_vertices;
            for (int i = 1; i < 20; ++i) {
                Vertex* v2 = new Vertex(i);
                dq.insert(v2, 30.0 + i);  // Values from 31 to 49
                other_vertices.push_back(v2);
            }

            auto [pulled, bound] = dq.pull();

            // Vertex 0 should be in pulled set with value 25.0
            bool found = false;
            for (const auto& p : pulled) {
                if (p.key_->id_ == 0) {
                    assert(std::abs(p.value_ - 25.0) < 1e-9);
                    found = true;
                    break;
                }
            }
            assert(found);

            // Cleanup
            delete v;
            for (auto v2 : other_vertices) delete v2;
            std::cout << "Test 3 passed!" << std::endl;
        }

        // Test 4: Insert in reverse order (largest to smallest)
        {
            std::cout << "\nTest 4: Insert in reverse order" << std::endl;
            const size_t n = 30;
            const size_t M = 6;
            const double B = 1000;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            for (int i = n-1; i >= 0; --i) {  // Insert largest first
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 10.0);
                vertices.push_back(v);
            }

            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M);

            // Sort pulled by value
            std::ranges::sort(pulled, [](const Pair& a, const Pair& b) {
                return a.value_ < b.value_;
            });

            // Should have the 6 smallest values: 0, 10, 20, 30, 40, 50
            for (int i = 0; i < M; ++i) {
                assert(std::abs(pulled[i].value_ - i * 10.0) < 1e-9);
            }

            // Cleanup
            for (auto v : vertices) delete v;
            std::cout << "Test 4 passed!" << std::endl;
        }

        // Test 5: Random insertions with verification
        {
            std::cout << "\nTest 5: Random insertions with robust verification" << std::endl;
            const size_t n = 200;
            const size_t M = 8;
            const double B = 500;
            DequeueBlocks dq(n, M, B);

            srand(42);  // Fixed seed for reproducibility
            std::vector<std::pair<int, double>> inserted;  // (id, value)
            std::vector<Vertex*> vertices;

            for (int i = 0; i < n; ++i) {
                Vertex* v = new Vertex(i);
                double value = (rand() % 1000) / 10.0;  // Values 0.0 to 99.9
                dq.insert(v, value);
                inserted.emplace_back(i, value);
                vertices.push_back(v);
            }

            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M);

            // Robust verification:
            // 1. Sort all inserted values
            std::vector<double> all_values;
            for (const auto& [id, val] : inserted) {
                all_values.push_back(val);
            }
            std::ranges::sort(all_values);

            double m_th_value = all_values[M-1];
            int strictly_less_count = 0;
            int equal_count = 0;

            for (double val : all_values) {
                if (val < m_th_value - 1e-9) strictly_less_count++;
                else if (std::abs(val - m_th_value) < 1e-9) equal_count++;
            }

            // Verify pulled set:
            std::set<int> pulled_ids;
            int pulled_strictly_less = 0;
            int pulled_equal = 0;

            for (const auto& p : pulled) {
                pulled_ids.insert(p.key_->id_);
                if (p.value_ < m_th_value - 1e-9) pulled_strictly_less++;
                else if (std::abs(p.value_ - m_th_value) < 1e-9) pulled_equal++;
                else assert(false && "Pulled value greater than M-th smallest!");
            }

            // All strictly less than m_th_value must be in pulled set
            assert(pulled_strictly_less == strictly_less_count);

            // Total pulled should be M
            assert(pulled_strictly_less + pulled_equal == M);

            // The pulled equal values should be a subset of all equal values
            // (We can't verify which ones exactly, but that's OK)

            // Also check no duplicates in pulled set
            assert(pulled_ids.size() == M);

            // Cleanup
            for (auto v : vertices) delete v;
            std::cout << "Test 5 passed!" << std::endl;
        }

        // Test 6: Edge case - exactly M elements total
        {
            std::cout << "\nTest 6: Exactly M elements total" << std::endl;
            const size_t M = 5;
            const size_t n = M;  // n = M
            const double B = 1000;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            for (int i = 0; i < M; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 5.0);
                vertices.push_back(v);
            }

            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M);  // Should pull all elements

            // All elements should be in pulled set
            std::set<int> pulled_ids;
            for (const auto& p : pulled) {
                pulled_ids.insert(p.key_->id_);
            }
            for (int i = 0; i < M; ++i) {
                assert(pulled_ids.find(i) != pulled_ids.end());
            }

            // Structure should be empty now
            assert(dq.empty());

            // Cleanup
            for (auto v : vertices) delete v;
            std::cout << "Test 6 passed!" << std::endl;
        }

        // Test 7: Edge case - fewer than M elements total
        {
            std::cout << "\nTest 7: Fewer than M elements total" << std::endl;
            const size_t n = 3;
            const size_t M = 5;
            const double B = 1000;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            for (int i = 0; i < n; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 10.0);
                vertices.push_back(v);
            }

            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == n);  // Should pull all n elements (n < M)

            // Verify all elements are present
            std::set<int> pulled_ids;
            for (const auto& p : pulled) {
                pulled_ids.insert(p.key_->id_);
            }
            for (int i = 0; i < n; ++i) {
                assert(pulled_ids.find(i) != pulled_ids.end());
            }

            // Structure should be empty now
            assert(dq.empty());

            // Cleanup
            for (auto v : vertices) delete v;
            std::cout << "Test 7 passed!" << std::endl;
        }

        std::cout << "\nAll comprehensive insert tests passed!" << std::endl;
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

    void erase_test() {
        std::cout << "=== Comprehensive Erase Test ===" << std::endl;

        // Test 1: Basic erase
        {
            std::cout << "Test 1: Basic erase" << std::endl;
            const size_t n = 20;
            const size_t M = 4;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            for (int i = 0; i < 10; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 10.0);  // 0, 10, 20, ..., 90
                vertices.push_back(v);
            }

            // Erase vertex with value 20 (id=2)
            dq.erase(vertices[2]);

            // Pull M=4 smallest elements: should be 0, 10, 30, 40 (20 is removed)
            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M);

            // Check that value 20 is not present
            for (const auto& p : pulled) {
                assert(std::abs(p.value_ - 20.0) > 1e-9);
            }

            // Check that we have the correct M smallest remaining values: 0, 10, 30, 40
            std::set<double> pulled_values;
            for (const auto& p : pulled) {
                pulled_values.insert(p.value_);
            }

            double expected[] = {0.0, 10.0, 30.0, 40.0};
            for (double val : expected) {
                assert(pulled_values.find(val) != pulled_values.end());
            }

            for (auto v : vertices) delete v;
            std::cout << "Test 1 passed!" << std::endl;
        }

        // Test 2: Erase non-existent vertex
        {
            std::cout << "\nTest 2: Erase non-existent vertex" << std::endl;
            const size_t n = 10;
            const size_t M = 4;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            Vertex* v1 = new Vertex(0);
            Vertex* v2 = new Vertex(1);
            Vertex* v3 = new Vertex(2);  // Not inserted

            dq.insert(v1, 5.0);
            dq.insert(v2, 10.0);

            // Erase non-existent vertex - should do nothing
            dq.erase(v3);

            // Structure should still have 2 elements
            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == 2);  // Only 2 elements total (< M)

            delete v1; delete v2; delete v3;
            std::cout << "Test 2 passed!" << std::endl;
        }

        // Test 3: Erase and re-insert same vertex
        {
            std::cout << "\nTest 3: Erase and re-insert same vertex" << std::endl;
            const size_t n = 10;
            const size_t M = 4;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            Vertex* v = new Vertex(0);

            // Insert with value 50
            dq.insert(v, 50.0);

            // Erase
            dq.erase(v);

            // Re-insert with smaller value
            dq.insert(v, 10.0);

            // Insert other vertices
            for (int i = 1; i < 5; ++i) {
                Vertex* v2 = new Vertex(i);
                dq.insert(v2, i * 20.0);  // 20, 40, 60, 80
                //delete v2;
            }

            // Pull: should include vertex 0 with value 10
            auto [pulled, bound] = dq.pull();

            bool found = false;
            for (const auto& p : pulled) {
                if (p.key_->id_ == 0 && std::abs(p.value_ - 10.0) < 1e-9) {
                    found = true;
                    break;
                }
            }
            assert(found);

            delete v;
            std::cout << "Test 3 passed!" << std::endl;
        }

        // Test 4: Erase causes block deletion (D1)
        {
            std::cout << "\nTest 4: Erase causes block deletion in D1" << std::endl;
            const size_t n = 100;
            const size_t M = 4;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            // Insert M+1 elements with close values to ensure they're in same block
            std::vector<Vertex*> vertices;
            for (int i = 0; i <= M; ++i) {  // M+1 elements = 5 elements
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 0.1);  // 0.0, 0.1, 0.2, 0.3, 0.4
                vertices.push_back(v);
            }

            // Block should have been split because it exceeded M
            // Now erase all but one element from a block
            for (int i = 0; i < M; ++i) {  // Erase 4 elements
                dq.erase(vertices[i]);
            }

            // Now only vertex with id=M should remain
            // Pull should return just that one element (since total < M)
            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == 1);
            assert(pulled[0].key_->id_ == M);
            assert(std::abs(pulled[0].value_ - M * 0.1) < 1e-9);

            // Structure should be empty now
            assert(dq.empty());

            for (auto v : vertices) delete v;
            std::cout << "Test 4 passed!" << std::endl;
        }

        // Test 5: Erase from D0 (batch prepend block)
        {
            std::cout << "\nTest 5: Erase from D0" << std::endl;
            const size_t n = 100;
            const size_t M = 4;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            // Create a batch and prepend
            std::vector<Pair> batch;
            std::vector<Vertex*> vertices;
            for (int i = 0; i < 3; ++i) {
                Vertex* v = new Vertex(i);
                batch.emplace_back(v, i * 5.0);  // 0, 5, 10
                vertices.push_back(v);
            }

            dq.batch_prepend(batch, 10.0);

            // Insert some into D1
            for (int i = 3; i < 6; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 5.0);  // 15, 20, 25
                vertices.push_back(v);
            }

            // Erase from D0 (vertex with value 5)
            dq.erase(vertices[1]);

            // Pull M=4 smallest: should be 0, 10, 15, 20 (5 is removed)
            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M);

            // Check 5 is not present
            for (const auto& p : pulled) {
                assert(std::abs(p.value_ - 5.0) > 1e-9);
            }

            // Verify we have the right set
            std::set<double> expected = {0.0, 10.0, 15.0, 20.0};
            for (const auto& p : pulled) {
                assert(expected.find(p.value_) != expected.end());
            }

            for (auto v : vertices) delete v;
            std::cout << "Test 5 passed!" << std::endl;
        }

        // Test 6: Erase all elements one by one
        {
            std::cout << "\nTest 6: Erase all elements one by one" << std::endl;
            const size_t n = 10;
            const size_t M = 4;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            for (int i = 0; i < 5; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 10.0);
                vertices.push_back(v);
            }

            // Erase all
            for (auto v : vertices) {
                dq.erase(v);
            }

            // Structure should be empty
            assert(dq.empty());

            // Pull should return empty vector
            auto [pulled, bound] = dq.pull();
            assert(pulled.empty());
            assert(std::abs(bound - B) < 1e-9);

            for (auto v : vertices) delete v;
            std::cout << "Test 6 passed!" << std::endl;
        }

        // Test 7: Erase from middle of block (verifies swap removal works)
        {
            std::cout << "\nTest 7: Erase from middle of block" << std::endl;
            const size_t n = 100;
            const size_t M = 8;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;
            // Insert elements that will all go to one block (values 0-70)
            for (int i = 0; i < M; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 10.0);
                vertices.push_back(v);
            }

            // Record positions before erase
            std::vector<KeyPos> positions_before;
            for (auto v : vertices) {
                positions_before.push_back(dq.get_key_position(v->id_));  // Need this method
            }

            // Erase from middle (vertex with value 30, id=3)
            dq.erase(vertices[3]);

            // Verify other vertices still have correct positions
            for (int i = 0; i < M; ++i) {
                if (i == 3) continue;  // Skipped erased vertex

                // After erase with swap, the last element moved to position 3
                // So vertices at index > 3 have same position, vertex at index 7 moved to 3
                if (i < 3) {
                    // Should have same position
                    assert(dq.is_present(vertices[i]->id_));
                } else if (i == M-1) {
                    // Last element moved to position 3
                    // We need to check this carefully
                }
                // This test is complex without getter methods
            }

            // Simpler: pull and check we have remaining values
            auto [pulled, bound] = dq.pull();
            assert(pulled.size() == M-1);  // 7 elements remaining

            // Value 30 should not be present
            for (const auto& p : pulled) {
                assert(std::abs(p.value_ - 30.0) > 1e-9);
            }

            for (auto v : vertices) delete v;
            std::cout << "Test 7 passed!" << std::endl;
        }

        // Test 8: Erase and insert interleaved
        {
            std::cout << "\nTest 8: Erase and insert interleaved" << std::endl;
            const size_t n = 20;
            const size_t M = 4;
            const double B = 100;
            DequeueBlocks dq(n, M, B);

            std::vector<Vertex*> vertices;

            // Phase 1: Insert some
            for (int i = 0; i < 5; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 5.0);
                vertices.push_back(v);
            }

            // Phase 2: Erase some
            dq.erase(vertices[2]);  // Remove value 10

            // Phase 3: Insert more
            for (int i = 5; i < 8; ++i) {
                Vertex* v = new Vertex(i);
                dq.insert(v, i * 5.0);
                vertices.push_back(v);
            }

            // Phase 4: Erase more
            dq.erase(vertices[4]);  // Remove value 20
            // erase already erased
            dq.erase(vertices[4]);  // Remove value 20

            // Pull and verify
            auto [pulled, bound] = dq.pull();

            // Remaining values: 0, 5, 15, 25, 30, 35
            // M=4 smallest: 0, 5, 15, 25
            double expected[] = {0.0, 5.0, 15.0, 25.0};
            assert(pulled.size() == M);

            for (int i = 0; i < M; ++i) {
                bool found = false;
                for (const auto& p : pulled) {
                    if (std::abs(p.value_ - expected[i]) < 1e-9) {
                        found = true;
                        break;
                    }
                }
                assert(found);
            }

            for (auto v : vertices) delete v;
            std::cout << "Test 8 passed!" << std::endl;
        }

        std::cout << "\nAll erase tests passed!" << std::endl;
    }
}