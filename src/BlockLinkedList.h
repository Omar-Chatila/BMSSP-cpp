#ifndef ALGO_SEMINAR_BLOCK_LINKED_LIST_H
#define ALGO_SEMINAR_BLOCK_LINKED_LIST_H

#include <algorithm>
#include <limits>
#include <list>
#include <map>

#include "Graph.h"


struct Pair {
    const Vertex* key_;
    double value_;

    bool operator<(const Pair& o) const noexcept {
        return this->value_ < o.value_;
    }

    Pair(const Vertex* k, const double v) : key_(k), value_(v){}
};

enum class BlockOwner {D0, D1};

struct Block {
    std::list<Pair> elems_;
    double upper_;
    BlockOwner owner_;

    explicit Block(const double B, const BlockOwner owner) : upper_(B), owner_(owner) {}
};

struct KeyPos {
    std::list<Block>::iterator block_;
    std::list<Pair>::iterator elem_;
};

class DequeueBlocks {
    // Batch-prepend
    std::list<Block> D0_;

    // Normal inserts
    std::list<Block> D1_;
    std::multimap<double, std::list<Block>::iterator> D1_tree_;     // Red-Black Tree

    // Key bookkeeping
    std::vector<KeyPos> key_poses_;
    std::vector<bool> present_;

    // Params
    size_t M_;
    double B_upper_;

    static double find_median_value(const std::list<Pair>& elems) {
        std::vector<double> vals;
        vals.reserve(elems.size());
        for (auto& p : elems)
            vals.push_back(p.value_);

        const auto mid = vals.begin() + vals.size() / 2;
        std::ranges::nth_element(vals, mid);
        return *mid;
    }

public:
    // Initialize(M, B)
    DequeueBlocks(const size_t N, const size_t M, const double B) : M_(M), B_upper_(B) {
        key_poses_ = std::vector<KeyPos>(N, KeyPos{});
        present_ = std::vector(N, false);
        D1_.emplace_back(B, BlockOwner::D1);
        auto it = D1_.begin();
        D1_tree_.emplace(B, it);
    }

    // Insert(a, b)
    void insert(const Vertex* a, const double b) {
        // To insert a key/value pair ⟨a, b⟩, we first check the existence of its key a
        if (present_[a->id_]) {
            // If a already exists, we delete original pair ⟨a, b′⟩ and insert new pair ⟨a, b⟩ only when b < b′.
            const auto& pos = key_poses_[a->id_];
            const double old_b = pos.elem_->value_;
            if (b >= old_b) return;
            erase(pos, a);
        }
        // We first locate the appropriate block for it, which is the block with the smallest upper bound greater than or equal to b,
        const auto it = D1_tree_.lower_bound(b);
        auto block_it = it == D1_tree_.end() ? std::prev(D1_.end()) : it->second;

        // ⟨a, b⟩ is then added to the corresponding linked list in O(1) time
        block_it->elems_.emplace_back(a, b);
        present_[a->id_] = true;

        // update upper bound
        if (b > block_it->upper_) {
            // erase old tree entry
            const double old_upper = block_it->upper_;
            const auto tree_it = D1_tree_.find(old_upper);
            if (tree_it != D1_tree_.end() && tree_it->second == block_it) {
                D1_tree_.erase(tree_it);
            }
            // refresh upper bound
            block_it->upper_ = b;
            // insert new
            D1_tree_.emplace(block_it->upper_, block_it);
        }

        // save key pos
        const auto elem_it = std::prev(block_it->elems_.end());
        key_poses_[a->id_] = {block_it, elem_it};
        if (block_it->elems_.size() > M_) {
            split(block_it);
        }
    }

    // Delete(a, b)
    void erase(const KeyPos& pos, const Vertex* key) {
        const auto block_it = pos.block_;
        const auto elem_it  = pos.elem_;
        // To delete the key/value pair ⟨a, b⟩, we remove it directly from the linked list
        block_it->elems_.erase(elem_it);
        present_[key->id_] = false;

        // if a block in D1 becomes empty after deletion, we need to remove its upper bound in the binary search tree
        if (block_it->elems_.empty()) {
            if (block_it->owner_ == BlockOwner::D1) {
                const auto tree_it = D1_tree_.find(block_it->upper_);
                if (tree_it != D1_tree_.end() && tree_it->second == block_it)
                    D1_tree_.erase(tree_it);
                D1_.erase(block_it);
            } else {
                D0_.erase(block_it);
            }
        }
    }

    void split(const std::list<Block>::iterator& block_it) {
        // First, we identify the median element within the block in O(M ) time
        std::vector<Pair*> temp;
        temp.reserve(block_it->elems_.size());
        for (auto& p : block_it->elems_)
            temp.push_back(&p);
        const auto mid = temp.begin() + temp.size() / 2;
        std::ranges::nth_element(temp, mid,
                                 [](const Pair* a, const Pair* b) { return a->value_ < b->value_; });

        const double median = (*mid)->value_;

        // partitioning the elements into two new blocks each with at most ⌈M/2⌉ elements
        Block left(median, BlockOwner::D1);
        Block right(block_it->upper_, BlockOwner::D1);

        // — elements smaller than the median are placed in the first block, while the rest are placed in the second.
        for (auto it = block_it->elems_.begin(); it != block_it->elems_.end(); ) {
            auto curr = it++;
            if (curr->value_ < median) {
                left.elems_.splice(left.elems_.end(), block_it->elems_, curr);
            } else {
                right.elems_.splice(right.elems_.end(), block_it->elems_, curr);
            }
        }

        /* After the split, we make the appropriate changes in the binary search tree of upper bounds
         * in O(max{1, log(N/M )}) time. */

        // remove old block from tree and list
        const auto old_upper = block_it->upper_;
        const auto tree_it = D1_tree_.find(old_upper);
        if (tree_it != D1_tree_.end() && tree_it->second == block_it) {
            D1_tree_.erase(tree_it);
        }

        // replace old block pos
        const auto pos = D1_.erase(block_it);

        // insert new blocks
        auto right_it = D1_.insert(pos, std::move(right));
        auto left_it = D1_.insert(pos, std::move(left));

        // update tree
        D1_tree_.emplace(left_it->upper_, left_it);
        D1_tree_.emplace(right_it->upper_, right_it);

        // update key-map
        for (auto elem_it = left_it->elems_.begin(); elem_it != left_it->elems_.end(); ++elem_it) {
            key_poses_[elem_it->key_->id_] = { left_it, elem_it };
        }
        for (auto elem_it = right_it->elems_.begin(); elem_it != right_it->elems_.end(); ++elem_it) {
            key_poses_[elem_it->key_->id_] = { right_it, elem_it };
        }
    }

    void batch_prepend(std::list<Pair>& batch, const double b_upper) {
        const size_t L = batch.size();

        // When L ≤ M , we simply create a new block for L and add it to the beginning of D0.
        if (L <= M_) {
            Block b{b_upper, BlockOwner::D0};
            b.elems_.splice(b.elems_.end(), batch);
            D0_.emplace_front(std::move(b));
            const auto block_it = D0_.begin();
            for (auto it = block_it->elems_.begin(); it != block_it->elems_.end(); ++it) {
                key_poses_[it->key_->id_] = { block_it, it };
            }
            return;
        }

        // Otherwise, we create O(L/M ) new blocks in the beginning of D0, each containing at most ⌈M/2⌉ element
        std::list<std::list<Pair>> work;
        work.push_back(std::move(batch));

        // We can achieve this by repeatedly taking medians which completes in O(L log(L/M )) time.
        const size_t target = (M_ + 1) / 2;
        while (!work.empty()) {
            auto curr = std::move(work.front());
            work.pop_front();

            if (curr.size() <= target) {
                Block b{b_upper, BlockOwner::D0};
                b.elems_.splice(b.elems_.end(), curr);
                D0_.emplace_front(std::move(b));
                const auto block_it = D0_.begin();
                for (auto it = block_it->elems_.begin(); it != block_it->elems_.end(); ++it) {
                    key_poses_[it->key_->id_] = { block_it, it };
                }
                continue;
            }

            // split at median
            const double median = find_median_value(curr);

            std::list<Pair> lower, upper;
            for (auto it = curr.begin(); it != curr.end(); ) {
                auto cur = it++;
                if (cur->value_ < median)
                    lower.splice(lower.end(), curr, cur);
                else
                    upper.splice(upper.end(), curr, cur);
            }

            work.push_front(std::move(upper));
            work.push_front(std::move(lower));
        }
    }

    /*
    Pull Return a subset S′ of keys where |S′| ≤ M associated with the smallest |S′| values and an upper
    bound x that separates S′ from the remaining values in the data structure, in amortized O(|S′|) time.
    Specifically, if there are no remaining values, x should be B. Otherwise, x should satisfy
    max(S′) < x ≤ min(D) where D is the set of elements in the data structure after the pull operation.
    */
    std::pair<std::vector<Pair>, double> pull() {
        // To retrieve the smallest M values from D0 ∪ D1, we collect a sufficient prefix of blocks from D0 and D1 separately
        std::vector<std::list<Block>::iterator> S0_blocks;
        std::vector<std::list<Block>::iterator> S1_blocks;

        size_t count0 = 0;
        size_t count1 = 0;
        /* That is, in D0 (D1) we start from the first block and stop collecting as long as we have collected all
         * the remaining elements or the number of collected elements in S′0 (S′1) has reached M*/
        for (auto it = D0_.begin(); it != D0_.end() && count0 < M_; ++it) {
            S0_blocks.push_back(it);
            count0 += it->elems_.size();
        }

        for (auto it = D1_tree_.begin(); it != D1_tree_.end() && count1 < M_; ++it) {
            S1_blocks.push_back(it->second);
            count1 += it->second->elems_.size();
        }
        /* if S′0 ∪ S′1 contains no more than M elements, it must contain all blocks in D0 ∪ D1,
         * so we return all elements in S′0 ∪ S′1 as S′ and set x to the upper bound B,*/
        if (count0 + count1 <= M_) {
            std::vector<Pair> S;
            for (const auto& block : S0_blocks) {
                S.insert(S.end(), block->elems_.begin(), block->elems_.end());
            }
            for (const auto& block : S1_blocks) {
                S.insert(S.end(), block->elems_.begin(), block->elems_.end());
            }
            for (const auto& [key, val] : S) {
                erase(key_poses_[key->id_], key);
            }
            return {S, B_upper_};
        }

        // Otherwise, we want to make |S′| = M, and because the block sizes are kept at most M, the collecting process takes O(M) time.
        std::vector<const Pair*> candidates;       // S'
        candidates.reserve(2 * M_);
        for (const auto& block : S0_blocks) {
            for (const auto& p : block->elems_) {
                candidates.push_back(&p);
            }
        }
        for (const auto& block : S1_blocks) {
            for (const auto& p : block->elems_) {
                candidates.push_back(&p);
            }
        }
        // Now we know the smallest M elements must be contained in S′0 ∪ S′1 and can be identified from S′0 ∪ S′1 as S′ in O(M) time.
        const auto m_th = candidates.begin() + M_;
        std::ranges::nth_element(candidates, m_th,
                                 [](const Pair* a, const Pair* b) { return a->value_ < b->value_; });
        
        std::vector<Pair> result;
        result.reserve(M_);
        for (size_t i = 0; i < M_; ++i) {
            result.push_back(*candidates[i]);
        }
        // Then we delete elements in S′ from D0 and D1, whose running time is amortized to insertion time
        for (size_t i = 0; i < M_; ++i) {
            const auto* element = candidates[i];
            erase(key_poses_[element->key_->id_], element->key_);
        }
        
        // Set returned value x to the smallest remaining value in D0 ∪ D1
        double x = std::numeric_limits<double>::infinity();
        for (size_t i = M_; i < candidates.size(); ++i)
            x = std::min(x, candidates[i]->value_);

        return {std::move(result), x};
    }

    [[nodiscard]] bool empty() const {
        return D0_.empty() && D1_.empty();
    }
};


#endif //ALGO_SEMINAR_BLOCK_LINKED_LIST_H
