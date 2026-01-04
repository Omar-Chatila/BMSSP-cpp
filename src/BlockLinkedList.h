#ifndef ALGO_SEMINAR_BLOCK_LINKED_LIST_H
#define ALGO_SEMINAR_BLOCK_LINKED_LIST_H

#include <algorithm>
#include <list>
#include <map>

#include "Graph.h"
#include "Utils.h"


struct Pair {
    Vertex* key_;
    double value_;

    bool operator<(const Pair& o) const {
        return this->value_ < o.value_;
    }
};

struct Block {
    std::list<Pair> elems_;
    double upper_;

    explicit Block(const double B) : upper_(B) {}
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
    std::unordered_map<Vertex*, KeyPos> key_map_;

    // Params
    size_t M_;
    double B_upper_;

    // Initialize(M, B)
    DequeueBlocks(const size_t M, const double B) : M_(M), B_upper_(B) {
        D1_.emplace_back(B);
    }

    // Insert(a, b)
    void insert(Vertex* a, const double b) {
        // To insert a key/value pair ⟨a, b⟩, we first check the existence of its key a
        if (key_map_.contains(a)) {
            // If a already exists, we delete original pair ⟨a, b′⟩ and insert new pair ⟨a, b⟩ only when b < b′.
            const auto& pos = key_map_[a];
            const double old_b = pos.elem_->value_;
            if (b >= old_b) return;
            erase(pos, a);
        }
        // We first locate the appropriate block for it, which is the block with the smallest upper bound greater than or equal to b,
        const auto it = D1_tree_.lower_bound(b);
        auto block_it = it->second;

        // ⟨a, b⟩ is then added to the corresponding linked list in O(1) time
        block_it->elems_.push_back({a, b});

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
        key_map_[a] = {block_it, elem_it};
        if (block_it->elems_.size() > M_) {
            split(block_it);
        }
    }

    // Delete(a, b)
    void erase(const KeyPos& pos, Vertex* key) {
        const auto block_it = pos.block_;
        const auto elem_it  = pos.elem_;
        // To delete the key/value pair ⟨a, b⟩, we remove it directly from the linked list
        block_it->elems_.erase(elem_it);
        key_map_.erase(key);

        // if a block in D1 becomes empty after deletion, we need to remove its upper bound in the binary search tree
        if (block_it->elems_.empty()) {
            const auto tree_it = D1_tree_.find(block_it->upper_);
            if (tree_it != D1_tree_.end() && tree_it->second == block_it)
                D1_tree_.erase(tree_it);
            D1_.erase(block_it);
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

        double median = (*mid)->value_;

        // partitioning the elements into two new blocks each with at most ⌈M/2⌉ elements
        Block left(median);
        Block right(block_it->upper_);

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
            key_map_[elem_it->key_] = { left_it, elem_it };
        }
        for (auto elem_it = right_it->elems_.begin(); elem_it != right_it->elems_.end(); ++elem_it) {
            key_map_[elem_it->key_] = { right_it, elem_it };
        }
    }

    double find_median_value(const std::list<Pair>& elems) {
        std::vector<double> vals;
        vals.reserve(elems.size());
        for (auto& p : elems)
            vals.push_back(p.value_);

        const auto mid = vals.begin() + vals.size() / 2;
        std::ranges::nth_element(vals, mid);
        return *mid;
    }

    void batch_prepend(std::list<Pair>& batch, const double b_upper) {
        const size_t L = batch.size();

        // hen L ≤ M , we simply create a new block for L and add it to the beginning of D0.
        if (L <= M_) {
            Block b{b_upper};
            b.elems_.splice(b.elems_.end(), batch);
            D0_.emplace_front(std::move(b));
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
                Block b{b_upper};
                b.elems_.splice(b.elems_.end(), curr);
                D0_.emplace_front(std::move(b));
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

};


#endif //ALGO_SEMINAR_BLOCK_LINKED_LIST_H