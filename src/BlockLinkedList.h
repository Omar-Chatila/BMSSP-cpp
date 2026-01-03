#ifndef ALGO_SEMINAR_BLOCK_LINKED_LIST_H
#define ALGO_SEMINAR_BLOCK_LINKED_LIST_H

#include <list>
#include <map>

#include "Graph.h"


struct Pair {
    Vertex* key_;
    double value_;
};

struct Block {
    std::list<Pair> elems_;
    double upper_;

    explicit Block(const int B) : upper_(B) {}
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
    int M_;
    double B_upper_;

    // Initialize(M, B)
    DequeueBlocks(const int M, const int B) : M_(M), B_upper_(B) {
        D1_.emplace_back(B);
    }

    // Insert(a, b)
    void insert(Vertex* a, const double b) {
        // To insert a key/value pair ⟨a, b⟩, we first check the existence of its key a
        if (key_map_.contains(a)) {
            // If a already exists, we delete original pair ⟨a, b′⟩ and insert new pair ⟨a, b⟩ only when b < b′.
            auto& pos = key_map_[a];
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

    void split(std::list<Block>::iterator& block_it) {
        return;
    }


};


#endif //ALGO_SEMINAR_BLOCK_LINKED_LIST_H