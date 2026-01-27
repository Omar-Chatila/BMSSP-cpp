#ifndef ALGO_SEMINAR_BLOCK_LINKED_LIST_H
#define ALGO_SEMINAR_BLOCK_LINKED_LIST_H

#include <algorithm>
#include <cassert>
#include <limits>
#include <list>
#include <map>

#include "Graph.h"


struct Pair {
    const Vertex* key_;
    double value_;

    Pair(const Vertex* k, const double v) : key_(k), value_(v) {}

    bool operator<(const Pair& o) const noexcept {
        if (value_ != o.value_) return value_ < o.value_;
        return key_->id_ < o.key_->id_;
    }
};

enum class BlockOwner {D0, D1};

struct Block {
    std::vector<Pair> elems_;
    double upper_;
    BlockOwner owner_;
    size_t block_id_;

    Block(const double B, const size_t M, const BlockOwner owner, const size_t id) : upper_(B), owner_(owner), block_id_(id) {
        elems_.reserve(M);
    }
};

struct BlockRef {
    size_t block_id;
    BlockOwner owner;
    bool operator==(const BlockRef& other) const {
        return block_id == other.block_id && owner == other.owner;
    }

    bool operator<(const BlockRef& other) const {
        if (owner != other.owner) return owner < other.owner;
        return block_id < other.block_id;
    }
};

struct KeyPos {
    BlockRef block_ref;
    size_t elem_idx;
};

class DequeueBlocks {
    std::list<Block> D0_;  // Batch-prepend
    std::list<Block> D1_;  // Normal inserts

    // Mapping from block_id to list iterator
    std::unordered_map<size_t, std::list<Block>::iterator> D0_map_;
    std::unordered_map<size_t, std::list<Block>::iterator> D1_map_;

    std::multimap<double, BlockRef> D1_tree_;  // Red-Black Tree for D1 blocks

    // Key bookkeeping
    std::vector<KeyPos> key_poses_; // Vertex id to KeyPos
    std::vector<bool> present_;     // Vertex id to whether KeyPos exists

    // Params
    size_t M_;
    double B_upper_;
    size_t next_block_id_ = 0;

    // helper methods

    // get D0 or D1 depending on owner
    std::list<Block>& get_deque(const BlockOwner owner) {
        return owner == BlockOwner::D0 ? D0_ : D1_;
    }
    // get the corresponding map
    std::unordered_map<size_t, std::list<Block>::iterator>& get_map(const BlockOwner owner) {
        return owner == BlockOwner::D0 ? D0_map_ : D1_map_;
    }
    // get block from D0 or D1
    Block& get_block(const BlockRef& ref) {
        auto& map = get_map(ref.owner);
        return *(map[ref.block_id]);
    }
    // get corresponding list iterator pointing to Block from ref
    std::list<Block>::iterator get_block_iterator(const BlockRef& ref) {
        auto& map = get_map(ref.owner);
        return map[ref.block_id];
    }

    // remove Block from D1-tree
    void remove_from_D1_tree(const double upper, const BlockRef& ref) {
        const auto& [fst, snd] = D1_tree_.equal_range(upper);    // find subsequence [fst:snd] matching upper
        for (auto it = fst; it != snd; ++it) {
            if (it->second.block_id == ref.block_id && it->second.owner == ref.owner) {
                D1_tree_.erase(it);
                break;
            }
        }
    }

    BlockRef create_block(const double upper, const BlockOwner owner, std::vector<Pair>&& elems) {
        size_t id = next_block_id_++;
        std::list<Block>& deque = get_deque(owner);
        auto& map = get_map(owner);

        if (owner == BlockOwner::D1) {
            auto it = deque.begin();
            while (it != deque.end() && it->upper_ < upper) {
                ++it;
            }
            it = deque.insert(it, Block(upper, M_, owner, id));
            map[id] = it;
            D1_tree_.emplace(upper, BlockRef{id, owner});
        } else {
            // D0 - prepend
            deque.emplace_front(upper, M_, owner, id);
            map[id] = deque.begin();
        }

        Block& block = *map[id];
        if (!elems.empty()) {
            block.elems_ = std::move(elems);
            // Mark vertices as present
            for (const auto& p : block.elems_) {
                present_[p.key_->id_] = true;
            }
        }

        return {id, owner};
    }

    void delete_block(const BlockRef& ref) {
        std::list<Block>& deque = get_deque(ref.owner);
        auto& map = get_map(ref.owner);

        if (ref.owner == BlockOwner::D1) {
            // remove from D1 tree
            const auto block_it = map[ref.block_id];
            remove_from_D1_tree(block_it->upper_, ref);
        }
        // remove from deque and map
        deque.erase(map[ref.block_id]);
        map.erase(ref.block_id);
    }

    void update_key_pos_for_block(const BlockRef& ref) {
        const Block& block = get_block(ref);
        for (size_t i = 0 ; i < block.elems_.size(); ++i) {
            const Vertex* key = block.elems_[i].key_;
            key_poses_[key->id_] = KeyPos(ref, i);
        }
    }

    static Pair find_median_pair(std::vector<Pair>& elems) {
        assert(!elems.empty());
        const auto mid = elems.begin() + static_cast<long>(elems.size()) / 2;
        std::ranges::nth_element(elems, mid, [](const Pair& a, const Pair& b) { return a < b; });
        return *mid;
    }
    
    // Delete(a, b)
    void erase(const KeyPos& pos, const Vertex* key) {
        const BlockRef block_ref = pos.block_ref;
        const size_t elem_idx  = pos.elem_idx;
        // To delete the key/value pair ⟨a, b⟩, we remove it directly from the linked list
        Block& block = get_block(block_ref);
        // Fast removal: swap with last element
        if (elem_idx != block.elems_.size() - 1) {
            block.elems_[elem_idx] = block.elems_.back();

            // Update position of the moved element
            const Vertex* moved_key = block.elems_[elem_idx].key_;
            key_poses_[moved_key->id_].elem_idx = elem_idx;
        }
        // Remove last element
        block.elems_.pop_back();
        present_[key->id_] = false;

        // if a block in D1 becomes empty after deletion, we need to remove its upper bound in the binary search tree
        if (block.elems_.empty()) {
            delete_block(block_ref);
        }
    }

public:
    // Initialize(M, B)
    explicit DequeueBlocks(const size_t N, const size_t M, const double B) : M_(M), B_upper_(B) {
        key_poses_.resize(N);
        present_.resize(N, false);
        // Initialize D1 with a single empty block with upper bound B
        create_block(B, BlockOwner::D1, {});
    }

    // Insert(a, b)
    void insert(const Vertex* a, const double b) {
        const size_t id = a->id_;
        // To insert a key/value pair ⟨a, b⟩, we first check the existence of its key a
        if (present_[id]) {
            // If a already exists, we delete original pair ⟨a, b′⟩ and insert new pair ⟨a, b⟩ only when b < b′.
            const auto&[block_ref, elem_idx] = key_poses_[id];
            const double old_b = get_block(block_ref).elems_[elem_idx].value_;
            if (b >= old_b) return;
            erase(key_poses_[id], a);
        }
        // We first locate the appropriate block for it, which is the block with the smallest upper bound greater than or equal to b,
        const auto tree_it = D1_tree_.lower_bound(b);
        BlockRef block_ref;

        std::vector to_add = {Pair(a, b)};

        if (tree_it == D1_tree_.end()) {
            // Use last block in D1
            if (!D1_map_.empty()) {
                const auto& [upper, ref] = *D1_tree_.rbegin();
                block_ref = ref;
            } else {
                // Create new block if D1 is empty
                block_ref = create_block(b, BlockOwner::D1, std::move(to_add));
            }
        } else {
            block_ref = tree_it->second;
        }

        Block& block = get_block(block_ref);

        // ⟨a, b⟩ is then added to the corresponding linked list in O(1) time
        block.elems_.emplace_back(a, b);
        present_[id] = true;

        // update upper bound
        if (b > block.upper_) {
            // erase old tree entry
            remove_from_D1_tree(block.upper_, block_ref);
            // refresh upper bound
            block.upper_ = b;
            // insert new
            D1_tree_.emplace(block.upper_, block_ref);
        }

        // save key pos
        key_poses_[id] = KeyPos{block_ref, block.elems_.size() - 1};
        // split if block is too large
        if (block.elems_.size() > M_) split(block_ref);
    }

    void erase(const Vertex* v) {
        if (present_[v->id_]) {
            const auto pos = key_poses_[v->id_];
            erase(pos, v);
        }
    }

    void split(const BlockRef& ref) {
        Block& block = get_block(ref);

        // Find median pair
        const Pair median_pair = find_median_pair(block.elems_);

        // Create new blocks
        const BlockRef left_ref = create_block(median_pair.value_, BlockOwner::D1, {});
        const BlockRef right_ref = create_block(block.upper_, BlockOwner::D1, {});

        Block& left_block = get_block(left_ref);
        Block& right_block = get_block(right_ref);

        // Partition using Pair comparator
        for (auto& p : block.elems_) {
            if (p < median_pair) {
                left_block.elems_.push_back(p);
            } else {
                right_block.elems_.push_back(p);
            }
        }

        // Delete old block
        delete_block(ref);

        // Update key positions
        update_key_pos_for_block(left_ref);
        update_key_pos_for_block(right_ref);
    }
    void batch_prepend(std::vector<Pair>& batch, const double b_upper) {
        const size_t L = batch.size();

        if (L <= M_) {
            BlockRef ref = create_block(b_upper, BlockOwner::D0, std::move(batch));
            update_key_pos_for_block(ref);
            return;
        }

        std::list<std::vector<Pair>> work;
        work.emplace_back(std::move(batch));
        const size_t target = (M_ + 1) / 2;

        while (!work.empty()) {
            auto curr = std::move(work.front());
            work.pop_front();

            if (curr.size() <= target) {
                BlockRef ref = create_block(b_upper, BlockOwner::D0, std::move(curr));
                update_key_pos_for_block(ref);
                continue;
            }

            // Find median pair
            Pair median_pair = find_median_pair(curr);
            std::vector<Pair> lower, upper;

            // Partition using the Pair comparator (compares value then key)
            for (auto& p : curr) {
                if (p < median_pair) {
                    lower.push_back(std::move(p));
                } else {
                    upper.push_back(std::move(p));
                }
            }

            work.push_front(std::move(lower));
            work.push_front(std::move(upper));
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
        std::vector<BlockRef> S0_blocks, S1_blocks;
        size_t count0 = 0, count1 = 0;

        /* That is, in D0 (D1) we start from the first block and stop collecting as long as we have collected all
         * the remaining elements or the number of collected elements in S′0 (S′1) has reached M*/

        // D0: take first blocks until M elements
        for (auto it = D0_.begin(); it != D0_.end() && count0 < M_; ++it) {
            S0_blocks.push_back(BlockRef{it->block_id_, BlockOwner::D0});
            count0 += it->elems_.size();
        }

        // D1: take blocks from tree in sorted order until M elements
        for (auto it = D1_tree_.begin(); it != D1_tree_.end() && count1 < M_; ++it) {
            S1_blocks.push_back(it->second);
            count1 += get_block(it->second).elems_.size();
        }
        /* if S′0 ∪ S′1 contains no more than M elements, it must contain all blocks in D0 ∪ D1,
         * so we return all elements in S′0 ∪ S′1 as S′ and set x to the upper bound B,*/
        if (count0 + count1 <= M_) {
            std::vector<Pair> S;
            // Collect all elements: O(|S|)
            for (const auto& ref : S0_blocks) {
                Block& block = get_block(ref);
                S.insert(S.end(), block.elems_.begin(), block.elems_.end());
            }
            for (const auto& ref : S1_blocks) {
                Block& block = get_block(ref);
                S.insert(S.end(), block.elems_.begin(), block.elems_.end());
            }

            // Delete all collected blocks
            for (const auto& ref : S0_blocks) delete_block(ref);
            for (const auto& ref : S1_blocks) delete_block(ref);

            return {std::move(S), B_upper_};
        }

        // Otherwise, we want to make |S′| = M, and because the block sizes are kept at most M, the collecting process takes O(M) time.
        std::vector<const Pair*> candidates;
        candidates.reserve(count0 + count1);

        for (const auto& ref : S0_blocks) {
            Block& block = get_block(ref);
            for (const auto& p : block.elems_) {
                candidates.push_back(&p);
            }
        }
        for (const auto& ref : S1_blocks) {
            Block& block = get_block(ref);
            for (const auto& p : block.elems_) {
                candidates.push_back(&p);
            }
        }

        // Now we know the smallest M elements must be contained in S′0 ∪ S′1 and can be identified from S′0 ∪ S′1 as S′ in O(M) time.
        const auto m_th = candidates.begin() + static_cast<long>(M_);
        std::ranges::nth_element(candidates, m_th,
                                 [](const Pair* a, const Pair* b) { return a->value_ < b->value_; });


        double x = std::numeric_limits<double>::infinity();
        for (size_t i = M_; i < candidates.size(); ++i) {
            x = std::min(x, candidates[i]->value_);
        }

        std::vector<Pair> result;
        result.reserve(M_);

        for (size_t i = 0; i < M_; ++i) {
            const Pair* p = candidates[i];
            result.push_back(*p);

            // Then we delete elements in S′ from D0 and D1, whose running time is amortized to insertion time
            const size_t key_id = p->key_->id_;
            const auto& key_pos = key_poses_[key_id];
            Block& block = get_block(key_pos.block_ref);

            // Fast removal by swapping
            if (key_pos.elem_idx <= block.elems_.size() - 1) {
                block.elems_[key_pos.elem_idx] = block.elems_.back();
                const Vertex* moved_key = block.elems_[key_pos.elem_idx].key_;
                key_poses_[moved_key->id_].elem_idx = key_pos.elem_idx;
            }
            block.elems_.pop_back();
            present_[key_id] = false;

            // Delete block if empty
            if (block.elems_.empty()) {
                delete_block(key_pos.block_ref);
            }
        }
        // Set returned value x to the smallest remaining value in D0 ∪ D1
        return {std::move(result), x};
    }

    [[nodiscard]] bool empty() const {
        return D0_.empty() && D1_.empty();
    }

    size_t size() const {
        size_t count = 0;
        for (const bool p : present_) if (p) count++;
        return count;
    }

    // TEST HELPER
    // Test helper: Check if vertex is present
    bool contains(const Vertex* v) const {
        size_t id = v->id_;
        return id < present_.size() && present_[id];
    }

    // Test helper: Get number of blocks in D0 and D1
    std::pair<size_t, size_t> block_counts() const {
        return {D0_.size(), D1_.size()};
    }

    KeyPos get_key_position(size_t id) const {
        assert(id < key_poses_.size());
        return key_poses_[id];
    }
    bool is_present(size_t id) const {
        return id < present_.size() && present_[id];
    }

};


#endif //ALGO_SEMINAR_BLOCK_LINKED_LIST_H
