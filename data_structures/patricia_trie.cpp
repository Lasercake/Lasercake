/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "patricia_trie.hpp"


template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
inline void pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::insert(loc_type leaf_loc, T* leaf_ptr, monoid_type leaf_monoid) {
  // invariant: this 'node' variable changes but is never nullptr.
  node_type* node_to_initialize;
  try {
    node_type*const node = &this->find_node(leaf_loc);
    caller_error_if(node->points_to_leaf() && node->box_.min_ == leaf_loc, "Inserting a leaf in a location that's already in the tree");
    if (node->is_empty()) {
      node_to_initialize = node;
      node_to_initialize->initialize_monoid_(std::move(leaf_monoid));
      //LOG << "Type 1 " << std::hex << size_t(node_to_initialize) << std::dec << "\n";
    }
    else {
      // That child's location was too specific (wrong) for us.
      sub_nodes_type* intermediate_nodes = node_allocator().allocate(1);
      if (!intermediate_nodes) {
        throw std::bad_alloc();
      }
      try {
        // nothrow except monoids
        new (intermediate_nodes) sub_nodes_type();
        for (node_type& intermediate_node : *intermediate_nodes) {
          intermediate_node.parent_ = node;
          assert(intermediate_node.box_.size_exponent_in_each_dimension_ == 0);
        }
      }
      catch(...) {
        node_allocator().deallocate(intermediate_nodes, 1);
        throw;
      }

      num_bits_type shared_size_exponent;
      node_type* new_location_for_node_original_contents;
      node_type* new_leaf_ptr_node;
      try {
        // loop is nothrow
        shared_size_exponent = 0;
        for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
          const num_bits_type dim_shared_size_exponent =
                  num_bits_in_integer_that_are_not_leading_zeroes(
                    to_unsigned_type(node->box_.min_[dim] ^ leaf_loc[dim]));
          if(shared_size_exponent < dim_shared_size_exponent) {
            shared_size_exponent = dim_shared_size_exponent;
          }
        }

        // assert is typically nothrow, and it's also okay
        // if it throws here.
        assert(shared_size_exponent > 0);
        //LOG << "~~" << shared_size_exponent << std::endl;

        // move node's contents to its new location
        new_location_for_node_original_contents =    // nothrow
            &child_matching(*intermediate_nodes, shared_size_exponent, node->box_.min_);
        new_leaf_ptr_node =    // nothrow
            &child_matching(*intermediate_nodes, shared_size_exponent, leaf_loc);

        assert(new_location_for_node_original_contents != new_leaf_ptr_node);

        // Monoid ops may throw. Do the copy before anything else so that if
        // it throws, we won't be in a partial state and have destructors
        // mess things up.
        new_location_for_node_original_contents->monoid_ = node->monoid_;
        new_location_for_node_original_contents->box_.min_ = node->box_.min_;
        new_location_for_node_original_contents->box_.size_exponent_in_each_dimension_ = node->box_.size_exponent_in_each_dimension_;

        // Update monoids.  If they throw, insert() will still
        // be a no-op except for monoid inconsistency
        //
        // is this impl a time waste? if starting at the root,
        // and if not worrying about exceptions,
        // we could have updated them on the way down,
        // though the short-circuit wouldn't take effect then.
        assert(new_leaf_ptr_node->parent_);
        assert(new_leaf_ptr_node->parent_ == node);
        new_leaf_ptr_node->initialize_monoid_(std::move(leaf_monoid));

        // Compute shared coords here in case some Coord ops can throw.
        loc_type shared_loc_min;
        const Coord mask = safe_left_shift(~Coord(0), shared_size_exponent);
        for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
          shared_loc_min[dim] = node->box_.min_[dim] & mask;
        }
        // If Coord move throws, we're in trouble, because we're moving
        // an array of them so some of node's coords could be overwritten
        // already and we have no reliable way to restore them without
        // nothrow move.  This is why we require nothrow Coord move.
        //
        // Nevertheless do this inside the try/catch so we at least
        // don't leak memory if it throws.
        node->box_.min_ = std::move(shared_loc_min);
      }
      catch(...) {
        intermediate_nodes->~sub_nodes_type();
        node_allocator().deallocate(intermediate_nodes, 1);
        throw;
      }

      // continue moving node's contents to its new location
      // nothrow
      new_location_for_node_original_contents->ptr_ = node->ptr_;
      if(sub_nodes_type* original_sub_nodes = new_location_for_node_original_contents->sub_nodes()) {
        for (node_type& sub_node : *original_sub_nodes) {
          sub_node.parent_ = new_location_for_node_original_contents;
        }
      }
      node->ptr_ = intermediate_nodes;
      node->box_.size_exponent_in_each_dimension_ = shared_size_exponent;
      //node->parent remains the same
      //node->monoid remains the same (it will be updated later as one of the parents)

      // nothrow
      node_to_initialize = new_leaf_ptr_node;
      //LOG << "Type 2 " << std::hex << size_t(node_to_initialize) << std::dec << "\n";
    }

    assert(node_to_initialize->ptr_ == nullptr);
    node_to_initialize->box_.min_ = std::move(leaf_loc);
  }
  catch(...) {
    leaf_deleter()(leaf_ptr);
    throw;
  }
  // nothrow; commits to deleting (using the deleter on) leaf_ptr
  // in node's destructor.
  node_to_initialize->box_.size_exponent_in_each_dimension_ = 0;
  node_to_initialize->ptr_ = leaf_ptr;
}


template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
inline bool pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::erase(loc_type leaf_loc) {
  node_type& node = find_node(leaf_loc);
  if (T* leaf = node.leaf()) {
    node.update_monoid(monoid_type());
    leaf_deleter()(leaf);
    node.ptr_ = nullptr;
    // TODO shorten tree where appropriate
    // (Could keep immediate-children-counts explicitly in nodes
    // to make that a little faster; probably fine either way.)
    return true;
  }
  else { return false; }
}


#include "../world.hpp"
template class pow2_radix_patricia_trie_node<3,
  tile_coordinate,
  the_decomposition_of_the_world_into_blocks_impl::worldblock,
  the_decomposition_of_the_world_into_blocks_impl::worldblock_trie_traits>;

#include "../tests/patricia_trie_tests.hpp"
template class pow2_radix_patricia_trie_node<3,
  patricia_trie_testing::coord,
  patricia_trie_testing::block,
  patricia_trie_testing::trie_traits>;
