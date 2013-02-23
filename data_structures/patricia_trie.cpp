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
pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>&
pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::insert(loc_type leaf_loc, T&& new_leaf, monoid_type leaf_monoid) {
  // invariant: this 'node' variable changes but is never nullptr.
  node_type* node_to_initialize;
  node_type*const node = &this->find_node(leaf_loc);
  caller_error_if(node->points_to_leaf() && node->min() == leaf_loc, "Inserting a leaf in a location that's already in the tree");
  if (node->is_empty()) {
    node_to_initialize = node;
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
        intermediate_node.set_parent(node);
        assert(intermediate_node.size_exponent_in_each_dimension() == 0);
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
                  to_unsigned_type(node->min()[dim] ^ leaf_loc[dim]));
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
          &child_matching(*intermediate_nodes, shared_size_exponent, node->min());
      new_leaf_ptr_node =    // nothrow
          &child_matching(*intermediate_nodes, shared_size_exponent, leaf_loc);

      assert(new_location_for_node_original_contents != new_leaf_ptr_node);

      // Monoid ops may throw. Do the copy before anything else so that if
      // it throws, we won't be in a partial state and have destructors
      // mess things up.
      new_location_for_node_original_contents->set_monoid(node->monoid());
      new_location_for_node_original_contents->set_min(node->min());
      new_location_for_node_original_contents->set_size_exponent_in_each_dimension(node->size_exponent_in_each_dimension());
      new_location_for_node_original_contents->set_stable_node_reference_keeper(node->stable_node_reference_keeper());

      // Compute shared coords here in case some Coord ops can throw.
      loc_type shared_loc_min;
      const Coord mask = safe_left_shift(~Coord(0), shared_size_exponent);
      for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
        shared_loc_min[dim] = node->min()[dim] & mask;
      }
      // If Coord move throws, we're in trouble, because we're moving
      // an array of them so some of node's coords could be overwritten
      // already and we have no reliable way to restore them without
      // nothrow move.  This is why we require nothrow Coord move.
      //
      // Nevertheless do this inside the try/catch so we at least
      // don't leak memory if it throws.
      node->set_min(std::move(shared_loc_min));
    }
    catch(...) {
      intermediate_nodes->~sub_nodes_type();
      node_allocator().deallocate(intermediate_nodes, 1);
      throw;
    }

    // continue moving node's contents to its new location
    // nothrow
    new_location_for_node_original_contents->set_sub_nodes(node->sub_nodes());
    new_location_for_node_original_contents->set_leaf(node->move_leaf());
    if(sub_nodes_type* original_sub_nodes = new_location_for_node_original_contents->sub_nodes()) {
      for (node_type& sub_node : *original_sub_nodes) {
        sub_node.set_parent(new_location_for_node_original_contents);
      }
    }
    node->set_size_exponent_in_each_dimension(shared_size_exponent);
    node->set_leaf();
    node->set_sub_nodes(intermediate_nodes);
    node->set_stable_node_reference_keeper();
    //node->parent remains the same
    //node->monoid remains the same (it will be updated later as one of the parents)

    // nothrow
    node_to_initialize = new_leaf_ptr_node;
    //LOG << "Type 2 " << std::hex << size_t(node_to_initialize) << std::dec << "\n";
  }

  assert(!node_to_initialize->sub_nodes());
  node_to_initialize->set_min(std::move(leaf_loc));

  // hopefully nothrow
  node_to_initialize->set_leaf(std::move(new_leaf));
  // hopefully nothrow
  node_to_initialize->initialize_monoid_(std::move(leaf_monoid));

  return *node_to_initialize;
}


template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
bool pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::erase(loc_type leaf_loc) {
  node_type& node = find_node(leaf_loc);
  if (node.points_to_leaf()) {
    node.update_monoid(monoid_type());
    node.set_leaf();
    // TODO shorten tree where appropriate
    // (Could keep immediate-children-counts explicitly in nodes
    // to make that a little faster; probably fine either way.)

    delete_empty_leaves_(node);
    // Now node (which might be *this) might no longer exist.
    return true;
  }
  else { return false; }
}
template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
bool pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::erase_if_empty() {
  node_type& node_that_exists = delete_empty_leaves_(*this);
  return (&node_that_exists != this);
}

template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>&
pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::delete_empty_leaves_(node_type& node1) {
  node_type* node = &node1;
  while(true) {
    sub_nodes_type* siblings = node->siblings();
    if(!siblings) { break; }
    bool siblings_all_empty = true;
    for(node_type& sibling : *siblings) {
      if(!sibling.is_empty()) { siblings_all_empty = false; }
    }
    if(siblings_all_empty) {
      node_type* parent = node->parent();
      for(node_type& sibling : *siblings) {
        if(auto sibling_keeper = sibling.stable_node_reference_keeper()) {
          if(auto parent_keeper = parent->stable_node_reference_keeper()) {
            sibling_keeper->look_at_this_one_instead_ = parent_keeper;
            sibling_keeper->here_ = nullptr;
          }
          else {
            sibling_keeper->here_ = parent;
            parent->set_stable_node_reference_keeper(sibling_keeper);
          }
        }
      }
      parent->delete_sub_nodes_();
      node = parent;
    }
    else {
      break;
    }
  }
  return *node;
}



#include "../world.hpp"
template class pow2_radix_patricia_trie_node<3,
  tile_coordinate,
  the_decomposition_of_the_world_into_blocks_impl::region_specification,
  the_decomposition_of_the_world_into_blocks_impl::worldblock_trie_traits>;

#include "../tests/patricia_trie_tests.hpp"
template class pow2_radix_patricia_trie_node<3,
  patricia_trie_testing::coord,
  std::unique_ptr<patricia_trie_testing::block>,
  patricia_trie_testing::trie_traits>;
