/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_PATRICIA_TRIE_HPP__
#define LASERCAKE_PATRICIA_TRIE_HPP__

#include "numbers.hpp"


typedef int32_t num_bits_type;
typedef int32_t num_coordinates_type;
class trivial_monoid {
public:
  // Default-constructor is monoid identity operator.
  trivial_monoid() {}
  // operator+ is monoid binary operator.
  friend inline trivial_monoid operator+(trivial_monoid, trivial_monoid) { return trivial_monoid(); }
  friend inline bool operator==(trivial_monoid, trivial_monoid) { return true; }
};
template<typename T1, typename T2>
class monoid_pair {
public:
  T1 first;
  T2 second;
  // Explicitly initialize the members ("value-initialize" in standard)
  // so that ints can be used as monoids under addition.
  monoid_pair() : first(), second() {}
  monoid_pair(T1 const& a, T1 const& b) : first(a), second(b) {}
  friend inline monoid_pair operator+(monoid_pair const& a, monoid_pair const& b) {
    return monoid_pair(a.first + b.first, a.second + b.second);
  }
  friend inline bool operator==(monoid_pair const& a, monoid_pair const& b) {
    return a.first == b.first && a.second == b.second;
  }
};
class noop_deleter {
public:
  template<typename T>
  void operator()(T const*)const {}
};
class default_deleter {
public:
  template<typename T>
  void operator()(T const* ptr)const {
    sizeof(*ptr); // Prevent deleting incomplete types,
                  // so that we are sure to call the destructor.
    delete ptr;
  }
};

// Exception safety:
//
// Member deleters and destructors,
// and Coord move-assignment and move-construction,
// must be nothrow or else the tree may be left in an
// inconsistent state.
//
// If any monoid operation can throw, then the monoids may
// be left in an inconsistent state but the strong guarantee*
// holds for all aspects of the tree except the monoids.
// (*every operation completely succeeds, or has no effect
// and throws an exception).
//
// 'insert' can throw errors from node_allocator.
//
// 'insert' and 'erase' can throw errors from 'monoid'.
//
// The default-constructor can only throw errors from
//   Coord's and monoid's default-constructors.
//
// Non-mutating operations cannot throw exceptions.
struct default_pow2_radix_patricia_trie_traits {
  typedef trivial_monoid monoid;
  typedef default_deleter leaf_deleter;
  typedef lasercake_nice_allocator<int>::type node_allocator;
};

template<num_coordinates_type Dims, typename Coord, typename T, typename Traits = default_pow2_radix_patricia_trie_traits>
class pow2_radix_patricia_trie_node {
public:
  static const num_coordinates_type dimensions = Dims;
  static const size_t radix = size_t(1) << dimensions;
  static const num_bits_type coordinate_bits = sizeof(Coord)*8; //hmm
  typedef pow2_radix_patricia_trie_node node_type;
  typedef std::array<node_type, radix> sub_nodes_type;
  typedef std::array<Coord, dimensions> loc_type;
  typedef typename Traits::monoid monoid_type;
  typedef typename Traits::leaf_deleter leaf_deleter;
  typedef typename Traits::node_allocator::template rebind<sub_nodes_type>::other node_allocator;

  pow2_radix_patricia_trie_node() : ptr(), parent(), siblings(), size_exponent_in_each_dimension(coordinate_bits), monoid(), loc_min() {}
  void delete_ptr() {
    if(sub_nodes_type* sub_nodes_ptr = sub_nodes()) {
      sub_nodes_ptr->~sub_nodes_type();
      node_allocator().deallocate(sub_nodes_ptr, 1);
    }
    if(T* leaf_ptr = leaf()) {
      leaf_deleter()(leaf_ptr);
    }
    ptr = nullptr;
  }
  void move_initialize_from_except_userdata(pow2_radix_patricia_trie_node& other) BOOST_NOEXCEPT {
    assert(ptr == nullptr);
    ptr = other.ptr;
    other.ptr = nullptr;
    parent = other.parent;
    siblings = other.siblings;
    size_exponent_in_each_dimension = other.size_exponent_in_each_dimension;
    if (siblings) {
      siblings = static_cast<sub_nodes_type*>(this - (&other - &other.siblings[0]));
      if(parent) {
        assert(parent->ptr == other.siblings);
        parent->ptr = siblings;
      }
    }
    if (sub_nodes_type* children = this->sub_nodes()) {
      for (node_type& child : *children) {
        child.parent = this;
      }
    }
  }
  ~pow2_radix_patricia_trie_node() { delete_ptr(); }

  // TODO maybe implement copy-constructor that copies whole tree? or subtree?
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node const& other) = delete;
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node const& other) = delete;

  // monoid operations might conceivably throw, but destructors shouldn't
  // and Coord assignment shouldn't and primitive assignment can't.
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node&& other)
          : ptr(nullptr), monoid(std::move(other.monoid)), loc_min(std::move(other.loc_min)) {
    move_initialize_from_except_userdata(other);
  }
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node&& other) {
    monoid = std::move(other.monoid);
    loc_min = std::move(other.loc_min);
    delete_ptr();
    move_initialize_from_except_userdata(other);
  }

  void* ptr;
  node_type* parent; // nullptr for root node
  sub_nodes_type* siblings; // nullptr for root node
  num_bits_type size_exponent_in_each_dimension; // 0 for ptr-to-leaf node; CoordBits for root node
  monoid_type monoid;
  loc_type loc_min; // after monoid for the sake of the move-constructor
  // TODO monoid...
  //TODO:   tile tile_that_everything_here_is; //if count_of_non_interior_tiles_here_==0 (and is all uniform!?)
  //  ah that monoids by if same, same, otherwise UNSPECIFIED.

  //enum ptr_type { CHILD_IS_T, CHILD_IS_HIERARCHY_NODE };
  //ptr_type which_ptr_type()const { return size_exponent_in_each_dimension == 0; }

  // It is possible for the root node to also be a ptr-to-leaf node.
  bool is_root_node()const { return !parent; }
  bool points_to_leaf()const { return ptr && size_exponent_in_each_dimension == 0; }
  bool points_to_sub_nodes()const { return ptr && size_exponent_in_each_dimension != 0; }
  bool is_empty()const { return !ptr; }
  sub_nodes_type* sub_nodes() { return points_to_sub_nodes() ? static_cast<sub_nodes_type*>(ptr) : nullptr; }
  sub_nodes_type const* sub_nodes()const { return points_to_sub_nodes() ? static_cast<sub_nodes_type*>(ptr) : nullptr; }
  T* leaf() { return points_to_leaf() ? static_cast<T*>(ptr) : nullptr; }
  T const* leaf()const { return points_to_leaf() ? static_cast<T*>(ptr) : nullptr; }

  bool contains(loc_type const& loc)const {
    if (size_exponent_in_each_dimension == coordinate_bits) {
      // Avoid left-shifting by an invalidly large amount.
      return true;
    }
    const Coord mask = (~Coord(0) << size_exponent_in_each_dimension);
    for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
      if ((loc_min[dim] & mask) != (loc[dim] & mask)) { return false; }
    }
    return true;
  }
  static node_type const& child_matching(sub_nodes_type const& sub_nodes, num_bits_type size_exponent_in_each_dimension, loc_type const& loc) {
    size_t idx = 0;
    for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
      idx |= ((loc[dim] >> (size_exponent_in_each_dimension - 1) & 1) << ((Dims - 1) - dim));
    }
    return sub_nodes[idx];
  }
  static node_type& child_matching(sub_nodes_type& sub_nodes, num_bits_type size_exponent_in_each_dimension, loc_type const& loc) {
    return const_cast<node_type&>(child_matching(const_cast<sub_nodes_type const&>(sub_nodes), size_exponent_in_each_dimension, loc));
  }
  node_type const& child_matching(loc_type const& loc)const {
    assert(contains(loc));
    sub_nodes_type const* sub_nodes_ptr = sub_nodes();
    assert(sub_nodes_ptr);
    return child_matching(*sub_nodes_ptr, size_exponent_in_each_dimension, loc);
  }
  node_type& child_matching(loc_type const& loc) {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->child_matching(loc));
  }

  node_type const* ascend_(loc_type const& leaf_loc)const {
    node_type const* node = this;
    while(!node->contains(leaf_loc) && node->parent) {
      node = node->parent;
    }
    return node;
  }
  node_type* ascend_(loc_type const& leaf_loc) {
    return const_cast<node_type*>(const_cast<const node_type*>(this)->ascend_(leaf_loc));
  }

  // Insert can be called on any node in the tree;
  // it makes no difference which, except for efficiency
  // (passing a nearby node rather than the root node can
  // be faster; passing a far-away node can be slower).
  // TODO but monoid operations! - they need to propagate to the top.
  // also, it could have subtraction and then it would be a bit faster...hmm.
  // and how can we check whether it doesn't need any propagation...
  // == identity? is-a-boring-type(like nan)? adding-it-didn't-change-the-value-so-aboves-dont-need-changing, ah.
  void insert(loc_type const& leaf_loc, T* leaf_ptr, monoid_type leaf_monoid = monoid_type()) {
    // invariant: this 'node' variable changes but is never nullptr.
    node_type* node;
    try {
      node = this->find_node(leaf_loc);
      caller_error_if(node->points_to_leaf() && node->loc_min == leaf_loc, "Inserting a leaf in a location that's already in the tree");
      if (!node->is_empty()) {
        // That child's location was too specific (wrong) for us.
        sub_nodes_type* intermediate_nodes = node_allocator().allocate(1);
        if (!intermediate_nodes) {
          throw std::bad_alloc();
        }
        try {
          // nothrow except monoids
          new (intermediate_nodes) sub_nodes_type();
          for (node_type& intermediate_node : *intermediate_nodes) {
            intermediate_node.ptr = nullptr;
            intermediate_node.size_exponent_in_each_dimension = 0;
            intermediate_node.parent = node;
            intermediate_node.siblings = intermediate_nodes;
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
                      to_unsigned_type(node->loc_min[dim] ^ leaf_loc[dim]));
            if(shared_size_exponent < dim_shared_size_exponent) {
              shared_size_exponent = dim_shared_size_exponent;
            }
          }

          // assert is typically nothrow, and it's also okay
          // if it throws here.
          assert(shared_size_exponent < coordinate_bits);
          assert(shared_size_exponent > 0);

          // move node's contents to its new location
          new_location_for_node_original_contents =    // nothrow
              &child_matching(*intermediate_nodes, shared_size_exponent, node->loc_min);
          new_leaf_ptr_node =    // nothrow
              &child_matching(*intermediate_nodes, shared_size_exponent, leaf_loc);

          assert(new_location_for_node_original_contents != new_leaf_ptr_node);

          // Monoid ops may throw. Do the copy before anything else so that if
          // it throws, we won't be in a partial state and have destructors
          // mess things up.
          new_location_for_node_original_contents->monoid = node->monoid;
          new_location_for_node_original_contents->loc_min = node->loc_min;
          new_location_for_node_original_contents->size_exponent_in_each_dimension = node->size_exponent_in_each_dimension;

          // Compute shared coords here in case some Coord ops can throw.
          loc_type shared_loc_min;
          const Coord mask = (~Coord(0) << shared_size_exponent);
          for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
            shared_loc_min[dim] = node->loc_min[dim] & mask;
          }
          // If Coord move throws, we're in trouble, because we're moving
          // an array of them so some of node's coords could be overwritten
          // already and we have no reliable way to restore them without
          // nothrow move.  This is why we require nothrow Coord move.
          //
          // Nevertheless do this inside the try/catch so we at least
          // don't leak memory if it throws.
          node->loc_min = std::move(shared_loc_min);
        }
        catch(...) {
          intermediate_nodes->~sub_nodes_type();
          node_allocator().deallocate(intermediate_nodes, 1);
          throw;
        }

        // continue moving node's contents to its new location
        // nothrow
        new_location_for_node_original_contents->ptr = node->ptr;
        node->ptr = intermediate_nodes;
        node->size_exponent_in_each_dimension = shared_size_exponent;
        //node->parent remains the same
        //node->siblings remains the same
        //node->monoid remains the same (it will be updated later as one of the parents)

        // nothrow
        node = new_leaf_ptr_node;
      }

      assert(node->ptr == nullptr);
      node->size_exponent_in_each_dimension = 0;
      node->loc_min = leaf_loc;
    }
    catch(...) {
      leaf_deleter()(leaf_ptr);
      throw;
    }
    // nothrow
    node->ptr = leaf_ptr;

    // is this a time waste? if starting at the root,
    // and if not worrying about exceptions,
    // we could have updated them on the way down,
    // though the short-circuit wouldn't take effect then.
    node_type* parent = node->parent;
    while(parent) {
      // += ?
      monoid_type sum = parent->monoid + leaf_monoid;
      if (sum == parent->monoid) { break; }
      parent->monoid = std::move(sum);
      parent = parent->parent;
    }
    node->monoid = std::move(leaf_monoid);
  }

  node_type const* find_node(loc_type leaf_loc)const {
    node_type const* node = this->ascend_(leaf_loc);
    while (true) {
      if (!node->contains(leaf_loc)) { return node; }
      if (!node->points_to_sub_nodes()) { return node; }
      node = &node->child_matching(leaf_loc);
    }
  }
  node_type* find_node(loc_type leaf_loc) {
    return const_cast<node_type*>(const_cast<const node_type*>(this)->find_node(leaf_loc));
  }
  // returns nullptr for no leaf in the tree at that loc
  node_type const* find_leaf_node(loc_type leaf_loc)const {
    node_type const*const node = find_node(leaf_loc);
    if (node->points_to_leaf()) { return node; }
    else { return nullptr; }
  }
  node_type* find_leaf_node(loc_type leaf_loc) {
    return const_cast<node_type*>(const_cast<const node_type*>(this)->find_leaf_node(leaf_loc));
  }
  T const* find_leaf(loc_type leaf_loc)const {
    node_type const*const node = find_node(leaf_loc);
    if (T* leaf = node->leaf()) { return leaf; }
    else { return nullptr; }
  }
  T* find_leaf(loc_type leaf_loc) {
    return const_cast<T*>(const_cast<const node_type*>(this)->find_leaf(leaf_loc));
  }

  // erase returns true iff something was erased.
  bool erase(loc_type leaf_loc) {
    node_type*const node = find_node(leaf_loc);
    if (T* leaf = node->leaf()) {
      leaf_deleter()(leaf);
      node->ptr = nullptr;
      // also could keep immediate-children-counts explicitly in nodes...
      // TODO shorten tree where appropriate
      
      if (!(node->monoid == monoid_type())) {
        sub_nodes_type* siblings = node->siblings;
        node_type* parent = node->parent;

        node->monoid = monoid_type();
        while(parent) {
          // -= ?
          monoid_type sum = monoid_type();
          for (node_type& sibling : *siblings) {
            sum = sum + sibling.monoid;
          }
          if (sum == parent->monoid) { break; }
          parent->monoid = sum;

          siblings = parent->siblings;
          parent = parent->parent;
        }
      }
      return true;
    }
    else { return false; }
  }
};
//weak_ptr<node_type> ?

#endif
