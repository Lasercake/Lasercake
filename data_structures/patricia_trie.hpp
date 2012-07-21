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
//   If it does, insert() had no side-effects.
//
// 'insert' and 'erase' can throw errors from 'monoid'.
//   If they do, the tree's monoids may be incorrect
//   but there are no other side-effects.
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

  pow2_radix_patricia_trie_node() : ptr_(), parent_(), siblings_(), size_exponent_in_each_dimension_(coordinate_bits), monoid_(), loc_min_() {}
  ~pow2_radix_patricia_trie_node() { delete_ptr_(); }

  // TODO maybe implement copy-constructor that copies whole tree? or subtree?
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node const& other) = delete;
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node const& other) = delete;

  // monoid operations might conceivably throw, but destructors shouldn't
  // and Coord assignment shouldn't and primitive assignment can't.
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node&& other)
          : ptr_(nullptr), monoid_(std::move(other.monoid)), loc_min_(std::move(other.loc_min)) {
    move_initialize_from_except_userdata_(other);
  }
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node&& other) {
    monoid_ = std::move(other.monoid_);
    loc_min_ = std::move(other.loc_min_);
    delete_ptr_();
    move_initialize_from_except_userdata_(other);
  }

  // It is possible for the root node to also be a ptr-to-leaf node.
  bool is_root_node()const { return !parent_; }
  bool points_to_leaf()const { return ptr_ && size_exponent_in_each_dimension_ == 0; }
  bool points_to_sub_nodes()const { return ptr_ && size_exponent_in_each_dimension_ != 0; }
  bool is_empty()const { return !ptr_; }
  sub_nodes_type* sub_nodes() { return points_to_sub_nodes() ? static_cast<sub_nodes_type*>(ptr_) : nullptr; }
  sub_nodes_type const* sub_nodes()const { return points_to_sub_nodes() ? static_cast<sub_nodes_type*>(ptr_) : nullptr; }
  T* leaf() { return points_to_leaf() ? static_cast<T*>(ptr_) : nullptr; }
  T const* leaf()const { return points_to_leaf() ? static_cast<T*>(ptr_) : nullptr; }

  bool contains(loc_type const& loc)const {
    if (size_exponent_in_each_dimension_ == coordinate_bits) {
      // Avoid left-shifting by an invalidly large amount.
      return true;
    }
    const Coord mask = (~Coord(0) << size_exponent_in_each_dimension_);
    for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
      if ((loc_min_[dim] & mask) != (loc[dim] & mask)) { return false; }
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
    return child_matching(*sub_nodes_ptr, size_exponent_in_each_dimension_, loc);
  }
  node_type& child_matching(loc_type const& loc) {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->child_matching(loc));
  }

  // Insert can be called on any node in the tree;
  // it makes no difference which, except for efficiency
  // (passing a nearby node rather than the root node can
  // be faster; passing a far-away node can be slower).
  // TODO but monoid operations! - they need to propagate to the top.
  // also, it could have subtraction and then it would be a bit faster...hmm.
  // and how can we check whether it doesn't need any propagation...
  // == identity? is-a-boring-type(like nan)? adding-it-didn't-change-the-value-so-aboves-dont-need-changing, ah.
  void insert(loc_type leaf_loc, T* leaf_ptr, monoid_type leaf_monoid = monoid_type());

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
  bool erase(loc_type leaf_loc);

  monoid_type const& monoid()const { return monoid_; }
  void update_monoid(monoid_type new_leaf_monoid) {
    caller_correct_if(this->points_to_leaf(),
        "patricia_trie: Only leaves can have their monoids explicitly set; "
        "parent and empty node monoids are automatically computed.");
    // If monoid_type had a -=, this could be faster.
    if (!(monoid_ == new_leaf_monoid)) {
      sub_nodes_type* siblings = siblings_;
      node_type* parent = parent_;

      monoid_ = std::move(new_leaf_monoid);
      while(parent) {
        monoid_type sum = monoid_type();
        for (node_type& sibling : *siblings) {
          sum = sum + sibling.monoid_;
        }
        if (sum == parent->monoid_) { break; }
        parent->monoid_ = std::move(sum);

        siblings = parent->siblings_;
        parent = parent->parent_;
      }
    }
  }

private:
  node_type const* ascend_(loc_type const& leaf_loc)const {
    node_type const* node = this;
    while(!node->contains(leaf_loc) && node->parent_) {
      node = node->parent_;
    }
    return node;
  }
  node_type* ascend_(loc_type const& leaf_loc) {
    return const_cast<node_type*>(const_cast<const node_type*>(this)->ascend_(leaf_loc));
  }

  void delete_ptr_() {
    if(sub_nodes_type* sub_nodes_ptr = sub_nodes()) {
      sub_nodes_ptr->~sub_nodes_type();
      node_allocator().deallocate(sub_nodes_ptr, 1);
    }
    if(T* leaf_ptr = leaf()) {
      leaf_deleter()(leaf_ptr);
    }
    ptr_ = nullptr;
  }
  void move_initialize_from_except_userdata_(pow2_radix_patricia_trie_node& other) BOOST_NOEXCEPT {
    assert(ptr_ == nullptr);
    ptr_ = other.ptr_;
    other.ptr_ = nullptr;
    parent_ = other.parent_;
    siblings_ = other.siblings_;
    size_exponent_in_each_dimension_ = other.size_exponent_in_each_dimension_;
    if (siblings_) {
      siblings_ = static_cast<sub_nodes_type*>(this - (&other - &other.siblings_[0]));
      if(parent_) {
        assert(parent_->ptr == other.siblings_);
        parent_->ptr = siblings_;
      }
    }
    if (sub_nodes_type* children = this->sub_nodes()) {
      for (node_type& child : *children) {
        child.parent = this;
      }
    }
  }

  void* ptr_;
  node_type* parent_; // nullptr for root node
  sub_nodes_type* siblings_; // nullptr for root node
  num_bits_type size_exponent_in_each_dimension_; // 0 for ptr-to-leaf node; CoordBits for root node
  monoid_type monoid_;
  loc_type loc_min_; // after monoid for the sake of the move-constructor
};


template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
inline void pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::insert(loc_type leaf_loc, T* leaf_ptr, monoid_type leaf_monoid) {
  // invariant: this 'node' variable changes but is never nullptr.
  node_type* node_to_initialize;
  try {
    node_type*const node = this->find_node(leaf_loc);
    caller_error_if(node->points_to_leaf() && node->loc_min_ == leaf_loc, "Inserting a leaf in a location that's already in the tree");
    if (node->is_empty()) {
      node_to_initialize = node;
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
          intermediate_node.ptr_ = nullptr;
          intermediate_node.size_exponent_in_each_dimension_ = 0;
          intermediate_node.parent_ = node;
          intermediate_node.siblings_ = intermediate_nodes;
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
                    to_unsigned_type(node->loc_min_[dim] ^ leaf_loc[dim]));
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
            &child_matching(*intermediate_nodes, shared_size_exponent, node->loc_min_);
        new_leaf_ptr_node =    // nothrow
            &child_matching(*intermediate_nodes, shared_size_exponent, leaf_loc);

        assert(new_location_for_node_original_contents != new_leaf_ptr_node);

        // Monoid ops may throw. Do the copy before anything else so that if
        // it throws, we won't be in a partial state and have destructors
        // mess things up.
        new_location_for_node_original_contents->monoid_ = node->monoid_;
        new_location_for_node_original_contents->loc_min_ = node->loc_min_;
        new_location_for_node_original_contents->size_exponent_in_each_dimension_ = node->size_exponent_in_each_dimension_;

        // Update monoids.  If they throw, insert() will still
        // be a no-op except for monoid inconsistency
        //
        // is this impl a time waste? if starting at the root,
        // and if not worrying about exceptions,
        // we could have updated them on the way down,
        // though the short-circuit wouldn't take effect then.
        node_type* parent = node;
        while(parent) {
          // += ?
          monoid_type sum = parent->monoid_ + leaf_monoid;
          if (sum == parent->monoid_) { break; }
          parent->monoid_ = std::move(sum);
          parent = parent->parent_;
        }
        new_leaf_ptr_node->monoid_ = std::move(leaf_monoid);

        // Compute shared coords here in case some Coord ops can throw.
        loc_type shared_loc_min;
        const Coord mask = (~Coord(0) << shared_size_exponent);
        for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
          shared_loc_min[dim] = node->loc_min_[dim] & mask;
        }
        // If Coord move throws, we're in trouble, because we're moving
        // an array of them so some of node's coords could be overwritten
        // already and we have no reliable way to restore them without
        // nothrow move.  This is why we require nothrow Coord move.
        //
        // Nevertheless do this inside the try/catch so we at least
        // don't leak memory if it throws.
        node->loc_min_ = std::move(shared_loc_min);
      }
      catch(...) {
        intermediate_nodes->~sub_nodes_type();
        node_allocator().deallocate(intermediate_nodes, 1);
        throw;
      }

      // continue moving node's contents to its new location
      // nothrow
      new_location_for_node_original_contents->ptr_ = node->ptr_;
      node->ptr_ = intermediate_nodes;
      node->size_exponent_in_each_dimension_ = shared_size_exponent;
      //node->parent remains the same
      //node->siblings remains the same
      //node->monoid remains the same (it will be updated later as one of the parents)

      // nothrow
      node_to_initialize = new_leaf_ptr_node;
    }

    assert(node_to_initialize->ptr_ == nullptr);
    node_to_initialize->loc_min_ = std::move(leaf_loc);
  }
  catch(...) {
    leaf_deleter()(leaf_ptr);
    throw;
  }
  // nothrow; commits to deleting (using the deleter on) leaf_ptr
  // in node's destructor.
  node_to_initialize->size_exponent_in_each_dimension_ = 0;
  node_to_initialize->ptr_ = leaf_ptr;
}


template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
inline bool pow2_radix_patricia_trie_node<Dims, Coord, T, Traits>::erase(loc_type leaf_loc) {
  node_type*const node = find_node(leaf_loc);
  if (T* leaf = node->leaf()) {
    node->update_monoid(monoid_type());
    leaf_deleter()(leaf);
    node->ptr_ = nullptr;
    // also could keep immediate-children-counts explicitly in nodes...
    // TODO shorten tree where appropriate
    return true;
  }
  else { return false; }
}


#endif
