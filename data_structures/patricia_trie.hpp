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

#ifndef LASERCAKE_PATRICIA_TRIE_HPP__
#define LASERCAKE_PATRICIA_TRIE_HPP__

#include <ostream>
#include <array>

#include "numbers.hpp"
#include "bounds_checked_int.hpp"
#include "../utils.hpp" //for lasercake_nice_allocator


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
    (void)sizeof(*ptr); // Prevent deleting incomplete types,
                        // so that we are sure to call the destructor.
    delete ptr;
  }
};

struct patricia_trie_tester;

template<num_coordinates_type Dims, typename Coord, typename T, typename Traits>
class pow2_radix_patricia_trie_node;

template<num_coordinates_type Dims, typename Coord>
class power_of_two_bounding_cube {
public:
  typedef std::array<Coord, Dims> loc_type;
  power_of_two_bounding_cube() : min_(), size_exponent_in_each_dimension_() {}
  power_of_two_bounding_cube(loc_type const& min, num_bits_type size_exponent_in_each_dimension)
   : min_(min), size_exponent_in_each_dimension_(size_exponent_in_each_dimension) {}
  power_of_two_bounding_cube(loc_type&& min, num_bits_type size_exponent_in_each_dimension)
   : min_(std::move(min)), size_exponent_in_each_dimension_(size_exponent_in_each_dimension) {}

  power_of_two_bounding_cube(power_of_two_bounding_cube const& other) = default;
  power_of_two_bounding_cube& operator=(power_of_two_bounding_cube const& other) = default;
  power_of_two_bounding_cube(power_of_two_bounding_cube&& other) = default;
  power_of_two_bounding_cube& operator=(power_of_two_bounding_cube&& other) = default;

  num_bits_type size_exponent_in_each_dimension()const { return size_exponent_in_each_dimension_; }

  Coord min(num_coordinates_type dim)const { return min_[dim]; }
  Coord size_minus_one(num_coordinates_type)const { return ~(safe_left_shift(~Coord(0), size_exponent_in_each_dimension_)); }
  Coord size(num_coordinates_type)const { return safe_left_shift(Coord(1), size_exponent_in_each_dimension_); }
  Coord max(num_coordinates_type dim)const { return min_[dim] + size_minus_one(dim); }
  loc_type min()const { return min_; }
  loc_type max()const {
    loc_type result;
    for(num_coordinates_type dim = 0; dim != Dims; ++dim) { result[dim] = max(dim); }
    return result;
  }
  loc_type size_minus_one()const {
    loc_type result;
    for(num_coordinates_type dim = 0; dim != Dims; ++dim) { result[dim] = size_minus_one(dim); }
    return result;
  }
  loc_type size()const {
    loc_type result;
    for(num_coordinates_type dim = 0; dim != Dims; ++dim) { result[dim] = size(dim); }
    return result;
  }

  friend inline std::ostream& operator<<(std::ostream& os, power_of_two_bounding_cube const& bb) {
    os << '[';
    for(num_coordinates_type dim = 0; dim != Dims; ++dim) { os << bb.min(dim) << ", "; }
    return os << " @ " << bb.size_exponent_in_each_dimension() << ']';
  }

private:
  // Friend patricia_trie so that it can be more paranoid about
  // exception-safety, specifically the order in which the data members
  // are changed in move-assignment.
  template<num_coordinates_type Dims2, typename Coord2, typename T, typename Traits>
  friend class pow2_radix_patricia_trie_node;

  loc_type min_;
  // 0 for ptr-to-leaf node, nonzero for ptr-to-sub-nodes nodes,
  // doesn't matter for nullptr nodes:
  num_bits_type size_exponent_in_each_dimension_;
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
  typedef pow2_radix_patricia_trie_node node_type;
  typedef std::array<node_type, radix> sub_nodes_type;
  typedef std::array<Coord, dimensions> loc_type;
  typedef power_of_two_bounding_cube<dimensions, Coord> power_of_two_bounding_cube_type;
  typedef typename Traits::monoid monoid_type;
  typedef typename Traits::leaf_deleter leaf_deleter;
  typedef typename Traits::node_allocator::template rebind<sub_nodes_type>::other node_allocator;

  pow2_radix_patricia_trie_node() : ptr_(), parent_(), siblings_(), box_(), monoid_() {}
  ~pow2_radix_patricia_trie_node() { delete_ptr_(); }

  // TODO maybe implement copy-constructor that copies whole tree? or subtree?
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node const& other) = delete;
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node const& other) = delete;

  // monoid operations might conceivably throw, but destructors shouldn't
  // and Coord assignment shouldn't and primitive assignment can't.
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node&& other)
          : ptr_(nullptr), monoid_(std::move(other.monoid_)) {
    // It was too hard to exception-safe-ly move-construct
    // monoid and box_.min and ptr in the right orders,
    // so we default-construct them and then move-assign some,
    // potentially alas.
    box_.min_ = std::move(other.box_.min_);
    move_initialize_from_except_userdata_(other);
  }
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node&& other) {
    monoid_ = std::move(other.monoid_);
    box_.min_ = std::move(other.box_.min_);
    delete_ptr_();
    move_initialize_from_except_userdata_(other);
    return *this;
  }

  // It is possible for the root node to also be a ptr-to-leaf node.
  bool is_root_node()const { return !parent_; }
  bool points_to_leaf()const { return ptr_ && box_.size_exponent_in_each_dimension_ == 0; }
  bool points_to_sub_nodes()const { return ptr_ && box_.size_exponent_in_each_dimension_ != 0; }
  bool is_empty()const { return !ptr_; }
  sub_nodes_type* sub_nodes() { return points_to_sub_nodes() ? static_cast<sub_nodes_type*>(ptr_) : nullptr; }
  sub_nodes_type const* sub_nodes()const { return points_to_sub_nodes() ? static_cast<sub_nodes_type*>(ptr_) : nullptr; }
  T* leaf() { return points_to_leaf() ? static_cast<T*>(ptr_) : nullptr; }
  T const* leaf()const { return points_to_leaf() ? static_cast<T*>(ptr_) : nullptr; }
  node_type* parent() { return parent_; }
  node_type const* parent()const { return parent_; }
  sub_nodes_type* siblings() { return siblings_; }
  sub_nodes_type const* siblings()const { return siblings_; }

  bool contains(loc_type const& loc)const {
    if (!shift_value_is_safe_for_type<Coord>(box_.size_exponent_in_each_dimension_)) {
      // Avoid left-shifting by an invalidly large amount.
      return true;
    }
    const Coord mask = (~Coord(0) << box_.size_exponent_in_each_dimension_);
    for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
      if ((box_.min_[dim] & mask) != (loc[dim] & mask)) { return false; }
    }
    return true;
  }
  static node_type const& child_matching(sub_nodes_type const& sub_nodes, num_bits_type size_exponent_in_each_dimension, loc_type const& loc) {
    size_t idx = 0;
    for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
      idx |= ((get_primitive_int(loc[dim]) >> (size_exponent_in_each_dimension - 1) & 1) << ((Dims - 1) - dim));
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
    return child_matching(*sub_nodes_ptr, box_.size_exponent_in_each_dimension_, loc);
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

  // find_node() returns the leaf node at leaf_loc, if it exists
  // (i.e. find_leaf_node()); otherwise it returns the deepest existent
  // node& along the path from the root that *would* be on the path to the
  // leaf node if the leaf node were to exist.
  //
  // Note: deepest node&, not deepest node.  This is a patricia trie that
  // adds a new branch by replacing the result R of find_node() with a new
  // intermediate node and placing R's former contents in a sub-node S_a
  // (which will be a different sub-node than the new one S_b that we make
  //  to put the new branch into).  If the found node's parent P exists,
  // then P is guaranteed to contain leaf_loc.
  //
  // This scheme of node replacement is necessary because of how we store
  // sub-nodes in a single array rather than a pointer for each potential
  // sub-node.
  node_type const& find_node(loc_type leaf_loc)const {
    node_type const* node = &this->ascend_(leaf_loc);
    while (true) {
      if (!node->contains(leaf_loc)) { return *node; }
      if (!node->points_to_sub_nodes()) { return *node; }
      node = &node->child_matching(leaf_loc);
    }
  }
  node_type& find_node(loc_type leaf_loc) {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->find_node(leaf_loc));
  }
  // returns nullptr for no leaf in the tree at that loc
  node_type const* find_leaf_node(loc_type leaf_loc)const {
    node_type const& node = find_node(leaf_loc);
    if (node.points_to_leaf()) { return &node; }
    else { return nullptr; }
  }
  node_type* find_leaf_node(loc_type leaf_loc) {
    return const_cast<node_type*>(const_cast<const node_type*>(this)->find_leaf_node(leaf_loc));
  }
  T const* find_leaf(loc_type leaf_loc)const {
    node_type const& node = find_node(leaf_loc);
    if (T const* leaf = node.leaf()) { return leaf; }
    else { return nullptr; }
  }
  T* find_leaf(loc_type leaf_loc) {
    return const_cast<T*>(const_cast<const node_type*>(this)->find_leaf(leaf_loc));
  }
  node_type const& find_root()const {
    node_type const* node = this;
    while(node->parent_) { node = node->parent_; }
    return *node;
  }
  node_type& find_root() {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->find_root());
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

      //monoid_type old_monoid = std::move(monoid_);
      monoid_ = std::move(new_leaf_monoid);
      while(parent) {
        monoid_type sum = monoid_type();
        for (node_type& sibling : *siblings) {
          sum = sum + sibling.monoid_;
        }
        if (sum == parent->monoid_) { break; }
        parent->monoid_ = std::move(sum);
        //parent->monoid_ -= old_monoid;
        //parent->monoid_ += monoid_;

        siblings = parent->siblings_;
        parent = parent->parent_;
      }
    }
  }

  power_of_two_bounding_cube_type const& bounding_box()const { return box_; }


  void debug_check_this()const {
    if(is_empty()) assert(monoid_ == monoid());
    if(sub_nodes_type const* direct_children = sub_nodes()) {
      monoid_type sum = monoid_type();
      for (node_type const& direct_child : *direct_children) {
        sum = sum + direct_child.monoid_;
      }
      assert(sum == monoid_);
    }
  }
  void debug_check_recursive(node_type const* parent = nullptr)const {
    assert(parent_ == parent);
    if(sub_nodes_type const* direct_children = sub_nodes()) {
      for (node_type const& direct_child : *direct_children) {
        direct_child.debug_check_recursive(this);
      }
    }
    debug_check_this();
  }

  void debug_print(size_t depth = 0)const {
    LOG << std::string(depth*2, ' ') << monoid_ << ' ' << std::hex << box_ << ' ' << size_t(this) << " < " << size_t(parent_) << std::dec << '\n';
    if(sub_nodes_type const* direct_children = sub_nodes()) {
      for (node_type const& direct_child : *direct_children) {
        direct_child.debug_print(depth + 1);
      }
    }
  }
private:
  friend struct patricia_trie_tester;
  node_type const& ascend_(loc_type const& leaf_loc)const {
    node_type const* node = this;
    while(!node->contains(leaf_loc) && node->parent_) {
      node = node->parent_;
    }
    return *node;
  }
  node_type& ascend_(loc_type const& leaf_loc) {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->ascend_(leaf_loc));
  }

  void initialize_monoid_(monoid_type new_leaf_monoid) {
    node_type* parent = this->parent_;
    //LOG << '@' << '(' << new_leaf_monoid << ')';
    while(parent) {
      // += ?
      monoid_type sum = parent->monoid_ + new_leaf_monoid;
      //LOG << '#' << '(' << parent->monoid_ << ',' << sum << ')';
      if (sum == parent->monoid_) { break; }
      parent->monoid_ = std::move(sum);
      parent = parent->parent_;
    }
    //LOG << '\n';
    this->monoid_ = std::move(new_leaf_monoid);
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
    box_.size_exponent_in_each_dimension_ = other.box_.size_exponent_in_each_dimension_;
    other.ptr_ = nullptr;
    parent_ = other.parent_;
    siblings_ = other.siblings_;
    if (siblings_) {
      siblings_ = reinterpret_cast<sub_nodes_type*>(this - (&other - &(*other.siblings_)[0]));
      if(parent_) {
        assert(parent_->ptr_ == other.siblings_);
        parent_->ptr_ = siblings_;
      }
    }
    if (sub_nodes_type* children = this->sub_nodes()) {
      for (node_type& child : *children) {
        child.parent_ = this;
      }
    }
  }

  void* ptr_;
  node_type* parent_; // nullptr for root node
  sub_nodes_type* siblings_; // nullptr for root node

  // box_ is after monoid for the sake of patricia_trie_node's
  // move-constructor.
  // box_'s exponent value determines the type that ptr_ points to:
  // 0 for ptr-to-leaf node, nonzero for ptr-to-sub-nodes nodes,
  // doesn't matter for nullptr nodes.
  power_of_two_bounding_cube_type box_;
  monoid_type monoid_;
  // TODO make box_ and monoid_ a compressed_pair?
};


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
          intermediate_node.siblings_ = intermediate_nodes;
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
      //node->siblings remains the same
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


#endif
