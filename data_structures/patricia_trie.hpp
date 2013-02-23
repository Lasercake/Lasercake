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
  loc_type const& min()const { return min_; }
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

  friend inline bool operator==(power_of_two_bounding_cube const& a, power_of_two_bounding_cube const& b) {
    return a.size_exponent_in_each_dimension_ == b.size_exponent_in_each_dimension_
      && a.min_ == b.min_;
  }
  friend inline bool operator!=(power_of_two_bounding_cube const& a, power_of_two_bounding_cube const& b) {
    return !(a == b);
  }

  void set_min(loc_type min) { min_ = min; }
  void set_size_exponent_in_each_dimension(num_bits_type exp) {
    size_exponent_in_each_dimension_ = exp;
  }
private:
  // Friend patricia_trie so that it can be more paranoid about
  // exception-safety, specifically the order in which the data members
  // are changed in move-assignment.
  template<num_coordinates_type Dims2, typename Coord2, typename T, typename Traits>
  friend class pow2_radix_patricia_trie_node;

  loc_type min_;
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
  typedef lasercake_nice_allocator<int>::type node_allocator;
};



// This class is here so that I can tweak the representation
// for efficiency without changing the tricky trie-manipulation
// code.
// Its destructor is responsible for destroying leaf/monoid info
// but not sub nodes.
template<num_coordinates_type Dims, typename Coord, typename T, typename Traits = default_pow2_radix_patricia_trie_traits>
struct patricia_trie_representation {
public:
  static const num_coordinates_type dimensions = Dims;
  static const size_t radix = size_t(1) << dimensions;
  typedef pow2_radix_patricia_trie_node<Dims, Coord, T, Traits> node_type;
  typedef std::array<node_type, radix> sub_nodes_type;
  typedef std::array<Coord, dimensions> loc_type;
  typedef power_of_two_bounding_cube<dimensions, Coord> power_of_two_bounding_cube_type;
  typedef typename Traits::monoid monoid_type;
  typedef typename Traits::node_allocator::template rebind<sub_nodes_type>::other node_allocator;

  patricia_trie_representation() : leaf_(), sub_nodes_(), parent_(), box_(), monoid_() {}
  ~patricia_trie_representation() {}
  // Until I need it to be copyable or movable, better not to risk
  // an untested implementation.
  patricia_trie_representation(patricia_trie_representation const&) = delete;
  patricia_trie_representation(patricia_trie_representation&&) = delete;
  patricia_trie_representation& operator=(patricia_trie_representation const&) = delete;
  patricia_trie_representation& operator=(patricia_trie_representation&&) = delete;

  // Hmm...
  static T null_leaf() { return T(); }
  static bool is_null_leaf(T const& t) { return t == null_leaf(); }

  // It is possible for the root node to also be a ptr-to-leaf node.
  bool is_root_node()const { return !parent_; }
  bool points_to_leaf()const { return !is_null_leaf(leaf_); }
  bool points_to_sub_nodes()const { return sub_nodes_; }
  bool is_empty()const { return !points_to_leaf() && !points_to_sub_nodes(); }
  sub_nodes_type* sub_nodes() { return sub_nodes_; }
  sub_nodes_type const* sub_nodes()const { return sub_nodes_; }
  // Beware leaf() may reference a default-constructed that indicates
  // that nothing's here.
  T& leaf() { return leaf_; }
  T const& leaf()const { return leaf_; }
  node_type* parent() { return parent_; }
  node_type const* parent()const { return parent_; }

  power_of_two_bounding_cube_type const& bounding_box()const { return box_; }
  loc_type const& min()const {
    return box_.min();
  }
  num_bits_type size_exponent_in_each_dimension()const {
    return box_.size_exponent_in_each_dimension();
  }

  monoid_type const& monoid()const { return monoid_; }

protected:
  void set_sub_nodes(sub_nodes_type* new_sub_nodes) {
    sub_nodes_ = new_sub_nodes;
  }
  T move_leaf() {
    T result = std::move(leaf_);
    leaf_ = null_leaf();
    return std::move(result);
  }
  void set_leaf(T&& new_leaf = null_leaf()) {
    leaf_ = std::move(new_leaf);
  }
  void set_parent(node_type* parent) {
    parent_ = parent;
  }
  void set_min(loc_type min) {
    box_.set_min(min);
  }
  void set_size_exponent_in_each_dimension(num_bits_type exp) {
    box_.set_size_exponent_in_each_dimension(exp);
  }
  void set_monoid(monoid_type const& monoid) {
    monoid_ = monoid;
  }
  void set_monoid(monoid_type&& monoid) {
    monoid_ = std::move(monoid);
  }

private:
  T leaf_;
  sub_nodes_type* sub_nodes_;
  node_type* parent_; // nullptr for root node
  power_of_two_bounding_cube_type box_;
  monoid_type monoid_;
  // TODO make box_ and monoid_ a compressed_pair?
};

// requires nothrow move for Coord, Monoid, and T.
// T must be default-constructable, and checkable whether it has been
// default-constructed, and its default-constructed state will be sitting around
// in the tree in places it doesn't belong.
template<num_coordinates_type Dims, typename Coord, typename T, typename Traits = default_pow2_radix_patricia_trie_traits>
class pow2_radix_patricia_trie_node
  : public patricia_trie_representation<Dims, Coord, T, Traits> {
public:
  typedef patricia_trie_representation<Dims, Coord, T, Traits> rep;
  static const num_coordinates_type dimensions = Dims;
  static const size_t radix = size_t(1) << dimensions;
  typedef pow2_radix_patricia_trie_node node_type;
  typedef std::array<node_type, radix> sub_nodes_type;
  typedef std::array<Coord, dimensions> loc_type;
  typedef power_of_two_bounding_cube<dimensions, Coord> power_of_two_bounding_cube_type;
  typedef typename Traits::monoid monoid_type;
  typedef typename Traits::node_allocator::template rebind<sub_nodes_type>::other node_allocator;

  pow2_radix_patricia_trie_node() : rep() {}
  ~pow2_radix_patricia_trie_node() { delete_sub_nodes_(); }

  // TODO maybe implement copy-constructor that copies whole tree? or subtree?
  // Until I need it to be copyable or movable, better not to risk
  // an untested implementation.
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node const&) = delete;
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node const&) = delete;
  pow2_radix_patricia_trie_node(pow2_radix_patricia_trie_node&&) = delete;
  pow2_radix_patricia_trie_node& operator=(pow2_radix_patricia_trie_node&&) = delete;

  sub_nodes_type* siblings() { return rep::parent() ? rep::parent()->sub_nodes() : nullptr; }
  sub_nodes_type const* siblings()const { return rep::parent() ? rep::parent()->sub_nodes() : nullptr; }

  bool contains(loc_type const& loc)const {
    if (!shift_value_is_safe_for_type<Coord>(rep::size_exponent_in_each_dimension())) {
      // Avoid left-shifting by an invalidly large amount.
      return true;
    }
    const Coord mask = (~Coord(0) << rep::size_exponent_in_each_dimension());
    for (num_coordinates_type dim = 0; dim != dimensions; ++dim) {
      if ((rep::min()[dim] & mask) != (loc[dim] & mask)) { return false; }
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
    sub_nodes_type const* sub_nodes_ptr = rep::sub_nodes();
    assert(sub_nodes_ptr);
    return child_matching(*sub_nodes_ptr, rep::size_exponent_in_each_dimension(), loc);
  }
  node_type& child_matching(loc_type const& loc) {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->child_matching(loc));
  }

  // Insert can be called on any node in the tree;
  // it makes no difference which, except for efficiency
  // (passing a nearby node rather than the root node can
  // be faster; passing a far-away node can be slower).
  //
  // The leaf_ptr will be taken ownership of, in the sense
  // that when it is erased or the tree is destroyed (or an
  // exception is thrown during insert()), the deleter will
  // be called on the pointer.
  //
  // An exception will be thrown if the leaf_loc already exists
  // in the tree.
  //
  // TODO but monoid operations! - they need to propagate to the top.
  // also, it could have subtraction and then it would be a bit faster...hmm.
  // and how can we check whether it doesn't need any propagation...
  // == identity? is-a-boring-type(like nan)? adding-it-didn't-change-the-value-so-aboves-dont-need-changing, ah.
  void insert(loc_type leaf_loc, T&& new_leaf, monoid_type leaf_monoid = monoid_type());
  template<typename TT> void insert(loc_type leaf_loc, TT const& new_leaf, monoid_type leaf_monoid = monoid_type()) {
    this->insert(leaf_loc, T(new_leaf), leaf_monoid);
  }

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
  T const& find_leaf(loc_type leaf_loc)const {
    node_type const& node = find_node(leaf_loc);
    return node.leaf();
  }
  T& find_leaf(loc_type leaf_loc) {
    node_type& node = find_node(leaf_loc);
    return node.leaf();
  }
  node_type const& find_root()const {
    node_type const* node = this;
    while(node->parent()) { node = node->parent(); }
    return *node;
  }
  node_type& find_root() {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->find_root());
  }


  // erase returns true iff something was erased.
  bool erase(loc_type leaf_loc);

  void update_monoid(monoid_type new_leaf_monoid) {
    caller_correct_if(this->points_to_leaf(),
        "patricia_trie: Only leaves can have their monoids explicitly set; "
        "parent and empty node monoids are automatically computed.");
    // If monoid_type had a -=, this could be faster.
    if (!(rep::monoid() == new_leaf_monoid)) {
      node_type* parent = rep::parent();

      //monoid_type old_monoid = std::move(monoid_);
      rep::set_monoid(std::move(new_leaf_monoid));
      while(parent) {
        sub_nodes_type const& siblings = *parent->sub_nodes();
        monoid_type sum = monoid_type();
        for (node_type const& sibling : siblings) {
          sum = sum + sibling.monoid();
        }
        if (sum == parent->monoid()) { break; }
        parent->set_monoid(std::move(sum));
        //parent->monoid_ -= old_monoid;
        //parent->monoid_ += monoid_;

        parent = parent->parent();
      }
    }
  }


  void debug_check_this()const {
    if(this->is_empty()) assert(rep::monoid() == monoid_type());
    if(sub_nodes_type const* direct_children = rep::sub_nodes()) {
      monoid_type sum = monoid_type();
      for (node_type const& direct_child : *direct_children) {
        sum = sum + direct_child.monoid();
      }
      assert(sum == rep::monoid());
    }
  }
  void debug_check_recursive(node_type const* parent = nullptr)const {
    assert(rep::parent() == parent);
    if(sub_nodes_type const* direct_children = rep::sub_nodes()) {
      for (node_type const& direct_child : *direct_children) {
        direct_child.debug_check_recursive(this);
      }
    }
    debug_check_this();
  }

  void debug_print(size_t depth = 0)const {
    LOG << std::string(depth*2, ' ') << rep::monoid()
        << ' ' << std::hex
        << rep::bounding_box()
        << ' ' << size_t(this) << " < " << size_t(rep::parent())
        << std::dec << '\n';
    if(sub_nodes_type const* direct_children = rep::sub_nodes()) {
      for (node_type const& direct_child : *direct_children) {
        direct_child.debug_print(depth + 1);
      }
    }
  }
private:
  friend struct patricia_trie_tester;
  node_type const& ascend_(loc_type const& leaf_loc)const {
    node_type const* node = this;
    while(!node->contains(leaf_loc) && node->parent()) {
      node = node->parent();
    }
    return *node;
  }
  node_type& ascend_(loc_type const& leaf_loc) {
    return const_cast<node_type&>(const_cast<const node_type*>(this)->ascend_(leaf_loc));
  }

  void initialize_monoid_(monoid_type new_leaf_monoid) {
    node_type* parent = this->parent();
    //LOG << '@' << '(' << new_leaf_monoid << ')';
    while(parent) {
      // += ?
      monoid_type sum = parent->monoid() + new_leaf_monoid;
      //LOG << '#' << '(' << parent->monoid_ << ',' << sum << ')';
      if (sum == parent->monoid()) { break; }
      parent->set_monoid(std::move(sum));
      parent = parent->parent();
    }
    //LOG << '\n';
    this->set_monoid(std::move(new_leaf_monoid));
  }

  void delete_sub_nodes_() {
    if(sub_nodes_type* sub_nodes_ptr = rep::sub_nodes()) {
      sub_nodes_ptr->~sub_nodes_type();
      node_allocator().deallocate(sub_nodes_ptr, 1);
    }
    rep::set_sub_nodes(nullptr);
  }

#if 0
  // Until I need it to be copyable or movable, better not to risk
  // an untested implementation.
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
  void move_initialize_from_except_userdata_(pow2_radix_patricia_trie_node& old) BOOST_NOEXCEPT {
    assert(ptr_ == nullptr);
    ptr_ = old.ptr_;
    box_.size_exponent_in_each_dimension_ = old.box_.size_exponent_in_each_dimension_;
    old.ptr_ = nullptr;
    parent_ = old.parent_;
    if (parent_) {
      // If we're moving a sub_nodes_type then each member will
      // be moved but only one of them should update the parent's
      // sub-nodes ptr.
      if(&old == &(*parent_->sub_nodes())[0]) {
        assert(parent_->ptr_ == reinterpret_cast<sub_nodes_type*>(&old));
        parent_->ptr_ = reinterpret_cast<sub_nodes_type*>(this);
      }
    }
    if (sub_nodes_type* children = this->sub_nodes()) {
      for (node_type& child : *children) {
        child.parent_ = this;
      }
    }
  }
#endif
};

#endif
