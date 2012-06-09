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


#include <boost/integer.hpp>
#include <boost/integer/static_log2.hpp>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <stack>
#include <cassert>
#include <cstdlib>
#include <boost/scoped_ptr.hpp>

#include "bbox_collision_detector.hpp"

using std::pair;
using std::make_pair;
using std::unordered_map;
using std::unordered_set;

// a hack to help test this code a bit more reliably
#ifdef BBOX_COLLISION_DETECTOR_IMPL_TREAT_AS_HEADER
#define INLINE_IF_HEADER inline
#else
#define INLINE_IF_HEADER
#endif

namespace collision_detector {
namespace impl {
struct access_visitor_found_objects {
  template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
  static std::unordered_set<ObjectIdentifier>&
  get(visitor<ObjectIdentifier, CoordinateBits, NumDimensions>& v) {
    return v.found_objects_;
  }
};
namespace /*anonymous*/ {


template<num_bits_type CoordinateBits>
struct coordinate_bit_math {
  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;
  
  static Coordinate safe_left_shift_one(num_bits_type shift) {
    if (shift >= CoordinateBits) return 0;
    return Coordinate(1) << shift;
  }

  static Coordinate this_many_low_bits(num_bits_type num_bits) {
    return safe_left_shift_one(num_bits) - 1;
  }
};


template<typename Coordinate, size_t/*template argument deduction requires this type here*/ NumDimensions>
INLINE_IF_HEADER
Coordinate max_in_array_of_unsigned(std::array<Coordinate, NumDimensions> const& arr) {
  if(NumDimensions == 0) {
    return 0;
  }
  else {
    Coordinate max_val = arr[0];
    for (size_t i = 1; i < NumDimensions; ++i) {
      if (arr[i] > max_val) max_val = arr[i];
    }
    return max_val;
  }
}

// bbox_collision_detector uses "z-ordering", named such because "z" is a visual for
// the zigzag the ordering creates when drawn.  See
//     https://en.wikipedia.org/wiki/Z-order_curve
//
// A zbox is:
//
// If you interleave the coordinates' bits in the z-ordering way, it is a contiguous
// range of interleaved bits from, for example, binary VVVVV000 through VVVVV111,
// for some VVVVV.  Consider the zbox to consist of VVVVV and the number of low bits
// that vary.
// (This example could be a two-dimensional bbox_collision_detector<foo, 4, 2>,
// if four-bit integers existed -- imagine lots more bits for a real zbox.)
//
// Equivalently, a zbox is a square/cube/etc. on a maximally-bit-aligned grid of
// square/cubes of its size, or a rectangle/etc. that consists of 2/etc. of these
// square/cubes adjacent (in the case that the number of low bits in the first
// explanation is not a multiple of NumDimensions).
template<num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class zbox {
private:
  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef coordinate_bit_math<CoordinateBits> math_;
  // We ensure that every bit except the ones specifically supposed to be on is off.
  // (Specifically, the "low bits" are zero.)
  std::array<Coordinate, NumDimensions> coords_;

  //small_num_bits_type can represent at least [0, CoordinateBits*NumDimensions] inclusive.
  //smaller_num_bits_type can represent at least [0, CoordinateBits] inclusive.
  typedef typename boost::uint_t<static_num_bits_in_integer_that_are_not_leading_zeroes<CoordinateBits*NumDimensions>::value>::least small_num_bits_type;
  typedef typename boost::uint_t<static_num_bits_in_integer_that_are_not_leading_zeroes<CoordinateBits>::value>::least smaller_num_bits_type;
  small_num_bits_type num_low_bits_;
  std::array<smaller_num_bits_type, NumDimensions> dim_num_low_bits_;

public:
  typedef std::ostream hack_to_make_bbox_collision_detector_zbox_ostreamable;

  zbox():num_low_bits_(CoordinateBits * NumDimensions){}

  // Named constructor idiom
  static zbox smallest_joint_parent(zbox zb1, zbox zb2) {
    zbox new_box;
    num_bits_type dim_low_bits_heuristic[NumDimensions];
    num_bits_type largest_dim_low_bits = 0;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      const Coordinate uncommon_bits =
          (zb1.coords_[i] | math_::this_many_low_bits(zb2.num_low_bits_by_dimension(i)))
        ^ (zb2.coords_[i] & ~math_::this_many_low_bits(zb1.num_low_bits_by_dimension(i)));
      const num_bits_type this_dimension_low_bits = num_bits_in_integer_that_are_not_leading_zeroes(uncommon_bits);
      dim_low_bits_heuristic[i] = this_dimension_low_bits;
      if(largest_dim_low_bits < this_dimension_low_bits) largest_dim_low_bits = this_dimension_low_bits;
    }
    const num_bits_type low_bits_minor = largest_dim_low_bits - 1;
    const num_bits_type low_bits_major = largest_dim_low_bits;
    num_bits_type lim_low_bits = low_bits_minor;
    num_bits_type dim_low_bits[NumDimensions];
    num_bits_type total_low_bits = 0;
    for (num_coordinates_type i = NumDimensions - 1; i >= 0; --i) {
      if(dim_low_bits_heuristic[i] ==/*a.k.a.>=*/ low_bits_major) lim_low_bits = low_bits_major;
      dim_low_bits[i] = lim_low_bits;
      new_box.dim_num_low_bits_[i] = lim_low_bits;
      total_low_bits += lim_low_bits;
    }
    new_box.num_low_bits_ = total_low_bits;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      assert_if_ASSERT_EVERYTHING(
            (zb1.coords_[i] & ~math_::this_many_low_bits(dim_low_bits[i]))
        == (zb2.coords_[i] & ~math_::this_many_low_bits(dim_low_bits[i]))
      );
      new_box.coords_[i] = zb1.coords_[i] & ~math_::this_many_low_bits(dim_low_bits[i]);
    }
    return new_box;
  }

  static zbox box_from_coords(std::array<Coordinate, NumDimensions> const& coords, num_bits_type num_low_bits) {
    zbox result;
    result.num_low_bits_ = num_low_bits;
    const num_bits_type base_num_low_bits = num_low_bits / NumDimensions;
    const num_bits_type tweak_num_low_bits = num_low_bits % NumDimensions;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      result.dim_num_low_bits_[i] = base_num_low_bits + (i < tweak_num_low_bits);
      result.coords_[i] = coords[i] & ~math_::this_many_low_bits(result.num_low_bits_by_dimension(i));
    }
    return result;
  }

  bool subsumes(zbox const& other)const {
    if (other.num_low_bits_ > num_low_bits_) return false;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      if (coords_[i] != (other.coords_[i] & ~math_::this_many_low_bits(num_low_bits_by_dimension(i)))) return false;
    }
    return true;
  }
  bool overlaps(zbox const& other)const {
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      if ( (coords_[i] & ~math_::this_many_low_bits(other.num_low_bits_by_dimension(i)))
        != (other.coords_[i] & ~math_::this_many_low_bits(num_low_bits_by_dimension(i)))) {
          return false;
      }
    }
    return true;
  }
  bool overlaps(bounding_box const& bbox)const {
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      const Coordinate this_size_i = math_::safe_left_shift_one(num_low_bits_by_dimension(i));
      if (bbox.min[i] + (bbox.size[i] - 1) <  coords_[i]) return false;
      if ( coords_[i] + (this_size_i  - 1) < bbox.min[i]) return false;
    }
    return true;
  }
  bool get_bit(num_bits_type bit)const {
    return coords_[bit % NumDimensions] & math_::safe_left_shift_one(bit / NumDimensions);
  }
  num_bits_type num_low_bits_by_dimension(num_coordinates_type dim)const {
    return dim_num_low_bits_[dim];
  }
  // note: gives "size=0" for max-sized things
  bounding_box get_bbox()const {
    bounding_box bbox;
    bbox.min = coords_;
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      bbox.size[i] = math_::safe_left_shift_one(num_low_bits_by_dimension(i));
    }
    return bbox;
  }
  num_bits_type num_low_bits()const {
    return num_low_bits_;
  }
  bool operator==(zbox const& other)const {
    return num_low_bits_ == other.num_low_bits_ && coords_ == other.coords_;
  }
  bool operator!=(zbox const& other)const {
    return !(*this == other);
  }
  friend inline std::ostream& operator<<(std::ostream& os, zbox const& zb) {
    return os << "0x" << std::hex << zb.get_bbox() << std::dec;
  }
};

// The bbox_collision_detector has a "z-tree".  This
// is a binary tree.  Keys in the tree are zboxes (see above).
// Values are ObjectIdentifiers; each zbox may have any number
// of ObjectIdentifiers.  This code is happy for objects to overlap
// each other, and besides, even non-overlapping objects often
// have a minimal containing zbox in common.
//
// Because of the definition of zboxes, either they are
// A: the same, and thus the same ztree_node
// B: one is smaller and fully within the node, and it is a
//          descendant of the other
// C: they don't overlap at all, and in the ztree neither is a
//          descendant of the other.
// (In particular, zboxes can't partially overlap each other.)
//
// An object may need to be in up to (2**NumDimensions) zboxes
// so that the area covered by its zboxes is only a constant
// factor larger than the object's regular (non-z-order) bounding
// box.  Consider an object that's a box of width 2 or 3 with
// x min-coordinate 10111111 and max 11000001 (binary).  The minimal
// common prefix there is just a single bit; a single bit means
// a huge box.  Conceptually, dimensions' bits are interleaved
// before looking for a common prefix; any dimension has the
// potential to differ in a high bit and disrupt the common prefix.
// Splitting the object across two zboxes, for each dimension,
// is sufficient to avoid this explosion.
//
// Specifically, a ztree is a Patricia trie on z-ordered
// bits (zboxes), where the bits are seen as a sequence with the
// highest-order bits first and the sequence-length equal to the
// number of high bits specified by a given key/ztree_node/zbox.
// The ztree_node happens to contain (unlike typical tries) the
// entire key that it represents, because the key is small and
// constant-sized and it's generally easier to do so.
//
// What goes into child0 vs. child1?  If trying to insert, say,
// the zbox B = 10100??? (binary, 5 high bits, 3 low bits) into
// A = 10??????, B goes at or below A's child1 because B's next bit
// after A's bits is 1.  (Current: 10, next: 101).
//
// If there would be a node with zero ObjectIdentifiers and
// only one child, then that child node goes there directly
// instead of that trivial node.  If the tree, to be correct,
// needs nodes with two children and zero ObjectIdentifiers,
// then it will have them.
} /* temporarily end anonymous namespace */
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
struct ztree_node {
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef impl::zbox<CoordinateBits, NumDimensions> zbox;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  
  const zbox here;
  ztree_node_ptr child0;
  ztree_node_ptr child1;
  
  typedef pair<const ObjectIdentifier, bounding_box> const* id;
  unordered_set<id> objects_here;
  
  ztree_node(zbox box):here(box),child0(nullptr),child1(nullptr){}
  ztree_node(ztree_node const& other) :
    here(other.here),
    child0(other.child0 ? new ztree_node(*other.child0) : nullptr),
    child1(other.child1 ? new ztree_node(*other.child1) : nullptr),
    objects_here(other.objects_here)
    {}
  // operator= could exist if we wanted to make zbox non-const.
  // ztree_node& operator=(ztree_node const& other) = delete;
};
namespace { /* resume anonymous namespace */
/*

If there's one zbox in the tree [call it Z]

tree = ztree_node {
  Z
  nullptr
  nullptr
}

Two zboxes that differ at bit B:
child0 of a node with B ignored bits is the child whose Bth bit is 0.
tree = ztree_node {
  the common leading bits of Z1 and Z2, with B ignored bits
  ptr to ztree_node {
    Z1
    nullptr
    nullptr
  }
  ptr to ztree_node {
    Z2
    nullptr
    nullptr
  }
}

*/



template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
struct ztree_ops {
  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;
  typedef coordinate_bit_math<CoordinateBits> math_;
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef impl::zbox<CoordinateBits, NumDimensions> zbox;
  typedef impl::ztree_node<ObjectIdentifier, CoordinateBits, NumDimensions> ztree_node;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  
  // A call to insert_zboxes_from_bbox() might be more clearly written
  //   for(zbox zb : zboxes_from_bbox(bbox)) {
  //     insert_zbox(objects_tree, id, zb);
  //   }
  // The only reason we combine insert_zbox and zboxes_from_bbox is so
  // we don't have to construct the intermediate container structure.
  static void insert_zboxes_from_bbox(ztree_node_ptr& objects_tree, pair<const ObjectIdentifier, bounding_box> const* id_and_bbox) {
    bounding_box const& bbox = id_and_bbox->second;
    const Coordinate max_width = max_in_array_of_unsigned(bbox.size);
    // max_width - 1: power-of-two-sized objects easily squeeze into the next smaller category.
    // i.e., exp = log2_rounding_up(max_width)
    const num_bits_type exp = num_bits_in_integer_that_are_not_leading_zeroes(max_width - 1);
    const Coordinate base_box_size = math_::safe_left_shift_one(exp);

    // The total number of zboxes we use to cover this bounding_box
    // is a power of two between 1 and 2**NumDimensions.
    // Imagine that we start with a set of one box and that,
    // for each dimension, we start with the previous set of boxes,
    // then zbox-ify this dimension, using either
    //   (A) exactly base_box_size width-in-this-dimension, or
    //   (B) twice base_box_size width-in-this-dimension, or
    // if the bit parity didn't work out so well, it
    //   (C) needs to split each zbox into two.
    num_coordinates_type num_dims_using_one_zbox_of_exactly_base_box_size = 0;
    num_coordinates_type num_dims_using_one_zbox_of_twice_base_box_size = 0;
    num_coordinates_type num_dims_using_two_zboxes_each_of_base_box_size;

    // Given that a coordinate is laid out in bits like XYZXYZXYZ,
    // We're at some exp in there (counted from the right); let's say 3.
    // Given exp 3, Z is a less-significant bit and X is more-significant.
    // ('exp' could also be a non-multiple-of-NumDimensions, in which case
    // the ordering of the dimensions would come out differently.)
    //
    // If the object happens to fit, aligned, in X with width base_box_size,
    // then we can just specify this X bit directly.  If that works for X,
    // we can try Y; if not, we can't try Y because X is already doing
    // something nontrivial (perhaps it could be done; the code would
    // be more complicated).  These are
    // "num_dimensions_that_need_one_zbox_of_exactly_base_box_size".
    // This is the best case.
    //
    // Then, if we can't specify where in all dimensions we are
    // z-box-ly yet in one zbox, we try starting from Z:
    // it's possible that Z (and so forth if Z is) can be included
    // in the zbox's low_bits.  This would make the zbox twice
    // as wide in that dimension (e.g. Z) as it would be in the
    // case of if X fits into a single base_box_size at the scale
    // we're looking at.  But it's better than making two separate
    // zboxes that take up that much space anyway.  If the bounding
    // box didn't happen to be aligned with the right parity,
    // we'll have to make two boxes for it anyway instead of putting
    // it in "num_dimensions_that_need_one_zbox_of_twice_base_box_size".
    //
    // All the dimensions in between will be split into two zboxes,
    // each of width base_box_size, for a total width of
    // twice base_box_size.  This is the worst case,
    // "num_dimensions_that_need_two_zboxes_each_of_base_box_size".
    for (num_coordinates_type i = NumDimensions - 1; i >= 0; --i) {
      if ((bbox.size[i] - 1) < base_box_size - (bbox.min[i] & (base_box_size - 1))) {
        ++num_dims_using_one_zbox_of_exactly_base_box_size;
      }
      else {
        break;
      }
    }
    for (num_coordinates_type i = 0; i < NumDimensions - num_dims_using_one_zbox_of_exactly_base_box_size; ++i) {
      if (!(bbox.min[i] & base_box_size)) {
        ++num_dims_using_one_zbox_of_twice_base_box_size;
      }
      else {
        break;
      }
    }
    num_dims_using_two_zboxes_each_of_base_box_size = NumDimensions - num_dims_using_one_zbox_of_exactly_base_box_size - num_dims_using_one_zbox_of_twice_base_box_size;

    const num_zboxes_type number_of_zboxes_to_use_if_necessary = num_zboxes_type(1) << num_dims_using_two_zboxes_each_of_base_box_size;

    for (num_zboxes_type i = 0; i < number_of_zboxes_to_use_if_necessary; ++i) {
      std::array<Coordinate, NumDimensions> coords = bbox.min;
      for (num_coordinates_type j = num_dims_using_one_zbox_of_twice_base_box_size; j < NumDimensions - num_dims_using_one_zbox_of_exactly_base_box_size; ++j) {
        // By checking this bit of "i" arbitrarily, by the last time
        // we get through the "number_of_zboxes_to_use_if_necessary" loop,
        // we have yielded every combination of possibilities of
        // each dimension that varies varying (between +0 and +base_box_size).
        if (i & (1 << (j - num_dims_using_one_zbox_of_twice_base_box_size))) {
          coords[j] += base_box_size;
        }
      }
      const zbox zb = zbox::box_from_coords(coords, exp * NumDimensions + num_dims_using_one_zbox_of_twice_base_box_size);
      if (zb.overlaps(bbox)) {
        insert_zbox(objects_tree, id_and_bbox, zb);
      }
    }
  }
  static void insert_zbox(ztree_node_ptr& tree, pair<const ObjectIdentifier, bounding_box> const* id_and_bbox, zbox box) {
    if (!tree) {
      tree.reset(new ztree_node(box));
      tree->objects_here.insert(id_and_bbox);
    }
    else {
      if (tree->here.subsumes(box)) {
        if (box.num_low_bits() == tree->here.num_low_bits()) {
          tree->objects_here.insert(id_and_bbox);
        }
        else {
          if (box.get_bit(tree->here.num_low_bits() - 1)) insert_zbox(tree->child1, id_and_bbox, box);
          else                                            insert_zbox(tree->child0, id_and_bbox, box);
        }
      }
      else {
        ztree_node_ptr new_tree(new ztree_node(zbox::smallest_joint_parent(tree->here, box)));

        assert_if_ASSERT_EVERYTHING(new_tree->here.num_low_bits() > tree->here.num_low_bits());
        assert_if_ASSERT_EVERYTHING(new_tree->here.subsumes(tree->here));
        assert_if_ASSERT_EVERYTHING(new_tree->here.subsumes(box));
        assert_if_ASSERT_EVERYTHING(box.subsumes(tree->here) || (tree->here.get_bit(new_tree->here.num_low_bits() - 1) != box.get_bit(new_tree->here.num_low_bits() - 1)));

        if (tree->here.get_bit(new_tree->here.num_low_bits() - 1)) tree.swap(new_tree->child1);
        else                                                       tree.swap(new_tree->child0);

        tree.swap(new_tree);
        insert_zbox(tree, id_and_bbox, box);
      }
    }
  }

  static void delete_object(ztree_node_ptr& tree, pair<const ObjectIdentifier, bounding_box> const* id_and_bbox) {
    if (!tree) return;
    if (tree->here.overlaps(id_and_bbox->second)) {
      tree->objects_here.erase(id_and_bbox);
      delete_object(tree->child0, id_and_bbox);
      delete_object(tree->child1, id_and_bbox);

      // collapse nodes with no objects and 0-1 children.
      if (tree->objects_here.empty()) {
        if (!tree->child0) {
          // (old 'child1' a.k.a. new 'tree' could be nullptr)
          ztree_node_ptr dead_tree;
          dead_tree.swap(tree);
          dead_tree->child1.swap(tree);
        }
        else if (!tree->child1) {
          ztree_node_ptr dead_tree;
          dead_tree.swap(tree);
          dead_tree->child0.swap(tree);
        }
      }
    }
  }
  typedef pair<const ObjectIdentifier, bounding_box> const* id_and_bbox_type;
  static void zget_objects_overlapping(
        ztree_node const* tree,
        typename lasercake_set<id_and_bbox_type>::type& results,
        bounding_box const& bbox) {
    if (tree && tree->here.overlaps(bbox)) {
      for(pair<const ObjectIdentifier, bounding_box> const* id_and_bbox : tree->objects_here) {
        if (id_and_bbox->second.overlaps(bbox)) results.insert(id_and_bbox);
      }
      zget_objects_overlapping(tree->child0.get(), results, bbox);
      zget_objects_overlapping(tree->child1.get(), results, bbox);
    }
  }
};

// With a bit of work, this could be an iterator ("custom_iterator"?).
// process_next() is increment.
// iterator == end is (frontier.empty() || handler->should_exit_early()).
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
struct ztree_custom_walker {
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef impl::zbox<CoordinateBits, NumDimensions> zbox;
  typedef impl::ztree_node<ObjectIdentifier, CoordinateBits, NumDimensions> ztree_node;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  typedef collision_detector::visitor<ObjectIdentifier, CoordinateBits, NumDimensions> visitor;
  
  visitor* handler;
  unordered_map<ObjectIdentifier, bounding_box> const& bboxes_by_object_;
  ztree_custom_walker(visitor* handler, unordered_map<ObjectIdentifier, bounding_box> const& bboxes_by_object):handler(handler),bboxes_by_object_(bboxes_by_object){}

  // The search's frontier (nodes for which we have explored their parent
  // but not them yet).
  //
  // This only contains nonnull pointers (since it would be a
  // waste of time to insert null pointers into the frontier).
  std::stack<ztree_node const*> frontier;

  void try_add(ztree_node const* z) {
    if (z && handler->should_be_considered__static(z->here.get_bbox()) && handler->should_be_considered__dynamic(z->here.get_bbox())) {
      frontier.push(z);
    }
  }

  bool process_next() {
    if (frontier.empty()) return false;

    ztree_node const* const next = frontier.top();
    frontier.pop();

    if (handler->should_be_considered__dynamic(next->here.get_bbox())) {
      for (auto id_and_bbox : next->objects_here) {
        if (handler->should_be_considered__static(id_and_bbox->second) && handler->should_be_considered__dynamic(id_and_bbox->second)) {
          if (access_visitor_found_objects::get(*handler).insert(id_and_bbox->first).second) {
            handler->handle_new_find(id_and_bbox->first);
            if (handler->should_exit_early()) return false;
          }
        }
      }

      ztree_node const* children[2] = { next->child0.get(), next->child1.get() };
      // If either child is nullptr, the order doesn't matter (and can't be checked).
      const bool child0_next = children[0] && children[1] &&
          handler->bbox_less_than(children[0]->here.get_bbox(), children[1]->here.get_bbox());
      try_add(child0_next ? children[0] : children[1]);
      try_add(child0_next ? children[1] : children[0]);
    }

    return true;
  }
};

} /* end anonymous namespace */
} /* end namespace impl */


template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
void bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
insert(ObjectIdentifier const& id, bounding_box const& bbox) {
  auto iter_and_new = bboxes_by_object_.insert(make_pair(id, bbox));
  caller_correct_if(
    iter_and_new.second,
    "bbox_collision_detector::insert() requires for your safety that the id "
    "is not already in this container.  Use .erase() or .exists() if you need "
    "any particular behaviour in this case."
  );

  impl::ztree_ops<ObjectIdentifier, CoordinateBits, NumDimensions>::insert_zboxes_from_bbox(objects_tree_, &*iter_and_new.first);
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bool bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
erase(ObjectIdentifier const& id) {
  const auto bbox_iter = bboxes_by_object_.find(id);
  if (bbox_iter == bboxes_by_object_.end()) return false;
  impl::ztree_ops<ObjectIdentifier, CoordinateBits, NumDimensions>::delete_object(objects_tree_, &*bbox_iter);
  bboxes_by_object_.erase(bbox_iter);
  return true;
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
void bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
get_objects_overlapping(std::vector<ObjectIdentifier>& results, bounding_box const& bbox)const {
  typedef pair<const ObjectIdentifier, bounding_box> const* id_and_bbox_type;
  //static so there's less reallocation; not threadsafe yet:
  static typename lasercake_set<id_and_bbox_type>::type uniquified_results;
  impl::ztree_ops<ObjectIdentifier, CoordinateBits, NumDimensions>::zget_objects_overlapping(objects_tree_.get(), uniquified_results, bbox);
  results.reserve(results.size() + uniquified_results.size());
  for(pair<const ObjectIdentifier, bounding_box> const* id_and_bbox : uniquified_results) {
    results.push_back(id_and_bbox->first);
  }
  uniquified_results.clear();
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
void bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
search(visitor* handler)const {
  impl::ztree_custom_walker<ObjectIdentifier, CoordinateBits, NumDimensions> data(handler, bboxes_by_object_);

  data.try_add(objects_tree_.get());
  while(data.process_next()){}
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>&
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::operator=(bbox_collision_detector const& other) {
  bboxes_by_object_ = other.bboxes_by_object_;
  objects_tree_.reset(other.objects_tree_ ? new ztree_node(*other.objects_tree_) : nullptr);
  return *this;
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
bbox_collision_detector()
: objects_tree_(nullptr) {}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
~bbox_collision_detector()
{}

} /* end namespace collision_detector */

#undef INLINE_IF_HEADER

#ifndef BBOX_COLLISION_DETECTOR_IMPL_TREAT_AS_HEADER
#include "world.hpp"
namespace collision_detector {
template class bbox_collision_detector<object_or_tile_identifier, 64, 3>;
}
#endif
