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

#ifndef LASERCAKE_BBOX_COLLISION_DETECTOR_HPP__
#define LASERCAKE_BBOX_COLLISION_DETECTOR_HPP__

#include <boost/integer.hpp>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <set>
#include <cassert>
#include <cstdlib>
#include <boost/scoped_ptr.hpp>
#include "utils.hpp"

using std::unordered_map;
using std::unordered_set;

typedef ptrdiff_t num_bits_type;
typedef ptrdiff_t num_coordinates_type;

// ObjectIdentifier needs hash and == and to be freely copiable. So, ints will do, pointers will do...
// CoordinateBits should usually be 32 or 64. I don't know if it works for other values.
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class bbox_collision_detector {
  static_assert(NumDimensions >= 0, "You can't make a space with negative dimensions!");
  static_assert(CoordinateBits >= 0, "You can't have an int type with negative bits!");
  friend class zbox_tester;

public:
  typedef typename boost::uint_t<CoordinateBits>::fast Coordinate;
  
  struct bounding_box {
    std::array<Coordinate, NumDimensions> min;
    std::array<Coordinate, NumDimensions> size;
    
    bool overlaps(bounding_box const& other)const {
      for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
        // this should correctly handle zboxes' "size=0" when all bits are ignored
        if (other.min[i] + (other.size[i] - 1) <       min[i]) return false;
        if (      min[i] + (      size[i] - 1) < other.min[i]) return false;
      }
      return true;
    }
  };
  
  bbox_collision_detector():objects_tree(nullptr){}
  bbox_collision_detector(bbox_collision_detector const& other) { *this = other; }
  bbox_collision_detector& operator=(bbox_collision_detector const& other) {
    bboxes_by_object = other.bboxes_by_object;
    objects_tree.reset(other.objects_tree ? new ztree_node(*other.objects_tree) : nullptr);
    return *this;
  }
  
private:
  static const num_bits_type total_bits = CoordinateBits * NumDimensions;
  
  static Coordinate safe_left_shift_one(num_bits_type shift) {
    if (shift >= CoordinateBits) return 0;
    return Coordinate(1) << shift;
  }

  static Coordinate this_many_low_bits(num_bits_type num_bits) {
    return safe_left_shift_one(num_bits) - 1;
  }

  static num_bits_type num_bits_in_integer_that_are_not_leading_zeroes(Coordinate i) {
    int upper_bound = CoordinateBits;
    int lower_bound = -1;
    while(true) {
      int halfway_bit_idx = (upper_bound + lower_bound) >> 1;
      if (halfway_bit_idx == lower_bound) return (lower_bound + 1);

      if (i & ~this_many_low_bits(halfway_bit_idx)) lower_bound = halfway_bit_idx;
      else                                          upper_bound = halfway_bit_idx;
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
  struct zbox {
  private:
    // We ensure that every bit except the ones specifically supposed to be on is off.
    std::array<Coordinate, NumDimensions> coords_;
    std::array<Coordinate, NumDimensions> interleaved_bits_;
    num_bits_type num_low_bits_ignored_;
    bounding_box bbox_;

    void compute_bbox_() {
      bbox_.min = coords_;
      for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
        bbox_.size[i] = safe_left_shift_one(num_bits_ignored_by_dimension(i));
      }
    }
  public:
    
    zbox():num_low_bits_ignored_(total_bits){ for (num_coordinates_type i = 0; i < NumDimensions; ++i) interleaved_bits_[i] = 0; }

    // Named constructors
    static zbox smallest_joint_parent(zbox zb1, zbox zb2) {
      zbox new_box;
      const num_bits_type min_low_bits_ignored = std::max(zb1.num_low_bits_ignored_, zb2.num_low_bits_ignored_);
      for (num_coordinates_type i = NumDimensions - 1; i >= 0; --i) {
        const num_bits_type bits_lower_than_this_part_of_interleaved_bits = i * CoordinateBits;
        const num_bits_type local_first_high_bit_idx = std::max(
          num_bits_in_integer_that_are_not_leading_zeroes(zb1.interleaved_bits_[i] ^ zb2.interleaved_bits_[i]),
          min_low_bits_ignored - bits_lower_than_this_part_of_interleaved_bits
        );
        const bool clearly_done = local_first_high_bit_idx > 0;

        assert_if_ASSERT_EVERYTHING((zb1.interleaved_bits_[i] & ~this_many_low_bits(local_first_high_bit_idx))
                                 == (zb2.interleaved_bits_[i] & ~this_many_low_bits(local_first_high_bit_idx)));

        new_box.interleaved_bits_[i] = zb1.interleaved_bits_[i] & ~this_many_low_bits(local_first_high_bit_idx);
        if (clearly_done) {
          new_box.num_low_bits_ignored_ = local_first_high_bit_idx + bits_lower_than_this_part_of_interleaved_bits;
          for (num_coordinates_type j = 0; j < NumDimensions; ++j) {
            assert_if_ASSERT_EVERYTHING(
                 (zb1.coords_[j] & ~this_many_low_bits(new_box.num_bits_ignored_by_dimension(j)))
              == (zb2.coords_[j] & ~this_many_low_bits(new_box.num_bits_ignored_by_dimension(j)))
            );

            new_box.coords_[j] = zb1.coords_[j] & ~this_many_low_bits(new_box.num_bits_ignored_by_dimension(j));
          }
          new_box.compute_bbox_();
          return new_box;
        }
      }
      new_box.num_low_bits_ignored_ = min_low_bits_ignored;

      assert_if_ASSERT_EVERYTHING(zb1.coords_ == zb2.coords_);

      new_box.coords_ = zb1.coords_;
      new_box.compute_bbox_();
      return new_box;
    }

    static zbox box_from_coords(std::array<Coordinate, NumDimensions> const& coords, num_bits_type num_low_bits_ignored) {
      zbox result;
      result.num_low_bits_ignored_ = num_low_bits_ignored;
      for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
        result.coords_[i] = coords[i] & ~this_many_low_bits(result.num_bits_ignored_by_dimension(i));
      }
      for (num_bits_type bit_within_interleaved_bits = num_low_bits_ignored;
                        bit_within_interleaved_bits < total_bits;
                      ++bit_within_interleaved_bits) {
        const num_bits_type bit_idx_within_coordinates = bit_within_interleaved_bits / NumDimensions;
        const num_coordinates_type which_coordinate    = bit_within_interleaved_bits % NumDimensions;
        const num_bits_type interleaved_bit_array_idx  = bit_within_interleaved_bits / CoordinateBits;
        const num_bits_type interleaved_bit_local_idx  = bit_within_interleaved_bits % CoordinateBits;

        assert_if_ASSERT_EVERYTHING(bit_idx_within_coordinates >= result.num_bits_ignored_by_dimension(which_coordinate));

        result.interleaved_bits_[interleaved_bit_array_idx] |= ((coords[which_coordinate] >> bit_idx_within_coordinates) & 1) << interleaved_bit_local_idx;
      }
      result.compute_bbox_();
      return result;
    }
    
    bool subsumes(zbox const& other)const {
      if (other.num_low_bits_ignored_ > num_low_bits_ignored_) return false;
      for (num_coordinates_type i = num_low_bits_ignored_ / CoordinateBits; i < NumDimensions; ++i) {
        Coordinate mask = ~Coordinate(0);
        if (i == num_low_bits_ignored_ / CoordinateBits) {
          mask &= ~this_many_low_bits(num_low_bits_ignored_ % CoordinateBits);
        }
        if ((interleaved_bits_[i] & mask) != (other.interleaved_bits_[i] & mask)) return false;
      }
      return true;
    }
    bool get_bit(num_bits_type bit)const {
      return interleaved_bits_[bit / CoordinateBits] & safe_left_shift_one(bit % CoordinateBits);
    }
    num_bits_type num_bits_ignored_by_dimension(num_coordinates_type dim)const {
      return (num_low_bits_ignored_ + (NumDimensions - 1) - dim) / NumDimensions;
    }
    // note: gives "size=0" for max-sized things
    bounding_box const& get_bbox()const {
      return bbox_;
    }
    num_bits_type num_low_bits()const {
      return num_low_bits_ignored_;
    }
  };
  
  struct ztree_node;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  struct ztree_node {
    const zbox here;
    ztree_node_ptr child0;
    ztree_node_ptr child1;
    unordered_set<ObjectIdentifier> objects_here;
    
    ztree_node(zbox box):here(box),child0(nullptr),child1(nullptr){}
    ztree_node(ztree_node const& other) { *this = other; }
    ztree_node& operator=(ztree_node const& other) {
      here = other.here;
      child0.reset(other.child0 ? new ztree_node(*other.child0) : nullptr);
      child1.reset(other.child1 ? new ztree_node(*other.child1) : nullptr);
      objects_here = other.objects_here;
      return *this;
    }
  };
  
  
  static void insert_box(ztree_node_ptr& tree, ObjectIdentifier const& obj, zbox box) {
    if (!tree) {
      tree.reset(new ztree_node(box));
      tree->objects_here.insert(obj);
    }
    else {
      if (tree->here.subsumes(box)) {
        if (box.num_low_bits() == tree->here.num_low_bits()) {
          tree->objects_here.insert(obj);
        }
        else {
          if (box.get_bit(tree->here.num_low_bits() - 1)) insert_box(tree->child1, obj, box);
          else                                                  insert_box(tree->child0, obj, box);
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
        insert_box(tree, obj, box);
      }
    }
  }
  
  static void delete_object(ztree_node_ptr& tree, ObjectIdentifier const& obj, bounding_box const& bbox) {
    if (!tree) return;
    if (tree->here.get_bbox().overlaps(bbox)) {
      tree->objects_here.erase(obj);
      delete_object(tree->child0, obj, bbox);
      delete_object(tree->child1, obj, bbox);
      
      if (tree->objects_here.empty()) {
        if (tree->child0) {
          if (!tree->child1) {
            ztree_node_ptr dead_tree;
            dead_tree.swap(tree);
            dead_tree->child0.swap(tree);
          }
        }
        else {
          // old 'child1' a.k.a. new 'tree' could be null
          ztree_node_ptr dead_tree;
          dead_tree.swap(tree);
          dead_tree->child1.swap(tree);
        }
      }
    }
  }
  
  void zget_objects_overlapping(ztree_node const* tree, unordered_set<ObjectIdentifier>& results, bounding_box const& bbox)const {
    if (tree && tree->here.get_bbox().overlaps(bbox)) {
      for (const ObjectIdentifier obj : tree->objects_here) {
        auto bbox_iter = bboxes_by_object.find(obj);      

        assert_if_ASSERT_EVERYTHING(bbox_iter != bboxes_by_object.end());

        if (bbox_iter->second.overlaps(bbox)) results.insert(obj);
      }
      zget_objects_overlapping(tree->child0.get(), results, bbox);
      zget_objects_overlapping(tree->child1.get(), results, bbox);
    }
  }
  
  unordered_map<ObjectIdentifier, bounding_box> bboxes_by_object;
  ztree_node_ptr objects_tree;
  
public:

  void insert(ObjectIdentifier const& id, bounding_box const& bbox) {
    caller_correct_if(
      bboxes_by_object.insert(std::make_pair(id, bbox)).second,
      "bbox_collision_detector::insert() requires for your safety that the id "
      "is not already in this container.  Use .erase() or .exists() if you need "
      "any particular behaviour in this case."
    );
    Coordinate max_dim = bbox.size[0];
    for (num_coordinates_type i = 1; i < NumDimensions; ++i) {
      if (bbox.size[i] > max_dim) max_dim = bbox.size[i];
    }
    // max_dim - 1: power-of-two-sized objects easily squeeze into the next smaller category.
    const num_bits_type exp = num_bits_in_integer_that_are_not_leading_zeroes(max_dim - 1);
    num_coordinates_type dimensions_we_can_single = 0;
    num_coordinates_type dimensions_we_can_double = 0;
    const Coordinate base_box_size = safe_left_shift_one(exp);
    const Coordinate used_bits_mask = ~(base_box_size - 1);

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
    // be more complicated).  These are "dimensions_we_can_single" because
    // they're one times the base_box_size.  This is the best case.
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
    // it in "dimensions_we_can_double".
    //
    // All the dimensions in between will be split into two zboxes,
    // each of width base_box_size, for a total width of
    // twice base_box_size.  This is the worst case.
    for (num_coordinates_type i = NumDimensions - 1; i >= 0; --i) {
      if ((bbox.min[i] & used_bits_mask) + base_box_size >= bbox.min[i] + bbox.size[i]) ++dimensions_we_can_single;
      else break;
    }
    for (num_coordinates_type i = 0; i < NumDimensions - dimensions_we_can_single; ++i) {
      if (!(bbox.min[i] & base_box_size)) ++dimensions_we_can_double;
      else break;
    }
    #ifdef ZTREE_TESTING
    std::cerr << dimensions_we_can_single << "... " << dimensions_we_can_double << "...\n";
    #endif
    const int number_of_zboxes_to_use_if_necessary = 1 << (NumDimensions - dimensions_we_can_single - dimensions_we_can_double);
    
    for (int i = 0; i < number_of_zboxes_to_use_if_necessary; ++i) {
      std::array<Coordinate, NumDimensions> coords = bbox.min;
      for (num_coordinates_type j = dimensions_we_can_double; j < NumDimensions - dimensions_we_can_single; ++j) {
        // By checking this bit of "i" arbitrarily, by the last time
        // we get through the "number_of_zboxes_to_use_if_necessary" loop,
        // we have yielded every combination of possibilities of
        // each dimension that varies varying (between +0 and +base_box_size).
        if (i & (1 << (j - dimensions_we_can_double))) {
          coords[j] += base_box_size;
        }
      }
      const zbox zb = zbox::box_from_coords(coords, exp * NumDimensions + dimensions_we_can_double);
      if (zb.get_bbox().overlaps(bbox)) {
        insert_box(objects_tree, id, zb);
      }
    }
  }
  
  bool exists(ObjectIdentifier const& id)const {
    return (bboxes_by_object.find(id) != bboxes_by_object.end());
  }
  
  bool erase(ObjectIdentifier const& id) {
    auto bbox_iter = bboxes_by_object.find(id);
    if (bbox_iter == bboxes_by_object.end()) return false;
    delete_object(objects_tree, id, bbox_iter->second);
    bboxes_by_object.erase(bbox_iter);
    return true;
  }

  bounding_box const* find_bounding_box(ObjectIdentifier const& id)const {
    return find_as_pointer(bboxes_by_object, id);
  }
  
  void get_objects_overlapping(unordered_set<ObjectIdentifier>& results, bounding_box const& bbox)const {
    zget_objects_overlapping(objects_tree.get(), results, bbox);
  }
  
private:
  struct generalized_object_collection_walker;

public:
  class generalized_object_collection_handler {
  public:
    virtual void handle_new_find(ObjectIdentifier) {}
    
    // Any bbox for which this one of these functions returns false is ignored.
    // The static one is checked only once for each box; the dynamic one is checked
    // both before the box is added to the frontier and when it comes up.
    virtual bool should_be_considered__static(bounding_box const&)const { return true; }
    virtual bool should_be_considered__dynamic(bounding_box const&)const { return true; }
    
    // returns "true" if the first one should be considered first, "false" otherwise.
    // This is primarily intended to allow the caller to run through objects in a specific
    // in-space order to get the first collision in some direction. One can combine it with
    // a dynamic should_be_considered function to stop looking in boxes that are entirely
    // after the first known collision.
    //
    // This function must be a Strict Weak Ordering and not change its behavior.
    // This function is only a heuristic; we don't guarantee that the boxes will be handled
    // exactly in the given order.
    virtual bool bbox_ordering(bounding_box const&, bounding_box const&)const { return false; } // arbitrary order
    
    // This is called often (specifically, every time we call a non-const function of handler);
    // if it returns true, we stop right away.
    // Some handlers might know they're done looking before the should_be_considered boxes
    // run out; this just makes that more efficient.
    virtual bool done()const { return false; }
    
    unordered_set<ObjectIdentifier> const& get_found_objects()const { return found_objects; }
  private:
    unordered_set<ObjectIdentifier> found_objects;
    friend struct generalized_object_collection_walker;
  };

private:
  struct generalized_object_collection_walker {
    generalized_object_collection_handler* handler;
    unordered_map<ObjectIdentifier, bounding_box> const& bboxes_by_object;
    generalized_object_collection_walker(generalized_object_collection_handler* handler, unordered_map<ObjectIdentifier, bounding_box> const& bboxes_by_object):handler(handler),bboxes_by_object(bboxes_by_object),frontier(set_compare(handler)){}
    
    struct set_compare {
      generalized_object_collection_handler* handler;
      set_compare(generalized_object_collection_handler* handler):handler(handler){}
      
      bool operator()(ztree_node const* z1, ztree_node const* z2)const {
        return handler->bbox_ordering(z1->here.get_bbox(), z2->here.get_bbox());
      }
    };
    
    // will only contain nonnull pointers (since it would be a waste of time to
    // insert null pointers into the set).
    //
    // TODO: we can probably find a more optimal data structure to use here than std::set.
    std::set<ztree_node const*, set_compare> frontier;
    
    void try_add(ztree_node* z) {
      if (z && handler->should_be_considered__static(z->here.get_bbox()) && handler->should_be_considered__dynamic(z->here.get_bbox())) {
        frontier.insert(z);
      }
    }
      
    bool process_next() {
      if (frontier.empty()) return false;
      
      ztree_node const* next = *frontier.begin();
      frontier.erase(frontier.begin());
      
      if (handler->should_be_considered__dynamic(next->here.get_bbox())) {
        for (const ObjectIdentifier obj : next->objects_here) {
          auto bbox_iter = bboxes_by_object.find(obj);

          assert_if_ASSERT_EVERYTHING(bbox_iter != bboxes_by_object.end());

          if (handler->should_be_considered__static(bbox_iter->second) && handler->should_be_considered__dynamic(bbox_iter->second)) {
            if (handler->found_objects.insert(obj).second) {
              handler->handle_new_find(obj);
              if (handler->done()) return false;
            }
          }
        }
        try_add(next->child0.get());
        try_add(next->child1.get());
      }
      
      return true;
    }
  };
    
public:
  void get_objects_generalized(generalized_object_collection_handler* handler)const {
    generalized_object_collection_walker data(handler, bboxes_by_object);
    
    data.try_add(objects_tree.get());
    while(data.process_next()){}
  }
};

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
  Z1/Z2, with B ignored bits
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

#endif

