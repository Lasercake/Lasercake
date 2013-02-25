/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

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


#include <boost/integer.hpp>
#include <boost/integer/static_log2.hpp>
#include <limits>
#include <stack>
#include <cassert>
#include <cstdlib>
#include <boost/dynamic_bitset.hpp>

#include "bbox_collision_detector.hpp"
#include "bbox_collision_detector_iteration.hpp"
#include "borrowed_bitset.hpp"

using std::pair;
using std::make_pair;

// a hack to help test this code a bit more reliably
#ifdef BBOX_COLLISION_DETECTOR_IMPL_TREAT_AS_HEADER
#define INLINE_IF_HEADER inline
#else
#define INLINE_IF_HEADER
#endif

namespace collision_detector {
namespace impl {

namespace /* anonymous */ {
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
  typedef typename impl::ztree_node_ptr<ztree_node>::type ztree_node_ptr;
  typedef impl::object_metadata<CoordinateBits, NumDimensions> object_metadata;
  typedef pair<const ObjectIdentifier, object_metadata> id_and_bbox_type;
  typedef id_and_bbox_type* id_and_bbox_ptr;
  
  
  // A call to insert_zboxes_from_bbox() might be more clearly written
  //   for(zbox zb : zboxes_from_bbox(bbox)) {
  //     insert_zbox(objects_tree, id, zb);
  //   }
  // The only reason we combine insert_zbox and zboxes_from_bbox is so
  // we don't have to construct the intermediate container structure.
  static void insert_zboxes_from_bbox(ztree_node_ptr& objects_tree, id_and_bbox_ptr id_and_bbox) {
    bounding_box const& bbox = id_and_bbox->second.bbox;

    const Coordinate max_width_minus_one = max_in_array_of_unsigned(bbox.size_minus_one());
    // max_width - 1: power-of-two-sized objects easily squeeze into the next smaller category.
    // i.e., exp = log2_rounding_up(max_width)
    const num_bits_type exp = num_bits_in_integer_that_are_not_leading_zeroes(max_width_minus_one);
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
    if(base_box_size == 0) {
      num_dims_using_one_zbox_of_exactly_base_box_size = NumDimensions;
    }
    else {
      for (num_coordinates_type i = NumDimensions - 1; i >= 0; --i) {
        if (bbox.size_minus_one(i) <= (base_box_size - 1) - (bbox.min(i) & (base_box_size - 1))) {
          ++num_dims_using_one_zbox_of_exactly_base_box_size;
        }
        else {
          break;
        }
      }
      for (num_coordinates_type i = 0; i < NumDimensions - num_dims_using_one_zbox_of_exactly_base_box_size; ++i) {
        if (!(bbox.min(i) & base_box_size)) {
          ++num_dims_using_one_zbox_of_twice_base_box_size;
        }
        else {
          break;
        }
      }
    }
    num_dims_using_two_zboxes_each_of_base_box_size = NumDimensions - num_dims_using_one_zbox_of_exactly_base_box_size - num_dims_using_one_zbox_of_twice_base_box_size;

    const num_zboxes_type number_of_zboxes_to_use_if_necessary = num_zboxes_type(1) << num_dims_using_two_zboxes_each_of_base_box_size;

    for (num_zboxes_type i = 0; i < number_of_zboxes_to_use_if_necessary; ++i) {
      array<Coordinate, NumDimensions> coords = bbox.min();
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
  static void insert_zbox(ztree_node_ptr& tree, id_and_bbox_ptr id_and_bbox, zbox box) {
    if (!tree) {
      tree.reset(lasercake_nice_new<ztree_node>(box));
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
        ztree_node_ptr new_tree(lasercake_nice_new<ztree_node>(zbox::smallest_joint_parent(tree->here, box)));

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

  static void delete_object(ztree_node_ptr& tree, id_and_bbox_ptr id_and_bbox) {
    if (!tree) return;
    if (tree->here.overlaps(id_and_bbox->second.bbox)) {
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
  static void zget_objects_overlapping(
        ztree_node const* tree,
        std::vector<ObjectIdentifier>& results,
        borrowed_bitset& bitmap_indicating_found_results,
        bounding_box const& bbox) {
    if (tree && tree->here.overlaps(bbox)) {
      for(id_and_bbox_ptr id_and_bbox : tree->objects_here) {
        if (!bitmap_indicating_found_results.test(id_and_bbox->second.numeric_id)) {
          bitmap_indicating_found_results.set(id_and_bbox->second.numeric_id);
          if(id_and_bbox->second.bbox.overlaps(bbox)) {
            results.push_back(id_and_bbox->first);
          }
        }
      }
      zget_objects_overlapping(tree->child0.get(), results, bitmap_indicating_found_results, bbox);
      zget_objects_overlapping(tree->child1.get(), results, bitmap_indicating_found_results, bbox);
    }
  }

  static void look_at_counts_impl(ztree_node const* tree, std::map<long, long>& counts) {
    if(tree) {
      ++counts.insert(pair<long, long>(tree->objects_here.size(), 0)).first->second;
      look_at_counts_impl(tree->child0.get(), counts);
      look_at_counts_impl(tree->child1.get(), counts);
    }
  }
  static void look_at_counts(ztree_node const* tree, std::ostream& os) {
    std::map<long, long> counts;
    look_at_counts_impl(tree, counts);
    for(auto const& c : counts) {
      os << c.first << ": " << c.second << "\n";
    }
  }
};

} /* end anonymous namespace */
} /* end namespace impl */


template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
void bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
insert(ObjectIdentifier const& id, bounding_box const& bbox) {
  auto numeric_id = objects_sequence_.size();
  caller_correct_if(numeric_id == impl::numeric_id_type(numeric_id),
                    "insert(): bbox_collision_detector can't exceed its max_size().");
  auto iter_and_new = bboxes_by_object_.insert(id_and_bbox_type(id, object_metadata(bbox, numeric_id)));
  caller_correct_if(
    iter_and_new.second,
    "bbox_collision_detector::insert() requires for your safety that the id "
    "is not already in this container.  Use .erase() or .exists() if you need "
    "any particular behaviour in this case."
  );
  objects_sequence_.push_back(&*iter_and_new.first);
  impl::ztree_ops<ObjectIdentifier, CoordinateBits, NumDimensions>::insert_zboxes_from_bbox(objects_tree_, &*iter_and_new.first);
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
  ++revision_count_;
#endif
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bool bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
erase(ObjectIdentifier const& id) {
  const auto bbox_iter = bboxes_by_object_.find(id);
  if (bbox_iter == bboxes_by_object_.end()) return false;
  impl::ztree_ops<ObjectIdentifier, CoordinateBits, NumDimensions>::delete_object(objects_tree_, &*bbox_iter);
  impl::numeric_id_type numeric_id_to_delete = bbox_iter->second.numeric_id;
  //move the last ID into this slot, so the IDs stay packed.
  //(a no-op when this was the last ID, but why waste a branch misprediction by using 'if')
  id_and_bbox_ptr replace_it_with = objects_sequence_.back();
  replace_it_with->second.numeric_id = numeric_id_to_delete;
  objects_sequence_[numeric_id_to_delete] = replace_it_with;
  objects_sequence_.pop_back();
  bboxes_by_object_.erase(bbox_iter);
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
  ++revision_count_;
#endif
  return true;
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
void bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
get_objects_overlapping(std::vector<ObjectIdentifier>& results, bounding_box const& bbox)const {
  // If it were not O(this->size()) we would get our this->size() zeroes like this:
  // impl::dynamic_bitset_type bitmap_indicating_found_results(objects_sequence_.size());

  borrowed_bitset bitmap_indicating_found_results(objects_sequence_.size());

  impl::ztree_ops<ObjectIdentifier, CoordinateBits, NumDimensions>::zget_objects_overlapping(
        objects_tree_.get(), results, bitmap_indicating_found_results, bbox);
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
void bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
print_debug_summary_information(std::ostream& os)const {
  os << this->size() << " objects, in piles of [[[\n";
  impl::ztree_ops<ObjectIdentifier, CoordinateBits, NumDimensions>::look_at_counts(objects_tree_.get(), os);
  os << "]]]\n";
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>&
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::operator=(bbox_collision_detector const& other) {
  bboxes_by_object_ = other.bboxes_by_object_;
  objects_sequence_ = other.objects_sequence_;
  objects_tree_.reset(other.objects_tree_ ? lasercake_nice_new<ztree_node>(*other.objects_tree_) : nullptr);
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
  revision_count_ = other.revision_count_;
#endif
  return *this;
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
bbox_collision_detector()
: objects_tree_(nullptr)
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
  , revision_count_(0)
#endif
{}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
INLINE_IF_HEADER
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
~bbox_collision_detector()
{}

} /* end namespace collision_detector */

#undef INLINE_IF_HEADER

#ifndef BBOX_COLLISION_DETECTOR_IMPL_TREAT_AS_HEADER
#include "../world.hpp"
namespace collision_detector {
template class bbox_collision_detector<object_or_tile_identifier, 64, 3>;
template class bbox_collision_detector<object_identifier, 64, 3>;

#if !LASERCAKE_NO_SELF_TESTS
// for bbox_collision_detector_tests.cpp
template class bbox_collision_detector<int32_t, 32, 1>;
template class bbox_collision_detector<int32_t, 32, 2>;
template class bbox_collision_detector<int32_t, 64, 3>;
#endif
}
#endif
