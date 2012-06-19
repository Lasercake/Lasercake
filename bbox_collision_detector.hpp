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
#include <boost/scoped_ptr.hpp>
#include "utils.hpp"

namespace collision_detector {
typedef ptrdiff_t num_bits_type;
typedef ptrdiff_t num_coordinates_type;

template<num_bits_type CoordinateBits>
struct coordinate_type_from_bits {
  typedef typename boost::uint_t<CoordinateBits>::fast type;
};


// This bounding_box's coordinates are deliberately modulo.
// This lets us naturally represent either signed or unsigned
// spaces, that themselves wrap or don't (if they don't wrap,
// then the callers just won't make any bounding_boxes that cross
// their self-imposed boundary over which it doesn't wrap).
//
// Example with NumDimensions=1: bbox
//   A = {min=2,size=3} = {min=2, size_minus_one=2}
//   B = {min=5,size=1} = {min=5, size_minus_one=0}
// are adjacent but not intersecting.
//
// All of these bounding-boxes contain some space; they are at least 1x1x...
//
// Their maximum size is the width of the entire coordinate space.  This
// doesn't quite fit in the unsigned integer.  Thus it is only correct
// to use size_minus_one for most arithmetic, rather than the size itself.
//
// These 1..space-size sizes are a design decision.  Degenerate zero-size
// bounding-boxes are not much use to bbox_collision_detector.  Max-size
// bounding-boxes are.  We can't have both because of unsigned integer
// ranges.
template<num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class bounding_box {
private:
  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;
public:
  typedef Coordinate coordinate_type;
  static const num_bits_type num_coordinate_bits = CoordinateBits;
  static const num_coordinates_type num_dimensions = NumDimensions;
  typedef std::array<Coordinate, NumDimensions> coordinate_array;

  // The default-constructor can reasonably be private or public; it creates
  // a bounding_box with undefined values.  We choose public (the default)
  // in case default-constructibility is useful to anyone for their
  // containers or similar.

  // Named "constructors".
  static bounding_box min_and_size_minus_one(coordinate_array min, coordinate_array size_minus_one) {
    bounding_box result;
    result.min_ = min;
    result.size_minus_one_ = size_minus_one;
    return result;
  }
  static bounding_box min_and_max(coordinate_array min, coordinate_array max) {
    bounding_box result;
    result.min_ = min;
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      result.size_minus_one_[i] = max[i] - min[i];
    }
    return result;
  }
  static bounding_box size_minus_one_and_max(coordinate_array size_minus_one, coordinate_array max) {
    bounding_box result;
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      result.min_[i] = max[i] - size_minus_one[i];
    }
    result.size_minus_one_ = size_minus_one;
    return result;
  }

  // Accessors (with array and indexing variants).
  coordinate_array min()const { return min_; }
  coordinate_array size_minus_one()const { return size_minus_one_; }
  coordinate_array max()const {
    coordinate_array result;
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      result[i] = min_[i] + size_minus_one_[i];
    }
    return result;
  }
  coordinate_type min(num_coordinates_type dim)const { return min_[dim]; }
  coordinate_type size_minus_one(num_coordinates_type dim)const { return size_minus_one_[dim]; }
  coordinate_type max(num_coordinates_type dim)const { return min_[dim] + size_minus_one_[dim]; }

  // Utility functions.
  bool overlaps(bounding_box const& other)const {
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      if ( other.size_minus_one(i) <       min(i) - other.min(i)
        &&       size_minus_one(i) < other.min(i) -       min(i)) return false;
    }
    return true;
  }

  bool operator==(bounding_box const& other)const {
    return min_ == other.min_ && size_minus_one_ == other.size_minus_one_;
  }
  bool operator!=(bounding_box const& other)const {
    return !(*this == other);
  }

  friend inline std::ostream& operator<<(std::ostream& os, bounding_box const& bb) {
    os << '[';
    for (num_coordinates_type i = 0; i < bounding_box::num_dimensions; ++i) {
      if(i != 0) os << ", ";
      os << bb.min(i) << '+' << bb.size_minus_one(i);
    }
    os << ']';
    return os;
  }

private:
  coordinate_array min_;
  coordinate_array size_minus_one_;
};

namespace impl {
  typedef ptrdiff_t num_zboxes_type;

  //to make get_objects_overlapping faster (faster uniquification than hash-set)
  //If the number of objects is n, each has a unique numeric ID in [0,n).
  typedef uint32_t numeric_id_type;
  template<num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
  struct object_metadata {
    bounding_box<CoordinateBits, NumDimensions> bbox;
    numeric_id_type numeric_id;

    object_metadata(bounding_box<CoordinateBits, NumDimensions> const& bbox, numeric_id_type numeric_id) : bbox(bbox), numeric_id(numeric_id) {}
  };

  template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
  struct ztree_node;

  struct access_visitor_found_objects;
  struct zbox_debug_visualizer;
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class visitor;

// bbox_collision_detector is a collection of pairs of
//   ObjectIdentifier and bounding_box<CoordinateBits, NumDimensions>
// which has O(1) ObjectIdentifier -> bounding_box
// and roughly O(log(n)) bounding_box -> overlapping ObjectIdentifiers.
//
// bounding_boxes are in NumDimensions-dimensional Euclidean space
// that is modulo 2**CoordinateBits.
//
// ObjectIdentifier needs hash and == and a copy-constructor.
// CoordinateBits should usually be 32 or 64. We haven't tested other values.
//
// The per-item memory overhead is large.
// It's somewhere on the order of a hectobyte (100 bytes),
// depending on CoordinateBits*NumDimensions, on sizeof(pointer),
// and on implementation details.
//
// In O() complexities, let "depth" be
//   typical case  min(log(n), CoordinateBits*NumDimensions)
//   worst case    min(n, CoordinateBits*NumDimensions)
//
// "depth" doesn't suffer from unnecessarily large CoordinateBits; if
// all your objects fit in a bounding box of size (max size among all
// dimensions) 2**b, then substitute (b+1) for CoordinateBits in the
// definition of depth.  (This falls naturally out of the ztree
// implementation.)
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class bbox_collision_detector {
  static_assert(NumDimensions >= 0, "You can't make a space with negative dimensions!");
  static_assert(NumDimensions < std::numeric_limits<impl::num_zboxes_type>::digits - 1,
    "We don't permit so many dimensions that one bounding_box might need more zboxes than we can count.");
  static_assert(CoordinateBits >= 0, "You can't have an int type with negative bits!");

  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;

public:
  typedef Coordinate coordinate_type;
  static const num_bits_type num_coordinate_bits = CoordinateBits;
  static const num_coordinates_type num_dimensions = NumDimensions;

  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef collision_detector::visitor<ObjectIdentifier, CoordinateBits, NumDimensions> visitor;
  
  bbox_collision_detector();
  bbox_collision_detector(bbox_collision_detector const& other) { *this = other; }
  bbox_collision_detector& operator=(bbox_collision_detector const& other);
  ~bbox_collision_detector();

  // insert throws std::logic_error iff the id already existed in this detector.
  // To avoid an exception, use detector.erase(id) or if(detector.exists(id))
  // depending on your needs.
  //
  // O(depth)
  void insert(ObjectIdentifier const& id, bounding_box const& bbox);

  // exists returns true iff there is an object of this id in this detector
  //
  // O(1)
  bool exists(ObjectIdentifier const& id)const {
    return (bboxes_by_object_.find(id) != bboxes_by_object_.end());
  }

  // erase returns true iff there was an object of this id (now deleted of course).
  //
  // O(depth)
  bool erase(ObjectIdentifier const& id);

  // size returns the number of objects in the bbox_collision_detector.
  //
  // O(1)
  size_t size()const {
    return bboxes_by_object_.size();
  }

  // find_bounding_box returns non-null iff there is an object of this id.
  //
  // O(1)
  bounding_box const* find_bounding_box(ObjectIdentifier const& id)const {
    auto i = bboxes_by_object_.find(id);
    if(i == bboxes_by_object_.end()) return nullptr;
    else return &(i->second.bbox);
  }

  // O(max(m, depth)) where m is the number of things collected plus other
  // nearby objects.  Things that are as far away from the bbox in a dimension
  // as its maximum width among all dimensions never count as nearby.
  void get_objects_overlapping(std::vector<ObjectIdentifier>& results, bounding_box const& bbox)const;

  // Derive from
  //   class collision_detector::visitor<>
  // and override its virtual functions
  // to make search() do something interesting for you.
  //
  // Search's complexity depends on your handler.
  void search(visitor* handler)const;

  // This can be useful for debugging the efficiency of your
  // bbox_collision_detector usage (though the output is not particularly
  // self-explanatory, alas).
  //
  // O(n)
  void print_debug_summary_information(std::ostream& os)const;
  
private:
  typedef impl::ztree_node<ObjectIdentifier, CoordinateBits, NumDimensions> ztree_node;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;

  typedef impl::object_metadata<CoordinateBits, NumDimensions> object_metadata;
  typedef std::pair<const ObjectIdentifier, object_metadata> id_and_bbox_type;
  typedef id_and_bbox_type* id_and_bbox_ptr;
  std::unordered_map<ObjectIdentifier, object_metadata> bboxes_by_object_;
  ztree_node_ptr objects_tree_;
  //(implicitly) indexed by numeric_id
  std::vector<id_and_bbox_ptr> objects_sequence_;

  friend struct impl::zbox_debug_visualizer;
};

// This describes how to traverse the elements of a bbox_collision_detector
//   optionally excluding areas you're uninterested in
//   and optionally in an order partly of your choosing.
//
// a la http://www.boost.org/doc/libs/1_48_0/libs/graph/doc/visitor_concepts.html
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class visitor {
public:
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  
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
  // This function is only a heuristic; we don't guarantee that the boxes will be handled
  // exactly in the given order.
  virtual bool bbox_less_than(bounding_box const&, bounding_box const&)const { return false; } // arbitrary order

  // This is called often (specifically, every time we call a non-const function of handler);
  // if it returns true, we stop right away.
  // Some handlers might know they're done looking before the should_be_considered boxes
  // run out; this just makes that more efficient.
  virtual bool should_exit_early()const { return false; }

  std::unordered_set<ObjectIdentifier> const& get_found_objects()const { return found_objects_; }
private:
  std::unordered_set<ObjectIdentifier> found_objects_;
  friend struct impl::access_visitor_found_objects;
};

}

using collision_detector::bbox_collision_detector;


//This promise to instantiate a template elsewhere required object_or_tile_identifier
//to be a complete type, according to GCC.  Thankfully it seems to be unnecessary
//to say this.  There also isn't much template code in this header that the compiler
//could waste time compiling in every file.
#if 0
struct object_or_tile_identifier;
namespace collision_detector {
extern template class bbox_collision_detector<object_or_tile_identifier, 64, 3>;
}
#endif

#endif

