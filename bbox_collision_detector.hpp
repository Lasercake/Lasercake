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

template<num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
struct bounding_box {
private:
  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;
public:
  // This bounding_box's coordinates are deliberately modulo.
  // This lets us naturally represent either signed or unsigned
  // spaces, that themselves wrap or don't (if they don't wrap,
  // then the callers just won't make any bounding_boxes that cross
  // their self-imposed boundary over which it doesn't wrap).
  typedef Coordinate coordinate_type;
  static const num_bits_type num_coordinate_bits = CoordinateBits;
  static const num_coordinates_type num_dimensions = NumDimensions;
  // Example with NumDimensions=1: bbox A={min=2,size=3}, B={min=5,size=1}
  // are adjacent but not intersecting.
  // "size=0" actually denotes "size = 2**CoordinateBits" (which probably
  // doesn't quite fit in a coordinate.)  This lets us handle objects
  // that can take up most or all the width in the world.
  // This makes width-0 objects impossible, thought it's dubious that they
  // would intersect something anyway.
  // TODO: we're also doing size - 1 everywhere; should we change size
  // to *be* one less than it is now? (interface change)
  std::array<Coordinate, NumDimensions> min;
  std::array<Coordinate, NumDimensions> size;

  bool overlaps(bounding_box const& other)const {
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      if ( (other.size[i] - 1) <       min[i] - other.min[i]
        && (      size[i] - 1) < other.min[i] -       min[i]) return false;
    }
    return true;
  }

  bool operator==(bounding_box const& other)const {
    return min == other.min && size == other.size;
  }
  bool operator!=(bounding_box const& other)const {
    return !(*this == other);
  }

  friend inline std::ostream& operator<<(std::ostream& os, bounding_box const& bb) {
    os << '[';
    for (size_t i = 0; i < bb.min.size(); ++i) {
      if(i != 0) os << ", ";
      os << bb.min[i] << '+' << bb.size[i];
    }
    os << ']';
    return os;
  }
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
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class visitor;

// ObjectIdentifier needs hash and == and to be freely copiable. So, ints will do, pointers will do...
// CoordinateBits should usually be 32 or 64. I don't know if it works for other values.
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
  void insert(ObjectIdentifier const& id, bounding_box const& bbox);

  // exists returns true iff there is an object of this id in this detector
  bool exists(ObjectIdentifier const& id)const {
    return (bboxes_by_object_.find(id) != bboxes_by_object_.end());
  }

  // erase returns true iff there was an object of this id (now deleted of course).
  bool erase(ObjectIdentifier const& id);

  // find_bounding_box returns non-null iff there is an object of this id.
  bounding_box const* find_bounding_box(ObjectIdentifier const& id)const {
    auto i = bboxes_by_object_.find(id);
    if(i == bboxes_by_object_.end()) return nullptr;
    else return &(i->second.bbox);
  }

  void get_objects_overlapping(std::vector<ObjectIdentifier>& results, bounding_box const& bbox)const;

  // Derive from
  //   class collision_detector::visitor<>
  // and override its virtual functions
  // to make search() do something interesting for you.
  void search(visitor* handler)const;
  
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

  friend class zbox_tester;
  friend class bbox_collision_detector_tester;
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

