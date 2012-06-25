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
#include "../utils.hpp"

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

  template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions, typename GetCost>
  struct iteration_types;
}

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
  
  bbox_collision_detector();
  bbox_collision_detector(bbox_collision_detector const& other) { *this = other; }
  bbox_collision_detector& operator=(bbox_collision_detector const& other);
  ~bbox_collision_detector();

  // insert throws std::logic_error iff the id already existed in this detector.
  // To avoid an exception, use detector.erase(id) or if(detector.exists(id))
  // depending on your needs.
  //
  // Invalidates all iterators into this container.
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
  // Invalidates all iterators into this container.
  //
  // O(depth)
  bool erase(ObjectIdentifier const& id);

  // size returns the number of objects in the bbox_collision_detector.
  //
  // O(1)
  impl::numeric_id_type size()const {
    return impl::numeric_id_type(bboxes_by_object_.size());
  }
  impl::numeric_id_type max_size()const {
    return std::numeric_limits<impl::numeric_id_type>::max();
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


  // *** Custom ordered and/or filtered search and iteration ***
  //
  // This documentation begins with an example:
  //
  // Iterate objects by how close their centres are to a given point:
  //
  //   for(obj_id x : detector.iterate(my_distance_measurer(point))) {
  //     [...]
  //   }
  //
  // where
  //
  //   typedef [...] obj_id;
  //   typedef [...] point_type;
  //   typedef num_coordinates_type which_dimension_type;
  //   typedef bbox_collision_detector<obj_id, 32, 2> detector_type;
  //   detector_type detector;
  //
  //   // utility function:
  //   uint32_t dimension_distance(which_dimension_type dim, point_type point, detector_type::bounding_box bbox) {
  //     // (assuming your world is not modulo)
  //     return point_(dim) < bbox.min(dim) ? bbox.min(dim) - point_(dim)
  //          : point_(dim) > bbox.max(dim) ? point_(dim) - bbox.max(dim)
  //          : 0;
  //   }
  //   struct my_distance_measurer {
  //     point_type point_;
  //     my_distance_measurer(point_type point) : point_(point) {}
  //
  //     // Represent cost as Euclidean distance squared
  //     // (Squared because why spend the time and loss-of-precision
  //     // by taking the square root... square root of nonnegative reals
  //     // is order-preserving [strictly monotonic], and square root with
  //     // rounding error is also [non-strictly] monotonic.)
  //     typedef uint64_t cost_type;
  //     cost_type min_cost(detector::bounding_box bbox) {
  //       const uint64_t xdist = dimension_distance(0, point_, bbox);
  //       const uint64_t ydist = dimension_distance(1, point_, bbox);
  //       // This could overflow; use larger ints, or make sure it doesn't
  //       // (even given large bounding-boxes you did not directly create).
  //       // For example, you could lose precision by dividing each by 2 or
  //       // so, or use doubles and lose all perfection but prevent overflow,
  //       // or filter out all faraway things[*].
  //       return xdist*xdist + ydist*ydist;
  //     }
  //     cost_type cost(obj_id obj, detector_type::bounding_box bbox) {
  //       // pretend get_center_of_object() exists
  //       const point obj_location = obj.get_center_of_object();
  //       // Be careful of overflow here too. You must use the same
  //       // distance/cost scale as min_cost does.
  //       const uint64_t xdist = std::abs(point_(0) - obj_location(0));
  //       const uint64_t ydist = std::abs(point_(1) - obj_location(1));
  //       return xdist*xdist + ydist*ydist;
  //     }
  //   };
  //
  // [*]filter out faraway things in my_distance_measurer perhaps like:
  //     typedef uint64_t cost_type;
  //     // for each of the two cost functions,
  //     boost::optional<cost_type> fn([...]) {
  //       [...]
  //       const uint32_t too_far = 3000;
  //       if(xdist > too_far || ydist > too_far) return boost::none;
  //       else return xdist*xdist + ydist*ydist;
  //     }
  //
  //
  // You must include bbox_collision_detector_iteration.hpp to use these
  // members (since they're not always needed and it's a fair amount of code).
  //
  // GetCost must be a class you provide that has two methods:
  //     min_cost(bounding_box)
  //     cost(ObjectIdentifier, bounding_box)
  // and type (perhaps typedef) member
  //     cost_type
  // .
  //
  // cost_type must be LessThanComparable.  cost() and min_cost() must return
  // either (1) a type convertible to cost_type, or
  //        (2) a type with bool(t) and *t
  //              with *t valid if bool(t), and *t convertible to cost_type
  //            (such as boost::optional<cost_type> or cost_type*).
  // The methods must also obey the mathematical rules described below.
  //
  // iterate() will iterate the container, in the order implied by those
  // return values (least to greatest), with empty return-values meaning to
  //     filter out that object (cost()), or
  //     partially skip that region (min_cost())
  //         (if you write min_cost() correctly, the only effect is
  //          optimization, and if you write it wrong, it's undefined
  //          behavior[**])
  //
  // The iterators' value_type is a struct containing
  //     ObjectIdentifier const& object;
  //     bounding_box const& bbox;
  //     cost_type cost;
  //
  // find_least() is a simpler interface when you just want the first (least)
  // match; it returns a boost::optional<value_type>.
  //
  // filter() is a hybrid between these and get_objects_overlapping().
  // Its GetCost functions must return bool or boolean-convertible
  // (true meaning keep) and needn't typedef cost_type (but if they do
  // typedef cost_type, then functions returning cost_type are treated
  // as true so that filter() is fairly compatible with regular GetCost
  // functors).
  //
  // Iterators returned by iterate() are invalidated by any change in the
  // bbox_collision_detector.  They are input iterators: an iterator from any
  // given call to iterate() can only be traversed once.  Furthermore, they
  // take up space related to the size of the bbox_collision_detector; it's
  // better not to have a ton of outstanding iterators at once.
  //
  // The worst-case complexity to iterate through a bbox_collision_detector is
  //     O(n) calls to min_cost() + O(n) calls to cost() + O(n log n) calls to
  //         operator<.
  // In practice, especially if you specify a good min_cost() and don't
  // want to examine every object in the bbox_collision_detector, it can be
  // significantly faster than that.  For example, the best nontrivial case
  // for find_least() is probably
  //     O(log n) * (calls to min_cost() + calls to cost() + constant).
  //
  // Your GetCost must[**] follow these rules:
  //
  // 1. "LessThanComparable":
  // cost_type's operator< must, as usual, be transitive and antisymmetric.
  //
  // In the following rules, consider a "none" return value from cost() or
  // min_cost() as an infinity, greater than all cost_type values and equal
  // to itself.
  //
  // 2. "Superset areas are costed at least as low as their children areas."
  // forall bbox1, bbox2 : bounding_box. zb1.contains(zb2) =>
  //      !(min_cost(bbox2) < min_cost(bbox1))
  //
  // 3. "In every possible regular grid of congruent bounding_boxes filling
  //     the space, every object intersects at least one bounding_box
  //     that has min_cost() <= its cost() that is one of that grid's boxes."
  // forall grid_base : bounding_box, object : (ObjectIdentifier, bounding_box).
  //   exists region : bounding_box.
  //     (region is in the grid of grid_base) &&
  //     !(cost(object) < min_cost(region))
  // where "in the grid" means there exists a separate integer N for each dimension
  //   where
  //     region.min(dim) = grid_base.min(dim) + N*grid_base.size(dim)
  //     region.size(dim) = grid_base.size(dim)
  //
  // [**] If you violate the rules, bbox_collision_detector gives no
  // guarantees that the objects will be returned in the correct order at all,
  // or (if you violate the guarantees with respect to returning none) return
  // objects that should be returned at all.
  //
  // If you're going to iterate the whole container and don't mind your
  // iteration being O(n log(n)) rather than nearer O(n), you could just
  // return std::numeric_limits<cost_type>::lowest() from min_cost(), which
  // guarantees that your GetCost is valid (assuming a numeric cost_type).
  template<typename GetCost>
  typename impl::iteration_types<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost>::iterator_range
  iterate(GetCost const&) const;

  template<typename GetCost>
  typename impl::iteration_types<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost>::optional_value_type
  find_least(GetCost const&) const;

  template<typename GetCostBool>
  void
  filter(std::vector<ObjectIdentifier>& results, GetCostBool)const;

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
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
  // If you're not sure if you're invalidating iterators by container
  // mutation (insert/erase), this can check, but it uses space
  // in multiple places so it's #ifdefed.
  public: size_t revision_count_;
#endif

  friend struct impl::zbox_debug_visualizer;
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

