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


#ifndef LASERCAKE_POLYGON_COLLISION_DETECTION_HPP__
#define LASERCAKE_POLYGON_COLLISION_DETECTION_HPP__

#include <cassert>
#include <vector>
#include <array>

#include "utils.hpp"

// So that we can make implicit conversions between their and our bbox:
#include "bbox_collision_detector.hpp"

// 64 bits though these ints are, you can't really do collision detection with them except for things
//  with a max distance of about 14 bits between any of the parts of them.

typedef lasercake_int<int64_t>::type polygon_int_type;

struct polygon_collision_info_cache {
  polygon_collision_info_cache():is_valid(false){}
  bool is_valid;
  vector3<polygon_int_type> translation_amount;
  polygon_int_type denom;
  polygon_int_type a_times_denom;
  polygon_int_type b_times_denom;
  int amount_twisted;
  std::vector<vector3<polygon_int_type>> adjusted_vertices;
};

struct bounding_box {
  bounding_box():is_anywhere(false){}
  bounding_box(vector3<polygon_int_type> loc):is_anywhere(true),min(loc),max(loc){}
  bounding_box(vector3<polygon_int_type> min, vector3<polygon_int_type> max):is_anywhere(true),min(min),max(max){}
  
  bool is_anywhere;
  vector3<polygon_int_type> min, max;
  
  bool contains(vector3<polygon_int_type> const& v)const;
  bool overlaps(bounding_box const& o)const;
  void combine_with(bounding_box const& o);
  void restrict_to(bounding_box const& o);
  void translate(vector3<polygon_int_type> t);

  // Implicit conversions to/from bbox_collision_detector bounding_box
  // for convenience interacting with it.
  typedef collision_detector::bounding_box<64, 3> equivalent_collision_detector_bounding_box;
  operator equivalent_collision_detector_bounding_box()const {
    caller_correct_if(is_anywhere, "Trying to pass nowhere-bounds to bbox_collision_detector, which has no such concept");
    
    typedef uint64_t uint_t; // a modulo type and modulo conversion
    typedef equivalent_collision_detector_bounding_box::coordinate_array uint_array;
    
    uint_array uint_min = {{
      uint_t(get_primitive_int(min.x)),
      uint_t(get_primitive_int(min.y)),
      uint_t(get_primitive_int(min.z))
    }};
    uint_array uint_max = {{
      uint_t(get_primitive_int(max.x)),
      uint_t(get_primitive_int(max.y)),
      uint_t(get_primitive_int(max.z))
    }};
    return equivalent_collision_detector_bounding_box::min_and_max(uint_min, uint_max);
  }
  bounding_box(equivalent_collision_detector_bounding_box const& b) {
    typedef int64_t int_t; // a modulo conversion
    min.x = int_t(b.min(X));
    min.y = int_t(b.min(Y));
    min.z = int_t(b.min(Z));
    max.x = int_t(b.max(X));
    max.y = int_t(b.max(Y));
    max.z = int_t(b.max(Z));
    is_anywhere = true;
  }
};
inline bool operator==(bounding_box const& b1, bounding_box const& b2) {
  return (b1.is_anywhere == false && b2.is_anywhere == false) ||
    (b1.is_anywhere == b2.is_anywhere && b1.min == b2.min && b1.max == b2.max);
}
inline bool operator!=(bounding_box const& b1, bounding_box const& b2) {
  return !(b1 == b2);
}
inline std::ostream& operator<<(std::ostream& os, bounding_box const& bb) {
  return os << '[' << bb.min << ", " << bb.max << ']';
}

// TODO: rays and lines
struct line_segment {
  line_segment(std::array<vector3<polygon_int_type>, 2> ends):ends(ends){}
  line_segment(vector3<polygon_int_type> end1, vector3<polygon_int_type> end2):ends({{end1, end2}}){}
  std::array<vector3<polygon_int_type>, 2> ends;
  
  void translate(vector3<polygon_int_type> t);
  bounding_box bounds()const;
};

struct convex_polygon {
  // The structure simply trusts that you will provide a convex, coplanar sequence of points. Failure to do so will result in undefined behavior.
  void setup_cache_if_needed()const;
  convex_polygon(std::vector<vector3<polygon_int_type>> const& vertices):vertices_(vertices){ caller_error_if(vertices_.size() < 3, "Trying to construct a polygon with fewer than three vertices"); } // And there's also something wrong with a polygon where all the points are collinear, but it's more complicated to check that.
  polygon_collision_info_cache const& get_cache()const { return cache_; }
  std::vector<vector3<polygon_int_type>> const& get_vertices()const { return vertices_; }
  void translate(vector3<polygon_int_type> t);
  bounding_box bounds()const;
private:
  std::vector<vector3<polygon_int_type>> vertices_;
  mutable polygon_collision_info_cache cache_;
};

class shape {
public:
  shape(                          ) : bounds_cache_is_valid_(false) {}
  shape(  line_segment const& init) : bounds_cache_is_valid_(false) { segments_.push_back(init); }
  shape(convex_polygon const& init) : bounds_cache_is_valid_(false) { polygons_.push_back(init); }
  shape(  bounding_box const& init) : bounds_cache_is_valid_(false) { boxes_   .push_back(init); }
  shape(lasercake_vector<convex_polygon>::type const& init)
                                    : bounds_cache_is_valid_(false), polygons_(init) {}
  
  shape(shape const& o):bounds_cache_(o.bounds_cache_),bounds_cache_is_valid_(o.bounds_cache_is_valid_),segments_(o.segments_),polygons_(o.polygons_),boxes_(o.boxes_) {}
  
  void translate(vector3<polygon_int_type> t);
  
  bool intersects(shape const& other)const;
  // returns (was there an intersection?, what fraction of the length of the line segment was the first)
  std::pair<bool, non_normalized_rational<polygon_int_type>> first_intersection(line_segment const& other)const;
  bounding_box bounds()const;
  vector3<polygon_int_type> arbitrary_interior_point()const;
  
  lasercake_vector<  line_segment>::type const& get_segments()const { return segments_; }
  lasercake_vector<convex_polygon>::type const& get_polygons()const { return polygons_; }
  lasercake_vector<  bounding_box>::type const& get_boxes   ()const { return boxes_   ; }
private:
  mutable bounding_box bounds_cache_;
  mutable bool bounds_cache_is_valid_;
  
  lasercake_vector<  line_segment>::type segments_;
  lasercake_vector<convex_polygon>::type polygons_;
  lasercake_vector<  bounding_box>::type boxes_   ;
};

/*bool intersects(line_segment l, convex_polygon const& p);
bool intersects(convex_polygon const& p1, convex_polygon const& p2);
bool intersects(line_segment const& l, std::vector<convex_polygon> const& ps);
bool intersects(std::vector<convex_polygon> const& ps1, std::vector<convex_polygon> const& ps2);*/

#endif

