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

// 64 bits though these ints are, you can't really do collision detection with them except for things
//  with a max distance of about 14 bits between any of the parts of them.

struct polygon_collision_info_cache {
  polygon_collision_info_cache():is_valid(false){}
  bool is_valid;
  vector3<int64_t> translation_amount;
  int64_t denom;
  int64_t a_times_denom;
  int64_t b_times_denom;
  int amount_twisted;
  std::vector<vector3<int64_t>> adjusted_vertices;
};

struct bounding_box {
  bounding_box():is_anywhere(false){}
  bounding_box(vector3<int64_t> loc):is_anywhere(true),min(loc),max(loc){}
  bounding_box(vector3<int64_t> min, vector3<int64_t> max):is_anywhere(true),min(min),max(max){}
  
  bool is_anywhere;
  vector3<int64_t> min, max;
  
  bool contains(vector3<int64_t> const& v)const;
  bool overlaps(bounding_box const& o)const;
  void combine_with(bounding_box const& o);
  void restrict_to(bounding_box const& o);
  void translate(vector3<int64_t> t);
};
inline bool operator==(bounding_box const& b1, bounding_box const& b2) {
  return (b1.is_anywhere == false && b2.is_anywhere == false) ||
    (b1.is_anywhere == b2.is_anywhere && b1.min == b2.min && b1.max == b2.max);
}
inline bool operator!=(bounding_box const& b1, bounding_box const& b2) {
  return !(b1 == b2);
}
inline std::ostream& operator<<(std::ostream& os, bounding_box const& bb) {
  return os << '[' << bb.min << ',' << bb.max << ']';
}

// TODO: rays and lines
struct line_segment {
  line_segment(std::array<vector3<int64_t>, 2> ends):ends(ends){}
  line_segment(vector3<int64_t> end1, vector3<int64_t> end2):ends({{end1, end2}}){}
  std::array<vector3<int64_t>, 2> ends;
  
  void translate(vector3<int64_t> t);
  bounding_box bounds()const;
};

struct convex_polygon {
  // The structure simply trusts that you will provide a convex, coplanar sequence of points. Failure to do so will result in undefined behavior.
  void setup_cache_if_needed()const;
  convex_polygon(std::vector<vector3<int64_t>> const& vertices):vertices(vertices){ caller_error_if(vertices.size() < 3, "Trying to construct a polygon with fewer than three vertices"); } // And there's also something wrong with a polygon where all the points are collinear, but it's more complicated to check that.
  polygon_collision_info_cache const& get_cache()const { return cache; }
  std::vector<vector3<int64_t>> const& get_vertices()const { return vertices; }
  void translate(vector3<int64_t> t);
  bounding_box bounds()const;
private:
  std::vector<vector3<int64_t>> vertices;
  mutable polygon_collision_info_cache cache;
};

class shape {
public:
  shape(                                       ): bounds_cache_is_valid(false)                {}
  shape(               line_segment const& init): bounds_cache_is_valid(false)                { segments.push_back(init); }
  shape(             convex_polygon const& init): bounds_cache_is_valid(false)                { polygons.push_back(init); }
  shape(               bounding_box const& init): bounds_cache_is_valid(false)                { boxes   .push_back(init); }
  shape(std::vector<convex_polygon> const& init): bounds_cache_is_valid(false), polygons(init){}
  
  shape(shape const& o):bounds_cache(o.bounds_cache),bounds_cache_is_valid(o.bounds_cache_is_valid),segments(o.segments),polygons(o.polygons),boxes(o.boxes) {}
  
  void translate(vector3<int64_t> t);
  
  bool intersects(shape const& other)const;
  // returns (was there an intersection?, what fraction of the length of the line segment was the first)
  std::pair<bool, non_normalized_rational<int64_t>> first_intersection(line_segment const& other)const;
  bounding_box bounds()const;
  vector3<int64_t> arbitrary_interior_point()const;
  
  std::vector<  line_segment> const& get_segments()const { return segments; }
  std::vector<convex_polygon> const& get_polygons()const { return polygons; }
  std::vector<  bounding_box> const& get_boxes   ()const { return boxes   ; }
private:
  mutable bounding_box bounds_cache;
  mutable bool bounds_cache_is_valid;
  
  std::vector<  line_segment> segments;
  std::vector<convex_polygon> polygons;
  std::vector<  bounding_box> boxes   ;
};

/*bool intersects(line_segment l, convex_polygon const& p);
bool intersects(convex_polygon const& p1, convex_polygon const& p2);
bool intersects(line_segment const& l, std::vector<convex_polygon> const& ps);
bool intersects(std::vector<convex_polygon> const& ps1, std::vector<convex_polygon> const& ps2);*/

#endif

