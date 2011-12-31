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

struct line_segment {
  line_segment(std::array<vector3<int64_t>, 2> ends):ends(ends){}
  line_segment(vector3<int64_t> end1, vector3<int64_t> end2):ends({{end1, end2}}){}
  std::array<vector3<int64_t>, 2> ends;
};

struct convex_polygon {
  // The structure simply trusts that you will provide a convex, coplanar sequence of points. Failure to do so will result in undefined behavior.
  void setup_cache_if_needed()const;
  convex_polygon(std::vector<vector3<int64_t>> const& vertices):vertices(vertices){ assert(vertices.size() >= 3); }
  polygon_collision_info_cache const& get_cache()const { return cache; }
  std::vector<vector3<int64_t>> const& get_vertices()const { return vertices; }
  void translate(vector3<int64_t> t);
private:
  std::vector<vector3<int64_t>> vertices;
  mutable polygon_collision_info_cache cache;
};

bool intersects(line_segment l, convex_polygon const& p);
bool intersects(convex_polygon const& p1, convex_polygon const& p2);
bool intersects(line_segment const& l, std::vector<convex_polygon> const& ps);
bool intersects(std::vector<convex_polygon> const& ps1, std::vector<convex_polygon> const& ps2);

#endif

