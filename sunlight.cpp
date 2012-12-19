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

#include "world.hpp"
#include "object_and_tile_iteration.hpp"
#include "data_structures/polygon_collision_detection.hpp"

const int SUN_AREA_SIZE = 5000;


struct sunlight_visitor {
  sunlight_visitor(world *w, vector3<fine_scalar> sun_direction): w(w),sun_direction(sun_direction) {
    memset(packets,0,sizeof(bool)*SUN_AREA_SIZE*SUN_AREA_SIZE);
  }
  bool packets[SUN_AREA_SIZE][SUN_AREA_SIZE];
  octant_number octant()const { return vector_octant(sun_direction); }
  octant_number octant_; //e.g. from vector_octant()

  int do_poly(convex_polyhedron const& p) {
    int result = 0;
    int max_x, max_y, min_x, min_y;
    bool any = false;
    for (vector3<fine_scalar> const& v : p.vertices()) {
      vector3<fine_scalar> projected_vertex = v;
      projected_vertex -= world_center_fine_coords;
      /*projected_vertex[X] -= projected_vertex(Z) * sun_direction(X) / sun_direction(Z);
      projected_vertex[Y] -= projected_vertex(Z) * sun_direction(Y) / sun_direction(Z);
      projected_vertex[Z] = 0;

      projected_vertex[X] = (projected_vertex(X) * 10 / tile_width) + (SUN_AREA_SIZE / 2);
      projected_vertex[Y] = (projected_vertex(Y) * 10 / tile_width) + (SUN_AREA_SIZE / 2);*/
      projected_vertex[X] = ((projected_vertex(X) * sun_direction(Z) - projected_vertex(Z) * sun_direction(X)) * 10 / (tile_width * sun_direction(Z))) + (SUN_AREA_SIZE / 2);
      projected_vertex[Y] = ((projected_vertex(Y) * sun_direction(Z) - projected_vertex(Z) * sun_direction(Y)) * 10 / (tile_width * sun_direction(Z))) + (SUN_AREA_SIZE / 2);
      if (!any || projected_vertex(X) > max_x) max_x = projected_vertex(X);
      if (!any || projected_vertex(Y) > max_y) max_y = projected_vertex(Y);
      if (!any || projected_vertex(X) < min_x) min_x = projected_vertex(X);
      if (!any || projected_vertex(Y) < min_y) min_y = projected_vertex(Y);
      any = true;
    }
    assert(any);
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= SUN_AREA_SIZE-1) max_x = SUN_AREA_SIZE-1;
    if (max_y >= SUN_AREA_SIZE-1) max_y = SUN_AREA_SIZE-1;

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        if (!packets[x][y]) {
          packets[x][y] = true;
          ++result;
        }
      }
    }
    return result;
  }
  
  int do_bbox(bounding_box bb) {
    int result = 0;

    bb.translate(-world_center_fine_coords);
    
    int max_x = ((bb.max(X) * sun_direction(Z) - (sun_direction(X) > 0 ? bb.max(Z) : bb.min(Z)) * sun_direction(X)) * 10 / (tile_width * sun_direction(Z))) + (SUN_AREA_SIZE / 2);
    int min_x = ((bb.min(X) * sun_direction(Z) - (sun_direction(X) > 0 ? bb.min(Z) : bb.max(Z)) * sun_direction(X)) * 10 / (tile_width * sun_direction(Z))) + (SUN_AREA_SIZE / 2);
    int max_y = ((bb.max(Y) * sun_direction(Z) - (sun_direction(Y) > 0 ? bb.max(Z) : bb.min(Z)) * sun_direction(Y)) * 10 / (tile_width * sun_direction(Z))) + (SUN_AREA_SIZE / 2);
    int min_y = ((bb.min(Y) * sun_direction(Z) - (sun_direction(Y) > 0 ? bb.min(Z) : bb.max(Z)) * sun_direction(Y)) * 10 / (tile_width * sun_direction(Z))) + (SUN_AREA_SIZE / 2);
    
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= SUN_AREA_SIZE-1) max_x = SUN_AREA_SIZE-1;
    if (max_y >= SUN_AREA_SIZE-1) max_y = SUN_AREA_SIZE-1;
    //std::cerr << max_x - min_x << "\n" << max_y - min_y << "!\n";

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        if (!packets[x][y]) {
          packets[x][y] = true;
          ++result;
        }
      }
    }
    return result;
  }
  
  int do_shape(shape const& s) {
    int result = 0;
    for (convex_polyhedron const& p : s.get_polyhedra()) {
      result += do_poly(p);
    }
    for (bounding_box const& b : s.get_boxes()) {
      result += do_bbox(b);
    }
    return result;
  }
  
  void found(tile_location const& loc) {
    w->litnesses_.insert(std::pair<object_or_tile_identifier, int>(loc, 0)).first->second += do_bbox(fine_bounding_box_of_tile(loc.coords()));
  }
  void found(object_identifier oid) {
    shape const* ods = find_as_pointer(w->get_object_detail_shapes(), oid); assert(ods);
    w->litnesses_.insert(std::pair<object_or_tile_identifier, int>(oid, 0)).first->second += do_shape(*ods);
  }

  world *w;
  vector3<fine_scalar> sun_direction;
};

void world::update_light(vector3<fine_scalar> sun_direction)
{
  litnesses_.clear();
  std::unique_ptr<sunlight_visitor> sv(new sunlight_visitor(this, sun_direction));
  visit_collidable_tiles_and_objects(*sv);
}
