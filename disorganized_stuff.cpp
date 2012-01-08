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

void world_building_gun::operator()(tile_contents new_contents, vector3<tile_coordinate> locv) {
  assert(bounds.contains(locv));
  if (new_contents == ROCK) {
    w->insert_rock_bypassing_checks(w->make_tile_location(locv, COMPLETELY_IMAGINARY));
  }
  else if (new_contents == WATER) {
    w->insert_water_bypassing_checks(w->make_tile_location(locv, COMPLETELY_IMAGINARY));
  }
  else assert("YOU CAN ONLY PLACE ROCK AND WATER" && false);
}


shape world::get_personal_space_shape_of_object_or_tile(object_or_tile_identifier id)const {
  if (tile_location const* tlocp = id.get_tile_location()) {
    return tile_shape(tlocp->coords());
  }
  if (object_identifier const* oidp = id.get_object_identifier()) {
    return object_personal_space_shapes.find(*oidp)->second;
  }
  assert(false);
}
shape world::get_detail_shape_of_object_or_tile(object_or_tile_identifier id)const {
  if (tile_location const* tlocp = id.get_tile_location()) {
    return tile_shape(tlocp->coords());
  }
  if (object_identifier const* oidp = id.get_object_identifier()) {
    return object_personal_space_shapes.find(*oidp)->second;
  }
  assert(false);
}


tile_location literally_random_access_removable_tiles_by_height::get_and_erase_random_from_the_top() {
  map_t::reverse_iterator iter = data.rbegin();
  tile_location result = iter->second.get_random();
  iter->second.erase(result);
  if (iter->second.empty()) data.erase(iter);
  return result;
}
bool literally_random_access_removable_tiles_by_height::erase(tile_location const& loc) {
  auto j = data.find(loc.coords().z);
  if (j != data.end()) {
    if (j->second.erase(loc)) {
      if (j->second.empty()) {
        data.erase(loc);
      }
      return true;
    }
  }
  return false;
}
void literally_random_access_removable_tiles_by_height::insert(tile_location const& loc)const {
  // Note: operator[] default-constructs an empty structure if there wasn't one
  data[loc.coords().z].insert(loc);
}

bool literally_random_access_removable_tiles_by_height::any_above(tile_coordinate height)const {
  return data.upper_bound(height) != data.end();
}
bool literally_random_access_removable_tiles_by_height::any_below(tile_coordinate height)const {
  return (!data.empty()) && (data.begin()->first < height);
}


