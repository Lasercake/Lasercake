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

#include <climits>
#include "world.hpp"

void world_building_gun::operator()(tile_contents new_contents, vector3<tile_coordinate> locv) {
  caller_correct_if(bounds_.contains(locv), "Trying to use world_building_gun to create tiles outside the given range");
  if (new_contents == ROCK || new_contents == GROUPABLE_WATER || new_contents == RUBBLE) {
    w_->initialize_tile_contents_(w_->make_tile_location(locv, COMPLETELY_IMAGINARY), new_contents);
  }
  else caller_error("Trying to place a type of tile other than ROCK, GROUPABLE_WATER, and RUBBLE");
}

bounding_box world::get_bounding_box_of_object_or_tile(object_or_tile_identifier id)const {
  if (tile_location const* tlocp = id.get_tile_location()) {
    return fine_bounding_box_of_tile(tlocp->coords());
  }
  if (object_identifier const* oidp = id.get_object_identifier()) {
    //TODO is this impl a hack? and what about non-mobile objects?
    bounding_box result = things_exposed_to_collision_.find_bounding_box(*oidp);
    assert(result.is_anywhere);
    return result;
  }
  assert(false);
}
shape world::get_personal_space_shape_of_object_or_tile(object_or_tile_identifier id)const {
  if (tile_location const* tlocp = id.get_tile_location()) {
    return tile_shape(tlocp->coords());
  }
  if (object_identifier const* oidp = id.get_object_identifier()) {
    return object_personal_space_shapes_.find(*oidp)->second;
  }
  assert(false);
}
shape world::get_detail_shape_of_object_or_tile(object_or_tile_identifier id)const {
  if (tile_location const* tlocp = id.get_tile_location()) {
    return tile_shape(tlocp->coords());
  }
  if (object_identifier const* oidp = id.get_object_identifier()) {
    return object_personal_space_shapes_.find(*oidp)->second;
  }
  assert(false);
}


tile_location literally_random_access_removable_tiles_by_height::get_and_erase_random_from_the_top() {
  map_t::reverse_iterator iter = data_.rbegin();
  const tile_location result = iter->second.get_random();
  iter->second.erase(result);
  if (iter->second.empty()) data_.erase(iter->first); // can't erase from a reverse_iterator
  return result;
}
tile_location literally_random_access_removable_tiles_by_height::get_and_erase_random_from_the_bottom() {
  map_t::iterator iter = data_.begin();
  const tile_location result = iter->second.get_random();
  iter->second.erase(result);
  if (iter->second.empty()) data_.erase(iter);
  return result;
}
bool literally_random_access_removable_tiles_by_height::erase(tile_location const& loc) {
  auto j = data_.find(loc.coords().z);
  if (j != data_.end()) {
    if (j->second.erase(loc)) {
      if (j->second.empty()) {
        data_.erase(j);
      }
      return true;
    }
  }
  return false;
}
void literally_random_access_removable_tiles_by_height::insert(tile_location const& loc) {
  // Note: operator[] default-constructs an empty structure if there wasn't one
  data_[loc.coords().z].insert(loc);
}

bool literally_random_access_removable_tiles_by_height::any_above(tile_coordinate height)const {
  return data_.upper_bound(height) != data_.end();
}
bool literally_random_access_removable_tiles_by_height::any_below(tile_coordinate height)const {
  return (!data_.empty()) && (data_.begin()->first < height);
}

bool tile_compare_xyz::operator()(tile_location const& i, tile_location const& j)const {
  vector3<tile_coordinate> c1 = i.coords();
  vector3<tile_coordinate> c2 = j.coords();
  return (c1.x < c2.x) || ((c1.x == c2.x) && ((c1.y < c2.y) || ((c1.y == c2.y) && (c1.z < c2.z))));
}
bool tile_compare_yzx::operator()(tile_location const& i, tile_location const& j)const {
  vector3<tile_coordinate> c1 = i.coords();
  vector3<tile_coordinate> c2 = j.coords();
  return (c1.y < c2.y) || ((c1.y == c2.y) && ((c1.z < c2.z) || ((c1.z == c2.z) && (c1.x < c2.x))));
}
bool tile_compare_zxy::operator()(tile_location const& i, tile_location const& j)const {
  vector3<tile_coordinate> c1 = i.coords();
  vector3<tile_coordinate> c2 = j.coords();
  return (c1.z < c2.z) || ((c1.z == c2.z) && ((c1.x < c2.x) || ((c1.x == c2.x) && (c1.y < c2.y))));
}


// Double-check (not thoroughly) that the compilers aren't lying about word sizes
// and use 8-bit bytes and two's complement arithmetic.
// Hopefully this means that integer variations won't cause two people's compiles
// of this program to behave differently.
static_assert(CHAR_BIT == 8, "8 bits in a byte (claims limits.h)");
static_assert(sizeof(uint64_t)*8 == 64, "uint64_t is 64 bit");
static_assert(sizeof(int64_t)*8 == 64, "int64_t is 64 bit");
static_assert(sizeof(uint32_t)*8 == 32, "uint32_t is 32 bit");
static_assert(sizeof(int32_t)*8 == 32, "int32_t is 32 bit");
static_assert(sizeof(uint16_t)*8 == 16, "uint16_t is 16 bit");
static_assert(sizeof(int16_t)*8 == 16, "int16_t is 16 bit");
static_assert(sizeof(uint8_t)*8 == 8, "uint8_t is 8 bit");
static_assert(sizeof(int8_t)*8 == 8, "int8_t is 8 bit");
static_assert((uint32_t)0xffffffffUL == 0xffffffffUL, "max uint32_t fits");
static_assert((uint32_t)0xffffffffUL + (uint32_t)1 == (uint32_t)0, "max uint32_t + 1 doesn't fit");
static_assert((int32_t)(uint32_t)0x80000000UL < 0, "min int32_t < 0");
static_assert((int32_t)(uint32_t)0x7fffffffUL > 0, "max int32_t > 0");
