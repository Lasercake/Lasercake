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
#include "worldgen.hpp"


world::world(worldgen_function_t f)
   : current_game_time_(0), tile_physics_state_(*this), next_object_identifier_(1), worldgen_function_(f) {}

void world::update(unordered_map<object_identifier, input_representation::input_news_t> input) {
  laser_sfxes_.clear();
  update_fluids();
  const input_representation::input_news_t no_input;
  for (auto& obj : autonomously_active_objects_) {
    input_representation::input_news_t const* input_for_obj = find_as_pointer(input, obj.first);
    if (input_for_obj == nullptr) { input_for_obj = &no_input; }
    obj.second->update(*this, *input_for_obj, obj.first);
  }
  update_moving_objects();
  update_light(vector3<fine_scalar>(3,10,-999), 3);
  current_game_time_ += time_units_per_fixed_frame;
}

object_identifier world::try_create_object(shared_ptr<object> obj) {
    // fail (and return NO_OBJECT) if there's something in the way
    const shape obj_shape = obj->get_initial_personal_space_shape();
    vector<object_identifier> objects_this_could_collide_with;
    objects_exposed_to_collision_.get_objects_overlapping(objects_this_could_collide_with, obj_shape.bounds());
    for (auto oid : objects_this_could_collide_with) {
      // We use volume_intersects so that tile aligned objects can be placed next to each other.
      if (object_personal_space_shapes_.find(oid)->second.volume_intersects(obj_shape)){
        return NO_OBJECT;
      }
    }
    
    object_identifier id = next_object_identifier_++;
    objects_.insert(make_pair(id, obj));
    bounding_box b; // TODO: in mobile_objects.cpp, include detail_shape in at least the final box left in the ztree
    object_personal_space_shapes_[id] = obj_shape;
    b.combine_with(object_personal_space_shapes_[id].bounds());
    object_detail_shapes_[id] = obj->get_initial_detail_shape();
    b.combine_with(object_detail_shapes_[id].bounds());
    objects_exposed_to_collision_.insert(id, b);
    if(shared_ptr<mobile_object> m = boost::dynamic_pointer_cast<mobile_object>(obj)) {
      moving_objects_.insert(make_pair(id, m));
    }
    // TODO: don't do this if you're in the middle of updating autonomous objects
    if(shared_ptr<autonomous_object> m = boost::dynamic_pointer_cast<autonomous_object>(obj)) {
      autonomously_active_objects_.insert(make_pair(id, m));
    }
    return id;
  }


bounding_box world::get_bounding_box_of_object_or_tile(object_or_tile_identifier id)const {
  if (tile_location const* tlocp = id.get_tile_location()) {
    return fine_bounding_box_of_tile(tlocp->coords());
  }
  if (object_identifier const* oidp = id.get_object_identifier()) {
    //TODO is this impl a hack? and what about non-mobile objects?
    auto const* result = objects_exposed_to_collision_.find_bounding_box(*oidp);
    assert(result != nullptr);
    return *result;
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
