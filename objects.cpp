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

// TODO HAAAAACK
#include "SDL/SDL.h"

shape robot::get_initial_personal_space_shape()const {
  return shape(bounding_box(
    location - vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10),
    location + vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10)
  ));
}

shape robot::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

void robot::update(world &w, object_identifier my_id) {
  bounding_box shape_bounds = w.get_object_personal_space_shapes().find(my_id)->second.bounds();
  vector3<fine_scalar> bottom_middle(
    (shape_bounds.min.x + shape_bounds.max.x) / 2,
    (shape_bounds.min.y + shape_bounds.max.y) / 2,
    shape_bounds.min.z);
  const tile_location l = w.make_tile_location(get_containing_tile_coordinates(bottom_middle), FULL_REALIZATION);
  const tile_location lminus = l + cdir_zminus;
  if (lminus.stuff_at().contents() != AIR) {
    // goal: decay towards levitating...
    fine_scalar target_height = (lower_bound_in_fine_units(l.coords().z, 2) + tile_height * 5 / 4);
    fine_scalar deficiency = target_height - shape_bounds.min.z;
    fine_scalar target_vel = deficiency * velocity_scale_factor / 8;
    if (velocity.z < target_vel) {
      velocity.z = std::min(velocity.z + gravity_acceleration_magnitude * 5, target_vel);
    }
  }
    
  // TODO HAAAAAACK
  Uint8 *keystate = SDL_GetKeyState(NULL);
  velocity.x -= velocity.x / 10;
  velocity.y -= velocity.y / 10;
  if (keystate[SDLK_UP]) {
    velocity.x = facing.x;
    velocity.y = facing.y;
  }
  if (keystate[SDLK_RIGHT]) {
    fine_scalar new_facing_x = facing.x + facing.y / 20;
    fine_scalar new_facing_y = facing.y - facing.x / 20;
    facing.x = new_facing_x; facing.y = new_facing_y;
  }
  if (keystate[SDLK_LEFT]) {
    fine_scalar new_facing_x = facing.x - facing.y / 20;
    fine_scalar new_facing_y = facing.y + facing.x / 20;
    facing.x = new_facing_x; facing.y = new_facing_y;
  }
  facing = facing * tile_width * velocity_scale_factor / 8 / facing.magnitude_within_32_bits();
}


shape laser_emitter::get_initial_personal_space_shape()const {
  return shape(bounding_box(
    location - vector3<fine_scalar>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10),
    location + vector3<fine_scalar>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10)
  ));
}

shape laser_emitter::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

void laser_emitter::update(world &w, object_identifier my_id) {
  bounding_box shape_bounds = w.get_object_personal_space_shapes().find(my_id)->second.bounds();
  vector3<fine_scalar> middle = (shape_bounds.min + shape_bounds.max) / 2;
  location = middle;
  facing = facing * tile_width * 2 / facing.magnitude_within_32_bits();
  
  line_segment laser_line(location, location + facing);
  for (int i = 0; i < 50; ++i) {
    unordered_set<object_or_tile_identifier> possible_hits;
    w.collect_things_exposed_to_collision_intersecting(possible_hits, laser_line.bounds());
    std::pair<bool, boost::rational<int64_t> > best_inters(false, 0);
    object_or_tile_identifier best_id /*no default-constructor so pick something arbitrary*/ = my_id;
    for (object_or_tile_identifier const& id : possible_hits) {
      if (id != my_id) {
        shape other_shape = w.get_detail_shape_of_object_or_tile(id);
        std::pair<bool, boost::rational<int64_t> > inters = other_shape.first_intersection(laser_line);
        if (inters.first && (!best_inters.first || inters.second < best_inters.second)) {
          best_inters = inters;
          best_id = id;
        }
      }
    }
    
    if (best_inters.first) {
      // TODO do I have to worry about overflow?
      w.add_laser_sfx(location, facing * i + facing * best_inters.second.numerator() / best_inters.second.denominator());
      if(tile_location const* locp = best_id.get_tile_location()) {
        if(rand()%100 == 0) {
          w.replace_substance(*locp, locp->stuff_at().contents(), AIR);
        }
      }
      return;
    }
    laser_line.translate(facing);
  }
  w.add_laser_sfx(location, facing * 50);
  return;
  
  /*struct laser_path_calculator {
    struct cross {
      fine_scalar dist_in_this_dimension;
      fine_scalar facing_in_this_dimension;
      int dimension;
      cross(fine_scalar d, fine_scalar f, int di):dist_in_this_dimension(d),facing_in_this_dimension(f),dimension(di){}
      bool operator<(cross const& other)const {
        return other.facing_in_this_dimension * dist_in_this_dimension < facing_in_this_dimension * other.dist_in_this_dimension;
      }
    };
    laser_path_calculator(laser_emitter *emi):emi(emi),current_laser_tile(get_containing_tile_coordinates(emi->location)){
      for (int i = 0; i < 3; ++i) {
        facing_signs[i] = sign(emi->facing[i]);
        facing_offs[i] = emi->facing[i] > 0;
        if (facing_signs[i] != 0) enter_next_cross(i);
      }
    }
    
    void enter_next_cross(int which_dimension) {
      coming_crosses.insert(cross(
        lower_bound_in_fine_units((current_laser_tile + facing_offs)[which_dimension], which_dimension) - emi->location[which_dimension],
        emi->facing[which_dimension],
        which_dimension
      ));
    }
    // returns which dimension we advanced
    int advance_to_next_location() {
      auto next_cross_iter = coming_crosses.begin();
      int which_dimension = next_cross_iter->dimension;
      coming_crosses.erase(next_cross_iter);
      current_laser_tile[which_dimension] += facing_signs[which_dimension];
      enter_next_cross(which_dimension);
      return which_dimension;
    }
    
    laser_emitter *emi;
    vector3<tile_coordinate> current_laser_tile;
    set<cross> coming_crosses;
    vector3<fine_scalar> facing_signs;
    vector3<tile_coordinate_signed_type> facing_offs;
  };
  
  laser_path_calculator calc(this);
  
  const vector3<fine_scalar> max_laser_delta = facing * 50;
  //const vector3<tile_coordinate> theoretical_end_tile = get_containing_tile_coordinates(location + max_laser_delta);
  
  int which_dimension_we_last_advanced = -1;
  while(true) {
    unordered_set<object_or_tile_identifier> possible_hits;
    w.collect_things_exposed_to_collision_intersecting(possible_hits, tile_bounding_box(calc.current_laser_tile));
    for (object_or_tile_identifier const& id : possible_hits) {
      if (id.get_tile_location()) {
        // it's the entire tile, of course we hit it!
        vector3<fine_scalar> laser_delta;
        if (which_dimension_we_last_advanced == -1) {
          laser_delta = vector3<fine_scalar>(0,0,0);
        }
        else {
          vector3<tile_coordinate> hitloc_finding_hack = calc.current_laser_tile;
          hitloc_finding_hack[which_dimension_we_last_advanced] += calc.facing_offs[which_dimension_we_last_advanced];
          const fine_scalar laser_delta_in_facing_direction = (lower_bound_in_fine_units(hitloc_finding_hack)[which_dimension_we_last_advanced] - location[which_dimension_we_last_advanced]);
          laser_delta = (facing * laser_delta_in_facing_direction) / facing[which_dimension_we_last_advanced];
        }
        w.add_laser_sfx(location, laser_delta);
        return; // TODO handle what happens if there are mobile objects and/or multiple objects and/or whatever
      }
      if (object_identifier const* oidp = id.get_object_identifier()) {
        shape const& their_shape = w.get_object_personal_space_shapes().find(*oidp)->second;
        if (their_shape)
      }
    }
    // TODO figure out a better end condition...
    if ((calc.current_laser_tile - get_containing_tile_coordinates(location)).magnitude_within_32_bits_is_greater_than(101)) {
      w.add_laser_sfx(location, max_laser_delta);
      return;
    }
    else which_dimension_we_last_advanced = calc.advance_to_next_location();
  }*/
}

