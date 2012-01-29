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

#ifndef LASERCAKE_TILE_PHYSICS_HPP__
#define LASERCAKE_TILE_PHYSICS_HPP__

#include "world.hpp"

namespace tile_physics_impl {
typedef uint64_t water_tile_count;
typedef uint64_t water_group_identifier;
const water_group_identifier NO_WATER_GROUP = 0;

struct state_t;

// "progress" is measured in the smaller, velocity units.
inline sub_tile_distance progress_necessary(cardinal_direction dir) {
  return tile_size[which_dimension_is_cardinal_direction(dir)] * velocity_scale_factor;
}

struct active_fluid_tile_info {
  // Constructing one of these in the default way yields the natural inactive state
  active_fluid_tile_info();
  bool is_in_inactive_state()const;

  vector3<sub_tile_distance> velocity;
  value_for_each_cardinal_direction<sub_tile_distance> progress;
  value_for_each_cardinal_direction<sub_tile_distance> blockage_amount_this_frame;

  //int frames_until_can_become_groupable = 0;
};

typedef unordered_map<tile_location, active_fluid_tile_info> active_fluids_t;

struct persistent_water_group_info {
  literally_random_access_removable_tiles_by_height suckable_tiles_by_height;
  literally_random_access_removable_tiles_by_height pushable_tiles_by_height;
  map<tile_coordinate, water_tile_count> num_tiles_by_height;
  unordered_set<tile_location> surface_tiles;

  mutable map<tile_coordinate, fine_scalar> pressure_caches;
  mutable map<tile_coordinate, water_tile_count> width_of_widest_level_so_far_caches;

  //bool is_infinite;
  //tile_coordinate infinite_ocean_height;

  void recompute_num_tiles_by_height_from_surface_tiles(state_t const& w);
  fine_scalar get_pressure_at_height(tile_coordinate height)const;

  tile_location get_and_erase_random_pushable_tile_below_weighted_by_pressure(tile_coordinate height);

  bool mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(state_t& state, tile_location const& loc, active_fluids_t &active_fluids);
  bool mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(state_t& state, tile_location const& loc, active_fluids_t &active_fluids);
};

typedef unordered_map<water_group_identifier, persistent_water_group_info> persistent_water_groups_t;
typedef unordered_map<tile_location, water_group_identifier> water_groups_by_location_t;




// We could easily keep lists of boundary tiles in all three dimensions
// (Just uncomment the six commented lines below.)
// The only reason we don't is because there's no need for any of the others right now.
// (And it would take that much extra space (proportional to the that-dimension surface area)
//   and time (proportional to how much the groupable-water landscape changes)).
struct groupable_water_dimensional_boundaries_TODO_name_this_better_t {
  set<tile_location, tile_compare_yzx> x_boundary_groupable_water_tiles;
  //set<tile_location, tile_compare_zxy> y_boundary_groupable_water_tiles;
  //set<tile_location, tile_compare_xyz> z_boundary_groupable_water_tiles;
  void handle_tile_insertion(tile_location const& loc) {
    handle_tile_insertion_in_dimension<tile_compare_yzx, xplus>(x_boundary_groupable_water_tiles, loc);
  //  handle_tile_insertion_in_dimension(y_boundary_groupable_water_tiles, loc, cdir_yplus);
  //  handle_tile_insertion_in_dimension(z_boundary_groupable_water_tiles, loc, cdir_zplus);
  }
  void handle_tile_removal(tile_location const& loc) {
    handle_tile_removal_in_dimension<tile_compare_yzx, xplus>(x_boundary_groupable_water_tiles, loc);
  //  handle_tile_removal_in_dimension(y_boundary_groupable_water_tiles, loc, cdir_yplus);
  //  handle_tile_removal_in_dimension(z_boundary_groupable_water_tiles, loc, cdir_zplus);
  }
private:
  template <typename Compare, cardinal_direction Dir>
  static void handle_tile_removal_in_dimension(set<tile_location, Compare> &boundary_tiles_set, tile_location const& loc) {
    // This tile is no longer groupable at all, so it can't be a boundary tile
    boundary_tiles_set.erase(loc);

    // If there are groupable tiles next to us, they must now be boundary tiles,
    // because our deletion exposed them
    const tile_location further_in_positive_direction_loc = loc.get_neighbor<Dir                     >(CONTENTS_ONLY);
    const tile_location further_in_negative_direction_loc = loc.get_neighbor<cdir_info<Dir>::opposite>(CONTENTS_ONLY);
    if (further_in_positive_direction_loc.stuff_at().contents() == GROUPABLE_WATER) {
      boundary_tiles_set.insert(further_in_positive_direction_loc);
    }
    if (further_in_negative_direction_loc.stuff_at().contents() == GROUPABLE_WATER) {
      boundary_tiles_set.insert(further_in_negative_direction_loc);
    }
  }
  template <typename Compare, cardinal_direction Dir>
  static void handle_tile_insertion_in_dimension(set<tile_location, Compare> &boundary_tiles_set, tile_location const& loc) {
    // We *may* have removed boundaries in either direction, and we *may* now be a boundary tile ourselves.
    const tile_location further_in_positive_direction_loc = loc.get_neighbor<Dir                     >(CONTENTS_ONLY);
    const tile_location further_in_negative_direction_loc = loc.get_neighbor<cdir_info<Dir>::opposite>(CONTENTS_ONLY);
    bool we_are_boundary_tile = false;
    if (further_in_positive_direction_loc.stuff_at().contents() == GROUPABLE_WATER) {
      if (further_in_positive_direction_loc.get_neighbor<Dir                     >(CONTENTS_ONLY).stuff_at().contents() == GROUPABLE_WATER) {
        boundary_tiles_set.erase(further_in_positive_direction_loc);
      }
    }
    else we_are_boundary_tile = true;
    if (further_in_negative_direction_loc.stuff_at().contents() == GROUPABLE_WATER) {
      if (further_in_negative_direction_loc.get_neighbor<cdir_info<Dir>::opposite>(CONTENTS_ONLY).stuff_at().contents() == GROUPABLE_WATER) {
        boundary_tiles_set.erase(further_in_negative_direction_loc);
      }
    }
    else we_are_boundary_tile = true;

    if (we_are_boundary_tile) boundary_tiles_set.insert(loc);
  }
};

struct state_t {
  //state_t(world_collision_detector& d):next_water_group_identifier(1), things_exposed_to_collision(d){}
  state_t(world& w):next_water_group_identifier(1), access_the_world(w){}

  water_group_identifier next_water_group_identifier;
  water_groups_by_location_t water_groups_by_surface_tile;
  persistent_water_groups_t persistent_water_groups;
  groupable_water_dimensional_boundaries_TODO_name_this_better_t groupable_water_dimensional_boundaries_TODO_name_this_better;
  active_fluids_t active_fluids;

  //only used in replace_substance(), to get world_collision_detector& things_exposed_to_collision:
  world& access_the_world;
  //world_collision_detector& things_exposed_to_collision;
};

} // end namespace tile_physics_impl



#endif

