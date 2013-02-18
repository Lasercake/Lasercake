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
typedef lint64_t water_tile_count;
typedef lint64_t water_group_identifier;
const water_group_identifier NO_WATER_GROUP = 0;

struct state_t;

// "progress" is measured in the smaller, velocity units.
inline sub_tile_distance progress_necessary(cardinal_direction dir) {
  return sub_tile_distance(
    tile_size[which_dimension_is_cardinal_direction(dir)]
    * identity(tile_physics_sub_tile_distance_units / fine_distance_units));
}

struct active_fluid_tile_info {
  // Constructing one of these in the default way yields the natural inactive state
  active_fluid_tile_info();
  bool is_in_inactive_state()const;

  vector3<sub_tile_velocity> velocity;
  value_for_each_cardinal_direction<sub_tile_distance> progress;
  value_for_each_cardinal_direction<sub_tile_distance> blockage_amount_this_frame;

  //int frames_until_can_become_groupable = 0;
};

typedef unordered_map<tile_location, active_fluid_tile_info> active_fluids_t;
typedef unordered_map<vector3<tile_coordinate>, minerals> altered_minerals_info_t;

struct persistent_water_group_info {
  literally_random_access_removable_tiles_by_height suckable_tiles_by_height;
  literally_random_access_removable_tiles_by_height pushable_tiles_by_height;
  map<tile_coordinate, water_tile_count> num_tiles_by_height;
  unordered_set<tile_location> surface_tiles;

  typedef map<tile_coordinate, pressure> pressure_caches_t;
  mutable pressure_caches_t pressure_caches;
  mutable map<tile_coordinate, water_tile_count> width_of_widest_level_so_far_caches;

  //bool is_infinite;
  //tile_coordinate infinite_ocean_height;

  void recompute_num_tiles_by_height_from_surface_tiles(state_t const& w);
  pressure get_pressure_at_height(tile_coordinate height)const;

  tile_location get_and_erase_random_pushable_tile_below_weighted_by_pressure(tile_coordinate height);

  bool mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(
    state_t& state, tile_location const& loc, active_fluids_t& active_fluids);
  bool mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(
    state_t& state, tile_location const& loc, active_fluids_t& active_fluids);
};

typedef unordered_map<water_group_identifier, persistent_water_group_info> persistent_water_groups_t;
typedef unordered_map<tile_location, water_group_identifier> water_groups_by_location_t;


template<cardinal_direction Dir> struct volume_calipers_tile_compare_for_dir;
template<> struct volume_calipers_tile_compare_for_dir<xplus > { typedef tile_compare_yzx type; };
template<> struct volume_calipers_tile_compare_for_dir<xminus> { typedef tile_compare_yzx type; };
template<> struct volume_calipers_tile_compare_for_dir<yplus > { typedef tile_compare_zxy type; };
template<> struct volume_calipers_tile_compare_for_dir<yminus> { typedef tile_compare_zxy type; };
template<> struct volume_calipers_tile_compare_for_dir<zplus > { typedef tile_compare_xyz type; };
template<> struct volume_calipers_tile_compare_for_dir<zminus> { typedef tile_compare_xyz type; };

// TileLocationIsPartOfVolume is a predicate (functor returning bool)
// with a member: static const level_of_tile_realization_needed realineeded = ...;
template<cardinal_direction Dir, typename TileLocationIsPartOfVolume>
struct volume_calipers {
  typedef typename volume_calipers_tile_compare_for_dir<Dir>::type tile_compare;
  static const level_of_tile_realization_needed realineeded = TileLocationIsPartOfVolume::realineeded;
  static const cardinal_direction forward  = Dir;
  static const cardinal_direction backward = cdir_info<Dir>::opposite;

  // This set contains the boundary tiles that are part of the volume,
  // but not the boundary tiles that are non-volume.
  set<tile_location, tile_compare> boundary_tiles_in_dimension;
  TileLocationIsPartOfVolume predicate;

  // These functions must be called whenever a relevant tile
  // changes its TileLocationIsPartOfVolume status.
  // ("inserted" = predicate is true, "removed" = predicate is false)
  void handle_tile_insertion(tile_location const& loc) {
    // We *may* have removed boundaries in either direction, and we *may* now be a boundary tile ourselves.
    const tile_location further_in_positive_direction_loc = loc.get_neighbor<forward >(realineeded);
    const tile_location further_in_negative_direction_loc = loc.get_neighbor<backward>(realineeded);
    bool we_are_boundary_tile = false;
    if (predicate(further_in_positive_direction_loc)) {
      if (predicate(further_in_positive_direction_loc.get_neighbor<forward >(realineeded))) {
        boundary_tiles_in_dimension.erase(further_in_positive_direction_loc);
      }
    }
    else we_are_boundary_tile = true;
    if (predicate(further_in_negative_direction_loc)) {
      if (predicate(further_in_negative_direction_loc.get_neighbor<backward>(realineeded))) {
        boundary_tiles_in_dimension.erase(further_in_negative_direction_loc);
      }
    }
    else we_are_boundary_tile = true;

    if (we_are_boundary_tile) boundary_tiles_in_dimension.insert(loc);
  }
  void handle_tile_removal(tile_location const& loc) {
    // This tile is no longer groupable at all, so it can't be a boundary tile
    boundary_tiles_in_dimension.erase(loc);

    // If there are groupable tiles next to us, they must now be boundary tiles,
    // because our deletion exposed them
    const tile_location further_in_positive_direction_loc = loc.get_neighbor<forward >(realineeded);
    const tile_location further_in_negative_direction_loc = loc.get_neighbor<backward>(realineeded);
    if (predicate(further_in_positive_direction_loc)) {
      boundary_tiles_in_dimension.insert(further_in_positive_direction_loc);
    }
    if (predicate(further_in_negative_direction_loc)) {
      boundary_tiles_in_dimension.insert(further_in_negative_direction_loc);
    }
  }
};

struct is_groupable_water {
  static const level_of_tile_realization_needed realineeded = CONTENTS_ONLY;
  bool operator()(tile_location const& loc)const {
    return loc.stuff_at().contents() == GROUPABLE_WATER;
  }
};

// We could easily keep lists of boundary tiles in all three dimensions.
// The only reason we don't is because there's no need for any of the others right now.
// (And it would take that much extra space (proportional to the that-dimension surface area)
//   and time (proportional to how much the groupable-water landscape changes)).
typedef volume_calipers<xplus, is_groupable_water> groupable_water_volume_calipers_t;


struct state_t {
  //state_t(world_collision_detector& d):next_water_group_identifier(1), things_exposed_to_collision(d){}
  state_t(world& w):next_water_group_identifier(1), access_the_world(w), rng(w.get_rng()){}

  water_group_identifier next_water_group_identifier;
  water_groups_by_location_t water_groups_by_surface_tile;
  persistent_water_groups_t persistent_water_groups;
  groupable_water_volume_calipers_t groupable_water_volume_calipers;
  active_fluids_t active_fluids;
  altered_minerals_info_t altered_minerals_info;

  //only used in replace_substance(), to get world_collision_detector& things_exposed_to_collision:
  world& access_the_world;

  large_fast_noncrypto_rng& rng;
};

} // end namespace tile_physics_impl



#endif

