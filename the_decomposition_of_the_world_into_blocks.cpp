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

namespace hacky_internals {

  // Water that starts out in a worldblock starts out inactive (observing the rule "the landscape takes zero time to process").
  //
  // We would have to make special rules for worldblocks that start out with
  // active water in them, because it could invalidate iterators into the
  // active_water_tiles map, because worldblocks can be created essentially any time in the processing.
  //
  // If we try to use a level-X value from a worldblock while it's busy computing a realization
  // level less-than-or-equal-to X, then we justly receive get an assertion failure.
  // Realizing a worldblock at a given level must not require same-level information.
  worldblock& worldblock::ensure_realization(level_of_tile_realization_needed realineeded, world *w_, vector3<tile_coordinate> global_position_) {
    assert(realineeded >= COMPLETELY_IMAGINARY);
    assert(realineeded <= FULL_REALIZATION);
    
    if ((             realineeded >= CONTENTS_ONLY) &&
        (current_tile_realization <  CONTENTS_ONLY)) {
    
      assert(!is_busy_realizing);
      is_busy_realizing = true;
      
      w = w_;
      global_position = global_position_;
      tile_bounding_box bounds(global_position, vector3<tile_coordinate>(worldblock_dimension,worldblock_dimension,worldblock_dimension));
      w->worldgen_function(world_building_gun(w, bounds), bounds);
      //std::cerr << "A worldblock has been created!\n";
      
      current_tile_realization = CONTENTS_ONLY;
      is_busy_realizing = false;
    }
    
    if ((             realineeded >= CONTENTS_AND_STICKYNESS_ONLY) &&
        (current_tile_realization <  CONTENTS_AND_STICKYNESS_ONLY)) {
    
      assert(!is_busy_realizing);
      is_busy_realizing = true;
      
      for (tile_coordinate x = global_position.x; x < global_position.x + worldblock_dimension; ++x) {
        for (tile_coordinate y = global_position.y; y < global_position.y + worldblock_dimension; ++y) {
          for (tile_coordinate z = global_position.z; z < global_position.z + worldblock_dimension; ++z) {
            tile_location loc(vector3<tile_coordinate>(x,y,z), this);
            if (should_be_sticky(loc)) {
              vector3<tile_coordinate> local_coords = loc.coords() - global_position;
              tiles[local_coords.x][local_coords.y][local_coords.z].set_water_stickyness(true);
            }
          }
        }
      }
      
      current_tile_realization = CONTENTS_AND_STICKYNESS_ONLY;
      is_busy_realizing = false;
    }
    
    if ((             realineeded >= FULL_REALIZATION) &&
        (current_tile_realization <  FULL_REALIZATION)) {
    
      assert(!is_busy_realizing);
      is_busy_realizing = true;
      
      for (tile_coordinate x = global_position.x; x < global_position.x + worldblock_dimension; ++x) {
        for (tile_coordinate y = global_position.y; y < global_position.y + worldblock_dimension; ++y) {
          for (tile_coordinate z = global_position.z; z < global_position.z + worldblock_dimension; ++z) {
            tile_location loc(vector3<tile_coordinate>(x,y,z), this);
            w->check_interiorness(loc);
            w->check_exposure_to_collision(loc);
          }
        }
      }
      
      current_tile_realization = FULL_REALIZATION;
      is_busy_realizing = false;
    }
    
    return (*this);
  }

  tile& worldblock::get_tile(vector3<tile_coordinate> global_coords) {
    vector3<tile_coordinate> local_coords = global_coords - global_position;
    return tiles[local_coords.x][local_coords.y][local_coords.z];
  }

  tile_location worldblock::get_neighboring_loc(vector3<tile_coordinate> const& old_coords, cardinal_direction dir, level_of_tile_realization_needed realineeded) {
    ensure_realization(realineeded);
    // this could be made more effecient, but I'm not sure how
    vector3<tile_coordinate> new_coords = old_coords + dir.v;
    if (new_coords.x < global_position.x) return get_loc_across_boundary(new_coords, cdir_xminus, realineeded);
    if (new_coords.y < global_position.y) return get_loc_across_boundary(new_coords, cdir_yminus, realineeded);
    if (new_coords.z < global_position.z) return get_loc_across_boundary(new_coords, cdir_zminus, realineeded);
    if (new_coords.x >= global_position.x + worldblock_dimension) return get_loc_across_boundary(new_coords, cdir_xplus, realineeded);
    if (new_coords.y >= global_position.y + worldblock_dimension) return get_loc_across_boundary(new_coords, cdir_yplus, realineeded);
    if (new_coords.z >= global_position.z + worldblock_dimension) return get_loc_across_boundary(new_coords, cdir_zplus, realineeded);
    return tile_location(new_coords, this);
  }

  tile_location worldblock::get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, cardinal_direction dir, level_of_tile_realization_needed realineeded) {
    if (worldblock* neighbor = neighbors[dir]) {
      neighbor->ensure_realization(realineeded);
      return tile_location(new_coords, neighbor);
    }
    return tile_location(
      new_coords,
      (
        neighbors[dir] =
        w->ensure_realization_of_and_get_worldblock(
          global_position + vector3<worldblock_dimension_type>(dir.v) * worldblock_dimension,
          realineeded
        )
      )
    );
  }

  tile_location worldblock::get_loc_guaranteed_to_be_in_this_block(vector3<tile_coordinate> coords) {
    return tile_location(coords, this);
  }

}


tile_location tile_location::operator+(cardinal_direction dir)const {
  return wb->get_neighboring_loc(v, dir, FULL_REALIZATION);
}
tile_location tile_location::get_neighbor(cardinal_direction dir, level_of_tile_realization_needed realineeded)const {
  return wb->get_neighboring_loc(v, dir, realineeded);
}
tile const& tile_location::stuff_at()const { return wb->get_tile(v); }

vector3<tile_coordinate> coordinates_of_containing_worldblock(vector3<tile_coordinate> const& coords) {
  return vector3<tile_coordinate>(
    coords.x & ~(hacky_internals::worldblock_dimension-1),
    coords.y & ~(hacky_internals::worldblock_dimension-1),
    coords.z & ~(hacky_internals::worldblock_dimension-1)
  );
}

tile_location world::make_tile_location(vector3<tile_coordinate> const& coords, level_of_tile_realization_needed realineeded) {
  return ensure_realization_of_and_get_worldblock(coordinates_of_containing_worldblock(coords), realineeded)->get_loc_guaranteed_to_be_in_this_block(coords);
}

hacky_internals::worldblock* world::ensure_realization_of_and_get_worldblock(vector3<tile_coordinate> position, level_of_tile_realization_needed realineeded) {
  return &(blocks[position].ensure_realization(realineeded, this, position));
}

void world::ensure_realization_of_space(tile_bounding_box space, level_of_tile_realization_needed realineeded) {
  const hacky_internals::worldblock_dimension_type wd = hacky_internals::worldblock_dimension;
  for (tile_coordinate
       x =  space.min.x                            / wd;
       x < (space.min.x + space.size.x + (wd - 1)) / wd;
       ++x) {
    for (tile_coordinate
         y =  space.min.y                            / wd;
         y < (space.min.y + space.size.y + (wd - 1)) / wd;
         ++y) {
      for (tile_coordinate
           z =  space.min.z                            / wd;
           z < (space.min.z + space.size.z + (wd - 1)) / wd;
           ++z) {
        const vector3<tile_coordinate> worldblock_position(x*wd, y*wd, z*wd);
        ensure_realization_of_and_get_worldblock(worldblock_position, realineeded);
      }
    }
  }
}


