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

  // When a worldblock is inited, it DOESN'T call insert_water for all the water in it - the water is 'already there'.
  // Water that starts out in a worldblock starts out inactive (observing the rule "the landscape takes zero time to process").
  //
  // We would have to make special rules for worldblocks that start out with
  // active water in them, because it could invalidate iterators into the
  // active_water_tiles map, because worldblocks can be created essentially any time in the processing.
  // TODO: "init_if_needed" is because we don't know how to make unordered_map's mapped_types be constructed in place in a non-default way.
  worldblock& worldblock::init_if_needed(world *w_, vector3<tile_coordinate> global_position_) {
    if (!inited) {
      inited = true;
      w = w_;
      global_position = global_position_;
      tile_bounding_box bounds{global_position, vector3<tile_coordinate>(worldblock_dimension,worldblock_dimension,worldblock_dimension)};
      w->worldgen_function(world_building_gun(w, bounds), bounds);
      std::cerr << "A worldblock has been created!\n";
    }
    return (*this);
  }

  tile& worldblock::get_tile(vector3<tile_coordinate> global_coords) {
    vector3<tile_coordinate> local_coords = global_coords - global_position;
    return tiles[local_coords.x][local_coords.y][local_coords.z];
  }

  tile_location worldblock::get_neighboring_loc(vector3<tile_coordinate> const& old_coords, cardinal_direction dir) {
    // this could be made more effecient, but I'm not sure how
    vector3<tile_coordinate> new_coords = old_coords + dir.v;
    if (new_coords.x < global_position.x) return get_loc_across_boundary(new_coords, cdir_xminus);
    if (new_coords.y < global_position.y) return get_loc_across_boundary(new_coords, cdir_yminus);
    if (new_coords.z < global_position.z) return get_loc_across_boundary(new_coords, cdir_zminus);
    if (new_coords.x >= global_position.x + worldblock_dimension) return get_loc_across_boundary(new_coords, cdir_xplus);
    if (new_coords.y >= global_position.y + worldblock_dimension) return get_loc_across_boundary(new_coords, cdir_yplus);
    if (new_coords.z >= global_position.z + worldblock_dimension) return get_loc_across_boundary(new_coords, cdir_zplus);
    return tile_location(new_coords, this);
  }

  tile_location worldblock::get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, cardinal_direction dir) {
    if (worldblock* neighbor = neighbors[dir]) return tile_location(new_coords, neighbor);
    return tile_location(new_coords, (neighbors[dir] = w->create_if_necessary_and_get_worldblock(global_position +    vector3<worldblock_dimension_type>(dir.v) * worldblock_dimension)));
  }

  tile_location worldblock::get_loc_guaranteed_to_be_in_this_block(vector3<tile_coordinate> coords) {
    return tile_location(coords, this);
  }

}


tile_location tile_location::operator+(cardinal_direction dir)const {
  return wb->get_neighboring_loc(v, dir);
}
tile const& tile_location::stuff_at()const { return wb->get_tile(v); }

tile_location world::make_tile_location(vector3<tile_coordinate> const& coords) {
  return create_if_necessary_and_get_worldblock(vector3<tile_coordinate>(
      coords.x & ~(hacky_internals::worldblock_dimension-1),
      coords.y & ~(hacky_internals::worldblock_dimension-1),
      coords.z & ~(hacky_internals::worldblock_dimension-1)
    ))->get_loc_guaranteed_to_be_in_this_block(coords);
}

hacky_internals::worldblock* world::create_if_necessary_and_get_worldblock(vector3<tile_coordinate> position) {
  return &(blocks[position].init_if_needed(this, position));
}

void world::ensure_space_exists(tile_bounding_box space) {
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
        create_if_necessary_and_get_worldblock(worldblock_position);
      }
    }
  }
}


