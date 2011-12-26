
#include "world.hpp"

namespace hacky_internals {

  // When a worldblock is inited, it DOESN'T call insert_water for all the water in it - the water is 'already there'.
  // Water that starts out in a worldblock starts out inactive (observing the rule "the landscape takes zero time to process").
  //
  // We would have to make special rules for worldblocks that start out with
  // active water in them, because it could invalidate iterators into the
  // active_tiles map, because worldblocks can be created essentially any time in the processing.
  // TODO: "init_if_needed" is because we don't know how to make unordered_map's mapped_types be constructed in place in a non-default way.
  worldblock& worldblock::init_if_needed(world *w_, vector3<location_coordinate> global_position_) {
    if (!inited) {
      w = w_;
      global_position = global_position_;
      axis_aligned_bounding_box bounds{global_position, vector3<location_coordinate>(worldblock_dimension,worldblock_dimension,worldblock_dimension)};
      w->worldgen_function(world_building_gun(w, bounds), bounds);
      std::cerr << "pirates";
      inited = true;
    }
    return (*this);
  }

  tile& worldblock::get_tile(vector3<location_coordinate> global_coords) {
    vector3<location_coordinate> local_coords = global_coords - global_position;
    return tiles[local_coords.x][local_coords.y][local_coords.z];
  }

  location worldblock::get_neighboring_loc(vector3<location_coordinate> const& old_coords, cardinal_direction dir) {
    // this could be made more effecient, but I'm not sure how
    vector3<location_coordinate> new_coords = old_coords + dir.v;
    if (new_coords.x < global_position.x) return location(new_coords, neighbors[cdir_xminus]);
    if (new_coords.y < global_position.y) return location(new_coords, neighbors[cdir_yminus]);
    if (new_coords.z < global_position.z) return location(new_coords, neighbors[cdir_zminus]);
    if (new_coords.x >= global_position.x + worldblock_dimension) return location(new_coords, neighbors[cdir_xplus]);
    if (new_coords.y >= global_position.y + worldblock_dimension) return location(new_coords, neighbors[cdir_yplus]);
    if (new_coords.z >= global_position.z + worldblock_dimension) return location(new_coords, neighbors[cdir_zplus]);
    return location(new_coords, this);
  }

  location worldblock::get_loc_across_boundary(vector3<location_coordinate> const& new_coords, cardinal_direction dir) {
    if (worldblock* neighbor = neighbors[dir]) return location(new_coords, neighbor);
    return location(new_coords, (neighbors[dir] = w->create_if_necessary_and_get_worldblock(global_position +    vector3<worldblock_dimension_type>(dir.v) * worldblock_dimension)));
  }

  location worldblock::get_loc_guaranteed_to_be_in_this_block(vector3<location_coordinate> coords) {
    return location(coords, this);
  }

}


location location::operator+(cardinal_direction dir)const {
  return wb->get_neighboring_loc(v, dir);
}
tile const& location::stuff_at()const { return wb->get_tile(v); }

location world::make_location(vector3<location_coordinate> const& coords) {
  return create_if_necessary_and_get_worldblock(vector3<location_coordinate>(coords.x & ~(hacky_internals::worldblock_dimension-1), coords.y & ~(hacky_internals::worldblock_dimension-1), coords.z & ~(hacky_internals::worldblock_dimension-1)))->get_loc_guaranteed_to_be_in_this_block(coords);
}

hacky_internals::worldblock* world::create_if_necessary_and_get_worldblock(vector3<location_coordinate> position) {
  return &(blocks[position].init_if_needed(this, position));
}


