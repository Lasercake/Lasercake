
#include "world.hpp"

const int worldblock_dimension_exp = 4;
typedef int worldblock_dimension_type;
const worldblock_dimension_type worldblock_dimension = (1 << worldblock_dimension_exp);

class worldblock {
  // When a worldblock is inited, it DOESN'T call insert_water for all the water in it - the water is 'already there'.
  // Water that starts out in a worldblock starts out inactive (observing the rule "the landscape takes zero time to process").
  //
  // We would have to make special rules for worldblocks that start out with
  // active water in them, because it could invalidate iterators into the
  // active_tiles map, because worldblocks can be created essentially any time in the processing.
private:
  friend class location;
  friend class world;
  
  worldblock& init_if_needed(world *w_) {
    if (!inited) {
      w = w_;
    }
    return (*this);
  }
  
  std::array<std::array<std::array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles;
  value_for_each_cardinal_direction<worldblock*> neighbors;
  vector3<location_coordinate> global_position; // the lowest x, y, and z among elements in this worldblock
  world *w;
  bool inited;
  
  // Only to be used in location::stuff_at():
  tile &get_tile(vector3<location_coordinate> global_coords) {
    vector3<location_coordinate> local_coords = global_coords - global_position;
    return tiles[local_coords.x][local_coords.y][local_coords.z];
  }
  
  // Only to be used in location::operator+(cardinal_direction):
  location get_neighboring_loc(vector3<location_coordinate> const& old_coords, cardinal_direction dir) {
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
  
  location get_loc_across_boundary(vector3<location_coordinate> const& new_coords, cardinal_direction dir) {
    if (worldblock* neighbor = neighbors[dir]) return location(new_coords, neighbor);
    return location(new_coords, (neighbors[dir] = w->create_if_necessary_and_get_worldblock(global_position + vector3<worldblock_dimension_type>(dir.v) * worldblock_dimension)));
  }
};


location location::operator+(cardinal_direction dir)const {
  return wb->get_neighboring_loc(v, dir);
}
tile const& location::stuff_at()const { return wb->get_tile(v); }

