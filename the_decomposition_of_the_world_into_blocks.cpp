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
#include "worldgen.hpp"

using namespace the_decomposition_of_the_world_into_blocks_impl;

namespace the_decomposition_of_the_world_into_blocks_impl {

  template<cardinal_direction Dir>
  void worldblock::check_local_caches_cross_worldblock_neighbor(tile_coordinate x, tile_coordinate y, tile_coordinate z) {
    const vector3<tile_coordinate> coords(x,y,z);
    tile& here = this->get_tile(coords);

    vector3<tile_coordinate> adj_coords = coords; cdir_info<Dir>::add_to(adj_coords);
    const tile adj = neighbors_[Dir]->get_tile(adj_coords);

    if(neighboring_tiles_with_these_contents_are_not_interior(adj.contents(), here.contents())) {
      const tile_location loc(coords, this);
      w_->initialize_tile_local_caches_relating_to_this_neighbor_(loc, adj);
    }
  }

  //Best out of 3.  Just taking one random location would be statistically pretty good
  //but could be twice as slow average as we'd like: for example a bunch of worldblocks
  //with all rock except one rubble (or a few) would pick rubble one in N times and each
  //of those times would be almost N times slower (where N = worldblock_dimension cubed).
  tile_contents worldblock::estimate_most_frequent_tile_contents_type()const {
    const tile_contents try1 = tiles_[rand()%worldblock_dimension][rand()%worldblock_dimension][rand()%worldblock_dimension].contents();
    const tile_contents try2 = tiles_[rand()%worldblock_dimension][rand()%worldblock_dimension][rand()%worldblock_dimension].contents();
    if(try1 == try2) return try1;
    else return tiles_[rand()%worldblock_dimension][rand()%worldblock_dimension][rand()%worldblock_dimension].contents();
  }

  // Water that starts out in a worldblock starts out inactive (observing the rule "the landscape takes zero time to process").
  //
  // We would have to make special rules for worldblocks that start out with
  // active water in them, because it could invalidate iterators into the
  // active_water_tiles map, because worldblocks can be created essentially any time in the processing.
  //
  // If we try to use a level-X value from a worldblock while it's busy computing a realization
  // level less-than-or-equal-to X, then we justly receive get an assertion failure.
  // Realizing a worldblock at a given level must not require same-level information.
  worldblock& worldblock::ensure_realization(level_of_tile_realization_needed realineeded) {
    // This function gets called to do nothing a LOT more than it gets called to actually do something;
    // bail ASAP if we don't have to do anything.
    if (realineeded <= current_tile_realization_) return *this;

    assert(this->is_constructed());
    caller_correct_if(realineeded >= COMPLETELY_IMAGINARY && realineeded <= FULL_REALIZATION, "Calling ensure_realization with an invalid realization level");
    
    if ((              realineeded >= CONTENTS_ONLY) &&
        (current_tile_realization_ <  CONTENTS_ONLY)) {
    
      caller_error_if(is_busy_realizing_, "Referring to a realization level currently being computed");
      is_busy_realizing_ = true;

      tile_bounding_box bounds(global_position_, vector3<tile_coordinate>(worldblock_dimension,worldblock_dimension,worldblock_dimension));
      w_->worldgen_function_(world_building_gun(w_, bounds, this), bounds);
      //std::cerr << "A worldblock has been created!\n";
      
      current_tile_realization_ = CONTENTS_ONLY;
      is_busy_realizing_ = false;
    }
    
    if ((              realineeded >= CONTENTS_AND_LOCAL_CACHES_ONLY) &&
        (current_tile_realization_ <  CONTENTS_AND_LOCAL_CACHES_ONLY)) {
    
      caller_error_if(is_busy_realizing_, "Referring to a realization level currently being computed");
      is_busy_realizing_ = true;

      const tile_contents estimated_most_frequent_tile_contents_type = this->estimate_most_frequent_tile_contents_type();
      
      for (tile_coordinate x = global_position_.x; x < global_position_.x + worldblock_dimension; ++x) {
        for (tile_coordinate y = global_position_.y; y < global_position_.y + worldblock_dimension; ++y) {
          for (tile_coordinate z = global_position_.z; z < global_position_.z + worldblock_dimension; ++z) {
            const vector3<tile_coordinate> coords(x,y,z);
            tile& here = this->get_tile(coords);
            if(here.contents() != estimated_most_frequent_tile_contents_type) {
              const tile_location loc(coords, this);
              w_->initialize_tile_local_caches_(loc);
            }
          }
        }
      }

      // We now have to explicitly check all the neighbor-faces with neighboring worldblocks,
      // because our optimization to skip the locally-most-common tile contents doesn't help
      // with these edges (both because the neighboring worldblock might not have the same
      // most-common-tile as us, and because that worldblock likely won't be
      // CONTENTS_AND_LOCAL_CACHES_ONLY-initialized at the same time as us).
      //
      // There are 6*(worldblock_dimension**2) of these faces-with-foreign-neighbors.
      // This is a lot better than naively checking all 6*(worldblock_dimension**3) faces
      // this worldblock's tiles contain.  This is also better than checking all the
      // 6*(nearly 6*(worldblock_dimension**2)) faces of this worldblock's edge *tiles*.
      
      // By ensuring neighbors' realization first, realization doesn't have to be checked for
      // every get_loc_across_boundary-equivalent (all 6*(worldblock_dimension**2) of them).
      ensure_neighbor_realization<xminus>(CONTENTS_ONLY);
      ensure_neighbor_realization<yminus>(CONTENTS_ONLY);
      ensure_neighbor_realization<zminus>(CONTENTS_ONLY);
      ensure_neighbor_realization<xplus>(CONTENTS_ONLY);
      ensure_neighbor_realization<yplus>(CONTENTS_ONLY);
      ensure_neighbor_realization<zplus>(CONTENTS_ONLY);

      // It makes enough of a performance difference to do this
      // check_local_caches_cross_worldblock_neighbor rather than
      // just call world::initialize_tile_local_caches_relating_to_this_neighbor_.
      
      for (tile_coordinate x = global_position_.x; x < global_position_.x + worldblock_dimension; ++x) {
        for (tile_coordinate y = global_position_.y; y < global_position_.y + worldblock_dimension; ++y) {
          {
            const tile_coordinate z = global_position_.z;
            this->check_local_caches_cross_worldblock_neighbor<zminus>(x,y,z);
          }
          {
            const tile_coordinate z = global_position_.z + worldblock_dimension - 1;
            this->check_local_caches_cross_worldblock_neighbor<zplus>(x,y,z);
          }
        }
      }
      for (tile_coordinate x = global_position_.x; x < global_position_.x + worldblock_dimension; ++x) {
        for (tile_coordinate z = global_position_.z; z < global_position_.z + worldblock_dimension; ++z) {
          {
            const tile_coordinate y = global_position_.y;
            this->check_local_caches_cross_worldblock_neighbor<yminus>(x,y,z);
          }
          {
            const tile_coordinate y = global_position_.y + worldblock_dimension - 1;
            this->check_local_caches_cross_worldblock_neighbor<yplus>(x,y,z);
          }
        }
      }
      for (tile_coordinate y = global_position_.y; y < global_position_.y + worldblock_dimension; ++y) {
        for (tile_coordinate z = global_position_.z; z < global_position_.z + worldblock_dimension; ++z) {
          {
            const tile_coordinate x = global_position_.x;
            this->check_local_caches_cross_worldblock_neighbor<xminus>(x,y,z);
          }
          {
            const tile_coordinate x = global_position_.x + worldblock_dimension - 1;
            this->check_local_caches_cross_worldblock_neighbor<xplus>(x,y,z);
          }
        }
      }
      
      current_tile_realization_ = CONTENTS_AND_LOCAL_CACHES_ONLY;
      is_busy_realizing_ = false;
    }
    
    if ((              realineeded >= FULL_REALIZATION) &&
        (current_tile_realization_ <  FULL_REALIZATION)) {
    
      caller_error_if(is_busy_realizing_, "Referring to a realization level currently being computed");
      is_busy_realizing_ = true;
      
      for (tile_coordinate x = global_position_.x; x < global_position_.x + worldblock_dimension; ++x) {
        for (tile_coordinate y = global_position_.y; y < global_position_.y + worldblock_dimension; ++y) {
          for (tile_coordinate z = global_position_.z; z < global_position_.z + worldblock_dimension; ++z) {
            const vector3<tile_coordinate> coords(x,y,z);
            tile& here = this->get_tile(coords);
            // Checking contents() here: significant speed improvement.
            // (Some from the inlining, some from not having to construct a tile_location if not GROUPABLE_WATER,
            // I believe.) --Isaac
            if (here.contents() == GROUPABLE_WATER) {
              w_->initialize_tile_water_group_caches_(tile_location(coords, this));
            }
          }
        }
      }
      
      current_tile_realization_ = FULL_REALIZATION;
      is_busy_realizing_ = false;
    }
    
    return (*this);
  }
  
  template<> bool worldblock::crossed_boundary<xminus>(tile_coordinate new_coord) { return new_coord < global_position_.x; }
  template<> bool worldblock::crossed_boundary<yminus>(tile_coordinate new_coord) { return new_coord < global_position_.y; }
  template<> bool worldblock::crossed_boundary<zminus>(tile_coordinate new_coord) { return new_coord < global_position_.z; }
  template<> bool worldblock::crossed_boundary<xplus>(tile_coordinate new_coord) { return new_coord >= global_position_.x + worldblock_dimension; }
  template<> bool worldblock::crossed_boundary<yplus>(tile_coordinate new_coord) { return new_coord >= global_position_.y + worldblock_dimension; }
  template<> bool worldblock::crossed_boundary<zplus>(tile_coordinate new_coord) { return new_coord >= global_position_.z + worldblock_dimension; }

  template<cardinal_direction Dir> tile_location worldblock::get_neighboring_loc(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded) {
    ensure_realization(realineeded);
    vector3<tile_coordinate> new_coords = old_coords; cdir_info<Dir>::add_to(new_coords);
    if (crossed_boundary<Dir>(new_coords[cdir_info<Dir>::dimension])) return tile_location(new_coords, &ensure_neighbor_realization<Dir>(realineeded));
    else return tile_location(new_coords, this);
  }

  template tile_location worldblock::get_neighboring_loc<xminus>(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);
  template tile_location worldblock::get_neighboring_loc<yminus>(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);
  template tile_location worldblock::get_neighboring_loc<zminus>(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);
  template tile_location worldblock::get_neighboring_loc<xplus>(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);
  template tile_location worldblock::get_neighboring_loc<yplus>(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);
  template tile_location worldblock::get_neighboring_loc<zplus>(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);

  template<cardinal_direction Dir> worldblock& worldblock::ensure_neighbor_realization(level_of_tile_realization_needed realineeded) {
    if (worldblock* neighbor = neighbors_[Dir]) {
      return neighbor->ensure_realization(realineeded);
    }
    else {
      return *(neighbors_[Dir] =
        w_->ensure_realization_of_and_get_worldblock_(
          global_position_ + vector3<worldblock_dimension_type>(cdir_info<Dir>::as_vector()) * worldblock_dimension,
          realineeded
        )
      );
    }
  }

  // It seems this function is sometimes faster and sometimes slower than
  //   tile_location(new_coords, &ensure_neighbor_realization<Dir>(realineeded));
  // ...what?  TODO experiment with the use in worldblock::get_neighboring_loc and/or delete this
  template<cardinal_direction Dir> tile_location worldblock::get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, level_of_tile_realization_needed realineeded) {
    if (worldblock* neighbor = neighbors_[Dir]) {
      neighbor->ensure_realization(realineeded);
      return tile_location(new_coords, neighbor);
    }
    else return tile_location(
      new_coords,
      (
        neighbors_[Dir] =
        w_->ensure_realization_of_and_get_worldblock_(
          global_position_ + vector3<worldblock_dimension_type>(cdir_info<Dir>::as_vector()) * worldblock_dimension,
          realineeded
        )
      )
    );
  }


  tile_location worldblock::get_loc_guaranteed_to_be_in_this_block(vector3<tile_coordinate> coords) {
    return tile_location(coords, this);
  }

}


tile_location tile_location::get_neighbor_by_variable(cardinal_direction dir, level_of_tile_realization_needed realineeded)const {
  switch(dir) {
    case xminus: return wb_->get_neighboring_loc<xminus>(v_, realineeded);
    case yminus: return wb_->get_neighboring_loc<yminus>(v_, realineeded);
    case zminus: return wb_->get_neighboring_loc<zminus>(v_, realineeded);
    case xplus: return wb_->get_neighboring_loc<xplus>(v_, realineeded);
    case yplus: return wb_->get_neighboring_loc<yplus>(v_, realineeded);
    case zplus: return wb_->get_neighboring_loc<zplus>(v_, realineeded);
    default: caller_error("calling get_neighbor_by_variable with an invalid direction");
  }
}

namespace { // anonymous
vector3<tile_coordinate> coordinates_of_containing_worldblock(vector3<tile_coordinate> const& coords) {
  return vector3<tile_coordinate>(
    coords.x & ~(worldblock_dimension-1),
    coords.y & ~(worldblock_dimension-1),
    coords.z & ~(worldblock_dimension-1)
  );
}
} // end anonymous namespace

tile_location world::make_tile_location(vector3<tile_coordinate> const& coords, level_of_tile_realization_needed realineeded) {
  return ensure_realization_of_and_get_worldblock_(coordinates_of_containing_worldblock(coords), realineeded)->get_loc_guaranteed_to_be_in_this_block(coords);
}

worldblock* world::ensure_realization_of_and_get_worldblock_(vector3<tile_coordinate> position, level_of_tile_realization_needed realineeded) {
  worldblock& wb = blocks_[position]; //default-construct in place if not there
  if(!wb.is_constructed()) {
    // Worldblocks are on the order of a kilobyte, and std containers have
    // no way to give elements proper constructor arguments for in-place
    // construction, so we do this sequence.
    wb.construct(this, position);
  }
  wb.ensure_realization(realineeded);
  return &wb;
}

void world::ensure_realization_of_space_(tile_bounding_box space, level_of_tile_realization_needed realineeded) {
  const worldblock_dimension_type wd = worldblock_dimension;
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
        ensure_realization_of_and_get_worldblock_(worldblock_position, realineeded);
      }
    }
  }
}


