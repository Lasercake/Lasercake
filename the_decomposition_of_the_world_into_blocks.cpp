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

  // For initialization purposes.  May only be called on a tile marked interior.
  void initialize_to_non_interior(
      worldblock* wb,
      tile& t,
      worldblock_dimension_type local_x, worldblock_dimension_type local_y, worldblock_dimension_type local_z,
      tiles_collision_detector& tiles_exposed_to_collision) {

    assert_if_ASSERT_EVERYTHING(t.is_interior());
    assert_if_ASSERT_EVERYTHING(&t == &wb->tiles_[local_x*worldblock_x_factor + local_y*worldblock_y_factor + local_z*worldblock_z_factor]);
    t.set_interiorness(false);
    wb->count_of_non_interior_tiles_here_ += 1;

    const vector3<tile_coordinate>& gp = wb->global_position_;
    const vector3<tile_coordinate> coords(gp.x + local_x, gp.y + local_y, gp.z + local_z);
    const tile_location loc = wb->get_loc_guaranteed_to_be_in_this_block(coords);
    tiles_exposed_to_collision.insert(loc, tile_coords_to_tiles_collision_detector_bbox(coords));
  }

  template<cardinal_direction Dir> struct neighbor_idx_offset;
  template<> struct neighbor_idx_offset<xminus> { static const worldblock_dimension_type value = -worldblock_x_factor; };
  template<> struct neighbor_idx_offset<yminus> { static const worldblock_dimension_type value = -worldblock_y_factor; };
  template<> struct neighbor_idx_offset<zminus> { static const worldblock_dimension_type value = -worldblock_z_factor; };
  template<> struct neighbor_idx_offset<xplus > { static const worldblock_dimension_type value =  worldblock_x_factor; };
  template<> struct neighbor_idx_offset<yplus > { static const worldblock_dimension_type value =  worldblock_y_factor; };
  template<> struct neighbor_idx_offset<zplus > { static const worldblock_dimension_type value =  worldblock_z_factor; };

  template<cardinal_direction Dir>
  inline void initialize_tile_neighbor_interiorness_within_worldblock(
      worldblock* wb,
      worldblock_dimension_type x, worldblock_dimension_type y, worldblock_dimension_type z,
      size_t idx,
      tiles_collision_detector& tiles_exposed_to_collision) {
    switch (Dir) {
      case xminus: if(x == 0) {return;} break;
      case yminus: if(y == 0) {return;} break;
      case zminus: if(z == 0) {return;} break;
      case xplus: if(x == worldblock_dimension-1) {return;} break;
      case yplus: if(y == worldblock_dimension-1) {return;} break;
      case zplus: if(z == worldblock_dimension-1) {return;} break;
    }
    const size_t adj_idx = idx + neighbor_idx_offset<Dir>::value;
    tile& t = wb->tiles_[idx];
    tile& adj_tile = wb->tiles_[adj_idx];
    if(neighboring_tiles_with_these_contents_are_not_interior(t.contents(), adj_tile.contents())) {
      if(t.is_interior() && t.contents() != AIR) {
        initialize_to_non_interior(wb, t, x, y, z, tiles_exposed_to_collision);
      }
      if(adj_tile.is_interior() && adj_tile.contents() != AIR) {
        initialize_to_non_interior(wb, adj_tile,
                                    x + cdir_info<Dir>::x_delta,
                                    y + cdir_info<Dir>::y_delta,
                                    z + cdir_info<Dir>::z_delta,
                                    tiles_exposed_to_collision);
      }
    }
  }

  // Argument coordinates are worldblock indices, not world-global.
  template<cardinal_direction Dir>
  inline void worldblock::check_local_caches_cross_worldblock_neighbor(
        size_t this_x, size_t this_y, size_t this_z,
        size_t that_x, size_t that_y, size_t that_z) {
    const size_t this_idx = this_x*worldblock_x_factor + this_y*worldblock_y_factor + this_z*worldblock_z_factor;
    const size_t that_idx = that_x*worldblock_x_factor + that_y*worldblock_y_factor + that_z*worldblock_z_factor;
    if( tiles_[this_idx].contents() != AIR && tiles_[this_idx].is_interior() &&
        neighboring_tiles_with_these_contents_are_not_interior(
          tiles_[this_idx].contents(),
          neighbors_[Dir]->tiles_[that_idx].contents())
      ) {
      initialize_to_non_interior(this, tiles_[this_idx], this_x, this_y, this_z, w_->tiles_exposed_to_collision_);
    }
  }

  //Best out of 3.  Just taking one random location would be statistically pretty good
  //but could be twice as slow average as we'd like: for example a bunch of worldblocks
  //with all rock except one rubble (or a few) would pick rubble one in N times and each
  //of those times would be almost N times slower (where N = worldblock_dimension cubed).
  tile_contents worldblock::estimate_most_frequent_tile_contents_type()const {
    // It doesn't really matter what RNG we use here - having good
    // randomness only affects speed.  The *correctness* of worldblock
    // would be maintained even with an RNG here that always returned 0.
    auto& rng = w_->get_rng();
    const boost::random::uniform_int_distribution<worldblock_dimension_type> random_tile(
              0, worldblock_volume - 1);
    const tile_contents try1 = tiles_[random_tile(rng)].contents();
    const tile_contents try2 = tiles_[random_tile(rng)].contents();
    if(try1 == try2) return try1;
    else return tiles_[random_tile(rng)].contents();
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
  worldblock& worldblock::ensure_realization_impl(level_of_tile_realization_needed realineeded) {
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

      // Per world_building_gun, tiles start out marked interior
      // (usually the common case, and also permits the algorithms here)
      // and don't need to be changed if they actually are interior.

      const tile_contents estimated_most_frequent_tile_contents_type = this->estimate_most_frequent_tile_contents_type();

      bool contents_are_all_in_the_same_interiorness_class = true;
      const uint64_t frequent_contents_mask = LASERCAKE_MAKE_UINT64_MASK_FROM_UINT8(
                tile::interior_bit_mask | estimated_most_frequent_tile_contents_type);
      for(size_t i = 0; i != worldblock_volume/8; ++i) {
        // If we make interiorness classes of more than one thing
        // then TODO handle them better here.
        if(tile_data_uint64_array[i] != frequent_contents_mask) {
          contents_are_all_in_the_same_interiorness_class = false;
          break;
        }
      }

      if(!contents_are_all_in_the_same_interiorness_class) {
        // initialize_tile_neighbor_interiorness_within_worldblock initializes both the specified tile and its neighbors.
        size_t idx = 0;
        for (tile_coordinate x = 0; x != worldblock_dimension; ++x) {
          for (tile_coordinate y = 0; y != worldblock_dimension; ++y) {
            for (tile_coordinate z = 0; z != worldblock_dimension; ++z, ++idx) {
              if (tiles_[idx].contents() != estimated_most_frequent_tile_contents_type) {
                initialize_tile_neighbor_interiorness_within_worldblock<xminus>(this, x,y,z, idx, w_->tiles_exposed_to_collision_);
                initialize_tile_neighbor_interiorness_within_worldblock<yminus>(this, x,y,z, idx, w_->tiles_exposed_to_collision_);
                initialize_tile_neighbor_interiorness_within_worldblock<zminus>(this, x,y,z, idx, w_->tiles_exposed_to_collision_);
                initialize_tile_neighbor_interiorness_within_worldblock<xplus>(this, x,y,z, idx, w_->tiles_exposed_to_collision_);
                initialize_tile_neighbor_interiorness_within_worldblock<yplus>(this, x,y,z, idx, w_->tiles_exposed_to_collision_);
                initialize_tile_neighbor_interiorness_within_worldblock<zplus>(this, x,y,z, idx, w_->tiles_exposed_to_collision_);
              }
            }
          }
        }
      }
      // Air is always interior, but this worldblock is anything but all-air,
      // we have to check whether the neighbors make our border tiles non-interior.
      bool this_worldblock_is_all_air = contents_are_all_in_the_same_interiorness_class
                                    && estimated_most_frequent_tile_contents_type == AIR;
      if(!this_worldblock_is_all_air) {
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

        // check_local_caches_cross_worldblock_neighbor initializes
        // the specified tile but not its neighbor.
        for (tile_coordinate x = 0; x != worldblock_dimension; ++x) {
          for (tile_coordinate y = 0; y != worldblock_dimension; ++y) {
            const tile_coordinate z1 = 0;
            const tile_coordinate z2 = worldblock_dimension - 1;
            this->check_local_caches_cross_worldblock_neighbor<zminus>(x,y,z1, x,y,z2);
            this->check_local_caches_cross_worldblock_neighbor<zplus >(x,y,z2, x,y,z1);
          }
        }
        for (tile_coordinate x = 0; x != worldblock_dimension; ++x) {
          for (tile_coordinate z = 0; z != worldblock_dimension; ++z) {
            const tile_coordinate y1 = 0;
            const tile_coordinate y2 = worldblock_dimension - 1;
            this->check_local_caches_cross_worldblock_neighbor<yminus>(x,y1,z, x,y2,z);
            this->check_local_caches_cross_worldblock_neighbor<yplus >(x,y2,z, x,y1,z);
          }
        }
        for (tile_coordinate y = 0; y != worldblock_dimension; ++y) {
          for (tile_coordinate z = 0; z != worldblock_dimension; ++z) {
            const tile_coordinate x1 = 0;
            const tile_coordinate x2 = worldblock_dimension - 1;
            this->check_local_caches_cross_worldblock_neighbor<xminus>(x1,y,z, x2,y,z);
            this->check_local_caches_cross_worldblock_neighbor<xplus >(x2,y,z, x1,y,z);
          }
        }
      }

      // TODO: if
      //     count_of_non_interior_tiles_here_ == 0
      //     && contents_are_all_in_the_same_interiorness_class
      // or especially if contents are all the same tile type and
      // are interior, perhaps we should arrange to have them not
      // exist in memory at all?
      
      current_tile_realization_ = CONTENTS_AND_LOCAL_CACHES_ONLY;
      is_busy_realizing_ = false;
    }
    
    if ((              realineeded >= FULL_REALIZATION) &&
        (current_tile_realization_ <  FULL_REALIZATION)) {
    
      caller_error_if(is_busy_realizing_, "Referring to a realization level currently being computed");
      is_busy_realizing_ = true;

      // Optimization:
      if (count_of_non_interior_tiles_here_ == 0) {
        // This check is to make sure that if we start inside a giant ocean, we'll still
        // be able to find out what water-group we're inside of:
        if(tiles_[0].contents() == GROUPABLE_WATER) {
          w_->initialize_tile_water_group_caches_(tile_location(global_position_, this));
        }
      }
      else {
        // I tried iterating the tile_collision_detector and it was about the
        // same speed as this where I tested.  (Maybe once we switch to a
        // faster tile-search structure it'd be worth it.) -Isaac
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
      }
      
      current_tile_realization_ = FULL_REALIZATION;
      is_busy_realizing_ = false;
    }
    
    return (*this);
  }

  template<cardinal_direction Dir> tile_location worldblock::get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, level_of_tile_realization_needed realineeded) {
    worldblock* neighbor = &ensure_neighbor_realization<Dir>(realineeded);
    return tile_location(new_coords, neighbor);
  }

  void worldblock::realize_nonexistent_neighbor(cardinal_direction dir, level_of_tile_realization_needed realineeded) {
    neighbors_[dir] =
        w_->ensure_realization_of_and_get_worldblock_(
          global_position_ + vector3<worldblock_dimension_type>(cardinal_direction_vectors[dir]) * worldblock_dimension,
          realineeded
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

void world::ensure_realization_of_space(tile_bounding_box space, level_of_tile_realization_needed realineeded) {
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


