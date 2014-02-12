/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_THE_DECOMPOSITION_OF_THE_WORLD_INTO_BLOCKS_HPP__
#define LASERCAKE_THE_DECOMPOSITION_OF_THE_WORLD_INTO_BLOCKS_HPP__

//#include <boost/variant/variant.hpp>

#include "utils.hpp"
#include "data_structures/patricia_trie.hpp"
#include "tiles.hpp"

class world;

namespace the_decomposition_of_the_world_into_blocks_impl {
  const int worldblock_dimension_exp = 4;
  const worldblock_dimension_type worldblock_dimension = (1 << worldblock_dimension_exp);
  const size_t worldblock_volume = worldblock_dimension*worldblock_dimension*worldblock_dimension;
  // For indexing worldblock tiles directly:
  const worldblock_dimension_type worldblock_x_factor = worldblock_dimension*worldblock_dimension;
  const worldblock_dimension_type worldblock_y_factor = worldblock_dimension;
  const worldblock_dimension_type worldblock_z_factor = 1;

  // x, y and z < worldblock_dimension
  inline worldblock_dimension_type interleave_worldblock_local_coords(
        worldblock_dimension_type x, worldblock_dimension_type y, worldblock_dimension_type z) {
    static const int16_t table[worldblock_dimension] = {
      0x0, 0x1, 0x8, 0x9, 0x40, 0x41, 0x48, 0x49,
      0x200, 0x201, 0x208, 0x209, 0x240, 0x241, 0x248, 0x249
    };
    return (table[x] << 2) + (table[y] << 1) + (table[z]);
  }

  class worldblock;

  struct worldblock_trie_traits : default_pow2_radix_patricia_trie_traits {
    typedef size_t monoid;
  };
  // tile_coordinates here are right-shifted by worldblock_dimension_exp
//  typedef boost::variant<boost::blank, worldblock*, tile_contents> region_specification;
  struct region_specification {
    tile_contents everything_here_is_interior_this_; // or UNSPECIFIED_TILE_CONTENTS
    worldblock* worldblock_; // or nullptr
    explicit region_specification(worldblock* wb)
      : everything_here_is_interior_this_(UNSPECIFIED_TILE_CONTENTS), worldblock_(wb) {}
    explicit region_specification(tile_contents t)
      : everything_here_is_interior_this_(t), worldblock_(nullptr) {}
    region_specification() : everything_here_is_interior_this_(UNSPECIFIED_TILE_CONTENTS), worldblock_(nullptr) {}
    explicit operator bool()const {
      return everything_here_is_interior_this_ != UNSPECIFIED_TILE_CONTENTS || worldblock_ != nullptr;
    }
  };
  typedef pow2_radix_patricia_trie_node<3, tile_coordinate, region_specification, worldblock_trie_traits>
    worldblock_trie_node;
  typedef worldblock_trie_node worldblock_trie; //makes sense if pointing to the root node


  class worldblock {
  public:
    worldblock() : neighbors_(nullptr), parent_(nullptr),
      current_tile_realization_(COMPLETELY_IMAGINARY),
      is_busy_realizing_(false), count_of_non_interior_tiles_here_(0), w_(nullptr),
      non_interior_bitmap_large_scale_(0), non_interior_bitmap_small_scale_{} {}
    ~worldblock();
    void construct(world* w, vector3<tile_coordinate> global_position);
    bool is_constructed() const { return w_ != nullptr; }
    tile_bounding_box bounding_box()const {
      return tile_bounding_box(global_position_,
        vector3<tile_coordinate>(worldblock_dimension,worldblock_dimension,worldblock_dimension));
    }
    tile_location global_position_loc() {
      return tile_location(global_position_, 0, this);
    }

    worldblock& ensure_realization(level_of_tile_realization_needed realineeded) {
      // This function gets called to do nothing a LOT more than it gets called to actually do something;
      // return ASAP if we don't have to do anything.
      if (realineeded > current_tile_realization_) { this->ensure_realization_impl(realineeded); }
      return *this;
    }
    // ensure_realization_impl is a lot of code, with nontrivial duration, and should not be inlined.
    worldblock& ensure_realization_impl(level_of_tile_realization_needed realineeded);

    static worldblock_dimension_type get_idx(vector3<tile_coordinate> global_coords) {
      return
          (get_primitive_int(global_coords.x) & (worldblock_dimension-1))*worldblock_x_factor
        + (get_primitive_int(global_coords.y) & (worldblock_dimension-1))*worldblock_y_factor
        + (get_primitive_int(global_coords.z) & (worldblock_dimension-1))*worldblock_z_factor;
    }
  
    // Prefer to use tile_location::stuff_at().
    inline tile& get_tile(vector3<tile_coordinate> global_coords) {
      return tiles_[get_idx(global_coords)];
    }

    void set_tile_non_interior(vector3<tile_coordinate> global_coords, worldblock_dimension_type idx);
    void set_tile_interior(vector3<tile_coordinate> global_coords, worldblock_dimension_type idx);

    // Returns whether this worldblock (and its neighbor faces) are
    // so uniform and interior that it's a waste of space to keep this
    // in memory.
    bool is_deletable()const;

    template<cardinal_direction Dir> worldblock& ensure_neighbor_realization(level_of_tile_realization_needed realineeded);
    void realize_nonexistent_neighbor(cardinal_direction dir, level_of_tile_realization_needed realineeded);

    // an implementation detail of ensure_realization
    template<cardinal_direction Dir> void check_local_caches_cross_worldblock_neighbor(
      size_t this_x, size_t this_y, size_t this_z, size_t that_x, size_t that_y, size_t that_z);
    tile_contents estimate_most_frequent_tile_contents_type()const;

    // contains static member functions that, as part of worldblock, are
    // friended by class world, but which don't need to be declared in
    // world.hpp:
    struct helpers;

    value_for_each_cardinal_direction<worldblock*> neighbors_;
    worldblock_trie_node* parent_;
    vector3<tile_coordinate> global_position_; // the lowest x, y, and z among elements in this worldblock
    level_of_tile_realization_needed current_tile_realization_;
    bool is_busy_realizing_;
    int32_t count_of_non_interior_tiles_here_;
    world* w_;
    //These have a hierarchy order generated by interleave_worldblock_inner_coords:
    uint64_t non_interior_bitmap_large_scale_;
    uint64_t non_interior_bitmap_small_scale_[64];

    // A three-dimensional array of tile, of worldblock_dimension in each dimension.
    // array<array<array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles_;

    // Accessing multiple members of unions as a cast does not meet the
    // standard's strict-aliasing rules[*], but most compilers treat unions of data
    // (not unions of pointers to data!) as aliasing (probably because they cannot help but be).
    // [*] only memcpy or casts to char* do - not casts from char*
    //       or casts to char-sized classes, I believe.
    union {
      tile tiles_[worldblock_volume];
      uint8_t tile_data_uint8_array[worldblock_volume];
      uint32_t tile_data_uint32_array[worldblock_volume/4];
      uint64_t tile_data_uint64_array[worldblock_volume/8];
    };
  };
}





// some worldblock-related impls here for inlining purposes (speed)
inline tile const& tile_location::stuff_at()const { return wb_->tiles_[idx_]; }
inline tile_location::tile_location(
  vector3<tile_coordinate> v,
  the_decomposition_of_the_world_into_blocks_impl::worldblock_dimension_type idx,
  the_decomposition_of_the_world_into_blocks_impl::worldblock *wb
) : v_(v), idx_(idx), wb_(wb) {
  maybe_assert(wb);
  maybe_assert(wb->bounding_box().contains(v));
  maybe_assert(idx == wb->get_idx(v));
}
namespace the_decomposition_of_the_world_into_blocks_impl {
  template<cardinal_direction Dir> bool next_to_boundary(vector3<tile_coordinate> const& coords);
  template<> inline bool next_to_boundary<xminus>(vector3<tile_coordinate> const& coords) { return (get_primitive_int(coords.x) & (worldblock_dimension-1)) == 0; }
  template<> inline bool next_to_boundary<yminus>(vector3<tile_coordinate> const& coords) { return (get_primitive_int(coords.y) & (worldblock_dimension-1)) == 0; }
  template<> inline bool next_to_boundary<zminus>(vector3<tile_coordinate> const& coords) { return (get_primitive_int(coords.z) & (worldblock_dimension-1)) == 0; }
  template<> inline bool next_to_boundary<xplus>(vector3<tile_coordinate> const& coords) { return (get_primitive_int(coords.x) & (worldblock_dimension-1)) == (worldblock_dimension-1); }
  template<> inline bool next_to_boundary<yplus>(vector3<tile_coordinate> const& coords) { return (get_primitive_int(coords.y) & (worldblock_dimension-1)) == (worldblock_dimension-1); }
  template<> inline bool next_to_boundary<zplus>(vector3<tile_coordinate> const& coords) { return (get_primitive_int(coords.z) & (worldblock_dimension-1)) == (worldblock_dimension-1); }

  template<cardinal_direction Dir> worldblock_dimension_type advance_idx_within_worldblock(worldblock_dimension_type idx);
  template<> inline worldblock_dimension_type advance_idx_within_worldblock<xminus>(worldblock_dimension_type idx) { return idx - worldblock_x_factor; }
  template<> inline worldblock_dimension_type advance_idx_within_worldblock<yminus>(worldblock_dimension_type idx) { return idx - worldblock_y_factor; }
  template<> inline worldblock_dimension_type advance_idx_within_worldblock<zminus>(worldblock_dimension_type idx) { return idx - worldblock_z_factor; }
  template<> inline worldblock_dimension_type advance_idx_within_worldblock<xplus>(worldblock_dimension_type idx) { return idx + worldblock_x_factor; }
  template<> inline worldblock_dimension_type advance_idx_within_worldblock<yplus>(worldblock_dimension_type idx) { return idx + worldblock_y_factor; }
  template<> inline worldblock_dimension_type advance_idx_within_worldblock<zplus>(worldblock_dimension_type idx) { return idx + worldblock_z_factor; }
  template<cardinal_direction Dir> worldblock_dimension_type advance_idx_across_worldblock_boundary(worldblock_dimension_type idx);
  template<> inline worldblock_dimension_type advance_idx_across_worldblock_boundary<xminus>(worldblock_dimension_type idx) { return idx + worldblock_x_factor*(worldblock_dimension-1); }
  template<> inline worldblock_dimension_type advance_idx_across_worldblock_boundary<yminus>(worldblock_dimension_type idx) { return idx + worldblock_y_factor*(worldblock_dimension-1); }
  template<> inline worldblock_dimension_type advance_idx_across_worldblock_boundary<zminus>(worldblock_dimension_type idx) { return idx + worldblock_z_factor*(worldblock_dimension-1); }
  template<> inline worldblock_dimension_type advance_idx_across_worldblock_boundary<xplus>(worldblock_dimension_type idx) { return idx - worldblock_x_factor*(worldblock_dimension-1); }
  template<> inline worldblock_dimension_type advance_idx_across_worldblock_boundary<yplus>(worldblock_dimension_type idx) { return idx - worldblock_y_factor*(worldblock_dimension-1); }
  template<> inline worldblock_dimension_type advance_idx_across_worldblock_boundary<zplus>(worldblock_dimension_type idx) { return idx - worldblock_z_factor*(worldblock_dimension-1); }

  template<cardinal_direction Dir> inline worldblock& worldblock::ensure_neighbor_realization(level_of_tile_realization_needed realineeded) {
    if (worldblock* neighbor = neighbors_[Dir]) {
      return neighbor->ensure_realization(realineeded);
    }
    else {
      realize_nonexistent_neighbor(Dir, realineeded);
      return *neighbors_[Dir];
    }
  }
}
template<cardinal_direction Dir> inline tile_location tile_location::get_neighbor(level_of_tile_realization_needed realineeded)const {
  using namespace the_decomposition_of_the_world_into_blocks_impl;
  wb_->ensure_realization(realineeded);
  const vector3<tile_coordinate> new_coords = cdir_info<Dir>::plus(v_);
  if (next_to_boundary<Dir>(v_)) {
    return tile_location(new_coords, advance_idx_across_worldblock_boundary<Dir>(idx_),
                                      &wb_->ensure_neighbor_realization<Dir>(realineeded));
  }
  else {
    return tile_location(new_coords, advance_idx_within_worldblock<Dir>(idx_), wb_);
  }
}

inline array<tile, num_cardinal_directions> tile_location::get_all_neighbor_tiles(level_of_tile_realization_needed realineeded)const {
  using namespace the_decomposition_of_the_world_into_blocks_impl;
  const worldblock_dimension_type local_x = get_primitive_int(v_.x) & (worldblock_dimension-1);
  const worldblock_dimension_type local_y = get_primitive_int(v_.y) & (worldblock_dimension-1);
  const worldblock_dimension_type local_z = get_primitive_int(v_.z) & (worldblock_dimension-1);
  const worldblock_dimension_type idx = local_x*worldblock_x_factor + local_y*worldblock_y_factor + local_z*worldblock_z_factor;

  wb_->ensure_realization(realineeded);
  array<tile, num_cardinal_directions> result = {{
    ((local_x == 0) ? wb_->ensure_neighbor_realization<xminus>(realineeded).tiles_[idx + (worldblock_x_factor*(worldblock_dimension-1))] : wb_->tiles_[idx - worldblock_x_factor]),
    ((local_y == 0) ? wb_->ensure_neighbor_realization<yminus>(realineeded).tiles_[idx + (worldblock_y_factor*(worldblock_dimension-1))] : wb_->tiles_[idx - worldblock_y_factor]),
    ((local_z == 0) ? wb_->ensure_neighbor_realization<zminus>(realineeded).tiles_[idx + (worldblock_z_factor*(worldblock_dimension-1))] : wb_->tiles_[idx - worldblock_z_factor]),
    ((local_x == worldblock_dimension-1) ? wb_->ensure_neighbor_realization<xplus>(realineeded).tiles_[idx - (worldblock_x_factor*(worldblock_dimension-1))] : wb_->tiles_[idx + worldblock_x_factor]),
    ((local_y == worldblock_dimension-1) ? wb_->ensure_neighbor_realization<yplus>(realineeded).tiles_[idx - (worldblock_y_factor*(worldblock_dimension-1))] : wb_->tiles_[idx + worldblock_y_factor]),
    ((local_z == worldblock_dimension-1) ? wb_->ensure_neighbor_realization<zplus>(realineeded).tiles_[idx - (worldblock_z_factor*(worldblock_dimension-1))] : wb_->tiles_[idx + worldblock_z_factor]),
  }};
  return result;
}


#endif

