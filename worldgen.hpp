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

#ifndef LASERCAKE_WORLDGEN_HPP__
#define LASERCAKE_WORLDGEN_HPP__

#include <limits>
#include <boost/functional/hash.hpp>

#include "world.hpp"

namespace std {
  template<> struct hash<pair<tile_coordinate, tile_coordinate> > {
    inline size_t operator()(pair<tile_coordinate, tile_coordinate> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.first);
      boost::hash_combine(seed, v.second);
      return seed;
    }
  };
}

// Don't construct worldgen_function_t:s yourself; use worldgen_from_tilespec().
template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
class worldgen_from_tilespec_t {
public:
  worldgen_from_tilespec_t(Functor const& xyz_to_tile_contents) : xyz_to_tile_contents_(xyz_to_tile_contents) {}
  void operator()(the_decomposition_of_the_world_into_blocks_impl::worldblock* wb, tile_bounding_box bounds)const {
    size_t i = 0;
    for (tile_coordinate x = bounds.min(X); x != bounds.min(X) + bounds.size(X); ++x) {
      for (tile_coordinate y = bounds.min(Y); y != bounds.min(Y) + bounds.size(Y); ++y) {
        for (tile_coordinate z = bounds.min(Z); z != bounds.min(Z) + bounds.size(Z); ++z, ++i) {
          const vector3<tile_coordinate> l(x, y, z);
          const tile_contents new_contents = xyz_to_tile_contents_(l);
          caller_correct_if(new_contents == ROCK || new_contents == AIR || new_contents == GROUPABLE_WATER || new_contents == RUBBLE,
                            "Trying to place a type of tile other than AIR, ROCK, GROUPABLE_WATER, and RUBBLE");
          wb->tiles_[i] = tile::make_tile_with_contents_and_interiorness(new_contents, true);
        }
      }
    }
  }
private:
  Functor xyz_to_tile_contents_;
};

class world_column_builder {
public:
  void specify_lowest(tile_contents contents);
  void specify(tile_coordinate min_z, tile_contents contents);
  // (TODO - worldblock_column_builder does not treat these correctly currently)
  void dont_specify_lowest();
  void dont_specify(tile_coordinate min_z);

  world_column_builder() : len(0) {}
  struct spec_from {
    tile_coordinate min;
    tile_contents contents;//or inited tile..
    bool operator<(spec_from other) { return min < other.min; }
  };
  size_t len;
  std::array<spec_from, 12> arr;
};
inline void world_column_builder::specify_lowest(tile_contents contents) {
  arr[len++] = spec_from{0, contents};
}
inline void world_column_builder::specify(tile_coordinate min_z, tile_contents contents) {
  arr[len++] = spec_from{min_z, contents};
}
inline void world_column_builder::dont_specify_lowest() {
  arr[len++] = spec_from{0, UNSPECIFIED_TILE_CONTENTS};
}
inline void world_column_builder::dont_specify(tile_coordinate min_z) {
  arr[len++] = spec_from{min_z, UNSPECIFIED_TILE_CONTENTS};
}

// Functor(world_column_builder& b, coord x, coord y, coord min_z_demanded, coord max_z_demanded)
namespace the_decomposition_of_the_world_into_blocks_impl {
template<typename Functor>
class worldblock_column_builder {
public:
  worldblock_column_builder(Functor const& column_spec) : column_spec_(column_spec) {}
  void operator()(worldblock* wb, tile_bounding_box bounds) {
    // for each x and y column
    // do we have all needed data for that column already?
    // otherwise ask for it, remember it.
    // use the data for the column to fill in the column.
    // (that's all for now.)
    // heyy.. first, just ask every wb and fill and see how it speeds ^_^
    const tile_coordinate min_z = bounds.min(Z);
    const tile_coordinate max_z = bounds.max(Z);
    std::array<world_column_builder, worldblock_dimension*worldblock_dimension>& worldblock_column =
      already_computed_columns_[make_pair(bounds.min(X), bounds.min(Y))];
    // Hack - if there is any data in the worldblock_column,
    // assume it has all the information we need. (TODO.)
    if(worldblock_column[0].len) {
      size_t column_base_idx = 0;
      for (tile_coordinate x = bounds.min(X); x != bounds.min(X) + bounds.size(X); ++x) {
        for (tile_coordinate y = bounds.min(Y); y != bounds.min(Y) + bounds.size(Y);
             ++y, column_base_idx += worldblock_y_factor) {
          world_column_builder& b = worldblock_column[column_base_idx / worldblock_y_factor];
          size_t arr_i = 0;
          while(b.arr[arr_i+1].min <= min_z) {++arr_i;}
          for (tile_coordinate z = bounds.min(Z); z != bounds.min(Z) + bounds.size(Z); ++z) {
            if(b.arr[arr_i+1].min == z) {++arr_i;}
            wb->tile_data_uint8_array[column_base_idx + get_primitive_int(z - wb->global_position_.z)]
              = (b.arr[arr_i].contents | tile::interior_bit_mask);
          }
        }
      }
    }
    else {
      size_t column_base_idx = 0;
      for (tile_coordinate x = bounds.min(X); x != bounds.min(X) + bounds.size(X); ++x) {
        for (tile_coordinate y = bounds.min(Y); y != bounds.min(Y) + bounds.size(Y);
             ++y, column_base_idx += worldblock_y_factor) {
          world_column_builder& b = worldblock_column[column_base_idx / worldblock_y_factor];
          column_spec_(b, x, y, min_z, max_z);
          // This is a bit of a hack:
          b.specify(std::numeric_limits<int32_t>::max(), UNSPECIFIED_TILE_CONTENTS);
          // no binary searches at the moment.
          //
          // These two impls seem to be similar fine speeds, but I haven't
          // looked at this closely -Isaac
          #if 1
            size_t arr_i = 0;
            while(b.arr[arr_i+1].min <= min_z) {++arr_i;}
            //worldblock_dimension_type z_idx_begin = 0;
            for (tile_coordinate z = bounds.min(Z); z != bounds.min(Z) + bounds.size(Z); ++z) {
              if(b.arr[arr_i+1].min == z) {++arr_i;}
              wb->tile_data_uint8_array[column_base_idx + get_primitive_int(z - wb->global_position_.z)]
                = (b.arr[arr_i].contents | tile::interior_bit_mask);
            }
          #else
            for (size_t i = 0; i != b.len - 1 /*due to the hack of +1 end thing*/; ++i) {
              world_column_builder::spec_from spec = b.arr[i];
              world_column_builder::spec_from next_spec = b.arr[i+1];
              if (spec.min <= max_z && next_spec.min > min_z) {
                worldblock_dimension_type z_idx_begin =
                  std::max(0, get_primitive_int(spec.min - wb->global_position_.z));
                worldblock_dimension_type z_idx_end =
                  std::min(worldblock_dimension, get_primitive_int(next_spec.min - wb->global_position_.z));
                memset(&wb->tiles_[column_base_idx + z_idx_begin],
                       spec.contents | tile::interior_bit_mask,
                       z_idx_end - z_idx_begin);
              }
            }
          #endif
        }
      }
    }
  }
private:
  Functor column_spec_;
  std::unordered_map<std::pair<tile_coordinate, tile_coordinate>,
                     std::array<world_column_builder, worldblock_dimension*worldblock_dimension>
  > already_computed_columns_;
};
}
template<typename Functor>
worldgen_function_t worldgen_from_column_spec(Functor const& column_spec) {
  return worldgen_function_t(the_decomposition_of_the_world_into_blocks_impl::worldblock_column_builder<Functor>(column_spec));
}


template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
worldgen_function_t worldgen_from_tilespec(Functor const& xyz_to_tile_contents) {
  return worldgen_function_t(worldgen_from_tilespec_t<Functor>(xyz_to_tile_contents));
}

#endif
