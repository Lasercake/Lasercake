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

#include "world.hpp"

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


template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
worldgen_function_t worldgen_from_tilespec(Functor const& xyz_to_tile_contents) {
  return worldgen_function_t(worldgen_from_tilespec_t<Functor>(xyz_to_tile_contents));
}

#endif
