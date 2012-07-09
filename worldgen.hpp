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

class world_building_gun {
public:
  world_building_gun(world* w, tile_bounding_box bounds,
                     the_decomposition_of_the_world_into_blocks_impl::worldblock* wb):w_(w),wb_(wb),bounds_(bounds){}
private:
  world* w_;
  the_decomposition_of_the_world_into_blocks_impl::worldblock* wb_;
  tile_bounding_box bounds_;
public:
  template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
  class worldgen_from_tilespec_t {
  public:
    worldgen_from_tilespec_t(Functor const& xyz_to_tile_contents) : xyz_to_tile_contents_(xyz_to_tile_contents) {}
    void operator()(world_building_gun make, tile_bounding_box bounds)const {
      size_t i = 0;
      for (tile_coordinate x = bounds.min.x; x < bounds.min.x + bounds.size.x; ++x) {
        for (tile_coordinate y = bounds.min.y; y < bounds.min.y + bounds.size.y; ++y) {
          for (tile_coordinate z = bounds.min.z; z < bounds.min.z + bounds.size.z; ++z, ++i) {
            const vector3<tile_coordinate> l(x, y, z);
            const tile_contents new_contents = xyz_to_tile_contents_(l);
            caller_correct_if(new_contents == ROCK || new_contents == AIR || new_contents == GROUPABLE_WATER || new_contents == RUBBLE,
                              "Trying to place a type of tile other than AIR, ROCK, GROUPABLE_WATER, and RUBBLE");
            tile new_tile; new_tile.set_contents(new_contents); new_tile.set_interiorness(true);
            make.wb_->tiles_[i] = new_tile;
          }
        }
      }
    }
  private:
    Functor xyz_to_tile_contents_;
  };
};

template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
worldgen_function_t worldgen_from_tilespec(Functor const& xyz_to_tile_contents) {
  return worldgen_function_t(world_building_gun::worldgen_from_tilespec_t<Functor>(xyz_to_tile_contents));
}

#endif
