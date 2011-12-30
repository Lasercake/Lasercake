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

void world_building_gun::operator()(tile_contents new_contents, vector3<tile_coordinate> locv) {
  assert(bounds.contains(locv));
  if (new_contents == ROCK) {
    w->insert_rock_bypassing_checks(w->make_tile_location(locv));
  }
  else if (new_contents == WATER) {
    w->insert_water_bypassing_checks(w->make_tile_location(locv));
  }
  else assert("YOU CAN ONLY PLACE ROCK AND WATER" && false);
}
