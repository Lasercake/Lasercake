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
#include "specific_worlds.hpp"


namespace std {
  template<> struct hash<pair<tile_coordinate, tile_coordinate>> {
    inline size_t operator()(pair<tile_coordinate, tile_coordinate> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.first);
      boost::hash_combine(seed, v.second);
      return seed;
    }
  };
}
namespace /* anonymous */ {

const int max_simple_hill_width = 20;

int get_hill(unordered_map<std::pair<tile_coordinate, tile_coordinate>, int> &hills_map, pair<tile_coordinate, tile_coordinate> loc) {
  auto iter = hills_map.find(loc);
  if (iter == hills_map.end()) {
    int hill = (rand()&255) ? 0 : (1 + (rand()%max_simple_hill_width));
    hills_map.insert(make_pair(loc, hill));
    return hill;
  }
  else return iter->second;
}
tile_coordinate get_height(unordered_map<std::pair<tile_coordinate, tile_coordinate>, tile_coordinate> &height_map, unordered_map<std::pair<tile_coordinate, tile_coordinate>, int> &hills_map, pair<tile_coordinate, tile_coordinate> loc) {
  auto iter = height_map.find(loc);
  if (iter == height_map.end()) {
    tile_coordinate height = world_center_coord - 100;
    const tile_coordinate x = loc.first;
    const tile_coordinate y = loc.second;
    for (tile_coordinate x2 = x - max_simple_hill_width; x2 <= x + max_simple_hill_width; ++x2) {
      for (tile_coordinate y2 = y - max_simple_hill_width; y2 <= y + max_simple_hill_width; ++y2) {
        height += std::max(0, get_hill(hills_map, make_pair(x2, y2)) - (int)i64sqrt((x2-x)*(x2-x) + (y2-y)*(y2-y)));
      }
    }
    height_map.insert(make_pair(loc, height));
    return height;
  }
  else return iter->second;
}

struct world_building_func {
  world_building_func(std::string scenario):scenario(scenario){}
  std::string scenario;
  unordered_map<std::pair<tile_coordinate, tile_coordinate>, int> hills_map;
  unordered_map<std::pair<tile_coordinate, tile_coordinate>, tile_coordinate> height_map;

  void operator()(world_building_gun make, tile_bounding_box bounds) {
    const tile_coordinate wc = world_center_coord;
    if (scenario == "simple_hills") {
      for (tile_coordinate x = bounds.min.x; x < bounds.min.x + bounds.size.x; ++x) {
        for (tile_coordinate y = bounds.min.y; y < bounds.min.y + bounds.size.y; ++y) {
          tile_coordinate height = get_height(height_map, hills_map, make_pair(x, y));
          for (tile_coordinate z = bounds.min.z; z < std::min(height, bounds.min.z + bounds.size.z); ++z) {
            vector3<tile_coordinate> l(x,y,z);
            make(ROCK, l);
          }
        }
      }
      return;
    }
    if (scenario.substr(0,15) == "pressure_tunnel") {
      for(vector3<tile_coordinate> l : bounds) {
        const tile_coordinate tower_lower_coord = wc;
        const tile_coordinate tower_upper_coord = wc+10;
        const tile_coordinate tower_height = 200;
        if (l.x < tower_lower_coord && l.y >= tower_lower_coord && l.y <= tower_lower_coord && l.z >= wc && l.z <= wc) {}
        else if (l.x < tower_lower_coord && l.y >= tower_lower_coord-1 && l.y <= tower_lower_coord+1 && l.z >= wc-1 && l.z <= wc+1)
          make(ROCK, l);
        else if (l.x >= tower_lower_coord && l.x < tower_upper_coord && l.y >= tower_lower_coord && l.y < tower_upper_coord && l.z >= wc && l.z < wc + tower_height)
          make(GROUPABLE_WATER, l);
        else if (l.x >= tower_lower_coord-1 && l.x < tower_upper_coord+1 && l.y >= tower_lower_coord-1 && l.y < tower_upper_coord+1 && l.z >= wc-1 && l.z < wc + tower_height+1)
          make(ROCK, l);
      }
      return;
    }
    for (tile_coordinate x = std::max(world_center_coord-1, bounds.min.x); x < std::min(world_center_coord+21, bounds.min.x + bounds.size.x); ++x) {
      for (tile_coordinate y = std::max(world_center_coord-1, bounds.min.y); y < std::min(world_center_coord+21, bounds.min.y + bounds.size.y); ++y) {
        for (tile_coordinate z = std::max(world_center_coord-1, bounds.min.z); z < std::min(world_center_coord+21, bounds.min.z + bounds.size.z); ++z) {
          vector3<tile_coordinate> l(x,y,z);
          if (x == world_center_coord-1 || x == world_center_coord+20 || y == world_center_coord-1 || y == world_center_coord+20 || z == world_center_coord-1 /*|| z == world_center_coord+20*/) {
            make(ROCK, l);
          }
          else {
            if ((scenario.substr(0,5) == "tower") &&
                  x >= wc+4 && x <= wc+6 &&
                  y >= wc+4 && y <= wc+6 &&
                  z >= wc+1) make(GROUPABLE_WATER, l);
            else if ((scenario == "tower2" || scenario == "tower3") &&
                  x >= wc+3 && x <= wc+7 &&
                  y >= wc+3 && y <= wc+7 &&
                  z >= wc+1) make(ROCK, l);
            else if (scenario == "tower3" && z < wc+3 && (
                  x == wc+1 || x == wc+9 || y == wc+1 || wc+y == 9)) make(ROCK, l);

            if (scenario == "shallow") {
                   if (z == wc+0 && x >= wc+5) make(ROCK, l);
              else if (z == wc+1 && x >= wc+10) make(ROCK, l);
              else if (z == wc+2 && x >= wc+15) make(ROCK, l);
              else if (x == wc+19) make(GROUPABLE_WATER, l);
            }
            if (scenario == "steep") {
              if (z < 20 - x) make(ROCK, l);
              else if (z >= wc+15 && (wc + 20 - x) >= 15) make(GROUPABLE_WATER, l);
            }
            if (scenario == "tank") {
              if (z < wc+8 && x > wc+10) make(ROCK, l);
              else if (x >= wc+12 && y <= wc+13 && y >= wc+7) make(GROUPABLE_WATER, l);
              else if (y != wc+10 && x > wc+10) make(ROCK, l);
              else if (z > wc+8 && x > wc+10 && x < wc+18) make(ROCK, l);
              else if (x > wc+10) make(GROUPABLE_WATER, l);
            }
            if (scenario == "tank2") {
              if (z < wc+8 && x > wc+10) make(ROCK, l);
              else if (x >= wc+12 && y <= wc+13 && y >= wc+7) make(GROUPABLE_WATER, l);
              else if (y != wc+9 && y != wc+10 && x > wc+10) make(ROCK, l);
              else if (z > wc+9 && x > wc+10 && x < wc+18) make(ROCK, l);
              else if (x > wc+10) make(GROUPABLE_WATER, l);
            }
            if (scenario.substr(0,6) == "twisty") {
              if (x == wc+0) make(GROUPABLE_WATER, l);
              else if (x == wc+1 && z > wc+0) make(ROCK, l);
              else if (x == wc+5) make(scenario == "twistyrubble" ? RUBBLE : ROCK, l);
              else if (x == wc+2 && (z % 4) == 1) make(ROCK, l);
              else if (x == wc+3 && (z % 2) == 1) make(ROCK, l);
              else if (x == wc+4 && (z % 4) == 3) make(ROCK, l);
            }
          }
        }
      }
    }
  }
};

} /* end anonymous namespace */

worldgen_function_t make_world_building_func(std::string scenario) {
  return worldgen_function_t(world_building_func(scenario));
}
