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
#include "specific_worlds.hpp"


typedef tile_coordinate coord;
typedef vector3<coord> coords;

namespace std {
  template<> struct hash<pair<coord, coord> > {
    inline size_t operator()(pair<coord, coord> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.first);
      boost::hash_combine(seed, v.second);
      return seed;
    }
  };
}
namespace /* anonymous */ {

const coord wcc = world_center_tile_coord;

const int max_simple_hill_width = 20;

int get_hill(
          unordered_map<std::pair<coord, coord>, int>& hills_map,
          pair<coord, coord> loc) {
  const auto iter = hills_map.find(loc);
  if (iter == hills_map.end()) {
    int hill = (rand()&255) ? 0 : (1 + (rand()%max_simple_hill_width));
    hills_map.insert(make_pair(loc, hill));
    return hill;
  }
  else return iter->second;
}
coord get_height(
          unordered_map<std::pair<coord, coord>, coord>& height_map,
          unordered_map<std::pair<coord, coord>, int>& hills_map,
          pair<coord, coord> loc) {
  const auto iter = height_map.find(loc);
  if (iter == height_map.end()) {
    coord height = world_center_tile_coord - 100;
    const coord x = loc.first;
    const coord y = loc.second;
    for (coord x2 = x - max_simple_hill_width; x2 <= x + max_simple_hill_width; ++x2) {
      for (coord y2 = y - max_simple_hill_width; y2 <= y + max_simple_hill_width; ++y2) {
        height += std::max(0, get_hill(hills_map, make_pair(x2, y2)) - get_un_bounds_checked<int>(i64sqrt((x2-x)*(x2-x) + (y2-y)*(y2-y))));
      }
    }
    height_map.insert(make_pair(loc, height));
    return height;
  }
  else return iter->second;
}



template<typename Functor>
struct with_state {
  shared_ptr<Functor> functor_;
  with_state() : functor_(new Functor()) {}
  with_state(shared_ptr<Functor> ptr) : functor_(ptr) {}
  tile_contents operator()(coords l)const {
    return (*functor_)(l);
  }
};


bool in_old_box(coords l) {
  return  l.x >= wcc-1 && l.x < wcc+21 &&
          l.y >= wcc-1 && l.y < wcc+21 &&
          l.z >= wcc-1 && l.z < wcc+21;
}
bool is_old_box(coords l) {
  return  l.x == wcc-1 || l.x == wcc+20 ||
          l.y == wcc-1 || l.y == wcc+20 ||
          l.z == wcc-1 /*|| l.z == wcc+20*/;
}

bool in_old_water_tower(coords l) {
  return  l.x >= wcc+4 && l.x <= wcc+6 &&
          l.y >= wcc+4 && l.y <= wcc+6 &&
          l.z >= wcc+1;
}

bool around_old_water_tower(coords l) {
  return  l.x >= wcc+3 && l.x <= wcc+7 &&
          l.y >= wcc+3 && l.y <= wcc+7 &&
          l.z >= wcc+1 &&
          !in_old_water_tower(l);
}
bool dike_surrounding_old_water_tower(coords l) {
  return  l.z < wcc+3 && (l.x == wcc+1 || l.x == wcc+9 || l.y == wcc+1 || wcc+l.y/*??*/ == 9);
}

template <tile_contents twist>
struct twisty {
  tile_contents operator()(coords l)const {
    return
      (!in_old_box(l)) ? AIR :
      (is_old_box(l)) ? ROCK :
      (l.x == wcc+0) ? GROUPABLE_WATER :
      (l.x == wcc+1 && l.z > wcc+0) ? ROCK :
      (l.x == wcc+2 && (l.z % 4) == 1) ? ROCK :
      (l.x == wcc+3 && (l.z % 2) == 1) ? ROCK :
      (l.x == wcc+4 && (l.z % 4) == 3) ? ROCK :
      (l.x == wcc+5) ? twist :
      AIR;
  }
};

} /* end anonymous namespace */

worldgen_function_t make_world_building_func(std::string scenario) {
  if(scenario == "vacuum") {
    return worldgen_from_tilespec([](coords) {
      return AIR;
    });
  }
  if (scenario == "flat") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z < wcc) ? ROCK : AIR;
    });
  }
  if (scenario == "plane") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z == wcc) ? ROCK : AIR;
    });
  }
  if (scenario == "simple_hills") {
    struct hills {
      unordered_map<std::pair<coord, coord>, int> hills_map;
      unordered_map<std::pair<coord, coord>, coord> height_map;
      tile_contents operator()(coords l) {
        const coord height = get_height(height_map, hills_map, make_pair(l.x, l.y));
        return
          (l.z < height) ? ROCK : AIR;
      }
    };
    return worldgen_from_tilespec(with_state<hills>());
  }
  if (scenario == "pressure_tunnel" || scenario == "pressure_tunnel_ground") {
    const bool has_ground = (scenario == "pressure_tunnel_ground");
    return worldgen_from_tilespec([has_ground](coords l)->tile_contents {
      const coord tower_lower_coord = wcc;
      const coord tower_upper_coord = wcc+10;
      const coord tower_height = 200;
      return
        ( l.x < tower_lower_coord &&
          l.y >= tower_lower_coord && l.y <= tower_lower_coord &&
          l.z >= wcc && l.z <= wcc
        ) ? AIR :
        
        ( l.x < tower_lower_coord &&
          l.y >= tower_lower_coord-1 && l.y <= tower_lower_coord+1 &&
          l.z >= wcc-1 && l.z <= wcc+1
        ) ? ROCK :
        
        ( l.x >= tower_lower_coord && l.x < tower_upper_coord &&
          l.y >= tower_lower_coord && l.y < tower_upper_coord &&
          l.z >= wcc && l.z < wcc + tower_height
        ) ? GROUPABLE_WATER :
        
        ( l.x >= tower_lower_coord-1 && l.x < tower_upper_coord+1 &&
          l.y >= tower_lower_coord-1 && l.y < tower_upper_coord+1 &&
          l.z >= wcc-1 && l.z < wcc + tower_height+1
        ) ? ROCK :

        ( l.z <= wcc-1 && has_ground
        ) ? ROCK :
        
        AIR;
    });
  }
  if (scenario == "stepped_pools") {
    return worldgen_from_tilespec([](coords l)->tile_contents {
      typedef lasercake_int<int64_t>::type number;
      const number block_width = 30;
      const number border_width = 3;
      const number block_height_shift = 30;
      const number pool_steepness = 1;
      const number pool_top_layers_missing = 1;

      // "lmwc" = "l minus world center": the position relative to the "center" of the world.
      const vector3<number> lmwc = vector3<number>(l) - vector3<number>(wcc,wcc,wcc);
      const number base_height = -20LL + (number(l.x / block_width) - number(wcc / block_width)) * block_height_shift;
      const number dist_from_block_edge = std::min(
        std::min(number(l.x % block_width), block_width - 1 - (l.x % block_width)),
        std::min(number(l.y % block_width), block_width - 1 - (l.y % block_width)));
      
      if (dist_from_block_edge <= border_width) {
        return (lmwc.z <= base_height) ? ROCK : AIR;
      }
      else {
        if (lmwc.z <= base_height) {
          const coord pool_depth_here = (dist_from_block_edge - border_width) * pool_steepness;
          if (lmwc.z > base_height - pool_depth_here) {
            return (lmwc.z <= base_height - pool_top_layers_missing) ? GROUPABLE_WATER : AIR;
          }
          else {
            return ROCK;
          }
        }
        else {
          return AIR;
        }
      }
    });
  }


  if(scenario == "default") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        AIR;
    });
  }
  if(scenario == "tower") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        AIR;
    });
  }
  if(scenario == "tower2") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        (around_old_water_tower(l)) ? ROCK :
        AIR;
    });
  }
  if(scenario == "tower3") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        (around_old_water_tower(l)) ? ROCK :
        (dike_surrounding_old_water_tower(l)) ? ROCK :
        AIR;
    });
  }
  
  if(scenario == "shallow") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (l.z == wcc+0 && l.x >= wcc+5) ? ROCK :
        (l.z == wcc+1 && l.x >= wcc+10) ? ROCK :
        (l.z == wcc+2 && l.x >= wcc+15) ? ROCK :
        (l.x == wcc+19) ? GROUPABLE_WATER :
        AIR;
    });
  }
  if(scenario == "steep") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (l.z < 20 - l.x) ? ROCK :
        (l.z >= wcc+15 && (wcc + 20 - l.x) >= 15) ? GROUPABLE_WATER :
        AIR;
    });
  }
  if(scenario == "tank") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (l.z < wcc+8 && l.x > wcc+10) ? ROCK :
        (l.x >= wcc+12 && l.y <= wcc+13 && l.y >= wcc+7) ? GROUPABLE_WATER :
        (l.y != wcc+10 && l.x > wcc+10) ? ROCK :
        (l.z > wcc+8 && l.x > wcc+10 && l.x < wcc+18) ? ROCK :
        (l.x > wcc+10) ? GROUPABLE_WATER :
        AIR;
    });
  }
  if(scenario == "tank2") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (l.z < wcc+8 && l.x > wcc+10) ? ROCK :
        (l.x >= wcc+12 && l.y <= wcc+13 && l.y >= wcc+7) ? GROUPABLE_WATER :
        (l.y != wcc+9 && l.y != wcc+10 && l.x > wcc+10) ? ROCK :
        (l.z > wcc+9 && l.x > wcc+10 && l.x < wcc+18) ? ROCK :
        (l.x > wcc+10) ? GROUPABLE_WATER :
        AIR;
    });
  }
#if 0
      /*GCC 4.6.2 segfaulted trying to compile this code. TODO: bugreport it.*/
      /*Might be related: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=52014 */
  auto a_twisty = [](tile_contents twist) {
    return [twist](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (l.x == wcc+0) ? GROUPABLE_WATER :
        (l.x == wcc+1 && l.z > wcc+0) ? ROCK :
        (l.x == wcc+2 && (l.z % 4) == 1) ? ROCK :
        (l.x == wcc+3 && (l.z % 2) == 1) ? ROCK :
        (l.x == wcc+4 && (l.z % 4) == 3) ? ROCK :
        (l.x == wcc+5) ? twist :
        AIR;
    };
  };
  if(scenario == "twisty") {
    return worldgen_from_tilespec(a_twisty(ROCK));
  }
  if(scenario == "twistyrubble") {
    return worldgen_from_tilespec(a_twisty(RUBBLE));
  }
#endif
  if(scenario == "twisty") {
    return worldgen_from_tilespec(twisty<ROCK>());
  }
  if(scenario == "twistyrubble") {
    return worldgen_from_tilespec(twisty<RUBBLE>());
  }

  // If it wasn't named, we have no function.
  // The caller should probably check for this unfortunateness.
  return worldgen_function_t();
}
