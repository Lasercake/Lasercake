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

#include <tuple>
#include <unordered_map>
#include <boost/random/ranlux.hpp>

#include "world.hpp"
#include "worldgen.hpp"
#include "specific_worlds.hpp"


typedef tile_coordinate coord;
typedef vector3<coord> coords;

namespace /* anonymous */ {

const coord wcc = world_center_tile_coord;
template<typename Functor>
struct with_state {
  shared_ptr<Functor> functor_;
  with_state() : functor_(new Functor()) {}
  with_state(shared_ptr<Functor> ptr) : functor_(ptr) {}
  tile_contents operator()(coords l)const {
    return (*functor_)(l);
  }
  void operator()(world_column_builder& b, coord x, coord y, coord min_z_demanded, coord max_z_demanded)const {
    (*functor_)(b, x, y, min_z_demanded, max_z_demanded);
  }
};


template<size_t Index, class...Types>
struct tuple_hash_impl {
  static BOOST_FORCEINLINE void impl(size_t& v, std::tuple<Types...> const& tup) {
    tuple_hash_impl<Index - 1, Types...>::impl(v, tup);
    boost::hash_combine(v, std::get<Index>(tup));
  }
};
template<class...Types>
struct tuple_hash_impl<0, Types...> {
  static BOOST_FORCEINLINE void impl(size_t& v, std::tuple<Types...> const& tup) {
    boost::hash_combine(v, std::get<0>(tup));
  }
};

// A bit of a hack & potential inefficiency by hash_combining this
// zero hash unnecessarily...
template<typename T>
struct unhashed {
  template<typename...Args>
  unhashed(Args&&... args) : t(std::forward<Args>(args)...) {}
  T t;
  operator T()const { return t; }
  T operator->()const { return t; }
  bool operator==(unhashed const&)const { return true; }
  bool operator!=(unhashed const&)const { return false; }
  friend inline size_t hash_value(unhashed const&) { return 0; }
};

} /*end anonymous namespace*/

namespace std {
  template<typename...Types>
  struct hash<std::tuple<Types...>> {
    size_t operator()(std::tuple<Types...> const& tup)const {
      size_t result = 0;
      tuple_hash_impl<
        std::tuple_size<std::tuple<Types...>>::value - 1,
        Types...
      >::impl(result, tup);
      return result;
    }
  };
}

namespace std {
  template<typename T>
  struct hash<unhashed<T>> {
    size_t operator()(unhashed<T> const&)const {
      return 0;
    }
  };
}

namespace /*anonymous*/ {

template<typename Functor, typename Signature>
struct memoized;
template<typename Functor, typename Result, typename...Arguments>
struct memoized<Functor, Result(Arguments...)> {
  Functor functor_;
  typedef std::tuple<Arguments...> arguments_tuple_type;
  typedef Result original_result_type;
  typedef original_result_type const& result_type;
  typedef std::unordered_map<arguments_tuple_type, original_result_type> memo_type;
  typedef typename memo_type::value_type memo_pair_type;
  mutable memo_type memo_;

  template<typename...Arguments2>
  result_type operator()(Arguments2&&... args)const {
    const arguments_tuple_type args_tuple(args...);
    typename memo_type::const_iterator i = memo_.find(args_tuple);
    if(i != memo_.end()) {
      return i->second;
    }
    else {
      return memo_.insert(
        memo_pair_type(
          args_tuple,
          functor_(std::forward<Arguments>(args)...)
        )
      ).first->second;
    }
  }

  memoized() : functor_() {}
  memoized(Functor&& f) : functor_(std::forward<Functor>(f)) {}
};

// This is a mechanism to make pseudo-random worlds whose contents
// don't depend on the order you explore them in.

// An RNG that's fast enough to init and gives good enough results
// for a good worldgen.
typedef boost::random::ranlux3 memo_rng;

// This doesn't get all the bits of the arguments into the rng seed init,
// but it seems to work fine.  Also it may be redundant hashing work with
// memoized that the compiler may or may not figure out...
//
// TODO this is also *wrong* because the std does not guarantee a particular
// hash implementation, nor that an implementation return the same result
// on every run of the program (the latter might be false for security reasons
// [a mild defense against computational-complexity attacks]).
//
// Using boost::hash and a fixed Boost version helps some, though Boost
// can also call types' custom hash functions... so still TODO think
// about this.  (Linux packagers wouldn't even want to use the fixed Boost
// version.  Probably we'll just use some code duplication at some point.)
template<typename...HashableArguments>
inline memo_rng make_rng(HashableArguments&&... args) {
  typedef std::tuple<HashableArguments...> arguments_tuple_type;
  memo_rng result(boost::hash<arguments_tuple_type>()(arguments_tuple_type(args...)));
  return result;
}


class simple_hills {
  static const int max_simple_hill_width = 20;
public:
  void operator()(world_column_builder& b, coord x, coord y, coord, coord)const {
    b.specify_lowest(ROCK);
    b.specify(height_memo_(this, x, y), AIR);
  }

  struct get_height {
    coord operator()(unhashed<simple_hills const*> that, coord x, coord y)const {
      coord height = world_center_tile_coord - 100;
      for (coord x2 = x - max_simple_hill_width; x2 <= x + max_simple_hill_width; ++x2) {
        for (coord y2 = y - max_simple_hill_width; y2 <= y + max_simple_hill_width; ++y2) {
          height += std::max(0, that->hills_memo_(x2, y2) - get_primitive<int>(i64sqrt((x2-x)*(x2-x) + (y2-y)*(y2-y))));
        }
      }
      return height;
    }
  };
  struct get_hill {
    int operator()(coord x, coord y)const {
      memo_rng rng_here = make_rng(x, y);
      const boost::random::uniform_int_distribution<int> random_in_256(0,255);
      const boost::random::uniform_int_distribution<int> random_hill_height(1,max_simple_hill_width);
      int hill = (random_in_256(rng_here) != 0 ? 0 : random_hill_height(rng_here));
      return hill;
    }
  };
private:
  memoized<get_hill, int (coord, coord)> hills_memo_;
  memoized<get_height, coord (unhashed<simple_hills const*>, coord, coord)> height_memo_;
};

class spiky1 {
public:
  tile_contents operator()(coords l) {
    const coord height = get_height(make_pair(l.x, l.y));
    return
      (l.z < height) ? ROCK : AIR;
  }
private:
  unordered_map<std::pair<coord, coord>, coord> height_map_;
  // RNG default-initialized for now
  // (so, deterministic except for worldblock realization order)
  large_fast_noncrypto_rng rng_;

  static const int a_spike_height = 20;

  coord get_height(pair<coord, coord> loc) {
    const auto iter = height_map_.find(loc);
    if (iter == height_map_.end()) {
      const int which = boost::random::uniform_int_distribution<int>(0,20)(rng_);
      int spike_max = a_spike_height;
      if (which < 5) spike_max *= 3;
      if (which == 0) spike_max *= 5;
      const boost::random::uniform_int_distribution<int> random_spike_height(0,spike_max);
      coord height = wcc + random_spike_height(rng_);
      height_map_.insert(make_pair(loc, height));
      return height;
    }
    else {
      return iter->second;
    }
  }
};

class spiky2 {
public:
  void operator()(world_column_builder& b, coord x, coord y, coord, coord) {
    b.specify_lowest(ROCK);
    b.specify(get_height(make_pair(x, y)), AIR);
  }
private:
  unordered_map<std::pair<coord, coord>, coord> height_map_;
  // RNG default-initialized for now
  // (so, deterministic except for worldblock realization order)
  large_fast_noncrypto_rng rng_;

  static const int a_spike_height = 20;

  coord get_height(pair<coord, coord> loc) {
    const auto iter = height_map_.find(loc);
    if (iter == height_map_.end()) {
      const int which = boost::random::uniform_int_distribution<int>(0,20)(rng_);
      int spike_max = a_spike_height;
      if (which < 5) spike_max *= 3;
      if (which == 0) spike_max *= 5;
      const boost::random::uniform_int_distribution<int> random_spike_height(0,spike_max);
      coord height = wcc + random_spike_height(rng_);
      height_map_.insert(make_pair(loc, height));
      return height;
    }
    else {
      return iter->second;
    }
  }
};


class spiky3 {
  static const int a_spike_height = 20;
public:
  void operator()(world_column_builder& b, coord x, coord y, coord, coord)const {
    b.specify_lowest(ROCK);
    b.specify(height_memo_(x, y), AIR);
  }

  struct get_height {
    coord operator()(coord x, coord y)const {
      // TODO include a constant-for-this-instance-of-the-world random-seed too.
      memo_rng rng_here = make_rng(x, y);

      const int which = boost::random::uniform_int_distribution<int>(0,20)(rng_here);
      int spike_max = a_spike_height;
      if (which < 5) spike_max *= 3;
      if (which == 0) spike_max *= 5;
      const boost::random::uniform_int_distribution<int> random_spike_height(0,spike_max);
      coord height = wcc + random_spike_height(rng_here);
      return height;
    }
  };
private:
  memoized<get_height, coord (coord, coord)> height_memo_;
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
  if (scenario == "flat2") {
    return worldgen_from_column_spec([](world_column_builder& b, coord, coord, coord, coord) {
      b.specify_lowest(ROCK);
      b.specify(wcc, AIR);
    });
  }
  if (scenario == "plane") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z == wcc) ? ROCK : AIR;
    });
  }
  if (scenario == "ceiling") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z > wcc + 100) ? ROCK : AIR;
    });
  }
  if (scenario == "low_ceiling") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z > wcc) ? ROCK : AIR;
    });
  }
  if (scenario == "simple_hills") {
    return worldgen_from_column_spec(simple_hills());
  }
  if (scenario == "spiky1") {
    return worldgen_from_tilespec(with_state<spiky1>());
  }
  if (scenario == "spiky2") {
    return worldgen_from_column_spec(with_state<spiky2>());
  }
  if (scenario == "spiky" || scenario == "spiky3") {
    return worldgen_from_column_spec(spiky3());
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
