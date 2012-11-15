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
  const size_t hash = boost::hash<arguments_tuple_type>()(arguments_tuple_type(args...));
  // ranlux3 uses a 32-bit seed.
  // (Don't ">> 32" to avoid silly compiler warnings on 32-bit.)
  const uint32_t seed = ((sizeof(hash) == 4) ? hash : (hash ^ (hash >> 16 >> 16)));
  memo_rng result(seed);
  return result;
}


/*

What is simple_hills?

Imagine this: Take an infinite, flat world and randomly stack conical "hills" on top of it. If hills overlap, you simply add up, in any individual column, the height increases called for by hills overlapping that column.

For each column, there's a certain chance if has a radius-1 hill centered on it, a chance it has a radius-2 hill, and so forth. The chances are the same for all columns, so we just have one function/sequence P_n : {1, 2, 3, ... } -> [0,1] that indicates the probability of each size. (We won't worry too much about whether those probabilities are independent.)

If some tail of P_n is all zeroes, then this is relatively simple: for each column, we can check for hills centered on all columns up to max-hill-radius away and add up their effects. But if P_n goes unto infinity, we need more cleverness. And it would be cool not to have an arbitrary maximum hill size.

Considering infinity:

In order for the map to be well-defined, the columns have to have a finite average height, i.e. there has to be a finite volume per column - equivalently, each column has to have a finite volume-contribution from the possible hills centered on it. Each hill of radius n contributes some constant factor of n^3 volume, so the required condition is that the series

(P_n * n^3)

converges. P_n = n^{-5} makes this converge nicely; we'll use that.

To look up what hills have already been computed, we use an infinite sequence of nested axis-aligned, power-of-2-aligned grids. If I want to know whether any hills less than rad-16 affect a column, I need only look for hill-centers in four 16x16 boxes. For bigger hills, I have to look in four 32x32 boxes, four 64x64 boxes, and so on.

How do I look in infinite boxes?

Let's say nothing is determined yet and I want to check the height of some column A. There's some function P'_n : N -> [0,1] that's the probability of there being any hill centers in a box of size 2^n. The probability is approximately
\int_{1/2}^1 x^5 dx * P_(2^n) * (2^n)^3
= (1/6 - 1/384) *  (2^n)^{-5} * (2^n)^3
~= 1/6 * (2^n)^{-2}
= 1/6 * 2^{-2n}
= 1/6 * (1/4)^n

With those factors there's less than a 1/8 chance that any hill will come near to touching this tile, but the constant factor doesn't really matter so we can adjust it until there's a reasonable amount of hills.

~= (1/4)^n

What's the probability that *any* hill will appear? That's a nice geometric series so it's easy to compute its sum (3/4), but that isn't exactly what we want because the probabilities are independent. What we really want is the infinite product of (1 - c(1/4)^n) for all natural numbers n. (c is our constant factor). I don't know an easy formula for that but it's easy to make approximations. Let's call it Q(c); it's the chance that there *aren't* any more hills for a starting probability c of having a hill.

The naive thing to do would be to go through all the numbers from 1 to infinity and have a (1/4)^n chance at each one of having a hill there. We can't do that; what we need to do is sometimes *stop*. Stopping eliminates an entire proportion of every remaining chance, so in the later stages we'll have to increase the probability to make up for it. Something like:

real probability_mod = 1;
for (int n = 0; ; ++n) {
  real chance_of_hill_here = (1/4)^n;
  if (chance_of_hill_here * probability_mod chance) {
    put a hill somewhere between size 2^(n - 1) and size 2^n
  }
  real chance_of_nothing_after_this = Q(chance_of_hill_here);
  if (chance_of_nothing_after_this) break;
  probability_mod *= (1 - chance_of_nothing_after_this);
}

However, that can fail; if we get past the "chance_of_nothing_after_this" then we'd have to guarantee that we place at least one hill after that, which this code doesn't guarantee. However, as long as (chance_of_hill_here * probability_mod chance) doesn't exceed 1, the probabilities at each n are necessarily the same as they were before. So the error is that this code makes the probabilities non-independent.

I've come up with a solution to that conceptual problem, but due to limitations of time and labor, I'm just going to implement it with a finite max hill size.

 */
    


class simple_hills {
  static const int max_simple_hill_width_shift = 8;
  static const int max_simple_hill_width = 1 << max_simple_hill_width_shift;
  struct hill {
    hill(coord x, coord y, coord height):x(x),y(y),height(height){}
    coord x;
    coord y;
    coord height;
  };
  struct hill_block {
    std::vector<hill> hills;
    bool inited = false;
    hill_block& init(coord min_x, coord min_y) {
      if (!inited) {
        memo_rng rng_here = make_rng(min_x, min_y);
        const boost::random::uniform_int_distribution<int> random_coord(0,max_simple_hill_width-1);
        for(int64_t height = 10; height<max_simple_hill_width; ++height) {
          // these numbers were picked essentially arbitrarily;
          // in order to be mathematically correct, they'd have to be normal distributions, I think.
          // right now, the borders of hill blocks are mathematically observable.
          // TODO: fix that.
          const int num_hills_of_this_height = ((height < 20) ? 10 : (10LL*20*20*20*20*20 / (height*height*height*height*height)));
          for(int i =0; i< num_hills_of_this_height; ++i) {
            hills.push_back(hill(min_x+random_coord(rng_here),min_y+random_coord(rng_here), height));
          }
        }
        inited = true;
      }
      return *this;
    }
  };
  std::unordered_map<vector3<coord>, hill_block> hill_blocks;
  
public:
  void operator()(world_column_builder& b, coord x, coord y, coord, coord) {
    b.specify_lowest(ROCK);
    b.specify(get_height(x, y), AIR);
  }

    coord get_height(coord x, coord y) {
      coord height = world_center_tile_coord - 100;
      coord base_block_x = x >> max_simple_hill_width_shift;
      coord base_block_y = y >> max_simple_hill_width_shift;
      for (int i = -1; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
          vector3<coord> hill_block(i+base_block_x, j+base_block_y, 0);
          coord block_min_x = (i+base_block_x) << max_simple_hill_width_shift;
          coord block_min_y = (j+base_block_y) << max_simple_hill_width_shift;
          auto const& hills = hill_blocks[hill_block].init(block_min_x, block_min_y);
          for(auto hill : hills.hills) {
            height += std::max(0, hill.height - get_primitive<int>(i64sqrt((hill.x-x)*(hill.x-x) + (hill.y-y)*(hill.y-y))));
          }
        }
      }
      return height;
    }
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

  worldgen_function_t pressure_tunnel(const bool has_ground) {
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

} /* end anonymous namespace */


typedef worldgen_function_t (*worldgen_function_creator)();
const std::pair<std::string, worldgen_function_creator> world_builders[] = {
  {
  "vacuum", []() {
    return worldgen_from_tilespec([](coords) {
      return AIR;
    });
  }}, {
  "flat", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z < wcc) ? ROCK : AIR;
    });
  }}, {
  "flat2", []() {
    return worldgen_from_column_spec([](world_column_builder& b, coord, coord, coord, coord) {
      b.specify_lowest(ROCK);
      b.specify(wcc, AIR);
    });
  }}, {

  "plane", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z == wcc) ? ROCK : AIR;
    });
  }}, {
  "ceiling", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z > wcc + 100) ? ROCK : AIR;
    });
  }}, {
  "low_ceiling", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z > wcc) ? ROCK : AIR;
    });
  }}, {
  "simple_hills", []() {
    return worldgen_from_column_spec(simple_hills());
  }}, {
  "spiky1", []() {
    return worldgen_from_tilespec(with_state<spiky1>());
  }}, {
  "spiky2", []() {
    return worldgen_from_column_spec(with_state<spiky2>());
  }}, {
  "spiky", []() {
    return worldgen_from_column_spec(spiky3());
  }}, {
  "spiky3", []() {
    return worldgen_from_column_spec(spiky3());
  }}, {
  "pressure_tunnel", []() {
    return pressure_tunnel(false);
  }}, {
  "pressure_tunnel_ground", []() {
    return pressure_tunnel(true);
  }}, {
  "stepped_pools", []() {
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
  }}, {


  "default", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        AIR;
    });
  }}, {
  "tower", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        AIR;
    });
  }}, {
  "tower2", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        (around_old_water_tower(l)) ? ROCK :
        AIR;
    });
  }}, {
  "tower3", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        (around_old_water_tower(l)) ? ROCK :
        (dike_surrounding_old_water_tower(l)) ? ROCK :
        AIR;
    });
  }}, {

  "shallow", []() {
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
  }}, {
  "steep", []() {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (l.z < 20 - l.x) ? ROCK :
        (l.z >= wcc+15 && (wcc + 20 - l.x) >= 15) ? GROUPABLE_WATER :
        AIR;
    });
  }}, {
  "tank", []() {
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
  }}, {
  "tank2", []() {
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
  }}, {
  "twisty", []() {
    return worldgen_from_tilespec(twisty<ROCK>());
  }}, {
  "twistyrubble", []() {
    return worldgen_from_tilespec(twisty<RUBBLE>());
  }}
};
const size_t num_world_builders = sizeof(world_builders)/sizeof(*world_builders);
const std::map<std::string, worldgen_function_creator> world_builder_table
  (world_builders + 0, world_builders + sizeof(world_builders)/sizeof(*world_builders));

worldgen_function_t make_world_building_func(std::string scenario) {
  if(auto const* worldgen = find_as_pointer(world_builder_table, scenario)) {
    return (*worldgen)();
  }
  // If it wasn't named, we have no function.
  // The caller should probably check for this unfortunateness.
  return worldgen_function_t();
}

std::vector<std::string> scenario_names() {
  std::vector<std::string> result;
  result.reserve(num_world_builders);
  for(size_t i = 0; i != num_world_builders; ++i) {
    result.push_back(world_builders[i].first);
  }
  return result;
}
