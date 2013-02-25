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

#include <tuple>
#include <unordered_map>
#include <boost/random/ranlux.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>

#include "world.hpp"
#include "worldgen.hpp"
#include "specific_worlds.hpp"


typedef tile_coordinate coord;
typedef vector3<coord> coords;

namespace /* anonymous */ {

const coord wcc = world_center_tile_coord;


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
  template<typename...Types>
  inline size_t hash_value(std::tuple<Types...> const& tup) {
    return std::hash<std::tuple<Types...>>()(tup);
  }
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
  const size_t hash = hash_value(arguments_tuple_type(args...));
  // ranlux3 uses a 32-bit seed.
  // (Don't ">> 32" to avoid silly compiler warnings on 32-bit.)
  const uint32_t seed = ((sizeof(hash) == 4) ? hash : (hash ^ (hash >> 16 >> 16)));
  memo_rng result(seed);
  return result;
}


/*

What is simple_hills?

Imagine this: Take an infinite, flat world and randomly stack conical
"hills" on top of it. If hills overlap, you simply add up, in any individual
column, the height increases called for by hills overlapping that column.

For each column, there's a certain chance if has a radius-1 hill centered
on it, a chance it has a radius-2 hill, and so forth. The chances are the
same for all columns, so we just have one function/sequence P_n : {1, 2,
3, ... } -> [0,1] that indicates the probability of each size. (We won't
worry too much about whether those probabilities are independent.)

If some tail of P_n is all zeroes, then this is relatively simple: for
each column, we can check for hills centered on all columns up to
max-hill-radius away and add up their effects. But if P_n goes unto
infinity, we need more cleverness. And it would be cool not to have
an arbitrary maximum hill size.

Considering infinity:

In order for the map to be well-defined, the columns have to have a finite
average height, i.e. there has to be a finite volume per column -
equivalently, each column has to have a finite volume-contribution from
the possible hills centered on it. Each hill of radius n contributes some
constant factor of n^3 volume, so the required condition is that the series

(P_n * n^3)

converges. P_n = n^{-5} makes this converge nicely; we'll use that.

To look up what hills have already been computed, we use an infinite
sequence of nested axis-aligned, power-of-2-aligned grids. If I want to
know whether any hills less than rad-16 affect a column, I need only
look for hill-centers in four 16x16 boxes. For bigger hills, I
have to look in four 32x32 boxes, four 64x64 boxes, and so on.

How do I look in infinite boxes?

Let's say nothing is determined yet and I want to check the height of
some column A. There's some function P'_n : N -> [0,1] that's the probability
of there being any hill centers in a box of size 2^n. The probability is
approximately
\int_{1/2}^1 x^5 dx * P_(2^n) * (2^n)^3
= (1/6 - 1/384) *  (2^n)^{-5} * (2^n)^3
~= 1/6 * (2^n)^{-2}
= 1/6 * 2^{-2n}
= 1/6 * (1/4)^n

With those factors there's less than a 1/8 chance that any hill will come
near to touching this tile, but the constant factor doesn't really matter
so we can adjust it until there's a reasonable amount of hills.

~= (1/4)^n

What's the probability that *any* hill will appear? That's a nice geometric
series so it's easy to compute its sum (3/4), but that isn't exactly what
we want because the probabilities are independent. What we really want is
the infinite product of (1 - c(1/4)^n) for all natural numbers n. (c is our
constant factor). I don't know an easy formula for that but it's easy to
make approximations. Let's call it Q(c); it's the chance that there *aren't*
any more hills for a starting probability c of having a hill.

The naive thing to do would be to go through all the numbers from 1 to
infinity and have a (1/4)^n chance at each one of having a hill there.
We can't do that; what we need to do is sometimes *stop*. Stopping eliminates
an entire proportion of every remaining chance, so in the later stages
we'll have to increase the probability to make up for it. Something like:

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

However, that can fail; if we get past the "chance_of_nothing_after_this"
then we'd have to guarantee that we place at least one hill after that,
which this code doesn't guarantee. However, as long as
(chance_of_hill_here * probability_mod chance) doesn't exceed 1,
the probabilities at each n are necessarily the same as they were before.
So the error is that this code makes the probabilities non-independent.

I've come up with a solution to that conceptual problem, but due to
limitations of time and labor, I'm just going to implement it with a
finite max hill size.

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
    bool inited;
    hill_block():inited(false){}
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
            height += std::max(coord(0), hill.height - coord(i64sqrt((hill.x-x)*(hill.x-x) + (hill.y-y)*(hill.y-y))));
          }
        }
      }
      return height;
    }
};




class fractal_hills {

public:
  fractal_hills() {
    auto inf = get_column_info(wcc - 3203, wcc - 2930);
    normalization = lint64_t(wcc) - std::max(inf.land_height, inf.potential_water_height);
  }
  void operator()(world_column_builder& b, coord x, coord y, coord, coord) {
    b.specify_lowest(ROCK);
    auto inf = get_column_info(lint64_t(x- 3203), lint64_t(y - 2930));
    lint64_t height = inf.land_height + normalization;
    assert(height >= 0);
    assert(height < (1LL << 32));
    lint64_t water_height = inf.potential_water_height + normalization;
    if (water_height > height) {
      assert(water_height >= 0);
      assert(water_height < (1LL << 32));
      b.specify(coord(height), GROUPABLE_WATER);
      b.specify(coord(water_height), AIR);
    }
    else {
      b.specify(coord(height), AIR);
    }
  }
  
private:
  lint64_t normalization;
  struct column_info {
    lint64_t land_height;
    lint64_t potential_water_height;
  };
  
    column_info get_column_info(lint64_t x, lint64_t y) {
      if (column_info const* h = find_as_pointer(column_memo_, vector3<lint64_t>(x,y,0))) {
        return *h;
      }

      column_info info;
      memo_rng rng_here = make_rng(x, y);
      uint32_t low_x_bits_zero = 0;
      uint32_t low_y_bits_zero = 0;
      luint64_t variance_num = 2;
      luint64_t variance_denom = 1;
      const uint32_t max_interesting_bits = 30;
      for (uint32_t i = 0; i <= max_interesting_bits; ++i) {
        if (!(x & (lint64_t(1) << i))) ++low_x_bits_zero;
        if (!(y & (lint64_t(1) << i))) ++low_y_bits_zero;
        if ((x | y) & (lint64_t(1) << i)) break;
        if (i < 5) {
          variance_num *= 13ULL;
          variance_denom *= 6ULL;
        }
        else {
          variance_num *= 8ULL;
          variance_denom *= 7ULL;
        }
        if (variance_num > (1ULL << 50)) {
          variance_num = (variance_num / variance_denom);
          variance_denom = 1ULL;
        }
      }
      luint64_t uvariance = (variance_num / variance_denom);
      assert(uvariance <= luint64_t(std::numeric_limits<lint64_t>::max()));
      lint64_t variance(uvariance);
      const uniform_int_distribution<lint64_t> random_coord(-variance,variance + 1);
      if (low_x_bits_zero >= max_interesting_bits && low_y_bits_zero >= max_interesting_bits) {
        // At a large enough scale, pick arbitrarily
        info.land_height = wcc + random_coord(rng_here);
        info.potential_water_height = 0;
      }
      else if (low_x_bits_zero < low_y_bits_zero) {
        // next are the ones with more zero x bits
        // TODO fix duplicate code
        lint64_t x0 = x & (~((2 << low_x_bits_zero) - 1));
        lint64_t x1 = x0 + (2 << low_x_bits_zero);
        assert((x1 - x) == (x - x0));
        column_info inf0 = get_column_info(x0, y);
        column_info inf1 = get_column_info(x1, y);
        info.land_height = (inf0.land_height + inf1.land_height + random_coord(rng_here)) / 2;
        info.potential_water_height = std::max(inf0.potential_water_height, inf1.potential_water_height);
      }
      else if (low_x_bits_zero > low_y_bits_zero) {
        // next are the ones with more zero y bits
        // TODO fix duplicate code
        lint64_t y0 = y & (~((2 << low_y_bits_zero) - 1));
        lint64_t y1 = y0 + (2 << low_y_bits_zero);
        assert((y1 - y) == (y - y0));
        column_info inf0 = get_column_info(x, y0);
        column_info inf1 = get_column_info(x, y1);
        info.land_height = (inf0.land_height + inf1.land_height + random_coord(rng_here)) / 2;
        info.potential_water_height = std::max(inf0.potential_water_height, inf1.potential_water_height);
      }
      else {
        assert(low_x_bits_zero == low_y_bits_zero);
        lint64_t x0 = x - (1 << low_x_bits_zero);
        lint64_t x1 = x + (1 << low_x_bits_zero);
        lint64_t y0 = y - (1 << low_y_bits_zero);
        lint64_t y1 = y + (1 << low_y_bits_zero);
        column_info infx0 = get_column_info(x0, y);
        column_info infx1 = get_column_info(x1, y);
        column_info infy0 = get_column_info(x, y0);
        column_info infy1 = get_column_info(x, y1);
        assert((x1 - x) == (x - x0));
        assert((y1 - y) == (y - y0));
        
        info.land_height = (infx0.land_height + infx1.land_height + infy0.land_height + infy1.land_height + random_coord(rng_here)) / 4;
        
        info.potential_water_height = std::max(std::max(infx0.potential_water_height, infx1.potential_water_height), std::max(infy0.potential_water_height, infy1.potential_water_height));
        
        if (low_x_bits_zero < 12) {
          lint64_t new_lake_height = (1LL << 62); // hack - "bigger than anything" //std::min(std::min(infx0.land_height, infx1.land_height), std::min(infy0.land_height, infy1.land_height));
          for (lint64_t offs = 0; offs <= (2 << low_x_bits_zero); ++offs) {
            column_info lakeborder_infx0 = get_column_info(x0, y0+offs);
            column_info lakeborder_infx1 = get_column_info(x1, y0+offs);
            column_info lakeborder_infy0 = get_column_info(x0+offs, y0);
            column_info lakeborder_infy1 = get_column_info(x0+offs, y1);
            if (lakeborder_infx0.land_height < new_lake_height) new_lake_height = lakeborder_infx0.land_height;
            if (lakeborder_infx1.land_height < new_lake_height) new_lake_height = lakeborder_infx1.land_height;
            if (lakeborder_infy0.land_height < new_lake_height) new_lake_height = lakeborder_infy0.land_height;
            if (lakeborder_infy1.land_height < new_lake_height) new_lake_height = lakeborder_infy1.land_height;
          }
          if (new_lake_height > info.potential_water_height) info.potential_water_height = new_lake_height;
        }
      }
      column_memo_.insert(std::make_pair(vector3<lint64_t>(x,y,0), info));
      return info;
    }
  std::unordered_map<vector3<lint64_t>, column_info> column_memo_;
};





class fractal_hills_tweaked_for_playground {

public:
  struct column_info {
    lint64_t land_height;
  };
  
    column_info get_column_info(lint64_t x, lint64_t y) {
      if (column_info const* h = find_as_pointer(column_memo_, vector3<lint64_t>(x,y,0))) {
        return *h;
      }

      column_info info;
      if ((x == 0) || (y == 0)) {
        info.land_height = 0;
        return info;
      }
      memo_rng rng_here = make_rng(x, y);
      uint32_t low_x_bits_zero = 0;
      uint32_t low_y_bits_zero = 0;
      luint64_t variance_num = 2;
      luint64_t variance_denom = 1;
      const uint32_t max_interesting_bits = 30;
      for (uint32_t i = 0; i <= max_interesting_bits; ++i) {
        if (!(x & (lint64_t(1) << i))) ++low_x_bits_zero;
        if (!(y & (lint64_t(1) << i))) ++low_y_bits_zero;
        if ((x | y) & (lint64_t(1) << i)) break;
        if (i < 5) {
          variance_num *= 13ULL;
          variance_denom *= 6ULL;
        }
        else {
          variance_num *= 8ULL;
          variance_denom *= 7ULL;
        }
        if (variance_num > (1ULL << 50)) {
          variance_num = (variance_num / variance_denom);
          variance_denom = 1ULL;
        }
      }
      luint64_t uvariance = (variance_num / variance_denom);
      assert(uvariance <= luint64_t(std::numeric_limits<lint64_t>::max()));
      lint64_t variance(uvariance);
      const uniform_int_distribution<lint64_t> random_coord(-variance,variance + 1);
      if (low_x_bits_zero >= max_interesting_bits && low_y_bits_zero >= max_interesting_bits) {
        // At a large enough scale, pick arbitrarily
        info.land_height = random_coord(rng_here);
      }
      else if (low_x_bits_zero < low_y_bits_zero) {
        // next are the ones with more zero x bits
        // TODO fix duplicate code
        lint64_t x0 = x & (~((2 << low_x_bits_zero) - 1));
        lint64_t x1 = x0 + (2 << low_x_bits_zero);
        assert((x1 - x) == (x - x0));
        column_info inf0 = get_column_info(x0, y);
        column_info inf1 = get_column_info(x1, y);
        info.land_height = (inf0.land_height + inf1.land_height + random_coord(rng_here)) / 2;
      }
      else if (low_x_bits_zero > low_y_bits_zero) {
        // next are the ones with more zero y bits
        // TODO fix duplicate code
        lint64_t y0 = y & (~((2 << low_y_bits_zero) - 1));
        lint64_t y1 = y0 + (2 << low_y_bits_zero);
        assert((y1 - y) == (y - y0));
        column_info inf0 = get_column_info(x, y0);
        column_info inf1 = get_column_info(x, y1);
        info.land_height = (inf0.land_height + inf1.land_height + random_coord(rng_here)) / 2;
      }
      else {
        assert(low_x_bits_zero == low_y_bits_zero);
        lint64_t x0 = x - (1 << low_x_bits_zero);
        lint64_t x1 = x + (1 << low_x_bits_zero);
        lint64_t y0 = y - (1 << low_y_bits_zero);
        lint64_t y1 = y + (1 << low_y_bits_zero);
        column_info infx0 = get_column_info(x0, y);
        column_info infx1 = get_column_info(x1, y);
        column_info infy0 = get_column_info(x, y0);
        column_info infy1 = get_column_info(x, y1);
        assert((x1 - x) == (x - x0));
        assert((y1 - y) == (y - y0));

        info.land_height = (infx0.land_height + infx1.land_height + infy0.land_height + infy1.land_height + random_coord(rng_here)) / 4;
      }
      column_memo_.insert(std::make_pair(vector3<lint64_t>(x,y,0), info));
      return info;
    }
  std::unordered_map<vector3<lint64_t>, column_info> column_memo_;
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

  shared_ptr<worldgen_type> pressure_tunnel(const bool has_ground) {
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


static const size_t ctr_begin = __COUNTER__ + 1;
typedef shared_ptr<worldgen_type>(*worldgen_generator_t)();

struct scenario_t {
  const char* name;
  worldgen_generator_t worldgen;
};

template<size_t N>
struct scen_n {
  static constexpr const char* name = nullptr;
  static constexpr worldgen_generator_t worldgen = nullptr;
};

#define SCENARIO_CLASS_NAMED(scen_name) \
  shared_ptr<worldgen_type> BOOST_PP_CAT(scen__, __LINE__)() { \
    return shared_ptr<worldgen_type>(new scen_name()); \
  } \
  template<> struct scen_n<(__COUNTER__ - ctr_begin)> { \
    static constexpr const char* name = BOOST_PP_STRINGIZE(scen_name); \
    typedef scen_name worldgen_type; \
    static constexpr worldgen_generator_t worldgen = &BOOST_PP_CAT(scen__, __LINE__); \
  }; \

// deprecated
#define SCENARIO_FUNCTION_NAMED(scen_name) \
  shared_ptr<worldgen_type> BOOST_PP_CAT(scen__, __LINE__)(); \
  template<> struct scen_n<(__COUNTER__ - ctr_begin)> { \
    static constexpr const char* name = scen_name; \
    static constexpr worldgen_generator_t worldgen = &BOOST_PP_CAT(scen__, __LINE__); \
  }; \
  shared_ptr<worldgen_type> BOOST_PP_CAT(scen__, __LINE__)()

SCENARIO_FUNCTION_NAMED("vacuum") {
    return worldgen_from_tilespec([](coords) {
      return AIR;
    });
}
SCENARIO_FUNCTION_NAMED("flat") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z < wcc) ? ROCK : AIR;
    });
}
SCENARIO_FUNCTION_NAMED("flat2") {
    return worldgen_from_column_spec([](world_column_builder& b, coord, coord, coord, coord) {
      b.specify_lowest(ROCK);
      b.specify(wcc, AIR);
    });
}

SCENARIO_FUNCTION_NAMED("plane") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z == wcc) ? ROCK : AIR;
    });
}
SCENARIO_FUNCTION_NAMED("ceiling") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z > wcc + 100) ? ROCK : AIR;
    });
}
SCENARIO_FUNCTION_NAMED("low_ceiling") {
    return worldgen_from_tilespec([](coords l) {
      return
        (l.z > wcc) ? ROCK : AIR;
    });
}
SCENARIO_FUNCTION_NAMED("simple_hills") {
    return worldgen_from_column_spec(simple_hills());
}
SCENARIO_FUNCTION_NAMED("fractal_hills") {
    return worldgen_from_column_spec(fractal_hills());
}
#if 0
class spiky : public worldgen_type {
  static const coord a_spike_height = 20;
  static const coord spike_multiplier_1 = 3;
  static const coord spike_multiplier_2 = 5;
  static const coord max_spike_height = a_spike_height*spike_multiplier_1*spike_multiplier_2;
  static const coord all_sky_above = wcc+max_spike_height+1;
  static const coord all_ground_below = wcc-1;
public:
  virtual worldgen_summary_of_area examine_region(tile_bounding_box region) {
    worldgen_summary_of_area result;
    if(region.max(Z) >= all_sky_above) {
      result.everything_here_is = AIR;
    }
    if(region.min(Z) <= all_ground_below) {
      result.everything_here_is = ROCK;
    }
    result.all_objects_whose_centres_are_here = worldgen_summary_of_area::no_objects();
    return result;
  }
  void create_tiles(block_initializer b) override {
    b.initialize_by_column([this](coord x, coord y, column_callback fz) {
      const coord air_begins_at_z(height_memo_(x, y));
      fz([air_begins_at_z](coord z) {
        return (z < air_begins_at_z) ? ROCK : AIR;
      });
    });
  }

  struct get_height {
    coord operator()(coord x, coord y)const {
      // TODO include a constant-for-this-instance-of-the-world random-seed too.
      memo_rng rng_here = make_rng(x, y);

      const int which = uniform_int_distribution<int>(0,20)(rng_here);
      coord spike_max = a_spike_height;
      if (which < 5) spike_max *= spike_multiplier_1;
      if (which == 0) spike_max *= spike_multiplier_2;
      const uniform_int_distribution<coord> random_spike_height(0,spike_max);
      coord height = wcc + random_spike_height(rng_here);
      return height;
    }
  };
private:
  memoized<get_height, coord (coord, coord)> height_memo_;
};
SCENARIO_CLASS_NAMED(spiky)
#endif

const coord tower_min = 80;
const coord tower_max = 90;
const coord tower_freq = 60;

bool tower_boundary(coord b) {
  if (b > wcc - tower_min) return false;
  if ((((wcc - tower_min) - b) % tower_freq) == 0) return true;
  if ((((wcc - tower_min) - b) % tower_freq) == (tower_max - tower_min)) return true;
  return false;
}
bool tower_any(coord b) {
  if (b > wcc - tower_min) return false;
  if ((((wcc - tower_min) - b) % tower_freq) <= (tower_max - tower_min)) return true;
  return false;
}
coord tower_center(coord b) {
  assert ((((wcc - tower_min) - b) % tower_freq) <= tower_freq);
  assert ((((wcc - tower_min) - b) % tower_freq) >= 0);
  /*LOG << b << "\n";
  LOG << (((wcc - tower_min) - b) % tower_freq) << "\n";
  LOG << ((tower_max - tower_min)/2) << "\n";
  LOG << b + (((wcc - tower_min) - b) % tower_freq) - ((tower_max - tower_min)/2) << "\n";*/
  return b + (((wcc - tower_min) - b) % tower_freq) - ((tower_max - tower_min)/2);
}

class playground : public worldgen_type {
public:
  virtual worldgen_summary_of_area examine_region(tile_bounding_box region) {
    worldgen_summary_of_area result;
#if 0
    if(region.min(Z) >= wcc + 2000) {
      result.everything_here_is = AIR;
    }
    if(region.max(Z) <= wcc - 7000) {
      result.everything_here_is = ROCK;
    }
    if (region.subsumes(tile_bounding_box(world_center_tile_coords))) {
      result.all_objects_whose_centres_are_here = worldgen_summary_of_area::no_objects();
      result.all_objects_whose_centres_are_here->first.push_back(shared_ptr<object>(new refinery(world_center_tile_coords)));
    }
    else {
      result.all_objects_whose_centres_are_here = worldgen_summary_of_area::no_objects();
    }
      result.all_objects_whose_centres_are_here->first.push_back(shared_ptr<object>(new refinery(world_center_tile_coords)));
      result.all_objects_whose_centres_are_here->second = tile_bounding_box(vector3<tile_coordinate>(wcc-200,wcc-200,wcc-200), vector3<tile_coordinate>(400,400,400));
#endif
    return result;
  }
  void create_tiles(block_initializer b) override {
    b.initialize_by_column([this](coord x, coord y, column_callback fz) {
      const coord air_begins_at_z(get_height(x, y));
      if (tower_any(x) && tower_any(y)) {
        const coord water_height = fhillsheight(tower_center(x), tower_center(y))+80;
        fz([air_begins_at_z, water_height](coord z) {
          return (z < air_begins_at_z) ? ROCK : (z < water_height) ? GROUPABLE_WATER : AIR;
        });
      }
      else {
        fz([air_begins_at_z](coord z) {
          return (z < air_begins_at_z) ? ROCK : AIR;
        });
      }
    });
  }

    coord get_height(coord x, coord y) {
      if (((x >= wcc - 4) && (x <= wcc + 4)) || ((y >= wcc - 4) && (y <= wcc + 4))) {
        return wcc;
      }
      else if ((x < wcc - 4) && (y < wcc - 4)) {
        if ((x == wcc - 5) && (y >= (wcc - 5 - 20))) return (2*wcc - 5 - y);
        else if ((x > wcc - 20) || (y > wcc - 20)) return wcc + 20;
        else if (tower_any(x) && tower_any(y) && (tower_boundary(x) || tower_boundary(y))) {
          return fhillsheight(tower_center(x), tower_center(y))+150;
        }
        else {
          return fhillsheight(x, y);
        }
      }
      else if ((x > wcc + 4) && (y > wcc + 4)) {
        if ((x == wcc + 5) && (y <= (wcc + 5 + 20))) return (2*wcc - y + 5);
        return wcc - 20;
      }
      else if ((x > wcc + 4) && (y < wcc - 4)) {
        return wcc;
      }
      else {
        assert((x < wcc - 4) && (y > wcc + 4));
        return wcc;
      }
    }
private:
  coord fhillsheight(coord x, coord y) { return wcc + 20 + fhills.get_column_info(lint64_t(wcc - 20) - x, lint64_t(wcc - 20) - y).land_height; }
  fractal_hills_tweaked_for_playground fhills;
};
SCENARIO_CLASS_NAMED(playground)

// These are worst cases in terms of number of non-interior (theoretically
// visible) tiles within a radius; useful to test as a stress-test and
// because someone might intentionally build them (to be mean, or more
// likely, because it's actually useful for some in-game-engineering
// reason).
SCENARIO_FUNCTION_NAMED("spiky_checkerboard") {
    return worldgen_from_column_spec([](world_column_builder& b, coord x, coord y, coord, coord) {
      b.specify_lowest(ROCK);
      b.specify(wcc - (1<<14) + (((x ^ y) & 1) << 18), AIR);
    });
}
SCENARIO_FUNCTION_NAMED("spiky_3dcheckerboard") {
    return worldgen_from_tilespec([](coords l) {
      return ((l.x ^ l.y ^ l.z) & 1) ? ROCK : AIR;
    });
}

SCENARIO_FUNCTION_NAMED("pressure_tunnel") {
    return pressure_tunnel(false);
}
SCENARIO_FUNCTION_NAMED("pressure_tunnel_ground") {
    return pressure_tunnel(true);
}
SCENARIO_FUNCTION_NAMED("stepped_pools") {
    return worldgen_from_tilespec([](coords l)->tile_contents {
      typedef lint64_t number;
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

SCENARIO_FUNCTION_NAMED("default") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        AIR;
    });
}
SCENARIO_FUNCTION_NAMED("tower") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        AIR;
    });
}
SCENARIO_FUNCTION_NAMED("tower2") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (in_old_water_tower(l)) ? GROUPABLE_WATER :
        (around_old_water_tower(l)) ? ROCK :
        AIR;
    });
}
SCENARIO_FUNCTION_NAMED("tower3") {
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

SCENARIO_FUNCTION_NAMED("shallow") {
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
SCENARIO_FUNCTION_NAMED("steep") {
    return worldgen_from_tilespec([](coords l) {
      return
        (!in_old_box(l)) ? AIR :
        (is_old_box(l)) ? ROCK :
        (l.z < 20 - l.x) ? ROCK :
        (l.z >= wcc+15 && (wcc + 20 - l.x) >= 15) ? GROUPABLE_WATER :
        AIR;
    });
}
SCENARIO_FUNCTION_NAMED("tank") {
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
SCENARIO_FUNCTION_NAMED("tank2") {
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
SCENARIO_FUNCTION_NAMED("twisty") {
    return worldgen_from_tilespec(twisty<ROCK>());
}
SCENARIO_FUNCTION_NAMED("twistyrubble") {
    return worldgen_from_tilespec(twisty<RUBBLE>());
}


static const size_t ctr_end = __COUNTER__;

#define ESTIMATED_NUMBER_OF_SCENARIOS 30
static const size_t actual_number_of_scenarios = ctr_end - ctr_begin;
#define SCEN(z, n, d) \
  { scen_n<(n)>::name, scen_n<(n)>::worldgen },
constexpr scenario_t scenarios[] = {
  BOOST_PP_REPEAT(ESTIMATED_NUMBER_OF_SCENARIOS, SCEN, _)
};
template<int N> struct check_number_of_scenarios {
  static_assert(
    // allow a little leeway but not wasting too much binary space
    ESTIMATED_NUMBER_OF_SCENARIOS >= N && ESTIMATED_NUMBER_OF_SCENARIOS <= N + 7,
  "update ESTIMATED_NUMBER_OF_SCENARIOS above to be the number shown in check_number_of_scenarios<N> in your error message, or a bit above that");
};
template struct check_number_of_scenarios<actual_number_of_scenarios>;

shared_ptr<worldgen_type> make_world_building_func(std::string scenario) {
  for(auto scen : scenarios) {
    if(scen.name && scenario == scen.name) {
      return (*scen.worldgen)();
    }
  }
  // If it wasn't named, we have no function.
  // The caller should probably check for this unfortunateness.
  return shared_ptr<worldgen_type>();
}

std::vector<std::string> scenario_names() {
  std::vector<std::string> result;
  const size_t num_world_builders = actual_number_of_scenarios;
  result.reserve(num_world_builders);
  for(size_t i = 0; i != num_world_builders; ++i) {
    result.push_back(scenarios[i].name);
  }
  return result;
}
