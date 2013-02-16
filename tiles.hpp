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

#ifndef LASERCAKE_TILES_HPP__
#define LASERCAKE_TILES_HPP__

#include <utility>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <ostream>

#include "utils.hpp"
#include "world_constants.hpp"
#include "data_structures/misc_structures.hpp"

using std::pair;
using std::make_pair;
using std::map;
using std::set;
using std::unordered_map;
using std::unordered_set;



// The minimum valid tile_coordinate is zero.
//
// This prevents accidentally dividing negative tile_coordinates,
// expecting it to round towards lowest, and possibly having it
// round towards zero instead.
//
// The type is signed, however, to make code more likely to be
// correct: subtracting two tile_coordinates should generally
// produce a signed number.
//
// These decisions mean that the unavoidable boundaries of
// the space are
// bounding_box::min_and_max(
//   vector3<fine_scalar>(0,0,0),
//   vector3<fine_scalar>(INT32_MAX*tile_width, INT32_MAX*tile_width,
//                                              INT32_MAX*tile_height),
// ).
// In practice they should be, at least, a few worldblocks away
// from that or more, so that local/regional effects like tile
// physics and lasers don't try to look in places beyond the edge.
//
// TODO it might be reasonable to also have a tile type of
// IMPENETRABLE_EDGE_OF_THE_WORLD and have a few (or few thousand)
// worldblocks worth of those as necessary at all the edges,
// just to be extra cautious. Currently we just rely on the user
// not bothering to go nearly that far from the world-center coordinate.
typedef lint32_t tile_coordinate;
typedef lint32_t tile_coordinate_signed_type;

struct tile_bounding_box {
  vector3<tile_coordinate> min_;
  vector3<tile_coordinate> size_;
  tile_bounding_box(){}
  tile_bounding_box(vector3<tile_coordinate> coords):min_(coords),size_(1,1,1){}
  tile_bounding_box(vector3<tile_coordinate> min, vector3<tile_coordinate> size):min_(min),size_(size){}
  vector3<tile_coordinate> min()const { return min_; }
  vector3<tile_coordinate> size()const { return size_; }
  vector3<tile_coordinate> max()const { return min_ + (size_ - vector3<tile_coordinate>(1,1,1)); }
  vector3<tile_coordinate> size_minus_one()const { return size_ - vector3<tile_coordinate>(1,1,1); }
  tile_coordinate min(which_dimension_type dim)const { return min_(dim); }
  tile_coordinate size(which_dimension_type dim)const { return size_(dim); }
  tile_coordinate max(which_dimension_type dim)const { return min_(dim) + (size_(dim) - 1); }
  tile_coordinate size_minus_one(which_dimension_type dim)const { return size_(dim) - 1; }

  bool contains(vector3<tile_coordinate> v) {
    return (v.x >= min_.x && v.x <= min_.x + (size_.x - 1) &&
            v.y >= min_.y && v.y <= min_.y + (size_.y - 1) &&
            v.z >= min_.z && v.z <= min_.z + (size_.z - 1));
  }
  bool overlaps(tile_bounding_box const& o)const {
    return
         min(X) < (o.min(X)+o.size(X)) && o.min(X) < (min(X)+size(X))
      && min(Y) < (o.min(Y)+o.size(Y)) && o.min(Y) < (min(Y)+size(Y))
      && min(Z) < (o.min(Z)+o.size(Z)) && o.min(Z) < (min(Z)+size(Z));
  }
  bool subsumes(tile_bounding_box const& o)const {
    return
         min(X) <= o.min(X) && (o.min(X)+o.size(X)) <= (min(X)+size(X))
      && min(Y) <= o.min(Y) && (o.min(Y)+o.size(Y)) <= (min(Y)+size(Y))
      && min(Z) <= o.min(Z) && (o.min(Z)+o.size(Z)) <= (min(Z)+size(Z));
  }

  class iterator : public boost::iterator_facade<iterator, vector3<tile_coordinate>, boost::forward_traversal_tag, vector3<tile_coordinate>>
  {
    public:
      iterator(){}
      bool equal(iterator other)const { return data_ == other.data_; }
      void increment() {
        data_.first.x += 1;
        if (data_.first.x >= data_.second->min(X) + data_.second->size(X)) {
          data_.first.x = data_.second->min(X);
          data_.first.y += 1;
          if (data_.first.y >= data_.second->min(Y) + data_.second->size(Y)) {
            data_.first.y = data_.second->min(Y);
            data_.first.z += 1;
          }
        }
      }
      vector3<tile_coordinate> dereference()const { return data_.first; }

      explicit iterator(pair<vector3<tile_coordinate>, tile_bounding_box const*> data):data_(data){}

      friend inline std::ostream& operator<<(std::ostream& os, tile_bounding_box::iterator const& it) {
        return os << '{' << it.data_.second << "@" << it.data_.first << '}';
      }
    private:
      friend class boost::iterator_core_access;
      friend struct tile_bounding_box;
      pair<vector3<tile_coordinate>, tile_bounding_box const*> data_;

  };
  typedef vector3<tile_coordinate> value_type;
  typedef value_type reference;
  typedef value_type const_reference;
  typedef value_type* pointer;
  typedef value_type const* const_pointer;
  typedef iterator iterator;
  typedef iterator const_iterator;
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;

  iterator begin()const{ return iterator(make_pair(min(), this)); }
  iterator end()const{ return iterator(make_pair(min() + vector3<tile_coordinate>(0, 0, size(Z)), this)); }
};

inline std::ostream& operator<<(std::ostream& os, tile_bounding_box const& bb) {
  return os << '[' << bb.min() << ", " << bb.max() << ']';
}



/*

Whether tiles are water or not
 -- dictates, at one tile distance --
Whether water tiles are "sticky water" ("fewer than two adjacent tiles are
air") or "free water" (at least two adjacent air tiles)
 -- dictates, at one tile distance --
Whether sticky water tiles are "interior water" ("every adjacent tile is sticky
water") or "membrane water" ("at least one tile is not a sticky water tile")

This is a one-way dictation - the latter things don't affect the former things
at all (until they cause water to actually move, anyway). It's not an exact
dictation.

*/

enum tile_contents {
  AIR = 0,
  ROCK,
  UNGROUPABLE_WATER,
  GROUPABLE_WATER,
  RUBBLE,
  UNSPECIFIED_TILE_CONTENTS // Only valid to use in particular parts of code that note thus.
  // NOTE: we currently rely on these fitting into three bits
};

inline bool is_fluid(tile_contents t) {
  return (t >= UNGROUPABLE_WATER) && (t <= RUBBLE);
}
inline bool is_water(tile_contents t) {
  return (t >= UNGROUPABLE_WATER) && (t <= GROUPABLE_WATER);
}

// This could be an equivalence relation other than equality someday.
inline bool these_tile_contents_are_identical_regarding_interiorness(tile_contents t1, tile_contents t2) {
  return t1 == t2;
}
// These tiles, when next to each other, are different enough to form
// an imaginary membrane.
inline bool neighboring_tiles_with_these_contents_are_not_interior(tile_contents t1, tile_contents t2) {
  return t1 != t2;
}

class tile {
public:
  // 'tile' is trivially-constructible; to get a valid tile you must
  // call make_tile_with_contents_and_interiorness().
  static tile make_tile_with_contents_and_interiorness(tile_contents contents, bool interior) {
    tile result;
    result.tile_data_ = contents | (uint8_t(interior) << interior_bit_position);
    return result;
  }

  // Air is expected to always be marked "interior".

  // For tile based physics (e.g. water movement)
  // This is so that we don't have to search the collision-detector for
  // relevant objects at every tile.
  bool there_is_an_object_here_that_affects_the_tile_based_physics()const {
    return tile_data_ & there_is_an_object_here_that_affects_the_tile_based_physics_mask; }
  bool is_interior()const { return tile_data_ & interior_bit_mask; }
  tile_contents contents()const{ return (tile_contents)(tile_data_ & contents_mask); }

  void set_contents(tile_contents new_contents) {
    tile_data_ = (tile_data_ & ~contents_mask) | (uint8_t(new_contents) & contents_mask);
  }
  void set_whether_there_is_an_object_here_that_affects_the_tile_based_physics(bool b) {
    tile_data_ = (tile_data_ & ~there_is_an_object_here_that_affects_the_tile_based_physics_mask)
         | (b ? there_is_an_object_here_that_affects_the_tile_based_physics_mask : uint8_t(0));
  }
  void set_interiorness(bool b) {
    tile_data_ = (tile_data_ & ~interior_bit_mask) | (b ? interior_bit_mask : uint8_t(0));
  }

  // Public for the sake of performance-related bit hacks:
  static const uint8_t contents_mask = 0x7;
  static const int interior_bit_position = 3;
  static const int there_is_an_object_here_that_affects_the_tile_based_physics_bit_position = 4;
  static const uint8_t interior_bit_mask = (1<<interior_bit_position);
  static const uint8_t there_is_an_object_here_that_affects_the_tile_based_physics_mask =
    (1<<there_is_an_object_here_that_affects_the_tile_based_physics_bit_position);

  // As a debugging aid; it could certainly produce more human-transparent output!
  friend inline std::ostream& operator<<(std::ostream& os, tile t) {
    return os << std::hex << t.tile_data_ << std::dec;
  }

private:
  uint8_t tile_data_;
};


class world;

namespace the_decomposition_of_the_world_into_blocks_impl {
  class worldblock;
  typedef int worldblock_dimension_type;
}

enum level_of_tile_realization_needed {
  COMPLETELY_IMAGINARY = 0,
  CONTENTS_ONLY = 1,
  CONTENTS_AND_LOCAL_CACHES_ONLY = 2,
  FULL_REALIZATION = 3
};

class tile_location;
namespace tile_physics_impl {
  tile& mutable_stuff_at(tile_location const& loc);
  void set_tile_interiorness(tile_location const& loc, bool interior);
}

class tile_location {
public:
  //tile_location operator+(cardinal_direction dir)const;
  // Equivalent to operator+, except allowing you to specify the amount of realization needed.
  template<cardinal_direction Dir> tile_location get_neighbor(level_of_tile_realization_needed realineeded)const;
  tile_location get_neighbor_by_variable(cardinal_direction dir, level_of_tile_realization_needed realineeded)const;
  std::array<tile, num_cardinal_directions> get_all_neighbor_tiles(level_of_tile_realization_needed realineeded)const;

  bool operator==(tile_location const& other)const { return v_ == other.v_; }
  bool operator!=(tile_location const& other)const { return v_ != other.v_; }
  bool operator<(tile_location const& other)const { return v_ < other.v_; }
  inline tile const& stuff_at()const;
  vector3<tile_coordinate> const& coords()const { return v_; }

  // null locations are occasionally permissible
  bool is_location()const { return wb_; }
  friend tile_location trivial_invalid_location();

  friend inline std::ostream& operator<<(std::ostream& os, tile_location const& l) {
    return os << l.v_;
  }
  friend inline size_t hash_value(tile_location const& l) { return std::hash<vector3<tile_coordinate>>()(l.coords()); }

  // This constructor should only be used when you know exactly what worldblock it's in!!
  // i.e. the worldblock code.
  tile_location(vector3<tile_coordinate> v,
                the_decomposition_of_the_world_into_blocks_impl::worldblock_dimension_type idx,
                the_decomposition_of_the_world_into_blocks_impl::worldblock *wb);

private:
  // tile_physics.cpp is the only code permitted to modify tile contents
  friend tile& tile_physics_impl::mutable_stuff_at(tile_location const& loc);
  friend void tile_physics_impl::set_tile_interiorness(tile_location const& loc, bool interior);

  tile_location() : v_(0,0,0), idx_(0), wb_(nullptr) {}

  vector3<tile_coordinate> v_;
  the_decomposition_of_the_world_into_blocks_impl::worldblock_dimension_type idx_;
  the_decomposition_of_the_world_into_blocks_impl::worldblock* wb_; // invariant: nonnull
};

inline tile_location trivial_invalid_location() { return tile_location(); }

namespace std {
  template<> struct hash<tile_location> {
    inline size_t operator()(tile_location const& l) const {
      return hash_value(l);
    }
  };
}

struct tile_compare_xyz { bool operator()(tile_location const& i, tile_location const& j)const; };
struct tile_compare_yzx { bool operator()(tile_location const& i, tile_location const& j)const; };
struct tile_compare_zxy { bool operator()(tile_location const& i, tile_location const& j)const; };

inline std::array<tile_location, num_cardinal_directions>
get_all_neighbors(
      tile_location const& loc,
      level_of_tile_realization_needed realineeded = FULL_REALIZATION
) {
  return std::array<tile_location, num_cardinal_directions>({{
    loc.get_neighbor<0>(realineeded),
    loc.get_neighbor<1>(realineeded),
    loc.get_neighbor<2>(realineeded),
    loc.get_neighbor<3>(realineeded),
    loc.get_neighbor<4>(realineeded),
    loc.get_neighbor<5>(realineeded)
  }});
}

inline std::array<tile_location, num_cardinal_directions-2>
get_perpendicular_neighbors(
      tile_location const& loc,
      cardinal_direction dir,
      level_of_tile_realization_needed realineeded = FULL_REALIZATION
) {
  std::array<tile_location, num_cardinal_directions-2> result = {{
    trivial_invalid_location(), trivial_invalid_location(),
    trivial_invalid_location(), trivial_invalid_location()
  }};
  size_t i = 0;
  if (cardinal_directions_are_perpendicular(dir, 0)) { result[i++] = loc.get_neighbor<0>(realineeded); }
  if (cardinal_directions_are_perpendicular(dir, 1)) { result[i++] = loc.get_neighbor<1>(realineeded); }
  if (cardinal_directions_are_perpendicular(dir, 2)) { result[i++] = loc.get_neighbor<2>(realineeded); }
  if (cardinal_directions_are_perpendicular(dir, 3)) { result[i++] = loc.get_neighbor<3>(realineeded); }
  if (cardinal_directions_are_perpendicular(dir, 4)) { result[i++] = loc.get_neighbor<4>(realineeded); }
  if (cardinal_directions_are_perpendicular(dir, 5)) { result[i++] = loc.get_neighbor<5>(realineeded); }
  assert(i == 4);
  return result;
}

inline std::array<tile_location, num_cardinal_directions>
get_perpendicular_neighbors_numbered_as_neighbors(
      tile_location const& loc,
      cardinal_direction dir,
      level_of_tile_realization_needed realineeded = FULL_REALIZATION
) {
  return std::array<tile_location, num_cardinal_directions>({{
    cardinal_directions_are_perpendicular(dir, 0) ? loc.get_neighbor<0>(realineeded) : trivial_invalid_location(),
    cardinal_directions_are_perpendicular(dir, 1) ? loc.get_neighbor<1>(realineeded) : trivial_invalid_location(),
    cardinal_directions_are_perpendicular(dir, 2) ? loc.get_neighbor<2>(realineeded) : trivial_invalid_location(),
    cardinal_directions_are_perpendicular(dir, 3) ? loc.get_neighbor<3>(realineeded) : trivial_invalid_location(),
    cardinal_directions_are_perpendicular(dir, 4) ? loc.get_neighbor<4>(realineeded) : trivial_invalid_location(),
    cardinal_directions_are_perpendicular(dir, 5) ? loc.get_neighbor<5>(realineeded) : trivial_invalid_location(),
  }});
}

/*
Nobody actually cares to learn what all the 2diagonals are without knowing the directions.
This stands as a reminder of what we could do, anyway.

inline std::array<tile_location, 12> get_all_2diagonals(std::array<tile_location, num_cardinal_directions>& neighbors) {
  return std::array<tile_location, 12>({{
    neighbors[zplus ].get_neighbor<xplus >(realineeded),
    neighbors[zplus ].get_neighbor<yplus >(realineeded),
    neighbors[zplus ].get_neighbor<xminus>(realineeded),
    neighbors[zplus ].get_neighbor<yminus>(realineeded),
    neighbors[zminus].get_neighbor<xplus >(realineeded),
    neighbors[zminus].get_neighbor<yplus >(realineeded),
    neighbors[zminus].get_neighbor<xminus>(realineeded),
    neighbors[zminus].get_neighbor<yminus>(realineeded),
    neighbors[xplus ].get_neighbor<yplus >(realineeded),
    neighbors[xplus ].get_neighbor<yminus>(realineeded),
    neighbors[xminus].get_neighbor<yplus >(realineeded),
    neighbors[xminus].get_neighbor<yminus>(realineeded)
  }});
}*/


class literally_random_access_removable_tiles_by_height {
public:
  typedef map<tile_coordinate, literally_random_access_set<tile_location>> map_t;

  template<typename RNG> tile_location get_and_erase_random_from_the_top(RNG& rng);
  template<typename RNG> tile_location get_and_erase_random_from_the_bottom(RNG& rng);
  bool erase(tile_location const& loc);
  void insert(tile_location const& loc);
  bool any_above(tile_coordinate height)const;
  bool any_below(tile_coordinate height)const;

  map_t const& as_map()const { return data_; }
private:
  map_t data_;
};


// Conversions between tile_coordinate and fine_scalar space:

// A fine_scalar location intersects 1-8 tiles.  Most of the time, it intersects
// just one tile.
//
// A tile at tile-space (x,y,z) occupies fine_scalar space
//   ([x*tile_width , (x+1)*tile_width ],
//    [y*tile_width , (y+1)*tile_width ],
//    [z*tile_height, (z+1)*tile_height]
//   ).
// The ranges are inclusive on both low and high ends ("closed ranges").
// This means neighboring tiles overlap by a zero-volume amount.
// This makes things a bit more complicated, but the alternatives are worse.
//
// If we used [x*tile_width, (x+1)*tile_width - 1], etc, then there
// would be empty space of one fine_scalar unit between tiles which
// lasers or players' eyes at the right angle could poke through.
//
// If we made them open ranges, then a correctly aimed horizontal/vertical
// laser could *still* poke between between two tiles.
//
// If we made them half-open ranges e.g. [x*tile_width , (x+1)*tile_width),
// then the world would be fundamentally asymmetric.  Running into the east
// side of a tile would have different physics from running into the west
// side of a tile.  For example, you could point a laser exactly along the
// edge of the east side of a tile but not the west side, or similar.
// (Both the display, and the physics of things that compute exactly with angles,
// are affected by whether something is "...x)" or "...x-1]".)
//
// All these "if we..." alternatives are horrible somehow, so instead we
// accept a little code complexity to allow for the fact that tiles touch.


// (here allowing old semantics of [x*tile_width, (x+1)*tile_width - 1], etc
//  to be enabled with a compiler flag so we can continue to test the
//  performance impacts - it affects bbox_collision_detector more than
//  would be nice.)
#if defined(LASERCAKE_OLD_SMALLER_TILES)
const bool older_smaller_nonintersecting_tiles_with_gaps_between_them = true;
#else
const bool older_smaller_nonintersecting_tiles_with_gaps_between_them = false;
#endif

inline tile_coordinate get_min_containing_tile_coordinate(fine_scalar c, which_dimension_type which_coordinate) {
  if (which_coordinate == Z) return tile_coordinate(c / tile_height);
  else                       return tile_coordinate(c / tile_width);
}
inline tile_coordinate get_max_containing_tile_coordinate(fine_scalar c, which_dimension_type which_coordinate) {
  if (older_smaller_nonintersecting_tiles_with_gaps_between_them) {
    return get_min_containing_tile_coordinate(c, which_coordinate);
  }
  if (which_coordinate == Z) return tile_coordinate((c + 1*fine_units) / tile_height);
  else                       return tile_coordinate((c + 1*fine_units) / tile_width);
}
inline vector3<tile_coordinate> get_min_containing_tile_coordinates(vector3<fine_scalar> v) {
  return vector3<tile_coordinate>(
    get_min_containing_tile_coordinate(v[0], 0),
    get_min_containing_tile_coordinate(v[1], 1),
    get_min_containing_tile_coordinate(v[2], 2)
  );
}
inline vector3<tile_coordinate> get_max_containing_tile_coordinates(vector3<fine_scalar> v) {
  if (older_smaller_nonintersecting_tiles_with_gaps_between_them) {
    return get_min_containing_tile_coordinates(v);
  }
  return vector3<tile_coordinate>(
    get_max_containing_tile_coordinate(v[0], 0),
    get_max_containing_tile_coordinate(v[1], 1),
    get_max_containing_tile_coordinate(v[2], 2)
  );
}
inline vector3<tile_coordinate> get_arbitrary_containing_tile_coordinates(vector3<fine_scalar> v) {
  return get_min_containing_tile_coordinates(v);
}
inline lasercake_vector<vector3<tile_coordinate>>::type
get_all_containing_tile_coordinates(vector3<fine_scalar> v) {
  lasercake_vector<vector3<tile_coordinate>>::type result;
  if (older_smaller_nonintersecting_tiles_with_gaps_between_them) {
    result.push_back(get_min_containing_tile_coordinates(v));
  }
  else {
    const vector3<tile_coordinate> base = get_min_containing_tile_coordinates(v);
    const bool on_x_boundary = (v.x % tile_width == 0);
    const bool on_y_boundary = (v.y % tile_width == 0);
    const bool on_z_boundary = (v.z % tile_height == 0);
    for(tile_coordinate xplus = on_x_boundary; xplus >= 0; --xplus) {
    for(tile_coordinate yplus = on_y_boundary; yplus >= 0; --yplus) {
    for(tile_coordinate zplus = on_z_boundary; zplus >= 0; --zplus) {
      result.push_back(base + vector3<tile_coordinate>(xplus, yplus, zplus));
    }
    }
    }
  }
  return result;
}
template<typename RNG>
inline vector3<tile_coordinate> get_random_containing_tile_coordinates(vector3<fine_scalar> v, RNG& rng) {
  if (older_smaller_nonintersecting_tiles_with_gaps_between_them) {
    return get_min_containing_tile_coordinates(v);
  }
  const boost::random::uniform_int_distribution<int32_t> random_bits(0, 7);
  const int32_t three_random_bits = random_bits(rng);
  const fine_scalar xplus = int(bool(three_random_bits & 1))*fine_distance_units;
  const fine_scalar yplus = int(bool(three_random_bits & 2))*fine_distance_units;
  const fine_scalar zplus = int(bool(three_random_bits & 4))*fine_distance_units;
  return vector3<tile_coordinate>(
    tile_coordinate((v.x + xplus) / tile_width),
    tile_coordinate((v.y + yplus) / tile_width),
    tile_coordinate((v.z + zplus) / tile_height)
  );
}
inline fine_scalar lower_bound_in_fine_units(tile_coordinate c, which_dimension_type which_coordinate) {
  if (which_coordinate == Z) return c * tile_height;
  else                       return c * tile_width;
}
inline fine_scalar upper_bound_in_fine_units(tile_coordinate c, which_dimension_type which_coordinate) {
  if (older_smaller_nonintersecting_tiles_with_gaps_between_them) {
    if (which_coordinate == Z) return c * tile_height + (tile_height - 1*fine_units);
    else                       return c * tile_width + (tile_width - 1*fine_units);
  }
  else {
    if (which_coordinate == Z) return c * tile_height + tile_height;
    else                       return c * tile_width + tile_width;
  }
}
inline vector3<fine_scalar> lower_bound_in_fine_units(vector3<tile_coordinate> v) {
  return vector3<fine_scalar>(
    lower_bound_in_fine_units(v[0], 0),
    lower_bound_in_fine_units(v[1], 1),
    lower_bound_in_fine_units(v[2], 2)
  );
}
inline vector3<fine_scalar> upper_bound_in_fine_units(vector3<tile_coordinate> v) {
  return vector3<fine_scalar>(
    upper_bound_in_fine_units(v[0], 0),
    upper_bound_in_fine_units(v[1], 1),
    upper_bound_in_fine_units(v[2], 2)
  );
}

inline bounding_box fine_bounding_box_of_tile(vector3<tile_coordinate> v) {
  return bounding_box::min_and_max(lower_bound_in_fine_units(v), upper_bound_in_fine_units(v));
}

inline bounding_box convert_to_fine_units(tile_bounding_box const& bb) {
  if (older_smaller_nonintersecting_tiles_with_gaps_between_them) {
    return bounding_box::min_and_max(
      lower_bound_in_fine_units(bb.min()),
      lower_bound_in_fine_units(bb.min() + bb.size())
        - vector3<fine_scalar>(1*fine_units, 1*fine_units, 1*fine_units)
    );
  }
  else {
    return bounding_box::min_and_max(
      lower_bound_in_fine_units(bb.min()),
      lower_bound_in_fine_units(bb.min() + bb.size())
    );
  }
}

inline tile_bounding_box get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(bounding_box const& bb) {
  const vector3<tile_coordinate> min = get_min_containing_tile_coordinates(bb.min());
  const vector3<tile_coordinate> max = get_max_containing_tile_coordinates(bb.max());
  const vector3<tile_coordinate> size = max - min + vector3<tile_coordinate>(1,1,1);
  return tile_bounding_box(min, size);
}



//in header file because they're templates:

template<typename RNG> inline
tile_location literally_random_access_removable_tiles_by_height::get_and_erase_random_from_the_top(RNG& rng) {
  const map_t::iterator iter_at_top = boost::prior(data_.end());
  literally_random_access_set<tile_location>& tiles_at_top = iter_at_top->second;
  const tile_location result = tiles_at_top.get_random(rng);
  tiles_at_top.erase(result);
  if (tiles_at_top.empty()) data_.erase(iter_at_top);
  return result;
}
template<typename RNG> inline
tile_location literally_random_access_removable_tiles_by_height::get_and_erase_random_from_the_bottom(RNG& rng) {
  const map_t::iterator iter_at_bottom = data_.begin();
  literally_random_access_set<tile_location>& tiles_at_bottom = iter_at_bottom->second;
  const tile_location result = tiles_at_bottom.get_random(rng);
  tiles_at_bottom.erase(result);
  if (tiles_at_bottom.empty()) data_.erase(iter_at_bottom);
  return result;
}

#endif

