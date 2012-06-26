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

#include "utils.hpp"
#include "world_constants.hpp"

using std::pair;
using std::make_pair;
using std::map;
using std::set;
using std::unordered_map;
using std::unordered_set;



typedef lasercake_int<int32_t>::type tile_coordinate;
typedef lasercake_int<int32_t>::type tile_coordinate_signed_type;

struct tile_bounding_box {
  vector3<tile_coordinate> min;
  vector3<tile_coordinate> size;
  tile_bounding_box(){}
  tile_bounding_box(vector3<tile_coordinate> coords):min(coords),size(1,1,1){}
  tile_bounding_box(vector3<tile_coordinate> min, vector3<tile_coordinate> size):min(min),size(size){}
  bool contains(vector3<tile_coordinate> v) {
    return (v.x >= min.x && v.x <= min.x + (size.x - 1) &&
            v.y >= min.y && v.y <= min.y + (size.y - 1) &&
            v.z >= min.z && v.z <= min.z + (size.z - 1));
  }

  class iterator : public boost::iterator_facade<iterator, vector3<tile_coordinate>, boost::forward_traversal_tag, vector3<tile_coordinate>>
  {
    public:
      iterator(){}
      bool equal(iterator other)const { return data_ == other.data_; }
      void increment() {
        data_.first.x += 1;
        if (data_.first.x >= data_.second->min.x + data_.second->size.x) {
          data_.first.x = data_.second->min.x;
          data_.first.y += 1;
          if (data_.first.y >= data_.second->min.y + data_.second->size.y) {
            data_.first.y = data_.second->min.y;
            data_.first.z += 1;
          }
        }
      }
      vector3<tile_coordinate> dereference()const { return data_.first; }

      explicit iterator(pair<vector3<tile_coordinate>, tile_bounding_box const*> data):data_(data){}
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

  iterator begin()const{ return iterator(make_pair(min, this)); }
  iterator end()const{ return iterator(make_pair(min + vector3<tile_coordinate>(0, 0, size.z), this)); }
};




/*

Whether tiles are water or not
 -- dictates, at one tile distance --
Whether water tiles are "sticky water" ("fewer than two adjacent tiles are air") or "free water" (at least two adjacent air tiles)
 -- dictates, at one tile distance --
Whether sticky water tiles are "interior water" ("every adjacent tile is sticky water") or "membrane water" ("at least one tile is not a sticky water tile")

This is a one-way dictation - the latter things don't affect the former things at all (until they cause water to actually move, anyway). It's not an exact dictation

*/

enum tile_contents {
  AIR = 0,
  ROCK,
  UNGROUPABLE_WATER,
  GROUPABLE_WATER,
  RUBBLE
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
  tile():data_(0){}


  // For tile based physics (e.g. water movement)
  // This is so that we don't have to search the collision-detector for relevant objects at every tile.
  bool there_is_an_object_here_that_affects_the_tile_based_physics()const { return data_ & there_is_an_object_here_that_affects_the_tile_based_physics_mask; }
  bool is_interior()const { return is_interior_bit_(); }
  tile_contents contents()const{ return (tile_contents)(data_ & contents_mask); }

  void set_contents(tile_contents new_contents) {
    data_ = (data_ & ~contents_mask) | (uint8_t(new_contents) & contents_mask);
  }
  void set_whether_there_is_an_object_here_that_affects_the_tile_based_physics(bool b) {
    data_ = (data_ & ~there_is_an_object_here_that_affects_the_tile_based_physics_mask)
         | (b ? there_is_an_object_here_that_affects_the_tile_based_physics_mask : uint8_t(0));
  }
  void set_interiorness(bool b) {
    data_ = (data_ & ~interior_bit_mask) | (b ? interior_bit_mask : uint8_t(0));
  }
private:
  static const uint8_t contents_mask = 0x7;
  static const uint8_t interior_bit_mask = (1<<3);
  static const uint8_t there_is_an_object_here_that_affects_the_tile_based_physics_mask = (1<<4);
  bool is_interior_bit_()const{ return data_ & interior_bit_mask; }
  uint8_t data_;
};


class world;

namespace the_decomposition_of_the_world_into_blocks_impl {
  class worldblock;
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
}

class tile_location {
public:
  //tile_location operator+(cardinal_direction dir)const;
  // Equivalent to operator+, except allowing you to specify the amount of realization needed.
  template<cardinal_direction Dir> tile_location get_neighbor(level_of_tile_realization_needed realineeded)const;

  tile_location get_neighbor_by_variable(cardinal_direction dir, level_of_tile_realization_needed realineeded)const;
  //tile_location operator-(cardinal_direction dir)const { return (*this)+(-dir); }
  bool operator==(tile_location const& other)const { return v_ == other.v_; }
  bool operator!=(tile_location const& other)const { return v_ != other.v_; }
  bool operator<(tile_location const& other)const { return v_ < other.v_; }
  inline tile const& stuff_at()const;
  vector3<tile_coordinate> const& coords()const { return v_; }
  friend inline std::ostream& operator<<(std::ostream& os, tile_location const& l) {
    return os << l.v_;
  }
  friend inline size_t hash_value(tile_location const& l) { return std::hash<vector3<tile_coordinate>>()(l.coords()); }
private:
  // This constructor should only be used when you know exactly what worldblock it's in!!
  tile_location(vector3<tile_coordinate> v, the_decomposition_of_the_world_into_blocks_impl::worldblock *wb):v_(v),wb_(wb){}

  // tile_physics.cpp is the only code permitted to modify tile contents
  friend tile& tile_physics_impl::mutable_stuff_at(tile_location const& loc);

  // null locations are occasionally permissible
  friend tile_location trivial_invalid_location();

  // worldblock creates tile_locations explicitly, and obviously knows
  // what worldblock they're in.
  friend class the_decomposition_of_the_world_into_blocks_impl::worldblock;

  vector3<tile_coordinate> v_;
  the_decomposition_of_the_world_into_blocks_impl::worldblock *wb_; // invariant: nonnull
};

inline tile_location trivial_invalid_location() { return tile_location(vector3<tile_coordinate>(0,0,0), nullptr); }

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

inline std::array<tile_location, num_cardinal_directions> get_all_neighbors(tile_location const& loc, level_of_tile_realization_needed realineeded = FULL_REALIZATION) {
  return std::array<tile_location, num_cardinal_directions>({{
    loc.get_neighbor<0>(realineeded),
    loc.get_neighbor<1>(realineeded),
    loc.get_neighbor<2>(realineeded),
    loc.get_neighbor<3>(realineeded),
    loc.get_neighbor<4>(realineeded),
    loc.get_neighbor<5>(realineeded)
  }});
}

inline std::array<tile_location, num_cardinal_directions> get_perpendicular_neighbors(tile_location const& loc, cardinal_direction dir, level_of_tile_realization_needed realineeded = FULL_REALIZATION) {
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
  typedef map<tile_coordinate, literally_random_access_removable_stuff<tile_location>> map_t;

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


// conversions between tile_coordinate and fine_scalar:

inline vector3<tile_coordinate> get_containing_tile_coordinates(vector3<fine_scalar> v) {
  return vector3<tile_coordinate>(
    tile_coordinate(v.x / tile_width),
    tile_coordinate(v.y / tile_width),
    tile_coordinate(v.z / tile_height)
  );
}
inline fine_scalar lower_bound_in_fine_units(tile_coordinate c, which_dimension_type which_coordinate) {
  if (which_coordinate == Z) return fine_scalar(c) * tile_height;
  else                       return fine_scalar(c) * tile_width;
}
inline fine_scalar upper_bound_in_fine_units(tile_coordinate c, which_dimension_type which_coordinate) {
  if (which_coordinate == Z) return fine_scalar(c) * tile_height + (tile_height-1);
  else                       return fine_scalar(c) * tile_width + (tile_width-1);
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
  return bounding_box(lower_bound_in_fine_units(v), upper_bound_in_fine_units(v));
}

inline bounding_box convert_to_fine_units(tile_bounding_box const& bb) {
  return bounding_box(
    lower_bound_in_fine_units(bb.min),
    lower_bound_in_fine_units(bb.min + bb.size) - vector3<fine_scalar>(1,1,1)
  );
}

inline tile_bounding_box get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(bounding_box const& bb) {
  tile_bounding_box result;
  result.min = get_containing_tile_coordinates(bb.min);
  result.size = get_containing_tile_coordinates(bb.max) - result.min + vector3<tile_coordinate>(1,1,1);
  return result;
}



//in header file because they're templates:

template<typename RNG> inline
tile_location literally_random_access_removable_tiles_by_height::get_and_erase_random_from_the_top(RNG& rng) {
  const map_t::iterator iter_at_top = boost::prior(data_.end());
  literally_random_access_removable_stuff<tile_location>& tiles_at_top = iter_at_top->second;
  const tile_location result = tiles_at_top.get_random(rng);
  tiles_at_top.erase(result);
  if (tiles_at_top.empty()) data_.erase(iter_at_top);
  return result;
}
template<typename RNG> inline
tile_location literally_random_access_removable_tiles_by_height::get_and_erase_random_from_the_bottom(RNG& rng) {
  const map_t::iterator iter_at_bottom = data_.begin();
  literally_random_access_removable_stuff<tile_location>& tiles_at_bottom = iter_at_bottom->second;
  const tile_location result = tiles_at_bottom.get_random(rng);
  tiles_at_bottom.erase(result);
  if (tiles_at_bottom.empty()) data_.erase(iter_at_bottom);
  return result;
}

#endif

