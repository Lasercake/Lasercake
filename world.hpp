
#ifndef ELISAACWATER_WORLD_HPP__
#define ELISAACWATER_WORLD_HPP__

#include <vector>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <map>
#include <set>
#include <array>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <boost/functional/hash.hpp>
#include <boost/utility.hpp>
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include <inttypes.h>
#include <functional>
#include <boost/iterator/iterator_facade.hpp>

#include "utils.hpp"

using std::map;
using std::unordered_map;
using std::unordered_set;
using std::pair;
using std::make_pair;
using std::set;
using std::vector;
using std::array;
using std::bitset;


typedef int32_t sub_tile_distance;
typedef uint32_t location_coordinate;
typedef int32_t location_coordinate_signed_type;

struct axis_aligned_bounding_box {
  vector3<location_coordinate> min, size;
  bool contains(vector3<location_coordinate> v) {
    return (v.x >= min.x && v.x <= min.x + (size.x - 1) &&
            v.y >= min.y && v.y <= min.y + (size.y - 1) &&
            v.z >= min.z && v.z <= min.z + (size.z - 1));
  }
  
  class iterator : public boost::iterator_facade<iterator, pair<vector3<location_coordinate>, axis_aligned_bounding_box*>, boost::forward_traversal_tag, vector3<location_coordinate>>
  {
    public:
      iterator(){}
    bool equal(iterator other)const { return data == other.data; }
    void increment() {
      data.first.x += 1;
      if (data.first.x >= data.second->min.x + data.second->size.x) {
        data.first.x = data.second->min.x;
        data.first.y += 1;
        if (data.first.y >= data.second->min.y + data.second->size.y) {
          data.first.y = data.second->min.y;
          data.first.z += 1;
        }
      }
    }
    vector3<location_coordinate> dereference()const { return data.first; }

    explicit iterator(pair<vector3<location_coordinate>, axis_aligned_bounding_box*> data):data(data){}
    private:
    friend class boost::iterator_core_access;
      friend class axis_aligned_bounding_box;
      pair<vector3<location_coordinate>, axis_aligned_bounding_box*> data;
      
  };
  
  iterator begin(){ return iterator(make_pair(min, this)); }
  iterator end(){ return iterator(make_pair(min + vector3<location_coordinate>(0, 0, size.z), this)); }
};

const location_coordinate world_center_coord = (location_coordinate(1) << (8*sizeof(location_coordinate) - 1));
const vector3<location_coordinate> world_center_coords(world_center_coord, world_center_coord, world_center_coord);

const sub_tile_distance precision_factor = 100;
const sub_tile_distance progress_necessary = 5000 * precision_factor; // loosely speaking, the conversion factor between mini-units and entire tiles
const sub_tile_distance min_convincing_speed = 100 * precision_factor;
const vector3<sub_tile_distance> gravity_acceleration(0, 0, -5*precision_factor); // in mini-units per frame squared
const sub_tile_distance friction_amount = 3 * precision_factor;

// as in 1 + d2 (except with the random based at zero, but who cares)
const sub_tile_distance pressure_motion_factor = 80 * precision_factor;
const sub_tile_distance pressure_motion_factor_random = 40 * precision_factor;
const sub_tile_distance extra_downward_speed_for_sticky_water = 100 * precision_factor;

const sub_tile_distance air_resistance_constant = (200000 * precision_factor * precision_factor);
const sub_tile_distance idle_progress_reduction_rate = 100 * precision_factor;
const sub_tile_distance sticky_water_velocity_reduction_rate = 5*precision_factor;

const vector3<sub_tile_distance> idle_water_velocity(0, 0, -min_convincing_speed);


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
  WATER
  // it matters that there are no more than four of these, currently
};

class tile {
public:
  tile():data(0){}

  bool is_sticky_water  ()const{ return contents() == WATER &&  is_sticky_bit()                      ; }
  bool is_free_water    ()const{ return contents() == WATER && !is_sticky_bit()                      ; }
  bool is_interior_water()const{ return contents() == WATER &&  is_sticky_bit() &&  is_interior_bit(); }
  bool is_membrane_water()const{ return contents() == WATER &&  is_sticky_bit() && !is_interior_bit(); }
  
  tile_contents contents()const{ return (tile_contents)(data & contents_mask); }
  
  void set_contents(tile_contents new_contents){ data = (data & ~contents_mask) | (new_contents & contents_mask); }
  void set_water_stickyness(bool b){ data = (data & ~sticky_bit_mask) | (b ? sticky_bit_mask : 0); }
  void set_water_interiorness(bool b){ data = (data & ~interior_bit_mask) | (b ? interior_bit_mask : 0); }
private:
  static const uint8_t contents_mask = 0x3;
  static const uint8_t sticky_bit_mask = (1<<2);
  static const uint8_t interior_bit_mask = (1<<3);
  bool is_sticky_bit()const{ return data & sticky_bit_mask; }
  bool is_interior_bit()const{ return data & interior_bit_mask; }
  uint8_t data;
};

struct water_movement_info {
  vector3<sub_tile_distance> velocity;
  value_for_each_cardinal_direction<sub_tile_distance> progress;
  value_for_each_cardinal_direction<sub_tile_distance> blockage_amount_this_frame;
  bool computed_sticky_last_frame;
  
  // Constructing one of these in the default way yields the natural idle state:
  water_movement_info():velocity(idle_water_velocity),progress(0),blockage_amount_this_frame(0),computed_sticky_last_frame(false) { progress[cdir_zminus] = progress_necessary; }
  
  // This is not a general-purpose function. Only use it during the move-processing part of update_water.
  void get_completely_blocked(cardinal_direction dir);
  bool is_in_idle_state()const;
};


namespace hacky_internals {
  class worldblock;
}

class location;

class ztree_entry {
private:
  vector3<location_coordinate> locv;
  hacky_internals::worldblock *worldblock_if_known;
  std::array<location_coordinate, 3> interleaved_bits;
  static const size_t bits_in_loc_coord = 8*sizeof(location_coordinate);
  void set_bit(size_t idx);
  void set_bits();
public:
  location loc()const;
  
  ztree_entry(location const& loc);
  ztree_entry(vector3<location_coordinate> const& locv);
  
  bool operator==(ztree_entry const& other)const;
  bool operator<(ztree_entry const& other)const;
};

class location {
public:
  // this constructor should only be used when you know exactly what worldblock it's in!!
  // TODO: It's bad that it's both public AND doesn't assert that condition
  
  location operator+(cardinal_direction dir)const;
  location operator-(cardinal_direction dir)const { return (*this)+(-dir); }
  bool operator==(location const& other)const { return v == other.v; }
  tile const& stuff_at()const;
  vector3<location_coordinate> const& coords()const { return v; }
private:
  friend tile& mutable_stuff_at(location const& loc);
  friend class hacky_internals::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  friend class ztree_entry;
  location(vector3<location_coordinate> v, hacky_internals::worldblock *wb):v(v),wb(wb){}
  vector3<location_coordinate> v;
  hacky_internals::worldblock *wb;
};

namespace std {
  template<> struct hash<location> {
    inline size_t operator()(location const& l) const {
      return hash<vector3<location_coordinate> >()(l.coords());
    }
  };
}




class world;

namespace hacky_internals {
  const int worldblock_dimension_exp = 4;
  typedef int worldblock_dimension_type;
  const worldblock_dimension_type worldblock_dimension = (1 << worldblock_dimension_exp);

  class worldblock {
public:
    worldblock():neighbors(nullptr),w(nullptr),inited(false){}
    worldblock& init_if_needed(world *w_, vector3<location_coordinate> global_position_);
  
    // Only to be used in location::stuff_at():
    tile& get_tile(vector3<location_coordinate> global_coords);
  
    // Only to be used in location::operator+(cardinal_direction):
    location get_neighboring_loc(vector3<location_coordinate> const& old_coords, cardinal_direction dir);
  
    location get_loc_across_boundary(vector3<location_coordinate> const& new_coords, cardinal_direction dir);
    location get_loc_guaranteed_to_be_in_this_block(vector3<location_coordinate> coords);
private:
    std::array<std::array<std::array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles;
    value_for_each_cardinal_direction<worldblock*> neighbors;
    vector3<location_coordinate> global_position; // the lowest x, y, and z among elements in this worldblock
    world *w;
    bool inited;
  };
}

class world_building_gun {
public:
  world_building_gun(world* w, axis_aligned_bounding_box bounds):w(w),bounds(bounds){}
  void operator()(tile_contents new_contents, vector3<location_coordinate> locv);
private:
  world* w;
  axis_aligned_bounding_box bounds;
};


class world {
public:
  friend class world_building_gun;
  typedef std::function<void (world_building_gun, axis_aligned_bounding_box)> worldgen_function_t;
  typedef unordered_map<location, water_movement_info> active_water_tiles_t;
  
  world(worldgen_function_t f):worldgen_function(f){}
  
  // The iterators are only valid until we activate any tiles.
  boost::iterator_range<active_water_tiles_t::iterator> active_water_tiles_range() {
    return boost::make_iterator_range(active_water_tiles.begin(), active_water_tiles.end());
  }
  water_movement_info* get_active_water_tile(location l) { return find_as_pointer(active_water_tiles, l); }
  
  location make_location(vector3<location_coordinate> const& coords);
  
  void collect_tiles_that_contain_anything_near(unordered_set<location> &results, axis_aligned_bounding_box bounds);
  
  void delete_rock(location const& loc);
  void insert_rock(location const& loc);
  void delete_water(location const& loc);
  water_movement_info& insert_water(location const& loc);
  void deactivate_water(location const& loc);
  water_movement_info& activate_water(location const& loc);
  
  void set_stickyness(location const& loc, bool new_stickyness);
  
private:
  unordered_map<vector3<location_coordinate>, hacky_internals::worldblock> blocks; // using the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  hacky_internals::worldblock* create_if_necessary_and_get_worldblock(vector3<location_coordinate> position);
  void ensure_space_exists(axis_aligned_bounding_box space);
  friend class hacky_internals::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  
  active_water_tiles_t active_water_tiles;
  set<ztree_entry> tiles_that_contain_anything;
  
  // Worldgen functions TODO describe them here
  worldgen_function_t worldgen_function;
  
  void check_interiorness(location const& loc);

  void something_changed_at(location const& loc);
  
  // Used only by world_building_gun
  void insert_rock_bypassing_checks(location const& loc);
  void insert_water_bypassing_checks(location const& loc);
};

void update_water(world &w);

#endif

