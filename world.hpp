
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

struct axis_aligned_bounding_box {
  vector3<location_coordinate> min, size;
  bool contains(vector3<location_coordinate> v) {
    return (v.x >= min.x && v.x <= min.x + (size.x - 1) &&
            v.y >= min.y && v.y <= min.y + (size.y - 1) &&
            v.z >= min.z && v.z <= min.z + (size.z - 1));
  }
};

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
  
  tile_contents contents()const{ return data & contents_mask; }
  void set_contents(tile_contents new_contents)const{ data = (data & ~contents_mask) | (new_contents & contents_mask); }
  void set_water_stickyness(bool b)const{ data = (data & ~sticky_bit_mask) | (b ? sticky_bit_mask : 0); }
  void set_water_interiorness(bool b)const{ data = (data & ~interior_bit_mask) | (b ? interior_bit_mask : 0); }
private:
  static const contents_mask = 0x3;
  static const sticky_bit_mask = (1<<2);
  static const interior_bit_mask = (1<<3);
  bool is_sticky_bit()const{ return data & sticky_bit_mask; }
  bool is_interior_bit()const{ return data & interior_bit_mask; }
  uint8_t data;
};

struct water_movement_info {
  vector3<sub_tile_distance> velocity;
  value_for_each_cardinal_direction<sub_tile_distance> progress;
  value_for_each_cardinal_direction<sub_tile_distance> new_progress; // TODO don't put this here, it's temporary in update_water
  value_for_each_cardinal_direction<sub_tile_distance> blockage_amount_this_frame;
  
  // Constructing one of these in the default way yields the natural idle state:
  water_movement_info():velocity(idle_water_velocity),progress(0),new_progress(0),blockage_amount_this_frame(0){ progress[cdir_zminus] = progress_necessary; }
  
  // This is not a general-purpose function. Only use it during the move-processing part of update_water.
  void get_completely_blocked(cardinal_direction dir) {
    const sub_tile_distance dp = velocity.dot<sub_tile_distance>(dir.v);
    const sub_tile_distance blocked_velocity = dp - min_convincing_speed;
    if (blocked_velocity > 0) {
      velocity -= dir.v * blocked_velocity;
    }
  }
  
  bool can_deactivate()const {
    // TODO: does it make sense that we're ignoring the 1-frame-duration variable "blockage_amount_this_frame"?
    for (EACH_CARDINAL_DIRECTION(dir)) {
      if (dir.v.z < 0) {
        if (progress[dir] != progress_necessary) return false;
      }
      else {
        if (progress[dir] != 0) return false;
      }
    }
    return velocity == idle_water_velocity;
  }
};



class worldblock;

class location {
public:
  // this constructor should only be used when you know exactly what worldblock it's in!!
  // TODO: It's bad that it's both public AND doesn't assert that condition
  location(vector3<location_coordinate> v, worldblock *wb):v(v),wb(wb){}
  
  location operator+(cardinal_direction dir)const;
  location operator-(cardinal_direction dir)const { return (*this)+(-dir); }
  tile const& stuff_at()const;
  vector3<location_coordinate> const& coords()const { return v; }
private:
  friend tile& mutable_stuff_at(location const& loc);
  vector3<location_coordinate> v;
  worldblock *wb;
};

namespace std {
  template<> struct hash<location> {
    inline size_t operator()(location const& l) const {
      return hash<vector3<location_coordinate> >()(l.coords());
    }
  };
}




class world;

class world_building_gun {
public:
  operator()(tile_contents new_contents, vector3<location_coordinate> locv) {
    assert(bounds.contains(locv));
    if (new_contents == ROCK) {
      // TODO put the rock
    }
    else if (new_contents == WATER) {
      w->insert_water(w->make_location(locv));
    }
    else assert("YOU CAN ONLY PLACE ROCK AND WATER" && false);
  }
private:
  world* w;
  axis_aligned_bounding_box bounds;
};


class ztree_entry;

class world {
public:
  world(worldgen_function_t f):worldgen_function(f){}

  worldblock* create_if_necessary_and_get_worldblock(vector3<location_coordinate> position) {
    return &(blocks[position].init_if_needed(this));
  }
  
  // The iterators are only valid until we activate any tiles.
  boost::iterator_range<active_tiles_t::iterator> active_tiles_range() {
    return boost::make_iterator_range(active_tiles.begin(), active_tiles.end());
  }
  water_movement_info* get_active_tile(location l) { return find_as_pointer(active_tiles, l); }
  
  location make_location(vector3<location_coordinate> const& coords) {
    return location(coords, create_if_necessary_and_get_worldblock(vector3<location_coordinate>(coords.x & ~(worldblock_dimension-1), coords.y & ~(worldblock_dimension-1), coords.z & ~(worldblock_dimension-1))));
  }
  
  void collect_tiles_that_contain_anything_near(unordered_set<location> &results, location center, int radius);
  
  void delete_water(location loc);
  water_movement_info& insert_water(location loc);
  void deactivate_water(location loc);
  water_movement_info& activate_water(location loc);
  
private:
  unordered_map<vector3<location_coordinate>, worldblock> blocks; // using the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  
  typedef unordered_map<location, water_movement_info> active_tiles_t;
  active_tiles_t active_tiles;
  set<ztree_entry> tiles_that_contain_anything;
  
  // Worldgen functions TODO describe them here
  typedef std::function<void (world_building_gun, axis_aligned_bounding_box)> worldgen_function_t;
  worldgen_function_t worldgen_function;
  
  // these functions update the tiles' stickyness/interiorness caches
  void check_stickyness(location loc);
  void check_interiorness(location loc);
};


#endif

