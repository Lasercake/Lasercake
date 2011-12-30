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

#ifndef LASERCAKE_WORLD_HPP__
#define LASERCAKE_WORLD_HPP__

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
#include <boost/functional/hash.hpp>
#include <boost/utility.hpp>
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include <inttypes.h>
#include <functional>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/variant.hpp>
#include <boost/variant/get.hpp>
#include <memory>

#include "utils.hpp"
#include "bbox_collision_detector.hpp"

using std::map;
using std::unordered_map;
using std::unordered_set;
using std::pair;
using std::make_pair;
using std::set;
using std::vector;
using std::array;
using std::shared_ptr;



typedef int32_t sub_tile_distance;
typedef uint32_t tile_coordinate;
typedef uint64_t high_resolution_coordinate;
typedef int64_t high_resolution_delta;
typedef int32_t tile_coordinate_signed_type;

inline high_resolution_coordinate convert_to_high_resolution(tile_coordinate c, size_t /*which_coordinate*/) {
  return high_resolution_coordinate(c) << 32;
}
inline vector3<high_resolution_coordinate> convert_to_high_resolution(vector3<tile_coordinate> v) {
  return vector3<high_resolution_coordinate>(
    convert_to_high_resolution(v[0], 0),
    convert_to_high_resolution(v[1], 1),
    convert_to_high_resolution(v[2], 2)
  );
}

inline vector3<tile_coordinate> get_tile_coordinates(vector3<high_resolution_coordinate> v) {
  return vector3<tile_coordinate>(
    v.x >> 32,
    v.y >> 32,
    v.z >> 32
  );
}

struct tile_bounding_box {
  vector3<tile_coordinate> min, size;
  tile_bounding_box(){}
  tile_bounding_box(vector3<tile_coordinate> coords):min(coords),size(1,1,1){}
  tile_bounding_box(vector3<tile_coordinate> min, vector3<tile_coordinate> size):min(min),size(size){}
  bool contains(vector3<tile_coordinate> v) {
    return (v.x >= min.x && v.x <= min.x + (size.x - 1) &&
            v.y >= min.y && v.y <= min.y + (size.y - 1) &&
            v.z >= min.z && v.z <= min.z + (size.z - 1));
  }
  
  class iterator : public boost::iterator_facade<iterator, pair<vector3<tile_coordinate>, tile_bounding_box*>, boost::forward_traversal_tag, vector3<tile_coordinate>>
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
    vector3<tile_coordinate> dereference()const { return data.first; }

    explicit iterator(pair<vector3<tile_coordinate>, tile_bounding_box*> data):data(data){}
    private:
    friend class boost::iterator_core_access;
      friend class tile_bounding_box;
      pair<vector3<tile_coordinate>, tile_bounding_box*> data;
      
  };
  
  iterator begin(){ return iterator(make_pair(min, this)); }
  iterator end(){ return iterator(make_pair(min + vector3<tile_coordinate>(0, 0, size.z), this)); }
};

struct high_resolution_bounding_box {
  high_resolution_bounding_box(){}
  high_resolution_bounding_box(vector3<high_resolution_coordinate> min, vector3<high_resolution_coordinate> size):min(min),size(size){}
  high_resolution_bounding_box(tile_bounding_box b):min(convert_to_high_resolution(b.min)),size(convert_to_high_resolution(b.size)){}
  
  vector3<high_resolution_coordinate> min, size;
  bool contains(vector3<high_resolution_coordinate> const& v) {
    return (v.x >= min.x && v.x <= min.x + (size.x - 1) &&
            v.y >= min.y && v.y <= min.y + (size.y - 1) &&
            v.z >= min.z && v.z <= min.z + (size.z - 1));
  }
};

inline tile_bounding_box convert_to_smallest_superset_at_tile_resolution(high_resolution_bounding_box const& bb) {
  const high_resolution_coordinate round_up = ((high_resolution_coordinate(1) << 32) - 1);
  tile_bounding_box result;
  result.min = vector3<tile_coordinate>(bb.min.x >> 32, bb.min.y >> 32, bb.min.z >> 32);
  result.size = vector3<tile_coordinate>(
    ((bb.min.x + bb.size.x + round_up) >> 32) - result.min.x,
    ((bb.min.y + bb.size.y + round_up) >> 32) - result.min.y,
    ((bb.min.z + bb.size.z + round_up) >> 32) - result.min.z
  );
  return result;
}




const tile_coordinate world_center_coord = (tile_coordinate(1) << (8*sizeof(tile_coordinate) - 1));
const vector3<tile_coordinate> world_center_coords(world_center_coord, world_center_coord, world_center_coord);

// TODO make these values more in line with "high_resolution" stuff
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

const vector3<sub_tile_distance> inactive_water_velocity(0, 0, -min_convincing_speed);


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
  
  // Constructing one of these in the default way yields the natural inactive state:
  water_movement_info():velocity(inactive_water_velocity),progress(0),blockage_amount_this_frame(0),computed_sticky_last_frame(false) { progress[cdir_zminus] = progress_necessary; }
  
  // This is not a general-purpose function. Only use it during the move-processing part of update_water.
  void get_completely_blocked(cardinal_direction dir);
  bool is_in_inactive_state()const;
};


class world;

//
class mobile_object {
public:
  // Something like this, later.
  // virtual something serialize = 0;
  virtual high_resolution_bounding_box bbox()const = 0;
  
  virtual void move_due_to_velocity() = 0;
  virtual void update(world *) {}
  
  mobile_object():velocity(0,0,0){}
  mobile_object(vector3<high_resolution_delta>):velocity(velocity){}

  vector3<high_resolution_delta> velocity;
};


namespace hacky_internals {
  class worldblock;
}

class tile_location;


class tile_location {
public:
  // this constructor should only be used when you know exactly what worldblock it's in!!
  // TODO: It's bad that it's both public AND doesn't assert that condition
  
  tile_location operator+(cardinal_direction dir)const;
  tile_location operator-(cardinal_direction dir)const { return (*this)+(-dir); }
  bool operator==(tile_location const& other)const { return v == other.v; }
  tile const& stuff_at()const;
  vector3<tile_coordinate> const& coords()const { return v; }
private:
  friend tile& mutable_stuff_at(tile_location const& loc);
  friend class hacky_internals::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  //friend class ztree_entry;
  tile_location(vector3<tile_coordinate> v, hacky_internals::worldblock *wb):v(v),wb(wb){}
  vector3<tile_coordinate> v;
  hacky_internals::worldblock *wb;
};

namespace std {
  template<> struct hash<tile_location> {
    inline size_t operator()(tile_location const& l) const {
      return hash<vector3<tile_coordinate> >()(l.coords());
    }
  };
}


typedef uint64_t mobile_object_identifier;
struct object_identifier {
  object_identifier(tile_location const& loc):data(loc){}
  object_identifier(mobile_object_identifier id):data(id){}
  tile_location const* get_tile_location()const { return boost::get<tile_location>(&data); }
  mobile_object_identifier const* get_mobile_object_identifier()const { return boost::get<mobile_object_identifier>(&data); }
  size_t hash()const {
    struct hash_visitor : public boost::static_visitor<size_t>
    {
      size_t operator()(tile_location const& i) const {
        return std::hash<tile_location>()(i);
      }
    
      size_t operator()(mobile_object_identifier i) const {
        return std::hash<mobile_object_identifier>()(i);
      }
    };
    return boost::apply_visitor( hash_visitor(), data );
  }
  bool operator==(object_identifier const& other)const { return data == other.data; }
private:
  boost::variant<tile_location, mobile_object_identifier> data;
};

namespace std {
  template<> struct hash<object_identifier> {
    inline size_t operator()(object_identifier const& id) const {
      return id.hash();
    }
  };
}

class world_collision_detector {
private:
  typedef bbox_collision_detector<object_identifier, 64, 3> internal_t;
  static internal_t::bounding_box convert_bb(high_resolution_bounding_box const& bb) {
    internal_t::bounding_box b;
    b.min[0] = bb.min.x;
    b.min[1] = bb.min.y;
    b.min[2] = bb.min.z;
    b.size[0] = bb.size.x;
    b.size[1] = bb.size.y;
    b.size[2] = bb.size.z;
    return b;
  }
public:
  world_collision_detector(){}
  
  void get_objects_overlapping(unordered_set<object_identifier>& results, high_resolution_bounding_box const& bb)const {
    detector.get_objects_overlapping(results, convert_bb(bb));
  }
  
  void insert(object_identifier id, high_resolution_bounding_box const& bb) {
    detector.insert(id, convert_bb(bb));
  }
  void erase(object_identifier id) {
    detector.erase(id);
  }
private:
  internal_t detector;
};


namespace hacky_internals {
  const int worldblock_dimension_exp = 4;
  typedef int worldblock_dimension_type;
  const worldblock_dimension_type worldblock_dimension = (1 << worldblock_dimension_exp);

  class worldblock {
public:
    worldblock():neighbors(nullptr),w(nullptr),inited(false){}
    worldblock& init_if_needed(world *w_, vector3<tile_coordinate> global_position_);
  
    // Only to be used in tile_location::stuff_at():
    tile& get_tile(vector3<tile_coordinate> global_coords);
  
    // Only to be used in tile_location::operator+(cardinal_direction):
    tile_location get_neighboring_loc(vector3<tile_coordinate> const& old_coords, cardinal_direction dir);
  
    tile_location get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, cardinal_direction dir);
    tile_location get_loc_guaranteed_to_be_in_this_block(vector3<tile_coordinate> coords);
private:
    std::array<std::array<std::array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles;
    value_for_each_cardinal_direction<worldblock*> neighbors;
    vector3<tile_coordinate> global_position; // the lowest x, y, and z among elements in this worldblock
    world *w;
    bool inited;
  };
}

class world_building_gun {
public:
  world_building_gun(world* w, tile_bounding_box bounds):w(w),bounds(bounds){}
  void operator()(tile_contents new_contents, vector3<tile_coordinate> locv);
private:
  world* w;
  tile_bounding_box bounds;
};


class world {
public:
  typedef std::function<void (world_building_gun, tile_bounding_box)> worldgen_function_t;
  typedef unordered_map<tile_location, water_movement_info> active_water_tiles_t;
  typedef unordered_map<mobile_object_identifier, shared_ptr<mobile_object> > mobile_objects_t;
  
  world(worldgen_function_t f):worldgen_function(f){}
  
  // The iterators are only valid until we activate any water tiles.
  boost::iterator_range<active_water_tiles_t::iterator> active_water_tiles_range() {
    return boost::make_iterator_range(active_water_tiles.begin(), active_water_tiles.end());
  }
  water_movement_info* get_active_water_tile(tile_location l) { return find_as_pointer(active_water_tiles, l); }
  
  // The iterators are only valid until we add any objects.
  boost::iterator_range<mobile_objects_t::iterator> mobile_objects_range() {
    return boost::make_iterator_range(mobile_objects.begin(), mobile_objects.end());
  }
  shared_ptr<mobile_object>* get_mobile_object(mobile_object_identifier id) { return find_as_pointer(mobile_objects, id); }
  
  tile_location make_tile_location(vector3<tile_coordinate> const& coords);
  
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_identifier> &results, high_resolution_bounding_box const& bounds) {
    ensure_space_exists(convert_to_smallest_superset_at_tile_resolution(bounds));
    things_exposed_to_collision.get_objects_overlapping(results, bounds);
  }
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_identifier> &results, tile_bounding_box const& bounds) {
    ensure_space_exists(bounds);
    things_exposed_to_collision.get_objects_overlapping(results, bounds); // implicitly converts to high res
  }
  
  void delete_rock(tile_location const& loc);
  void insert_rock(tile_location const& loc);
  void delete_water(tile_location const& loc);
  water_movement_info& insert_water(tile_location const& loc);
  void deactivate_water(tile_location const& loc);
  water_movement_info& activate_water(tile_location const& loc);
  
  void set_stickyness(tile_location const& loc, bool new_stickyness);
  
  void queue_creating_mobile_object(shared_ptr<mobile_object> obj) {
    mobile_objects_to_add.push_back(obj);
  }
  
  // Risky function: Don't use this while iterating through mobile objects (any other time is okay)
  void create_queued_objects() {
    for (shared_ptr<mobile_object> const& obj : mobile_objects_to_add) {
      mobile_objects.insert(make_pair(next_mobile_object_identifier++, obj));
    }
    mobile_objects_to_add.clear();
  }
  
  void add_laser_sfx(vector3<high_resolution_coordinate> laser_source, vector3<high_resolution_delta> laser_delta) {
    laser_sfxes.push_back(make_pair(laser_source, laser_delta));
  }
  std::vector<std::pair<vector3<high_resolution_coordinate>, vector3<high_resolution_delta>>> laser_sfxes;
  
private:
  friend class world_building_gun;
  friend class hacky_internals::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  
  // This map uses the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  unordered_map<vector3<tile_coordinate>, hacky_internals::worldblock> blocks; 
  
  active_water_tiles_t active_water_tiles;
  mobile_objects_t mobile_objects;
  mobile_object_identifier next_mobile_object_identifier;
  vector<shared_ptr<mobile_object>> mobile_objects_to_add;
  
  // This currently means all mobile objects, all water, and surface rock tiles. TODO I haven't actually implemented restricting to sufface rock yet
  world_collision_detector things_exposed_to_collision;
  
  // Worldgen functions TODO describe them
  worldgen_function_t worldgen_function;
  
 
  
  
  hacky_internals::worldblock* create_if_necessary_and_get_worldblock(vector3<tile_coordinate> position);
  void ensure_space_exists(tile_bounding_box space);
  
  void check_interiorness(tile_location const& loc);

  void something_changed_at(tile_location const& loc);
  
  // Used only by world_building_gun
  void insert_rock_bypassing_checks(tile_location const& loc);
  void insert_water_bypassing_checks(tile_location const& loc);
};

void update_water(world &w);
void update_mobile_objects(world &w);

inline void update(world &w) {
  w.create_queued_objects();
  w.laser_sfxes.clear();
  update_water(w);
  update_mobile_objects(w);
}

class laser_emitter : public mobile_object {
public:
  laser_emitter(vector3<high_resolution_coordinate> location, vector3<high_resolution_delta> facing):location(location),facing(facing){}
  virtual high_resolution_bounding_box bbox()const {
  // TODO define tile width???????
    return high_resolution_bounding_box(location - vector3<high_resolution_coordinate>(1ULL<<30, 1ULL<<30, 1ULL<<30), vector3<high_resolution_coordinate>(1ULL<<31, 1ULL<<31, 1ULL<<31)); // TODO lolhack
  }
  virtual void move_due_to_velocity() {
    // gravity. TODO hacky
    velocity += vector3<high_resolution_delta>(0, 0, -(1 << 22));
    
    location += velocity;
    //std::cerr << std::hex << location.z;
  }
  virtual void update(world *w) {
    struct laser_path_calculator {
      laser_path_calculator(laser_emitter *emi):emi(emi),current_laser_tile(get_tile_coordinates(emi->location)),facing_signs(sign(emi->facing.x), sign(emi->facing.y), sign(emi->facing.z)),facing_offs(facing_signs.x > 0, facing_signs.y > 0, facing_signs.z > 0){
        for (size_t i = 0; i < 3; ++i) {
          if (facing_signs[i] != 0) enter_next_cross(i);
        }
      }
      
      void enter_next_cross(size_t which_dimension) {
        coming_crosses.insert(make_pair(
          convert_to_high_resolution(
            (current_laser_tile + facing_offs)[which_dimension] - emi->location[which_dimension],
            which_dimension
          )
          / emi->facing[which_dimension],
          
          0
        ));
      }
      // returns which dimension we advanced
      size_t advance_to_next_location() {
        auto next_cross_iter = coming_crosses.begin();
        size_t which_dimension = next_cross_iter->second;
        coming_crosses.erase(next_cross_iter);
        current_laser_tile[which_dimension] += facing_signs[which_dimension];
        enter_next_cross(which_dimension);
        return which_dimension;
      }
      
      laser_emitter *emi;
      vector3<tile_coordinate> current_laser_tile;
      std::map<uint64_t, size_t> coming_crosses;
      vector3<high_resolution_delta> facing_signs;
      vector3<tile_coordinate_signed_type> facing_offs;
    };
    
    laser_path_calculator calc(this);
    
    //std::cerr << facing.x << " foo " << facing.y << " foo " << facing.z << "\n";
    const vector3<high_resolution_delta> max_laser_delta = ((facing * (100LL << 32)) / facing.magnitude()); // TODO fix overflow
    //std::cerr << max_laser_delta.x << " foo " << max_laser_delta.y << " foo " << max_laser_delta.z << " bar " << facing.magnitude() << " bar " << (facing * (100LL << 32)).x << "\n";
    const vector3<tile_coordinate> theoretical_end_tile = get_tile_coordinates(location + max_laser_delta);
    
    int which_dimension_we_last_advanced = -1;
    while(true) {
      unordered_set<object_identifier> possible_hits;
      w->collect_things_exposed_to_collision_intersecting(possible_hits, tile_bounding_box(calc.current_laser_tile));
      for (object_identifier const& id : possible_hits) {
        if (id.get_tile_location()) {
          // it's the entire tile, of course we hit it!
          vector3<high_resolution_delta> laser_delta;
          if (which_dimension_we_last_advanced == -1) {
            laser_delta = vector3<high_resolution_delta>(0,0,0);
          }
          else {
            vector3<tile_coordinate> hitloc_finding_hack = calc.current_laser_tile;
            hitloc_finding_hack[which_dimension_we_last_advanced] += calc.facing_offs[which_dimension_we_last_advanced];
            const high_resolution_delta laser_delta_in_facing_direction = (convert_to_high_resolution(hitloc_finding_hack)[which_dimension_we_last_advanced] - location[which_dimension_we_last_advanced]);
            laser_delta = (facing * laser_delta_in_facing_direction) / facing[which_dimension_we_last_advanced];
          }
          w->add_laser_sfx(location, laser_delta);
          return; // TODO handle what happens if there are mobile objects and/or multiple objects and/or whatever
        }
      }
      if (calc.current_laser_tile == theoretical_end_tile) {
        w->add_laser_sfx(location, max_laser_delta);
        return;
      }
      else which_dimension_we_last_advanced = calc.advance_to_next_location();
    }
  }
private:
  vector3<high_resolution_coordinate> location;
  vector3<high_resolution_delta> facing;
};

#endif

