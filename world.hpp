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
#include "polygon_collision_detection.hpp"
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


typedef uint32_t tile_coordinate;
typedef int64_t fine_scalar; // Fine as opposed to coarse, that is.
typedef int32_t tile_coordinate_signed_type;
typedef int32_t sub_tile_distance; // We can fit it within 32 bits, so we might as well do faster math

const fine_scalar tile_width = (fine_scalar(1) << 10);
const fine_scalar tile_height = (fine_scalar(1) << 10) / 5 + 1;
const vector3<fine_scalar> tile_size(tile_width, tile_width, tile_height);

const fine_scalar velocity_scale_factor = (fine_scalar(1) << 6);

const tile_coordinate world_center_coord = (tile_coordinate(1) << (8*sizeof(tile_coordinate) - 1));
const vector3<tile_coordinate> world_center_coords(world_center_coord, world_center_coord, world_center_coord);

const sub_tile_distance min_convincing_speed = 20 * velocity_scale_factor;
const sub_tile_distance gravity_acceleration_magnitude = 1 * velocity_scale_factor;
const vector3<sub_tile_distance> gravity_acceleration(0, 0, -gravity_acceleration_magnitude); // in mini-units per frame squared
const sub_tile_distance friction_amount = (velocity_scale_factor * 3 / 5);

// as in 1 + d2 (except with the random based at zero, but who cares)
const sub_tile_distance pressure_motion_factor = 16 * velocity_scale_factor;
const sub_tile_distance pressure_motion_factor_random = 8 * velocity_scale_factor;
const sub_tile_distance extra_downward_speed_for_sticky_water = 20 * velocity_scale_factor;

const sub_tile_distance air_resistance_constant = (10000 * velocity_scale_factor * velocity_scale_factor);
const sub_tile_distance idle_progress_reduction_rate = 20 * velocity_scale_factor;
const sub_tile_distance sticky_water_velocity_reduction_rate = 1 * velocity_scale_factor;

const vector3<sub_tile_distance> inactive_water_velocity(0, 0, -min_convincing_speed);

const fine_scalar max_object_speed_through_water = tile_width * velocity_scale_factor / 16;

inline vector3<tile_coordinate> get_containing_tile_coordinates(vector3<fine_scalar> v) {
  return vector3<tile_coordinate>(
    tile_coordinate(v.x / tile_width),
    tile_coordinate(v.y / tile_width),
    tile_coordinate(v.z / tile_height)
  );
}
inline fine_scalar lower_bound_in_fine_units(tile_coordinate c, int which_coordinate) {
  if (which_coordinate == 2) return c * tile_height;
  else                       return c * tile_width;
}
inline vector3<fine_scalar> lower_bound_in_fine_units(vector3<tile_coordinate> v) {
  return vector3<fine_scalar>(
    lower_bound_in_fine_units(v[0], 0),
    lower_bound_in_fine_units(v[1], 1),
    lower_bound_in_fine_units(v[2], 2)
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

inline bounding_box convert_to_fine_units(tile_bounding_box const& bb) {
  return bounding_box(
    lower_bound_in_fine_units(bb.min),
    lower_bound_in_fine_units(bb.min + bb.size) - vector3<fine_scalar>(1,1,1)
  );
}

inline tile_bounding_box convert_to_smallest_superset_at_tile_resolution(bounding_box const& bb) {
  tile_bounding_box result;
  result.min = get_containing_tile_coordinates(bb.min);
  result.size = get_containing_tile_coordinates(bb.max) + vector3<tile_coordinate>(1,1,1) - result.min;
  return result;
}

inline shape tile_shape(vector3<tile_coordinate> tile) {
  return shape(convert_to_fine_units(tile_bounding_box(tile)));
}



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
  bool is_interior_rock ()const{ return contents() == ROCK  &&                      is_interior_bit(); }
  
  // For tile based physics (e.g. water movement)
  // This is so that we don't have to search the collision-detector for relevant objects at every tile.
  bool there_is_an_object_here_that_affects_the_tile_based_physics()const { return data & there_is_an_object_here_that_affects_the_tile_based_physics_mask; }
  tile_contents contents()const{ return (tile_contents)(data & contents_mask); }
  
  void set_contents(tile_contents new_contents){ data = (data & ~contents_mask) | (uint8_t(new_contents) & contents_mask); }
  void set_whether_there_is_an_object_here_that_affects_the_tile_based_physics(bool b){ data = (data & ~there_is_an_object_here_that_affects_the_tile_based_physics_mask) | (b ? there_is_an_object_here_that_affects_the_tile_based_physics_mask : uint8_t(0)); }
  void set_water_stickyness(bool b){ data = (data & ~sticky_bit_mask) | (b ? sticky_bit_mask : uint8_t(0)); }
  void set_water_interiorness(bool b){ assert(contents() == WATER); data = (data & ~interior_bit_mask) | (b ? interior_bit_mask : uint8_t(0)); }
  void set_rock_interiorness(bool b){ assert(contents() == ROCK); data = (data & ~interior_bit_mask) | (b ? interior_bit_mask : uint8_t(0)); }
private:
  static const uint8_t contents_mask = 0x3;
  static const uint8_t sticky_bit_mask = (1<<2);
  static const uint8_t interior_bit_mask = (1<<3);
  static const uint8_t there_is_an_object_here_that_affects_the_tile_based_physics_mask = (1<<4);
  bool is_sticky_bit()const{ return data & sticky_bit_mask; }
  bool is_interior_bit()const{ return data & interior_bit_mask; }
  uint8_t data;
};

// "progress" is measured in the smaller, velocity units.
inline sub_tile_distance progress_necessary(cardinal_direction dir) {
  return tile_size[dir.which_dimension()] * velocity_scale_factor;
}

struct water_movement_info {
  vector3<sub_tile_distance> velocity;
  value_for_each_cardinal_direction<sub_tile_distance> progress;
  value_for_each_cardinal_direction<sub_tile_distance> blockage_amount_this_frame;
  bool computed_sticky_last_frame;
  
  // Constructing one of these in the default way yields the natural inactive state:
  water_movement_info():velocity(inactive_water_velocity),progress(0),blockage_amount_this_frame(0),computed_sticky_last_frame(false) { progress[cdir_zminus] = progress_necessary(cdir_zminus); }
  
  // This is not a general-purpose function. Only use it during the move-processing part of update_water.
  void get_completely_blocked(cardinal_direction dir);
  bool is_in_inactive_state()const;
};


class world;

namespace hacky_internals {
  class worldblock;
}

enum level_of_tile_realization_needed {
  COMPLETELY_IMAGINARY = 0,
  CONTENTS_ONLY = 1,
  CONTENTS_AND_STICKYNESS_ONLY = 2,
  FULL_REALIZATION = 3
};

class tile_location {
public:
  // this constructor should only be used when you know exactly what worldblock it's in!!
  // TODO: It's bad that it's both public AND doesn't assert that condition
  
  tile_location operator+(cardinal_direction dir)const;
  // Equivalent to operator+, except allowing you to specify the amount of realization needed.
  tile_location get_neighbor(cardinal_direction dir, level_of_tile_realization_needed realineeded)const;
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

bool should_be_sticky(tile_location loc);


typedef uint64_t object_identifier;
struct object_or_tile_identifier {
  object_or_tile_identifier(tile_location const& loc):data(loc){}
  object_or_tile_identifier(object_identifier id):data(id){}
  tile_location const* get_tile_location()const { return boost::get<tile_location>(&data); }
  object_identifier const* get_object_identifier()const { return boost::get<object_identifier>(&data); }
  size_t hash()const {
    struct hash_visitor : public boost::static_visitor<size_t>
    {
      size_t operator()(tile_location const& i) const {
        return std::hash<tile_location>()(i);
      }
    
      size_t operator()(object_identifier i) const {
        return std::hash<object_identifier>()(i);
      }
    };
    return boost::apply_visitor( hash_visitor(), data );
  }
  bool operator==(object_or_tile_identifier const& other)const { return data == other.data; }
private:
  boost::variant<tile_location, object_identifier> data;
};

namespace std {
  template<> struct hash<object_or_tile_identifier> {
    inline size_t operator()(object_or_tile_identifier const& id) const {
      return id.hash();
    }
  };
}
class object {
public:
  virtual shape get_initial_personal_space_shape()const = 0;
  virtual shape get_initial_detail_shape()const = 0;
  /*
private:
  virtual void this_virtual_function_makes_this_class_virtual_which_we_need_in_order_to_dynamic_cast_it(){};*/
};

class mobile_object : virtual public object {
public:
  //virtual void move_due_to_velocity() = 0;

  mobile_object():velocity(0,0,0){}
  mobile_object(vector3<fine_scalar>):velocity(velocity){}
  vector3<fine_scalar> velocity;
};

class tile_aligned_object : virtual public object {
public:
  
};

class autonomous_object : virtual public object {
public:
  virtual void update(world &w, object_identifier my_id) = 0;
};


class world_collision_detector {
private:
  typedef bbox_collision_detector<object_or_tile_identifier, 64, 3> internal_t;
  static internal_t::bounding_box convert_bb(bounding_box const& bb) {
    assert(bb.is_anywhere);
    internal_t::bounding_box b;
    b.min[0] = bb.min.x;
    b.min[1] = bb.min.y;
    b.min[2] = bb.min.z;
    b.size[0] = bb.max.x + 1 - bb.min.x;
    b.size[1] = bb.max.y + 1 - bb.min.y;
    b.size[2] = bb.max.z + 1 - bb.min.z;
    return b;
  }
public:
  world_collision_detector(){}
  
  void get_objects_overlapping(unordered_set<object_or_tile_identifier>& results, bounding_box const& bb)const {
    detector.get_objects_overlapping(results, convert_bb(bb));
  }
  
  void insert(object_or_tile_identifier id, bounding_box const& bb) {
    detector.insert(id, convert_bb(bb));
  }
  bool erase(object_or_tile_identifier id) {
    return detector.erase(id);
  }
  bool exists(object_or_tile_identifier id) {
    return detector.exists(id);
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
    worldblock():neighbors(nullptr),w(nullptr),current_tile_realization(COMPLETELY_IMAGINARY){}
    worldblock& ensure_realization(world *w_, vector3<tile_coordinate> global_position_, level_of_tile_realization_needed realineeded);
  
    // Only to be used in tile_location::stuff_at():
    tile& get_tile(vector3<tile_coordinate> global_coords);
  
    // Only to be used in tile_location::operator+(cardinal_direction) and tile_location::get_neighbor:
    tile_location get_neighboring_loc(vector3<tile_coordinate> const& old_coords, cardinal_direction dir, level_of_tile_realization_needed realineeded);
  
    tile_location get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, cardinal_direction dir, level_of_tile_realization_needed realineeded);
    tile_location get_loc_guaranteed_to_be_in_this_block(vector3<tile_coordinate> coords);
private:
    std::array<std::array<std::array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles;
    value_for_each_cardinal_direction<worldblock*> neighbors;
    vector3<tile_coordinate> global_position; // the lowest x, y, and z among elements in this worldblock
    world *w;
    level_of_tile_realization_needed current_tile_realization;
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


typedef std::function<void (world_building_gun, tile_bounding_box)> worldgen_function_t;
typedef unordered_map<tile_location, water_movement_info> active_water_tiles_t;
typedef unordered_map<object_identifier, shape> object_shapes_t;
template<typename ObjectSubtype>
struct objects_map {
  typedef unordered_map<object_identifier, shared_ptr<ObjectSubtype>> type;
};

void update_water(world &w);

class world {
public:
  world(worldgen_function_t f):next_object_identifier(1),worldgen_function(f){}
  
  void update_moving_objects();

  inline void update() {
    laser_sfxes.clear();
    update_water(*this); // TODO update_water to be a member
    for (auto &obj : autonomously_active_objects) obj.second->update(*this, obj.first);
    update_moving_objects();
  }
  
  // TODO replace these with const accessor functions...?
  // The iterators are only valid until we activate any water tiles.
  water_movement_info* get_active_water_tile(tile_location l) { return find_as_pointer(active_water_tiles, l); }
  boost::iterator_range<active_water_tiles_t::iterator> active_water_tiles_range() {
    return boost::make_iterator_range(active_water_tiles.begin(), active_water_tiles.end());
  }
  
  // The iterators are only valid until we add any objects.
  shared_ptr<object>* get_object(object_identifier id) { return find_as_pointer(objects, id); }
  /*boost::iterator_range<mobile_objects_map<mobile_object>::iterator> mobile_objects_range() {
    return boost::make_iterator_range(mobile_objects.begin(), mobile_objects.end());
  }*/
  boost::iterator_range<objects_map<mobile_object>::type::iterator> moving_objects_range() {
    return boost::make_iterator_range(moving_objects.begin(), moving_objects.end());
  }
  
  tile_location make_tile_location(vector3<tile_coordinate> const& coords, level_of_tile_realization_needed realineeded);
  
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_or_tile_identifier> &results, bounding_box const& bounds) {
    ensure_realization_of_space(convert_to_smallest_superset_at_tile_resolution(bounds), FULL_REALIZATION);
    things_exposed_to_collision.get_objects_overlapping(results, bounds);
  }
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_or_tile_identifier> &results, tile_bounding_box const& bounds) {
    ensure_realization_of_space(bounds, FULL_REALIZATION);
    things_exposed_to_collision.get_objects_overlapping(results, convert_to_fine_units(bounds));
  }
  
  void delete_rock(tile_location const& loc);
  void insert_rock(tile_location const& loc);
  void delete_water(tile_location const& loc);
  water_movement_info& insert_water(tile_location const& loc);
  void deactivate_water(tile_location const& loc);
  water_movement_info& activate_water(tile_location const& loc);
  
  void set_stickyness(tile_location const& loc, bool new_stickyness);
  
  bool try_create_object(shared_ptr<object> obj) {
    // TODO: fail if there's something in the way
    object_identifier id = next_object_identifier++;
    objects.insert(make_pair(id, obj));
    bounding_box b; // TODO: in mobile_objects.cpp, include detail_shape in at least the final box left in the ztree
    object_personal_space_shapes[id] = obj->get_initial_personal_space_shape();
    b.combine_with(object_personal_space_shapes[id].bounds());
    object_detail_shapes[id] = obj->get_initial_detail_shape();
    b.combine_with(object_detail_shapes[id].bounds());
    things_exposed_to_collision.insert(id, b);
    if(shared_ptr<mobile_object> m = std::dynamic_pointer_cast<mobile_object>(obj)) {
      moving_objects.insert(make_pair(id, m));
    }
    // TODO: don't do this if you're in the middle of updating autonomous objects
    if(shared_ptr<autonomous_object> m = std::dynamic_pointer_cast<autonomous_object>(obj)) {
      autonomously_active_objects.insert(make_pair(id, m));
    }
    return true;
  }
  
  // If objects overlap with the new position, returns their IDs. If not, changes the shape and returns an empty set.
  unordered_set<object_or_tile_identifier> try_to_change_personal_space_shape(object_identifier id, shape const& new_shape);
  // Objects can't fail to change their detail shape, but it may cause effects (like blocking a laser beam)
  void change_detail_shape(object_identifier id, shape const& new_shape);
  
  void add_laser_sfx(vector3<fine_scalar> laser_source, vector3<fine_scalar> laser_delta) {
    laser_sfxes.push_back(make_pair(laser_source, laser_delta));
  }
  std::vector<std::pair<vector3<fine_scalar>, vector3<fine_scalar>>> laser_sfxes;
  
  
  objects_map<object>::type const& get_objects()const { return objects; }
  object_shapes_t const& get_object_personal_space_shapes()const { return object_personal_space_shapes; }
private:
  friend class world_building_gun;
  friend class hacky_internals::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  
  // This map uses the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  unordered_map<vector3<tile_coordinate>, hacky_internals::worldblock> blocks; 
  
  active_water_tiles_t active_water_tiles;
  objects_map<object>::type objects;
  objects_map<mobile_object>::type moving_objects;
  objects_map<autonomous_object>::type autonomously_active_objects;
  
  object_identifier next_object_identifier;
  vector<shared_ptr<object>> objects_to_add;
  object_shapes_t object_personal_space_shapes;
  object_shapes_t object_detail_shapes;
  
  // This currently means all mobile objects, all water, and surface rock tiles. TODO I haven't actually implemented restricting to sufface rock yet
  world_collision_detector things_exposed_to_collision;
  
  // Worldgen functions TODO describe them
  worldgen_function_t worldgen_function;
  
  
  
  tile_location make_barely_existing_tile_location(vector3<tile_coordinate> const& coords);
  hacky_internals::worldblock* ensure_realization_of_and_get_worldblock(vector3<tile_coordinate> position, level_of_tile_realization_needed realineeded);
  void ensure_realization_of_space(tile_bounding_box space, level_of_tile_realization_needed realineeded);
  
  void check_interiorness(tile_location const& loc);
  void check_exposure_to_collision(tile_location const& loc);

  void something_changed_at(tile_location const& loc);
  
  // Used only by world_building_gun
  void insert_rock_bypassing_checks(tile_location const& loc);
  void insert_water_bypassing_checks(tile_location const& loc);
};

class robot : public mobile_object, public autonomous_object {
public:
  robot(vector3<fine_scalar> location, vector3<fine_scalar> facing):location(location),facing(facing){}
  
  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;
  
  virtual void update(world &w, object_identifier my_id);
  vector3<fine_scalar> get_facing()const { return facing; }
private:
  vector3<fine_scalar> location;
  vector3<fine_scalar> facing;
};

class laser_emitter : public mobile_object, public autonomous_object {
public:
  laser_emitter(vector3<fine_scalar> location, vector3<fine_scalar> facing):location(location),facing(facing){}
  
  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;
  
  virtual void update(world &w, object_identifier id);
private:
  vector3<fine_scalar> location;
  vector3<fine_scalar> facing;
};

#endif

