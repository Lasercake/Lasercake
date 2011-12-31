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
const fine_scalar tile_height = (fine_scalar(1) << 10);
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
  
  // For tile based physics (e.g. water movement)
  // This is so that we don't have to search the collision-detector for relevant objects at every tile.
  bool there_is_an_object_here_that_affects_the_tile_based_physics()const { return data & there_is_an_object_here_that_affects_the_tile_based_physics_mask; }
  tile_contents contents()const{ return (tile_contents)(data & contents_mask); }
  
  void set_contents(tile_contents new_contents){ data = (data & ~contents_mask) | (uint8_t(new_contents) & contents_mask); }
  void set_whether_there_is_an_object_here_that_affects_the_tile_based_physics(bool b){ data = (data & ~there_is_an_object_here_that_affects_the_tile_based_physics_mask) | (b ? there_is_an_object_here_that_affects_the_tile_based_physics_mask : uint8_t(0)); }
  void set_water_stickyness(bool b){ data = (data & ~sticky_bit_mask) | (b ? sticky_bit_mask : uint8_t(0)); }
  void set_water_interiorness(bool b){ data = (data & ~interior_bit_mask) | (b ? interior_bit_mask : uint8_t(0)); }
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

class object {
private:
  virtual void this_virtual_function_makes_this_class_virtual_which_we_need_in_order_to_dynamic_cast_it(){};
};

class mobile_object : virtual public object {
public:
  //virtual void move_due_to_velocity() = 0;

  mobile_object():velocity(0,0,0){}
  mobile_object(vector3<fine_scalar>):velocity(velocity){}
  vector3<fine_scalar> velocity;
};

class tile_aligned_object {
public:
  
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


class laser_emitter;

typedef std::function<void (world_building_gun, tile_bounding_box)> worldgen_function_t;
typedef unordered_map<tile_location, water_movement_info> active_water_tiles_t;
typedef unordered_map<object_identifier, shape> object_shapes_t;
template<typename ObjectSubtype>
struct objects_map {
  typedef unordered_map<object_identifier, shared_ptr<ObjectSubtype>> type;
};

class world {
public:
  world(worldgen_function_t f):next_object_identifier(1),worldgen_function(f){}
  
  void update_moving_objects();
  
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
  boost::iterator_range<objects_map<laser_emitter>::type::iterator> active_lasers_range() {
    return boost::make_iterator_range(active_lasers.begin(), active_lasers.end());
  }
  
  tile_location make_tile_location(vector3<tile_coordinate> const& coords);
  
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_or_tile_identifier> &results, bounding_box const& bounds) {
    ensure_space_exists(convert_to_smallest_superset_at_tile_resolution(bounds));
    things_exposed_to_collision.get_objects_overlapping(results, bounds);
  }
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_or_tile_identifier> &results, tile_bounding_box const& bounds) {
    ensure_space_exists(bounds);
    things_exposed_to_collision.get_objects_overlapping(results, convert_to_fine_units(bounds));
  }
  
  void delete_rock(tile_location const& loc);
  void insert_rock(tile_location const& loc);
  void delete_water(tile_location const& loc);
  water_movement_info& insert_water(tile_location const& loc);
  void deactivate_water(tile_location const& loc);
  water_movement_info& activate_water(tile_location const& loc);
  
  void set_stickyness(tile_location const& loc, bool new_stickyness);
  
  void queue_creating_object(shared_ptr<object> obj) {
    objects_to_add.push_back(obj);
  }
  
  // Risky function: Don't use this while iterating through objects (any other time is okay)
  void create_queued_objects() {
    for (shared_ptr<object> const& obj : objects_to_add) {
      object_identifier id = next_object_identifier++;
      objects.insert(make_pair(id, obj));
      // TODO put it in the collision detector
      // TODO add it to the subcategories if it wants to be in them
      if(shared_ptr<mobile_object> m = std::dynamic_pointer_cast<mobile_object>(obj)) {
        moving_objects.insert(make_pair(id, m));
      }
    }
    objects_to_add.clear();
  }
  
  // If objects overlap with the new position, returns their IDs. If not, changes the shape and returns an empty set.
  unordered_set<object_or_tile_identifier> try_to_change_personal_space_shape(object_identifier id, shape const& new_shape);
  // Objects can't fail to change their detail shape, but it may cause effects (like blocking a laser beam)
  void change_detail_shape(object_identifier id, shape const& new_shape);
  
  void add_laser_sfx(vector3<fine_scalar> laser_source, vector3<fine_scalar> laser_delta) {
    laser_sfxes.push_back(make_pair(laser_source, laser_delta));
  }
  std::vector<std::pair<vector3<fine_scalar>, vector3<fine_scalar>>> laser_sfxes;
  
  
  object_shapes_t const& get_object_personal_space_shapes()const { return object_personal_space_shapes; }
private:
  friend class world_building_gun;
  friend class hacky_internals::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  
  // This map uses the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  unordered_map<vector3<tile_coordinate>, hacky_internals::worldblock> blocks; 
  
  active_water_tiles_t active_water_tiles;
  objects_map<object>::type objects;
  objects_map<mobile_object>::type moving_objects;
  objects_map<laser_emitter>::type active_lasers;
  
  object_identifier next_object_identifier;
  vector<shared_ptr<object>> objects_to_add;
  object_shapes_t object_personal_space_shapes;
  object_shapes_t object_detail_shapes;
  
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

inline void update(world &w) {
  w.create_queued_objects();
  w.laser_sfxes.clear();
  update_water(w);
  w.update_moving_objects();
}

class laser_emitter : public mobile_object {
public:
  laser_emitter(vector3<fine_scalar> location, vector3<fine_scalar> facing):location(location),facing(facing){}
  /*virtual fine_bounding_box bbox()const {
  // TODO define tile width???????
    return fine_bounding_box(location - tile_size / 3, tile_size * 2 / 3); // TODO lolhack
  }*/
  /*virtual void move_due_to_velocity() {
    // gravity. TODO hacky
    velocity += vector3<fine_scalar>(0, 0, -1);
    
    location += velocity;
    //std::cerr << std::hex << location.z;
  }*/
  virtual void update(world *w) {
    struct laser_path_calculator {
      struct cross {
        fine_scalar dist_in_this_dimension;
        fine_scalar facing_in_this_dimension;
        int dimension;
        cross(fine_scalar d, fine_scalar f, int di):dist_in_this_dimension(d),facing_in_this_dimension(f),dimension(di){}
        bool operator<(cross const& other)const {
          return other.facing_in_this_dimension * dist_in_this_dimension < facing_in_this_dimension * other.dist_in_this_dimension;
        }
      };
      laser_path_calculator(laser_emitter *emi):emi(emi),current_laser_tile(get_containing_tile_coordinates(emi->location)){
        for (int i = 0; i < 3; ++i) {
          facing_signs[i] = sign(emi->facing[i]);
          facing_offs[i] = emi->facing[i] > 0;
          if (facing_signs[i] != 0) enter_next_cross(i);
        }
      }
      
      void enter_next_cross(int which_dimension) {
        coming_crosses.insert(cross(
          lower_bound_in_fine_units((current_laser_tile + facing_offs)[which_dimension], which_dimension) - emi->location[which_dimension],
          emi->facing[which_dimension],
          which_dimension
        ));
      }
      // returns which dimension we advanced
      int advance_to_next_location() {
        auto next_cross_iter = coming_crosses.begin();
        int which_dimension = next_cross_iter->dimension;
        coming_crosses.erase(next_cross_iter);
        current_laser_tile[which_dimension] += facing_signs[which_dimension];
        enter_next_cross(which_dimension);
        return which_dimension;
      }
      
      laser_emitter *emi;
      vector3<tile_coordinate> current_laser_tile;
      set<cross> coming_crosses;
      vector3<fine_scalar> facing_signs;
      vector3<tile_coordinate_signed_type> facing_offs;
    };
    
    laser_path_calculator calc(this);
    
    //std::cerr << facing.x << " foo " << facing.y << " foo " << facing.z << "\n";
    const vector3<fine_scalar> max_laser_delta = ((facing * (100LL << 10)) / facing.magnitude_within_32_bits()); // TODO fix overflow
    //std::cerr << max_laser_delta.x << " foo " << max_laser_delta.y << " foo " << max_laser_delta.z << " bar " << facing.magnitude() << " bar " << (facing * (100LL << 10)).x << "\n";
    //const vector3<tile_coordinate> theoretical_end_tile = get_containing_tile_coordinates(location + max_laser_delta);
    
    int which_dimension_we_last_advanced = -1;
    while(true) {
      unordered_set<object_or_tile_identifier> possible_hits;
      w->collect_things_exposed_to_collision_intersecting(possible_hits, tile_bounding_box(calc.current_laser_tile));
      for (object_or_tile_identifier const& id : possible_hits) {
        if (id.get_tile_location()) {
          // it's the entire tile, of course we hit it!
          vector3<fine_scalar> laser_delta;
          if (which_dimension_we_last_advanced == -1) {
            laser_delta = vector3<fine_scalar>(0,0,0);
          }
          else {
            vector3<tile_coordinate> hitloc_finding_hack = calc.current_laser_tile;
            hitloc_finding_hack[which_dimension_we_last_advanced] += calc.facing_offs[which_dimension_we_last_advanced];
            const fine_scalar laser_delta_in_facing_direction = (lower_bound_in_fine_units(hitloc_finding_hack)[which_dimension_we_last_advanced] - location[which_dimension_we_last_advanced]);
            laser_delta = (facing * laser_delta_in_facing_direction) / facing[which_dimension_we_last_advanced];
          }
          w->add_laser_sfx(location, laser_delta);
          return; // TODO handle what happens if there are mobile objects and/or multiple objects and/or whatever
        }
      }
      // TODO figure out a better end condition...
      if ((calc.current_laser_tile - get_containing_tile_coordinates(location)).magnitude_within_32_bits_is_greater_than(101)) {
        w->add_laser_sfx(location, max_laser_delta);
        return;
      }
      else which_dimension_we_last_advanced = calc.advance_to_next_location();
    }
  }
private:
  vector3<fine_scalar> location;
  vector3<fine_scalar> facing;
};

#endif

