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

#ifndef LASERCAKE_WORLD_HPP__
#define LASERCAKE_WORLD_HPP__

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <cmath>
#include <boost/functional/hash.hpp>

#include <cstdlib>
#include <cassert>
#include <map>
#include <set>
#include <algorithm>
#include <boost/utility.hpp>
#include <boost/range/iterator_range.hpp>
#include <inttypes.h>
#include <functional>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/variant.hpp>
#include <boost/variant/get.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include "utils.hpp"
#include "data_structures/geometry.hpp"
#include "data_structures/bbox_collision_detector.hpp"
#include "data_structures/patricia_trie.hpp"
#include "the_decomposition_of_the_world_into_blocks.hpp"
#include "tiles.hpp"
#include "input_representation.hpp"
#include "world_constants.hpp"

using std::pair;
using std::make_pair;
using std::map;
using std::set;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using std::array;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;


struct minerals {
  minerals(physical_quantity<lint64_t, units<dim::meter<3>>> metal):metal(metal){}
  physical_quantity<lint64_t, units<dim::meter<3>>> metal;
  // nonmetal rock is (200-metal)
};

inline minerals initial_minerals(vector3<tile_coordinate> coords) {
  return minerals(((coords.x ^ coords.y ^ coords.z) & 31) * meters * meters * meters);
}

inline shape tile_shape(vector3<tile_coordinate> tile) {
  return shape(fine_bounding_box_of_tile(tile));
}


constexpr tile_coordinate world_center_tile_coord = (tile_coordinate(1) << (8*sizeof(tile_coordinate) - 2)) - 5;
// A possibility:
// 0x5 is binary 0101.  We alternate 0 and 1 bits so that
// we don't start particularly near high-bit changes that
// require larger power-of-two-aligned bounding boxes
// in order to encompass the world.  This isn't fundamentally
// important; if it's more useful to the user we could start
// at decimal 1000000000 instead (binary 111011100110101100101000000000).
//const tile_coordinate world_center_tile_coord = (tile_coordinate(1) << (8*sizeof(tile_coordinate) - 2)) - 5;//0x55555555;

constexpr vector3<tile_coordinate> world_center_tile_coords(world_center_tile_coord, world_center_tile_coord, world_center_tile_coord);
constexpr vector3<distance> world_center_fine_coords = lower_bound_in_fine_distance_units(world_center_tile_coords);





typedef uint64_t object_identifier;
const object_identifier NO_OBJECT = 0;

struct object_or_tile_identifier {
  object_or_tile_identifier():data_(NO_OBJECT){}
  object_or_tile_identifier(tile_location const& loc):data_(loc){}
  object_or_tile_identifier(object_identifier id):data_(id){}
  tile_location const* get_tile_location()const { return boost::get<tile_location>(&data_); }
  object_identifier const* get_object_identifier()const { return boost::get<object_identifier>(&data_); }
  friend inline size_t hash_value(object_or_tile_identifier const& id) {
    struct hash_visitor : public boost::static_visitor<size_t>
    {
      size_t operator()(tile_location const& i) const {
        return std::hash<tile_location>()(i);
      }
    
      size_t operator()(object_identifier i) const {
        return std::hash<object_identifier>()(i);
      }
    };
    return boost::apply_visitor( hash_visitor(), id.data_ );
  }
  bool operator==(object_or_tile_identifier const& other)const { return data_ == other.data_; }
  bool operator==(object_identifier const& other)const {
    if (object_identifier const* foo = get_object_identifier()) { return *foo == other; }
    else return false;
  }
  bool operator==(tile_location const& other)const {
    if (tile_location const* foo = get_tile_location()) { return *foo == other; }
    else return false;
  }
  bool operator!=(object_or_tile_identifier const& other)const { return !(*this == other); }
  bool operator!=(object_identifier const& other)const { return !(*this == other); }
  bool operator!=(tile_location const& other)const { return !(*this == other); }

  bool operator<(object_or_tile_identifier const& other)const { return data_ < other.data_; }

  friend inline std::ostream& operator<<(std::ostream& os, object_or_tile_identifier const& id) {
    struct ostream_visitor : public boost::static_visitor<void>
    {
      ostream_visitor(std::ostream& os) : os(os) {}
      std::ostream& os;
      
      void operator()(tile_location const& i) const { os << i; }
      void operator()(object_identifier i) const { os << i; }
    };
    boost::apply_visitor( ostream_visitor(os), id.data_ );
    return os;
  }
private:
  boost::variant<tile_location, object_identifier> data_;
};

namespace std {
  template<> struct hash<object_or_tile_identifier> {
    inline size_t operator()(object_or_tile_identifier const& id) const {
      return hash_value(id);
    }
  };
}
class object {
public:
  virtual shape get_initial_personal_space_shape()const = 0;
  virtual shape get_initial_detail_shape()const = 0;

  virtual ~object() {}
};

class mobile_object : virtual public object {
public:
  //virtual void move_due_to_velocity() = 0;

  mobile_object():velocity_(0,0,0){}
  mobile_object(vector3<velocity1d> velocity):velocity_(velocity){}
  vector3<velocity1d> const& velocity() const { return velocity_; }
  vector3<velocity1d> velocity_;
};

// The player can use these objects.
class object_with_eye_direction : virtual public object {
public:
  virtual vector3<distance> get_facing()const = 0;
};
class object_with_player_instructions : virtual public object {
public:
  virtual std::string player_instructions()const = 0;
};

#if 0
// A possibility:
class mind_controllable_object : virtual public object {
public:
  // These can be possessed as by a spirit so the player can
  // control and see from it.  We may need to split this class
  // in the future.
  virtual void mind_control(input_representation::input_news_t const& input_news) {}
  // Current assumption of this class: these have excellent eyes
  // that a human can see from.
  virtual vector3<distance> eye_location() const = 0;
  virtual vector3<distance> eye_direction() const = 0;
};
#endif

class tile_aligned_object : virtual public object {
public:
  
};

class autonomous_object : virtual public object {
public:
  virtual void update(world& w, input_representation::input_news_t const& mind_control, object_identifier my_id) = 0;
};


typedef bbox_collision_detector<object_or_tile_identifier, 64, 3> world_collision_detector;
typedef bbox_collision_detector<object_identifier, 64, 3> objects_collision_detector;


// worldgen_function_t is responsible for initializing every tile in
// the worldblock, making them initialized to 'interior' (worldblock
// will take care of detecting the true interiorness), and not marked
// as anything else.
typedef std::function<void (the_decomposition_of_the_world_into_blocks_impl::worldblock*, tile_bounding_box)>
  worldgen_function_t;


typedef unordered_map<object_identifier, shape> object_shapes_t;
template<typename ObjectSubtype>
struct objects_map {
  typedef unordered_map<object_identifier, shared_ptr<ObjectSubtype>> type;
};







namespace tile_physics_impl {
struct state_t;
class tile_physics_state_t {
public:
  tile_physics_state_t(world& w);
  tile_physics_state_t(tile_physics_state_t const& other);
  tile_physics_state_t& operator=(tile_physics_state_t const& other);
  ~tile_physics_state_t() BOOST_NOEXCEPT;
private:
  boost::scoped_ptr<state_t> state_;
  friend state_t& get_state(tile_physics_state_t& s);
  friend state_t const& get_state(tile_physics_state_t const& s);
};
inline state_t& get_state(tile_physics_state_t& s) {
  return *s.state_;
}
inline state_t const& get_state(tile_physics_state_t const& s) {
  return *s.state_;
}
} // end namespace tile_physics_impl
using tile_physics_impl::tile_physics_state_t;





class worldgen_type;



class world {
public:
  // lolhack. TODO: Replace this with a for-real thing
std::unordered_map<object_identifier, int> object_litnesses_;
std::unordered_map<vector3<tile_coordinate>, int> tile_litnesses_;
void update_light(vector3<distance> sun_direction, uint32_t sun_direction_z_shift);

  // TODO does this make more sense as a unique_ptr?
  world(shared_ptr<worldgen_type> gen);
  
  // If worlds being copiable is ever needed (a world is totally copiable in principle),
  // the error will remind us to make sure to implement the copy-constructor correctly
  // (e.g. without sharing any data via pointers by accident).
  world(world const& other) = delete;
  world& operator=(world const& other) = delete;
  
  void update_moving_objects();
  void update_fluids();

  // "Game time" attempts to match real world seconds, but it might
  // get behind due to slow CPUs or ahead due to someone fast-forwarding
  // (if that were a thing)
  // (and currently, also can get ahead because we don't cap the frame-rate.)
  time_unit game_time_elapsed()const { return current_game_time_; }

  void update(unordered_map<object_identifier, input_representation::input_news_t> input);

  large_fast_noncrypto_rng& get_rng() { return rng_; }
  
  // I *think* this pointer is valid as long as the shared_ptr exists
  shared_ptr<object>* get_object(object_identifier id) { return find_as_pointer(objects_, id); }
  /*boost::iterator_range<mobile_objects_map<mobile_object>::iterator> mobile_objects_range() {
    return boost::make_iterator_range(mobile_objects.begin(), mobile_objects.end());
  }*/
  boost::iterator_range<objects_map<mobile_object>::type::iterator> moving_objects_range() {
    return boost::make_iterator_range(moving_objects_.begin(), moving_objects_.end());
  }
  
  tile_location make_tile_location(vector3<tile_coordinate> const& coords, level_of_tile_realization_needed realineeded);

  minerals get_minerals(vector3<tile_coordinate> const& coords)const;

  void ensure_realization_of_space(tile_bounding_box space, level_of_tile_realization_needed realineeded);
  void ensure_realization_of_space(bounding_box space, level_of_tile_realization_needed realineeded) {
    ensure_realization_of_space(get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(space), realineeded);
  }
  
  // old_substance_type doesn't actually do anything except give an assertion.
  // However, it's imperative that your code not accidentally overwrite a cool
  // type of substance that you weren't considering, so the assertion is built
  // into the function call to make sure that you use it.
  void replace_substance(
     tile_location const& loc,
     tile_contents old_substance_type,
     tile_contents new_substance_type);
  
  object_identifier try_create_object(shared_ptr<object> obj);
  
  //  If objects overlap with the new position, returns their IDs. If not,
  //  changes the shape and returns an empty set.
  //unordered_set<object_or_tile_identifier> try_to_change_personal_space_shape(object_identifier id, shape const& new_shape);
  //  Objects can't fail to change their detail shape, but it may cause
  //  effects (like blocking a laser beam)
  //void change_detail_shape(object_identifier id, shape const& new_shape);

  typedef std::vector<std::pair<vector3<distance>, vector3<distance>>> laser_sfxes_type;
  void add_laser_sfx(vector3<distance> laser_source, vector3<distance> laser_delta) {
    laser_sfxes_.push_back(make_pair(laser_source, laser_delta));
  }
  laser_sfxes_type const& get_laser_sfxes()const { return laser_sfxes_; }

  // TODO: Either
  // 1) split this function into a personal_space version and a
  //      detail version, or
  // 2) make it explicit that it's the bounding box including both
  //      (that's what it is currently, or at least what it SHOULD be
  //       currently), or
  // 3) remove it / come up with something different to replace it with
  bounding_box get_bounding_box_of_object_or_tile(object_or_tile_identifier id)const;
  shape get_personal_space_shape_of_object_or_tile(object_or_tile_identifier id)const;
  shape get_detail_shape_of_object_or_tile(object_or_tile_identifier id)const;
  
  objects_map<object>::type const& get_objects()const { return objects_; }
  object_shapes_t const& get_object_personal_space_shapes()const { return object_personal_space_shapes_; }
  object_shapes_t const& get_object_detail_shapes()const { return object_detail_shapes_; }

  tile_physics_state_t& tile_physics() { return tile_physics_state_; }
  tile_physics_state_t const& tile_physics()const { return tile_physics_state_; }
  objects_collision_detector const& objects_exposed_to_collision()const { return objects_exposed_to_collision_; }

  // Requires ensure_realization_of_space(bounds, CONTENTS_AND_LOCAL_CACHES_ONLY)
  // but does not call it (because ensure_realization_of_space on an
  // already-realized space takes asymptotically more time for large spaces
  // with not so many tiles-exposed-to-collision than the rest of
  // get_tiles_exposed_to_collision_within does.
  // (TODO improve ensure_realization_of_space).
  void get_tiles_exposed_to_collision_within(std::vector<tile_location>& results, tile_bounding_box bounds);

  // Include tile_iteration.hpp to use visit_collidable_tiles.
  // You are responsible for ensuring realization of any tiles you
  // want to make sure to capture.
  template<typename Visitor> void visit_collidable_tiles(Visitor&& visitor);

  // Include object_and_tile_iteration.hpp to use
  // visit_collidable_tiles_and_objects.  This function is a HACK because
  // among other things, it is not very precise in the ordering among objects
  // (though tiles will all be in a correct order).
  template<typename Visitor> void visit_collidable_tiles_and_objects(/*bounding_box bbox,*/ Visitor&& visitor);

  // For displaying in debug-display mode:
  the_decomposition_of_the_world_into_blocks_impl::worldblock_trie const&
  debug_get_worldblock_trie()const { return worldblock_trie_; }

  // For when a worldblock notices it is all super interior
  void suggest_deleting_worldblock(the_decomposition_of_the_world_into_blocks_impl::worldblock* wb) {
    worldblocks_suggested_to_delete_.insert(wb);
  }
private:
  // No harm in doing this, because worldblock is by definition already hacky:
  friend class the_decomposition_of_the_world_into_blocks_impl::worldblock;
  
  time_unit current_game_time_;

  //TODO move this, worldblock_trie, etc into a struct in
  //  the_decomposition_of_the_world_into_blocks_impl ?
  // This map uses the same coordinates as worldblock::global_position -
  // i.e. worldblocks' coordinates are multiples of worldblock_dimension,
  // and it is an error to give a coordinate that's not.
  unordered_map<vector3<tile_coordinate>, the_decomposition_of_the_world_into_blocks_impl::worldblock> blocks_;
  //currently we just delete them all at the end of every frame
  std::unordered_set<the_decomposition_of_the_world_into_blocks_impl::worldblock*> worldblocks_suggested_to_delete_;

  tile_physics_state_t tile_physics_state_;
  
  objects_map<object>::type objects_;
  objects_map<mobile_object>::type moving_objects_;
  objects_map<autonomous_object>::type autonomously_active_objects_;
  
  object_identifier next_object_identifier_;
  vector<shared_ptr<object>> objects_to_add_;
  object_shapes_t object_personal_space_shapes_;
  object_shapes_t object_detail_shapes_;
  
  // "exposed to collision" currently means all mobile objects,
  // and all non-interior, non-air tiles.
  objects_collision_detector objects_exposed_to_collision_;
  the_decomposition_of_the_world_into_blocks_impl::worldblock_trie worldblock_trie_;

  laser_sfxes_type laser_sfxes_;
  
  shared_ptr<worldgen_type> worldgen_;
  
  // RNG, default-initialized for now
  large_fast_noncrypto_rng rng_;
  
  
  the_decomposition_of_the_world_into_blocks_impl::worldblock* ensure_realization_of_and_get_worldblock_(
    vector3<tile_coordinate> position, level_of_tile_realization_needed realineeded);
  
  // Used only in the worldblock stuff
  void initialize_tile_water_group_caches_(tile_location const& loc);
};



#endif

