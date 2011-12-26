
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

using std::map;
using std::unordered_map;
using std::unordered_set;
using std::pair;
using std::make_pair;
using std::set;
using std::vector;
using std::array;
using std::bitset;

template<typename Map>
typename Map::mapped_type* find_as_pointer(Map& m, typename Map::key_type const& k) {
  auto i = m.find(k);
  if(i == m.end()) return nullptr;
  else return &(i->second);
}

class bounds_checked_int {
public:
	bounds_checked_int():value(0){}
	bounds_checked_int(int value):value(value){}
	bounds_checked_int &operator=(int other) { value = other; return *this; }
	bounds_checked_int operator+(int other)const {
		assert((int64_t)value + (int64_t)other < (1LL << 31));
		assert((int64_t)value + (int64_t)other > -(1LL << 31));
		return bounds_checked_int(value + other);
	}
	void operator+=(int other) {
		assert((int64_t)value + (int64_t)other < (1LL << 31));
		assert((int64_t)value + (int64_t)other > -(1LL << 31));
		value += other;
	}
	bounds_checked_int& operator++() {
		*this += 1; return *this;
	}
	bounds_checked_int operator++(int) {
		bounds_checked_int result(value);
		*this += 1;
		return result;
	}
	bounds_checked_int& operator--() {
		*this -= 1; return *this;
	}
	bounds_checked_int operator--(int) {
		bounds_checked_int result(value);
		*this -= 1;
		return result;
	}
	bounds_checked_int operator-(int other)const {
		assert((int64_t)value - (int64_t)other < (1LL << 31));
		assert((int64_t)value - (int64_t)other > -(1LL << 31));
		return bounds_checked_int(value - other);
	}
	void operator-=(int other) {
		assert((int64_t)value - (int64_t)other < (1LL << 31));
		assert((int64_t)value - (int64_t)other > -(1LL << 31));
		value -= other;
	}
	bounds_checked_int operator*(int other)const {
		assert((int64_t)value * (int64_t)other < (1LL << 31));
		assert((int64_t)value * (int64_t)other > -(1LL << 31));
		return bounds_checked_int(value * other);
	}
	void operator*=(int other) {
		assert((int64_t)value * (int64_t)other < (1LL << 31));
		assert((int64_t)value * (int64_t)other > -(1LL << 31));
		value *= other;
	}
	bounds_checked_int operator/(int other)const {
		return bounds_checked_int(value / other);
	}
	void operator/=(int other) {
		value /= other;
	}
	operator int()const{ return value; }
private:
	int value;
};

template<typename scalar_type> scalar_type divide_rounding_towards_zero(scalar_type dividend, scalar_type divisor)
{
	assert(divisor != 0);
	int abs_result = std::abs(dividend) / std::abs(divisor);
	if ((dividend > 0 && divisor > 0) || (dividend < 0 && divisor < 0)) return abs_result;
	else return -abs_result;
}

template<typename scalar_type> class vector3 {
public:
	scalar_type x, y, z;
	vector3():x(0),y(0),z(0){}
	vector3(scalar_type x, scalar_type y, scalar_type z):x(x),y(y),z(z){}
	template<typename OtherType> explicit vector3(vector3<OtherType> const& other):
	  x(other.x),y(other.y),z(other.z){}
	
	// Note: The operators are biased towards the type of the left operand (e.g. vector3<int> + vector3<int64_t> = vector3<int>)
	template<typename OtherType> vector3 operator+(vector3<OtherType> const& other)const {
		return vector3(x + other.x, y + other.y, z + other.z);
	}
	template<typename OtherType> void operator+=(vector3<OtherType> const& other) {
		x += other.x; y += other.y; z += other.z;
	}
	template<typename OtherType> vector3 operator-(vector3<OtherType> const& other)const {
		return vector3(x - other.x, y - other.y, z - other.z);
	}
	template<typename OtherType> void operator-=(vector3<OtherType> const& other) {
		x -= other.x; y -= other.y; z -= other.z;
	}
	vector3 operator*(scalar_type other)const {
		return vector3(x * other, y * other, z * other);
	}
	void operator*=(scalar_type other) {
		x *= other; y *= other; z *= other;
	}
	vector3 operator/(scalar_type other)const {
		return vector3(divide_rounding_towards_zero(x, other), divide_rounding_towards_zero(y, other), divide_rounding_towards_zero(z, other));
	}
	void operator/=(scalar_type other) {
		x = divide_rounding_towards_zero(x, other); y = divide_rounding_towards_zero(y, other); z = divide_rounding_towards_zero(z, other);
	}
	vector3 operator-()const { // unary minus
		return vector3(-x, -y, -z);
	}
	bool operator==(vector3 const& other)const {return x == other.x && y == other.y && z == other.z; }
	bool operator!=(vector3 const& other)const {return x != other.x || y != other.y || z != other.z; }
	
	// Do not try to use this if either vector has an unsigned scalar_type. It might work in some situations, but why would you ever do that anyway?
	// You are required to specify an output type, because of the risk of overflow. Make sure to choose one that can fit the squares of the numbers you're dealing with.
	template<typename OutputType, typename OtherType> OutputType dot(vector3<OtherType> const& other)const {
		return (OutputType)x * (OutputType)other.x +
		       (OutputType)y * (OutputType)other.y +
		       (OutputType)z * (OutputType)other.z;
	}
	
	// Warning - might be slightly inaccurate (in addition to the integer rounding error) for very large vectors, due to floating-point inaccuracy.
	scalar_type magnitude()const { return (scalar_type)std::sqrt(dot<double>(*this)); }
	
	// Choose these the way you'd choose dot's output type (see the comment above)
	// we had trouble making these templates, so now they just always use int64_t
	bool magnitude_within_32_bits_is_less_than(scalar_type amount)const {
	  return dot<int64_t>(*this) < (int64_t)amount * (int64_t)amount;
	}
	bool magnitude_within_32_bits_is_greater_than(scalar_type amount)const {
	  return dot<int64_t>(*this) > (int64_t)amount * (int64_t)amount;
	}
	bool operator<(vector3 const& other)const { return (x < other.x) || ((x == other.x) && ((y < other.y) || ((y == other.y) && (z < other.z)))); }
};

typedef int32_t sub_tile_distance;
typedef int8_t neighboring_tile_differential;
typedef uint32_t location_coordinate;

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

typedef int8_t cardinal_direction_index;
const cardinal_direction_index NUM_CARDINAL_DIRECTIONS = 6;
struct cardinal_direction {
  cardinal_direction(vector3<neighboring_tile_differential> v, cardinal_direction_index i):v(v),cardinal_direction_idx(i){}
  vector3<neighboring_tile_differential> v;
  cardinal_direction_index cardinal_direction_idx;
  cardinal_direction operator-()const;
};

vector3<sub_tile_distance> project_onto_cardinal_direction(vector3<sub_tile_distance> src, cardinal_direction dir) {
  return vector3<sub_tile_distance>(src.x * std::abs((sub_tile_distance)dir.v.x), src.y * std::abs((sub_tile_distance)dir.v.y), src.z * std::abs((sub_tile_distance)dir.v.z));
}

const vector3<neighboring_tile_differential> xunitv(1, 0, 0);
const vector3<neighboring_tile_differential> yunitv(0, 1, 0);
const vector3<neighboring_tile_differential> zunitv(0, 0, 1);
// the order of this must be in sync with the order of hacky_vector_indexing_internals::cardinal_direction_vector_to_index
const cardinal_direction cdir_xminus = cardinal_direction(-xunitv, 0);
const cardinal_direction cdir_yminus = cardinal_direction(-yunitv, 1);
const cardinal_direction cdir_zminus = cardinal_direction(-zunitv, 2);
const cardinal_direction cdir_xplus = cardinal_direction(xunitv, 3);
const cardinal_direction cdir_yplus = cardinal_direction(yunitv, 4);
const cardinal_direction cdir_zplus = cardinal_direction(zunitv, 5);
const cardinal_direction cardinal_directions[NUM_CARDINAL_DIRECTIONS] = { cdir_xminus, cdir_yminus, cdir_zminus, cdir_xplus, cdir_yplus, cdir_zplus };
#define EACH_CARDINAL_DIRECTION(varname) cardinal_direction const& varname : cardinal_directions
cardinal_direction cardinal_direction::operator-()const { return cardinal_directions[(cardinal_direction_idx + 3)%6]; }

template<typename value_type> class value_for_each_cardinal_direction {
public:
  value_for_each_cardinal_direction& operator=(value_for_each_cardinal_direction const& other){
    for(cardinal_direction_index dir_idx=0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      data[dir_idx] = other.data[dir_idx];
    }
    return *this;
  }
  value_for_each_cardinal_direction(value_type initial_value) {
    for(cardinal_direction_index dir_idx=0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      data[dir_idx] = initial_value;
    }
  }
  value_type      & operator[](cardinal_direction const& dir) { return data[dir.cardinal_direction_idx]; }
  value_type const& operator[](cardinal_direction const& dir)const { return data[dir.cardinal_direction_idx]; }
private:
  typedef array<value_type, NUM_CARDINAL_DIRECTIONS> internal_array;
public:
  typename internal_array::iterator begin() { return data.begin(); }
  typename internal_array::iterator end() { return data.end(); }
  typename internal_array::const_iterator cbegin()const { return data.cbegin(); }
  typename internal_array::const_iterator cend()const { return data.cend(); }
private:
  internal_array data;
};


struct water_movement_info {
  vector3<sub_tile_distance> velocity;
  value_for_each_cardinal_direction<sub_tile_distance> progress;
  value_for_each_cardinal_direction<sub_tile_distance> new_progress;
  value_for_each_cardinal_direction<sub_tile_distance> blockage_amount_this_frame;
  
  // Constructing one of these in the default way yields the natural idle state:
  // however we should NOT use std::unordered_map's operator[] to activate it; it should only be created through tiles' activate_water and insert_water.
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


enum tile_contents {
  ROCK,
  WATER,
  AIR
};

struct tile
{
  tile():contents(AIR),is_sticky(false),is_interior(false){}
  tile_contents contents;
  bool is_sticky;
  bool is_interior;
  bool is_sticky_water  ()const{ return contents == WATER &&  is_sticky                ; }
  bool is_free_water    ()const{ return contents == WATER && !is_sticky                ; }
  bool is_interior_water()const{ return contents == WATER &&  is_sticky &&  is_interior; }
  bool is_membrane_water()const{ return contents == WATER &&  is_sticky && !is_interior; }
};


const int worldblock_dimension_exp = 4;
typedef int worldblock_dimension_type;
const worldblock_dimension_type worldblock_dimension = (1 << worldblock_dimension_exp);

class worldblock;

class location {
public:
  // this constructor should only be used when you know exactly what worldblock it's in!!
  // TODO: It's bad that it's both public AND doesn't assert that condition
  location(vector3<location_coordinate> v, worldblock *wb):v(v),wb(wb){}
  
  location operator+(cardinal_direction dir)const;
  location operator-(cardinal_direction dir)const { return (*this)+(-dir); }
  tile& stuff_at()const;
  vector3<location_coordinate> const& coords()const { return v; }
private:
  vector3<location_coordinate> v;
  worldblock *wb;
};

namespace std {
  template<typename scalar_type> struct hash<vector3<scalar_type> > {
    inline size_t operator()(vector3<scalar_type> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.x);
      boost::hash_combine(seed, v.y);
      boost::hash_combine(seed, v.z);
      return seed;
    }
  };
  template<> struct hash<location> {
    inline size_t operator()(location const& l) const {
      return hash<vector3<location_coordinate> >()(l.coords());
    }
  };
}

class world;

class worldblock {
public:
  // When a worldblock is inited, it DOESN'T call insert_water for all the water in it - the water is 'already there'.
  // Water that starts out in a worldblock starts out inactive (observing the rule "the landscape takes zero time to process").
  //
  // We would have to make special rules for worldblocks that start out with
  // active water in them, because it could invalidate iterators into the
  // active_tiles map, because worldblocks can be created essentially any time in the processing.
  worldblock& init_if_needed(world *w_) {
    if (!inited) {
      w = w_;
    }
    return (*this);
  }
private:
  std::array<std::array<std::array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles;
  value_for_each_cardinal_direction<worldblock*> neighbors;
  vector3<location_coordinate> global_position; // the lowest x, y, and z among elements in this worldblock
  world *w;
  bool inited;
  
  friend class location;
  // Only to be used in location::stuff_at():
  tile &get_tile(vector3<location_coordinate> global_coords) {
    vector3<location_coordinate> local_coords = global_coords - global_position;
    return tiles[local_coords.x][local_coords.y][local_coords.z];
  }
  
  // Only to be used in location::operator+(cardinal_direction):
  location get_neighboring_loc(vector3<location_coordinate> const& old_coords, cardinal_direction dir) {
    // this could be made more effecient, but I'm not sure how
    vector3<location_coordinate> new_coords = old_coords + dir.v;
    if (new_coords.x < global_position.x) return location(new_coords, neighbors[cdir_xminus]);
    if (new_coords.y < global_position.y) return location(new_coords, neighbors[cdir_yminus]);
    if (new_coords.z < global_position.z) return location(new_coords, neighbors[cdir_zminus]);
    if (new_coords.x >= global_position.x + worldblock_dimension) return location(new_coords, neighbors[cdir_xplus]);
    if (new_coords.y >= global_position.y + worldblock_dimension) return location(new_coords, neighbors[cdir_yplus]);
    if (new_coords.z >= global_position.z + worldblock_dimension) return location(new_coords, neighbors[cdir_zplus]);
    return location(new_coords, this);
  }
  
  location get_loc_across_boundary(vector3<location_coordinate> const& new_coords, cardinal_direction dir);
};

location location::operator+(cardinal_direction dir)const {
  return wb->get_neighboring_loc(v, dir);
}
tile& location::stuff_at()const { return wb->get_tile(v); }


struct ztree_entry {
private:
  location loc_;
  std::array<location_coordinate, 3> interleaved_bits;
  static const size_t bits_in_loc_coord = 8*sizeof(location_coordinate);
  void set_bit(size_t idx) {
    interleaved_bits[idx / bits_in_loc_coord] &= (location_coordinate(1) << (idx % bits_in_loc_coord));
  }
public:
  location const& loc() { return loc_; }
  
  ztree_entry(location const& loc_):loc_(loc_),interleaved_bits() {
    interleaved_bits[0] = 0;
    interleaved_bits[1] = 0;
    interleaved_bits[2] = 0;
    for (size_t bit = 0; bit < bits_in_loc_coord; ++bit) {
      if (loc_.coords().x & (location_coordinate(1) << bit)) set_bit(3*bit + 0);
      if (loc_.coords().y & (location_coordinate(1) << bit)) set_bit(3*bit + 1);
      if (loc_.coords().z & (location_coordinate(1) << bit)) set_bit(3*bit + 2);
    }
  }
  
  inline bool operator==(ztree_entry const& other)const { return loc_.coords() == other.loc_.coords(); }
  inline bool operator<(ztree_entry const& other)const {
    if (interleaved_bits[2] < other.interleaved_bits[2]) return true;
    if (interleaved_bits[2] > other.interleaved_bits[2]) return false;
    if (interleaved_bits[1] < other.interleaved_bits[1]) return true;
    if (interleaved_bits[1] > other.interleaved_bits[1]) return false;
    return (interleaved_bits[0] < other.interleaved_bits[0]);
  }
};



class world {
private:
  unordered_map<vector3<location_coordinate>, worldblock> blocks; // using the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  
  typedef unordered_map<location, water_movement_info> active_tiles_t;
  active_tiles_t active_tiles;
  set<ztree_entry> tiles_that_contain_anything;
  
public:
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
  
  void collect_tiles_that_contain_anything_near(unordered_set<location> &results, location center, int radius) {
    // TODO use something nicer than "int"
    const int total_width = 2*radius + 1;
    int exp = 0; while ((1 << exp) < total_width) ++exp;
    const int x_shift = (center.coords().x & ((1 << exp) - 1)) < (1 << (exp - 1)) ? -1 : 0;
    const int y_shift = (center.coords().y & ((1 << exp) - 1)) < (1 << (exp - 1)) ? -1 : 0;
    const int z_shift = (center.coords().z & ((1 << exp) - 1)) < (1 << (exp - 1)) ? -1 : 0;
    for (int x = 0; x < 2; ++x) { for (int y = 0; y < 2; ++y) { for (int z = 0; z < 2; ++z) {
      set<ztree_entry>::iterator lower_bound = tiles_that_contain_anything.lower_bound(
        ztree_entry(make_location(vector3<location_coordinate>(
          (center.coords().x & ~((1 << exp) - 1)) + ((x+x_shift) * (1 << exp)),
          (center.coords().y & ~((1 << exp) - 1)) + ((y+y_shift) * (1 << exp)),
          (center.coords().z & ~((1 << exp) - 1)) + ((z+z_shift) * (1 << exp))
        ))
      ));
      set<ztree_entry>::iterator upper_bound = tiles_that_contain_anything.upper_bound(
        ztree_entry(make_location(vector3<location_coordinate>(
          (center.coords().x | ((1 << exp) - 1)) + ((x+x_shift) * (1 << exp)),
          (center.coords().y | ((1 << exp) - 1)) + ((y+y_shift) * (1 << exp)),
          (center.coords().z | ((1 << exp) - 1)) + ((z+z_shift) * (1 << exp))
        ))
      ));
      results.insert(lower_bound, upper_bound);
    }}}
  }
  
  void check_stickyness(location loc) {
    tile &t = loc.stuff_at();
    if (t.contents == WATER) { // we don't care whether it's marked sticky if it's not water
      int airs = 0;
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location other_loc = loc + dir;
        if (other_loc.stuff_at().contents == AIR) ++airs;
      }
      bool should_be_sticky = (airs <= 1);
      if (t.is_sticky != should_be_sticky) {
        t.is_sticky = should_be_sticky;
        if (t.is_sticky) check_interiorness(loc);
        for (EACH_CARDINAL_DIRECTION(dir)) check_interiorness(loc + dir);
      }
    }
  }
  void check_interiorness(location loc) {
    tile &t = loc.stuff_at();
    if (t.contents == WATER && t.is_sticky) { // we don't care whether it's marked interior if it's not sticky water
      bool should_be_interior = true;
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location other_loc = loc + dir;
        if (!other_loc.stuff_at().is_sticky_water()) should_be_interior = false;
      }
      if (t.is_interior != should_be_interior) {
        t.is_interior = should_be_interior;
      }
    }
  }
  
  void delete_water(location loc) {
    tile &t = loc.stuff_at();
    assert(t.contents == WATER);
    t.contents = AIR;
    active_tiles.erase(loc);
    
    for (EACH_CARDINAL_DIRECTION(dir)) check_stickyness(loc + dir);
    
    tiles_that_contain_anything.erase(loc); // TODO This will need extra checks if there can be water and something else in a tile
  }
  water_movement_info& insert_water(location loc) {
    tile &t = loc.stuff_at();
    assert(t.contents == AIR);
    t.contents = WATER;
    
    check_stickyness(loc);
    for (EACH_CARDINAL_DIRECTION(dir)) check_stickyness(loc + dir);
    
    tiles_that_contain_anything.insert(loc);
    
    // created water always starts out active.
    return active_tiles[loc]; // inserts it, default-constructed
  }
  void deactivate_water(location loc) {
    active_tiles.erase(loc);
  }
  water_movement_info& activate_water(location loc) {
    assert(loc.stuff_at().contents == WATER);
    return active_tiles[loc]; // if it's not there, this inserts it, default-constructed
  }
};

location worldblock::get_loc_across_boundary(vector3<location_coordinate> const& new_coords, cardinal_direction dir) {
  if (worldblock* neighbor = neighbors[dir]) return location(new_coords, neighbor);
  return location(new_coords, (neighbors[dir] = w->create_if_necessary_and_get_worldblock(global_position + vector3<worldblock_dimension_type>(dir.v) * worldblock_dimension)));
}







/*

Whether tiles are water or not
 -- dictates, at one tile distance --
Whether water tiles are "sticky water" ("fewer than two adjacent tiles are air") or "free water" (at least two adjacent air tiles)
 -- dictates, at one tile distance --
Whether sticky water tiles are "interior water" ("every adjacent tile is sticky water") or "membrane water" ("at least one tile is not a sticky water tile")

This is a one-way dictation - the latter things don't affect the former things at all (until they cause water to actually move, anyway). It's not an exact dictation 

*/





typedef int group_number_t;
const group_number_t NO_GROUP = -1;
const group_number_t FIRST_GROUP = 0;




struct water_groups_structure {
  unordered_map<location, group_number_t> group_numbers_by_location;
  vector<unordered_set<location> > locations_by_group_number;
  vector<location_coordinate> max_z_by_group_number;
};

void collect_membrane(location const& loc, group_number_t group_number, water_groups_structure &result) {
  vector<location> frontier;
  frontier.push_back(loc);
  while(!frontier.empty())
  {
    const location next_loc = frontier.back();
    frontier.pop_back();
    if (result.group_numbers_by_location.find(next_loc) == result.group_numbers_by_location.end()) {
      result.group_numbers_by_location.insert(make_pair(next_loc, group_number));
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location adj_loc = next_loc + dir;
        if (adj_loc.stuff_at().is_membrane_water()) {
          frontier.push_back(adj_loc);
        }
      }
    }
  }
}

// the argument must start out default-initialized
void compute_groups_that_need_to_be_considered(world &w, water_groups_structure &result) {
  group_number_t next_membrane_number = FIRST_GROUP;
  for (pair<const location, water_movement_info> const& p : w.active_tiles_range()) {
    location const& loc = p.first;
    if (loc.stuff_at().is_membrane_water() && result.group_numbers_by_location.find(loc) == result.group_numbers_by_location.end()) {
      collect_membrane(loc, next_membrane_number, result);
      ++next_membrane_number;
    }
  }
  
  // Make a list of locations, sorted by X then Y then Z
  vector<pair<location, group_number_t> > columns; columns.reserve(result.group_numbers_by_location.size());
  columns.insert(columns.end(), result.group_numbers_by_location.begin(), result.group_numbers_by_location.end());
  std::sort(columns.begin(), columns.end()); // TODO: don't rely on location's < to be x-then-y-then-z
  
  group_number_t containing_group;
  for (pair<location, group_number_t> const& p : columns) {
    if ((p.first + cdir_zminus).stuff_at().is_sticky_water()) {
      // The tile we've just hit is the boundary of a bubble in the larger group (or it's extra bits of the outer shell, in which case the assignment does nothing).
      result.group_numbers_by_location[p.first] = containing_group;
    }
    else {
      // We're entering a group from the outside, which means it functions as a containing group.
      containing_group = p.second;
    }
  }
  
  // Now we might have some sort of group numbers like "1, 2, 7" because they've consolidated in essentially a random order. Convert them to a nice set of indices like 0, 1, 2.
  map<group_number_t, group_number_t> key_compacting_map;
  group_number_t next_group_number = FIRST_GROUP;
  for (pair<const location, group_number_t> &p : result.group_numbers_by_location) {
    if (group_number_t *new_number = find_as_pointer(key_compacting_map, p.second)) {
      p.second = *new_number;
    }
    else {
      p.second = key_compacting_map[p.second] = next_group_number++; // Bwa ha ha ha ha.
      result.locations_by_group_number.push_back(unordered_set<location>());
      result.max_z_by_group_number.push_back(p.first.coords().z);
    }
    result.locations_by_group_number[p.second].insert(p.first);
    result.max_z_by_group_number[p.second] = std::max(result.max_z_by_group_number[p.second], p.first.coords().z);
  }
}


struct wanted_move {
  location src;
  cardinal_direction dir;
  group_number_t group_number_or_NO_GROUP_for_velocity_movement;
  sub_tile_distance amount_of_the_push_that_sent_us_over_the_threshold;
  sub_tile_distance excess_progress;
  wanted_move(location src,cardinal_direction dir,group_number_t g,sub_tile_distance a,sub_tile_distance e):src(src),dir(dir),group_number_or_NO_GROUP_for_velocity_movement(g),amount_of_the_push_that_sent_us_over_the_threshold(a),excess_progress(e){}
};

  
template<typename Stuff> struct literally_random_access_removable_stuff {
public:
  void add(Stuff stuff) {
    stuffs.push_back(stuff);
    blacklist_list.push_back(false);
  }
  void blacklist(size_t which) {
    ++num_blacklisted;
    blacklist_list[which] = true;
    if (num_blacklisted * 2 > stuffs.size()) {
      purge_blacklisted_stuffs();
    }
  }
  Stuff get_and_blacklist_random() {
    assert(!stuffs.empty());
    size_t idx;
    do { idx = (size_t)(rand()%(stuffs.size())); } while (blacklist_list[idx]);
    Stuff result = stuffs[idx];
    blacklist(idx);
    return result;
  }
  bool empty()const { return stuffs.empty(); }
private:
  vector<Stuff> stuffs;
  vector<bool> blacklist_list;
  size_t num_blacklisted;
  void purge_blacklisted_stuffs() {
    size_t next_insert_idx = 0;
    for (size_t i = 0; i < stuffs.size(); ++i) {
      if (!blacklist_list[i]) {
        stuffs[next_insert_idx] = stuffs[i];
        blacklist_list[next_insert_idx] = false;
        ++next_insert_idx;
      }
    }
    stuffs.erase(stuffs.begin() + next_insert_idx, stuffs.end());
    blacklist_list.erase(blacklist_list.begin() + next_insert_idx, blacklist_list.end());
    num_blacklisted = 0;
  }
};

void update_water(world &w) {
  for (pair<const location, water_movement_info> &p : w.active_tiles_range()) {
    tile const& t = p.first.stuff_at();
    assert(t.contents == WATER);
    for (sub_tile_distance &np : p.second.new_progress) np = 0;
    
    if (t.is_sticky_water()) {
      vector3<sub_tile_distance> &vel_ref = p.second.velocity;
      const vector3<sub_tile_distance> current_velocity_wrongness = vel_ref - idle_water_velocity;
      if (current_velocity_wrongness.magnitude_within_32_bits_is_less_than(sticky_water_velocity_reduction_rate)) {
        vel_ref = idle_water_velocity;
      }
      else {
        vel_ref -= current_velocity_wrongness * sticky_water_velocity_reduction_rate / current_velocity_wrongness.magnitude();
      }
    }
  }
  
  water_groups_structure groups;
  compute_groups_that_need_to_be_considered(w, groups);

  for (group_number_t group_number = FIRST_GROUP; group_number < (group_number_t)groups.locations_by_group_number.size(); ++group_number) {
    for (location const& loc : groups.locations_by_group_number[group_number]) {
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location dst_loc = loc + dir;
        tile const& dst_tile = dst_loc.stuff_at();
        if (!dst_tile.is_sticky_water() && dst_tile.contents != ROCK) { // i.e. is air or free water. Those exclusions aren't terribly important, but it'd be slightly silly to remove either of them (and we currently rely on both exclusions to make the idle state what it is.)
          const double pressure = (double)(groups.max_z_by_group_number[group_number] - loc.coords().z) - 0.5 - 0.5*dir.v.z; // proportional to depth, assuming side surfaces are at the middle of the side. This is less by 1.0 than it naturally should be, to prevent water that should be stable (if unavoidably uneven by 1 tile or less) from fluctuating.
          if (pressure > 0) {
            w.activate_water(loc).new_progress[dir] += (sub_tile_distance)((pressure_motion_factor + (rand()%pressure_motion_factor_random)) * std::sqrt(pressure));
          }
        }
      }
    }
  }
  
  for (pair<const location, water_movement_info> &p : w.active_tiles_range()) {
    location const& loc = p.first;
    tile const& t = loc.stuff_at();
    assert(t.contents == WATER);
    if (t.is_sticky_water()) {
      p.second.new_progress[cdir_zminus] += extra_downward_speed_for_sticky_water;
    }
    else {
      vector3<sub_tile_distance> &vel_ref = p.second.velocity;
      vel_ref += gravity_acceleration;
      
      // Slight air resistance proportional to the square of the velocity (has very little effect at our current 20x20x20 scale; mostly there to make a natural cap velocity for falling water)
      vel_ref -= (vel_ref * vel_ref.magnitude()) / air_resistance_constant;
      
      // Relatively large friction against the ground
      for (EACH_CARDINAL_DIRECTION(dir)) {
        if (p.second.blockage_amount_this_frame[dir] > 0) {
          vector3<sub_tile_distance> copy_stationary_in_blocked_direction = vel_ref; copy_stationary_in_blocked_direction -= project_onto_cardinal_direction(copy_stationary_in_blocked_direction, dir);
          if (copy_stationary_in_blocked_direction.magnitude_within_32_bits_is_less_than(friction_amount)) {
            vel_ref -= copy_stationary_in_blocked_direction;
          }
          else {
            vel_ref -= copy_stationary_in_blocked_direction * friction_amount / copy_stationary_in_blocked_direction.magnitude();
          }
        }
      }
      
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const sub_tile_distance dp = vel_ref.dot<sub_tile_distance>(dir.v);
        if (dp > 0) p.second.new_progress[dir] += dp;

        // Water that's blocked, but can go around in a diagonal direction, makes "progress" towards all those possible directions (so it'll go in a random direction if it could go around in several different diagonals, without having to 'choose' one right away and gain velocity only in that direction). The main purpose of this is to make it so that water doesn't stack nicely in pillars, or sit still on steep slopes.
        const location dst_loc = loc + dir;
        if (dst_loc.stuff_at().contents == AIR) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            const location diag_loc = dst_loc + d2;
            if (p.second.blockage_amount_this_frame[d2] > 0 && d2.v.dot<sub_tile_distance>(dir.v) == 0 && diag_loc.stuff_at().contents == AIR) {
              p.second.new_progress[dir] += p.second.blockage_amount_this_frame[d2];
            }
          }
        }
      }
    }
      
    // This always happens, even if water becomes sticky right after it gets blocked.
    for (sub_tile_distance &bb : p.second.blockage_amount_this_frame) bb = 0;
  }
  
  vector<wanted_move> wanted_moves;
  for (pair<const location, water_movement_info> &p : w.active_tiles_range()) {
    location const& loc = p.first;
    tile const& t = loc.stuff_at();
    assert(t.contents == WATER);
    for (EACH_CARDINAL_DIRECTION(dir)) {
      sub_tile_distance &progress_ref = p.second.progress[dir];
      const sub_tile_distance new_progress = p.second.new_progress[dir];
      if (p.second.new_progress[dir] == 0) {
        if (progress_ref < idle_progress_reduction_rate) progress_ref = 0;
        else progress_ref -= idle_progress_reduction_rate;
      }
      else {
        assert(new_progress >= 0);
        assert(progress_ref >= 0);
        assert(progress_ref <= progress_necessary);
        progress_ref += new_progress;
        if (progress_ref > progress_necessary) {
          wanted_moves.push_back(wanted_move(loc, dir, t.is_sticky_water() ? groups.group_numbers_by_location[loc] : NO_GROUP, new_progress, progress_ref - progress_necessary));
          progress_ref = progress_necessary;
        }
      }
    }
  }
  
  std::random_shuffle(wanted_moves.begin(), wanted_moves.end());
  std::set<location> disturbed_tiles;

  vector<map<location_coordinate, literally_random_access_removable_stuff<location> > > tiles_by_z_location_by_group_number(groups.locations_by_group_number.size());
  
  for (group_number_t group_number = FIRST_GROUP; group_number < (group_number_t)groups.locations_by_group_number.size(); ++group_number) {
    unordered_set<location> nearby_free_waters;
    for (location const& loc : groups.locations_by_group_number[group_number]) {
      tiles_by_z_location_by_group_number[group_number][loc.coords().z].add(loc);
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location dst_loc = loc + dir;
        tile const& dst_tile = dst_loc.stuff_at();
        if (dst_tile.is_free_water()) {
          nearby_free_waters.insert(dst_loc);
        }
        // Hack? Include tiles connected diagonally, if there's air in between (this makes sure that water using the 'fall off pillars' rule to go into a lake is grouped with the lake)
        if (dst_tile.contents == AIR) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            if (d2.v.dot<sub_tile_distance>(dir.v) == 0) {
              const location diag_loc = dst_loc + d2;
              if (diag_loc.stuff_at().is_free_water()) {
                nearby_free_waters.insert(diag_loc);
              }
            }
          }
        }
      }
    }
    for (location const& loc : nearby_free_waters) {
      tiles_by_z_location_by_group_number[group_number][loc.coords().z].add(loc);
    }
  }
  
  for (const wanted_move move : wanted_moves) {
    const location dst = move.src + move.dir;
    tile &src_tile = move.src.stuff_at();
    tile &dst_tile = dst.stuff_at();
    tile src_copy = src_tile; // only for use in gdb
    tile dst_copy = dst_tile; // only for use in gdb
    // in certain situations we shouldn't try to move water more than once
    if (disturbed_tiles.find(move.src) != disturbed_tiles.end()) continue;
    // anything where the water was yanked away should have been marked "disturbed"
    assert(src_tile.contents == WATER);
    
    water_movement_info *fbdkopsw = w.get_active_tile(move.src);
    assert(fbdkopsw);
    water_movement_info &src_water = *fbdkopsw;
    sub_tile_distance& progress_ref = src_water.progress[move.dir];
    
    if (dst_tile.contents == AIR) {
      progress_ref -= progress_necessary;
      water_movement_info &dst_water = w.insert_water(dst);
      
      if (move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP) {
        dst_water = src_water;
        w.delete_water(move.src);
        disturbed_tiles.insert(move.src);
      }
      else {
        // the teleported tile starts with zero everything, for some reason (TODO: why should it be like this?)
        dst_water.velocity = vector3<sub_tile_distance>(0,0,0); dst_water.progress[cdir_zminus] = 0;
        
        // Pick a random top tile.
        map<location_coordinate, literally_random_access_removable_stuff<location> > m = tiles_by_z_location_by_group_number[move.group_number_or_NO_GROUP_for_velocity_movement];
        location chosen_top_tile(move.src); // really I shouldn't be constructing a location, but there's no default constructor, and for good reason !!!! !!!! !!!
        while (true) {
          assert(!m.empty());
          map<location_coordinate, literally_random_access_removable_stuff<location> >::iterator l = boost::prior(m.end());
          if (l->second.empty()) m.erase(l);
          else {
            chosen_top_tile = l->second.get_and_blacklist_random();
            if (disturbed_tiles.find(chosen_top_tile) == disturbed_tiles.end()) break;
          }
        }
        // Since the source tile is still water, there always must be a real tile to choose
        
        w.delete_water(chosen_top_tile);
        disturbed_tiles.insert(chosen_top_tile);
      }
      
      // If a tile moves, we're now content to assume that it was moving because it had a realistic velocity in that direction, so we should continue with that assumption.
      const sub_tile_distance amount_of_new_vel_in_movement_dir = dst_water.velocity.dot<sub_tile_distance>(move.dir.v);
      const sub_tile_distance deficiency_of_new_vel_in_movement_dir = move.amount_of_the_push_that_sent_us_over_the_threshold - amount_of_new_vel_in_movement_dir;
      if (deficiency_of_new_vel_in_movement_dir > 0) {
        dst_water.velocity += move.dir.v * deficiency_of_new_vel_in_movement_dir;
      }
      // Also, don't lose movement to rounding error during progress over multiple tiles:
      dst_water.progress[move.dir] = std::min(move.excess_progress, progress_necessary);
    }
    else {
      // we're blocked
      src_water.blockage_amount_this_frame[move.dir] = move.excess_progress;
      
      if (dst_tile.contents == WATER) {
        if (move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP) {
          if (dst_tile.is_sticky_water()) {
            //TODO... right now, same as rock. Dunno if it should be different.
            src_water.get_completely_blocked(move.dir);
          }
          else {
            water_movement_info &dst_water = w.activate_water(dst);
            
            const vector3<sub_tile_distance> vel_diff = src_water.velocity - dst_water.velocity;
            const vector3<sub_tile_distance> exchanged_velocity = project_onto_cardinal_direction(vel_diff, move.dir) / 2;
            src_water.velocity -= exchanged_velocity;
            dst_water.velocity += exchanged_velocity;
            // Since we *don't* move the 'progress' back then they will keep colliding, which means their velocity keeps getting equalized - that's probably a good thing.
          }
        }
        else {
          // we tried to move due to pressure, but bumped into free water! Turn our movement into velocity, at some conversion factor.
          w.activate_water(dst).velocity += (move.dir.v * move.excess_progress) / 10;
        }
      }
      else if (dst_tile.contents == ROCK) {
        // TODO figure out what to actually do about the fact that water can become sticky while having lots of progress.
        //assert(move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP);
        src_water.get_completely_blocked(move.dir);
      }
      else assert(false);
    }
  }
  
  const auto range = w.active_tiles_range();
  for (auto i = range.begin(); i != range.end(); ) {
    if (i->second.can_deactivate()) {
      location const& loc = i->first;
      ++i;
      w.deactivate_water(loc);
    }
    else ++i;
  }
}

