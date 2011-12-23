
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <map>
#include <set>

#include <iostream>
#include <inttypes.h>

using std::map;
using std::pair;
using std::make_pair;
using std::set;
using std::vector;

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

//typedef bounds_checked_int scalar_type;
//typedef int64_t scalar_type;
typedef int scalar_type;

int divide_rounding_towards_zero(int dividend, int divisor)
{
	assert(divisor != 0);
	int abs_result = std::abs(dividend) / std::abs(divisor);
	if ((dividend > 0 && divisor > 0) || (dividend < 0 && divisor < 0)) return abs_result;
	else return -abs_result;
}

int sign(int number)
{
	if (number > 0) return 1;
	if (number < 0) return -1;
	return 0;
}


class vector3 {
public:
	scalar_type x, y, z;
	vector3():x(0),y(0),z(0){}
	vector3(int x, int y, int z):x(x),y(y),z(z){}
	vector3 operator+(vector3 const& other)const {
		return vector3(x + other.x, y + other.y, z + other.z);
	}
	void operator+=(vector3 const& other) {
		x += other.x; y += other.y; z += other.z;
	}
	vector3 operator-(vector3 const& other)const {
		return vector3(x - other.x, y - other.y, z - other.z);
	}
	void operator-=(vector3 const& other) {
		x -= other.x; y -= other.y; z -= other.z;
	}
	vector3 operator*(int other)const {
		return vector3(x * other, y * other, z * other);
	}
	void operator*=(int other) {
		x *= other; y *= other; z *= other;
	}
	vector3 operator/(int other)const {
		return vector3(divide_rounding_towards_zero(x, other), divide_rounding_towards_zero(y, other), divide_rounding_towards_zero(z, other));
	}
	void operator/=(int other) {
		x = divide_rounding_towards_zero(x, other); y = divide_rounding_towards_zero(y, other); z = divide_rounding_towards_zero(z, other);
	}
	vector3 operator-()const { // unary minus
		return vector3(-x, -y, -z);
	}
	bool operator==(vector3 const& other)const {return x == other.x && y == other.y && z == other.z; }
	bool operator!=(vector3 const& other)const {return x != other.x || y != other.y || z != other.z; }
	scalar_type dot(vector3 const& other)const {
		return x * other.x + y * other.y + z * other.z;
	}
	scalar_type magnitude()const { return std::sqrt((int)dot(*this));}
	bool operator<(vector3 const& other)const { return (x < other.x) || ((x == other.x) && ((y < other.y) || ((y == other.y) && (z < other.z)))); }
};

typedef vector3 one_tile_direction_vector;
typedef vector3 location;

vector3 project_onto_cardinal_direction(vector3 src, one_tile_direction_vector dir) {
  return vector3(src.x * std::abs(dir.x), src.y * std::abs(dir.y), src.z * std::abs(dir.z));
}

#define MAX_X 20
#define MAX_Y 20
#define MAX_Z 20

void incr_location(location &foo){
	++foo.x;
	if (foo.x >= MAX_X) { foo.x = 0; ++foo.y; }
	if (foo.y >= MAX_Y) { foo.y = 0; ++foo.z; }
}
void decr_location(location &foo){
	--foo.x;
	if (foo.x < 0) { foo.x = MAX_X - 1; --foo.y; }
	if (foo.y < 0) { foo.y = MAX_Y - 1; --foo.z; }
}
void incr_direction(one_tile_direction_vector &foo){
	++foo.x;
	if (foo.x == 0 && foo.y == 0 && foo.z == 0) ++foo.x;
	if (foo.x >= 2) { foo.x = -1; ++foo.y; }
	if (foo.y >= 2) { foo.y = -1; ++foo.z; }
}
void incr_cardinal_direction(one_tile_direction_vector &foo){
	     if (foo.x == -1) foo = vector3( 0,-1, 0);
	else if (foo.y == -1) foo = vector3( 0, 0,-1);
	else if (foo.z == -1) foo = vector3( 1, 0, 0);
	else if (foo.x == 1) foo = vector3( 0, 1, 0);
	else if (foo.y == 1) foo = vector3( 0, 0, 1);
	else if (foo.z == 1) foo = vector3( -2, 0, 0); // hack - "invalid value"
}

#define EACH_LOCATION(varname) location varname(0,0,0); varname.z < MAX_Z; incr_location(varname)
#define EACH_LOCATION_REVERSE(varname) location varname(MAX_X - 1,MAX_Y - 1,MAX_Z - 1); varname.z >= 0; decr_location(varname)
#define EACH_DIRECTION(varname) one_tile_direction_vector varname(-1,-1,-1); varname.z < 2; incr_direction(varname)
#define EACH_CARDINAL_DIRECTION(varname) one_tile_direction_vector varname(-1,0,0); varname.x != -2; incr_cardinal_direction(varname)

//const scalar_type precision_scale = 1000;
// This constant makes an object be travelling at one tile per frame when it hits the ground after a 100 tile fall.
//vector3 default_gravity_in_subdivisions_per_frame_squared(0, 0, -(precision_scale / 200));
//#define NUM_WATER_UNITS

#if 0
scalar_type stupid_distance(one_tile_direction_vector d)
{
	int grid_dist = std::abs(d.x) + std::abs(d.y) + std::abs(d.z);
	     if (grid_dist == 0)  return 0;
	else if (grid_dist == 1)  return precision_scale;
	else if (grid_dist == 2)  return precision_scale *  92682 / (1 << 16);
	else /*(grid_dist == 3)*/ return precision_scale * 113512 / (1 << 16);
}
#endif


/*struct water_tile
{
	vector3 velocity[NUM_WATER_UNITS]; // in tiles per frame, scaled up by precision_scale
	vector3 progress[NUM_WATER_UNITS]; // From -precision_scale/2 (bordering the negative-direction tile) to precision_scale/2 (bordering the positive-direction tile).
	scalar_type pressure;
	scalar_type amount;
	water_tile():velocity(0,0,0),total_force(0),amount(0){}
};*/

enum tile_contents {
  ROCK,
  WATER,
  AIR
};

struct tile
{
  tile_contents contents;
  vector3 water_velocity;
  int water_group_number; // a temporary variable during the computation
};

bool out_of_bounds(location loc){return loc.x < 0 || loc.y < 0 || loc.z < 0 || loc.x >= MAX_X || loc.y >= MAX_Y || loc.z >= MAX_Z; }

struct tiles_type {
	tile tiles[MAX_X][MAX_Y][MAX_Z];
	tile& operator[](location loc){ assert(!out_of_bounds(loc)); return tiles[loc.x][loc.y][loc.z]; }
	tile const& operator[](location loc)const { assert(!out_of_bounds(loc)); return tiles[loc.x][loc.y][loc.z]; }
};
tiles_type tiles;


bool can_group(location loc) {
  if (out_of_bounds(loc)) return false;
  if (tiles[loc].contents != WATER) return false;
  return true;
}

bool can_be_exit_tile(location loc) {
  if (!can_group(loc)) return false;
  int airs = 0;
  for (EACH_CARDINAL_DIRECTION(dir)) {
    if (!out_of_bounds(loc + dir) && tiles[loc + dir].contents == AIR) ++airs;
  }
  if (airs > 1) return false;
  return true;
}

struct water_group {
  int max_tile_z;
  int max_exit_tile_z;
  std::set<location> tiles;
  set<pair<location, one_tile_direction_vector> > exit_surfaces;
  water_group():max_tile_z(0),max_exit_tile_z(0){}
};

void mark_water_group_that_includes(location loc, int group_number, map<int, water_group>& groups) {
  set<location> frontier;
  frontier.insert(loc);
  groups.insert(make_pair(group_number, water_group()));
  while(!frontier.empty())
  {
    const location next_loc = *(frontier.begin());
    frontier.erase(frontier.begin());
    
    tiles[next_loc].water_group_number = group_number;
    groups[group_number].tiles.insert(next_loc);
    if (next_loc.z > groups[group_number].max_tile_z) groups[group_number].max_tile_z = next_loc.z;
    
    //bool is_exit_tile = false;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const location adj_loc = next_loc + dir;
      if (!out_of_bounds(adj_loc)) {
        tile &adj_tile = tiles[adj_loc];
        if (can_group(adj_loc) && adj_tile.water_group_number == 0) {
          frontier.insert(adj_loc);
        }
        if (can_be_exit_tile(next_loc) && (adj_tile.contents == AIR || (adj_tile.contents == WATER && !can_be_exit_tile(adj_loc)))) {
          groups[group_number].exit_surfaces.insert(std::make_pair(next_loc, dir));
          if (next_loc.z > groups[group_number].max_exit_tile_z) groups[group_number].max_exit_tile_z = next_loc.z;
          //is_exit_tile = true;
        }
      }
    }
    //if (is_exit_tile && can_be_exit_tile(next_loc)) exit_tiles_by_group[group_number].insert(next_loc);
  }
}

void update_water() {
  for (EACH_LOCATION(loc)) {
    tiles[loc].water_group_number = 0;
  }
  int next_group_number = 1;
  map<int, water_group > groups;
  for (EACH_LOCATION(loc)) {
    if(can_group(loc) && tiles[loc].water_group_number == 0) {
      mark_water_group_that_includes(loc, ++next_group_number, groups);
    }
  }
  for (auto i = groups.begin(); i != groups.end(); ++i) {
    const int group_number = i->first;
    water_group const& group = i->second;
    vector<location> top_tiles;
    set<pair<location, one_tile_direction_vector> > non_top_surfaces;
    for (auto j = group.tiles.begin(); j != group.tiles.end(); ++j) {
      if (j->z == group.max_tile_z) top_tiles.push_back(*j);
    }
    
    for(auto surface = group.exit_surfaces.begin(); surface != group.exit_surfaces.end(); ++surface) {
      const double pressure = group.max_exit_tile_z + 0.5 - ((double)surface->first.z + 0.5*surface->second.z); // proportional to depth, assuming side surfaces are at the middle of the side
      tile &surface_target_tile = tiles[surface->first + surface->second];
      if (surface_target_tile.contents == AIR) {
        if (rand()%5000 < 10 * std::sqrt(pressure)) {
          tiles[top_tiles[rand()%top_tiles.size()]].contents = AIR;
          surface_target_tile.contents = WATER;
          surface_target_tile.water_velocity = vector3(0,0,0);
        }
      }
      else if (surface_target_tile.contents == WATER) {
        surface_target_tile.water_velocity += surface->second * (int)(10 * std::sqrt(pressure));
      }
      else assert(false);
    }
  }
  
  for (EACH_LOCATION(loc)) {
    if (can_be_exit_tile(loc)) {
      tiles[loc].water_velocity = vector3(0,0,0);
    }
  }
  
  for (EACH_LOCATION(loc)) {
    if (tiles[loc].contents == WATER && !can_be_exit_tile(loc)) {
      tiles[loc].water_velocity.z -= 5;
      one_tile_direction_vector travel_dir(0,0,0);
      int randvalue = rand()%145600;
      if (randvalue < std::abs(tiles[loc].water_velocity.x)) travel_dir = one_tile_direction_vector(1,0,0) * sign(tiles[loc].water_velocity.x);
      else if (randvalue < std::abs(tiles[loc].water_velocity.x) + std::abs(tiles[loc].water_velocity.y)) travel_dir = one_tile_direction_vector(0,1,0) * sign(tiles[loc].water_velocity.y);
      else if (randvalue < std::abs(tiles[loc].water_velocity.x) + std::abs(tiles[loc].water_velocity.y) + std::abs(tiles[loc].water_velocity.z)) travel_dir = one_tile_direction_vector(0,0,1) * sign(tiles[loc].water_velocity.z);
      
      if (loc + travel_dir != loc) {
        if (out_of_bounds(loc + travel_dir) || tiles[loc + travel_dir].contents == ROCK) {
          tiles[loc].water_velocity -= project_onto_cardinal_direction(tiles[loc].water_velocity, travel_dir);
        }
        else if (tiles[loc + travel_dir].contents == AIR) {
          tiles[loc].contents = AIR;
          tiles[loc + travel_dir].contents = WATER;
          tiles[loc + travel_dir].water_velocity = tiles[loc].water_velocity;
        }
        else if (tiles[loc + travel_dir].contents = WATER) {
          if (can_be_exit_tile(loc + travel_dir)) {
            //TODO... right now, same as rock
            tiles[loc].water_velocity -= project_onto_cardinal_direction(tiles[loc].water_velocity, travel_dir);
          }
          else {
            const vector3 vel_diff = tiles[loc].water_velocity - tiles[loc + travel_dir].water_velocity;
            const vector3 exchanged_velocity = project_onto_cardinal_direction(vel_diff, travel_dir) / 2;
            tiles[loc].water_velocity -= exchanged_velocity;
            tiles[loc + travel_dir].water_velocity += exchanged_velocity;
          }
        }
        else assert(false);
      }
    }
  }
}

