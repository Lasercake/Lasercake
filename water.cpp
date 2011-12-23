
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
  int water_group_number; // a temporary variable during the computation
};

bool out_of_bounds(location loc){return loc.x < 0 || loc.y < 0 || loc.z < 0 || loc.x >= MAX_X || loc.y >= MAX_Y || loc.z >= MAX_Z; }

struct tiles_type {
	tile tiles[MAX_X][MAX_Y][MAX_Z];
	tile& operator[](location loc){ assert(!out_of_bounds(loc)); return tiles[loc.x][loc.y][loc.z]; }
	tile const& operator[](location loc)const { assert(!out_of_bounds(loc)); return tiles[loc.x][loc.y][loc.z]; }
};
tiles_type tiles;

// 'velocity' in a diagonal direction is scaled up (i.e. velocity going to (1,1,1) is sqrt(3) times as gret as the same amount going in a cardinal direction)
scalar_type outpushing_velocity[MAX_X][MAX_Y][MAX_Z][3][3][3];
scalar_type outgoing_forces_from_pressure[MAX_X][MAX_Y][MAX_Z][3][3][3];
scalar_type outgoing_water[MAX_X][MAX_Y][MAX_Z][3][3][3];



void mark_water_group_that_includes(location loc, int group_number, map<int, set<location> >& edge_tiles_by_group) {
  set<location> frontier;
  frontier.insert(loc);
  edge_tiles_by_group.insert(make_pair(group_number, set<location>()));
  while(!frontier.empty())
  {
    const location next_loc = *(frontier.begin());
    frontier.erase(frontier.begin());
    tiles[next_loc].water_group_number = group_number;
    bool is_edge_tile = false;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const location adj_loc = next_loc + dir;
      if (!out_of_bounds(adj_loc)) {
        tile &adj_tile = tiles[adj_loc];
        if (adj_tile.contents == WATER && adj_tile.water_group_number == 0) {
          frontier.insert(adj_loc);
        }
        if (adj_tile.contents == AIR) {
         is_edge_tile = true;
        }
      }
    }
    if (is_edge_tile) edge_tiles_by_group[group_number].insert(next_loc);
  }
}

void update_water() {
  for (EACH_LOCATION(loc)) {
    tiles[loc].water_group_number = 0;
  }
  int next_group_number = 1;
  map<int, set<location> > edge_tiles_by_group;
  for (EACH_LOCATION(loc)) {
    if(tiles[loc].contents == WATER && tiles[loc].water_group_number == 0) {
      mark_water_group_that_includes(loc, ++next_group_number, edge_tiles_by_group);
    }
  }
  for (map<int, set<location> >::const_iterator i = edge_tiles_by_group.begin(); i != edge_tiles_by_group.end(); ++i) {
    const int group_number = i->first;
    const set<location> edge_tiles = i->second;
    int max_z = 0;
    vector<location> top_tiles;
    for (set<location>::const_iterator j = edge_tiles.begin(); j != edge_tiles.end(); ++j) {
      if (j->z > max_z) max_z = j->z;
    }
    set<pair<location, one_tile_direction_vector> > non_top_surfaces;
    for (set<location>::const_iterator j = edge_tiles.begin(); j != edge_tiles.end(); ++j) {
      if (j->z == max_z) top_tiles.push_back(*j);
      for (EACH_CARDINAL_DIRECTION(dir)) {
        if (!(j->z == max_z && dir.z == 1)) {
          const location adj_loc = *j + dir;
          if (!out_of_bounds(adj_loc)) {
            if (tiles[adj_loc].contents == AIR) non_top_surfaces.insert(make_pair(*j, dir));
          }
        }
      }
    }
    
    for(set<pair<location, one_tile_direction_vector> >::const_iterator surface = non_top_surfaces.begin(); surface != non_top_surfaces.end(); ++surface) {
      if (rand()%10000 < 10 * std::sqrt(max_z + 0.5 - ((double)surface->first.z + 0.5*surface->second.z))) {
        tiles[top_tiles[rand()%top_tiles.size()]].contents = AIR;
        tiles[surface->first + surface->second].contents = WATER;
        break;
      }
    }
  }
}

