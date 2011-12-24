
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <map>
#include <set>
#include <algorithm>

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

// This constant makes an object be travelling at one tile per frame when it hits the ground after a 100 tile fall.
//vector3 default_gravity_in_subdivisions_per_frame_squared(0, 0, -(precision_scale / 200));


enum tile_contents {
  ROCK,
  WATER,
  AIR
};

struct water_movement_info {
  vector3 velocity;
  scalar_type progress[3][3][3];
  water_movement_info(){
    for(int x=0;x<3;++x){for(int y=0;y<3;++y){for(int z=0;z<3;++z){
      progress[x][y][z] = 0;
    }}}
  }
  water_movement_info& operator=(water_movement_info const& other){
    velocity = other.velocity;
    for(int x=0;x<3;++x){for(int y=0;y<3;++y){for(int z=0;z<3;++z){
      progress[x][y][z] = other.progress[x][y][z];
    }}}
  }
};
const scalar_type progress_necessary = 5000;

struct tile
{
  tile_contents contents;
  water_movement_info water_movement;
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
  std::set<location> exit_tiles;
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
    
    bool is_exit_tile = false;
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
          is_exit_tile = true;
        }
      }
    }
    if (is_exit_tile) groups[group_number].exit_tiles.insert(next_loc);
  }
}

struct wanted_move {
  location src;
  one_tile_direction_vector dir;
  int group_number_or_zero_for_velocity_movement;
  scalar_type amount_of_the_push_that_sent_us_over_the_threshold;
  wanted_move(location src,one_tile_direction_vector dir,int g,scalar_type a):src(src),dir(dir),group_number_or_zero_for_velocity_movement(g),amount_of_the_push_that_sent_us_over_the_threshold(a){}
};

// WARNING: This is glitchy (or at least silly) if you do it more than once for the same loc/dir pair. TODO adjust?
void do_progress(vector<wanted_move> &wanted_moves, location loc, one_tile_direction_vector dir, int group_number_or_zero_for_velocity_movement, scalar_type amount) {
  scalar_type &progress_ref = tiles[loc].water_movement.progress[1+dir.x][1+dir.y][1+dir.z];
  assert(amount >= 0);
  assert(progress_ref >= 0);
  progress_ref += amount;
  if (progress_ref > progress_necessary)
    wanted_moves.push_back(wanted_move(loc, dir, group_number_or_zero_for_velocity_movement, amount));
}

void update_water() {
  for (EACH_LOCATION(loc)) {
    tiles[loc].water_group_number = 0;
  }
  int next_group_number = 1;
  map<int, water_group> groups;
  for (EACH_LOCATION(loc)) {
    if(can_group(loc) && tiles[loc].water_group_number == 0) {
      mark_water_group_that_includes(loc, ++next_group_number, groups);
    }
  }

  vector<wanted_move> wanted_moves;
  for (auto i = groups.begin(); i != groups.end(); ++i) {
    const int group_number = i->first;
    water_group const& group = i->second;
    
    for(auto surface = group.exit_surfaces.begin(); surface != group.exit_surfaces.end(); ++surface) {
      const double pressure = group.max_exit_tile_z + 0.5 - ((double)surface->first.z + 0.5*surface->second.z); // proportional to depth, assuming side surfaces are at the middle of the side
      tile &surface_source_tile = tiles[surface->first];
      do_progress(wanted_moves, surface->first, surface->second, tiles[surface->first].water_group_number, (scalar_type)(100 * std::sqrt(pressure)));
    }
  }
  
  for (EACH_LOCATION(loc)) {
    if (tiles[loc].contents == WATER && !can_be_exit_tile(loc)) {
      bool already_at_the_bottom = (tiles[loc].water_movement.progress[1+0][1+0][1-1] >= progress_necessary);
      
      tiles[loc].water_movement.velocity.z -= 5;
      
      // Slight air resistance proportional to the square of the velocity
      
      
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const scalar_type dp = tiles[loc].water_movement.velocity.dot(dir);
        scalar_type new_progress = 0;
        if (dp > 0) new_progress += dp;

        // Hack: Water sitting on pillars falls off
        const one_tile_direction_vector down(0, 0, -1);
        if (already_at_the_bottom && dir.z == 0 && !out_of_bounds(loc + dir) && tiles[loc + dir].contents == AIR && !out_of_bounds(loc + dir + down) && tiles[loc + dir + down].contents == AIR) new_progress += 500;
        
        do_progress(wanted_moves, loc, dir, 0, new_progress);
      }
    }
  }
  
  std::random_shuffle(wanted_moves.begin(), wanted_moves.end());
  std::set<location> disturbed_tiles;
  
  for (const wanted_move move : wanted_moves) {
    const location dst = move.src + move.dir;
    tile &src_tile = tiles[move.src];
    // in certain situations we shouldn't try to move water more than once
    if (disturbed_tiles.find(move.src) != disturbed_tiles.end()) continue;
    // anything where the water was yanked away should have been marked "disturbed"
    assert(src_tile.contents = WATER);
    
    scalar_type& progress_ref = src_tile.water_movement.progress[1+move.dir.x][1+move.dir.y][1+move.dir.z];
    const scalar_type excess_progress = progress_ref - progress_necessary;
    assert(excess_progress >= 0);
    
    if (out_of_bounds(dst)) {
      // TODO remove this duplicate code: we behave the same as going to rock
      // TODO figure out what to actually do about the fact that water can change to exit-tile-capable while having lots of progress.
      //assert(move.group_number_or_zero_for_velocity_movement == 0);
      if (src_tile.water_movement.velocity.dot(move.dir) > 0)
        src_tile.water_movement.velocity -= project_onto_cardinal_direction(src_tile.water_movement.velocity, move.dir);
      progress_ref = progress_necessary;
      continue;
    }
    
    tile &dst_tile = tiles[dst];
    
    tile src_copy = src_tile;
    tile dst_copy = dst_tile;
    
    if (dst_tile.contents == AIR) {
      progress_ref -= progress_necessary;
      dst_tile.contents = WATER;
      
      if (move.group_number_or_zero_for_velocity_movement == 0) {
        src_tile.contents = AIR;
        disturbed_tiles.insert(move.src);
        dst_tile.water_movement = src_tile.water_movement;
      }
      else {
        dst_tile.water_movement = water_movement_info(); // the teleported tile starts with zero everything
        
        // HACK - pick a random top tile in an inefficient way. (We have to recompute stuff in case a lot of tiles were yanked from the top at once.)
        location chosen_top_tile = location(-1,-1,-1);
        int num_top_tiles = 0;
        for (const location loc : groups[move.group_number_or_zero_for_velocity_movement].tiles) {
          if (tiles[loc].contents != WATER) continue;
          if (loc.z > chosen_top_tile.z) {
            chosen_top_tile = loc;
            num_top_tiles = 1;
          }
          else if (loc.z == chosen_top_tile.z) {
            ++num_top_tiles;
            if (rand()%num_top_tiles == 0) {
              chosen_top_tile = loc;
            }
          }
        }
        // Since the source tile is still water, there always must be a real tile to choose
        assert(num_top_tiles > 0);
        
        tiles[chosen_top_tile].contents = AIR;
        disturbed_tiles.insert(chosen_top_tile);
      }
      
      // If a tile moves, we're now content to assume that it was moving because it had a realistic velocity in that direction, so we should continue with that assumption.
      const scalar_type amount_of_new_vel_in_movement_dir = dst_tile.water_movement.velocity.dot(move.dir);
      const scalar_type deficiency_of_new_vel_in_movement_dir = move.amount_of_the_push_that_sent_us_over_the_threshold - amount_of_new_vel_in_movement_dir;
      if (deficiency_of_new_vel_in_movement_dir > 0) {
        dst_tile.water_movement.velocity += move.dir * deficiency_of_new_vel_in_movement_dir;
      }
    }
    else if (dst_tile.contents == WATER) {
      if (move.group_number_or_zero_for_velocity_movement == 0) {
        if (can_be_exit_tile(dst)) {
          //TODO... right now, same as rock
          if (src_tile.water_movement.velocity.dot(move.dir) > 0)
            src_tile.water_movement.velocity -= project_onto_cardinal_direction(src_tile.water_movement.velocity, move.dir);
          progress_ref = progress_necessary;
        }
        else {
          const vector3 vel_diff = src_tile.water_movement.velocity - dst_tile.water_movement.velocity;
          const vector3 exchanged_velocity = project_onto_cardinal_direction(vel_diff, move.dir) / 2;
          src_tile.water_movement.velocity -= exchanged_velocity;
          dst_tile.water_movement.velocity += exchanged_velocity;
          // hmm... since we *don't* move the 'progress' back then they will keep colliding... that might be a good thing? Well, only if we don't let the progress build up extra.
          // TODO: should the velocity-sharing be proportional to the excess progress?
          progress_ref = progress_necessary;
        }
      }
      else {
        // we tried to move due to pressure, but bumped into free water! Turn our movement into velocity, at some conversion factor.
        dst_tile.water_movement.velocity += (move.dir * excess_progress) / 10;
        progress_ref -= excess_progress;
        assert(progress_ref == progress_necessary);
      }
    }
    else if (dst_tile.contents == ROCK) {
      // TODO figure out what to actually do about the fact that water can change to exit-tile-capable while having lots of progress.
      // Also note that this code is currently duplicated in three places...
      //assert(move.group_number_or_zero_for_velocity_movement == 0);
      if (src_tile.water_movement.velocity.dot(move.dir) > 0)
        src_tile.water_movement.velocity -= project_onto_cardinal_direction(src_tile.water_movement.velocity, move.dir);
      progress_ref = progress_necessary;
    }
    assert(progress_ref >= 0);
    assert(progress_ref <= progress_necessary);
  }
}

