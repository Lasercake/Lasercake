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

/*

Eli says: The following is a description of how the water rules work,
written by Isaac and slightly edited by myself. I'm not terribly
satisfied with it, but it's waaaay better than not having a description at all.
I want to write my own description of the water rules, but I'm too busy!
Remind me to do that sometime.

* * * * *

Water is simulated something like this:

Water is tile-based.  There's a 3D grid of tiles, and each of them
either is water or something else (like rock or air).  Tiles are significantly
shorter than they are wide; nothing relies on the exact tile width and tile height,
but they affect the physics. (It takes longer to move vertically by a tile
if the height is greater - I don't think there are any other rules that refer to the
width and height right now.)

Each water tile can be "sticky" or "free" water.  This affects
how it behaves.  Water is generally "free" water if more than one of the
six adjacent tiles is air; otherwise it's sticky.

"free" water tiles move individually based on velocity, and "sticky" water tiles
are computed as contiguous groups of sticky water. Contiguousness is measured in the
six orthogonal directions.

A water's freeness or stickiness has a special 1-frame delay between when it meets
the conditions for freeness/stickyness (the amount of air nearby) and when it becomes
free/sticky. This makes sticky water flow more smoothly (it doesn't lose its group
properties by snapping to free water in some situations.)

Imagine it were 2D.

W=water, #=rock, space=air:

 W     W
## WWW
##WWWWWW#
##WWWWWW#
###WWWW##
#########

That's, F=free water, S=sticky water,

 F     F
## FSF
##SSSSSS#
##SSSSSS#
###SSSS##
#########

[[[[
As an optimization, we also define "interior water" as that sticky water
which has sticky water on all six sides ("I" below) :

 F     F
## FSF
##SSISSS#
##SIIIIS#
###SSSS##
#########

Air/rock/water/free/sticky/interior are stored in the tile's byte.
Interior is just a cache that is updated whenever an interior tile
or any of its neighbors changes.
]]]]

Sticky-water groups exert pressure on their neighbors.  This pressure
is similar to z_top - z, where z_top is the top z coordinate in the
water group, and z is the particular neighbor's z coordinate.
Pressure outward from the edge of a group causes a water tile to
make "progress" in that direction (a water can make progress in
several contradictory-seeming directions at once).  Then if a water
tile has made sufficient progress in a direction, and sees an air in that direction,
it "flows" into that direction.  To "flow" means that a water tile teleports
from a random one of the highest (in z) water-tiles in this group,
to this air location. The more pressure, the more velocity a
free water has when it comes out. If a sticky water tile makes sufficient
progress in a direction but there's free water in the way, it pushes that
free water away from it.

There are special rules for free and sticky water to slide off the
edges of tiles (in the above example, the top left free-water will
slide towards the lake despite comfortably sitting still on top
of a nice rectangular rock).  Free water has slight air resistance in air,
and significant friction when it's running into something.

[[[[
As another optimization, we define "active water" as water which might
plausibly do something.  Water starts out presumed to be active, and
we must (for performance reasons) make sure that any water that reaches
a stable state becomes inactive.  "Active water" is stored in world as a
hash from tile_location to active_water_temporary_data; the temporary data
can be discarded for water which reaches a stable state.

Free water becomes inactive when it has zero x/y velocity, rock/water below it,
and the stable amount of down-velocity caused by gravity and the thing below
it pushing back against gravity.

Sticky water becomes inactive when it isn't exerting pressure in any direction.
Sticky water doesn't use its own velocity, but we have it keep any velocity it
had as free water for a little while so that it doesn't get weird if a tile
becomes sticky and quickly becomes free again. Sticky water stays "active" until
its lingering velocity goes away.
]]]]

Because of the group rules, water can flow efficiently down tunnels; only in some cases
is it slowed down by being forced through a particularly narrow hole/tunnel
Basically, it's slowed down in those cases where the exit-surface is narrow.
This is one reason that it's important that water tend to become free when it
spurts out a small hole, so that it can't easily, right outside the hole,
form part of a large exit-surface for the large water group.

These shenanigans make the water tend to be more realistic,
and don't add high simulation cost.

Water without the grouping/pressure mechanic flows far too slowly. If we
only had water propagate pressure/velocity to adjacent tiles each frame, then it
would have terrible difficulty flowing down long, thin tunnels.
The grouping/pressure mechanism does admirably at handling a wide variety of cases,
and also is able to completely skip simulating the "interior" water-tiles that might
make up the mainstay of the volume of a lake.

*/


#include "world.hpp"

bool should_be_sticky(tile_location loc) {
  if (loc.stuff_at().contents() != WATER) return false;
  
  int airs = 0;
  for (EACH_CARDINAL_DIRECTION(dir)) {
    const tile_location other_loc = loc.get_neighbor(dir, CONTENTS_ONLY);
    if (other_loc.stuff_at().contents() == AIR) ++airs;
  }
  return (airs <= 1);
}


void world::deactivate_water(tile_location const& loc) {
  active_water_tiles.erase(loc);
}

water_movement_info& world::activate_water(tile_location const& loc) {
  assert(loc.stuff_at().contents() == WATER);
  
  auto water_iter = active_water_tiles.find(loc);
  if (water_iter == active_water_tiles.end()) {
    return active_water_tiles[loc]; // inserts it, default-constructed
  }
  else return water_iter->second;
}

void water_movement_info::get_completely_blocked(cardinal_direction dir) {
  const sub_tile_distance dp = velocity.dot<sub_tile_distance>(dir.v);
  const sub_tile_distance blocked_velocity = dp - min_convincing_speed;
  if (blocked_velocity > 0) {
    velocity -= vector3<sub_tile_distance>(dir.v) * blocked_velocity;
  }
}
  
bool water_movement_info::is_in_inactive_state()const {
  // TODO: does it make sense that we're ignoring the 1-frame-duration variable "blockage_amount_this_frame"?
  for (EACH_CARDINAL_DIRECTION(dir)) {
    if (dir.v.z < 0) {
      if (progress[dir] != progress_necessary(dir)) return false;
    }
    else {
      if (progress[dir] != 0) return false;
    }
  }
  return velocity == inactive_water_velocity;
}



typedef int group_number_t;
const group_number_t NO_GROUP = -1;
const group_number_t FIRST_GROUP = 0;




struct water_groups_structure {
  unordered_map<tile_location, group_number_t> group_numbers_by_tile_location;
  vector<unordered_set<tile_location> > tile_locations_by_group_number;
  vector<unordered_set<tile_location> > nearby_free_waters_by_group_number;
  vector<tile_coordinate> max_z_including_nearby_free_waters_by_group_number;
};

void collect_membrane(tile_location const& loc, group_number_t group_number, water_groups_structure &result) {
  vector<tile_location> frontier;
  frontier.push_back(loc);
  while(!frontier.empty())
  {
    const tile_location next_loc = frontier.back();
    frontier.pop_back();
    if (result.group_numbers_by_tile_location.find(next_loc) == result.group_numbers_by_tile_location.end()) {
      result.group_numbers_by_tile_location.insert(make_pair(next_loc, group_number));
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const tile_location adj_loc = next_loc + dir;
        if (adj_loc.stuff_at().is_membrane_water()) {
          frontier.push_back(adj_loc);
        }
        // Handle a tricky situation
        if (adj_loc.stuff_at().is_interior_water()) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            const tile_location adj_loc_2 = adj_loc + d2;
            if (adj_loc_2.stuff_at().is_membrane_water()) {
              frontier.push_back(adj_loc_2);
            }
          }
        }
      }
    }
  }
}

bool columns_sorter_lt(pair<tile_location, group_number_t> const& i, pair<tile_location, group_number_t> const& j) {
  vector3<tile_coordinate> c1 = i.first.coords();
  vector3<tile_coordinate> c2 = j.first.coords();
  return (c1.x < c2.x) || ((c1.x == c2.x) && ((c1.y < c2.y) || ((c1.y == c2.y) && (c1.z < c2.z))));
}

// the argument must start out default-initialized
void compute_groups_that_need_to_be_considered(world &w, water_groups_structure &result) {
  group_number_t next_membrane_number = FIRST_GROUP;
  for (pair<const tile_location, water_movement_info> const& p : w.active_water_tiles_range()) {
    tile_location const& loc = p.first;
    if (loc.stuff_at().is_membrane_water() && result.group_numbers_by_tile_location.find(loc) == result.group_numbers_by_tile_location.end()) {
      collect_membrane(loc, next_membrane_number, result);
      ++next_membrane_number;
    }
  }
  
  // Make a list of locations, sorted by X then Y then Z
  vector<pair<tile_location, group_number_t> > columns; columns.reserve(result.group_numbers_by_tile_location.size());
  columns.insert(columns.end(), result.group_numbers_by_tile_location.begin(), result.group_numbers_by_tile_location.end());
  std::sort(columns.begin(), columns.end(), columns_sorter_lt);
  
  group_number_t containing_group = NO_GROUP;
  for (pair<tile_location, group_number_t> const& p : columns) {
    if ((p.first + cdir_zminus).stuff_at().is_sticky_water()) {
      // The tile we've just hit is the boundary of a bubble in the larger group (or it's extra bits of the outer shell, in which case the assignment does nothing).
      assert(containing_group != NO_GROUP);
      result.group_numbers_by_tile_location[p.first] = containing_group;
    }
    else {
      // We're entering a group from the outside, which means it functions as a containing group.
      containing_group = p.second;
    }
  }
  
  // Now we might have some sort of group numbers like "1, 2, 7" because they've consolidated in essentially a random order. Convert them to a nice set of indices like 0, 1, 2.
  map<group_number_t, group_number_t> key_compacting_map;
  group_number_t next_group_number = FIRST_GROUP;
  for (pair<const tile_location, group_number_t> &p : result.group_numbers_by_tile_location) {
    tile_location const& loc = p.first;
    if (group_number_t *new_number = find_as_pointer(key_compacting_map, p.second)) {
      p.second = *new_number;
    }
    else {
      p.second = key_compacting_map[p.second] = next_group_number++; // Bwa ha ha ha ha.
      result.tile_locations_by_group_number.push_back(unordered_set<tile_location>());
      result.nearby_free_waters_by_group_number.push_back(unordered_set<tile_location>());
      result.max_z_including_nearby_free_waters_by_group_number.push_back(loc.coords().z);
    }
    result.tile_locations_by_group_number[p.second].insert(loc);
    result.max_z_including_nearby_free_waters_by_group_number[p.second] = std::max(result.max_z_including_nearby_free_waters_by_group_number[p.second], loc.coords().z);
    
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const tile_location dst_loc = loc + dir;
      tile const& dst_tile = dst_loc.stuff_at();
      if (dst_tile.is_free_water()) {
        result.nearby_free_waters_by_group_number[p.second].insert(dst_loc);
        result.max_z_including_nearby_free_waters_by_group_number[p.second] = std::max(result.max_z_including_nearby_free_waters_by_group_number[p.second], dst_loc.coords().z);
      }
      // Hack? Include tiles connected diagonally, if there's air in between (this makes sure that water using the 'fall off pillars' rule to go into a lake is grouped with the lake)
      // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
      if (dst_tile.contents() == AIR) {
        for (EACH_CARDINAL_DIRECTION(d2)) {
          if (d2.v.dot<sub_tile_distance>(dir.v) == 0) {
            const tile_location diag_loc = dst_loc + d2;
            if (diag_loc.stuff_at().is_free_water()) {
              result.nearby_free_waters_by_group_number[p.second].insert(diag_loc);
              result.max_z_including_nearby_free_waters_by_group_number[p.second] = std::max(result.max_z_including_nearby_free_waters_by_group_number[p.second], diag_loc.coords().z);
            }
          }
        }
      }
    }
  }
}


struct wanted_move {
  tile_location src;
  cardinal_direction dir;
  group_number_t group_number_or_NO_GROUP_for_velocity_movement;
  sub_tile_distance amount_of_the_push_that_sent_us_over_the_threshold;
  sub_tile_distance excess_progress;
  wanted_move(tile_location src,cardinal_direction dir,group_number_t g,sub_tile_distance a,sub_tile_distance e):src(src),dir(dir),group_number_or_NO_GROUP_for_velocity_movement(g),amount_of_the_push_that_sent_us_over_the_threshold(a),excess_progress(e){}
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

struct active_water_temporary_data {
  active_water_temporary_data():new_progress(0),exerted_pressure(false),was_sticky_at_frame_start(false){}
  value_for_each_cardinal_direction<sub_tile_distance> new_progress;
  
  // The basic rule for sticky water becoming inactive is that it does so if it's in the stable state AND didn't exert pressure directly onto another tile for a frame. These variables are primarily to control the process of sticky water becoming inactive.
  bool exerted_pressure;
  bool was_sticky_at_frame_start;
};

// TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
int obstructiveness_for_the_purposes_of_the_fall_off_pillars_rule(tile const& t) {
  if (t.contents() == ROCK) return 4;
  else if (t.is_sticky_water()) return 3;
  else if (t.is_free_water()) return 2;
  else if (t.contents() == AIR) return 1;
  else assert(false);
}

void update_water(world &w) {
  unordered_map<tile_location, active_water_temporary_data> temp_data;
  for (pair<const tile_location, water_movement_info> &p : w.active_water_tiles_range()) {
    tile_location const& loc = p.first;
    tile const& t = loc.stuff_at();
    assert(t.contents() == WATER);
    
    // initialize...
    temp_data[loc];
    
    if (t.is_sticky_water()) {
      temp_data[loc].was_sticky_at_frame_start = true;
      vector3<sub_tile_distance> &vel_ref = p.second.velocity;
      const vector3<sub_tile_distance> current_velocity_wrongness = vel_ref - inactive_water_velocity;
      if (current_velocity_wrongness.magnitude_within_32_bits_is_less_than(sticky_water_velocity_reduction_rate)) {
        vel_ref = inactive_water_velocity;
      }
      else {
        vel_ref -= current_velocity_wrongness * sticky_water_velocity_reduction_rate / current_velocity_wrongness.magnitude_within_32_bits();
      }
    }
  }
  
  water_groups_structure groups;
  compute_groups_that_need_to_be_considered(w, groups);

  for (group_number_t group_number = FIRST_GROUP; group_number < (group_number_t)groups.tile_locations_by_group_number.size(); ++group_number) {
    for (tile_location const& loc : groups.tile_locations_by_group_number[group_number]) {
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const tile_location dst_loc = loc + dir;
        tile const& dst_tile = dst_loc.stuff_at();
        if ((!dst_tile.is_sticky_water()) && (dst_tile.contents() != ROCK)) { // i.e. is air or free water. Those exclusions aren't terribly important, but it'd be slightly silly to remove either of them (and we currently rely on both exclusions to make the inactive state what it is.)
          // Exert pressure proportional to the depth of the target tile. The value is tuned to prevent water that should be stable (if unavoidably uneven by 1 tile or less) from fluctuating.
          const tile_coordinate_signed_type depth = tile_coordinate_signed_type(1 + groups.max_z_including_nearby_free_waters_by_group_number[group_number] - dst_loc.coords().z) - 1;
          double pressure = double(depth) - 0.5; // The 0.5 could be anything in the interval (0, 1).
          // A variation on the "fall off pillars" rule that covers us against some glitchy situations.
          // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
          if (dir.v.z == 0) {
            if (obstructiveness_for_the_purposes_of_the_fall_off_pillars_rule((loc + dir + cdir_zminus).stuff_at()) < obstructiveness_for_the_purposes_of_the_fall_off_pillars_rule((loc + cdir_zminus).stuff_at())) {
              pressure += 1.0;
            }
          }
          if (pressure > 0) {
            if (temp_data.find(loc) == temp_data.end()) {
              w.activate_water(loc);
              // create and initialize...
              temp_data[loc].was_sticky_at_frame_start = true;
            }
            temp_data[loc].exerted_pressure = true;
            temp_data[loc].new_progress[dir] += (sub_tile_distance)((pressure_motion_factor + (rand()%pressure_motion_factor_random)) * std::sqrt(pressure));
            //std::cerr << "Foo: " << depth << ", " << temp_data[loc].new_progress[dir] << ", " << int64_t(dst_loc.coords().z) - world_center_coord << "\n";
          }
        }
      }
    }
  }
  
  for (pair<const tile_location, water_movement_info> &p : w.active_water_tiles_range()) {
    tile_location const& loc = p.first;
    tile const& t = loc.stuff_at();
    assert(t.contents() == WATER);
    if (t.is_sticky_water()) {
      tile const& t_down = (loc+cdir_zminus).stuff_at();
      const bool t_down_obstructive = t_down.contents() == ROCK || t_down.is_sticky_water();
      // This doesn't count as a disturbance, it's part of decaying towards the stable state...
      // If the bottom tile is open, we already marked it above.
      assert(temp_data[loc].exerted_pressure || t_down_obstructive);
      // If it's blocked, then we're not going to move in that direction anyway, so it's not real pressure.
      temp_data[loc].new_progress[cdir_zminus] += extra_downward_speed_for_sticky_water;
      if (t_down_obstructive) {
        sub_tile_distance excess_progress = temp_data[loc].new_progress[cdir_zminus] + p.second.progress[cdir_zminus] - progress_necessary(cdir_zminus);
        if (excess_progress > 0) {
          temp_data[loc].new_progress[cdir_zminus] -= excess_progress;
          if (temp_data[loc].new_progress[cdir_zminus] < 0) 
            temp_data[loc].new_progress[cdir_zminus] = 0;
        }
      }
    }
    else {
      vector3<sub_tile_distance> &vel_ref = p.second.velocity;
      vel_ref += gravity_acceleration;
      
      // Slight air resistance proportional to the square of the velocity (has very little effect at our current 20x20x20 scale; mostly there to make a natural cap velocity for falling water)
      vel_ref -= (vel_ref * vel_ref.magnitude_within_32_bits()) / air_resistance_constant;
      
      // Relatively large friction against the ground
      for (EACH_CARDINAL_DIRECTION(dir)) {
        if (p.second.blockage_amount_this_frame[dir] > 0) {
          vector3<sub_tile_distance> copy_stationary_in_blocked_direction = vel_ref; copy_stationary_in_blocked_direction -= project_onto_cardinal_direction(copy_stationary_in_blocked_direction, dir);
          if (copy_stationary_in_blocked_direction.magnitude_within_32_bits_is_less_than(friction_amount)) {
            vel_ref -= copy_stationary_in_blocked_direction;
          }
          else {
            vel_ref -= copy_stationary_in_blocked_direction * friction_amount / copy_stationary_in_blocked_direction.magnitude_within_32_bits();
          }
        }
      }
      
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const sub_tile_distance dp = vel_ref.dot<sub_tile_distance>(dir.v);
        if (dp > 0) temp_data[loc].new_progress[dir] += dp;

        // Water that's blocked, but can go around in a diagonal direction, makes "progress" towards all those possible directions (so it'll go in a random direction if it could go around in several different diagonals, without having to 'choose' one right away and gain velocity only in that direction). The main purpose of this is to make it so that water doesn't stack nicely in pillars, or sit still on steep slopes.
        // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
        const tile_location dst_loc = loc + dir;
        if (dst_loc.stuff_at().contents() == AIR) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            const tile_location diag_loc = dst_loc + d2;
            if (p.second.blockage_amount_this_frame[d2] > 0 && d2.v.dot<sub_tile_distance>(dir.v) == 0 && obstructiveness_for_the_purposes_of_the_fall_off_pillars_rule(diag_loc.stuff_at()) < obstructiveness_for_the_purposes_of_the_fall_off_pillars_rule((loc + d2).stuff_at())) {
              temp_data[loc].new_progress[dir] += p.second.blockage_amount_this_frame[d2];
            }
          }
        }
      }
    }
      
    // This always happens, even if water becomes sticky right after it gets blocked.
    for (sub_tile_distance &bb : p.second.blockage_amount_this_frame) bb = 0;
  }
  
  vector<wanted_move> wanted_moves;
  for (pair<const tile_location, water_movement_info> &p : w.active_water_tiles_range()) {
    tile_location const& loc = p.first;
    tile const& t = loc.stuff_at();
    assert(t.contents() == WATER);
    for (EACH_CARDINAL_DIRECTION(dir)) {
      sub_tile_distance &progress_ref = p.second.progress[dir];
      const sub_tile_distance new_progress = temp_data[loc].new_progress[dir];
      if (temp_data[loc].new_progress[dir] == 0) {
        if (progress_ref < idle_progress_reduction_rate) progress_ref = 0;
        else progress_ref -= idle_progress_reduction_rate;
      }
      else {
        assert(new_progress >= 0);
        assert(progress_ref >= 0);
        assert(progress_ref <= progress_necessary(dir));
        progress_ref += new_progress;
        if (progress_ref > progress_necessary(dir)) {
          wanted_moves.push_back(wanted_move(loc, dir, t.is_sticky_water() ? groups.group_numbers_by_tile_location[loc] : NO_GROUP, new_progress, progress_ref - progress_necessary(dir)));
          progress_ref = progress_necessary(dir);
        }
      }
    }
  }
  
  std::random_shuffle(wanted_moves.begin(), wanted_moves.end());
  std::unordered_set<tile_location> disturbed_tiles;

  vector<map<tile_coordinate, literally_random_access_removable_stuff<tile_location> > > tiles_by_z_coordinate_by_group_number(groups.tile_locations_by_group_number.size());
  
  for (group_number_t group_number = FIRST_GROUP; group_number < (group_number_t)groups.tile_locations_by_group_number.size(); ++group_number) {
    for (tile_location const& loc : groups.tile_locations_by_group_number[group_number]) {
      tiles_by_z_coordinate_by_group_number[group_number][loc.coords().z].add(loc);
    }
    for (tile_location const& loc : groups.nearby_free_waters_by_group_number[group_number]) {
      tiles_by_z_coordinate_by_group_number[group_number][loc.coords().z].add(loc);
    }
  }
  
  // It doesn't matter that we won't add temp_data for nearby tiles that are activated during this loop by the insertion and deletion of water.
  for (const wanted_move move : wanted_moves) {
    const tile_location dst = move.src + move.dir;
    tile const& src_tile = move.src.stuff_at();
    tile const& dst_tile = dst.stuff_at();
    // in certain situations we shouldn't try to move water more than once
    if (disturbed_tiles.find(move.src) != disturbed_tiles.end()) continue;
    // anything where the water was yanked away should have been marked "disturbed"
    assert(src_tile.contents() == WATER);
    
    water_movement_info *fbdkopsw = w.get_active_water_tile(move.src);
    assert(fbdkopsw);
    water_movement_info &src_water = *fbdkopsw;
    sub_tile_distance& progress_ref = src_water.progress[move.dir];
    
    if (dst_tile.contents() == AIR) {
      progress_ref -= progress_necessary(move.dir);
      water_movement_info &dst_water = w.insert_water(dst);
      w.set_stickyness(dst, should_be_sticky(dst));
      
      if (move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP) {
        dst_water = src_water;
        temp_data[dst] = temp_data[move.src];
        w.delete_water(move.src);
        temp_data.erase(move.src);
        disturbed_tiles.insert(move.src);
      }
      else {
        // the teleported tile starts with zero everything, for some reason (TODO: why should it be like this?)
        dst_water.velocity = vector3<sub_tile_distance>(0,0,0); dst_water.progress[cdir_zminus] = 0;
        
        // Pick a random top tile.
        map<tile_coordinate, literally_random_access_removable_stuff<tile_location> > m = tiles_by_z_coordinate_by_group_number[move.group_number_or_NO_GROUP_for_velocity_movement];
        tile_location chosen_top_tile(move.src); // really I shouldn't be constructing a location, but there's no default constructor, and for good reason !!!! !!!! !!!
        while (true) {
          assert(!m.empty());
          map<tile_coordinate, literally_random_access_removable_stuff<tile_location> >::iterator l = boost::prior(m.end());
          if (l->second.empty()) m.erase(l);
          else {
            chosen_top_tile = l->second.get_and_blacklist_random();
            if (disturbed_tiles.find(chosen_top_tile) == disturbed_tiles.end()) break;
          }
        }
        // Since the source tile is still water, there always must be a real tile to choose
        
        w.delete_water(chosen_top_tile);
        temp_data.erase(chosen_top_tile);
        disturbed_tiles.insert(chosen_top_tile);
      }
      
      // If a tile moves, we're now content to assume that it was moving because it had a realistic velocity in that direction, so we should continue with that assumption.
      const sub_tile_distance amount_of_new_vel_in_movement_dir = dst_water.velocity.dot<sub_tile_distance>(move.dir.v);
      const sub_tile_distance deficiency_of_new_vel_in_movement_dir = move.amount_of_the_push_that_sent_us_over_the_threshold - amount_of_new_vel_in_movement_dir;
      if (deficiency_of_new_vel_in_movement_dir > 0) {
        dst_water.velocity += vector3<sub_tile_distance>(move.dir.v) * deficiency_of_new_vel_in_movement_dir;
      }
      // Also, don't lose movement to rounding error during progress over multiple tiles:
      dst_water.progress[move.dir] = std::min(move.excess_progress, progress_necessary(move.dir));
    }
    else {
      // we're blocked
      src_water.blockage_amount_this_frame[move.dir] = move.excess_progress;
      
      if (dst_tile.contents() == WATER) {
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
          water_movement_info &dst_water = w.activate_water(dst);
          // we tried to move due to pressure, but bumped into free water! Force the free water to move at the rate we wanted our expelled water to move.
          const sub_tile_distance amount_of_their_vel_in_movement_dir = dst_water.velocity.dot<sub_tile_distance>(move.dir.v);
          const sub_tile_distance deficiency_of_their_vel_in_movement_dir = move.amount_of_the_push_that_sent_us_over_the_threshold - amount_of_their_vel_in_movement_dir;
          if (deficiency_of_their_vel_in_movement_dir > 0) {
            dst_water.velocity += vector3<sub_tile_distance>(move.dir.v) * deficiency_of_their_vel_in_movement_dir;
          }
          // Formerly: Turn our movement into velocity, at some conversion factor.
          //w.activate_water(dst).velocity += (vector3<sub_tile_distance>(move.dir.v) * move.excess_progress) / 10;
        }
      }
      else if (dst_tile.contents() == ROCK) {
        // TODO figure out what to actually do about the fact that water can become sticky while having lots of progress.
        //assert(move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP);
        src_water.get_completely_blocked(move.dir);
      }
      else assert(false);
    }
  }
  
  for (auto const& p : temp_data) {
    tile_location const& loc = p.first;
    if (water_movement_info *water = w.get_active_water_tile(loc)) {
      bool wanted_stickyness = should_be_sticky(loc);
      if (wanted_stickyness != loc.stuff_at().is_sticky_water() && water->computed_sticky_last_frame == wanted_stickyness) {
        w.set_stickyness(loc, wanted_stickyness);
      }
      water->computed_sticky_last_frame = wanted_stickyness;
    }
  }
  
  for (auto const& p : temp_data) {
    tile_location const& loc = p.first;
    if (water_movement_info *water = w.get_active_water_tile(loc)) {
      if (water->is_in_inactive_state()) {
        if (loc.stuff_at().is_sticky_water()) {
          if (p.second.was_sticky_at_frame_start && !p.second.exerted_pressure) {
            w.deactivate_water(loc);
          }
        }
        else {
          // Be a little paranoid about making sure free water obeys all the proper conditions of inactivity
          bool can_deactivate = true;
          for (EACH_CARDINAL_DIRECTION(dir)) {
            if (dir.v.z < 0) {
              if (water->blockage_amount_this_frame[dir] != min_convincing_speed + (-gravity_acceleration.z)) {
                can_deactivate = false; break;
              }
            }
            else {
              if (water->blockage_amount_this_frame[dir] != 0) {
                can_deactivate = false; break;
              }
            }
          }
          if (!can_deactivate) continue;
          const tile_location downloc = loc + cdir_zminus;
          if (!(downloc.stuff_at().is_sticky_water() || downloc.stuff_at().contents() == ROCK)) continue;
          // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
          for (EACH_CARDINAL_DIRECTION(dir)) {
            if (dir.v.dot<neighboring_tile_differential>(zunitv) == 0 &&
                (loc + dir).stuff_at().contents() == AIR &&
                (((downloc + dir).stuff_at().contents() == AIR) || (downloc + dir).stuff_at().is_free_water())) {
              can_deactivate = false; break;
            }
          }
          
          if (can_deactivate) w.deactivate_water(loc);
        }
      }
    }
  }
}

