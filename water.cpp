
#include "world.hpp"


void world::deactivate_water(location const& loc) {
  active_tiles.erase(loc);
}

water_movement_info& world::activate_water(location const& loc) {
  assert(loc.stuff_at().contents() == WATER);
  return active_tiles[loc]; // if it's not there, this inserts it, default-constructed
}




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

bool columns_sorter_lt(pair<location, group_number_t> const& i, pair<location, group_number_t> const& j) {
  vector3<location_coordinate> c1 = i.first.coords();
  vector3<location_coordinate> c2 = j.first.coords();
  return (c1.x < c2.x) || ((c1.x == c2.x) && ((c1.y < c2.y) || ((c1.y == c2.y) && (c1.z < c2.z))));
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
  std::sort(columns.begin(), columns.end(), columns_sorter_lt);
  
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
    assert(t.contents() == WATER);
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
        if (!dst_tile.is_sticky_water() && dst_tile.contents() != ROCK) { // i.e. is air or free water. Those exclusions aren't terribly important, but it'd be slightly silly to remove either of them (and we currently rely on both exclusions to make the idle state what it is.)
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
    assert(t.contents() == WATER);
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
        if (dst_loc.stuff_at().contents() == AIR) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            const location diag_loc = dst_loc + d2;
            if (p.second.blockage_amount_this_frame[d2] > 0 && d2.v.dot<sub_tile_distance>(dir.v) == 0 && diag_loc.stuff_at().contents() == AIR) {
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
    assert(t.contents() == WATER);
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
  std::unordered_set<location> disturbed_tiles;

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
        if (dst_tile.contents() == AIR) {
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
    tile const& src_tile = move.src.stuff_at();
    tile const& dst_tile = dst.stuff_at();
    tile src_copy = src_tile; // only for use in gdb
    tile dst_copy = dst_tile; // only for use in gdb
    // in certain situations we shouldn't try to move water more than once
    if (disturbed_tiles.find(move.src) != disturbed_tiles.end()) continue;
    // anything where the water was yanked away should have been marked "disturbed"
    assert(src_tile.contents() == WATER);
    
    water_movement_info *fbdkopsw = w.get_active_tile(move.src);
    assert(fbdkopsw);
    water_movement_info &src_water = *fbdkopsw;
    sub_tile_distance& progress_ref = src_water.progress[move.dir];
    
    if (dst_tile.contents() == AIR) {
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
          // we tried to move due to pressure, but bumped into free water! Turn our movement into velocity, at some conversion factor.
          w.activate_water(dst).velocity += (move.dir.v * move.excess_progress) / 10;
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

