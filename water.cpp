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


#include "world.hpp"


/*

- Only water (not other fluids) behaves as groups

*/

// TODO make a way to obsessively check the caches to find out exactly what code breaks them
// (assuming that this code has at least a few bugs in it)

// Current remaining major TODOs: making world stuff, and cache checking. And activating water when other stuff disappears
  
template<typename Stuff> struct literally_random_access_removable_stuff {
public:
  void insert(Stuff const& stuff) {
    if (stuffs_set.insert(stuff).first) {
      stuffs_superset_vector.push_back(stuff);
    }
  }
  bool erase(Stuff const& which) {
    if (stuffs_set.erase(stuff).first) {
      if (stuffs_set.size() * 2 <= stuffs.size()) {
        purge_nonexistent_stuffs();
      }
      return true;
    }
    return false;
  }
  Stuff const& get_random()const {
    assert(!stuffs_set.empty());
    size_t idx;
    do {
      idx = (size_t)(rand()%(stuffs_superset_vector.size()));
    } while (stuffs_set.find(stuffs_superset_vector[idx]) == stuffs_set.end());
    return stuffs_superset_vector[idx];
  }
  bool empty()const { return stuffs.empty(); }
  unordered_set<Stuff> const& as_unordered_set() { return stuffs_set; }
private:
  vector<Stuff> stuffs_superset_vector;
  unordered_set<Stuff> stuffs_set;
  void purge_nonexistent_stuffs() {
    size_t next_insert_idx = 0;
    for (Stuff const& st : stuffs_set) {
      stuffs_superset_vector[next_insert_idx] = st;
      ++next_insert_idx;
    }
    stuffs.erase(stuffs.begin() + next_insert_idx, stuffs.end());
  }
};

struct literally_random_access_removable_tiles_by_height {
  typedef map<tile_coordinate, literally_random_access_removable_stuff<tile_location>> map_t;
  
  tile_location get_and_erase_random_from_the_top() {
    map_t::reverse_iterator iter = data.rbegin();
    tile_location result = iter->second.get_random();
    iter->second.erase(result);
    if (iter->second.empty()) data.erase(iter);
    return result;
  }
  bool erase(tile_location const& loc) {
    auto j = data.find(loc.coords().z);
    if (j != data.end()) {
      if (j->second.erase(loc)) {
        if (j->second.empty()) {
          data.erase(loc);
        }
        return true;
      }
    }
    return false;
  }
  void insert(tile_location const& loc) {
    // Note: operator[] default-constructs an empty structure if there wasn't one
    data[loc.coords().z].insert(loc);
  }
  
  bool any_suckable_tiles_remain() {
    return !suckable_tiles_by_height.as_map().empty();
  }
  bool any_above(tile_coordinate height) {
    return data.upper_bound(height) != data.end();
  }
  bool any_below(tile_coordinate height) {
    return (!data.empty()) && (data.begin()->first < height);
  }
  map_t const& as_map() { return data; }

private:
  map_t data;
}


typedef uint64_t water_tile_count;
typedef uint64_t water_group_identifier;
water_group_identifier NO_WATER_GROUP = 0;

typedef uint64_t pressure_amount;

struct persistent_water_group_info {
  literally_random_access_removable_tiles_by_height suckable_tiles_by_height;
  literally_random_access_removable_tiles_by_height pushable_tiles_by_height;
  map<tile_coordinate, water_tile_count> num_tiles_by_height;
  unordered_set<tile_location> surface_tiles;
  
  mutable map<tile_coordinate, pressure_amount> pressure_caches;
  mutable map<tile_coordinate, water_tile_count> width_of_widest_level_so_far_caches;
  
  void recompute_num_tiles_by_height_from_surface_tiles(world &w) {
    // Here's how we compute the total volume in less than linear time:
    // We take each contiguous row of surface tiles and compute its total length by
    // using a cleverly sorted set, so that we can start with a tile at the beginning
    // of the row and just jump to the tile at the other end.
    //
    // Here, we look up all "low-x ends of x-rows" and jump to the high-x ends
    num_tiles_by_height.clear();
    
    auto const& groupable_water_dimensional_boundaries_TODO_name_this_better = w.get_groupable_water_dimensional_boundaries_TODO_name_this_better();
    
    for (tile_location const& surface_loc : surface_tiles) {
      auto surface_loc_iter = groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.find(surface_loc);
      if (surface_loc_iter != groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.end()) {
        // We're only interested in starting at low-x boundaries
        const tile_location loc_less_x = surface_loc + cdir_xminus;
        if (loc_less_x.stuff_at().contents() != GROUPABLE_WATER) {
          // If we're *also* the high-x boundary, there's only one water tile in this row
          const tile_location loc_more_x = surface_loc + cdir_xplus;
          if (loc_more_x.stuff_at().contents() != GROUPABLE_WATER) {
            num_tiles_by_height[surface_loc.coords().z] += 1;
          }
          else {
            ++surface_loc_iter;
            tile_location const& end_tile = *surface_loc_iter;
            assert(end_tile.coords().y == surface_loc.coords().y && end_tile.coords().z == surface_loc.coords().z);
            assert(surface_tiles.find(end_tile) != surface_tiles.end());
            num_tiles_by_height[surface_loc.coords().z] += 1 + end_tile.x - surface_loc.x;
          }
        }
      }
    }
  }
  
  pressure_amount get_pressure_at_height(tile_coordinate height) {
    // The pressure computation here is a kludge. The main difference between it and "the height of the top level minus the height of the current level" is that it doesn't make teeny water towers on top of oceans exert pressure over the whole ocean.
    map<tile_coordinate, pressure_amount>::iterator iter = pressure_caches.lower_bound(height);
    tile_coordinate current_height = 0;
    pressure_amount current_pressure = 0;
    if (iter == pressure_caches.end()) {
      current_height = num_tiles_by_height.rbegin()->first;
      pressure_caches[current_height] = 0;
      width_of_widest_level_so_far_caches[current_height] = num_tiles_by_height.find(current_height-1)->second;
    }
    else {
      current_height = iter->first;
      current_pressure = iter->second;
    }
    while (current_height != height) {
      --current_height;
      water_tile_count last_count = width_of_widest_level_so_far_caches.find(current_height+1)->second;
      water_tile_count new_count = (width_of_widest_level_so_far_caches[current_height] = std::max(last_count, num_tiles_by_height.find(current_height-1)->second));
      current_pressure = (current_pressure + pressure_constant) * std::min(new_count, last_count) / new_count;
      pressure_caches[current_height] = current_pressure;
    }
    return current_pressure;
  }
  
  tile_location get_and_erase_random_pushable_tile_below_weighted_by_pressure(tile_coordinate height) {
    // It's annoying that this is as bad as linear in the height of the group;
    // I think I could do better, but it would be more complicated.
    pressure_amount total_weight;
    for (auto const& p = pushable_tiles_by_height.as_map().begin(); p != pushable_tiles_by_height.as_map().upper_bound(height); ++p) {
      // Caution: duplicate definition of "weight" (see below)
      const pressure_amount weight = get_pressure_at_height(p->first) * p->second.as_unordered_set().size();
      total_weight += weight;
    }
    pressure_amount choice = rand()%total_weight;
    for (auto const& p = pushable_tiles_by_height.as_map().begin(); p != pushable_tiles_by_height.as_map().upper_bound(height); ++p) {
      // Caution: duplicate definition of "weight" (see above)
      const pressure_amount weight = get_pressure_at_height(p->first) * p->second.as_unordered_set().size();
      choice -= weight;
      if (choice < 0) {
        tile_location result = p->second.get_random();
        pushable_tiles_by_height.erase(result);
        return result;
      }
    }
  }
  
  bool mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(world &w, tile_location const& loc) {
    if (!pushable_tiles_by_height.any_below(loc.coords().z)) {
      suckable_tiles_by_height.insert(loc);
      return false;
    }
    const tile_location pushed_tile = get_and_erase_random_pushable_tile_below_weighted_by_pressure();
    w.suck_out_suckable_water(loc);
    w.push_water_into_pushable_tile(pushed_tile);
    return true;
  }
  bool mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(tile_location const& loc) {
    if (!suckable_tiles_by_height.any_above(loc.coords().z)) {
      pushable_tiles_by_height.insert(loc);
      return false;
    }
    const tile_location sucked_tile = suckable_tiles_by_height.get_and_erase_random_from_the_top();
    w.suck_out_suckable_water(sucked_tile);
    w.push_water_into_pushable_tile(loc);
    return true;
  }
}


bool tile_compare_xyz(tile_location const& i, tile_location const& j) {
  vector3<tile_coordinate> c1 = i.coords();
  vector3<tile_coordinate> c2 = j.coords();
  return (c1.x < c2.x) || ((c1.x == c2.x) && ((c1.y < c2.y) || ((c1.y == c2.y) && (c1.z < c2.z))));
}

// We could easily keep lists of boundary tiles in all three dimensions
// (Just uncomment the six commented lines below.)
// The only reason we don't is because there's no need for any of the others right now.
// (And it would take that much extra space (proportional to the that-dimension surface area)
//   and time (proportional to how much the groupable-water landscape changes)).
struct groupable_water_dimensional_boundaries_TODO_name_this_better_t {
  set<tile_location, tile_compare_yzx> x_boundary_groupable_water_tiles;
  //set<tile_location, tile_compare_zxy> y_boundary_groupable_water_tiles;
  //set<tile_location, tile_compare_xyz> z_boundary_groupable_water_tiles;
  void handle_tile_insertion(tile_location const& loc) {
    handle_tile_insertion_in_direction(x_boundary_groupable_water_tiles, loc, cdir_xplus);
  //  handle_tile_insertion_in_direction(y_boundary_groupable_water_tiles, loc, cdir_yplus);
  //  handle_tile_insertion_in_direction(z_boundary_groupable_water_tiles, loc, cdir_zplus);
  }
  void handle_tile_removal(tile_location const& loc) {
    handle_tile_removal_in_direction(x_boundary_groupable_water_tiles, loc, cdir_xplus);
  //  handle_tile_removal_in_direction(y_boundary_groupable_water_tiles, loc, cdir_yplus);
  //  handle_tile_removal_in_direction(z_boundary_groupable_water_tiles, loc, cdir_zplus);
  }
private:
  template <typename Compare> void handle_tile_removal_in_dimension(set<tile_location, Compare> boundary_tiles_set, tile_location const& loc, cardinal_direction dir);
  {
    // This tile is no longer groupable at all, so it can't be a boundary tile
    boundary_tiles_set.erase(loc);
    
    // If there are groupable tiles next to us, they must now be boundary tiles,
    // because our deletion exposed them
    const tile_location further_in_positive_direction_loc = loc + dir;
    const tile_location further_in_negative_direction_loc = loc - dir;
    if (further_in_positive_direction_loc.stuff_at().contents() == GROUPABLE_WATER) {
      boundary_tiles_set.insert(further_in_positive_direction_loc);
    }
    if (further_in_negative_direction_loc.stuff_at().contents() == GROUPABLE_WATER) {
      boundary_tiles_set.insert(further_in_negative_direction_loc);
    }
  }
  template <typename Compare> void handle_tile_insertion_in_dimension(set<tile_location, Compare> boundary_tiles_set, tile_location const& loc, cardinal_direction dir);
  {
    // We *may* have removed boundaries in either direction, and we *may* now be a boundary tile ourselves.
    const tile_location further_in_positive_direction_loc = loc + dir;
    const tile_location further_in_negative_direction_loc = loc - dir;
    bool we_are_boundary_tile = false;
    if (further_in_positive_direction_loc.stuff_at().contents == GROUPABLE_WATER) {
      if ((further_in_positive_direction_loc + dir).stuff_at().contents == GROUPABLE_WATER) {
        boundary_tiles_set.erase(further_in_positive_direction_loc);
      }
    }
    else we_are_boundary_tile = true;
    if (further_in_negative_direction_loc.stuff_at().contents == GROUPABLE_WATER) {
      if ((further_in_negative_direction_loc + dir).stuff_at().contents == GROUPABLE_WATER) {
        boundary_tiles_set.erase(further_in_negative_direction_loc);
      }
    }
    else we_are_boundary_tile = true;
    
    if (we_are_boundary_tile) boundary_tiles_set.insert(loc);
  }
}

//TODO: in world:
typedef unordered_map<water_group_identifier, persistent_water_groups_t> persistent_water_groups_t;
typedef unordered_map<tile_location, water_group_identifier> water_groups_by_location_t;
water_groups_by_location_t water_groups_by_surface_tile;
persistent_water_groups_t persistent_water_groups;
groupable_water_dimensional_boundaries_TODO_name_this_better_t groupable_water_dimensional_boundaries_TODO_name_this_better;
typedef unordered_map<tile_location, active_fluid_tile_info> active_fluids_t;
unordered_map<tile_location, active_fluid_tile_info> active_fluids;

// ONLY to be called by replace_substance
water_group_identifier make_new_water_group(water_group_identifier &next_water_group_identifier, persistent_water_groups_t &persistent_water_groups) {
  water_group_identifier this_id = next_water_group_identifier++;
  persistent_water_groups[this_id]; // operator[] inserts a default-constructed one
  return this_id;
}

// ONLY to be called by replace_substance
water_group_identifier merge_water_groups(water_group_identifier id_1, water_group_identifier id_2,
   persistent_water_groups_t &persistent_water_groups,
   water_groups_by_location_t &water_groups_by_surface_tile) {
  persistent_water_group_info &group_1 = persistent_water_groups.find(id_1)->second;
  persistent_water_group_info &group_2 = persistent_water_groups.find(id_2)->second;
  
  bool group_1_is_smaller = group_1.surface_tiles.size() < group_2.surface_tiles.size();
  persistent_water_group_info &smaller_group = (group_1_is_smaller ? group_1 : group_2);
  persistent_water_group_info & larger_group = (group_1_is_smaller ? group_2 : group_1);
  water_group_identifier  remaining_group_id = (group_1_is_smaller ?    id_2 :    id_1);
  water_group_identifier  destroyed_group_id = (group_1_is_smaller ?    id_1 :    id_2);
  
  for (auto const& s : smaller_group.suckable_tiles_by_height.as_map()) {
    for (auto const& t : s.second.as_unordered_set()) {
      larger_group.suckable_tiles_by_height.insert(t);
    }
  }
  for (auto const& p : smaller_group.num_tiles_by_height) {
    // Note: the [] operator default-constructs a zero if there's nothing there
    larger_group.num_tiles_by_height[p.first] += p.second;
  }
  for (auto const& l : smaller_group.surface_tiles) {
    larger_group.surface_tiles.insert(l);
    water_groups_by_surface_tile[l] = remaining_group_id;
  }
  
  larger_group.pressure_caches.erase(larger_group.pressure_caches.begin(), larger_group.pressure_caches.lower_bound(smaller_group.num_tiles_by_height.rbegin()->first + 1));
  
  persistent_water_groups.erase(destroyed_group_id);
  
  return remaining_group_id;
}

void acquire_group_id_of_water_tile(tile_location const& loc) {
  auto i = water_groups_by_surface_tile.find(loc);
  if (i != water_groups_by_surface_tile.end()) {
    return i->second;
  }
  else {
    // Crap, we don't know what group we're part of unless we find a surface tile!
    // Find the next surface tile in some arbitrary direction.
    // That tile will tell us what group we're in.
    location const& surface_loc = *(groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.lower_bound(loc));
    return water_groups_by_surface_tile.find(surface_loc)->second;
  }
}



void suck_out_suckable_water_(world &w,
   tile_location const& loc,
   active_fluids_t &active_fluids);
   
void world::suck_out_suckable_water(tile_location const& loc) {
  teleport_out_suckable_water_(*this, loc, active_fluids);
}

void suck_out_suckable_water_(world &w,
   tile_location const& loc,
   active_fluids_t &active_fluids)
{
  // Suckable water works like this:
  // The idle/inactive state of water, including groupable water, is
  // to have maximum downward progress and a little downward velocity.
  // Water is considered "suckable" if it has enough downward progress to
  // immediately enter another tile of the group - so all idle water is, by default, suckable.
  // 
  // BUT...
  //
  // We don't want it to cascade downwards. If there's a huge suck at the bottom, we *don't* want
  // a huge mass of water to suddenly vanish from the top - that would make the top layer move
  // way faster than gravity, and the fiction is that it's really being moved by gravity,
  // so that would be bizarre and/or immersion-breaking. The top layer shouldn't move faster
  // than gravity.
  //
  // SO...
  //
  // 1) Water is only considered suckable if it has the right progress AND has no other water above it, and...
  // 2) When a water gets sucked from above another water, we behave as if the *lower* water got teleported away
  //     and the *upper* water just fell naturally. That way, the upper water (now in the location of the lower
  //     water) has no downwards progress left, and has to build up progress again using gravity.
  
  const location downward_loc = loc + cdir_zminus;
  if (downward_loc.stuff_at().contents() == GROUPABLE_WATER) {
    auto here_af_iter = active_fluids.find(loc);
    auto down_af_iter = active_fluids.find(downward_loc);
    active_fluid_info here_info = (here_af_iter == active_fluids.end() ? active_fluid_info() : here_af_iter->second);
    here_info.progress[cdir_zminus] -= progress_necessary(cdir_zminus); // The usual effect of moving downwards
    if (down_af_iter == active_fluids.end()) {
      active_fluids.insert(downward_loc, here_info);
    }
    else {
      // If the downward water is active, we *could* combine its values with ours somehow.
      // However, the downward water is groupable, so it should already be pretty slow,
      // and combining them would make it more complicated to understand how sucking works -
      // I feel like it would invite weird corner cases.
      down_af_iter->second = here_info;
    }
    active_fluids.erase(here_af_iter);
  }
  else {
    // If there's no groupable water below us, then we've gotten to the bottom of the barrel
    // and so the system doesn't need to make any extra accomodations. (If all the water is
    // gone, then there isn't enough left to be unrealistic when it moves fast...)
    active_fluids.erase(loc);
  }
  
  // replace_substance handles marking the water no-longer-suckable.
  w.replace_substance(loc, GROUPABLE_WATER, AIR);
}


void push_water_into_pushable_tile_(world &w,
   tile_location const& loc,
   active_fluids_t &active_fluids);
   
void world::push_water_into_pushable_tile(tile_location const& loc) {
  push_water_into_pushable_tile_(*this, loc, active_fluids);
}
void push_water_into_pushable_tile_(world &w,
   tile_location const& loc,
   active_fluids_t &active_fluids)
{
  // We always create ungroupable water at first due to pressure;
  // creating groupable water runs the risk
  // of creating explosions where more and more water is drawn out.
  // The water will become groupable in its own time.
  // replace_substance automatically makes the tile no-longer-pushable.
  w.replace_substance(loc, AIR, UNGROUPABLE_WATER);
      
  // Activate it.
  // The [] operator creates a default-constructed version.
  // In this case, we don't want the default attributes - water moved by pressure
  // starts with no downward progress or velocity.
  // It'll be given its new velocity in the next frame, by the normal pressure-pushes-water rules.
  active_fluids[loc].progress[cdir_zminus] = 0;
  active_fluids[loc].velocity.z = 0;
}

void become_interior(tile_location const& loc, world_collision_detector &things_exposed_to_collision) {
  mutable_stuff_at(loc).set_interiorness(true);
  // Now interior: remove us from the collision detection.
  // TODO: figure out whether surface air should be collision detected with
  // (maybe there's something that detects contact with air? Sodium...?)
  if (loc.stuff_at().contents() != AIR) {
    assert(things_exposed_to_collision.exists(adj_loc));
    things_exposed_to_collision.insert(adj_loc);
  }
}

void become_not_interior(tile_location const& loc, world_collision_detector &things_exposed_to_collision)
  mutable_stuff_at(loc).set_interiorness(false);
  // No longer interior! Unless you're air, add to the collision detection struct.
  // TODO: figure out whether surface air should be collision detected with
  // (maybe there's something that detects contact with air? Sodium...?)
  assert(!things_exposed_to_collision.exists(adj_loc));
  if (loc.stuff_at().contents() != AIR) {
    things_exposed_to_collision.insert(adj_loc);
  }
}

void world::replace_substance(
   tile_location const& loc,
   tile_contents old_substance_type,
   tile_contents new_substance_type) {
  replace_substance_(loc, old_substance_type, new_substance_type, );
}

void replace_substance_(
   tile_location const& loc,
   tile_contents old_substance_type,
   tile_contents new_substance_type,
  
   groupable_water_dimensional_boundaries_TODO_name_this_better_t &groupable_water_dimensional_boundaries_TODO_name_this_better,
   active_fluids_t &active_fluids,
   water_group_identifier &next_water_group_identifier,
   persistent_water_groups_t &persistent_water_groups,
   water_groups_by_location_t &water_groups_by_surface_tile)
{
  tile const& t = loc.stuff_at();
  
  assert(t.contents() == old_substance_type);
  
  if (old_substance_type != AIR && new_substance_type == AIR) {
    // We might have moved away from a tile that was trying to push water into us.
    // This system formerly used rules where the emptied tile naturally didn't refill until the following frame.
    // The downside of those rules was that the simulation could immediately mark the tile that had been trying
    // to exert pressure as having too much air next to it, which would make it stop exerting pressure;
    // To combat that effect, we made the pressure-exertingness linger for a frame, but that was a kludge.
    // 
    // So the rule now is that the tile *immediately* fills with the pressure-moved water as soon as it empties.
    // This saves us a kludge, and also saves us from having to do the "deleted water" and "inserted water" recomputations
    // in the cases where the water would just be replaced anyway.
    
    // TODO prefer: is opposite the direction we moved (can we do that...?)
    // TODO prefer: is where our velocity is leaving
    vector<tile_location> adj_tiles_that_want_to_fill_us_via_pressure;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const tile_location adj_loc = loc + dir;
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        water_group_identifier group_id = acquire_group_id_of_water_tile(adj_loc);
        group = &persistent_water_groups.find(group_id)->second;
        if (group.suckable_tiles_by_height.any_above(loc.coords().z)) {
          adj_tiles_that_want_to_fill_us_via_pressure.push_bac(adj_loc);
        }
      }
    }
    
    if (!adj_tiles_that_want_to_fill_us_via_pressure.empty()) {
      tile_location const& tile_pulled_from = adj_tiles_that_want_to_fill_us_via_pressure[rand()%(adj_tiles_that_want_to_fill_us_via_pressure.size())];
      water_group_identifier group_id = acquire_group_id_of_water_tile(tile_pulled_from);
      group = &persistent_water_groups.find(group_id)->second;
      
      assert(group.mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(loc));
    }
    else {
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const tile_location adj_loc = loc + dir;
        if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
          water_group_identifier group_id = acquire_group_id_of_water_tile(adj_loc);
          group = &persistent_water_groups.find(group_id)->second;
          assert(!group.mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(loc));
        }
      }
    }
  }
  
  // old_substance_type and new_substance_type shouldn't change after this point
  
  // If we're removing water, we have complicated computations to do. We do it in this order:
  // 1) Gather as much information as possible BEFORE physically changing anything (while we're still marked as water)
  // 2) Physically change us
  // 3) Update the relatively-isolated cached info like interiorness
  // 4) Update the relatively-interconnected cached info of our group
  
  persistent_water_group_info *water_group = nullptr;
  if (old_substance_type == GROUPABLE_WATER && new_substance_type != GROUPABLE_WATER) {
    // ==============================================================================
    // 1) Gather as much information as possible without physically changing anything
    //    (while we're still marked as water)
    // ==============================================================================
    water_group = &persistent_water_groups.find(acquire_group_id_of_water_tile(loc))->second;
  }
  
  // For inserting water, we check group membership BEFORE
  // updating the caches, because the adjacent tiles won't know their group
  // membership if they've all become interior. (We could look it up, but
  // it's easier this way.) And it's more convenient to merge the groups
  // now, too.
  
  if (new_substance_type == GROUPABLE_WATER && old_substance_type != GROUPABLE_WATER) {
    // Inserting water: Are we adjacent to an existing group?
    // If we're next to one adjacent group, we will merely join that group.
    // If we're adjacent to more than one adjacent group, then our existence
    // constitutes a merger of those two groups, so we execute that.
    water_group_identifier group_id = NO_WATER_GROUP;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const location adj_loc = loc + dir;
      auto iter = water_groups_by_surface_tile.find(adj_loc);
      if (iter != water_groups_by_surface_tile.end()) {
        if (group_id == NO_WATER_GROUP) {
          group_id = iter->second;
        }
        else {
          // Two groups we join! Merge them. merge_water_groups() automatically
          // handles the "merge the smaller one into the larger one" optimization
          // and returns the ID of the group that remains.
          group_id = merge_water_groups(group_id, iter->second, water_groups_by_surface_tile);
        }
      }
      
      // Now we're either adjacent to one water group or none. If none, we have to create a new one for ourself.
      if (group_id == NO_WATER_GROUP) {
        group_id = make_new_water_group(next_water_group_identifier, persistent_water_groups);
      }
      
      water_group = &persistent_water_groups.find(group_id)->second;
    }
  }
  
  // ==============================================================================
  // 2) Physically change us
  // ==============================================================================
  mutable_stuff_at(loc).set_contents(new_substance_type); // TODO figure out how to integrate this with tile_changing_implementations.cpp
  if (!is_fluid(new_substance_type)) {
    active_fluids.erase(loc);
  }
  
  // ==============================================================================
  // 3) Update the relatively-isolated cached info like interiorness
  // ==============================================================================
  if (old_substance_type == GROUPABLE_WATER && new_substance_type != GROUPABLE_WATER) {
    groupable_water_dimensional_boundaries_TODO_name_this_better.handle_tile_removal(loc);
  }
  if (new_substance_type == GROUPABLE_WATER && old_substance_type != GROUPABLE_WATER) {
    groupable_water_dimensional_boundaries_TODO_name_this_better.handle_tile_insertion(loc);
  }
  if (new_substance_type != old_substance_type) {
    // Changes can activate nearby water.
    
    // NOTE "Adjacent tile conditions for activation/deactivation": The only relevant ones are
    // the one directly below, the ones cardinally-horizontally, and the ones horizontally-and-below.
    // at the 2-diagonals. This comment is duplicated one one other place in this file.
    
    // Here, that means we need to check the opposite - the tiles horizontally, above, and above-horizontally.
    // Water is activated-if-necessary by calling unordered_set::operator[], which default-constructs one
    // if there isn't one there already.
    const tile_location uploc = loc + cdir_zplus;
    if (is_fluid(uploc.stuff_at().contents()) active_fluids[uploc];
    
    for (EACH_CARDINAL_DIRECTION(dir)) {
      if (dir.v.dot<sub_tile_distance>(cdir_zplus.v) == 0) {
        const tile_location adj_loc = loc + dir;
        const tile_location diag_loc = adj_loc + cdir_zplus;
        // We could be more conservative about what we activate, by checking the actual details of
        // what changed and how that affects things, but that would just invite missing cases,
        // and it wouldn't speed stuff up noticeably because the thing will just deactivate again
        // on the following frame if it's wrongly activated.
        if (is_fluid(adj_loc.stuff_at().contents()) active_fluids[adj_loc];
        if (is_fluid(diag_loc.stuff_at().contents()) active_fluids[diag_loc];
      }
    }
  }
  if (interiorness_checking_type(new_substance_type) != interiorness_checking_type(old_substance_type)) {
    bool we_are_now_interior = true;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const location adj_loc = loc + dir;
      if (interiorness_checking_type(adj_loc) != interiorness_checking_type(new_substance_type)) {
        // Between us and them is a 'different types' interface, so neither us nor them is interior:
        we_are_now_interior = false;
        if (adj_loc.stuff_at().is_interior()) {
          become_not_interior(adj_loc, things_exposed_to_collision);
        }
      }
      else {
        // If we've become the same as the other tile, we may have made that tile interior
        if (!adj_loc.stuff_at().is_interior()) {
          bool they_should_be_interior = true;
          for (EACH_CARDINAL_DIRECTION(d2)) {
            if (interiorness_checking_type((adj_loc + d2).stuff_at().contents()) != interiorness_checking_type(adj_loc.stuff_at())) {
              they_should_be_interior = false;
              break;
            }
          }
          if (they_should_be_interior) {
            become_interior(adj_loc, things_exposed_to_collision);
          }
        }
      }
    }
    if (we_are_now_interior && !loc.stuff_at().is_interior()) {
      become_interior(loc, things_exposed_to_collision);
    }
    if (!we_are_now_interior && loc.stuff_at().is_interior()) {
      become_not_interior(loc, things_exposed_to_collision);
    }
  }
  
  // ==============================================================================
  // 4) Update the relatively-interconnected cached info of water groups
  // ==============================================================================
  
  // For creating water, there are only a few more caches to update.
  if (new_substance_type == GROUPABLE_WATER && old_substance_type != GROUPABLE_WATER) {
    // We might need to designate ourself as a surface tile,
    // and we might need to designate neighbors as non-surface tiles,
    // if we've closed their last liberty (to use Go terms)
    bool there_are_any_adjacent_tiles_that_are_not_groupable_water = false;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const location adj_loc = loc + dir;
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        bool adjacent_tile_has_any_adjacent_tiles_that_are_not_groupable_water = false;
        for (EACH_CARDINAL_DIRECTION(d2)) {
          if ((adj_loc + d2).stuff_at().contents() != GROUPABLE_WATER) {
            adjacent_tile_has_any_adjacent_tiles_that_are_not_groupable_water = true;
            break;
          }
        }
        if (!adjacent_tile_has_any_adjacent_tiles_that_are_not_groupable_water) {
          water_groups_by_surface_tile.erase(adj_loc);
          water_group->surface_tiles.erase(adj_loc);
        }
      }
      else {
        there_are_any_adjacent_tiles_that_are_not_groupable_water = true;
      }
    }
    if (there_are_any_adjacent_tiles_that_are_not_groupable_water) {
      water_groups_by_surface_tile.insert(loc);
      water_group->surface_tiles.insert(loc);
    }
    
    ++water_group->num_tiles_by_height[loc.z];
    
    // Adding a tile at a specific height can change pressure at that height, but not anywhere above
    water_group->pressure_caches.erase(water_group->pressure_caches.begin(), water_group->pressure_caches.lower_bound(loc.z+1));
  }
  
  if (old_substance_type == GROUPABLE_WATER && new_substance_type != GROUPABLE_WATER) {
    // If we were a surface tile of the group, we are no longer.
    water_groups_by_surface_tile.erase(loc);
    water_group->surface_tiles.erase(loc);
    
    // We may have exposed other group tiles, which now need to be marked as the group surface.
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const location adj_loc = loc + dir;
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        water_groups_by_surface_tile.insert(adj_loc);
        water_group->surface_tiles.insert(adj_loc);
      }
    }
    
    water_group->suckable_tiles_by_height.erase(loc);
    
    --water_group->num_tiles_by_height[loc.z];
    
    // Removing a tile from a specific height can change pressure at that height, but not anywhere above
    water_group->pressure_caches.erase(water_group->pressure_caches.begin(), water_group->pressure_caches.lower_bound(loc.z+1));
    
    // Our disappearance might have split the group.
    // If we *did* split the group, we're going to have to flood-fill along the surfaces of all the components in order to divide the groups from each other.
    // If we *didn't*, and we start a breadth-first flood fill from the adjacent tiles, then the fill groups will probably run into each other almost immediately. When that happens, we know that the adjacent tiles are connected.
  
    // ==============================================================================
    //  Start of flood-fill-based group-splitting algorithm
    // ==============================================================================
    
    struct water_splitting_info {
      unordered_map<tile_location, unordered_set<tile_location>> collected_locs_by_neighbor;
      unordered_map<tile_location, tile_location> neighbors_by_collected_loc;
      unordered_map<tile_location, std::queue<tile_location>> disconnected_frontiers;
      
      // Try to collect a location into one of the fills.
      // If no one has claimed it, claim it and add it to your frontier.
      // If you've claimed it, ignore it and move on.
      // If one of the other collectors has claimed it, then we now know that those two are connected,
      //     so merge the two of them.
      // returns true if-and-only-if the operation should destroy the collector by merging it into another collector.
      bool try_collect_loc(tile_location collector, tile_location collectee) {
        if (collectee.stuff_at().is_membrane_water()) {
          unordered_map<tile_location, tile_location>::iterator iter = neighbors_by_collected_loc.find(collectee);
          
          if (iter == neighbors_by_collected_loc.end()) {
            disconnected_frontiers[collector].push_back(collectee);
            collected_locs_by_neighbor[collector].insert(collectee);
            neighbors_by_collected_loc[collectee] = collector;
          }
          else {
            if (iter->second != collector) {
              // Found an overlap - merge the two!
              // Hack: If there are only two collectors left, then we don't need to do any more processing - just get destroyed, and bail.
              if (disconnected_frontiers.size() == 2) return true;
              
              // The collectors will normally be the same size, but if two merged already, then one might be bigger. And we want to merge the smaller one into the bigger one for performance reasons.
              bool this_collector_is_bigger = collected_locs_by_neighbor[collector].size() > collected_locs_by_neighbor[iter->second].size()
              tile_location  bigger_collector = (this_collector_is_bigger ? collector    : iter->second);
              tile_location smaller_collector = (this_collector_is_bigger ? iter->second : collector   );
              
              for (tile_location const& loc : collected_locs_by_neighbor[smaller_collector]) {
                collected_locs_by_neighbor[bigger_collector].insert(loc);
                neighbors_by_collected_loc[loc] = bigger_collector;
              }
              while (!disconnected_frontiers[smaller_collector].empty()) {
                disconnected_frontiers[bigger_collector].push_back(disconnected_frontiers[smaller_collector].front());
                disconnected_frontiers[smaller_collector].pop();
              }
              
              return (!this_collector_is_bigger);
            }
          }
        }
      }
    }
    water_splitting_info inf;
    
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const location adj_loc = loc + dir;
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        inf.frontiers[adj_loc].push_back(adj_loc);
      }
    }
    while(inf.disconnected_frontiers.size() > 1) {
      for (auto p = inf.disconnected_frontiers.begin(); p != inf.disconnected_frontiers.end(); ) {
        tile_location const& which_neighbor = p->first;
        std::queue<tile_location> &frontier = p->second;
        bool destroy_this_frontier = false;
        
        if (frontier.empty()) {
          // If the frontier is empty, that means this route has finished its search and not found any
          // connections to the rest of the water. Therefore, we need to mark this frontier off as a
          // separate group.
          destroy_this_frontier = true;
          
          new_group_id = make_new_water_group(next_water_group_identifier, persistent_water_groups);
          
          persistent_water_group_info &new_group = persistent_water_groups.find(new_group_id)->second;
          for (location const& new_grouped_loc : collected_locs_by_neighbor[which_neighbor]) {
            water_groups_by_surface_tile[new_grouped_loc] = new_group_id;
            new_group.surface_tiles.insert(new_grouped_loc);
            water_group->surface_tiles.erase(new_grouped_loc);
            if (water_group->suckable_tiles_by_height.erase(new_grouped_loc)) {
              new_group.suckable_tiles_by_height.insert(new_grouped_loc);
            }
          }
          new_group.recompute_num_tiles_by_height_from_surface_tiles();
          for (auto const& p : new_group.num_tiles_by_height) {
            water_group->num_tiles_by_height[p.first] -= p.second;
          }
        }
        else {
          tile_location search_loc = frontier.front();
          frontier.pop();
          
          for (EACH_CARDINAL_DIRECTION(dir)) {
            tile_location adj_loc = search_loc + dir;
            
            destroy_this_frontier = try_collect_loc(adj_loc));
            if (destroy_this_frontier) break;
            
            if (adj_loc.stuff_at().is_interior_water()) {
              for (EACH_CARDINAL_DIRECTION(d2)) {
                if (d2.dot<neighboring_tile_differential>(dir) == 0) {
                  destroy_this_frontier = try_collect_loc(adj_loc + d2);
                  if (destroy_this_frontier) break;
                }
              }
            }
          }
        }
          
        if (destroy_this_frontier) {
          inf.disconnected_frontiers.erase(p++);
        }
        else {
          ++p;
        }
      }
    }
  
    // ==============================================================================
    //  End of flood-fill-based group-splitting algorithm
    // ==============================================================================
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

// TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
int obstructiveness(tile_contents tc) {
  if (tc == ROCK) return 4;
  else if (tc == RUBBLE) return 3;
  else if (is_water(tc)) return 2;
  else if (tc == AIR) return 1;
  else assert(false);
}

struct active_fluid_temporary_data {
  active_water_temporary_data():new_progress(0){}
  value_for_each_cardinal_direction<sub_tile_distance> new_progress;
};

void update_fluids(world &w, active_fluids_t &active_fluids, persistent_water_groups_t &persistent_water_groups) {
  // ==============================================================================
  //  Phase 1
  //  Compute all the velocities and movement.
  //  There are a lot of relatively-separable rules governing how water tiles move.
  // ==============================================================================
  
  unordered_map<tile_location, active_fluid_temporary_data> temp_data;
  for (pair<const tile_location, active_fluid_tile_info> &p : active_fluids) {
    tile_location const& loc = p.first;
    active_fluid_tile_info &fluid = p.second;
    tile const& t = loc.stuff_at();
    assert(is_fluid(t.contents()));
    
    // initialize...
    temp_data[loc];
    
    // Gravity applies to everyone
    fluid.velocity += gravity_acceleration;
      
    // Slight air resistance proportional to the square of the velocity (mostly there to make a natural cap velocity for falling water)
    fluid.velocity -= (fluid.velocity * fluid.velocity.magnitude_within_32_bits()) / air_resistance_constant;
      
    // Relatively large friction against the ground
    for (EACH_CARDINAL_DIRECTION(dir)) {
      if (fluid.blockage_amount_this_frame[dir] > 0) {
        vector3<sub_tile_distance> copy_stationary_in_blocked_direction = fluid.velocity; copy_stationary_in_blocked_direction -= project_onto_cardinal_direction(copy_stationary_in_blocked_direction, dir);
        if (copy_stationary_in_blocked_direction.magnitude_within_32_bits_is_less_than(friction_amount)) {
          fluid.velocity -= copy_stationary_in_blocked_direction;
        }
        else {
          fluid.velocity -= copy_stationary_in_blocked_direction * friction_amount / copy_stationary_in_blocked_direction.magnitude_within_32_bits();
        }
      }
    }
    
    // Groupable water pushes other fluids away from it according to pressure.
    if (t.contents() != GROUPABLE_WATER) {
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const tile_location adj_loc = loc + dir;
        if (adj_loc.stuff_at().contents == GROUPABLE_WATER) {
          if ((loc - dir).stuff_at().contents == GROUPABLE_WATER) {
            // Hack - if we have groupable water pushing from both sides, become groupable ASAP
            fluid.velocity -= project_onto_cardinal_direction(fluid.velocity, dir);
            //fluid.frames_until_can_become_groupable = 0;
          }
          else {
            persistent_water_group_info const& group = w.get_persistent_water_group_by_surface_tile(adj_loc);
            const sub_tile_distance amount_of_vel_in_pressure_receiving_dir = fluid.velocity.dot<sub_tile_distance>(-dir.v);
            const sub_tile_distance deficiency_of_vel = i64sqrt(group.get_pressure_at_height(loc.coords().z)) - amount_of_vel_in_pressure_receiving_dir;
            if (deficiency_of_vel > 0) {
              fluid.velocity += vector3<sub_tile_distance>(-dir.v) * deficiency_of_vel;
            }
          }
        }
      }
    }
    
    // Velocity causes progress.
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const sub_tile_distance dp = vel_ref.dot<sub_tile_distance>(dir.v);
      if (dp > 0) temp_data[loc].new_progress[dir] += dp;
    }
    
    // Water that's blocked, but can go around in a diagonal direction, also makes "progress" towards all those possible directions (so it'll go in a random direction if it could go around in several different diagonals, without having to 'choose' one right away and gain velocity only in that direction). The main purpose of this is to make it so that water doesn't stack nicely in pillars, or sit still on steep slopes.
    // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const tile_location dst_loc = loc + dir;
      if (dst_loc.stuff_at().contents() == AIR) {
        for (EACH_CARDINAL_DIRECTION(d2)) {
          const tile_location diag_loc = dst_loc + d2;
          if (fluid.blockage_amount_this_frame[d2] > 0 && d2.v.dot<sub_tile_distance>(dir.v) == 0 && obstructiveness(diag_loc.stuff_at()) < obstructiveness((loc + d2).stuff_at())) {
            temp_data[loc].new_progress[dir] += fluid.blockage_amount_this_frame[d2];
          }
        }
      }
    }
    
    // blockage_amount_this_frame is just a temporary value that lingers from one frame to the next -
    // we erase it after we use it.
    for (sub_tile_distance &bb : fluid.blockage_amount_this_frame) bb = 0;
  }
  
  // ==============================================================================
  //  Phase 2
  //  We've computed all the velocities and progress. Now we check which tiles
  //  have made enough progress to move. For each move someone wants to make,
  //  we record that 'wanted move', then we shuffle all the wanted moves and
  //  execute them in a random order, as opposed to trying to find some way to
  //  composite them when they conflict with each other.
  // ==============================================================================
  
  vector<wanted_move> wanted_moves;
  for (pair<const tile_location, active_fluid_tile_info> &p : active_fluids) {
    tile_location const& loc = p.first;
    active_fluid_tile_info &fluid = p.second;
    tile const& t = loc.stuff_at();
    assert(t.contents() == WATER);
    for (EACH_CARDINAL_DIRECTION(dir)) {
      sub_tile_distance &progress_ref = fluid.progress[dir];
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
  
  
  // It doesn't matter that we won't add temp_data for nearby tiles that are activated during this loop by the insertion and deletion of water.
  for (const wanted_move move : wanted_moves) {
    const tile_location dst = move.src + move.dir;
    tile const& src_tile = move.src.stuff_at();
    tile const& dst_tile = dst.stuff_at();
    // in certain situations we shouldn't try to move water more than once
    if (disturbed_tiles.find(move.src) != disturbed_tiles.end()) continue;
    // anything where the water was yanked away should have been marked "disturbed"
    assert(is_water(src_tile.contents()));
    
    active_fluid_tile_info &src_fluid = active_fluids.find(move.src)->second;
    
    // Rubble is 'heavier' than water and basically ignores it, while water has to yield to rubble.
    int src_weight = obstructiveness(src_tile.contents());
    int dst_weight = obstructiveness(dst_tile.contents());
    if (src_weight > dst_weight) {
      bool dst_was_active_fluid = false;
      active_fluid_tile_info dst_info_store;
      tile_contents dst_old_contents = dst_tile.contents();
      auto i = active_fluids.find(dst);
      if (i != active_fluids.end()) {
        dst_was_active_fluid = true;
        dst_info_store = i->second;
      }
    
      src_fluid.progress[move.dir] -= progress_necessary(move.dir);
      // activate dst by creating active fluid info via operator[], move us to it
      active_fluid_tile_info &dst_fluid = (active_fluids[dst] = src_fluid);
      // Water always becomes ungroupable when it moves
      w.replace_substance(dst, AIR, UNGROUPABLE_WATER);
      temp_data[dst] = temp_data[move.src];
      
      w.replace_substance(move.src, src_tile.contents(), dst_old_contents);
      if (dst_was_active_fluid) src_water = dst_info_store;
      temp_data.erase(move.src);
      disturbed_tiles.insert(move.src);
      disturbed_tiles.insert(move.dst);
      
      // If a tile moves, we're now content to assume that it was moving because it had a realistic velocity in that direction, so we should continue with that assumption.
      const sub_tile_distance amount_of_new_vel_in_movement_dir = dst_water.velocity.dot<sub_tile_distance>(move.dir.v);
      const sub_tile_distance deficiency_of_new_vel_in_movement_dir = move.amount_of_the_push_that_sent_us_over_the_threshold - amount_of_new_vel_in_movement_dir;
      if (deficiency_of_new_vel_in_movement_dir > 0) {
        dst_water.velocity += vector3<sub_tile_distance>(move.dir.v) * deficiency_of_new_vel_in_movement_dir;
      }
      
      // Don't lose movement to rounding error during progress over multiple tiles:
      dst_water.progress[move.dir] = std::min(move.excess_progress, progress_necessary(move.dir));
    }
      
    else {
      // We're blocked... However, in one case, we don't register as blocked.
      // If the current move is "go down into obstruction while being part of a group of
      // groupable water with free lower exterior surfaces", then 
      // instead of getting blocked, we get sucked to one of the exterior surfaces.
      bool blocked = true;
      const bool becoming_suckable = (src_tile.contents() == GROUPABLE_WATER) && (move.dir == cdir_zminus) && ((move.src + cdir_zplus).stuff_at().contents() != GROUPABLE_WATER);
      
      if (becoming_suckable) {
        const water_group_identifier group_id = acquire_group_id_of_water_tile(move.src);
        persistent_water_group_info &group = persistent_water_groups.find(group_id)->second;
        // Caution - if this returns true, it has removed the water we're currently looking at.
        // (Otherwise, it hasn't.)
        if (group.mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(move.src)) {
          blocked = false;
        }
      }
      
      if (blocked) {
        // We're blocked. If we're trying to run into them faster than they think they're moving away from us, stop it.
        // (Unless we're slower than min_convincing_speed, which is a kludge that's important for the "fall off pillars" rule.)
        src_water.blockage_amount_this_frame[move.dir] = move.excess_progress;
        
        sub_tile_distance dst_vel_in_movement_dir = 0;
        auto i = active_fluids.find(dst);
        if (i != active_fluids.end()) dst_vel_in_movement_dir = i->second.velocity.dot<sub_tile_distance>(move.dir.v);
        const sub_tile_distance acceptable_remaining_vel_for_us = std::max(dst_vel_in_movement_dir - gravity_acceleration_magnitude * 2, min_convincing_speed);
        const sub_tile_distance our_vel_in_movement_dir = src_water.velocity.dot<sub_tile_distance>(move.dir.v);
        const sub_tile_distance excess_vel = our_vel_in_movement_dir - acceptable_remaining_movement_for_us;
        if (excess_vel > 0) {
          src_water.velocity -= vector3<sub_tile_distance>(move.dir.v) * excess_vel;
        }
      }
    }
  }
  
  // ==============================================================================
  //  Phase 3
  //  We've done all the moving we want to. Now, do some final calculations.
  //  Make water groupable if it's ready for that.
  //  Deactive water if needs to be deactivated.
  // ==============================================================================
  
  for (pair<const tile_location, active_fluid_tile_info> &p : active_fluids) {
    tile_location const& loc = p.first;
    active_fluid_tile_info &fluid = p.second;
    tile const& t = loc.stuff_at();
    if (t.contents() == UNGROUPABLE_WATER) {
      //--fluid.frames_until_can_become_groupable;
      //if (fluid.frames_until_can_become_groupable <= 0) {
      //  fluid.frames_until_can_become_groupable = 0;
        if (!fluid.velocity.magnitude_within_32_bits_is_greater_than(min_convincing_speed)) {
          w.replace_substance(loc, UNGROUPABLE_WATER, GROUPABLE_WATER);
        }
      //}
    }
    if (t.contents() == GROUPABLE_WATER) {
      if (fluid.velocity.magnitude_within_32_bits_is_greater_than(min_convincing_speed)) {
        w.replace_substance(loc, GROUPABLE_WATER, UNGROUPABLE_WATER);
      }
    }
  }
  
  for (pair<const tile_location, active_fluid_tile_info> &p : active_fluids) {
    tile_location const& loc = p.first;
    active_fluid_tile_info &fluid = p.second;
    tile const& t = loc.stuff_at();
    if (fluid.is_in_inactive_state()) {
      // Be a little paranoid about making sure fluids obeys all the proper conditions of inactivity
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
      
      // NOTE "Adjacent tile conditions for activation/deactivation": The only relevant ones are
      // the one directly below, the ones cardinally-horizontally, and the ones horizontally-and-below.
      // at the 2-diagonals. This comment is duplicated one one other place in this file.
      const tile_location downloc = loc + cdir_zminus;
      if (!(downloc.stuff_at().contents() == GROUPABLE_WATER || downloc.stuff_at().contents() == ROCK)) continue;

      // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
      for (EACH_CARDINAL_DIRECTION(dir)) {
        if (dir.v.dot<sub_tile_distance>(cdir_zminus.v) == 0) {
          const tile_location dst_loc = loc + dir;
          if (dst_loc.stuff_at().contents() == AIR) {
            const tile_location diag_loc = dst_loc + cdir_zminus;
            if (obstructiveness(diag_loc.stuff_at()) < obstructiveness((loc + cdir_zminus).stuff_at())) {
              can_deactivate = false; break;
            }
          }
        }
      }
        
      if (can_deactivate) w.deactivate_water(loc);
    }
  }
  for (auto const& p : temp_data) {
    tile_location const& loc = p.first;
    if (water_movement_info *water = w.get_active_water_tile(loc)) {
      if (water->is_in_inactive_state()) {
    }
  }
}

// Constructing one of these in the default way yields the natural inactive state:
active_fluid_tile_info::active_fluid_tile_info():
   velocity(inactive_fluid_velocity),
   progress(0),
   blockage_amount_this_frame(0)
{
  progress[cdir_zminus] = progress_necessary(cdir_zminus);
}
  
bool active_fluid_tile_info::is_in_inactive_state()const {
  // TODO: does it make sense that we're ignoring the 1-frame-duration variable "blockage_amount_this_frame"?
  for (EACH_CARDINAL_DIRECTION(dir)) {
    if (dir.v.z < 0) {
      if (progress[dir] != progress_necessary(dir)) return false;
    }
    else {
      if (progress[dir] != 0) return false;
    }
  }
  return velocity == inactive_fluid_velocity;
}

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
free water away from it (by setting that free water's velocity).

Free water velocity also moves by incrementing 'progress'
in the direction(s) it's moving.

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




