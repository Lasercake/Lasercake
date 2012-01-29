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
#include "tile_physics.hpp"
#include <queue>

/*

This file describes the tile-based physics system.

We use a 3D grid of tiles for some parts of our physics.
The main tile-based part of the physics is FLUID MOTION.

Real-life fluid motion is very complicated, so we use a wide variety of
simplifications in order to simulate it quickly on a large scale.
There are also many rules to cover specific situations; this comment will be
a general overview, not a complete description. Comments elsewhere in the file
describe the nature and purpose of some of those specific rules.

=== OVERVIEW ===

Each tile is a solid mass of some type of substance (rock, water, air, etc);
When they move, they move by exactly an entire tile at a time, and swap with
whatever substance was in the other tile.

Fluid tiles carry a "velocity" with them, which is fairly self-explanatory.
In order to reconcile the fine-grained velocity with the granular tile movement,
fluid tiles also carry "progress" in each of the six directions - when the
progress exceeds a certain threshold, the tile makes a step in that direction.
A tile can have 'progress' in two opposite directions at once.

When a tile runs into something that it can't pass, it gets blocked, and loses
most of its velocity in that direction. Also, if there's room for it to
go past the blocking tile diagonally, then the amount of excess "progress"
that was blocked gets added to its "progress" in each of the four perpendicular
directions. Tiles can't actually move diagonally, but the perpendicular progress
will eventually build up to be enough to move it into the corner, whereupon
its original velocity/progress in the original direction will carry it around
the corner. I call this rule the "fall off pillars" rule, because its original
purpose was to make it so that a stack of single tiles of water wouldn't
stay stacked.

When a tile makes a step in a direction, the threshold amount is subtracted
from its progress in that direction. Lingering progress in unused directions
decays over time.

For some fluids, that's it. Sand is considered a fluid, and it obeys only
the above rules. It falls down due to gravity, and pours off shallow slopes,
but even if there's a huge pile of sand in one side of a J tube, it won't
go up the other side of the J tube, because it doesn't consider pressure.

The main exception is water, which has lots of special rules to deal with
pressure. We mark some water as "groupable", and we keep caches of water
"groups" - masses of "groupable water" that are conneced through direct
cardinal-direction adjacency. And we compute the "pressure" at each level
from the depth of tiles in the group, and at each open surface, we push
water out, and at the top surfaces, we pull water in to replace it.

For optimization, we merely keep a list (actually a hash table from
location to active fluid info) of the active fluid tiles, with their velocity
and progress and stuff. And we carefully remove tiles from the list when
they become inactive, so that our basic computations are no slower than
linear in the number of tiles that are *actually* moving.

=== GROUPS AND PRESSURE: RULES ===

Tiles at the top of a group, which have fallen enough to run into the
next tile down, are considered "suckable".

Air tiles just outside the mass of a group are considered "pushable".

Whenever a single group has pushable and suckable tiles at the same time,
if the pushable tiles are at a lower height than the suckable tiles,
it immediately teleports the topmost suckable waters to the locations of the
bottommost pushable tiles until it's in a stable situation again.

If there's a tile that *would* be pushable but has a fluid tile in it, that
fluid tile gets a velocity related to the pressure at its height.
(This is actually done in the external fluid tile's computation, not the
group's computation. In fact, groups themselves never do any computation per
frame; they just react to things that happen around them.)

There's one catch here: If all water was groupable all the time, a weird
thing would happen when you tried to dump a large lake through a small tube
into an open area. When it first came out of the small tube, it would move
slowly, because the tube only had a small surface area. But when it had gotten
out a little, it would suddenly have more surface area, and shed water much
faster. So there would be a slowly building explosion, which is nothing like
the way water works in real life.

To avoid that, we make it so that water that's *moving* fast enough
is marked not-groupable. So as it pours out of a hole, it becomes ungroupable
until later, and never allows the group to dump its high pressure onto any
tile except the ones right at the mouth of the tube.

So, in the J-tube situation, here's what happens.
The lower end has pushable tiles, and the higher end has suckable tiles,
so it pushes a lot of water out the lower end. That water is probably moving
quickly upwards. So it's not groupable right away. As soon as it moves one
tile upwards, though, it frees up the space it was at, and if there are
suckable tiles left, those spaces are refilled again. And the water moving
upwards eventually stops due to gravity and becomes groupable. This continues
until the tube has levelled off.


=== GROUPS AND PRESSURE: HOW THE CACHING WORKS ===

We don't keep a cache of all the internal tiles of a group. In fact, our
algorithms are designed so that we never have to do any computation for
the interior tiles at all; the worst-case computation issues are only
proportional to the surface area (including water-to-rock surfaces) of
the group. Typical-use cases are proportional to the surface area in
water-to-air surfaces that are actually moving.

For each group, we DO keep a cache of all the surface tiles of the group.
(i.e. tiles that are groupable water but have a neighbor which is not;
that is the meaning of "surface tile" wherever we use that phrase.)
In fact, each surface tile is stored in 2-3 places, for different purposes.
There's a hash table that maps surface tiles to group IDs, so that you can
look up the groups by surface tile. The group info (which you can look up
by ID) contains a hashset of surface tiles, so that you can look up surface
tiles by group. There's a third set that takes the tiles that are surfaces
specifically in the X dimension, and is ordered within each row, so that
we can take an arbitrary interior tile and find the "next" surface tile,
in order to look up what group the interior tile is in, in log(n) time even
if the interior tile is a long way from any surface. That set also allows us
to measure the volume of a group in less-than-linear-in-the-total-volume
time (there's a comment later in the file describing how that works.)

When a new groupable-water tile appears, if it's next to an existing surface,
it joins that group. If it isn't, then it makes its own new group.
("A groupable water tile appears" describes situations in which one appears
from thin air, or one moves from one tile to another, or an existing tile
becomes groupable. Actually, we currently never allow water to remain
groupable when it moves, but anyway.)

When a groupable-water tile disappears, it removes itself from its group.

So, in most cases, a tile moving only takes constant time to compute, and we
never actually have to iterate through all the surface tiles of a group -
just handle the local situation when individual tiles move.

However, there are two problem cases:

1) When a tile is added next to TWO DIFFERENT groups. In this case, we have
to merge the groups; this operation takes time proportional to the surface
area of the smaller group, because we have to update the cached group info
for all of those surface tiles, to have them join the larger group.

2) When a tile is removed in such a way that it SPLITS a group. In fact, when
we remove a tile, we CAN'T IMMEDIATELY KNOW whether the group is being split.
In this case, we use a flood-fill algorithm from each adjacent tile, over
group surface tiles, to figure out the total area of the split groups (and if
the flood-fills run into each other, then we know that the group is still
in one piece.) Under normal circumstances, the group isn't split, and the
flood-fills run into each other almost immediately, so removing a tile is still
no worse than constant time. When the group actually DOES split, we have to take
time proportional to the total surface area of the *smaller* group. (It also
takes time proportional to the size of the group in certain ugly cases, like
when there's a circle of water that splits at one side - it's technically still
the same group, but you have to look all the way around to discover that.

=== INFINITE OCEANS ===

(Infinite oceans aren't implemented yet, but this will be correct once they are.)

Both of those cases have a worst-case scenario that takes the size of the
SMALLER group - which means that water merging or joining with an infinite ocean
does not take infinite time! And since an ocean is almost entirely inactive,
we don't need to do infinite computations to figure out how it moves each frame,
either.

But what about the "pushable" and "suckable" tiles? For infinite oceans, we just
assume that they have infinitely many pushable tiles at their surface height,
and infinitely many suckable tiles at their surface height. This actually
makes computations *simpler* for the rest of the group - you never have to check
for the real pushable or suckable tiles, just the assumed ones - and since they're,
on average, infinitely far away, you don't have to figure out what actually happens
at their locations - just pull water out of thin air, or push it into nonexistence.

In fact, if you know that a tile is connected to an infinite ocean, you don't need
ANY of its other group information at all - because of the way the pressure is
computed, its pressure is directly proportional to its depth from the ocean height.
This also solves the "two infinite oceans split from each other" case that could
take infinite time if we didn't handle it specially - if two groups are connected
to an infinite ocean, we can just assume that that means they're connected to each
other, because the group info doesn't matter.

So how do we learn, when we're pathfinding over a group, whether it's connected to
an infinite ocean, without looking infinitely far?

TODO: Figure out a good way to do that (I can think of some, but they all have
worst-case scenarios that get really bad in the case of an infinitely long dike
dividing two infinite oceans.)

*/


using namespace tile_physics_impl;

////////////////////////////////////////////
// Miscellaneous helpful stuff.

// This is the only file that's allowed to change the contents of tiles at locations.
// Other files should change tile contents through replace_substance,
// and shouldn't be in the business of altering tile caches at all.
// (TODO: also add something for the there_is_an_object_here_that_affects_the_tile_based_physics thing)
tile& tile_physics_impl::mutable_stuff_at(tile_location const& loc) { return loc.wb->get_tile(loc.v); }


state_t& get_state(world& w) {
  return get_state(w.tile_physics());
}

water_group_identifier make_new_water_group(water_group_identifier &next_water_group_identifier, persistent_water_groups_t &persistent_water_groups) {
  water_group_identifier this_id = next_water_group_identifier++;
  persistent_water_groups[this_id]; // operator[] inserts a default-constructed one
  return this_id;
}

tile_physics_state_t::tile_physics_state_t(world& w) : state_(new state_t(w)) {}
tile_physics_state_t::tile_physics_state_t(tile_physics_state_t const& other)
  : state_(new state_t(*other.state_)) {}
tile_physics_state_t& tile_physics_state_t::operator=(tile_physics_state_t const& other) {
  state_.reset(new state_t(*other.state_));
  return *this;
}
tile_physics_state_t::~tile_physics_state_t() noexcept {}

void replace_substance(
   state_t& state,
   tile_location const& loc,
   tile_contents old_substance_type,
   tile_contents new_substance_type) {
  state.access_the_world.replace_substance(loc, old_substance_type, new_substance_type);
}

water_group_identifier get_water_group_id_by_grouped_tile(state_t const& state, tile_location const& loc) {
  {
    water_groups_by_location_t::const_iterator i = state.water_groups_by_surface_tile.find(loc);
    if (i != state.water_groups_by_surface_tile.end()) {
      return i->second;
    }
  }

  // Crap, we don't know what group we're part of unless we find a surface tile!
  // Find the next surface tile in some arbitrary direction.
  // That tile will tell us what group we're in.
  auto iter = state.groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.lower_bound(loc);
  if (iter == state.groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.end()) {
    return NO_WATER_GROUP;
  }
  tile_location const& surface_loc = *iter;
  if (surface_loc.coords().y != loc.coords().y || surface_loc.coords().z != loc.coords().z) {
    return NO_WATER_GROUP;
  }
  return state.water_groups_by_surface_tile.find(surface_loc)->second;
}
persistent_water_group_info const& get_water_group_by_grouped_tile(state_t const& state, tile_location const& loc) {
  return state.persistent_water_groups.find(get_water_group_id_by_grouped_tile(state, loc))->second;
}


///////////////////////////////
// Debugging functions:
void dump_boundary_stuff(groupable_water_dimensional_boundaries_TODO_name_this_better_t &g) {
  for (auto const& foo : g.x_boundary_groupable_water_tiles)
    std::cerr << "  " << foo.coords() << "\n";
}
void dump_group_info(persistent_water_group_info const& g) {
  std::cerr << "Suckable tiles by height:\n";
  for (auto const& foo : g.suckable_tiles_by_height.as_map()) {
    std::cerr << "  At height " << foo.first << ":\n";
    for(auto const& bar : foo.second.as_unordered_set()) {
      std::cerr << "    " << bar.coords() << "\n";
    }
  }
  std::cerr << "Pushable tiles by height:\n";
  for (auto const& foo : g.pushable_tiles_by_height.as_map()) {
    std::cerr << "  At height " << foo.first << ":\n";
    for(auto const& bar : foo.second.as_unordered_set()) {
      std::cerr << "    " << bar.coords() << "\n";
    }
  }
  std::cerr << "Surface tiles:\n";
  for (auto const& foo : g.surface_tiles) {
    std::cerr << "  " << foo.coords() << "\n";
  }
  std::cerr << "Width-of-widest-level-so-far caches:\n";
  for (auto const& foo : g.width_of_widest_level_so_far_caches) {
    std::cerr << "  " << foo.first << ": " << foo.second << "\n";
  }
  std::cerr << "Pressure caches:\n";
  for (auto const& foo : g.pressure_caches) {
    std::cerr << "  " << foo.first << ": " << foo.second << "\n";
  }
  std::cerr << "Num tiles by height:\n";
  for (auto const& foo : g.num_tiles_by_height) {
    std::cerr << "  " << foo.first << ": " << foo.second << "\n";
  }
}
void dump_all_groups(state_t& state) {
  for (auto const& p : state.persistent_water_groups) {
    std::cerr << "\n==GROUP "<< p.first<<"==\n";
    dump_group_info(p.second);
  }
}

/*void check_pushable_tiles_sanity(state_t& state) {
  for (auto const& p : state.persistent_water_groups) {
    auto const& g = p.second;
    for (auto const& foo : g.pushable_tiles_by_height.as_map()) {
      for(auto const& bar : foo.second.as_unordered_set()) {
        bool sane = false;
        for(cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
          if(g.surface_tiles.find(bar + dir) != g.surface_tiles.end()) {
            sane = true;
            break;
          }
        }
        assert(sane);
      }
    }
  }
}*/

// Hacky debugging function that duplicates code from elsewhere.
void check_group_surface_tiles_cache_and_layer_size_caches(state_t& state, persistent_water_group_info const& g) {
  persistent_water_group_info h;
  // Scan from an arbitrary surface tile of g
  if (g.surface_tiles.empty()) {
    std::cerr << "Checking an empty group, why the hell...?";
    return;
  }
  
  struct group_collecting_info {
    group_collecting_info(tile_location const& start_loc, persistent_water_group_info &new_group):new_group(new_group) {
      try_collect_loc(start_loc);
    }
    
    persistent_water_group_info &new_group;
    std::vector<tile_location> frontier;
    void try_collect_loc(tile_location const& loc) {
      if (loc.stuff_at().contents() == GROUPABLE_WATER && !loc.stuff_at().is_interior() && new_group.surface_tiles.find(loc) == new_group.surface_tiles.end()) {
        frontier.push_back(loc);
        new_group.surface_tiles.insert(loc);
        if (loc.get_neighbor<zplus>(CONTENTS_ONLY).stuff_at().contents() != GROUPABLE_WATER) new_group.suckable_tiles_by_height.insert(loc);
      }
    }
  };
  
  group_collecting_info inf(*g.surface_tiles.begin(), h);
  while(!inf.frontier.empty()) {
    const tile_location search_loc = inf.frontier.back();
    inf.frontier.pop_back();
    
    std::array<tile_location, num_cardinal_directions> search_neighbors = get_all_neighbors(search_loc, CONTENTS_AND_LOCAL_CACHES_ONLY);
    for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
      tile_location const& adj_loc = search_neighbors[dir];
      inf.try_collect_loc(adj_loc);
      
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER && adj_loc.stuff_at().is_interior()) {
        std::array<tile_location, num_cardinal_directions> adj_neighbors = get_perpendicular_neighbors(adj_loc, dir, CONTENTS_AND_LOCAL_CACHES_ONLY);
        for (cardinal_direction d2 = 0; d2 < num_cardinal_directions; ++d2) {
          if (cardinal_directions_are_perpendicular(dir, d2)) {
            inf.try_collect_loc(adj_neighbors[d2]);
          }
        }
      }
    }
  }
  
  h.recompute_num_tiles_by_height_from_surface_tiles(state);
  
  bool check_succeeded = true;
  
  for (auto const& s : h.surface_tiles) {
    if(g.surface_tiles.find(s) == g.surface_tiles.end()) {
      std::cerr << "Missing surface tile: " << s.coords() << "\n";
      check_succeeded = false;
    }
  }
  for (auto const& s : g.surface_tiles) {
    if(h.surface_tiles.find(s) == h.surface_tiles.end()) {
      std::cerr << "Extra surface tile: " << s.coords() << "\n";
      check_succeeded = false;
    }
  }
  for (auto const& p : h.num_tiles_by_height) {
    auto i = g.num_tiles_by_height.find(p.first);
    if (i == g.num_tiles_by_height.end()) {
      std::cerr << "Missing height listing: " << p.first << "\n";
      check_succeeded = false;
    }
    else {
      if (i->second != p.second) {
        std::cerr << "Number of tiles at height " << p.first << " listed as " << i->second << " when it should be " << p.second << "\n";
        check_succeeded = false;
      }
    }
  }
  for (auto const& p : g.num_tiles_by_height) {
    auto i = h.num_tiles_by_height.find(p.first);
    if (i == h.num_tiles_by_height.end()) {
      std::cerr << "Extra height listing: " << p.first << "\n";
      check_succeeded = false;
    }
  }
  assert(check_succeeded);
}




// Initialization.
// When we initialize a tile...
// If it's not groupable water, we only need to compute its interiorness.
// If it is, then we need to make sure the caches of the group it's in are in order.
// We can check whether the tile is in a group that's already been computed, but otherwise,
// that means doing a flood-fill search to find the group's entire surface, then computing stuff about it.
// The complication comes when the water tile in question is attached to an infinite ocean - we can't
// flood-fill an entire infinite ocean.
//
// Infinite oceans simplify the group computations significantly. One thing they don't simplify is this:
// If a tile, for some reason, disappears from the middle of an infinite ocean, then we still want to be
// able to quickly find out which group it's part of, using the dimensional-boundary-tile thing.
// Which means that *either* we need to make an arbitrary end-surface for infinity (and keep updating
// it every time the player reaches a location that far away)... *or* we can make the infinite groups
// not have a surface *at all*, so that if a tile searches for a surface and finds a badly-formed one,
// then it knows it's part of an infinite group. (Or we could combine those - mark all the real surface
// tiles - if you hit one, it tells you you're in the infinite group, and if you don't, you know from
// that fact.)
//
// I haven't actually implemented infinite oceans yet - TODO do that sometime
//
// Technically, we don't have to initialize a group when it is generated - we only have to initialize it
// when it would DO anything. That's a highly unnecessary optimization, considering that initalizing a
// single group hardly takes any time.
bool is_ungrouped_surface_tile(tile_location const& loc, water_groups_by_location_t const& water_groups_by_surface_tile) { return loc.stuff_at().contents() == GROUPABLE_WATER && !loc.stuff_at().is_interior() && water_groups_by_surface_tile.find(loc) == water_groups_by_surface_tile.end(); }

void initialize_water_group_from_tile_if_necessary(state_t& state, tile_location const& loc,
   groupable_water_dimensional_boundaries_TODO_name_this_better_t &groupable_water_dimensional_boundaries_TODO_name_this_better,
   water_group_identifier &next_water_group_identifier,
   persistent_water_groups_t &persistent_water_groups,
   water_groups_by_location_t &water_groups_by_surface_tile) {
  
  if (get_water_group_id_by_grouped_tile(state, loc) != NO_WATER_GROUP) return;
  
  // Walk until we get a location on the surface.
  // We *could* just ignore interior tiles and only do the initialization if we started from a surface
  // tile, since we never initialize interior tiles without also initializing surface tiles... well,
  // unless we started in the middle of a giant ocean or something. Anyway, it's cleaner to handle
  // interior tiles, and the cost is trivial since we only ever do this once per group-initialization.
  tile_location surface_loc = loc;
  while (true) {
    const tile_location next_loc = surface_loc.get_neighbor<zplus>(CONTENTS_AND_LOCAL_CACHES_ONLY);
    if (next_loc.stuff_at().contents() != GROUPABLE_WATER) break;
    surface_loc = next_loc;
  }
  
  // Flood-fill the surface tiles.
  // This partially duplicates the "groups splitting" algorithm later in this file.
  
  const water_group_identifier new_group_id = make_new_water_group(next_water_group_identifier, persistent_water_groups);
  persistent_water_group_info &new_group = persistent_water_groups.find(new_group_id)->second;
  
  struct group_collecting_info {
    group_collecting_info(tile_location const& start_loc, water_group_identifier new_group_id, persistent_water_group_info &new_group, groupable_water_dimensional_boundaries_TODO_name_this_better_t &groupable_water_dimensional_boundaries_TODO_name_this_better, water_groups_by_location_t &water_groups_by_surface_tile):new_group_id(new_group_id),new_group(new_group),groupable_water_dimensional_boundaries_TODO_name_this_better(groupable_water_dimensional_boundaries_TODO_name_this_better),water_groups_by_surface_tile(water_groups_by_surface_tile) {
      try_collect_loc(start_loc);
    }
    
    water_group_identifier new_group_id;
    persistent_water_group_info &new_group;
    groupable_water_dimensional_boundaries_TODO_name_this_better_t &groupable_water_dimensional_boundaries_TODO_name_this_better;
    water_groups_by_location_t &water_groups_by_surface_tile;
    std::vector<tile_location> frontier;
    void try_collect_loc(tile_location const& loc) {
      if (is_ungrouped_surface_tile(loc, water_groups_by_surface_tile)) {
        frontier.push_back(loc);
        assert(new_group.surface_tiles.insert(loc).second);
        assert(water_groups_by_surface_tile.insert(make_pair(loc, new_group_id)).second);
        if (loc.get_neighbor<zplus>(CONTENTS_ONLY).stuff_at().contents() != GROUPABLE_WATER) new_group.suckable_tiles_by_height.insert(loc);
    
        // This DOES handle first-initialization properly; it makes some unnecessary checks, but
        // we're not worried about super speed here.
        groupable_water_dimensional_boundaries_TODO_name_this_better.handle_tile_insertion(loc);
      }
    }
  };
  
  group_collecting_info inf(surface_loc, new_group_id, new_group, groupable_water_dimensional_boundaries_TODO_name_this_better, water_groups_by_surface_tile);
  while(!inf.frontier.empty()) {
    const tile_location search_loc = inf.frontier.back();
    inf.frontier.pop_back();
    
    std::array<tile_location, num_cardinal_directions> neighbors = get_all_neighbors(search_loc, CONTENTS_AND_LOCAL_CACHES_ONLY);
    for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
      tile_location const& adj_loc = neighbors[dir];
      inf.try_collect_loc(adj_loc);
      
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER && adj_loc.stuff_at().is_interior()) {
        std::array<tile_location, num_cardinal_directions> adj_neighbors = get_perpendicular_neighbors(adj_loc, dir, CONTENTS_AND_LOCAL_CACHES_ONLY);
        for (cardinal_direction d2 = 0; d2 < num_cardinal_directions; ++d2) {
          if (cardinal_directions_are_perpendicular(dir, d2)) {
            inf.try_collect_loc(adj_neighbors[d2]);
          }
        }
      }
    }
  }
  
  new_group.recompute_num_tiles_by_height_from_surface_tiles(state);
  
  //check_group_surface_tiles_cache_and_layer_size_caches(state, new_group);
}

void world::initialize_tile_contents(tile_location const& loc, tile_contents contents) {
  mutable_stuff_at(loc).set_contents(contents);
}

void world::initialize_tile_local_caches(tile_location const& loc) {
  bool should_be_interior = true;
  std::array<tile_location, num_cardinal_directions> neighbors = get_all_neighbors(loc, CONTENTS_ONLY);
  for (tile_location const& adj_loc : neighbors) {
    if (neighboring_tiles_with_these_contents_are_not_interior(adj_loc.stuff_at().contents(), loc.stuff_at().contents())) {
      // Between us and them is a 'different types' interface, so we're not interior:
      should_be_interior = false;
      break;
    }
  }
  
  // A tile is either interior or exposed to collision:
  if (should_be_interior) {
    mutable_stuff_at(loc).set_interiorness(true);
  }
  else {
    // TODO: figure out whether surface air should be collision detected with
    // (maybe there's something that detects contact with air? Sodium...?)
    if (loc.stuff_at().contents() != AIR)
      things_exposed_to_collision.insert(loc, convert_to_fine_units(tile_bounding_box(loc.coords())));
  }
}

void world::initialize_tile_water_group_caches(tile_location const& loc) {
  if (loc.stuff_at().contents() == GROUPABLE_WATER) {
    state_t& state = get_state(*this);
    initialize_water_group_from_tile_if_necessary(state, loc,
       state.groupable_water_dimensional_boundaries_TODO_name_this_better,
       state.next_water_group_identifier,
       state.persistent_water_groups,
       state.water_groups_by_surface_tile);
  }
}

void persistent_water_group_info::recompute_num_tiles_by_height_from_surface_tiles(state_t const& state) {
  // Here's how we compute the total volume in less than linear time:
  // We take each contiguous row of surface tiles and compute its total length by
  // using a cleverly sorted set, so that we can start with a tile at the beginning
  // of the row and just jump to the tile at the other end.
  //
  // Here, we look up all "low-x ends of x-rows" and jump to the high-x ends
  num_tiles_by_height.clear();
  
  auto const& groupable_water_dimensional_boundaries_TODO_name_this_better = state.groupable_water_dimensional_boundaries_TODO_name_this_better;
  
  for (tile_location const& surface_loc : surface_tiles) {
    // We're only interested in starting at low-x boundaries
    if (surface_loc.get_neighbor<xminus>(CONTENTS_ONLY).stuff_at().contents() != GROUPABLE_WATER) {
      // If we're *also* the high-x boundary, there's only one water tile in this row
      if (surface_loc.get_neighbor<xplus>(CONTENTS_ONLY).stuff_at().contents() != GROUPABLE_WATER) {
        num_tiles_by_height[surface_loc.coords().z] += 1;
      }
      else {
        // Otherwise, we have to jump to the end of the row using the fancy sorted caches.
        auto surface_loc_iter = groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.find(surface_loc);
        assert (surface_loc_iter != groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.end());
        ++surface_loc_iter;
        assert (surface_loc_iter != groupable_water_dimensional_boundaries_TODO_name_this_better.x_boundary_groupable_water_tiles.end());
        tile_location const& end_tile = *surface_loc_iter;
#ifdef ASSERT_EVERYTHING
        assert(end_tile.coords().y == surface_loc.coords().y && end_tile.coords().z == surface_loc.coords().z);
        assert(end_tile.coords().x > surface_loc.coords().x);
        assert(surface_tiles.find(end_tile) != surface_tiles.end());
#endif
        num_tiles_by_height[surface_loc.coords().z] += 1 + end_tile.coords().x - surface_loc.coords().x;
      }
    }
  }
}
  
fine_scalar persistent_water_group_info::get_pressure_at_height(tile_coordinate height)const {
  // The pressure computation here is a kludge. The main difference between it and "the height of the top level minus the height of the current level" is that it doesn't make teeny water towers on top of oceans exert pressure over the whole ocean.
  map<tile_coordinate, fine_scalar>::iterator iter = pressure_caches.lower_bound(height);
  tile_coordinate current_height = 0;
  fine_scalar current_pressure = 0;
  if (iter == pressure_caches.end()) {
    auto foo = num_tiles_by_height.rbegin();
#ifdef ASSERT_EVERYTHING
    assert(foo != num_tiles_by_height.rend());
#endif
    current_height = foo->first;
    if (height > current_height) return 0;
    pressure_caches.insert(make_pair(current_height, 0));
    // TODO make less stupid
    width_of_widest_level_so_far_caches.erase(current_height);
    width_of_widest_level_so_far_caches.insert(make_pair(current_height, foo->second));
  }
  else {
    current_height = iter->first;
    current_pressure = iter->second;
  }
  while (current_height != height) {
    --current_height;
    water_tile_count last_count = width_of_widest_level_so_far_caches.find(current_height+1)->second;
    auto foo = num_tiles_by_height.find(current_height);
    water_tile_count new_count = std::max(last_count, (foo != num_tiles_by_height.end()) ? foo->second : 0);
    width_of_widest_level_so_far_caches.erase(current_height);
    width_of_widest_level_so_far_caches.insert(make_pair(current_height, new_count));
    current_pressure = (current_pressure + pressure_per_depth_in_tile_heights) * std::min(new_count, last_count) / new_count;
    pressure_caches.insert(make_pair(current_height, current_pressure));
  }
  return current_pressure;
}


void suck_out_suckable_water(state_t& state,
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
  
  const tile_location downward_loc = loc.get_neighbor<zminus>(FULL_REALIZATION);
  if (downward_loc.stuff_at().contents() == GROUPABLE_WATER) {
    auto here_af_iter = active_fluids.find(loc);
    auto down_af_iter = active_fluids.find(downward_loc);
    active_fluid_tile_info here_info = ((here_af_iter == active_fluids.end()) ? active_fluid_tile_info() : here_af_iter->second);
    here_info.progress[zminus] -= progress_necessary(zminus); // The usual effect of moving downwards
#ifdef ASSERT_EVERYTHING
    assert(here_info.progress[zminus] >= 0);
#endif
    if (down_af_iter == active_fluids.end()) {
      active_fluids.insert(make_pair(downward_loc, here_info));
    }
    else {
      // If the downward water is active, we *could* combine its values with ours somehow.
      // However, the downward water is groupable, so it should already be pretty slow,
      // and combining them would make it more complicated to understand how sucking works -
      // I feel like it would invite weird corner cases.
      down_af_iter->second = here_info;
    }
    if (here_af_iter != active_fluids.end()) active_fluids.erase(here_af_iter);
  }
  else {
    // If there's no groupable water below us, then we've gotten to the bottom of the barrel
    // and so the system doesn't need to make any extra accomodations. (If all the water is
    // gone, then there isn't enough left to be unrealistic when it moves fast...)
    active_fluids.erase(loc);
  }
  
  // replace_substance handles marking the water no-longer-suckable.
  replace_substance(state, loc, GROUPABLE_WATER, AIR);
}

void push_water_into_pushable_tile(state_t& state,
   tile_location const& loc,
   active_fluids_t &active_fluids)
{
  // We always create ungroupable water at first when it moves due to pressure;
  // creating groupable water runs the risk
  // of creating explosions where more and more water is drawn out.
  // The water will become groupable in its own time.
  // replace_substance automatically makes the tile no-longer-pushable.
  // (Or maybe the tile was already made non-pushable by what called this.)
  //
  // Semi-hack - we trust that this has been called on a reasonable tile,
  // so we just copy the tile's contents into old_substance_type.
  replace_substance(state, loc, loc.stuff_at().contents(), UNGROUPABLE_WATER);
      
  // Activate it.
  // The [] operator creates a default-constructed version.
  // In this case, we don't want the default attributes - water moved by pressure
  // starts with no downward progress or velocity.
  // It'll be given its new velocity in the next frame, by the normal pressure-pushes-water rules.
  active_fluids[loc].progress[zminus] = 0;
  active_fluids[loc].velocity.z = 0;
}


bool persistent_water_group_info::mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(state_t& state, tile_location const& loc, active_fluids_t &active_fluids) {
  if (!pushable_tiles_by_height.any_below(loc.coords().z)) {
    suckable_tiles_by_height.insert(loc);
    return false;
  }
  const tile_location pushed_tile = pushable_tiles_by_height.get_and_erase_random_from_the_bottom();
  suck_out_suckable_water(state, loc, active_fluids);
  push_water_into_pushable_tile(state, pushed_tile, active_fluids);
  return true;
}
bool persistent_water_group_info::mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(state_t& state, tile_location const& loc, active_fluids_t &active_fluids) {
  if (!suckable_tiles_by_height.any_above(loc.coords().z)) {
    pushable_tiles_by_height.insert(loc);
    return false;
  }
  const tile_location sucked_tile = suckable_tiles_by_height.get_and_erase_random_from_the_top();
  suck_out_suckable_water(state, sucked_tile, active_fluids);
  push_water_into_pushable_tile(state, loc, active_fluids);
  return true;
}
void correct_all_suckable_pushable_pairs(state_t& state, persistent_water_group_info &g, active_fluids_t &active_fluids) {
  while (!g.suckable_tiles_by_height.as_map().empty() && !g.pushable_tiles_by_height.as_map().empty() && g.suckable_tiles_by_height.as_map().rbegin()->first > g.pushable_tiles_by_height.as_map().begin()->first) {
    const tile_location sucked_tile = g.suckable_tiles_by_height.get_and_erase_random_from_the_top();
    const tile_location pushed_tile = g.pushable_tiles_by_height.get_and_erase_random_from_the_bottom();
    suck_out_suckable_water      (state, sucked_tile, active_fluids);
    push_water_into_pushable_tile(state, pushed_tile, active_fluids);
  }
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
  for (auto const& s : smaller_group.pushable_tiles_by_height.as_map()) {
    for (auto const& t : s.second.as_unordered_set()) {
#ifdef ASSERT_EVERYTHING
      assert(t.stuff_at().contents() == AIR);
#endif
      larger_group.pushable_tiles_by_height.insert(t);
    }
  }
  for (auto const& p : smaller_group.num_tiles_by_height) {
    // Note: the [] operator default-constructs a zero if there's nothing there
#ifdef ASSERT_EVERYTHING
    assert(p.second > 0);
#endif
    larger_group.num_tiles_by_height[p.first] += p.second;
  }
  for (auto const& l : smaller_group.surface_tiles) {
    assert(larger_group.surface_tiles.insert(l).second);
    auto foo = water_groups_by_surface_tile.find(l);
#ifdef ASSERT_EVERYTHING
    assert(foo != water_groups_by_surface_tile.end());
    assert(foo->second == destroyed_group_id);
#endif
    foo->second = remaining_group_id;
  }
  
  larger_group.pressure_caches.erase(larger_group.pressure_caches.begin(), larger_group.pressure_caches.upper_bound(smaller_group.num_tiles_by_height.rbegin()->first));
  
  persistent_water_groups.erase(destroyed_group_id);
  
  return remaining_group_id;
}



void replace_substance_impl(
   state_t& state,
   tile_location const& loc,
   tile_contents old_substance_type,
   tile_contents new_substance_type,
   
   world_collision_detector &things_exposed_to_collision,
   groupable_water_dimensional_boundaries_TODO_name_this_better_t &groupable_water_dimensional_boundaries_TODO_name_this_better,
   active_fluids_t &active_fluids,
   water_group_identifier &next_water_group_identifier,
   persistent_water_groups_t &persistent_water_groups,
   water_groups_by_location_t &water_groups_by_surface_tile);

void world::replace_substance(
   tile_location const& loc,
   tile_contents old_substance_type,
   tile_contents new_substance_type) {
  
  state_t& state = get_state(*this);
  replace_substance_impl(state, loc, old_substance_type, new_substance_type,
                         this->things_exposed_to_collision,
                         state.groupable_water_dimensional_boundaries_TODO_name_this_better,
                         state.active_fluids,
                         state.next_water_group_identifier,
                         state.persistent_water_groups,
                         state.water_groups_by_surface_tile);
}

void replace_substance_impl(
   state_t& state,
   tile_location const& loc,
   tile_contents old_substance_type,
   tile_contents new_substance_type,
   
   world_collision_detector &things_exposed_to_collision,
   groupable_water_dimensional_boundaries_TODO_name_this_better_t &groupable_water_dimensional_boundaries_TODO_name_this_better,
   active_fluids_t &active_fluids,
   water_group_identifier &next_water_group_identifier,
   persistent_water_groups_t &persistent_water_groups,
   water_groups_by_location_t &water_groups_by_surface_tile)
{
#ifdef ASSERT_EVERYTHING
  assert(loc.stuff_at().contents() == old_substance_type);
#endif
  //check_pushable_tiles_sanity(state);
  
  std::array<tile_location, num_cardinal_directions> neighbors = get_all_neighbors(loc);
  
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
    for (tile_location const& adj_loc : neighbors) {
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        water_group_identifier group_id = get_water_group_id_by_grouped_tile(state, adj_loc);
        persistent_water_group_info const& group = persistent_water_groups.find(group_id)->second;
        if (group.suckable_tiles_by_height.any_above(loc.coords().z)) {
          adj_tiles_that_want_to_fill_us_via_pressure.push_back(adj_loc);
        }
      }
    }
    
    if (!adj_tiles_that_want_to_fill_us_via_pressure.empty()) {
      tile_location const& tile_pulled_from = adj_tiles_that_want_to_fill_us_via_pressure[rand()%(adj_tiles_that_want_to_fill_us_via_pressure.size())];
      water_group_identifier group_id = get_water_group_id_by_grouped_tile(state, tile_pulled_from);
      persistent_water_group_info &group = persistent_water_groups.find(group_id)->second;
      
      assert(group.mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(state, loc, active_fluids));
      // That function will have run the body of replace_substance on this tile; we don't need to.
      return;
    }
    else {
      for (tile_location const& adj_loc : neighbors) {
        if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
          water_group_identifier group_id = get_water_group_id_by_grouped_tile(state, adj_loc);
          persistent_water_group_info &group = persistent_water_groups.find(group_id)->second;
          assert(!group.mark_tile_as_pushable_and_return_true_if_it_is_immediately_pushed_into(state, loc, active_fluids));
        }
      }
    }
  }
  // Or, if we've become non-air, pressure can no longer push water into us.
  if (old_substance_type == AIR && new_substance_type != AIR) {
    for (tile_location const& adj_loc : neighbors) {
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        water_group_identifier group_id = get_water_group_id_by_grouped_tile(state, adj_loc);
#ifdef ASSERT_EVERYTHING
        assert(group_id != NO_WATER_GROUP);
#endif
        auto foo = persistent_water_groups.find(group_id);
#ifdef ASSERT_EVERYTHING
        assert(foo != persistent_water_groups.end());
#endif
        persistent_water_group_info &group = foo->second;
        group.pushable_tiles_by_height.erase(loc);
      }
    }
    
    //check_pushable_tiles_sanity(state);
  }
  
  // If we're removing water, we have complicated computations to do. We do it in this order:
  // 1) Gather as much information as possible BEFORE physically changing anything (while we're still marked as water)
  // 2) Physically change us
  // 3) Update the relatively-isolated cached info like interiorness
  // 4) Update the relatively-interconnected cached info of our group
  
  persistent_water_group_info *water_group = nullptr;
  water_group_identifier water_group_id = NO_WATER_GROUP;
  if (old_substance_type == GROUPABLE_WATER && new_substance_type != GROUPABLE_WATER) {
    // ==============================================================================
    // 1) Gather as much information as possible without physically changing anything
    //    (while we're still marked as water)
    // ==============================================================================
    water_group_id = get_water_group_id_by_grouped_tile(state, loc);
#ifdef ASSERT_EVERYTHING
    assert(water_group_id != NO_WATER_GROUP);
#endif
    auto iter = persistent_water_groups.find(water_group_id);
#ifdef ASSERT_EVERYTHING
    assert(iter != persistent_water_groups.end());
#endif
    water_group = &iter->second;
    //check_group_surface_tiles_cache_and_layer_size_caches(state, *water_group);
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
    for (tile_location const& adj_loc : neighbors) {
      water_groups_by_location_t::const_iterator iter = water_groups_by_surface_tile.find(adj_loc);
      if (iter != water_groups_by_surface_tile.end()) {
        if (water_group_id == NO_WATER_GROUP) {
          water_group_id = iter->second;
        }
        else if (iter->second != water_group_id) {
          // Two groups we join! Merge them. merge_water_groups() automatically
          // handles the "merge the smaller one into the larger one" optimization
          // and returns the ID of the group that remains.
          water_group_id = merge_water_groups(water_group_id, iter->second, persistent_water_groups, water_groups_by_surface_tile);
        }
      }
    }
    
    // Now we're either adjacent to one water group or none. If none, we have to create a new one for ourself.
    if (water_group_id == NO_WATER_GROUP) {
      water_group_id = make_new_water_group(next_water_group_identifier, persistent_water_groups);
    }
    
    water_group = &persistent_water_groups.find(water_group_id)->second;
  }
  
  // ==============================================================================
  // 2) Physically change us
  // ==============================================================================
  mutable_stuff_at(loc).set_contents(new_substance_type);
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
    if (is_fluid(new_substance_type)) active_fluids[loc];
    if (is_fluid(neighbors[zplus].stuff_at().contents())) active_fluids[neighbors[zplus]];
    
    for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
      if (cardinal_directions_are_perpendicular(dir, zplus)) {
        tile_location const& adj_loc = neighbors[dir];
        const tile_location diag_loc = adj_loc.get_neighbor<zplus>(FULL_REALIZATION);
        // We could be more conservative about what we activate, by checking the actual details of
        // what changed and how that affects things, but that would just invite missing cases,
        // and it wouldn't speed stuff up noticeably because the thing will just deactivate again
        // on the following frame if it's wrongly activated.
        if (is_fluid(adj_loc.stuff_at().contents())) active_fluids[adj_loc];
        if (is_fluid(diag_loc.stuff_at().contents())) active_fluids[diag_loc];
      }
    }
  }
  if (!these_tile_contents_are_identical_regarding_interiorness(new_substance_type, old_substance_type)) {
    bool we_are_now_interior = true;
    for (tile_location const& adj_loc : neighbors) {
      if (neighboring_tiles_with_these_contents_are_not_interior(adj_loc.stuff_at().contents(), new_substance_type)) {
        // Between us and them is a 'different types' interface, so neither us nor them is interior:
        we_are_now_interior = false;
        if (adj_loc.stuff_at().is_interior()) {
          mutable_stuff_at(adj_loc).set_interiorness(false);
          // No longer interior! Unless you're air, add to the collision detection struct.
          // TODO: figure out whether surface air should be collision detected with
          // (maybe there's something that detects contact with air? Sodium...?)
#ifdef ASSERT_EVERYTHING
          assert(!things_exposed_to_collision.exists(adj_loc));
#endif
          if (adj_loc.stuff_at().contents() != AIR) {
            things_exposed_to_collision.insert(adj_loc, convert_to_fine_units(tile_bounding_box(adj_loc.coords())));
          }
        }
      }
      else {
        // If we've become the same as the other tile, we may have made that tile interior
        if (!adj_loc.stuff_at().is_interior()) {
          bool they_should_be_interior = true;
          std::array<tile_location, num_cardinal_directions> adj_neighbors = get_all_neighbors(adj_loc);
          for (tile_location const& adj_adj_loc : adj_neighbors) {
            if (neighboring_tiles_with_these_contents_are_not_interior(adj_adj_loc.stuff_at().contents(), adj_loc.stuff_at().contents())) {
              they_should_be_interior = false;
              break;
            }
          }
          if (they_should_be_interior) {
            mutable_stuff_at(adj_loc).set_interiorness(true);
            // Now interior: remove us from the collision detection.
            // TODO: figure out whether surface air should be collision detected with
            // (maybe there's something that detects contact with air? Sodium...?)
            if (adj_loc.stuff_at().contents() != AIR) {
#ifdef ASSERT_EVERYTHING
              assert(things_exposed_to_collision.exists(adj_loc));
#endif
              things_exposed_to_collision.erase(adj_loc);
            }
          }
        }
      }
    }
    bool should_be_collidable = (!we_are_now_interior && !new_substance_type == AIR);
    bool should_have_been_collidable = (!loc.stuff_at().is_interior() && !old_substance_type == AIR);
    mutable_stuff_at(loc).set_interiorness(we_are_now_interior);
    if (should_be_collidable && !should_have_been_collidable) {
#ifdef ASSERT_EVERYTHING
      assert(!things_exposed_to_collision.exists(loc));
#endif
      things_exposed_to_collision.insert(loc, convert_to_fine_units(tile_bounding_box(loc.coords())));
    }
    if (!should_be_collidable && should_have_been_collidable) {
#ifdef ASSERT_EVERYTHING
      assert(things_exposed_to_collision.exists(loc));
#endif
      things_exposed_to_collision.erase(loc);
    }
  }
  
  // ==============================================================================
  // 4) Update the relatively-interconnected cached info of water groups
  // ==============================================================================
  
  // For creating water, there are only a few more caches to update.
  if (new_substance_type == GROUPABLE_WATER && old_substance_type != GROUPABLE_WATER) {
    // The tile below us, if any, is no longer suckable.
    if (neighbors[zminus].stuff_at().contents() == GROUPABLE_WATER) water_group->suckable_tiles_by_height.erase(neighbors[zminus]);
    
    // We might need to designate ourself as a surface tile,
    // and we might need to designate neighbors as non-surface tiles,
    // if we've closed their last liberty (to use Go terms)
    bool there_are_any_adjacent_tiles_that_are_not_groupable_water = false;
    for (tile_location const& adj_loc : neighbors) {
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        bool adjacent_tile_has_any_adjacent_tiles_that_are_not_groupable_water = false;
        
        std::array<tile_location, num_cardinal_directions> adj_neighbors = get_all_neighbors(adj_loc);
        for (tile_location const& adj_adj_loc : adj_neighbors) {
          if (adj_adj_loc.stuff_at().contents() != GROUPABLE_WATER) {
            adjacent_tile_has_any_adjacent_tiles_that_are_not_groupable_water = true;
            break;
          }
        }
        if (!adjacent_tile_has_any_adjacent_tiles_that_are_not_groupable_water) {
          if (water_group->surface_tiles.erase(adj_loc))
            assert(water_groups_by_surface_tile.erase(adj_loc));
        }
      }
      else {
        there_are_any_adjacent_tiles_that_are_not_groupable_water = true;
      }
    }
    if (there_are_any_adjacent_tiles_that_are_not_groupable_water) {
      if (water_group->surface_tiles.insert(loc).second)
        assert(water_groups_by_surface_tile.insert(make_pair(loc, water_group_id)).second);
    }
    
    ++water_group->num_tiles_by_height[loc.coords().z];
    
    // Adding a tile at a specific height can change pressure at that height, but not anywhere above
    water_group->pressure_caches.erase(water_group->pressure_caches.begin(), water_group->pressure_caches.upper_bound(loc.coords().z));
    //check_group_surface_tiles_cache_and_layer_size_caches(state, *water_group);
    
    // Our surfaces are now pushable surfaces.
    for (tile_location const& adj_loc : neighbors) {
      if (adj_loc.stuff_at().contents() == AIR) {
        water_group->pushable_tiles_by_height.insert(adj_loc);
      }
    }
    //check_pushable_tiles_sanity(state);
    
    // If the tile above us was idle groupable water, it's now suckable...
    // TODO come up with a way to handle pushable and suckable tiles that isn't so likely to have missed cases.
    tile_location const& uploc = neighbors[zplus];
    if (uploc.stuff_at().contents() == GROUPABLE_WATER && (uploc.get_neighbor<zplus>(CONTENTS_ONLY)).stuff_at().contents() != GROUPABLE_WATER && active_fluids.find(uploc) == active_fluids.end()) {
      water_group->mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(state, uploc, active_fluids);
    }
    correct_all_suckable_pushable_pairs(state, *water_group, active_fluids);
  }
  
  if (old_substance_type == GROUPABLE_WATER && new_substance_type != GROUPABLE_WATER) {
    // If we were a surface tile of the group, we are no longer.
    if (water_group->surface_tiles.erase(loc))
      assert(water_groups_by_surface_tile.erase(loc));
    
    // We may have exposed other group tiles, which now need to be marked as the group surface.
    for (tile_location const& adj_loc : neighbors) {
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        water_groups_by_location_t::const_iterator existing_marker_iter = water_groups_by_surface_tile.find(adj_loc);
        if (existing_marker_iter != water_groups_by_surface_tile.end()) {
#ifdef ASSERT_EVERYTHING
          assert(existing_marker_iter->second == water_group_id);
#endif
        }
        else {
          if (water_group->surface_tiles.insert(adj_loc).second)
            assert(water_groups_by_surface_tile.insert(make_pair(adj_loc, water_group_id)).second);
        }
      }
    }
    
    water_group->suckable_tiles_by_height.erase(loc);
    
    auto iter = water_group->num_tiles_by_height.find(loc.coords().z);
#ifdef ASSERT_EVERYTHING
    assert(iter != water_group->num_tiles_by_height.end());
    assert(iter->second > 0);
#endif
    --iter->second;
    if (iter->second == 0) water_group->num_tiles_by_height.erase(iter);
    
    if (water_group->num_tiles_by_height.empty()) {
      persistent_water_groups.erase(water_group_id);
      // TODO figure out a more elegant way to avoid the following computations than a return
      return;
    }
    
    // Removing a tile from a specific height can change pressure at that height, but not anywhere above
    water_group->pressure_caches.erase(water_group->pressure_caches.begin(), water_group->pressure_caches.upper_bound(loc.coords().z));
    
    // Our surfaces are no longer pushable surfaces.
    for (tile_location const& adj_loc : neighbors) {
      if (adj_loc.stuff_at().contents() == AIR) {
        bool can_be_pushable = false;
        std::array<tile_location, num_cardinal_directions> neighbor_neighbors = get_all_neighbors(adj_loc);
        for (tile_location const& adj_adj_loc : neighbor_neighbors) {
          if (adj_adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
            water_groups_by_location_t::const_iterator foo = water_groups_by_surface_tile.find(adj_adj_loc);
            if (foo != water_groups_by_surface_tile.end() && foo->second == water_group_id) {
              can_be_pushable = true;
              break;
            }
          }
        }
        if (!can_be_pushable) water_group->pushable_tiles_by_height.erase(adj_loc);
      }
    }
    //check_pushable_tiles_sanity(state);
    
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
      unordered_set<tile_location> defunct_frontiers;
      
      // Try to collect a location into one of the fills.
      // If no one has claimed it, claim it and add it to your frontier.
      // If you've claimed it, ignore it and move on.
      // If one of the other collectors has claimed it, then we now know that those two are connected,
      //     so merge the two of them.
      // returns true if-and-only-if the operation should destroy the collector by merging it into another collector.
      bool try_collect_loc(tile_location const& collector, tile_location const& collectee) {
        if (collectee.stuff_at().contents() == GROUPABLE_WATER && !collectee.stuff_at().is_interior()) {
          unordered_map<tile_location, tile_location>::iterator iter = neighbors_by_collected_loc.find(collectee);
          
          if (iter == neighbors_by_collected_loc.end()) {
            disconnected_frontiers[collector].push(collectee);
            collected_locs_by_neighbor[collector].insert(collectee);
            neighbors_by_collected_loc.insert(make_pair(collectee, collector));
          }
          else {
            if (iter->second != collector) {
              // Found an overlap - merge the two!
              // Hack: If there are only two collectors left, then we don't need to do any more processing - just get destroyed, and bail.
              if (disconnected_frontiers.size() == 2) return true;
              
              // The collectors will normally be the same size, but if two merged already, then one might be bigger. And we want to merge the smaller one into the bigger one for performance reasons.
              bool this_collector_is_bigger = (collected_locs_by_neighbor[collector].size() > collected_locs_by_neighbor[iter->second].size());
              const tile_location  bigger_collector = (this_collector_is_bigger ? collector    : iter->second);
              const tile_location smaller_collector = (this_collector_is_bigger ? iter->second : collector   );
              
              for (tile_location const& loc : collected_locs_by_neighbor[smaller_collector]) {
                collected_locs_by_neighbor[bigger_collector].insert(loc);
                neighbors_by_collected_loc.find(loc)->second = bigger_collector;
              }
              while (!disconnected_frontiers[smaller_collector].empty()) {
                disconnected_frontiers[bigger_collector].push(disconnected_frontiers[smaller_collector].front());
                disconnected_frontiers[smaller_collector].pop();
              }
              defunct_frontiers.insert(smaller_collector);
              
              return (!this_collector_is_bigger);
            }
          }
        }
        return false;
      }
    };
    water_splitting_info inf;
    
    for (tile_location const& adj_loc : neighbors) {
      if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
        inf.try_collect_loc(adj_loc, adj_loc);
      }
    }
    while(inf.disconnected_frontiers.size() > 1) {
      for (auto p = inf.disconnected_frontiers.begin(); p != inf.disconnected_frontiers.end(); ) {
        tile_location const& which_neighbor = p->first;
        std::queue<tile_location> &frontier = p->second;
        bool destroy_this_frontier = (inf.defunct_frontiers.find(which_neighbor) != inf.defunct_frontiers.end());
        
        if (!destroy_this_frontier && frontier.empty()) {
          //check_pushable_tiles_sanity(state);
          // If the frontier is empty, that means this route has finished its search and not found any
          // connections to the rest of the water. Therefore, we need to mark this frontier off as a
          // separate group.
          destroy_this_frontier = true;
          inf.defunct_frontiers.insert(which_neighbor);
          
          const water_group_identifier new_group_id = make_new_water_group(next_water_group_identifier, persistent_water_groups);
          persistent_water_group_info &new_group = persistent_water_groups.find(new_group_id)->second;
          
          for (tile_location const& new_grouped_loc : inf.collected_locs_by_neighbor[which_neighbor]) {
            auto foo = water_groups_by_surface_tile.find(new_grouped_loc);
#ifdef ASSERT_EVERYTHING
            assert(foo != water_groups_by_surface_tile.end());
            assert(foo->second == water_group_id);
#endif
            foo->second = new_group_id;
            assert(new_group.surface_tiles.insert(new_grouped_loc).second);
            assert(water_group->surface_tiles.erase(new_grouped_loc));
            if (water_group->suckable_tiles_by_height.erase(new_grouped_loc)) {
              new_group.suckable_tiles_by_height.insert(new_grouped_loc);
            }
          }
          new_group.recompute_num_tiles_by_height_from_surface_tiles(state);
          for (auto const& p : new_group.num_tiles_by_height) {
            auto iter = water_group->num_tiles_by_height.find(p.first);
#ifdef ASSERT_EVERYTHING
            assert(iter != water_group->num_tiles_by_height.end());
            assert(iter->second > 0);
            assert(iter->second >= p.second);
#endif
            iter->second -= p.second;
            if (iter->second == 0) water_group->num_tiles_by_height.erase(iter);
          }
          water_group->pressure_caches.erase(water_group->pressure_caches.begin(), water_group->pressure_caches.upper_bound(new_group.num_tiles_by_height.rbegin()->first));
          
          unordered_set<tile_location> original_pushable_tiles_removed;
          for (auto const& ph : water_group->pushable_tiles_by_height.as_map()) {
            for(auto const& p : ph.second.as_unordered_set()) {
#ifdef ASSERT_EVERYTHING
              assert(p.stuff_at().contents() == AIR);
#endif
              bool is_pushable_for_original_group = false;
              std::array<tile_location, num_cardinal_directions> p_neighbors = get_all_neighbors(p);
              for (tile_location const& adj_loc : p_neighbors) {
                if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
                  const water_group_identifier pushable_having_group_id = get_water_group_id_by_grouped_tile(state, adj_loc);
                  if (pushable_having_group_id == water_group_id) {
                    is_pushable_for_original_group = true;
                  }
                  else {
                    auto foo = persistent_water_groups.find(pushable_having_group_id);
#ifdef ASSERT_EVERYTHING
                    assert(foo != persistent_water_groups.end());
#endif
                    foo->second.pushable_tiles_by_height.insert(p);
                  }
                }
              }
              if (!is_pushable_for_original_group) original_pushable_tiles_removed.insert(p);
            }
          }
          for (auto const& p : original_pushable_tiles_removed) {
            water_group->pushable_tiles_by_height.erase(p);
          }
          //check_group_surface_tiles_cache_and_layer_size_caches(state, new_group);
          //check_pushable_tiles_sanity(state);
        }
        else if (!destroy_this_frontier) {
          tile_location search_loc = frontier.front();
          
#ifdef ASSERT_EVERYTHING
          {
            water_groups_by_location_t::const_iterator iter = water_groups_by_surface_tile.find(search_loc);
            assert(iter != water_groups_by_surface_tile.end());
            assert(iter->second == water_group_id);
          }
#endif

          std::array<tile_location, num_cardinal_directions> search_neighbors = get_all_neighbors(search_loc);
          for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
            tile_location const& adj_loc = search_neighbors[dir];
            
            destroy_this_frontier = inf.try_collect_loc(which_neighbor, adj_loc);
            if (destroy_this_frontier) goto fake_continue;
            
            if (adj_loc.stuff_at().contents() == GROUPABLE_WATER && adj_loc.stuff_at().is_interior()) {
              std::array<tile_location, num_cardinal_directions> adj_neighbors = get_perpendicular_neighbors(adj_loc, dir, FULL_REALIZATION);
              for (cardinal_direction d2 = 0; d2 < num_cardinal_directions; ++d2) {
                if (cardinal_directions_are_perpendicular(dir, d2)) {
                  destroy_this_frontier = inf.try_collect_loc(which_neighbor, adj_neighbors[d2]);
                  if (destroy_this_frontier) goto fake_continue;
                }
              }
            }
          }
          frontier.pop();
        }
        
        fake_continue:
        
        if (destroy_this_frontier) {
          // Hack - if we're merging the last two frontiers, just bail.
          if (inf.disconnected_frontiers.size() <= 2) goto doublebreak;
#ifdef ASSERT_EVERYTHING
          assert(inf.defunct_frontiers.find(which_neighbor) != inf.defunct_frontiers.end());
#endif
          inf.disconnected_frontiers.erase(p++);
        }
        else {
#ifdef ASSERT_EVERYTHING
          assert(inf.defunct_frontiers.find(which_neighbor) == inf.defunct_frontiers.end());
#endif
          ++p;
        }
      }
    }
    doublebreak: ;
  
    // ==============================================================================
    //  End of flood-fill-based group-splitting algorithm
    // ==============================================================================
    
    //check_group_surface_tiles_cache_and_layer_size_caches(state, *water_group);
  }
  //check_pushable_tiles_sanity(state);
}


struct wanted_move {
  tile_location src;
  cardinal_direction dir;
  sub_tile_distance amount_of_the_push_that_sent_us_over_the_threshold;
  sub_tile_distance excess_progress;
  wanted_move(tile_location src,cardinal_direction dir,sub_tile_distance a,sub_tile_distance e):src(src),dir(dir),amount_of_the_push_that_sent_us_over_the_threshold(a),excess_progress(e){}
};

// TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
int obstructiveness(tile_contents tc) {
  if (tc == ROCK) return 4;
  else if (tc == RUBBLE) return 3;
  else if (is_water(tc)) return 2;
  else if (tc == AIR) return 1;
  else assert(false); // reaching this would mean we implemented a new material type but forgot to set its obstructiveness
}

void update_fluids_impl(state_t& state, active_fluids_t &active_fluids, persistent_water_groups_t &persistent_water_groups);

void world::update_fluids() {
  state_t& state = get_state(*this);
  update_fluids_impl(state, state.active_fluids, state.persistent_water_groups);
}

void update_fluids_impl(state_t& state, active_fluids_t &active_fluids, persistent_water_groups_t &persistent_water_groups) {
  // ==============================================================================
  //  Phase 1
  //  Compute all the velocities and movement.
  //  There are a lot of relatively-separable rules governing how water tiles move.
  // ==============================================================================
  
  vector<wanted_move> wanted_moves;
  
  for (pair<const tile_location, active_fluid_tile_info> &p : active_fluids) {
    tile_location const& loc = p.first;
    active_fluid_tile_info &fluid = p.second;
    tile const& t = loc.stuff_at();
#ifdef ASSERT_EVERYTHING
    assert(is_fluid(t.contents()));
#endif
    std::array<tile_location, num_cardinal_directions> neighbors = get_all_neighbors(loc);
    
    value_for_each_cardinal_direction<sub_tile_distance> new_progress(0);
    
    // Gravity applies to everyone
    fluid.velocity += gravity_acceleration;
      
    // Slight air resistance proportional to the square of the velocity (mostly there to make a natural cap velocity for falling water)
    fluid.velocity -= (fluid.velocity * fluid.velocity.magnitude_within_32_bits()) / air_resistance_constant;
      
    // Relatively large friction against the ground
    for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
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
    
    // Groupable water pushes fluids away from it according to pressure.
    if (t.contents() != GROUPABLE_WATER) {
      for (int dir = 0; dir < num_cardinal_directions; ++dir) {
        tile_location const& adj_loc = neighbors[dir];
        if (adj_loc.stuff_at().contents() == GROUPABLE_WATER) {
          tile_location const& opposite_loc = neighbors[opposite_cardinal_direction(dir)];
          tile_location walking_loc = opposite_loc;
          bool pinned_between_obstacles;
          while(true) {
            tile_contents contents_there = walking_loc.stuff_at().contents();
            if (contents_there == GROUPABLE_WATER || obstructiveness(contents_there) > obstructiveness(GROUPABLE_WATER)) {
              pinned_between_obstacles = true;
              break;
            }
            if (contents_there == AIR) {
              pinned_between_obstacles = false;
              break;
            }
            walking_loc = walking_loc.get_neighbor_by_variable(opposite_cardinal_direction(dir), CONTENTS_ONLY);
          }
          if (pinned_between_obstacles) {
            // Hack? - if we're pinned between obstructions, just become groupable immediately
            fluid.velocity = inactive_fluid_velocity;
            // Hack? - if we have obstructions on both sides, just stop... unless it's vertical, in which case stop to the inactive level:
            //fluid.velocity -= project_onto_cardinal_direction(fluid.velocity - inactive_fluid_velocity, dir);
            //fluid.frames_until_can_become_groupable = 0;
          }
          else {
            persistent_water_group_info const& group = get_water_group_by_grouped_tile(state, adj_loc);
            const fine_scalar pressure = group.get_pressure_at_height(opposite_loc.coords().z);
            if (pressure > 0) { // Hack - TODO examine the fact that "0 pressure" forces velocity to be at least 0 away from the tile (incorrect behavior for tiles directly above the group) and if the principles that causes that problem also causes subtler problems.
              const sub_tile_distance amount_of_vel_in_pressure_receiving_dir = fluid.velocity.dot<sub_tile_distance>(-cardinal_direction_vectors[dir]);
              // This velocity pushes us towards opposite_loc, and pressure is a measure of how much water wants to
              // *go to* the height pressure is being measured at, so we measure at opposite_loc.
              const sub_tile_distance deficiency_of_vel = i64sqrt(2*pressure) - amount_of_vel_in_pressure_receiving_dir;
              if (deficiency_of_vel > 0) {
                fluid.velocity += vector3<sub_tile_distance>(-cardinal_direction_vectors[dir]) * deficiency_of_vel;
              }
            }
          }
        }
      }
    }
    
    // Velocity causes progress.
    for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
      const sub_tile_distance dp = fluid.velocity.dot<sub_tile_distance>(cardinal_direction_vectors[dir]);
      if (dp > 0) new_progress[dir] += dp;
    }
    
    // Water that's blocked, but can go around in a diagonal direction, also makes "progress" towards all those possible directions (so it'll go in a random direction if it could go around in several different diagonals, without having to 'choose' one right away and gain velocity only in that direction). The main purpose of this is to make it so that water doesn't stack nicely in pillars, or sit still on steep slopes.
    // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
    for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
      tile_location const& dst_loc = neighbors[dir];
      if (dst_loc.stuff_at().contents() == AIR) {
        std::array<tile_location, num_cardinal_directions> neighbor_neighbors = get_perpendicular_neighbors(dst_loc, dir, CONTENTS_ONLY);
        for (cardinal_direction d2 = 0; d2 < num_cardinal_directions; ++d2) {
          if (cardinal_directions_are_perpendicular(dir, d2)) {
            tile_location const& diag_loc = neighbor_neighbors[d2];
            if (fluid.blockage_amount_this_frame[d2] > 0 && obstructiveness(diag_loc.stuff_at().contents()) < obstructiveness(neighbors[d2].stuff_at().contents())) {
              new_progress[dir] += fluid.blockage_amount_this_frame[d2];
            }
          }
        }
      }
    }
    
    // blockage_amount_this_frame is just a temporary value that lingers from one frame to the next -
    // we erase it after we use it.
    for (sub_tile_distance &bb : fluid.blockage_amount_this_frame) bb = 0;
    
    // ==============================================================================
    //  Phase 2
    //  We've computed our velocity and progress for this frame. Now we check
    //  whether we've made enough progress to move, and if we have, we record
    //  that as a "wanted move".
    // ==============================================================================
    
    for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
      sub_tile_distance &progress_ref = fluid.progress[dir];
      if (new_progress[dir] == 0) {
        if (progress_ref < idle_progress_reduction_rate) progress_ref = 0;
        else progress_ref -= idle_progress_reduction_rate;
      }
      else {
#ifdef ASSERT_EVERYTHING
        assert(new_progress[dir] >= 0);
        assert(progress_ref >= 0);
        assert(progress_ref <= progress_necessary(dir));
#endif
        progress_ref += new_progress[dir];
        if (progress_ref > progress_necessary(dir)) {
          wanted_moves.push_back(wanted_move(loc, dir, new_progress[dir], progress_ref - progress_necessary(dir)));
          progress_ref = progress_necessary(dir);
        }
      }
    }
  }
  
  // ==============================================================================
  //  Phase 2
  //  We shuffle all the wanted moves and
  //  execute them in a random order, as opposed to trying to find some way to
  //  composite them when they conflict with each other.
  // ==============================================================================
  
  
  std::random_shuffle(wanted_moves.begin(), wanted_moves.end());
  std::unordered_set<tile_location> disturbed_tiles;
  
  
  for (const wanted_move move : wanted_moves) {
    const tile_location dst = move.src.get_neighbor_by_variable(move.dir, FULL_REALIZATION);
    tile const& src_tile = move.src.stuff_at();
    tile const& dst_tile = dst.stuff_at();
    // Avoid making stupid situations where we do things like trying to move water more than once
    // (which would actually be moving DIFFERENT water, quite incorrectly)
    if (disturbed_tiles.find(move.src) != disturbed_tiles.end()) continue;
    disturbed_tiles.insert(move.src);
    // It's also possible that our moves caused other water to be sucked away due to the
    // group-pressure rules.
    if (!is_fluid(src_tile.contents())) {
      continue;
    }
    
    active_fluid_tile_info &src_fluid = active_fluids.find(move.src)->second;
    // Hack - we also might have been overwritten by the sucking rules, in which case we shouldn't move.
    // TODO - find a better way to invalidate sucked tiles.
    if (src_fluid.progress[move.dir] != progress_necessary(move.dir)) {
      continue;
    }
    
    // Rubble is 'heavier' than water and basically ignores it, while water has to yield to rubble.
    int src_weight = obstructiveness(src_tile.contents());
    int dst_weight = obstructiveness(dst_tile.contents());
    if (src_weight > dst_weight) {
      bool dst_was_active_fluid = false;
      active_fluid_tile_info dst_info_store;
      tile_contents src_old_contents = src_tile.contents();
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
      replace_substance(state, dst, dst_old_contents, src_old_contents == GROUPABLE_WATER ? UNGROUPABLE_WATER : src_old_contents);
      replace_substance(state, move.src, src_old_contents, dst_old_contents);
      
      if (dst_was_active_fluid) src_fluid = dst_info_store;
      disturbed_tiles.insert(dst);
      
      // If a tile moves, we're now content to assume that it was moving because it had a realistic velocity in that direction, so we should continue with that assumption.
      const sub_tile_distance amount_of_new_vel_in_movement_dir = dst_fluid.velocity.dot<sub_tile_distance>(cardinal_direction_vectors[move.dir]);
      const sub_tile_distance deficiency_of_new_vel_in_movement_dir = move.amount_of_the_push_that_sent_us_over_the_threshold - amount_of_new_vel_in_movement_dir;
      if (deficiency_of_new_vel_in_movement_dir > 0) {
        dst_fluid.velocity += vector3<sub_tile_distance>(cardinal_direction_vectors[move.dir]) * deficiency_of_new_vel_in_movement_dir;
      }
      
      // Don't lose movement to rounding error during progress over multiple tiles:
      dst_fluid.progress[move.dir] = std::min(move.excess_progress, progress_necessary(move.dir));
    }
      
    else {
      // We're blocked... However, in one case, we don't register as blocked.
      // If the current move is "go down into obstruction while being part of a group of
      // groupable water with free lower exterior surfaces", then 
      // instead of getting blocked, we get sucked to one of the exterior surfaces.
      bool blocked = true;
      const bool becoming_suckable = (src_tile.contents() == GROUPABLE_WATER) && (move.dir == zminus) && (move.src.get_neighbor<zplus>(CONTENTS_ONLY).stuff_at().contents() != GROUPABLE_WATER);
      
      if (becoming_suckable) {
        const water_group_identifier group_id = get_water_group_id_by_grouped_tile(state, move.src);
        persistent_water_group_info &group = persistent_water_groups.find(group_id)->second;
        // Caution - if this returns true, it has removed the water we're currently looking at.
        // (Otherwise, it hasn't.)
        if (group.mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(state, move.src, active_fluids)) {
          blocked = false;
        }
      }
      
      if (blocked) {
        // We're blocked. If we're trying to run into them faster than they think they're moving away from us, stop it.
        // (Unless we're slower than min_convincing_speed, which is a kludge that's important for the "fall off pillars" rule.)
        src_fluid.blockage_amount_this_frame[move.dir] = move.excess_progress;
        
        sub_tile_distance dst_vel_in_movement_dir;
        auto i = active_fluids.find(dst);
        if (i == active_fluids.end()) dst_vel_in_movement_dir = inactive_fluid_velocity.dot<sub_tile_distance>(cardinal_direction_vectors[move.dir]);
        else                          dst_vel_in_movement_dir = i->second.velocity     .dot<sub_tile_distance>(cardinal_direction_vectors[move.dir]);
        const sub_tile_distance acceptable_remaining_vel_for_us = std::max(dst_vel_in_movement_dir - gravity_acceleration_magnitude * 2, min_convincing_speed);
        const sub_tile_distance our_vel_in_movement_dir = src_fluid.velocity.dot<sub_tile_distance>(cardinal_direction_vectors[move.dir]);
        const sub_tile_distance excess_vel = our_vel_in_movement_dir - acceptable_remaining_vel_for_us;
        if (excess_vel > 0) {
          src_fluid.velocity -= vector3<sub_tile_distance>(cardinal_direction_vectors[move.dir]) * excess_vel;
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
        if (!(fluid.velocity - inactive_fluid_velocity).magnitude_within_32_bits_is_greater_than(min_convincing_speed/2)) {
          replace_substance(state, loc, UNGROUPABLE_WATER, GROUPABLE_WATER);
        }
      //}
    }
    if (t.contents() == GROUPABLE_WATER) {
      if ((fluid.velocity - inactive_fluid_velocity).magnitude_within_32_bits_is_greater_than(min_convincing_speed/2)) {
        // HACK: Water with groupable water directly under it doesn't stop being groupable due to high velocity,
        // purely to facilitate the water-being-sucked-out-very-fast situation.
        // TODO: find a cleaner way to facilitate that if possible.
        if (loc.get_neighbor<zminus>(CONTENTS_ONLY).stuff_at().contents() != GROUPABLE_WATER)
          replace_substance(state, loc, GROUPABLE_WATER, UNGROUPABLE_WATER);
      }
    }
  }
  
  for (auto i = active_fluids.begin(); i != active_fluids.end(); ) {
    tile_location const& loc = i->first;
    active_fluid_tile_info const& fluid = i->second;
    tile const& t = loc.stuff_at();
    
    bool can_deactivate = false;
    if (fluid.is_in_inactive_state()) {
      // Be a little paranoid about making sure fluids obeys all the proper conditions of inactivity
      for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
        if (dir == zminus) {
          if (fluid.blockage_amount_this_frame[dir] > min_convincing_speed + (-gravity_acceleration.z) || fluid.blockage_amount_this_frame[dir] <= min_convincing_speed) {
            goto fake_continue;
          }
        }
        else {
          if (fluid.blockage_amount_this_frame[dir] != 0) {
            goto fake_continue;
          }
        }
      }
      
      // NOTE "Adjacent tile conditions for activation/deactivation": The only relevant ones are
      // the one directly below, the ones cardinally-horizontally, and the ones horizontally-and-below.
      // at the 2-diagonals. This comment is duplicated one one other place in this file.
      std::array<tile_location, num_cardinal_directions> cneighbors = get_all_neighbors(loc, CONTENTS_ONLY);
      if (obstructiveness(cneighbors[zminus].stuff_at().contents()) < obstructiveness(t.contents())) goto fake_continue;

      // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
      for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
        if (cardinal_directions_are_perpendicular(dir, zminus)) {
          tile_location const& dst_loc = cneighbors[dir];
          if (dst_loc.stuff_at().contents() == AIR) {
            tile_location const& diag_loc = dst_loc.get_neighbor<zminus>(CONTENTS_ONLY);
            if (obstructiveness(diag_loc.stuff_at().contents()) < obstructiveness(cneighbors[zminus].stuff_at().contents())) {
              goto fake_continue;
            }
          }
        }
      }
      
      can_deactivate = true;
    }
    
    fake_continue:
    if (can_deactivate) {
      const tile_location loc_preserved = loc;
#ifdef ASSERT_EVERYTHING
      assert(t.contents() != UNGROUPABLE_WATER); // we assume that any inactive water must be groupable
#endif
      bool is_groupable_water = t.contents() == GROUPABLE_WATER;
      active_fluids.erase(i++);
      
      // Hack: What if, in a single frame, we become groupable and then deactivate?
      // Then we missed the check for becoming a suckable tile because we were ungroupable,
      // but the following frame, we won't make the check because we'll be inactive!
      // Semi-hack: Cover for that scenario here.
      // TODO come up with a way to handle pushable and suckable tiles that isn't so likely to have missed cases.
      if (is_groupable_water && loc_preserved.get_neighbor<zplus>(CONTENTS_ONLY).stuff_at().contents() != GROUPABLE_WATER) {
        persistent_water_groups
          .find(get_water_group_id_by_grouped_tile(state, loc_preserved))->second
            .mark_tile_as_suckable_and_return_true_if_it_is_immediately_sucked_away(state, loc_preserved, active_fluids);
      }
    }
    else ++i;
  }
}

// Constructing one of these in the default way yields the natural inactive state:
active_fluid_tile_info::active_fluid_tile_info():
   velocity(inactive_fluid_velocity),
   progress(0),
   blockage_amount_this_frame(0)
{
  progress[zminus] = progress_necessary(zminus);
}
  
bool active_fluid_tile_info::is_in_inactive_state()const {
  // TODO: does it make sense that we're ignoring the 1-frame-duration variable "blockage_amount_this_frame"?
  for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
    if (dir == zminus) {
      if (progress[dir] != progress_necessary(dir)) return false;
    }
    else {
      if (progress[dir] != 0) return false;
    }
  }
  return velocity == inactive_fluid_velocity;
}


