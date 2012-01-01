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

tile& mutable_stuff_at(tile_location const& loc) { return loc.wb->get_tile(loc.v); }

void world::set_stickyness(tile_location const& loc, bool new_stickyness) {
  tile &t = mutable_stuff_at(loc);
  assert(t.contents() == WATER);
  if (t.is_sticky_water() != new_stickyness) {
    t.set_water_stickyness(new_stickyness);
    if (new_stickyness) check_interiorness(loc);
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const tile_location adj_loc = loc + dir;
      // TODO: right now, activate_water automatically calls check_interiorness. Should we remove this redundance...? If so, how to justify "activate_water" being expected to do that? If not, how to justify the pointless repeated computation?
      check_interiorness(adj_loc);
      if (adj_loc.stuff_at().contents() == WATER) activate_water(loc + dir);
    }
  }
}

void world::check_interiorness(tile_location const& loc) {
  tile &t = mutable_stuff_at(loc);
  if (t.is_sticky_water()) {
    bool should_be_interior = true;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const tile_location other_loc = loc.get_neighbor(dir, CONTENTS_AND_STICKYNESS_ONLY);
      if (!other_loc.stuff_at().is_sticky_water()) should_be_interior = false;
    }
    if (t.is_interior_water() != should_be_interior) {
      t.set_water_interiorness(should_be_interior);
      check_exposure_to_collision(loc);
    }
  }
  if (t.contents() == ROCK) {
    bool should_be_interior = true;
    for (EACH_CARDINAL_DIRECTION(dir)) {
      const tile_location other_loc = loc.get_neighbor(dir, CONTENTS_ONLY);
      if (!other_loc.stuff_at().contents() == ROCK) should_be_interior = false;
    }
    if (t.is_interior_rock() != should_be_interior) {
      t.set_rock_interiorness(should_be_interior);
      check_exposure_to_collision(loc);
    }
  }
}

void world::check_exposure_to_collision(tile_location const& loc) {
  tile const& t = loc.stuff_at();
  if ((t.contents() == WATER || t.contents() == ROCK) &&
     !(t.is_interior_water() || t.is_interior_rock())) {
    if (!things_exposed_to_collision.exists(loc)) {
      things_exposed_to_collision.insert(loc, convert_to_fine_units(tile_bounding_box(loc.coords())));
    }
  }
  else {
    if (things_exposed_to_collision.exists(loc)) {
      things_exposed_to_collision.erase(loc);
    }
  }
}

void world::something_changed_at(tile_location const& loc) {
  tile const& t = loc.stuff_at();
  check_exposure_to_collision(loc);
  
  for (EACH_CARDINAL_DIRECTION(dir)) check_interiorness(loc + dir);
  
  if (t.contents() == WATER) {
    check_interiorness(loc);
    activate_water(loc);
  }
  for (EACH_CARDINAL_DIRECTION(dir)) {
    const tile_location adj_loc = loc + dir;
    if (adj_loc.stuff_at().contents() == WATER) {
      activate_water(adj_loc);
    }
    // Because of the "fall off pillars" rule, we may need to activate more stuff.
    // TODO: figure out a way to reduce the definition-duplication for the "fall off pillars" rule.
    if (adj_loc.stuff_at().contents() == AIR) {
      for (EACH_CARDINAL_DIRECTION(d2)) {
        if (d2.v.dot<neighboring_tile_differential>(dir.v) == 0) {
          const tile_location diag_loc = adj_loc + d2;
          if (diag_loc.stuff_at().contents() == WATER) {
            activate_water(diag_loc);
          }
        }
      }
    }
  }
}

void world::delete_rock(tile_location const& loc) {
  tile &t = mutable_stuff_at(loc);
  assert(t.contents() == ROCK);
  t.set_contents(AIR);
  
  something_changed_at(loc);
}

void world::insert_rock(tile_location const& loc) {
  tile &t = mutable_stuff_at(loc);
  assert(t.contents() == AIR);
  t.set_contents(ROCK);
  
  something_changed_at(loc);
}

void world::delete_water(tile_location const& loc) {
  tile &t = mutable_stuff_at(loc);
  assert(t.contents() == WATER);
  t.set_contents(AIR);
  active_water_tiles.erase(loc);
  
  something_changed_at(loc);
}

water_movement_info& world::insert_water(tile_location const& loc) {
  tile &t = mutable_stuff_at(loc);
  assert(t.contents() == AIR);
  t.set_contents(WATER);
    
  something_changed_at(loc);
  
  return active_water_tiles[loc]; // This will always be present, because something_changed_at already activates it.
}

void world::insert_rock_bypassing_checks(tile_location const& loc) {
  tile &t = mutable_stuff_at(loc);
  assert(t.contents() == AIR);
  t.set_contents(ROCK);
}
void world::insert_water_bypassing_checks(tile_location const& loc) {
  tile &t = mutable_stuff_at(loc);
  assert(t.contents() == AIR);
  t.set_contents(WATER);
}

