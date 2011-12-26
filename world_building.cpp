
#include "world.hpp"

void world_building_gun::operator()(tile_contents new_contents, vector3<location_coordinate> locv) {
  assert(bounds.contains(locv));
  if (new_contents == ROCK) {
    w->insert_rock_bypassing_checks(w->make_location(locv));
  }
  else if (new_contents == WATER) {
    w->insert_water_bypassing_checks(w->make_location(locv));
  }
  else assert("YOU CAN ONLY PLACE ROCK AND WATER" && false);
}
