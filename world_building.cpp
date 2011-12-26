
#include "world.hpp"

void operator()(tile_contents new_contents, vector3<location_coordinate> locv) {
  assert(bounds.contains(locv));
  if (new_contents == ROCK) {
    w->insert_rock(w->make_location(locv));
  }
  else if (new_contents == WATER) {
    w->insert_water(w->make_location(locv));
  }
  else assert("YOU CAN ONLY PLACE ROCK AND WATER" && false);
}
