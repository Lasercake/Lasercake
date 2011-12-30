
#include "world.hpp"

void update_mobile_objects(world &w) {
  for (pair<const mobile_object_identifier, shared_ptr<mobile_object>> &p : w.mobile_objects_range()) {
    p.second->move_due_to_velocity();
  }
  for (pair<const mobile_object_identifier, shared_ptr<mobile_object>> &p : w.mobile_objects_range()) {
    p.second->update(&w);
  }
}
