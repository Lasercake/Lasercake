
#include "world.hpp"

// If objects overlap with the new position, returns their IDs. If not, changes the shape and returns an empty set.
unordered_set<object_identifier> world::try_to_change_personal_space_shape(mobile_object_identifier id, shape const& new_shape) {
  unordered_set<object_identifier> potential_collisions;
  unordered_set<object_identifier> actual_personal_space_collisions;
  collect_things_exposed_to_collision_intersecting(potential_collisions, new_shape.bounds());
  for (auto const& pc : potential_collisions){
    if (auto tlocp = pc.get_tile_location()) {
      if (tile_shape(tlocp->coords)->intersects(new_shape)) {
        actual_personal_space_collisions.insert(pc);
      }
    }
    if (auto oidp = pc.get_mobile_object_identifier()) {
      if (auto ps_shape = find_as_pointer(mobile_object_personal_space_shapes, *oidp)) {
        if (ps_shape->intersects(new_shape)) {
          actual_personal_space_collisions.insert(pc);
        }
      }
    }
  }
  
  if (actual_personal_space_collisions.empty()) {
    // Actually move:
    mobile_object_personal_space_shapes[id] = new_shape;
  }
  // (And if we didn't actually move, we don't actually make the detail collisions)
  
  return actual_personal_space_collisions;
}

// Objects can't fail to change their detail shape, but it may cause effects (like blocking a laser beam)
void change_detail_shape(mobile_object_identifier id, shape const& new_shape) {
  // Move
  mobile_object_detail_shapes[id] = new_shape;
  
  // Currently, moving a detail shape changes nothing.
  
  /*unordered_set<object_identifier> potential_collisions;
  collect_things_exposed_to_collision_intersecting(potential_collisions, new_shape.bounds());
  for (auto const& pc : potential_collisions){
    if (auto oidp = pc.get_mobile_object_identifier()) {
      if (auto d_shape = find_as_pointer(mobile_object_detail_shapes, *oidp)) {
        if (d_shape->intersects(new_shape)) {
          
        }
      }
    }
  }*/
}

void update_mobile_objects(world &w) {
  for (pair<const mobile_object_identifier, shared_ptr<mobile_object>> &p : w.mobile_objects_range()) {
    p.second->move_due_to_velocity();
  }
  for (pair<const mobile_object_identifier, shared_ptr<mobile_object>> &p : w.mobile_objects_range()) {
    p.second->update(&w);
  }
}

