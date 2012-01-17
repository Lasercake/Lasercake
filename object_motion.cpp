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

void collect_collisions_if_object_personal_space_is_at(world& w, unordered_set<object_or_tile_identifier> &results, object_identifier id, shape const& new_shape) {
  unordered_set<object_or_tile_identifier> potential_collisions;
  w.collect_things_exposed_to_collision_intersecting(potential_collisions, new_shape.bounds());
  for (auto const& pc : potential_collisions){
    if (const auto tlocp = pc.get_tile_location()) {
      if (tile_shape(tlocp->coords()).intersects(new_shape)) {
        results.insert(pc);
      }
    }
    if (const auto oidp = pc.get_object_identifier()) {
      if (*oidp != id) {
        if (const auto ps_shape = find_as_pointer(w.get_object_personal_space_shapes(), *oidp)) {
          if (ps_shape->intersects(new_shape)) {
            results.insert(pc);
          }
        }
      }
    }
  }
}

/*
void actually_change_personal_space_shape(
   object_identifier id, shape const& new_shape,
   object_shapes_t           &personal_space_shapes,
   world_collision_detector  &things_exposed_to_collision) {
  personal_space_shapes[id] = new_shape;
  things_exposed_to_collision.erase(id);
  things_exposed_to_collision.insert(id, new_shape.bounds());
}
*/

cardinal_direction approximate_direction_of_entry(vector3<fine_scalar> const& velocity, bounding_box const& my_bounds, bounding_box const& other_bounds) {
  bounding_box overlapping_bounds(my_bounds);
  overlapping_bounds.restrict_to(other_bounds);
  caller_correct_if(overlapping_bounds.is_anywhere, "calling approximate_direction_of_entry with non-overlapping bounds");
  caller_correct_if(velocity != vector3<fine_scalar>(0,0,0), "calling approximate_direction_of_entry an object that's not moving");
  cardinal_direction best_dir = xminus; // it shouldn't matter what I initialize it to
  fine_scalar best = -1;
  for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
    if (velocity.dot<fine_scalar>(cardinal_direction_vectors[dir]) > 0) {
      const fine_scalar overlap_in_this_dimension = overlapping_bounds.max[which_dimension_is_cardinal_direction(dir)] - overlapping_bounds.min[which_dimension_is_cardinal_direction(dir)];
      if (best == -1 || overlap_in_this_dimension < best) {
        best = overlap_in_this_dimension;
        best_dir = dir;
      }
    }
  }
#ifdef ASSERT_EVERYTHING
  assert(best != -1);
#endif
  return best_dir;
}

  
const int64_t whole_frame_duration = 1ULL << 32;

// Currently unused...
void cap_remaining_displacement_to_new_velocity(vector3<fine_scalar> &remaining_displacement, vector3<fine_scalar> const& new_velocity, int64_t remaining_time) {
  for (int i = 0; i < 3; ++i) {
    if (new_velocity[i] * remaining_displacement[i] > 0) {
      const fine_scalar natural_remaining_displacement_in_this_direction = divide_rounding_towards_zero(new_velocity[i] * remaining_time, whole_frame_duration);
      if (std::abs(natural_remaining_displacement_in_this_direction) < std::abs(remaining_displacement[i])) {
        remaining_displacement[i] = natural_remaining_displacement_in_this_direction;
      }
    }
  }
}

void update_moving_objects_(
   world                             &w,
   objects_map<mobile_object>::type  &moving_objects,
   object_shapes_t                   &personal_space_shapes,
   object_shapes_t                   &detail_shapes,
   world_collision_detector          &things_exposed_to_collision) {
   
  // This entire function is kludgy and horrifically un-optimized.
  
  // Accelerate everything due to gravity.
  for (pair<const object_identifier, shared_ptr<mobile_object>> &p : moving_objects) {
    p.second->velocity += gravity_acceleration;
    
    // Check any tile it happens to be in for whether there's water there.
    // If the object is *partially* in water, it'll crash into a surface water on its first step, which will make this same adjustment.
    if (is_water(w.make_tile_location(get_containing_tile_coordinates(personal_space_shapes[p.first].arbitrary_interior_point()), CONTENTS_ONLY).stuff_at().contents())) {
      const fine_scalar current_speed = p.second->velocity.magnitude_within_32_bits();
      if (current_speed > max_object_speed_through_water) {
        p.second->velocity = p.second->velocity * max_object_speed_through_water / current_speed;
      }
    }
  }
  
  world_collision_detector sweep_box_cd;
  
  unordered_set<object_identifier> objects_with_some_overlap;
  for (auto const& p : moving_objects) {
    if (shape const* old_personal_space_shape = find_as_pointer(personal_space_shapes, p.first)) {
      shape dst_personal_space_shape(*old_personal_space_shape);
      dst_personal_space_shape.translate(p.second->velocity / velocity_scale_factor);
      
      bounding_box sweep_bounds = dst_personal_space_shape.bounds();
      sweep_bounds.combine_with(old_personal_space_shape->bounds());
      
      sweep_box_cd.insert(p.first, sweep_bounds);
      
      unordered_set<object_or_tile_identifier> this_overlaps;
      sweep_box_cd.get_objects_overlapping(this_overlaps, sweep_bounds);
      w.collect_things_exposed_to_collision_intersecting(this_overlaps, sweep_bounds);
      
      bool this_is_overlapping = false;
      for (object_or_tile_identifier const& foo : this_overlaps) {
        if (object_identifier const* bar = foo.get_object_identifier()) {
          if (*bar != p.first) {
            objects_with_some_overlap.insert(*bar);
            this_is_overlapping = true;
          }
        }
        else {
          // we must be a tile! nothing to do about that...
          this_is_overlapping = true;
        }
      }
      if (this_is_overlapping) objects_with_some_overlap.insert(p.first);
    }
  }
  
  
  struct object_trajectory_info {
    object_trajectory_info(){}
    object_trajectory_info(vector3<fine_scalar>r):last_step_time(0),remaining_displacement(r),accumulated_displacement(0,0,0){}
    
    int64_t last_step_time;
    vector3<fine_scalar> remaining_displacement;
    vector3<fine_scalar> accumulated_displacement;
  };
  
  unordered_map<object_identifier, object_trajectory_info> trajinfo;
  
  struct stepping_times {
    int64_t current_time;
    std::multimap<int64_t, object_identifier> queued_steps;
    stepping_times():current_time(0){}
    
    void queue_next_step(object_identifier id, object_trajectory_info info) {
      if (info.last_step_time == whole_frame_duration) return;
      
      fine_scalar dispmag = info.remaining_displacement.magnitude_within_32_bits();
      
      int64_t step_time;
      if (dispmag == 0) step_time = whole_frame_duration;
      else {
        // We're content to move in increments of tile_width >> 5 or less
        // ((whole_frame_duration - info.last_step_time) / dispmag) is simply the inverse speed (in normal fine units per time unit) over the rest of the frame
        // With (max step distance) = (max step time) * (speed)
        // (max step time) = (max step distance) / speed
        // Recall that dispmag is in regular fine units instead of velocity units.
        const int64_t max_normal_step_duration = (((tile_width >> 5) * (whole_frame_duration - info.last_step_time)) / dispmag);
        if (info.last_step_time + max_normal_step_duration > current_time) step_time = info.last_step_time + max_normal_step_duration;
        else step_time = current_time;
      }
      
      queued_steps.insert(make_pair(std::min(whole_frame_duration, step_time), id));
    }
    
    object_identifier pop_next_step() {
      current_time = queued_steps.begin()->first;
      object_identifier foo = queued_steps.begin()->second;
      queued_steps.erase(queued_steps.begin());
      return foo;
    }
  };
  
  stepping_times times;
  
  for (object_identifier id : objects_with_some_overlap) {
    shared_ptr<mobile_object> objp = moving_objects[id];
    trajinfo[id] = object_trajectory_info(objp->velocity / velocity_scale_factor);
    times.queue_next_step(id, trajinfo[id]);
  }
  
  while(!times.queued_steps.empty()) {
    object_identifier id = times.pop_next_step();
    shared_ptr<mobile_object> objp = moving_objects[id];
    object_trajectory_info &info = trajinfo[id];
    
    shape new_shape(personal_space_shapes[id]);
      
    vector3<fine_scalar> wanted_displacement_this_step = info.remaining_displacement * (times.current_time - info.last_step_time) / (whole_frame_duration - info.last_step_time);
      
    new_shape.translate(wanted_displacement_this_step);
      
    unordered_set<object_or_tile_identifier> this_overlaps;
    w.collect_things_exposed_to_collision_intersecting(this_overlaps, new_shape.bounds());
    
    // This collision code is kludgy because of the way it handles one collision at a time.
    // TODO properly consider multiple collisions in the same step.
    bool this_step_needs_adjusting = false;
    for (object_or_tile_identifier const& them : this_overlaps) {
      if (them != id) {
        tile_location const* locp = them.get_tile_location();
        if (locp && is_water(locp->stuff_at().contents())) {
          const fine_scalar current_speed = objp->velocity.magnitude_within_32_bits();
          if (current_speed > max_object_speed_through_water) {
            objp->velocity = objp->velocity * max_object_speed_through_water / current_speed;
            info.remaining_displacement = info.remaining_displacement * max_object_speed_through_water / current_speed;
            this_step_needs_adjusting = true;
            break;
          }
        }
        else {
          shape their_shape = w.get_personal_space_shape_of_object_or_tile(them);
          if (new_shape.intersects(their_shape) && !personal_space_shapes[id].intersects(their_shape)) {
            this_step_needs_adjusting = true;
            cardinal_direction approx_impact_dir = approximate_direction_of_entry(objp->velocity, new_shape.bounds(), their_shape.bounds());
            
            objp->velocity -= project_onto_cardinal_direction(objp->velocity, approx_impact_dir);
            info.remaining_displacement -= project_onto_cardinal_direction(info.remaining_displacement, approx_impact_dir);
            break;
          }
        }
      }
    }
    
    if (this_step_needs_adjusting) {
      // don't update last_step_time - it will compute another step at current_time
    }
    else {
      // Hack: We put ourselves in the tree with only our personal space shape, not
      // the combined bounds of that and the detail shape. I think this is just to save
      // complication and time (since we currently have no reason to consider the
      // detail shape during movement/collisions.)
      personal_space_shapes[id] = new_shape;
      things_exposed_to_collision.erase(id);
      things_exposed_to_collision.insert(id, new_shape.bounds());
      
      info.accumulated_displacement += wanted_displacement_this_step;
      info.remaining_displacement -= wanted_displacement_this_step;
      info.last_step_time = times.current_time;
    }
    times.queue_next_step(id, info);
      
    if (false) { // if our velocity changed...
        info.remaining_displacement = (objp->velocity * (whole_frame_duration - times.current_time)) / (whole_frame_duration * velocity_scale_factor);
        unordered_set<object_or_tile_identifier> new_sweep_overlaps;
        
        shape dst_personal_space_shape(personal_space_shapes[id]);
        dst_personal_space_shape.translate(info.remaining_displacement);
        bounding_box sweep_bounds = dst_personal_space_shape.bounds();
        sweep_bounds.combine_with(personal_space_shapes[id].bounds());
        sweep_box_cd.erase(id);
        sweep_box_cd.insert(id, sweep_bounds);
        sweep_box_cd.get_objects_overlapping(new_sweep_overlaps, sweep_bounds);
      
        for (object_or_tile_identifier const& foo : this_overlaps) {
          if (object_identifier const* bar = foo.get_object_identifier()) {
            if (*bar != id) {
              if (objects_with_some_overlap.find(*bar) == objects_with_some_overlap.end()) {
                objects_with_some_overlap.insert(*bar);
                trajinfo[*bar] = object_trajectory_info(moving_objects[*bar]->velocity / velocity_scale_factor);
                times.queue_next_step(*bar, trajinfo[*bar]);
              }
            }
          }
        }
    }
  }
  
  for (auto &p : moving_objects) {
    if (objects_with_some_overlap.find(p.first) == objects_with_some_overlap.end()) {
      personal_space_shapes[p.first].translate(p.second->velocity / velocity_scale_factor);
      detail_shapes[p.first].translate(p.second->velocity / velocity_scale_factor);
    }
    else {
      detail_shapes[p.first].translate(trajinfo[p.first].accumulated_displacement);
    }
      
    bounding_box new_bounds = personal_space_shapes[p.first].bounds();
    new_bounds.combine_with(detail_shapes[p.first].bounds());
    things_exposed_to_collision.erase(p.first);
    things_exposed_to_collision.insert(p.first, new_bounds);
  }
}

void world::update_moving_objects() {
  update_moving_objects_(*this, moving_objects, object_personal_space_shapes, object_detail_shapes, things_exposed_to_collision);
}

#if 0
// If objects overlap with the new position, returns their IDs. If not, changes the shape and returns an empty set.
unordered_set<object_or_tile_identifier> try_to_change_personal_space_shape_(world &w, object_shapes_t &personal_space_shapes, world_collision_detector &things_exposed_to_collision, object_identifier id, shape const& new_shape) {
  unordered_set<object_or_tile_identifier> collisions;
  collect_collisions_if_object_personal_space_is_at(w, collisions, id, new_shape);
  
  if (collisions.empty()) {
    // Actually move (and create if you weren't there):
    actually_change_personal_space_shape(id, new_shape, personal_space_shapes, things_exposed_to_collision);
  }
  
  return collisions;
}

unordered_set<object_or_tile_identifier> world::try_to_change_personal_space_shape(object_identifier id, shape const& new_shape) {
  return try_to_change_personal_space_shape_(*this, object_personal_space_shapes, things_exposed_to_collision, id, new_shape);
}

// Objects can't fail to change their detail shape, but it may cause effects (like blocking a laser beam)
void world::change_detail_shape(object_identifier id, shape const& new_shape) {
  // Move
  object_detail_shapes[id] = new_shape;
  
  // Currently, moving a detail shape changes nothing.
  
  /*unordered_set<object_identifier> potential_collisions;
  collect_things_exposed_to_collision_intersecting(potential_collisions, new_shape.bounds());
  for (auto const& pc : potential_collisions){
    if (auto oidp = pc.get_object_identifier()) {
      if (auto d_shape = find_as_pointer(mobile_object_detail_shapes, *oidp)) {
        if (d_shape->intersects(new_shape)) {
          
        }
      }
    }
  }*/
}
#endif

