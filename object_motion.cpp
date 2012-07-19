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

#include <queue>
#include "world.hpp"

namespace /*anonymous*/ {
#if 0
void collect_collisions_if_object_personal_space_is_at(
    world& w,
    unordered_set<object_or_tile_identifier>& results,
    object_identifier id,
    shape const& new_shape
) {
  vector<tile_location> potential_collisions_tile;
  vector<object_identifier> potential_collisions_object;
  const bounding_box new_shape_bounds = new_shape.bounds();
  w.ensure_realization_of_space(new_shape_bounds, FULL_REALIZATION);
  w.tiles_exposed_to_collision().get_objects_overlapping(potential_collisions_tile,
      tile_bbox_to_tiles_collision_detector_bbox(get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(new_shape_bounds)));
  w.objects_exposed_to_collision().get_objects_overlapping(potential_collisions_object,
      new_shape_bounds);
  for (const tile_location loc : potential_collisions_tile) {
    if (tile_shape(loc.coords()).intersects(new_shape)) {
      results.insert(loc);
    }
  }
  for (const object_identifier oid : potential_collisions_object) {
    if (oid != id) {
      if (const auto ps_shape = find_as_pointer(w.get_object_personal_space_shapes(), oid)) {
        if (ps_shape->intersects(new_shape)) {
          results.insert(oid);
        }
      }
    }
  }
}

/*
void actually_change_personal_space_shape(
   object_identifier id, shape const& new_shape,
   object_shapes_t           & personal_space_shapes,
   world_collision_detector  & things_exposed_to_collision) {
  personal_space_shapes[id] = new_shape;
  things_exposed_to_collision.erase(id);
  things_exposed_to_collision.insert(id, new_shape.bounds());
}
*/

cardinal_direction approximate_direction_of_entry(vector3<fine_scalar> const& velocity, bounding_box const& my_bounds, bounding_box const& other_bounds) {
  bounding_box overlapping_bounds(my_bounds);
  overlapping_bounds.restrict_to(other_bounds);
  caller_correct_if(overlapping_bounds.is_anywhere(), "calling approximate_direction_of_entry with non-overlapping bounds");
  caller_correct_if(velocity != vector3<fine_scalar>(0,0,0), "calling approximate_direction_of_entry an object that's not moving");
  cardinal_direction best_dir = xminus; // it shouldn't matter what I initialize it to
  fine_scalar best = -1;
  for (int dim = 0; dim < 3; ++dim) {
    if ((velocity(dim) > 0 && my_bounds.max(dim) < other_bounds.max(dim)) || (velocity(dim) < 0 && my_bounds.min(dim) > other_bounds.min(dim))) {
      const fine_scalar overlap_in_this_dimension = overlapping_bounds.max(dim) - overlapping_bounds.min(dim);
      if (best == -1 || overlap_in_this_dimension < best) {
        best = overlap_in_this_dimension;
        best_dir = cardinal_direction_of_dimension_and_positiveness(dim, velocity[dim] > 0);
      }
    }
  }

  assert_if_ASSERT_EVERYTHING(best != -1);

  return best_dir;
}
#endif

/*
typedef lasercake_int<int64_t>::type stepping_time_type;
const stepping_time_type whole_frame_duration = 1ULL << 32;*/

typedef lasercake_int<int64_t>::type time_int_type;
typedef non_normalized_rational<time_int_type> time_type;
typedef faux_optional<time_type> optional_time;

#if 0
// Currently unused...
void cap_remaining_displacement_to_new_velocity(vector3<fine_scalar>& remaining_displacement, vector3<fine_scalar> const& new_velocity, stepping_time_type remaining_time) {
  for (int i = 0; i < 3; ++i) {
    if (new_velocity[i] * remaining_displacement[i] > 0) {
      const fine_scalar natural_remaining_displacement_in_this_direction = divide_rounding_towards_zero(new_velocity[i] * remaining_time, whole_frame_duration);
      if (std::abs(natural_remaining_displacement_in_this_direction) < std::abs(remaining_displacement[i])) {
        remaining_displacement[i] = natural_remaining_displacement_in_this_direction;
      }
    }
  }
}
#endif


vector3<fine_scalar> movement_delta_from_start_to(vector3<fine_scalar> const& velocity, time_type end_time) {
  // Always round 'up'.
  // This is the simplest way to play nicely with the polyhedron sweep code.
  // It has the downside that it makes everything move, on average, 0.5 fine scalar units faster than it should
  // in each dimension of its movement. 
  return vector3<fine_scalar>(
    sign(velocity.x) * (
      ((std::abs(velocity.x) * end_time.numerator) + (end_time.denominator * velocity_scale_factor) - 1)
      / (                                             end_time.denominator * velocity_scale_factor)
    ),
    sign(velocity.y) * (
      ((std::abs(velocity.y) * end_time.numerator) + (end_time.denominator * velocity_scale_factor) - 1)
      / (                                             end_time.denominator * velocity_scale_factor)
    ),
    sign(velocity.z) * (
      ((std::abs(velocity.z) * end_time.numerator) + (end_time.denominator * velocity_scale_factor) - 1)
      / (                                             end_time.denominator * velocity_scale_factor)
    )
  );
  // Round to nearest:
  /*
  return vector3<fine_scalar>(
    divide_rounding_towards_zero(
      ((2 * velocity.x * end_time.numerator) + (end_time.denominator * velocity_scale_factor)),
      ( 2 *                                     end_time.denominator * velocity_scale_factor )
    ),
    divide_rounding_towards_zero(
      ((2 * velocity.y * end_time.numerator) + (end_time.denominator * velocity_scale_factor)),
      ( 2 *                                     end_time.denominator * velocity_scale_factor )
    ),
    divide_rounding_towards_zero(
      ((2 * velocity.z * end_time.numerator) + (end_time.denominator * velocity_scale_factor)),
      ( 2 *                                     end_time.denominator * velocity_scale_factor )
    )
  );*/
}

vector3<fine_scalar> movement_delta_intermediate(vector3<fine_scalar> const& velocity, time_type min, time_type max) {
  return movement_delta_from_start_to(velocity, max) - movement_delta_from_start_to(velocity, min);
}

// You can round up by up to 1.
// Something you're facing head-on can round up in the other direction by up to 1.
// 
const fine_scalar max_error_dist = 2;

optional_time get_first_moment_of_intersection_(shape const* s1, shape const* s2, vector3<fine_scalar> s1_velocity, vector3<fine_scalar> s2_velocity, time_type s1_last_time_updated, time_type s2_last_time_updated, vector3<fine_scalar> max_error, time_int_type which_step, time_int_type inverse_step_size) {
  std::cerr << "Argh1: " << s1_last_time_updated << ", " << s2_last_time_updated << ", " << which_step <<  ", " << inverse_step_size << "\n";
  time_type step_begin(which_step, inverse_step_size);
  time_type step_end(which_step+1, inverse_step_size);
  if ((step_end < s1_last_time_updated) || (step_end < s2_last_time_updated)) return boost::none;
  
  // I'm not sure exactly what the step-ends should be; this is relatively conservative (because the rounding error will get them covered anyway).
  const vector3<fine_scalar> s1_delta_from_stored_location_to_step_begin = movement_delta_from_start_to(s1_velocity, step_begin - s1_last_time_updated);
  const vector3<fine_scalar> s2_delta_from_stored_location_to_step_begin = movement_delta_from_start_to(s2_velocity, step_begin - s2_last_time_updated);
  const vector3<fine_scalar> s1_delta_from_stored_location_to_step_end = movement_delta_from_start_to(s1_velocity, step_end - s1_last_time_updated);
  const vector3<fine_scalar> s2_delta_from_stored_location_to_step_end = movement_delta_from_start_to(s2_velocity, step_end - s2_last_time_updated);
    
  vector3<fine_scalar> relative_delta_to_step_begin =
    s1_delta_from_stored_location_to_step_begin
    - s2_delta_from_stored_location_to_step_begin;
  vector3<fine_scalar> relative_delta_to_step_end =
    s1_delta_from_stored_location_to_step_end
    - s2_delta_from_stored_location_to_step_end;
  
  vector3<fine_scalar> local_error = max_error;
  /*for (int dim = 0; dim < num_dimensions; ++dim) {
    fine_scalar fail = 
      (std::abs(s1_delta_from_stored_location_to_step_begin(dim)) >= 1) +
      (std::abs(s2_delta_from_stored_location_to_step_begin(dim)) >= 1);
    relative_delta_to_step_begin[dim] -= sign(max_error(dim)) * fail;
    local_error[dim] += sign(max_error(dim)) * fail;
  }*/
  const vector3<fine_scalar> movement_this_step = relative_delta_to_step_end - relative_delta_to_step_begin;
  std::cerr << "Argh2: " << which_step << ", " << inverse_step_size << ", " << s1_velocity << ", " << s2_velocity << ", " << relative_delta_to_step_begin << ", " << relative_delta_to_step_end << ", " << movement_this_step << " ERROR: " << max_error << "\n";
  bool intersects = false;
  // Hack, TODO fix: Only polyhedra collide properly!
  for (convex_polyhedron /*copy, not reference*/ p : s1->get_polyhedra()) {
    p.translate(relative_delta_to_step_begin);
    std::vector<vector3<polygon_int_type>> sweep_vertices;
    polyhedron_planes_info_for_intersection sweep_planes;
    compute_sweep_allowing_rounding_error(p, movement_this_step, local_error, sweep_vertices, sweep_planes);
    for (convex_polyhedron const& p2 : s2->get_polyhedra()) {
      if (sweep_intersects(sweep_vertices, sweep_planes, p2)) {
        intersects = true;
        goto doublebreak;
      }
    }
    for (bounding_box const& bb : s2->get_boxes()) {
      if (sweep_intersects(sweep_vertices, sweep_planes, bb)) {
        intersects = true;
        goto doublebreak;
      }
    }
  }
  doublebreak:
  // HACK: TODO FIX: Ignore results below the top level, because we're getting bad behavior when we don't.
  // Presumably this is the result of false-negatives in the sweep intersection code...
  if ((inverse_step_size > 1) || /*(((time_type(0) < s1_last_time_updated) || (time_type(0) < s2_last_time_updated)) && (s1_velocity != s2_velocity)) ||*/ intersects) {
  std::cerr  << "argh3";
   // assert((s1_velocity != s2_velocity) || s1->intersects(*s2));
    // Because of the rounding error for the two shapes,
    // we may never get below a delta like (2,2,2).
    // Therefore, just stop at that minimum.
    if ((movement_this_step(X) <= 2) && (movement_this_step(X) >= -2) &&
        (movement_this_step(Y) <= 2) && (movement_this_step(Y) >= -2) &&
        (movement_this_step(Z) <= 2) && (movement_this_step(Z) >= -2)) {
      time_type result((which_step == 0) ? 0 : (which_step - 1), inverse_step_size);
  std::cerr  << "argh4";
      if ((result < s1_last_time_updated) || (result < s2_last_time_updated)) return boost::none;
      else return result;
    }
    else {
  std::cerr  << "argh5";
      if (optional_time result =
               get_first_moment_of_intersection_(s1, s2, s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated, max_error,
                 (which_step * 2)    , inverse_step_size * 2)) {
        return result;
      }
      else {
        return get_first_moment_of_intersection_(s1, s2, s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated, max_error,
                 (which_step * 2) + 1, inverse_step_size * 2);
      }
    }
  }
  else {
  std::cerr  << "argh6";
    shape seire(*s1);
    seire.translate(relative_delta_to_step_end);
    assert(!seire.intersects(*s2));
    return boost::none;
  }
}

inline optional_time get_first_moment_of_intersection(shape const* s1, shape const* s2, vector3<fine_scalar> s1_velocity, vector3<fine_scalar> s2_velocity, time_type s1_last_time_updated, time_type s2_last_time_updated) {
  //assert(!s1->intersects(*s2));
  //assert(false);
  return get_first_moment_of_intersection_(s1, s2, s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated,
    vector3<fine_scalar>(
      sign(s1_velocity(X) - s2_velocity(X)) * max_error_dist,
      sign(s1_velocity(Y) - s2_velocity(Y)) * max_error_dist,
      sign(s1_velocity(Z) - s2_velocity(Z)) * max_error_dist
    ),
    0, 1);
}

struct collision_info {
  collision_info(time_type t,object_or_tile_identifier o1,int v1,object_or_tile_identifier o2,int v2):
    time(t),oid1(o1),validation1(v1),oid2(o2),validation2(v2){}
  time_type time;
  object_or_tile_identifier oid1;
  int validation1;
  object_or_tile_identifier oid2;
  int validation2;
  // Backwards sense because std::priority_queue is kinda backwards.
  bool operator<(collision_info const& other)const { return other.time < time; }
};

struct moving_object_info {
  moving_object_info():invalidation_counter(0),last_time_updated(0){}
  int invalidation_counter;
  time_type last_time_updated;
};

// Move the object physically, and update its caches.
// NOTE: This does NOT change its entry in the world's collision detector!
void update_object_to_time(boost::shared_ptr<mobile_object>& obj, shape& personal_space_shape, shape& detail_shape, moving_object_info& inf, time_type time) {
  caller_error_if(time < inf.last_time_updated, "You can't go back in time!");
  if (time > inf.last_time_updated) {
    vector3<fine_scalar> delta = movement_delta_from_start_to(obj->velocity(), time - inf.last_time_updated);
    personal_space_shape.translate(delta);
    detail_shape.translate(delta);
    std::cerr << "(this message may be out-of-date/misleading) Moving object by " << delta << "; ignoring updated already " << movement_delta_from_start_to(obj->velocity(), time) << "; full delta " << movement_delta_from_start_to(obj->velocity(), time_type(1)) << "\n";
    ++inf.invalidation_counter;
    inf.last_time_updated = time;
  }
}
void update_object_for_whole_frame(boost::shared_ptr<mobile_object>& obj, shape& personal_space_shape, shape& detail_shape) {
  vector3<fine_scalar> delta = movement_delta_from_start_to(obj->velocity(), time_type(1));
  personal_space_shape.translate(delta);
  detail_shape.translate(delta);
    std::cerr << "!Moving object by " << delta << "\n";
}

void collect_collisions(time_type min_time, bool erase_old_sweep, object_identifier id, boost::shared_ptr<mobile_object> const& obj, shape const* personal_space_shape, world &w, objects_collision_detector& sweep_box_cd, std::priority_queue<collision_info, std::vector<collision_info>>& anticipated_collisions, std::unordered_map<object_or_tile_identifier, moving_object_info>& objects_info, objects_map<mobile_object>::type const& moving_objects) {
  if (erase_old_sweep) sweep_box_cd.erase(id);
  
  bounding_box sweep_bounds = personal_space_shape->bounds();
  auto delta = movement_delta_from_start_to(obj->velocity(), time_type(1) - min_time);
  if (delta != vector3<fine_scalar>(0,0,0)) {
    // Round up, so that our sweep bounds contains at least the area the rounded polygon sweep will contain
    sweep_bounds.translate(delta + vector3<fine_scalar>(sign(delta.x) * max_error_dist,sign(delta.y) * max_error_dist,sign(delta.z) * max_error_dist));
    sweep_bounds.combine_with(personal_space_shape->bounds());
    assert(sweep_bounds != personal_space_shape->bounds());
  }
  
  w.ensure_realization_of_space(sweep_bounds, CONTENTS_AND_LOCAL_CACHES_ONLY);

  // Did this object (1) potentially pass through another object's sweep
  // and/or (2) potentially hit a stationary object/tile?
  //
  // These checks are conservative and bounding-box based.
  vector<object_identifier> objects_this_could_collide_with;
  sweep_box_cd.get_objects_overlapping(objects_this_could_collide_with, sweep_bounds);
  w.objects_exposed_to_collision().get_objects_overlapping(objects_this_could_collide_with, sweep_bounds);
  
  for (object_identifier other_id : objects_this_could_collide_with) {
  //for (auto const& p1 : moving_objects) {
  //  object_identifier other_id = p1.first;
    if (other_id != id) {
        std::cerr << "Considering " << id << ", " << other_id << "\n";
      shape const* opss = find_as_pointer(w.get_object_personal_space_shapes(), other_id);
      assert(opss);
      vector3<fine_scalar> other_velocity(0,0,0);
      if (boost::shared_ptr<mobile_object> const* other_obj = find_as_pointer(moving_objects, other_id)) {
        other_velocity = (*other_obj)->velocity();
        std::cerr << "Other velocity is relevant." << obj->velocity() << (*other_obj)->velocity();
      }
      const auto i1 = objects_info.find(id);
      const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
      assert(ltu1 == min_time); // TODO just use this instead and not pass min_time?
      const auto i2 = objects_info.find(other_id);
      const time_type ltu2 = (i2 == objects_info.end()) ? time_type(0) : i2->second.last_time_updated;
      if (optional_time result = get_first_moment_of_intersection(personal_space_shape, opss, obj->velocity(), other_velocity, ltu1, ltu2)) {
        anticipated_collisions.push(collision_info(
          *result,
          // operator[] creates the entry if needed
          object_or_tile_identifier(id), objects_info[id].invalidation_counter,
          object_or_tile_identifier(other_id), objects_info[other_id].invalidation_counter));
        std::cerr << "Added " << id << ", " << other_id << "\n";
      }
    }
  }
  
  vector<tile_location> tiles_this_could_collide_with;
  w.tiles_exposed_to_collision().get_objects_overlapping(tiles_this_could_collide_with,
    tile_bbox_to_tiles_collision_detector_bbox(get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(sweep_bounds)));
  for (auto const& loc : tiles_this_could_collide_with) {
      std::cerr << "Considering " << id << ", " << loc << "\n";
    shape t = tile_shape(loc.coords());
    const auto i1 = objects_info.find(id);
    const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
    if (optional_time result = get_first_moment_of_intersection(personal_space_shape, &t, obj->velocity(), vector3<fine_scalar>(0,0,0), ltu1, time_type(0))) {
      anticipated_collisions.push(collision_info(
        *result,
        // operator[] creates the entry if needed
        object_or_tile_identifier(id), objects_info[id].invalidation_counter,
        object_or_tile_identifier(loc), objects_info[loc].invalidation_counter));
      std::cerr << "Added " << id << ", " << loc << "\n";
    }
  }

  sweep_box_cd.insert(id, sweep_bounds);
}

void assert_about_overlaps(objects_map<mobile_object>::type & moving_objects,
   object_shapes_t                  & personal_space_shapes,
   std::unordered_map<object_or_tile_identifier, moving_object_info>& objects_info, bool consider_updated_time = true) {
  
  for (auto const& p1 : moving_objects) {
    for (auto const& p2 : moving_objects) {
      if (p1.first != p2.first) {
        shape s1(personal_space_shapes.find(p1.first)->second);
        shape s2(personal_space_shapes.find(p2.first)->second);
        if (consider_updated_time) {
      const auto i1 = objects_info.find(p1.first);
      const time_type ltu1 = (i1 == objects_info.end()) ? 0 : i1->second.last_time_updated;
      const auto i2 = objects_info.find(p2.first);
      const time_type ltu2 = (i2 == objects_info.end()) ? 0 : i2->second.last_time_updated;
      if (ltu1 > ltu2) {
        s2.translate(movement_delta_from_start_to(p2.second->velocity(), ltu1 - ltu2));
      }
      if (ltu2 > ltu1) {
        s1.translate(movement_delta_from_start_to(p1.second->velocity(), ltu2 - ltu1));
      }
        
        std::cerr << p1.second->velocity() << ":" << ltu1 << ", " << p2.second->velocity() << ":" << ltu2 << "\n";
      }
      else {
      std::cerr << p1.second->velocity() << ", " << p2.second->velocity() << "\n";
      }
        assert(!s1.intersects(s2));
      }
    }
  }
}

void update_moving_objects_impl(
   world                            & w,
   objects_map<mobile_object>::type & moving_objects,
   object_shapes_t                  & personal_space_shapes,
   object_shapes_t                  & detail_shapes,
   objects_collision_detector       & objects_exposed_to_collision) {
   
   
  // Accelerate everything due to gravity.
  // TODO think about: Objects that are standing on the ground shouldn't
  // need to go through the whole computation rigamarole just to learn that
  // they don't end up falling through the ground.
  // Possibly omit them here somehow.
  for (auto& p : moving_objects) {
    p.second->velocity_ += gravity_acceleration;
  }
  
  objects_collision_detector sweep_box_cd;
  std::priority_queue<collision_info, std::vector<collision_info>> anticipated_collisions;
  std::unordered_map<object_or_tile_identifier, moving_object_info> objects_info;
  
  // Initialize all the objects' presumed sweeps through space this frame.
  for (auto const& p : moving_objects) {
    object_identifier const& id = p.first;
    shared_ptr<mobile_object> const& obj = p.second;
    if (shape const* personal_space_shape = find_as_pointer(personal_space_shapes, id)) {
    
      // But first, cap objects' speed in water.
      // Check any tile it happens to be in for whether there's water there.
      // If some tiles are water and some aren't, this is arbitrary, but in that case, it'll crash into a surface water on its first step, which will make this same adjustment.
      if (is_water(w.make_tile_location(get_arbitrary_containing_tile_coordinates(personal_space_shape->arbitrary_interior_point()), CONTENTS_ONLY).stuff_at().contents())) {
        const fine_scalar current_speed = obj->velocity().magnitude_within_32_bits();
        if (current_speed > max_object_speed_through_water) {
          obj->velocity_ = obj->velocity() * max_object_speed_through_water / current_speed;
        }
      }
      
      collect_collisions(time_type(0), false, id, obj, personal_space_shape, w, sweep_box_cd, anticipated_collisions, objects_info, moving_objects);
      
    }
  }
  
  
      std::cerr << "poopPOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOPpoop!\n";
  while(!anticipated_collisions.empty()) {
    std::cerr << "..." << anticipated_collisions.size() << "?\n";
    randomized_vector<collision_info> now_collisions;
    do {
      now_collisions.insert(anticipated_collisions.top(), w.get_rng());
      anticipated_collisions.pop();
    }
    while ((!anticipated_collisions.empty()) && (anticipated_collisions.top().time == now_collisions.begin()->time));
    
    assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
    
    std::cerr << "..." << now_collisions.size() << "!\n";
    for (collision_info collision : now_collisions) {
      std::cerr << "Wham!\n";
      const auto inf1_iter = objects_info.find(collision.oid1);
      assert(inf1_iter != objects_info.end());
      moving_object_info& inf1 = inf1_iter->second;
      if (collision.validation1 == inf1.invalidation_counter){
        const auto inf2_iter = objects_info.find(collision.oid2);
        assert(inf2_iter != objects_info.end());
        moving_object_info& inf2 = inf2_iter->second;
        if (collision.validation2 == inf2.invalidation_counter){
          std::cerr << "BLAM!\n";
          // oid1 is always an object identifier
          object_identifier const* oid1p = collision.oid1.get_object_identifier(); assert(oid1p);
          object_identifier const& oid1 = *oid1p;
          boost::shared_ptr<mobile_object>* obj1p = find_as_pointer(moving_objects, oid1); assert(obj1p);
          boost::shared_ptr<mobile_object>& obj1 = *obj1p;
          shape* o1pss = find_as_pointer(personal_space_shapes, oid1); assert(o1pss);
          shape* o1ds = find_as_pointer(detail_shapes, oid1); assert(o1ds);
        
          if (tile_location const* locp = collision.oid2.get_tile_location()) {
            if (is_water(locp->stuff_at().contents())) {
              const fine_scalar current_speed = obj1->velocity().magnitude_within_32_bits();
              if (current_speed > max_object_speed_through_water) {
                update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
                ++inf1.invalidation_counter; // required in case update_object_to_time did nothing
                obj1->velocity_ = obj1->velocity() * max_object_speed_through_water / current_speed;
                
                assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
              }
            }
            else {
              // We hit a solyd objyct, o noez.
              if (o1pss->intersects(tile_shape(locp->coords()))) {
                // If we were already inside the object, ignore the collision;
                // we wouldn't want an infinite loop.
                // However, this should never happen.
                std::cerr << "Warning: Objects intersecting each other is deprecated.\n";
              }
              else {
                std::cerr << inf1.last_time_updated;
                std::cerr << collision.time;
                update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
                std::cerr << "!!!" << inf1.last_time_updated << "\n";
                ++inf1.invalidation_counter; // required in case update_object_to_time did nothing
                // TODO TODO MAXIMALLY TEMPORARY HACK: Stop moving!
                obj1->velocity_ = vector3<fine_scalar>(0,0,0);
                
                assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
              }
            }
          }
          if (object_identifier const* oid2p = collision.oid2.get_object_identifier()) {
            object_identifier const& oid2 = *oid2p;
            boost::shared_ptr<mobile_object>* obj2p = find_as_pointer(moving_objects, oid2); assert(obj2p);
            boost::shared_ptr<mobile_object>& obj2 = *obj2p;
            shape* o2pss = find_as_pointer(personal_space_shapes, oid2); assert(o2pss);
            shape* o2ds = find_as_pointer(detail_shapes, oid2); assert(o2ds);
            
            // CRASH!!
            std::cerr << oid1 << oid2;
            assert (!o1pss->intersects(*o2pss));
            update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
            ++inf1.invalidation_counter; // required in case update_object_to_time did nothing
            update_object_to_time(obj2, *o2pss, *o2ds, inf2, collision.time);
            ++inf2.invalidation_counter; // required in case update_object_to_time did nothing
            assert (!o1pss->intersects(*o2pss));
            assert_about_overlaps(moving_objects, personal_space_shapes,objects_info);
            
            // TODO TEMPORARY HACK: Just stick together and equalize velocities
            obj1->velocity_ = (obj1->velocity() + obj2->velocity()) / 2;
            obj2->velocity_ = obj1->velocity();
          
            // If we updated anything about the objects, they might have new collisions.
            // If we *didn't*, they won't repeat the same collision again, since we have
            //     deleted it and not replaced it.
            // Note: Duplicate comment with the below
            if (inf2.invalidation_counter > collision.validation2) {
              collect_collisions(collision.time, true, oid2, obj2, o2pss, w, sweep_box_cd, anticipated_collisions, objects_info, moving_objects);
            }
          }
          // If we updated anything about the objects, they might have new collisions.
          // If we *didn't*, they won't repeat the same collision again, since we have
          //     deleted it and not replaced it.
          // Note: Duplicate comment with the above
          if (inf1.invalidation_counter > collision.validation1) {
            collect_collisions(collision.time, true, oid1, obj1, o1pss, w, sweep_box_cd, anticipated_collisions, objects_info, moving_objects);
          }
        }
      }
    }
  }
  
  assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
  std::cerr << "Excellently.\n";
  for (auto const& p : moving_objects) {
    auto const& oid = p.first;
    boost::shared_ptr<mobile_object>* objp = find_as_pointer(moving_objects, oid); assert(objp);
    boost::shared_ptr<mobile_object>& obj = *objp;
    shape* opss = find_as_pointer(personal_space_shapes, oid); assert(opss);
    shape* ods = find_as_pointer(detail_shapes, oid); assert(ods);
    auto inf = objects_info.find(object_or_tile_identifier(oid));
    if (inf == objects_info.end()) {
      update_object_for_whole_frame(obj, *opss, *ods);
      std::cerr << "Minimally so.\n";
    }
    else {
      update_object_to_time(obj, *opss, *ods, inf->second, time_type(1));
      std::cerr << "Quite so.\n";
    }
    
    // Update the collision detector entries
    bounding_box new_bounds = opss->bounds();
    new_bounds.combine_with(ods->bounds());
    objects_exposed_to_collision.erase(oid);
    objects_exposed_to_collision.insert(oid, new_bounds);
  }
  assert_about_overlaps(moving_objects, personal_space_shapes, objects_info, false);
   
   
   
   
#if 0
  // 0LD C0DE
   
  // This entire function is kludgy and horrifically un-optimized.
  
  // Accelerate everything due to gravity.
  for (pair<const object_identifier, shared_ptr<mobile_object>>& p : moving_objects) {
    vector3<fine_scalar> obj_velocity_temp = p.second->velocity();
    obj_velocity_temp += gravity_acceleration;
    
    // Check any tile it happens to be in for whether there's water there.
    // If the object is *partially* in water, it'll crash into a surface water on its first step, which will make this same adjustment.
    if (is_water(w.make_tile_location(get_arbitrary_containing_tile_coordinates(personal_space_shapes[p.first].arbitrary_interior_point()), CONTENTS_ONLY).stuff_at().contents())) {
      const fine_scalar current_speed = obj_velocity_temp.magnitude_within_32_bits();
      if (current_speed > max_object_speed_through_water) {
        obj_velocity_temp = obj_velocity_temp * max_object_speed_through_water / current_speed;
      }
    }
    
    p.second->velocity_ = obj_velocity_temp;
  }
  
  objects_collision_detector sweep_box_cd;
  unordered_set<object_identifier> objects_with_some_overlap;

  // Initialize all the objects' presumed sweeps through space this frame.
  for (auto const& p : moving_objects) {
    if (shape const* old_personal_space_shape = find_as_pointer(personal_space_shapes, p.first)) {
      bounding_box dst_bounds = old_personal_space_shape->bounds()
      dst_personal_space_shape.translate(p.second->velocity() / velocity_scale_factor);
      
      bounding_box sweep_bounds = dst_personal_space_shape.bounds();
      sweep_bounds.combine_with(old_personal_space_shape->bounds());

      // Did this object (1) potentially pass through another object's sweep
      // and/or (2) potentially hit a stationary object/tile?
      //
      // These checks are conservative and bounding-box based.
      vector<object_identifier> objects_this_could_collide_with;
      sweep_box_cd.get_objects_overlapping(objects_this_could_collide_with, sweep_bounds);
      w.ensure_realization_of_space(sweep_bounds, CONTENTS_AND_LOCAL_CACHES_ONLY);
      w.objects_exposed_to_collision().get_objects_overlapping(objects_this_could_collide_with, sweep_bounds);
      
      bool this_is_overlapping = false;
      for (object_identifier obj_id : objects_this_could_collide_with) {
        if (obj_id != p.first) {
          objects_with_some_overlap.insert(obj_id);
          this_is_overlapping = true;
        }
      }
      // Optimization - if this is already overlapping an object's sweep,
      // we don't need to look at the tiles now.
      if(!this_is_overlapping) {
        vector<tile_location> tiles_this_could_collide_with;
        // TODO we don't even need to get all of them, just see if there are any of them.
        w.tiles_exposed_to_collision().get_objects_overlapping(tiles_this_could_collide_with,
          tile_bbox_to_tiles_collision_detector_bbox(get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(sweep_bounds)));
        if (!tiles_this_could_collide_with.empty()) {
          // All tiles_exposed_to_collision affect you as you touch them.
          // Tiles themselves cannot move in this moving_objects way so we don't add
          // them to any collections here.
          this_is_overlapping = true;
        }
      }

      if (this_is_overlapping) objects_with_some_overlap.insert(p.first);

      sweep_box_cd.insert(p.first, sweep_bounds);
    }
  }
  
  
  struct object_trajectory_info {
    object_trajectory_info(){}
    object_trajectory_info(vector3<fine_scalar>r):last_step_time(0),remaining_displacement(r),accumulated_displacement(0,0,0){}
    
    stepping_time_type last_step_time;
    vector3<fine_scalar> remaining_displacement;
    vector3<fine_scalar> accumulated_displacement;
  };
  
  unordered_map<object_identifier, object_trajectory_info> trajinfo;
  
  struct stepping_times {
    stepping_time_type current_time;
    std::multimap<stepping_time_type, object_identifier> queued_steps;
    stepping_times():current_time(0){}
    
    void queue_next_step(object_identifier id, object_trajectory_info info) {
      if (info.last_step_time == whole_frame_duration) return;
      
      fine_scalar dispmag = info.remaining_displacement.magnitude_within_32_bits();
      
      stepping_time_type step_time;
      if (dispmag == 0) step_time = whole_frame_duration;
      else {
        // We're content to move in increments of tile_width >> 5 or less
        // ((whole_frame_duration - info.last_step_time) / dispmag) is simply the inverse speed (in normal fine units per time unit) over the rest of the frame
        // With (max step distance) = (max step time) * (speed)
        // (max step time) = (max step distance) / speed
        // Recall that dispmag is in regular fine units instead of velocity units.
        const stepping_time_type max_normal_step_duration = (((tile_width >> 5) * (whole_frame_duration - info.last_step_time)) / dispmag);
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
    trajinfo[id] = object_trajectory_info(objp->velocity() / velocity_scale_factor);
    times.queue_next_step(id, trajinfo[id]);
  }
  
  while(!times.queued_steps.empty()) {
    object_identifier id = times.pop_next_step();
    shared_ptr<mobile_object> objp = moving_objects[id];
    object_trajectory_info& info = trajinfo[id];
    
    shape new_shape(personal_space_shapes[id]);
      
    vector3<fine_scalar> wanted_displacement_this_step = info.remaining_displacement * (times.current_time - info.last_step_time) / (whole_frame_duration - info.last_step_time);
      
    new_shape.translate(wanted_displacement_this_step);

    vector<object_identifier> objects_this_would_collide_with;
    vector<tile_location> tiles_this_would_collide_with;
    // TODO fix if you'd run into water AND rock/object on the same step
    // (or rewrite this code!)
    w.ensure_realization_of_space(new_shape.bounds(), CONTENTS_AND_LOCAL_CACHES_ONLY);
    w.objects_exposed_to_collision().get_objects_overlapping(objects_this_would_collide_with, new_shape.bounds());
    w.tiles_exposed_to_collision().get_objects_overlapping(tiles_this_would_collide_with,
      tile_bbox_to_tiles_collision_detector_bbox(get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(new_shape.bounds())));
    
    // This collision code is kludgy because of the way it handles one collision at a time.
    // TODO properly consider multiple collisions in the same step.
    bool this_step_needs_adjusting = false;
    for (tile_location them : tiles_this_would_collide_with) {
      if (is_water(them.stuff_at().contents())) {
        const fine_scalar current_speed = objp->velocity_.magnitude_within_32_bits();
        if (current_speed > max_object_speed_through_water) {
          objp->velocity_ = objp->velocity_ * max_object_speed_through_water / current_speed;
          info.remaining_displacement = info.remaining_displacement * max_object_speed_through_water / current_speed;
          this_step_needs_adjusting = true;
          break;
        }
      }
      else {
        const bounding_box their_shape = fine_bounding_box_of_tile(them.coords());
        if (new_shape.intersects(their_shape) && !personal_space_shapes[id].intersects(their_shape)) {
          this_step_needs_adjusting = true;
          cardinal_direction approx_impact_dir = approximate_direction_of_entry(objp->velocity_, new_shape.bounds(), their_shape);

          objp->velocity_ -= project_onto_cardinal_direction(objp->velocity_, approx_impact_dir);
          info.remaining_displacement -= project_onto_cardinal_direction(info.remaining_displacement, approx_impact_dir);
          break;
        }
      }
    }
    if(!this_step_needs_adjusting) {
      for (object_identifier const& them : objects_this_would_collide_with) {
        if (them != id) {
          shape their_shape = w.get_personal_space_shape_of_object_or_tile(them);
          if (new_shape.intersects(their_shape) && !personal_space_shapes[id].intersects(their_shape)) {
            this_step_needs_adjusting = true;
            cardinal_direction approx_impact_dir = approximate_direction_of_entry(objp->velocity_, new_shape.bounds(), their_shape.bounds());

            objp->velocity_ -= project_onto_cardinal_direction(objp->velocity_, approx_impact_dir);
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
      objects_exposed_to_collision.erase(id);
      objects_exposed_to_collision.insert(id, new_shape.bounds());
      
      info.accumulated_displacement += wanted_displacement_this_step;
      info.remaining_displacement -= wanted_displacement_this_step;
      info.last_step_time = times.current_time;
    }
    times.queue_next_step(id, info);
      
    if (false) { // if our velocity changed...
        info.remaining_displacement = (objp->velocity() * (whole_frame_duration - times.current_time)) / (whole_frame_duration * velocity_scale_factor);
        vector<object_identifier> new_sweep_overlaps;
        
        shape dst_personal_space_shape(personal_space_shapes[id]);
        dst_personal_space_shape.translate(info.remaining_displacement);
        bounding_box sweep_bounds = dst_personal_space_shape.bounds();
        sweep_bounds.combine_with(personal_space_shapes[id].bounds());
        sweep_box_cd.erase(id);
        sweep_box_cd.insert(id, sweep_bounds);
        sweep_box_cd.get_objects_overlapping(new_sweep_overlaps, sweep_bounds);
      
        for (object_identifier const& obj_id : new_sweep_overlaps) {
          if (obj_id != id) {
            if (objects_with_some_overlap.find(obj_id) == objects_with_some_overlap.end()) {
              objects_with_some_overlap.insert(obj_id);
              trajinfo[obj_id] = object_trajectory_info(moving_objects[obj_id]->velocity() / velocity_scale_factor);
              times.queue_next_step(obj_id, trajinfo[obj_id]);
            }
          }
        }
    }
  }
  
  for (auto& p : moving_objects) {
    if (objects_with_some_overlap.find(p.first) == objects_with_some_overlap.end()) {
      personal_space_shapes[p.first].translate(p.second->velocity() / velocity_scale_factor);
      detail_shapes[p.first].translate(p.second->velocity() / velocity_scale_factor);
    }
    else {
      detail_shapes[p.first].translate(trajinfo[p.first].accumulated_displacement);
    }
      
    bounding_box new_bounds = personal_space_shapes[p.first].bounds();
    new_bounds.combine_with(detail_shapes[p.first].bounds());
    objects_exposed_to_collision.erase(p.first);
    objects_exposed_to_collision.insert(p.first, new_bounds);
  }
#endif
}

} /* end anonymous namespace */

void world::update_moving_objects() {
  update_moving_objects_impl(*this, moving_objects_, object_personal_space_shapes_, object_detail_shapes_, objects_exposed_to_collision_);
}

#if 0
// If objects overlap with the new position, returns their IDs. If not, changes the shape and returns an empty set.
unordered_set<object_or_tile_identifier> try_to_change_personal_space_shape_impl(world& w, object_shapes_t& personal_space_shapes, world_collision_detector& things_exposed_to_collision, object_identifier id, shape const& new_shape) {
  unordered_set<object_or_tile_identifier> collisions;
  collect_collisions_if_object_personal_space_is_at(w, collisions, id, new_shape);
  
  if (collisions.empty()) {
    // Actually move (and create if you weren't there):
    actually_change_personal_space_shape(id, new_shape, personal_space_shapes, things_exposed_to_collision);
  }
  
  return collisions;
}

unordered_set<object_or_tile_identifier> world::try_to_change_personal_space_shape(object_identifier id, shape const& new_shape) {
  return try_to_change_personal_space_shape_impl(*this, object_personal_space_shapes, things_exposed_to_collision, id, new_shape);
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

