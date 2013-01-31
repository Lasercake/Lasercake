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

typedef lasercake_int<int64_t>::type time_int_type;
typedef non_normalized_rational<time_int_type> time_type;
typedef faux_optional<time_type> optional_time;

vector3<fine_scalar> movement_delta_from_start_to(vector3<fine_scalar> const& velocity, time_type end_time) {
  // Round to nearest:
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
  );
}

vector3<fine_scalar> movement_delta_rounding_up(vector3<fine_scalar> const& velocity, time_type end_time) {
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
}

vector3<fine_scalar> movement_delta_rounding_down(vector3<fine_scalar> const& velocity, time_type end_time) {
  return vector3<fine_scalar>(
    divide_rounding_towards_zero(
      (velocity.x * end_time.numerator),
      (end_time.denominator * velocity_scale_factor)
    ),
    divide_rounding_towards_zero(
      (velocity.y * end_time.numerator),
      (end_time.denominator * velocity_scale_factor)
    ),
    divide_rounding_towards_zero(
      (velocity.z * end_time.numerator),
      (end_time.denominator * velocity_scale_factor)
    )
  );
}

vector3<fine_scalar> movement_delta_intermediate(vector3<fine_scalar> const& velocity, time_type min, time_type max) {
  return movement_delta_from_start_to(velocity, max) - movement_delta_from_start_to(velocity, min);
}

// You can round up by up to 1.
// Something you're facing head-on can round up in the other direction by up to 1.
// 
const fine_scalar max_error_dist = 2;


time_type round_time_downwards(time_type time, time_int_type max_meaningful_precision) {
  if (time.denominator < 0) {
    time.numerator = -time.numerator;
    time.denominator = -time.denominator;
  }
    //ilog2 is giving me weird errors I don't understand. TODO understand that and do this the right way
  /*if (farthest_plane_cross.denominator > max_meaningful_precision) {
    int shift_amount = ilog2(farthest_plane_cross.denominator) - ilog2(max_meaningful_precision);
    farthest_plane_cross.numerator >>= shift_amount;
    farthest_plane_cross.denominator = (farthest_plane_cross.denominator + (1 << shift_amount) - 1) >> shift_amount;
  }*/
  while (time.denominator > max_meaningful_precision) {
    int shift_amount = 2;
    time.numerator >>= shift_amount;
    time.denominator = (time.denominator + (1 << shift_amount) - 1) >> shift_amount;
  }
  return time;
}

time_type round_time_upwards(time_type time, time_int_type max_meaningful_precision) {
  if (time.denominator < 0) {
    time.numerator = -time.numerator;
    time.denominator = -time.denominator;
  }
    //ilog2 is giving me weird errors I don't understand. TODO understand that and do this the right way
  /*if (farthest_plane_cross.denominator > max_meaningful_precision) {
    int shift_amount = ilog2(farthest_plane_cross.denominator) - ilog2(max_meaningful_precision);
    farthest_plane_cross.numerator >>= shift_amount;
    farthest_plane_cross.denominator = (farthest_plane_cross.denominator + (1 << shift_amount) - 1) >> shift_amount;
  }*/
  while(time.denominator > max_meaningful_precision) {
    int shift_amount = 2;
    time.denominator >>= shift_amount;
    time.numerator = (time.numerator + (1 << shift_amount) - 1) >> shift_amount;
  }
  return time;
}

// This is essentially arbitrary - beyond a certain value it doesn't really matter, but there's no clear cutoff.
time_int_type max_meaningful_time_precision(vector3<fine_scalar> const& velocity) {
  return 16+std::max(std::max(std::abs(velocity(X)),std::abs(velocity(Y))),std::abs(velocity(Z))) * 16 / velocity_scale_factor;
}


struct get_first_moment_of_intersection_results {
  optional_time time;
  vector3<fine_scalar> normal;
  operator bool()const { return time; }
};

get_first_moment_of_intersection_results get_first_moment_of_intersection(shape const* s1, shape const* s2, vector3<fine_scalar> s1_velocity, vector3<fine_scalar> s2_velocity, time_type s1_last_time_updated, time_type s2_last_time_updated) {
  // Standardize: s1 is the one whose position is up-to-date.
  if (s2_last_time_updated > s1_last_time_updated) {
    return get_first_moment_of_intersection(s2, s1, s2_velocity, s1_velocity, s2_last_time_updated, s1_last_time_updated);
  }

  const auto relative_velocity = s1_velocity - s2_velocity;
  get_first_moment_of_intersection_results results;
  
  // We currently don't care too much about a bit of rounding error.
  shape s2t(*s2);
  s2t.translate(movement_delta_from_start_to(-relative_velocity, s1_last_time_updated - s2_last_time_updated));

  // Hack? - only polyhedra and bboxes work
  // Hack - Pointlessly wasting processor time to make this code simpler
  std::vector<convex_polyhedron> p1s(s1->get_polyhedra().begin(), s1->get_polyhedra().end());
  std::vector<convex_polyhedron> p2s(s2t.get_polyhedra().begin(), s2t.get_polyhedra().end());
  for (auto bb : s1->get_boxes()) {
    p1s.push_back(convex_polyhedron(bb));
  }
  for (auto bb : s2t.get_boxes()) {
    p2s.push_back(convex_polyhedron(bb));
  }
  for (auto const& p1 : p1s) {
    for (auto const& p2 : p2s) {
      auto coll_info = when_do_polyhedra_intersect(p1,p2,relative_velocity / velocity_scale_factor);
      if ((coll_info.is_anywhere) && (coll_info.max >= polygon_rational_type(0)) && (coll_info.min <= (time_type(1) - s1_last_time_updated))) {
        if ((coll_info.min >= polygon_rational_type(0)) || (coll_info.arbitrary_plane_of_closest_exclusion.normal.dot<fine_scalar>(relative_velocity) > 0)) {
          if (!results.time || (coll_info.min < *results.time)) {
            results.time = ((coll_info.min > polygon_rational_type(0)) ? coll_info.min : polygon_rational_type(0));
            // TODO: Make it so that it prefers axis-aligned normals again in order to avoid
            // hitting the sides of elements of flat surfaces, or solve that problem in another way?
            results.normal = coll_info.arbitrary_plane_hit_first.normal;
          }
        }
      }
    }
  }
  if (results.time) {
    //std::cerr << *results.time << "...\n";
    *results.time += s1_last_time_updated;
    *results.time = round_time_downwards(*results.time, max_meaningful_time_precision(relative_velocity));
    if (*results.time < s1_last_time_updated) *results.time = s1_last_time_updated;
    //std::cerr << *results.time << ", "<< s1_last_time_updated << "\n";
  }
  return results;
}


struct collision_info {
  collision_info(time_type t,time_type ct,vector3<fine_scalar> n,object_or_tile_identifier o1,int v1,object_or_tile_identifier o2,int v2):
    time(t),computation_time(ct),normal(n),oid1(o1),validation1(v1),oid2(o2),validation2(v2){}
  time_type time;
  time_type computation_time;
  vector3<fine_scalar> normal; // to the plane of collision
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
inline std::ostream& operator<<(std::ostream& os, moving_object_info const& moi) {
  return os << "moving_object_info:{"
  << moi.invalidation_counter
  << "; " << moi.last_time_updated << '}';
}

// Move the object physically, and update its caches.
// NOTE: This does NOT change its entry in the world's collision detector!
void update_object_to_time(boost::shared_ptr<mobile_object>& obj, shape& personal_space_shape, shape& detail_shape, moving_object_info& inf, time_type time) {
  caller_error_if(time < inf.last_time_updated, "You can't go back in time!");
  if (time > inf.last_time_updated) {
    vector3<fine_scalar> delta = movement_delta_from_start_to(obj->velocity(), time - inf.last_time_updated);
    personal_space_shape.translate(delta);
    detail_shape.translate(delta);
    //std::cerr << "(this message may be out-of-date/misleading) Moving object by " << delta << "; ignoring updated already " << movement_delta_from_start_to(obj->velocity(), time) << "; full delta " << movement_delta_from_start_to(obj->velocity(), time_type(1)) << "\n";
    ++inf.invalidation_counter;
    inf.last_time_updated = time;
  }
}
void update_object_for_whole_frame(boost::shared_ptr<mobile_object>& obj, shape& personal_space_shape, shape& detail_shape) {
  vector3<fine_scalar> delta = movement_delta_from_start_to(obj->velocity(), time_type(1));
  personal_space_shape.translate(delta);
  detail_shape.translate(delta);
    //std::cerr << "!Moving object by " << delta << "\n";
}

void collect_collisions(time_type min_time, bool erase_old_sweep, object_identifier id, boost::shared_ptr<mobile_object> const& obj, shape const* personal_space_shape, world &w, objects_collision_detector& sweep_box_cd, std::priority_queue<collision_info, std::vector<collision_info>>& anticipated_collisions, std::unordered_map<object_or_tile_identifier, moving_object_info>& objects_info, objects_map<mobile_object>::type const& moving_objects) {
  if (erase_old_sweep) sweep_box_cd.erase(id);
  
  bounding_box sweep_bounds = personal_space_shape->bounds();
  // Round up, so that our sweep bounds contains all area we might reach.
  auto delta = movement_delta_rounding_up(obj->velocity(), time_type(1) - min_time);
  if (delta != vector3<fine_scalar>(0,0,0)) {
    sweep_bounds.translate(delta);
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
  /*std::cerr << "Warning: Doing paranoid checks, remove this;\n";
  for (auto const& p1 : moving_objects) {
    object_identifier other_id = p1.first;*/
    if (other_id != id) {
        //std::cerr << "Considering " << id << ", " << other_id << "\n";
      shape const* opss = find_as_pointer(w.get_object_personal_space_shapes(), other_id);
      assert(opss);
      vector3<fine_scalar> other_velocity(0,0,0);
      if (boost::shared_ptr<mobile_object> const* other_obj = find_as_pointer(moving_objects, other_id)) {
        other_velocity = (*other_obj)->velocity();
        //std::cerr << "Other velocity is relevant." << obj->velocity() << (*other_obj)->velocity();
      }
      const auto i1 = objects_info.find(id);
      const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
      assert(ltu1 == min_time); // TODO just use this instead and not pass the min_time argument?
      const auto i2 = objects_info.find(other_id);
      const time_type ltu2 = (i2 == objects_info.end()) ? time_type(0) : i2->second.last_time_updated;
      if (auto result = get_first_moment_of_intersection(personal_space_shape, opss, obj->velocity(), other_velocity, ltu1, ltu2)) {
        assert(result.time);
        assert(*result.time >= min_time);
        anticipated_collisions.push(collision_info(
          *result.time, min_time, result.normal,
          // operator[] creates the entry if needed
          object_or_tile_identifier(id), objects_info[id].invalidation_counter,
          object_or_tile_identifier(other_id), objects_info[other_id].invalidation_counter));
        //std::cerr << "Added " << id << ", " << other_id << "\n";
      }
    }
  }
  
  vector<tile_location> tiles_this_could_collide_with;
  w.get_tiles_exposed_to_collision_within(tiles_this_could_collide_with,
      get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(sweep_bounds));
  for (auto const& loc : tiles_this_could_collide_with) {
      //std::cerr << "Considering " << id << ", " << loc << "\n";
    shape t = tile_shape(loc.coords());
    const auto i1 = objects_info.find(id);
    const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
    assert(ltu1 == min_time); // TODO just use this instead and not pass the min_time argument?
    if (auto result = get_first_moment_of_intersection(personal_space_shape, &t, obj->velocity(), vector3<fine_scalar>(0,0,0), ltu1, time_type(0))) {
      assert(result.time);
      assert(*result.time >= min_time);
      anticipated_collisions.push(collision_info(
        *result.time, min_time, result.normal,
        // operator[] creates the entry if needed
        object_or_tile_identifier(id), objects_info[id].invalidation_counter,
        object_or_tile_identifier(loc), objects_info[loc].invalidation_counter));
      //std::cerr << "Added " << id << ", " << loc << "\n";
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
      const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
      const auto i2 = objects_info.find(p2.first);
      const time_type ltu2 = (i2 == objects_info.end()) ? time_type(0) : i2->second.last_time_updated;
      if (ltu1 > ltu2) {
        s2.translate(movement_delta_from_start_to(p2.second->velocity(), ltu1 - ltu2));
      }
        if (ltu2 > ltu1) {
        s1.translate(movement_delta_from_start_to(p1.second->velocity(), ltu2 - ltu1));
      }
        
        //std::cerr << p1.second->velocity() << ":" << ltu1 << ", " << p2.second->velocity() << ":" << ltu2 << "\n";
      }
      else {
      //std::cerr << p1.second->velocity() << ", " << p2.second->velocity() << "\n";
      }
        assert(!s1.intersects(s2));
      }
    }
  }
}

void update_moving_objects_impl(
   world                            & w,
   objects_map<object>       ::type & objects,
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
  
  time_type last_time(0);
  int collisions_at_last_time = 0;
      //std::cerr << "poopPOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOPpoop!\n";
  while(!anticipated_collisions.empty()) {
    //std::cerr << "..." << anticipated_collisions.size() << "?\n";
    randomized_vector<collision_info> now_collisions;
    do {
      now_collisions.insert(anticipated_collisions.top(), w.get_rng());
      anticipated_collisions.pop();
    }
    while ((!anticipated_collisions.empty()) && (anticipated_collisions.top().time == now_collisions.begin()->time));
    
    //assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
    
    //std::cerr << "..." << now_collisions.size() << "!\n";
    for (collision_info collision : now_collisions) {
      //std::cerr << "Wham!\n";
      const auto inf1_iter = objects_info.find(collision.oid1);
      assert(inf1_iter != objects_info.end());
      moving_object_info& inf1 = inf1_iter->second;
      if (collision.validation1 == inf1.invalidation_counter){
        const auto inf2_iter = objects_info.find(collision.oid2);
        assert(inf2_iter != objects_info.end());
        moving_object_info& inf2 = inf2_iter->second;
        if (collision.validation2 == inf2.invalidation_counter){
          bool failsafe_mode = false;
          if (collision.time == last_time) {
            ++collisions_at_last_time;
            if (collisions_at_last_time > 100) {
              std::cerr << "FAILSAFE: There were 100 collisions at the same moment. Zeroing the velocities of everything...\n";
              failsafe_mode = true;
            }
          }
          else {
            caller_error_if(collision.time < last_time, "You can't go back in time!");
            last_time = collision.time;
            collisions_at_last_time = 1;
          }
          //std::cerr << "BLAM!\n";
          // oid1 is always an object identifier
          object_identifier const* oid1p = collision.oid1.get_object_identifier(); assert(oid1p);
          object_identifier const& oid1 = *oid1p;
          boost::shared_ptr<mobile_object>* obj1p = find_as_pointer(moving_objects, oid1); assert(obj1p);
          boost::shared_ptr<mobile_object>& obj1 = *obj1p;
          shape* o1pss = find_as_pointer(personal_space_shapes, oid1); assert(o1pss);
          shape* o1ds = find_as_pointer(detail_shapes, oid1); assert(o1ds);

          bool o2_is_immovable_obstruction = false;

          // TODO: collapse this duplicate code (between the object-tile case and the object-object case)
          if (tile_location const* locp = collision.oid2.get_tile_location()) {
            if (is_water(locp->stuff_at().contents())) {
              const fine_scalar current_speed = obj1->velocity().magnitude_within_32_bits();
              if (current_speed > max_object_speed_through_water) {
                update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
                ++inf1.invalidation_counter; // required in case update_object_to_time did nothing
                obj1->velocity_ = obj1->velocity() * max_object_speed_through_water / current_speed;
                
                //assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
              }
            }
            else {
              o2_is_immovable_obstruction = true;
                
                //assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
              
            }
          }
          if (object_identifier const* oid2p = collision.oid2.get_object_identifier()) {
           object_identifier const& oid2 = *oid2p;
           boost::shared_ptr<object>* obj2p = find_as_pointer(objects, oid2); assert(obj2p);
           if (boost::shared_ptr<mobile_object> obj2 = boost::dynamic_pointer_cast<mobile_object>(*obj2p)) {
            shape* o2pss = find_as_pointer(personal_space_shapes, oid2); assert(o2pss);
            shape* o2ds = find_as_pointer(detail_shapes, oid2); assert(o2ds);
            
            // CRASH!!
            //std::cerr << oid1 << oid2;
            // This assertion is wrong:
            //assert (!o1pss->intersects(*o2pss));
            // Because one object can follow another closely, with the one in front known to be
            // not colliding with anything else until after the time the back one intersects
            // where the front one was.
            /*these are no longer needed OR WANTED
            if (inf1.last_time_updated < collision.computation_time)
              update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.computation_time);
            if (inf2.last_time_updated < collision.computation_time)
              update_object_to_time(obj2, *o2pss, *o2ds, inf2, collision.computation_time);
            assert (!o1pss->intersects(*o2pss) && true);*/
            update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
            ++inf1.invalidation_counter; // required in case update_object_to_time did nothing
            update_object_to_time(obj2, *o2pss, *o2ds, inf2, collision.time);
            ++inf2.invalidation_counter; // required in case update_object_to_time did nothing
            //assert (!o1pss->intersects(*o2pss));
            //assert_about_overlaps(moving_objects, personal_space_shapes,objects_info);
            
            auto relative_velocity = obj1->velocity() - obj2->velocity();
            auto veldiff_num = (collision.normal * relative_velocity.dot<fine_scalar>(collision.normal));
            auto veldiff_denom = collision.normal.dot<fine_scalar>(collision.normal);
            // Hack: To avoid getting locked in a nonzero velocity due to rounding error,
            // make sure to round down your eventual velocity!
            // Note: This changes each object's velocity by -2/3 of the total difference,
            // hence the relative velocity is changed by -4/3 of itself and they bounce a little.
            obj1->velocity_ = ((obj1->velocity_ * veldiff_denom * 3) - veldiff_num * 2) / (veldiff_denom * 3);
            obj2->velocity_ = ((obj2->velocity_ * veldiff_denom * 3) + veldiff_num * 2) / (veldiff_denom * 3);
            // Also push the objects away from each other a little if the surface isn't axis-aligned.
            // This is a bit of a hack and might violate conservation of energy.
            if (((collision.normal(X) != 0) + (collision.normal(Y) != 0) + (collision.normal(Z) != 0)) > 1) {
              obj1->velocity_ -= relative_velocity * 4 / relative_velocity.magnitude_within_32_bits();
              obj2->velocity_ += relative_velocity * 4 / relative_velocity.magnitude_within_32_bits();
            }
            
            //obj1->velocity_ = (obj1->velocity() + obj2->velocity()) / 2;
            //obj2->velocity_ = obj1->velocity();
            
            if (failsafe_mode) obj1->velocity_ = vector3<fine_scalar>(0,0,0);
            if (failsafe_mode) obj2->velocity_ = vector3<fine_scalar>(0,0,0);
          
            // If we updated anything about the objects, they might have new collisions.
            // If we *didn't*, they won't repeat the same collision again, since we have
            //     deleted it and not replaced it.
            // Note: Duplicate comment with the below
            if (inf2.invalidation_counter > collision.validation2) {
              collect_collisions(collision.time, true, oid2, obj2, o2pss, w, sweep_box_cd, anticipated_collisions, objects_info, moving_objects);
            }
           }
           else {
             o2_is_immovable_obstruction = true;
           }
          }

          if (o2_is_immovable_obstruction) {
              //std::cerr << inf1.last_time_updated;
              //std::cerr << collision.time;
              update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
              //std::cerr << "!!!" << inf1.last_time_updated << "\n";
              const auto old_vel = obj1->velocity();
              auto vel_change = (collision.normal * obj1->velocity().dot<fine_scalar>(collision.normal));
              if (vel_change != vector3<fine_scalar>(0,0,0)) {
                fine_scalar vel_change_denom = collision.normal.dot<fine_scalar>(collision.normal);
                // Hack: To avoid getting locked in a nonzero velocity due to rounding error,
                // make sure to round down your eventual velocity!
                //std::cerr << obj1->velocity();
                obj1->velocity_ = ((obj1->velocity() * vel_change_denom) - vel_change) / vel_change_denom;
                //std::cerr << vel_change << vel_change_denom << obj1->velocity() << "\n";
              }
              // Also push the objects away from each other a little if the surface isn't axis-aligned.
              // This is a bit of a hack and might violate conservation of energy.
              if (((collision.normal(X) != 0) + (collision.normal(Y) != 0) + (collision.normal(Z) != 0)) > 1) {
                obj1->velocity_ -= old_vel * 4 / old_vel.magnitude_within_32_bits();
              }
              if (failsafe_mode) obj1->velocity_ = vector3<fine_scalar>(0,0,0);
              if (obj1->velocity() != old_vel) ++inf1.invalidation_counter;
              else {
                std::cerr << "Warning: No velocity change on collision. This can potentially cause overlaps.\n";
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
  
  // assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
  //std::cerr << "Excellently.\n";
  for (auto const& p : moving_objects) {
    auto const& oid = p.first;
    boost::shared_ptr<mobile_object>* objp = find_as_pointer(moving_objects, oid); assert(objp);
    boost::shared_ptr<mobile_object>& obj = *objp;
    shape* opss = find_as_pointer(personal_space_shapes, oid); assert(opss);
    shape* ods = find_as_pointer(detail_shapes, oid); assert(ods);
    auto inf = objects_info.find(object_or_tile_identifier(oid));
    if (inf == objects_info.end()) {
      update_object_for_whole_frame(obj, *opss, *ods);
      //std::cerr << "Minimally so.\n";
    }
    else {
      update_object_to_time(obj, *opss, *ods, inf->second, time_type(1));
      //std::cerr << "Quite so.\n";
    }
    
    // Update the collision detector entries
    bounding_box new_bounds = opss->bounds();
    new_bounds.combine_with(ods->bounds());
    objects_exposed_to_collision.erase(oid);
    objects_exposed_to_collision.insert(oid, new_bounds);
  }
  //assert_about_overlaps(moving_objects, personal_space_shapes, objects_info, false);
}

} /* end anonymous namespace */

void world::update_moving_objects() {
  update_moving_objects_impl(*this, objects_, moving_objects_, object_personal_space_shapes_, object_detail_shapes_, objects_exposed_to_collision_);
}


