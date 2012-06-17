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

#include "specific_object_types.hpp"


namespace /* anonymous */ {

typedef non_normalized_rational<polygon_int_type> rational;
typedef std::pair<bool, rational> bool_and_rational;

struct beam_first_contact_finder : world_collision_detector::visitor {
  beam_first_contact_finder(world const& w, line_segment beam):w(w),beam(beam),best_intercept_point(false, rational(1)){}
  world const& w;
  line_segment beam;
  bool_and_rational best_intercept_point;
  unordered_set<object_or_tile_identifier> ignores;
  object_or_tile_identifier thing_hit;
  
  void handle_new_find(object_or_tile_identifier id) {
    if (ignores.find(id) != ignores.end()) return;
    
    // TODO : long beams, overflow?
    bool_and_rational result = w.get_detail_shape_of_object_or_tile(id).first_intersection(beam);
    if (result.first) {
      if (!best_intercept_point.first || result.second < best_intercept_point.second) {
        best_intercept_point = result;
        thing_hit = id;
      }
    }
  }
  bool should_be_considered__dynamic(bounding_box const& bb)const {
    // hack - avoid overflow
    const uint64_t too_large = (1ULL << 32) - 1;
    if (bb.size_minus_one(X) >= too_large || bb.size_minus_one(Y) >= too_large || bb.size_minus_one(Z) >= too_large) {
      // if there might be overflow, only do a bounds check
      return beam.bounds().overlaps(bb);
    }
    bool_and_rational result = shape(bb).first_intersection(beam);
    return result.first && (!best_intercept_point.first || result.second < best_intercept_point.second);
  }
  bool bbox_less_than(bounding_box const& bb1, bounding_box const& bb2)const {
    for (int dim = 0; dim < 3; ++dim) {
      if (beam.ends[0][dim] < beam.ends[1][dim]) {
        if (bb1.min(dim) < bb2.min(dim)) return true;
        if (bb1.min(dim) > bb2.min(dim)) return false;
      }
      if (beam.ends[0][dim] > beam.ends[1][dim]) {
        if (bb1.min(dim) > bb2.min(dim)) return true;
        if (bb1.min(dim) < bb2.min(dim)) return false;
      }
    }
    return false;
  }
};

void fire_standard_laser(world& w, object_identifier my_id, vector3<fine_scalar> location, vector3<fine_scalar> facing) {
  facing = facing * tile_width * 2 / facing.magnitude_within_32_bits();
  
  beam_first_contact_finder finder(w, line_segment(location, location + facing * 50));
  finder.ignores.insert(my_id);
  w.get_things_exposed_to_collision().search(&finder);
  
  if (finder.best_intercept_point.first) {
    // TODO do I have to worry about overflow?
    w.add_laser_sfx(location, facing * 50 * finder.best_intercept_point.second.numerator / finder.best_intercept_point.second.denominator);
    if(tile_location const* locp = finder.thing_hit.get_tile_location()) {
      if (locp->stuff_at().contents() == ROCK) {
        w.replace_substance(*locp, ROCK, RUBBLE);
      }
    }
  }
  else {
    w.add_laser_sfx(location, facing * 50);
  }
}

const int robot_max_carrying_capacity = 4;

} /* end anonymous namespace */

shape robot::get_initial_personal_space_shape()const {
  return shape(bounding_box(
    location_ - vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10),
    location_ + vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10)
  ));
}

shape robot::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

void robot::update(world& w, object_identifier my_id) {
  const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(my_id)->second.bounds();
  const vector3<fine_scalar> middle = (shape_bounds.min + shape_bounds.max) / 2;
  location_ = middle;
  const vector3<fine_scalar> bottom_middle(middle.x, middle.y, shape_bounds.min.z);
  const tile_location l = w.make_tile_location(get_containing_tile_coordinates(bottom_middle), CONTENTS_ONLY);
  const tile_location lminus = l.get_neighbor<zminus>(CONTENTS_ONLY);
  if (lminus.stuff_at().contents() != AIR) {
    // goal: decay towards levitating...
    fine_scalar target_height = (lower_bound_in_fine_units(l.coords().z, 2) + tile_height * 5 / 4);
    fine_scalar deficiency = target_height - shape_bounds.min.z;
    fine_scalar target_vel = gravity_acceleration_magnitude + deficiency * velocity_scale_factor / 8;
    if (velocity.z < target_vel) {
      velocity.z = std::min(velocity.z + gravity_acceleration_magnitude * 5, target_vel);
    }
  }
    
  input_representation::input_news_t const& input_news = w.input_news();
  velocity.x -= velocity.x / 2;
  velocity.y -= velocity.y / 2;
  const fine_scalar xymag = i64sqrt(facing_.x*facing_.x + facing_.y*facing_.y);
  if (input_news.is_currently_pressed("x")) {
    velocity.x = facing_.x * tile_width * velocity_scale_factor / 8 / xymag;
    velocity.y = facing_.y * tile_width * velocity_scale_factor / 8 / xymag;
  }
  if (input_news.is_currently_pressed("right")) {
    fine_scalar new_facing_x = facing_.x + facing_.y / 20;
    fine_scalar new_facing_y = facing_.y - facing_.x / 20;
    facing_.x = new_facing_x; facing_.y = new_facing_y;
  }
  if (input_news.is_currently_pressed("left")) {
    fine_scalar new_facing_x = facing_.x - facing_.y / 20;
    fine_scalar new_facing_y = facing_.y + facing_.x / 20;
    facing_.x = new_facing_x; facing_.y = new_facing_y;
  }
  if (input_news.is_currently_pressed("up") != input_news.is_currently_pressed("down")) {
    const fine_scalar which_way = (input_news.is_currently_pressed("up") ? 1 : -1);
    const fine_scalar new_xymag = xymag - (which_way * facing_.z / 20);
    if (new_xymag > tile_width / 8) {
      facing_.z += which_way * xymag / 20;
      facing_.y = facing_.y * new_xymag / xymag;
      facing_.x = facing_.x * new_xymag / xymag;
    }
  }
  facing_ = facing_ * tile_width / facing_.magnitude_within_32_bits();
  
  vector3<fine_scalar> beam_delta = facing_ * 3 / 2;
  
  if (input_news.is_currently_pressed("c") || input_news.is_currently_pressed("v")) {
    beam_first_contact_finder finder(w, line_segment(location_, location_ + beam_delta));
    finder.ignores.insert(my_id);
    w.get_things_exposed_to_collision().search(&finder);
    
    if (finder.best_intercept_point.first) {
      // TODO do I have to worry about overflow?
      w.add_laser_sfx(location_, beam_delta * finder.best_intercept_point.second.numerator / finder.best_intercept_point.second.denominator);
      if(tile_location const* locp = finder.thing_hit.get_tile_location()) {
        if (input_news.is_currently_pressed("c") && (carrying_ < robot_max_carrying_capacity) && (locp->stuff_at().contents() == ROCK || locp->stuff_at().contents() == RUBBLE)) {
          ++carrying_;
          w.replace_substance(*locp, locp->stuff_at().contents(), AIR);
        }
        /*if (carrying && locp->stuff_at().contents() == ROCK) {
          w.replace_substance(*locp, ROCK, RUBBLE);
        }*/
      }
    }
    else {
      w.add_laser_sfx(location_, beam_delta);
      if (input_news.is_currently_pressed("v") && (carrying_ > 0)) {
        --carrying_;
        const tile_location target_loc = w.make_tile_location(get_containing_tile_coordinates(location_ + beam_delta), FULL_REALIZATION);
        w.replace_substance(target_loc, AIR, RUBBLE);
      }
    }
  }
  if (input_news.is_currently_pressed("b")) {
    const vector3<fine_scalar> offset(-facing_.y / 4, facing_.x / 4, 0);
    fire_standard_laser(w, my_id, location_ + offset, facing_);
    fire_standard_laser(w, my_id, location_ - offset, facing_);
  }
}


shape laser_emitter::get_initial_personal_space_shape()const {
  return shape(bounding_box(
    location_ - vector3<fine_scalar>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10),
    location_ + vector3<fine_scalar>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10)
  ));
}

shape laser_emitter::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

void laser_emitter::update(world& w, object_identifier my_id) {
  const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(my_id)->second.bounds();
  const vector3<fine_scalar> middle = (shape_bounds.min + shape_bounds.max) / 2;
  
  location_ = middle;
  const boost::random::uniform_int_distribution<get_primitive_int_type<fine_scalar>::type> random_delta(-1023, 1023);
  for (int i = 0; i < 100; ++i) {
    do {
      facing_.x = random_delta(w.get_rng());
      facing_.y = random_delta(w.get_rng());
      facing_.z = random_delta(w.get_rng());
    } while (facing_.magnitude_within_32_bits_is_greater_than(1023) || facing_.magnitude_within_32_bits_is_less_than(512));

    fire_standard_laser(w, my_id, location_, facing_);
  }
}

