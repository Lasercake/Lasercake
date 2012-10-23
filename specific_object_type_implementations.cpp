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

#include "data_structures/bbox_collision_detector_iteration.hpp"
#include "tile_iteration.hpp"
#include "tile_physics.hpp"

namespace /* anonymous */ {

typedef polygon_rational_type rational;

namespace laserbeam {
  static const uint64_t too_large = (1ULL << 32) - 1;
  inline bool bbox_is_too_large(world_collision_detector::bounding_box const& bbox) {
    return (bbox.size_minus_one(X) >= too_large || bbox.size_minus_one(Y) >= too_large || bbox.size_minus_one(Z) >= too_large);
  }
  inline bool bbox_is_too_large(power_of_two_bounding_cube<3, tile_coordinate> const& bbox) {
    return (bbox.size_minus_one(X) >= tile_coordinate(too_large/get_primitive_int(tile_width))
         || bbox.size_minus_one(Y) >= tile_coordinate(too_large/get_primitive_int(tile_width))
         || bbox.size_minus_one(Z) >= tile_coordinate(too_large/get_primitive_int(tile_height)));
  }
  struct beam_first_contact_finder {
    beam_first_contact_finder(world const& w, line_segment beam, object_or_tile_identifier ignore):w_(w),beam_(beam) {ignores_.insert(ignore);}
    typedef rational cost_type;
    typedef optional_rational result_type;
    result_type min_cost(world_collision_detector::bounding_box const& bbox) const {
      // hack - avoid overflow - don't try to filter out too-large regions
      if(bbox_is_too_large(bbox)) return rational(0);
      return get_first_intersection(beam_, bbox);
    }
    result_type cost(object_identifier id, world_collision_detector::bounding_box const& bbox) const {
      // hack - avoid overflow - effect: incredibly large objects can't be hit by lasers
      if(bbox_is_too_large(bbox)) return result_type();
      if(ignores_.find(id) != ignores_.end()) return result_type();
      return get_first_intersection(beam_, w_.get_detail_shape_of_object_or_tile(id));
    }
  private:
    world const& w_;
    line_segment beam_;
    unordered_set<object_or_tile_identifier> ignores_;
  };

  struct beam_first_contact_finder_tile {
    beam_first_contact_finder_tile(octant_number octant, line_segment beam)
      : octant_(octant), beam_(beam), latest_distance_(0), found_tile_() {}

    tribool look_here(power_of_two_bounding_cube<3, tile_coordinate> const& bbox) {
      // hack - avoid overflow - don't try to filter out too-large regions
      if(bbox_is_too_large(bbox)) {
        return indeterminate;
      }
      const tile_bounding_box tile_bbox = cube_bbox_to_tile_bounding_box(bbox);
      const optional_rational new_distance =
        get_first_intersection(beam_, convert_to_fine_units(tile_bbox));
      if(new_distance) {
        assert_if_ASSERT_EVERYTHING(*new_distance >= latest_distance_);
        latest_distance_ = *new_distance;
        return indeterminate;
      }
      else {
        return false;
      }
    }

    bool collidable_tile(tile_location const& loc) {
      found_tile_ = loc;
      return false;
    }

    octant_number octant()const { return octant_; }

    octant_number octant_;
    line_segment beam_;
    rational latest_distance_;
    boost::optional<tile_location> found_tile_;
  };
}

// Returns object_or_tile_identifier() if nothing was hit.
// Has no side_effects unless add_laser_sfx==true.
object_or_tile_identifier laser_find(
    world& w,
    object_identifier dont_hit_this,
    vector3<fine_scalar> source,
    vector3<fine_scalar> beam_vector,
    bool add_laser_sfx
) {
  const line_segment beam(source, source + beam_vector);
  const laserbeam::beam_first_contact_finder finder(w, beam, dont_hit_this);
  laserbeam::beam_first_contact_finder_tile finder_tile(vector_octant(beam_vector), beam);
  w.visit_collidable_tiles(finder_tile);
  auto hit_object =  w.objects_exposed_to_collision().find_least(finder);
  const rational farther_away_than_possible_intercept_point = rational(2); //the valid max is 1
  rational best_intercept_point = farther_away_than_possible_intercept_point;
  object_or_tile_identifier hit_thing;
  if(finder_tile.found_tile_) {
    hit_thing = *finder_tile.found_tile_;
    best_intercept_point = finder_tile.latest_distance_;
  }
  if(hit_object && hit_object->cost <= best_intercept_point) {
    hit_thing = hit_object->object;
    best_intercept_point = hit_object->cost;
  }
  if(add_laser_sfx) {
    if(hit_thing != object_or_tile_identifier()) {
      // TODO do I have to worry about overflow?
      w.add_laser_sfx(source, multiply_rational_into(beam_vector, best_intercept_point));
    }
    else {
      w.add_laser_sfx(source, beam_vector);
    }
  }
  return hit_thing;
}

void fire_standard_laser(world& w, object_identifier my_id, vector3<fine_scalar> location, vector3<fine_scalar> facing) {
  const vector3<fine_scalar> beam_vector = facing * tile_width * 2 / facing.magnitude_within_32_bits() * 50;
  const object_or_tile_identifier hit_thing = laser_find(w, my_id, location, beam_vector, true);
  if(tile_location const* locp = hit_thing.get_tile_location()) {
    if (locp->stuff_at().contents() == ROCK) {
      w.replace_substance(*locp, ROCK, RUBBLE);
    }
  }
}

const int robot_max_carrying_capacity = 4;

// Our robot "location" gets out of date relative to the personal_space_shape
// that the world defines us with (and changes due to velocity),
// so update it.
void update_location(vector3<fine_scalar>& location_, world& w, object_identifier id) {
  const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(id)->second.bounds();
  const vector3<fine_scalar> middle = (shape_bounds.min() + shape_bounds.max()) / 2; //hmm, rounding.
  location_ = middle;
}

void float_above_ground(vector3<fine_scalar>& velocity_, world& w, object_identifier id) {
  const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(id)->second.bounds();
  const vector3<fine_scalar> middle = (shape_bounds.min() + shape_bounds.max()) / 2; //hmm, rounding.
  const vector3<fine_scalar> bottom_middle(middle(X), middle(Y), shape_bounds.min(Z));
  const auto tiles_containing_bottom_middle = get_all_containing_tile_coordinates(bottom_middle);
  bool ground_below = false;
  for(vector3<tile_coordinate> tile_containing_bottom_middle : tiles_containing_bottom_middle) {
    if(w.make_tile_location(tile_containing_bottom_middle, COMPLETELY_IMAGINARY)
        .get_neighbor<zminus>(CONTENTS_ONLY)
        .stuff_at().contents()
      != AIR) {
      ground_below = true;
    }
  }
  if (ground_below) {
    // goal: decay towards levitating...
    fine_scalar target_height = (lower_bound_in_fine_units(get_max_containing_tile_coordinate(shape_bounds.min(Z), Z), Z) + tile_height * 5 / 4);
    fine_scalar deficiency = target_height - shape_bounds.min(Z);
    fine_scalar target_vel = gravity_acceleration_magnitude + deficiency * velocity_scale_factor / 8;
    if (velocity_.z < target_vel) {
      velocity_.z = std::min(velocity_.z + gravity_acceleration_magnitude * 5, target_vel);
    }
  }
}

} /* end anonymous namespace */

shape robot::get_initial_personal_space_shape()const {
  /*return shape(convex_polyhedron(bounding_box::min_and_max(
    location_ - vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10),
    location_ + vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10)
  )));*/
  std::vector<vector3<fine_scalar>> verts;
  verts.push_back(location_ + vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<fine_scalar>(tile_width * 3 / 10, -tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<fine_scalar>(-tile_width * 3 / 10, -tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<fine_scalar>(-tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<fine_scalar>(0, 0, -tile_width * 3 / 10));
  return shape(convex_polyhedron(verts));
}

shape robot::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}


std::string robot::player_instructions()const {
  static const std::string instructions =
    "x: go forward\n"
    //friction: implicit, we don't mention it, i guess.
    "0 (zero): jump\n" //hmm should we have a jump and have it do this
    "right: turn right\n"
    "left: turn left\n"
    "up: look up\n"
    "down: look down\n"
    "b: fire dual lasers\n"
    "m: make new auto digging robot, going towards the current facing\n"
    "        and leaving rubble at the current location\n"
    "(c, v: commented-out ability to pick up things, too bad.)\n"
    ;
  return instructions;
}

void robot::update(world& w, input_representation::input_news_t const& input_news, object_identifier my_id) {
  update_location(location_, w, my_id);
  //float_above_ground(velocity_, w, my_id);
    
  velocity_.x = divide_rounding_towards_zero(velocity_.x, 2);
  velocity_.y = divide_rounding_towards_zero(velocity_.y, 2);
  const fine_scalar xymag = i64sqrt(facing_.x*facing_.x + facing_.y*facing_.y);
  if (input_news.is_currently_pressed("x")) {
    velocity_.x = facing_.x * tile_width * velocity_scale_factor / 8 / xymag;
    velocity_.y = facing_.y * tile_width * velocity_scale_factor / 8 / xymag;
  }
  if (input_news.is_currently_pressed("0")) {
    velocity_.z = tile_width * velocity_scale_factor / 8;
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
    #if 0
    beam_first_contact_finder finder(w, line_segment(location_, location_ + beam_delta), my_id);
    if(auto hit = w.get_things_exposed_to_collision().find_least(finder)) {
      // TODO do I have to worry about overflow?
      w.add_laser_sfx(location_, multiply_rational_into(beam_delta, hit->cost));
      if(tile_location const* locp = hit->object.get_tile_location()) {
        if (input_news.is_currently_pressed("c") && (carrying_ < robot_max_carrying_capacity) && (locp->stuff_at().contents() == ROCK || locp->stuff_at().contents() == RUBBLE)) {
          ++carrying_;
          w.replace_substance(*locp, locp->stuff_at().contents(), AIR);
        }
        /*if (carrying && locp->stuff_at().contents() == ROCK) {
          w.replace_substance(*locp, ROCK, RUBBLE);
        }*/
      }
    }
    else
    #endif
    {
      w.add_laser_sfx(location_, beam_delta);
      if (input_news.is_currently_pressed("v") && (carrying_ > 0)) {
        --carrying_;
        const tile_location target_loc = w.make_tile_location(get_random_containing_tile_coordinates(location_ + beam_delta, w.get_rng()), FULL_REALIZATION);
        w.replace_substance(target_loc, AIR, RUBBLE);
      }
    }
  }
  if (input_news.is_currently_pressed("b")) {
    const vector3<fine_scalar> offset(-facing_.y / 4, facing_.x / 4, 0);
    const vector3<fine_scalar> beam_vector = facing_ * tile_width * 2 / facing_.magnitude_within_32_bits() * 50;
    {
      const object_or_tile_identifier hit_thing = laser_find(w, my_id, location_ + offset, beam_vector, true);
      if(tile_location const* locp = hit_thing.get_tile_location()) {
        if (locp->stuff_at().contents() == ROCK) {
          w.replace_substance(*locp, ROCK, RUBBLE);
        }
      }
      if(object_identifier const* oidp = hit_thing.get_object_identifier()) {
        if (shared_ptr<object>* obj = w.get_object(*oidp)) {
          if (shared_ptr<mobile_object> mobj = boost::dynamic_pointer_cast<mobile_object>(*obj)) {
            mobj->velocity_ -= facing_ * velocity_scale_factor / 20;
          }
        }
      }
    }
    {
      const object_or_tile_identifier hit_thing = laser_find(w, my_id, location_ - offset, beam_vector, true);
      if(tile_location const* locp = hit_thing.get_tile_location()) {
        if (locp->stuff_at().contents() == ROCK) {
          w.replace_substance(*locp, ROCK, RUBBLE);
        }
      }
      if(object_identifier const* oidp = hit_thing.get_object_identifier()) {
        if (shared_ptr<object>* obj = w.get_object(*oidp)) {
          if (shared_ptr<mobile_object> mobj = boost::dynamic_pointer_cast<mobile_object>(*obj)) {
            mobj->velocity_ -= facing_ * velocity_scale_factor;
          }
        }
      }
    }
    //fire_standard_laser(w, my_id, location_ + offset, facing_);
    //fire_standard_laser(w, my_id, location_ - offset, facing_);
  }
  if (input_news.num_times_pressed("m")) {
    const shared_ptr<autorobot> aur (new autorobot(location_ + facing_ * 2, facing_));
    w.try_create_object(aur);
  }
}


shape laser_emitter::get_initial_personal_space_shape()const {
  return shape(convex_polyhedron(bounding_box::min_and_max(
    location_ - vector3<fine_scalar>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10),
    location_ + vector3<fine_scalar>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10)
  )));
}

shape laser_emitter::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}


void laser_emitter::update(world& w, input_representation::input_news_t const&, object_identifier my_id) {
  update_location(location_, w, my_id);
  
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




shape autorobot::get_initial_personal_space_shape()const {
  return shape(convex_polyhedron(bounding_box::min_and_max(
    location_ - vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10),
    location_ + vector3<fine_scalar>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10)
  )));
}

shape autorobot::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

//hack impl.
bool once_a_second(world& w, int num = 1, int denom = 1) {
  return ((w.game_time_elapsed() * num / denom) % time_units_per_second) == 0;
}

autorobot::autorobot(vector3<fine_scalar> location, vector3<fine_scalar> facing):location_(location),initial_location_(((location / tile_width) * tile_width) + vector3<fine_scalar>(tile_width / 2, tile_width / 2, 0)),facing_(facing),carrying_(0){
  fine_scalar bigger_dir;
  if (std::abs(facing_.x) > std::abs(facing_.y)) {
    bigger_dir = facing.x;
    facing_.y = 0;
    facing_.x = facing_.x > 0 ? tile_width : -tile_width;
  }
  else {
    bigger_dir = facing.y;
    facing_.x = 0;
    facing_.y = facing_.y > 0 ? tile_width : -tile_width;
  }
  if (std::abs(bigger_dir) < std::abs(facing.z) * 8) {
    facing_.z = facing_.z > 0 ? tile_height : -tile_height;
  }
  else facing_.z = 0;
}

void autorobot::update(world& w, input_representation::input_news_t const&, object_identifier my_id) {
  auto& rng = w.get_rng();
  update_location(location_, w, my_id);
  float_above_ground(velocity_, w, my_id);
  tile_location i_am_in = w.make_tile_location(get_random_containing_tile_coordinates(location_, rng), FULL_REALIZATION);
  std::array<tile_location, num_cardinal_directions> my_neighbors = get_all_neighbors(i_am_in, FULL_REALIZATION);

  /*int levitate = 0;
  int horiz[] = { xplus, xminus, yplus, yminus };
  for(auto dir : horiz) {
    levitate += (my_neighbors[dir].stuff_at().contents() != AIR);
  }
  velocity_.z += levitate * 1000;
  //if(my_neighbors[xplus].stuff_at().contents() != AIR) velocity_.x += velocity_scale_factor * 20;

  //velocity_ = velocity_ - (velocity_ * (velocity_.magnitude_within_32_bits() / 1000) / 1000000) 

  if(levitate == 0 && my_neighbors[zminus].stuff_at().contents() != AIR) {
    if(once_a_second(w, 1, 2)) {
      const boost::random::uniform_int_distribution<get_primitive_int_type<fine_scalar>::type> random_vel(-4000, 4000);
      velocity_.x /= 2;
      velocity_.y /= 2;
      velocity_.x += random_vel(rng);
      velocity_.y += random_vel(rng);
    }
  }*/
  //facing_ = facing_ * tile_width / facing_.magnitude_within_32_bits();
  
  auto direction_home = initial_location_ - location_;
  auto mag = to_signed_type(i64sqrt(direction_home.x*direction_home.x + direction_home.y*direction_home.y));
  
  if (carrying_ < 4) {
    velocity_.x = facing_.x * velocity_scale_factor / 8;
    velocity_.y = facing_.y * velocity_scale_factor / 8;
    
    if (facing_.z >= 0 || mag >= 10*tile_width) {
    const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(my_id)->second.bounds();
    const vector3<fine_scalar> middle = (shape_bounds.min() + shape_bounds.max()) / 2; //hmm, rounding.
    const vector3<fine_scalar> top_middle(middle(X), middle(Y), shape_bounds.max(Z));
    /*const vector3<fine_scalar> top_middle(
      boost::random::uniform_int_distribution<get_primitive_int_type<fine_scalar>::type>(shape_bounds.min(X), shape_bounds.max(X))(rng),
      boost::random::uniform_int_distribution<get_primitive_int_type<fine_scalar>::type>(shape_bounds.min(Y), shape_bounds.max(Y))(rng),
      shape_bounds.max(Z));*/
    const vector3<fine_scalar> beam_vector_1 = facing_;
    const object_or_tile_identifier hit1 = laser_find(w, my_id, top_middle, beam_vector_1, true);
    if (hit1 != object_or_tile_identifier()) {
      if (tile_location const* locp = hit1.get_tile_location()) {
        if ((locp->stuff_at().contents() == ROCK || locp->stuff_at().contents() == RUBBLE)) {
          w.replace_substance(*locp, locp->stuff_at().contents(), AIR);
          ++carrying_;
        }
      }
    }
    else {
      const vector3<fine_scalar> beam_vector_2(facing_.x / -4, facing_.y / -4, 1 - shape_bounds.size(Z));
      const object_or_tile_identifier hit2 = laser_find(w, my_id, top_middle + beam_vector_1, beam_vector_2, true);
      if (hit2 != object_or_tile_identifier()) {
        if (tile_location const* locp = hit2.get_tile_location()) {
          if ((locp->stuff_at().contents() == ROCK || locp->stuff_at().contents() == RUBBLE)) {
            w.replace_substance(*locp, locp->stuff_at().contents(), AIR);
            ++carrying_;
          }
        }
      }
    }
    }
  }
  else {
    if (mag != 0) {
      velocity_.x = direction_home.x * tile_width * velocity_scale_factor / 8 / mag;
      velocity_.y = direction_home.y * tile_width * velocity_scale_factor / 8 / mag;
    }
    if (mag <= tile_width) {
      for (auto loc : my_neighbors) {
        if (loc.stuff_at().contents() == AIR) {
          w.replace_substance(loc, AIR, RUBBLE);
          --carrying_;
        }
      }
    }
  }
}





