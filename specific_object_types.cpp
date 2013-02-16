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

namespace laserbeam {
  static const uint64_t too_large = (1ULL << 32) - 1; // HACK
  static const tile_coordinate too_many_tiles_wide(too_large / get_primitive_int(get(tile_width, fine_distance_units)));
  static const tile_coordinate too_many_tiles_high(too_large / get_primitive_int(get(tile_height, fine_distance_units)));
  inline bool bbox_is_too_large(world_collision_detector::bounding_box const& bbox) {
    return (bbox.size_minus_one(X) >= too_large
         || bbox.size_minus_one(Y) >= too_large
         || bbox.size_minus_one(Z) >= too_large);
  }
  inline bool bbox_is_too_large(power_of_two_bounding_cube<3, tile_coordinate> const& bbox) {
    return (bbox.size_minus_one(X) >= too_many_tiles_wide
         || bbox.size_minus_one(Y) >= too_many_tiles_wide
         || bbox.size_minus_one(Z) >= too_many_tiles_high);
  }
  struct beam_first_contact_finder {
    beam_first_contact_finder(world const& w, geom::line_segment beam, object_or_tile_identifier ignore)
      : w_(w), beam_(beam) {ignores_.insert(ignore);}
    typedef geom::dimensionless_rational cost_type;
    typedef geom::optional_dimensionless_rational result_type;
    result_type min_cost(world_collision_detector::bounding_box const& bbox) const {
      // hack - avoid overflow - don't try to filter out too-large regions
      if(bbox_is_too_large(bbox)) return cost_type(0);
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
    geom::line_segment beam_;
    unordered_set<object_or_tile_identifier> ignores_;
  };

  struct beam_first_contact_finder_tile {
    beam_first_contact_finder_tile(octant_number octant, geom::line_segment beam)
      : octant_(octant), beam_(beam), latest_distance_(0), found_tile_() {}

    tribool look_here(power_of_two_bounding_cube<3, tile_coordinate> const& bbox) {
      // hack - avoid overflow - don't try to filter out too-large regions
      if(bbox_is_too_large(bbox)) {
        return indeterminate;
      }
      const tile_bounding_box tile_bbox = cube_bbox_to_tile_bounding_box(bbox);
      const geom::optional_dimensionless_rational new_distance =
        geom::get_first_intersection(beam_, convert_to_fine_distance_units(tile_bbox));
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
    geom::line_segment beam_;
    geom::dimensionless_rational latest_distance_;
    boost::optional<tile_location> found_tile_;
  };
}

// Returns object_or_tile_identifier() if nothing was hit.
// Has no side_effects unless add_laser_sfx==true.
object_or_tile_identifier laser_find(
    world& w,
    object_identifier dont_hit_this,
    vector3<distance> source,
    vector3<distance> beam_vector,
    bool add_laser_sfx,
    vector3<distance>* result_beam_vector_ptr = nullptr
) {
  const geom::line_segment beam(source, source + beam_vector);
  const laserbeam::beam_first_contact_finder finder(w, beam, dont_hit_this);
  laserbeam::beam_first_contact_finder_tile finder_tile(vector_octant(beam_vector), beam);
  w.visit_collidable_tiles(finder_tile);
  auto hit_object =  w.objects_exposed_to_collision().find_least(finder);
  const geom::dimensionless_rational farther_away_than_possible_intercept_point(2); //the valid max is 1
  geom::dimensionless_rational best_intercept_point = farther_away_than_possible_intercept_point;
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
      vector3<distance> result_beam_vector = multiply_rational_into(beam_vector, best_intercept_point);
      w.add_laser_sfx(source, result_beam_vector);
      if (result_beam_vector_ptr) *result_beam_vector_ptr = result_beam_vector;
    }
    else {
      w.add_laser_sfx(source, beam_vector);
      if (result_beam_vector_ptr) *result_beam_vector_ptr = beam_vector;
    }
  }
  return hit_thing;
}

void fire_standard_laser(world& w, object_identifier my_id, vector3<distance> location, vector3<distance> facing) {
  const vector3<distance> beam_vector = facing * tile_width * 2 / facing.magnitude_within_32_bits() * 50;
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
void update_location(vector3<distance>& location_, world& w, object_identifier id) {
  const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(id)->second.bounds();
  const vector3<distance> middle = (shape_bounds.min() + shape_bounds.max()) / 2; //hmm, rounding.
  location_ = middle;
}

void float_above_ground(vector3<velocity1d>& velocity_, world& w, object_identifier id) {
  const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(id)->second.bounds();
  const vector3<distance> middle = (shape_bounds.min() + shape_bounds.max()) / 2; //hmm, rounding.
  const vector3<distance> bottom_middle(middle(X), middle(Y), shape_bounds.min(Z));
  const auto tiles_containing_bottom_middle = get_all_containing_tile_coordinates(bottom_middle);
  distance target_height = shape_bounds.min(Z);
  for(vector3<tile_coordinate> tile_containing_bottom_middle : tiles_containing_bottom_middle) {
    tile_location loc_below_bottom_middle =
      w.make_tile_location(tile_containing_bottom_middle, COMPLETELY_IMAGINARY).get_neighbor<zminus>(CONTENTS_ONLY);
    if (loc_below_bottom_middle.stuff_at().contents() != AIR) {
      target_height = lower_bound_in_fine_distance_units(loc_below_bottom_middle.coords()(Z), Z) + tile_height * 9 / 4;
    }
    else {
      loc_below_bottom_middle = loc_below_bottom_middle.get_neighbor<zminus>(CONTENTS_ONLY);
      if (loc_below_bottom_middle.stuff_at().contents() != AIR) {
        target_height = lower_bound_in_fine_distance_units(loc_below_bottom_middle.coords()(Z), Z) + tile_height * 9 / 4;
      }
    }
  }
  distance deficiency = target_height - shape_bounds.min(Z);
  if (deficiency > 0) {
    // goal: decay towards levitating...
    velocity1d target_vel = (deficiency * 15 / 4) / seconds
      // Hack: Add in the one-frame acceleration due to gravity, so that it meets an equilibrium at the specified height.
      // TODO: Figure out a nicer way for the object-motion system to interact with the autonomous-object system.
      + (gravity_acceleration_magnitude / identity(fixed_frame_lengths / seconds) * fixed_frame_lengths);
    if (velocity_.z < target_vel) {
      velocity_.z =
        std::min(velocity_.z +
                    gravity_acceleration_magnitude * 5
                      / identity(fixed_frame_lengths / seconds) * fixed_frame_lengths,
                 target_vel);
    }
  }
}

} /* end anonymous namespace */

shape robot::get_initial_personal_space_shape()const {
  /*return shape(geom::convex_polyhedron(bounding_box::min_and_max(
    location_ - vector3<distance>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10),
    location_ + vector3<distance>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10)
  )));*/
  std::vector<vector3<distance>> verts;
  verts.push_back(location_ + vector3<distance>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(tile_width * 3 / 10, -tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(-tile_width * 3 / 10, -tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(-tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(0, 0, -tile_width * 3 / 10));
  return shape(geom::convex_polyhedron(verts));
}

shape robot::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}


std::string robot::player_instructions()const {
  static const std::string instructions =
    "5, s: go forward; "
    //friction: implicit, we don't mention it, i guess.
    "left, a: turn left; "
    "right, d: turn right\n"
    "up, w: look up; "
    "down, x: look down\n"
    "space: jump\n" //hmm should we have a jump and have it do this
    "l: fire dual lasers\n"
    "m: make digging robot that goes towards the current facing"
    " and leaves rubble at the current location\n"
    "p: create solar panel\n"
    "o: create refinery\n"
    "r: fire rockets\n"
    //"(c, v: commented-out ability to pick up things, too bad.)\n"
    ;
  return instructions;
}

void robot::update(world& w, input_representation::input_news_t const& input_news, object_identifier my_id) {
  update_location(location_, w, my_id);
  //float_above_ground(velocity_, w, my_id);

  // TODO is this the best rounding strategy? (do we care here?)
  velocity_.x = divide(velocity_.x, 2, rounding_strategy<round_down, negative_mirrors_positive>());
  velocity_.y = divide(velocity_.y, 2, rounding_strategy<round_down, negative_mirrors_positive>());
  const distance xymag = i64sqrt(facing_.x*facing_.x + facing_.y*facing_.y);
  if (input_news.is_currently_pressed("5") || input_news.is_currently_pressed("s")) {
    velocity_.x = (facing_.x * tile_width * 15 / (4 * xymag)) / seconds;
    velocity_.y = (facing_.y * tile_width * 15 / (4 * xymag)) / seconds;
  }
  if (input_news.is_currently_pressed("space")) {
    velocity_.z = tile_width * 15 / 4 / seconds;
  }
  const bool turn_right = input_news.is_currently_pressed("right") || input_news.is_currently_pressed("d");
  const bool turn_left = input_news.is_currently_pressed("left") || input_news.is_currently_pressed("a");
  const bool turn_up = input_news.is_currently_pressed("up") || input_news.is_currently_pressed("w");
  const bool turn_down = input_news.is_currently_pressed("down") || input_news.is_currently_pressed("x");
  if (turn_right != turn_left) {
    const int which_way = (turn_right ? 1 : -1);
    const distance new_facing_x = facing_.x + which_way * facing_.y / 20;
    const distance new_facing_y = facing_.y - which_way * facing_.x / 20;
    facing_.x = new_facing_x; facing_.y = new_facing_y;
  }
  if (turn_up != turn_down) {
    const int which_way = (turn_up ? 1 : -1);
    const distance new_xymag = xymag - (which_way * facing_.z / 20);
    if (new_xymag > tile_width / 8) {
      facing_.z += which_way * xymag / 20;
      facing_.y = facing_.y * new_xymag / xymag;
      facing_.x = facing_.x * new_xymag / xymag;
    }
  }
  facing_ = facing_ * tile_width / facing_.magnitude_within_32_bits();
  
  vector3<distance> digging_laser_delta = facing_ * 3 / 2;
  vector3<distance> result_delta;
  const object_or_tile_identifier digging_laser_target = laser_find(w, my_id, location_, digging_laser_delta, true, &result_delta);
  if (input_news.num_times_pressed("c")) {
    if (tile_location const* locp = digging_laser_target.get_tile_location()) {
      if ((carrying_ < robot_max_carrying_capacity) &&
          (locp->stuff_at().contents() == ROCK || locp->stuff_at().contents() == RUBBLE)) {
        ++carrying_;
        w.replace_substance(*locp, locp->stuff_at().contents(), AIR);
      }
    }
  }
  else if (input_news.num_times_pressed("v") && (carrying_ > 0) && digging_laser_target == NO_OBJECT) {
    --carrying_;
    const tile_location target_loc = w.make_tile_location(
        get_random_containing_tile_coordinates(location_ + digging_laser_delta, w.get_rng()),
        FULL_REALIZATION);
    assert(target_loc.stuff_at().contents() == AIR);
    w.replace_substance(target_loc, AIR, RUBBLE);
  }
  else if (digging_laser_target != NO_OBJECT) {
    // hack - draw a thing indicating where we'd hit
    vector3<distance> hitloc = location_ + result_delta;
    w.add_laser_sfx(hitloc + vector3<distance>(-100*fine_distance_units, -100*fine_distance_units, -100*fine_distance_units),
                             vector3<distance>( 200*fine_distance_units,  200*fine_distance_units,  200*fine_distance_units));
    w.add_laser_sfx(hitloc + vector3<distance>( 100*fine_distance_units,  100*fine_distance_units, -100*fine_distance_units),
                             vector3<distance>(-200*fine_distance_units, -200*fine_distance_units,  200*fine_distance_units));
    w.add_laser_sfx(hitloc + vector3<distance>(-100*fine_distance_units,  100*fine_distance_units, -100*fine_distance_units),
                             vector3<distance>( 200*fine_distance_units, -200*fine_distance_units,  200*fine_distance_units));
    w.add_laser_sfx(hitloc + vector3<distance>( 100*fine_distance_units, -100*fine_distance_units, -100*fine_distance_units),
                             vector3<distance>(-200*fine_distance_units,  200*fine_distance_units,  200*fine_distance_units));
  }
    
  if (input_news.is_currently_pressed("l")) {
    const vector3<distance> offset(-facing_.y / 4, facing_.x / 4, 0);
    const vector3<distance> beam_vector = facing_ * tile_width * 2 / facing_.magnitude_within_32_bits() * 50;
    for (int i = 0; i < 2; ++i)
    {
      const object_or_tile_identifier hit_thing = laser_find(
          w, my_id,
          (i ? (location_ + offset) : (location_ - offset)),
          beam_vector, true);
      if(tile_location const* locp = hit_thing.get_tile_location()) {
        if (locp->stuff_at().contents() == ROCK) {
          w.replace_substance(*locp, ROCK, RUBBLE);
        }
      }
      if(object_identifier const* oidp = hit_thing.get_object_identifier()) {
        if (shared_ptr<object>* obj = w.get_object(*oidp)) {
          if (shared_ptr<mobile_object> mobj = boost::dynamic_pointer_cast<mobile_object>(*obj)) {
            const vector3<acceleration1d> tractor_beam_acceleration =
              -(facing_ * 45) / seconds / seconds;
            mobj->velocity_ += tractor_beam_acceleration
              / identity(fixed_frame_lengths / seconds) * fixed_frame_lengths;
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
  if (input_news.num_times_pressed("p")) {
    const shared_ptr<solar_panel> sol (new solar_panel(get_building_tile(w)));
    w.try_create_object(sol);
  }
  if (input_news.num_times_pressed("o")) {
    const shared_ptr<refinery> ref (new refinery(get_building_tile(w)));
    w.try_create_object(ref);
  }
  if (input_news.is_currently_pressed("r")) {
    const uniform_int_distribution<distance> random_delta(-tile_width, tile_width);
    for (int i = 0; i < 20; ++i) {
      const shared_ptr<random_walk_rocket> roc (new random_walk_rocket(
        location_ + facing_ * 2 +
          vector3<distance>(random_delta(w.get_rng()), random_delta(w.get_rng()), random_delta(w.get_rng())),
        facing_));
      w.try_create_object(roc);
    }
  }
}

vector3<tile_coordinate> robot::get_building_tile(world& w)const { // TODO: This use of world& should be able to be world const&
  tile_location first_guess = w.make_tile_location(
      get_min_containing_tile_coordinates(location_ + facing_ * 2),
      CONTENTS_ONLY);
  tile_location loc = first_guess;
  tile_location locd = loc.get_neighbor<zminus>(CONTENTS_ONLY);
  if ((loc.stuff_at().contents() == AIR) && (locd.stuff_at().contents() != AIR)) return loc.coords();

  tile_location uploc = loc.get_neighbor<zplus>(CONTENTS_ONLY);
  tile_location uplocd = loc;
  for (int offs = 0; offs < 15; ++offs) {
    loc = locd;
    locd = locd.get_neighbor<zminus>(CONTENTS_ONLY);
    if ((loc.stuff_at().contents() == AIR) && (locd.stuff_at().contents() != AIR)) return loc.coords();
    
    uplocd = uploc;
    uploc = uploc.get_neighbor<zplus>(CONTENTS_ONLY);
    if ((uploc.stuff_at().contents() == AIR) && (uplocd.stuff_at().contents() != AIR)) return uploc.coords();
  }
  return first_guess.coords();
}

shape laser_emitter::get_initial_personal_space_shape()const {
  return shape(geom::convex_polyhedron(bounding_box::min_and_max(
    location_ - vector3<distance>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10),
    location_ + vector3<distance>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10)
  )));
}

shape laser_emitter::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}


void laser_emitter::update(world& w, input_representation::input_news_t const&, object_identifier my_id) {
  update_location(location_, w, my_id);
  
  const uniform_int_distribution<distance> random_delta(-1023*fine_distance_units, 1023*fine_distance_units);
  for (int i = 0; i < 100; ++i) {
    do {
      facing_.x = random_delta(w.get_rng());//TODO implement vector3_location
      facing_.y = random_delta(w.get_rng());
      facing_.z = random_delta(w.get_rng());
    } while (facing_.magnitude_within_32_bits_is_greater_than(1023*fine_distance_units)
              || facing_.magnitude_within_32_bits_is_less_than(512*fine_distance_units));

    fire_standard_laser(w, my_id, location_, facing_);
  }
}




shape autorobot::get_initial_personal_space_shape()const {
  return shape(geom::convex_polyhedron(bounding_box::min_and_max(
    location_ - vector3<distance>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10),
    location_ + vector3<distance>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10)
  )));
}

shape autorobot::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

//hack impl.
bool once_a_second(world& w, int num = 1, int denom = 1) {
  return ((w.game_time_elapsed() / time_units * num / denom)
          % (identity(time_units / seconds) * seconds / time_units))
    == 0;
}

autorobot::autorobot(vector3<distance> location, vector3<distance> facing)
  : location_(location),
    initial_location_(((location / tile_width) * tile_width) + vector3<distance>(tile_width / 2, tile_width / 2, 0)),
    facing_(facing),
    carrying_(0) {
  distance bigger_dir;
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
      const boost::random::uniform_int_distribution<get_primitive_int_type<distance>::type> random_vel(-4000, 4000);
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
    velocity_.x = facing_.x * 15 / 4 / seconds;
    velocity_.y = facing_.y * 15 / 4 / seconds;
    
    if (facing_.z >= 0 || mag >= 10*tile_width) {
    const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(my_id)->second.bounds();
    const vector3<distance> middle = (shape_bounds.min() + shape_bounds.max()) / 2; //hmm, rounding.
    const vector3<distance> top_middle(middle(X), middle(Y), shape_bounds.max(Z) + tile_height / 4);
    /*const vector3<distance> top_middle(
      uniform_int_distribution<distance>(shape_bounds.min(X), shape_bounds.max(X))(rng),
      uniform_int_distribution<distance>(shape_bounds.min(Y), shape_bounds.max(Y))(rng),
      shape_bounds.max(Z));*/
    vector3<distance> beam_vector_1 = facing_;
    beam_vector_1[Z] -= tile_height / 2;
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
      const vector3<distance> beam_vector_2(facing_.x / -4, facing_.y / -4,
                                               1*fine_distance_units - shape_bounds.size(Z));
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
      velocity_.x = direction_home.x * tile_width * 15 / (4 * mag) / seconds;
      velocity_.y = direction_home.y * tile_width * 15 / (4 * mag) / seconds;
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



shape random_walk_rocket::get_initial_personal_space_shape()const {
  return shape(geom::convex_polyhedron(bounding_box::min_and_max(
    initial_location_ - vector3<distance>(tile_width * 2 / 15, tile_width * 2 / 15, tile_width * 2 / 15),
    initial_location_ + vector3<distance>(tile_width * 2 / 15, tile_width * 2 / 15, tile_width * 2 / 15)
  )));
}

shape random_walk_rocket::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

random_walk_rocket::random_walk_rocket(vector3<distance> location, vector3<distance> facing)
    : initial_location_(location) {
  velocity_ = (facing * tile_width * 120) / facing.magnitude_within_32_bits() / seconds;
}

void random_walk_rocket::update(world& w, input_representation::input_news_t const&, object_identifier) {
  auto& rng = w.get_rng();
  if (velocity_.magnitude_within_32_bits_is_greater_than(tile_width * 3 / 10 / seconds)) {
    velocity_ -= velocity_ * tile_width * 3 / (10 * velocity_.magnitude_within_32_bits()) / seconds;
  }
  else {
    velocity_[X] = 0;
    velocity_[Y] = 0;
    velocity_[Z] = 0;
  }

  const uniform_int_distribution<velocity1d>
          random_delta(-tile_width / seconds,
                        tile_width / seconds);
  velocity_[X] += random_delta(rng);
  velocity_[Y] += random_delta(rng);
  velocity_[Z] += random_delta(rng);
}


shape solar_panel::get_initial_personal_space_shape()const {
  return tile_shape(initial_location_);
}
shape solar_panel::get_initial_detail_shape()const {
  return tile_shape(initial_location_);
}



shape refinery::get_initial_personal_space_shape()const {
  return shape(bounding_box::min_and_max(lower_bound_in_fine_distance_units(initial_location_ - vector3<tile_coordinate>(1,1,0)), upper_bound_in_fine_distance_units(initial_location_ + vector3<tile_coordinate>(1,1,4))));
}
shape refinery::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}


void refinery::update(world& w, input_representation::input_news_t const&, object_identifier) {
  tile_location             input_loc =
      w.make_tile_location(initial_location_ - vector3<tile_coordinate_signed_type>(2, 0, 0), FULL_REALIZATION);
  tile_location waste_rock_output_loc =
      w.make_tile_location(initial_location_ + vector3<tile_coordinate_signed_type>(2, 0, 3), FULL_REALIZATION);
  tile_location      metal_output_loc =
      w.make_tile_location(initial_location_ + vector3<tile_coordinate_signed_type>(0, 2, 3), FULL_REALIZATION);

  if ((input_loc.stuff_at().contents() == RUBBLE) && (waste_rock_inside_ < 100) && (metal_inside_ < 100))  {
    w.replace_substance(input_loc, RUBBLE, AIR);
    waste_rock_inside_ += 80;
    metal_inside_ += 20;
  }
  if ((waste_rock_inside_ >= 100) && (waste_rock_output_loc.stuff_at().contents() == AIR)) {
    w.replace_substance(waste_rock_output_loc, AIR, RUBBLE);
    waste_rock_inside_ -= 100;
  }
  if ((     metal_inside_ >= 100) && (     metal_output_loc.stuff_at().contents() == AIR)) {
    w.replace_substance(     metal_output_loc, AIR, RUBBLE);
         metal_inside_ -= 100;
  }
}



