/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "specific_object_types.hpp"

#include "data_structures/bbox_collision_detector_iteration.hpp"
#include "tile_iteration.hpp"
#include "tile_physics.hpp"

namespace /* anonymous */ {
//TODO make downrobots dig sideways first?
const distance default_laser_length = 10000*tile_width;

namespace laserbeam {
  struct beam_first_contact_finder {
    beam_first_contact_finder(world const& w, geom::line_segment beam, object_or_tile_identifier ignore)
      : w_(w), beam_(beam) {ignores_.insert(ignore);}
    typedef geom::dimensionless_rational cost_type;
    typedef geom::optional_dimensionless_rational result_type;
    result_type min_cost(world_collision_detector::bounding_box const& bbox) const {
      return get_first_intersection(beam_, bbox);
    }
    result_type cost(object_identifier id, world_collision_detector::bounding_box const&) const {
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
      // You can't convert the world-bbox to tile coords (and also it will never rule out the laser)
      // TODO: Is there a better way to specify this than using std::numeric_limits?
      if((1LL << bbox.size_exponent_in_each_dimension()) > std::numeric_limits<tile_coordinate>::max()) {
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
  if(add_laser_sfx || result_beam_vector_ptr) {
    if(hit_thing != object_or_tile_identifier()) {
      // TODO do I have to worry about overflow?
      vector3<distance> result_beam_vector = multiply_rational_into(beam_vector, best_intercept_point);
      if (add_laser_sfx) w.add_laser_sfx(source, result_beam_vector);
      if (result_beam_vector_ptr) *result_beam_vector_ptr = result_beam_vector;
    }
    else {
      if (add_laser_sfx) w.add_laser_sfx(source, beam_vector);
      if (result_beam_vector_ptr) *result_beam_vector_ptr = beam_vector;
    }
  }
  return hit_thing;
}

void fire_standard_laser(world& w, object_identifier my_id, vector3<distance> location, vector3<distance> facing) {
  const vector3<distance> beam_vector = facing * default_laser_length / facing.magnitude_within_32_bits();
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


// hack
const auto tile_volume = 200*meters*meters*meters;


shape robot::get_initial_personal_space_shape()const {
  return shape(geom::convex_polyhedron(bounding_box::min_and_max(
    location_ - vector3<distance>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10),
    location_ + vector3<distance>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10)
  )));
  /*std::vector<vector3<distance>> verts;
  verts.push_back(location_ + vector3<distance>(tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(tile_width * 3 / 10, -tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(-tile_width * 3 / 10, -tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(-tile_width * 3 / 10, tile_width * 3 / 10, tile_width * 3 / 10));
  verts.push_back(location_ + vector3<distance>(0, 0, -tile_width * 3 / 10));
  return shape(geom::convex_polyhedron(verts));*/
}

cubic_meters robot::storage_volume()const {
  return tile_width * tile_width * tile_width * 15 * 15 * 15 / (20LL*20*20 * identity(fine_distance_units*fine_distance_units*fine_distance_units / meters/meters/meters));
}

shape robot::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}

cubic_meters conveyor_cost = 10*meters*meters*meters;
cubic_meters refinery_cost = 50*meters*meters*meters;
cubic_meters autorobot_cost = 100*meters*meters*meters;

// TODO remove duplicate stuff
cubic_meters object_cost(shared_ptr<object> objp) {
  if(dynamic_pointer_cast<conveyor_belt>(objp)) {
    return conveyor_cost;
  }
  else if(dynamic_pointer_cast<refinery>(objp)) {
    return refinery_cost;
  }
  else if(dynamic_pointer_cast<autorobot>(objp)) {
    return autorobot_cost;
  }
  return 0;
}


std::string draw_m3(cubic_meters m3) {
  return std::to_string(get_primitive_int(m3 / meters / meters / meters))/* + " m^3"*/; // omitting "m^3" at least until we can get a proper superscript (TODO?)
}

std::string robot::player_instructions()const {
  const std::string instructions = (
      (mode_ == "digging") ? "Digging mode: click to turn rock to rubble, throw rubble, collect pure metal, or deconstruct objects." :
      (mode_ == "laser")   ? "Laser mode: Hold mouse button to fire dual lasers." :
      (mode_ == "rockets") ? "Rockets mode: Hold mouse to create many silly fast-moving objects for testing. (This usually slows down the simulation.)" :
      (mode_ == "building_conveyor") ? "Conveyor mode: Click to build a conveyor belt (costs "+draw_m3(conveyor_cost)+") or click a conveyor to rotate it. Conveyors move rubble only." :
      (mode_ == "building_refinery") ? "Refinery mode: Click to build a refinery (costs "+draw_m3(refinery_cost)+"). Once built, refineries take rubble at the in-arrow and convert it to pure metal and waste rock." :
      (mode_ == "building_autorobot") ? "Autorobot mode: Click to build a digging robot (costs "+draw_m3(autorobot_cost)+").\nIt will dig up/down/straight depending on the up/down angle you're facing when you build it, move in the cardinal direction closest to the left/right angle you're facing, and dump its rubble at the x/y position where it was created." :
      "Unknown mode, this is an error!"
    ) + "\n\n"
    "Metal carried: " + draw_m3(metal_carried_) + "/" + draw_m3(storage_volume()) + "\n\n"
    "WASD: move  |  arrows/mouse: rotate view  |  space: jump/fly  |  ZXCVBR: switch mode"
    "\n"
    ;
  return instructions;
}

void robot::update(world& w, input_representation::input_news_t const& input_news, object_identifier my_id) {
  update_location(location_, w, my_id);
  float_above_ground(velocity_, w, my_id);

  // TODO is this the best rounding strategy? (do we care here?)
  velocity_.x = divide(velocity_.x, 2, rounding_strategy<round_down, negative_mirrors_positive>());
  velocity_.y = divide(velocity_.y, 2, rounding_strategy<round_down, negative_mirrors_positive>());
  const distance xymag = i64sqrt(facing_.x*facing_.x + facing_.y*facing_.y);
  /*if (input_news.is_currently_pressed("5") || input_news.is_currently_pressed("s")) {
    velocity_.x = (facing_.x * tile_width * 15 / (4 * xymag)) / seconds;
    velocity_.y = (facing_.y * tile_width * 15 / (4 * xymag)) / seconds;
  }*/
  if (input_news.is_currently_pressed("w") || input_news.is_currently_pressed("a") || input_news.is_currently_pressed("s") || input_news.is_currently_pressed("d")) {
    velocity_.x = 0;
    velocity_.y = 0;
    if (input_news.is_currently_pressed("w")) {
      velocity_.x += (facing_.x * tile_width * 15 / (4 * xymag)) / seconds;
      velocity_.y += (facing_.y * tile_width * 15 / (4 * xymag)) / seconds;
    }
    if (input_news.is_currently_pressed("s")) {
      velocity_.x -= (facing_.x * tile_width * 15 / (4 * xymag)) / seconds;
      velocity_.y -= (facing_.y * tile_width * 15 / (4 * xymag)) / seconds;
    }
    if (input_news.is_currently_pressed("a")) {
      velocity_.x -= (facing_.y * tile_width * 15 / (4 * xymag)) / seconds;
      velocity_.y += (facing_.x * tile_width * 15 / (4 * xymag)) / seconds;
    }
    if (input_news.is_currently_pressed("d")) {
      velocity_.x += (facing_.y * tile_width * 15 / (4 * xymag)) / seconds;
      velocity_.y -= (facing_.x * tile_width * 15 / (4 * xymag)) / seconds;
    }
  }
  if (input_news.is_currently_pressed("space")) {
    if (velocity_.z < tile_width * 15 / 4 / seconds) {
      velocity_.z += tile_width / 4 / seconds;
    }
  }
  const bool turn_right = input_news.is_currently_pressed("right")/* || input_news.is_currently_pressed("d")*/;
  const bool turn_left = input_news.is_currently_pressed("left")/* || input_news.is_currently_pressed("a")*/;
  const bool turn_up = input_news.is_currently_pressed("up")/* || input_news.is_currently_pressed("w")*/;
  const bool turn_down = input_news.is_currently_pressed("down")/* || input_news.is_currently_pressed("x")*/;
  const int64_t key_speed_factor = 22;
  const int64_t mouse_speed_factor = 2;
  const int64_t speed_divisor = 20*key_speed_factor;
  const int64_t turn_right_amount = mouse_speed_factor * input_news.mouse_displacement().x
                                    + input_news.mouse_displacement().x * abs(input_news.mouse_displacement().x) / 10
                                    + key_speed_factor * (turn_right - turn_left);
  const int64_t turn_up_amount    = mouse_speed_factor * input_news.mouse_displacement().y
                                    + input_news.mouse_displacement().y * abs(input_news.mouse_displacement().y) / 10
                                    + key_speed_factor * (turn_up - turn_down);
  if (turn_right_amount != 0) {
    const distance new_facing_x = facing_.x + turn_right_amount * facing_.y / speed_divisor;
    const distance new_facing_y = facing_.y - turn_right_amount * facing_.x / speed_divisor;
    const distance new_xymag = i64sqrt(new_facing_x*new_facing_x + new_facing_y*new_facing_y);
    const distance target_xymag = i64sqrt(tile_width*tile_width - facing_.z*facing_.z);
    facing_.x = new_facing_x * target_xymag / new_xymag;
    facing_.y = new_facing_y * target_xymag / new_xymag;
  }
  if (turn_up_amount != 0) {
    const distance new_xymag = xymag - (turn_up_amount * facing_.z / speed_divisor);
    if (new_xymag > tile_width / 8) {
      facing_.z += turn_up_amount * xymag / speed_divisor;
      facing_.y = facing_.y * new_xymag / xymag;
      facing_.x = facing_.x * new_xymag / xymag;
    }
  facing_ = facing_ * tile_width / facing_.magnitude_within_32_bits();
  }

  if (
       (((mode_ == "laser") || (mode_ == "rockets")) && input_news.is_currently_pressed(input_representation::left_mouse_button))
    || input_news.num_times_pressed(input_representation::left_mouse_button)
     ) {
    perform_click_action(w, my_id, get_current_click_action(w, my_id));
  }
  
  if (input_news.num_times_pressed("z")) mode_ = "digging";
  if (input_news.num_times_pressed("x")) mode_ = "laser";
  if (input_news.num_times_pressed("r")) mode_ = "rockets";
  if (input_news.num_times_pressed("v")) mode_ = "building_refinery";
  if (input_news.num_times_pressed("b")) mode_ = "building_autorobot";
  if (input_news.num_times_pressed("c")) mode_ = "building_conveyor";
}

click_action robot::get_current_click_action(world& w, object_identifier my_id)const { // TODO: This use of world& should be able to be world const&
  click_action result;
  if (mode_ == "rockets") {
    result.type = FIRE_ROCKETS;
    return result;
  }
  
  vector3<distance> action_laser_delta = facing_ * 2;
  if (mode_ == "laser") {
    action_laser_delta = facing_ * default_laser_length / facing_.magnitude_within_32_bits();
    result.type = SHOOT_LASERS;
  }
  if (mode_ == "building_refinery") action_laser_delta = facing_ * 3;
  vector3<distance> result_delta;
  const object_or_tile_identifier thing_hit = laser_find(w, my_id, location_, action_laser_delta, false, &result_delta);
  result.fine_target_location = location_ + result_delta;

  // TODO fix duplicate list of mode names
  if (mode_ == "building_refinery" || mode_ == "building_conveyor" || mode_ == "building_autorobot") {
    tile_location first_guess = w.make_tile_location(
        get_min_containing_tile_coordinates(result.fine_target_location), // TODO: min gives direction bias, what to do?
        CONTENTS_ONLY);
    tile_location loc = first_guess;
    tile_location locd = loc.get_neighbor<zminus>(CONTENTS_ONLY);
    result.which_affected = loc;
    if ((loc.stuff_at().contents() == AIR) && (locd.stuff_at().contents() != AIR)) {
      // the first guess was good
    }
    else {
      tile_location uploc = loc;
      tile_location uplocd = locd;
      for (int offs = 0; offs < 15; ++offs) {
        loc = locd;
        locd = locd.get_neighbor<zminus>(CONTENTS_ONLY);
        if ((loc.stuff_at().contents() == AIR) && (locd.stuff_at().contents() != AIR)) {
          result.which_affected = loc;
          break;
        }
        uplocd = uploc;
        uploc = uploc.get_neighbor<zplus>(CONTENTS_ONLY);
        if ((uploc.stuff_at().contents() == AIR) && (uplocd.stuff_at().contents() != AIR)) {
          result.which_affected = uploc;
          break;
        }
      }
      // if the loop finishes with no result, leave it at the first guess
    }

    tile_contents foundation = result.which_affected.get_tile_location()->get_neighbor<zminus>(FULL_REALIZATION).stuff_at().contents();
    bool can_build_building = ((foundation != AIR) && !is_water(foundation));

    if (mode_ == "building_refinery" && can_build_building && metal_carried_ >= refinery_cost) {
      result.type = BUILD_OBJECT;
      assert(result.which_affected.get_tile_location());
      result.object_built = shared_ptr<object>(new refinery(result.which_affected.get_tile_location()->coords()));
      result.metal_spent = refinery_cost;
    }
    if (mode_ == "building_conveyor") {
      if (object_identifier const* oidp = thing_hit.get_object_identifier()) {
        if (shared_ptr<object>* obj = w.get_object(*oidp)) {
          if (shared_ptr<conveyor_belt> belt = boost::dynamic_pointer_cast<conveyor_belt>(*obj)) {
            result.type = ROTATE_CONVEYOR;
            result.which_affected = *oidp;
          }
        }
      }
      if (result.type != ROTATE_CONVEYOR && can_build_building && metal_carried_ >= conveyor_cost) {
        result.type = BUILD_OBJECT;
        assert(result.which_affected.get_tile_location());
        result.object_built = shared_ptr<object>(new conveyor_belt(result.which_affected.get_tile_location()->coords()));
        result.metal_spent = conveyor_cost;
      }
    }
    if (mode_ == "building_autorobot" && metal_carried_ >= autorobot_cost) {
      result.type = BUILD_OBJECT;
      result.object_built = shared_ptr<object>(new autorobot(result.which_affected.get_tile_location()->coords(), facing_));
      result.metal_spent = autorobot_cost;
    }
  }

  if (mode_ == "digging") {
    if (tile_location const* locp = thing_hit.get_tile_location()) {
      result.which_affected = *locp;
      if (locp->stuff_at().contents() == ROCK) {
        result.type = DIG_ROCK_TO_RUBBLE;
      }
      else if (locp->stuff_at().contents() == RUBBLE) {
        // we can pick up pure metal
        if (w.get_minerals(locp->coords()).metal == tile_volume) {
          if (metal_carried_ + tile_volume <= storage_volume()) {
            result.type = COLLECT_METAL;
          }
        }
        else {
          result.type = THROW_RUBBLE;
        }
      }
    }
    if (object_identifier const* oidp = thing_hit.get_object_identifier()) {
      if (shared_ptr<object>* objpp = w.get_object(*oidp)) {
        if (metal_carried_ + object_cost(*objpp) <= storage_volume()) {
          result.type = DECONSTRUCT_OBJECT;
          result.which_affected = *oidp;
        }
      }
    }
  }

  return result;
}

void robot::perform_click_action(world& w, object_identifier my_id, click_action a) {
  switch (a.type) {
    case DIG_ROCK_TO_RUBBLE: {
      tile_location const* locp = a.which_affected.get_tile_location();
      assert(locp);
      
      w.replace_substance(*locp, ROCK, RUBBLE);
    } break;
    
    case THROW_RUBBLE: {
      tile_location const* locp = a.which_affected.get_tile_location();
      assert(locp);
      
          // hack way to speed the tile?
          vector3<velocity1d> push_dir = facing_ / seconds;
          push_dir.z = 0;
          push_dir = push_dir * (tile_width * 3 / seconds) / push_dir.magnitude_within_32_bits();
          if ((((push_dir.x > 0) ? locp->get_neighbor<xplus>(CONTENTS_ONLY) : locp->get_neighbor<xminus>(CONTENTS_ONLY)).stuff_at().contents() != AIR) &&
            (((push_dir.y > 0) ? locp->get_neighbor<yplus>(CONTENTS_ONLY) : locp->get_neighbor<yminus>(CONTENTS_ONLY)).stuff_at().contents() != AIR)) {
            push_dir = push_dir * 2 / 3;
            push_dir.z += (tile_width * 2 / seconds);
          }
          get_state(w.tile_physics()).active_fluids[*locp].velocity += vector3<sub_tile_velocity>(
            push_dir * identity(tile_physics_sub_tile_distance_units / fine_distance_units) / identity(fixed_frame_lengths / seconds));
    } break;

    case COLLECT_METAL: {
      tile_location const* locp = a.which_affected.get_tile_location();
      assert(locp);
      
      w.replace_substance(*locp, RUBBLE, AIR);
      metal_carried_ += tile_volume;
    } break;

    case ROTATE_CONVEYOR: {
      object_identifier const* oidp = a.which_affected.get_object_identifier();
      assert(oidp);
      shared_ptr<object>* obj = w.get_object(*oidp);
      assert(obj);
      shared_ptr<conveyor_belt> belt = boost::dynamic_pointer_cast<conveyor_belt>(*obj);
      assert(belt);
      belt->rotate();
    } break;

    case DECONSTRUCT_OBJECT: {
      object_identifier const* oidp = a.which_affected.get_object_identifier();
      assert(oidp);
      if (shared_ptr<object>* objpp = w.get_object(*oidp)) {
        w.delete_object_soon(*oidp);
        metal_carried_ += object_cost(*objpp);
      }
    } break;

    case FIRE_ROCKETS: {
      const uniform_int_distribution<distance> random_delta(-tile_width, tile_width);
      for (int i = 0; i < 20; ++i) {
        const shared_ptr<random_walk_rocket> roc (new random_walk_rocket(
          location_ + facing_ * 2 +
            vector3<distance>(random_delta(w.get_rng()), random_delta(w.get_rng()), random_delta(w.get_rng())),
          facing_));
        w.try_create_object(roc);
      }
    } break;

    case SHOOT_LASERS: {
      const vector3<distance> offset(-facing_.y / 4, facing_.x / 4, 0);
      const vector3<distance> beam_vector = facing_ * default_laser_length / facing_.magnitude_within_32_bits();

      const uniform_int_distribution<distance> random_delta(-(default_laser_length / 10), (default_laser_length / 10));
      for (int i = 0; i < 20; ++i)
      {
        const object_or_tile_identifier hit_thing = laser_find(
            w, my_id,
            ((i & 1) ? (location_ + offset) : (location_ - offset)),
            beam_vector + vector3<distance>(random_delta(w.get_rng()), random_delta(w.get_rng()), random_delta(w.get_rng())), true);
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
    } break;

    case BUILD_OBJECT: {
      if (w.try_create_object(a.object_built) != NO_OBJECT) metal_carried_ -= a.metal_spent;
    } break;

    case NO_CLICK_ACTION: {
    } break;

    default: assert(false);
  }
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
    location_ - vector3<distance>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10),
    location_ + vector3<distance>(tile_width * 4 / 10, tile_width * 4 / 10, tile_width * 4 / 10)
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

autorobot::autorobot(vector3<tile_coordinate> location, vector3<distance> facing)
  : location_(lower_bound_in_fine_distance_units(location) + vector3<distance>(tile_width, tile_width, tile_width) / 2),
    initial_location_(location),
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
  if (std::abs(bigger_dir) < std::abs(facing.z) * 4) {
    facing_.z = facing_.z > 0 ? tile_height : -tile_height;
  }
  else facing_.z = 0;
}

void autorobot::update(world& w, input_representation::input_news_t const&, object_identifier my_id) {
  auto& rng = w.get_rng();
  update_location(location_, w, my_id);
  float_above_ground(velocity_, w, my_id);

  const bounding_box shape_bounds = w.get_object_personal_space_shapes().find(my_id)->second.bounds();
  const vector3<distance> middle = (shape_bounds.min() + shape_bounds.max()) / 2; //hmm, rounding.
  const vector3<distance> top_middle(middle(X), middle(Y), shape_bounds.max(Z));
  const vector3<distance> bottom_middle(middle(X), middle(Y), shape_bounds.min(Z));
  
  vector3<distance> direction_home = lower_bound_in_fine_distance_units(initial_location_) + vector3<distance>(tile_width, tile_width, tile_width) / 2 - location_;
  direction_home.z = 0;
  distance direction_home_xymag = direction_home.magnitude_within_32_bits();
  vector3<distance> facing_xy = facing_; facing_xy.z = 0;
  vector3<distance> dir_in_proper_direction = facing_xy * direction_home.dot<lint64_t>(facing_xy) / facing_xy.dot<lint64_t>(facing_xy);
  vector3<distance> dir_in_wrong_directions = (direction_home - dir_in_proper_direction);
  if (dir_in_wrong_directions.magnitude_within_32_bits_is_greater_than(tile_width / 20)) {
    // We're off course: Get back on.
    // Note that we rely on slow movement for precision, because of framerate issues.
    // TODO (long-term): have something clean to do about that.
    distance dir_in_wrong_directions_xymag = dir_in_wrong_directions.magnitude_within_32_bits();
    velocity_.x = -dir_in_wrong_directions.x * tile_width / dir_in_wrong_directions_xymag / seconds;
    velocity_.y = -dir_in_wrong_directions.y * tile_width / dir_in_wrong_directions_xymag / seconds;
  }
  else if (carrying_ >= 2) {
    // We're full: go home.
    if (direction_home_xymag != 0) {
      velocity_.x = direction_home.x * tile_width * 15 / (4 * direction_home_xymag) / seconds;
      velocity_.y = direction_home.y * tile_width * 15 / (4 * direction_home_xymag) / seconds;
    }
    // If we got home we need to drop our stuff.
    if (direction_home_xymag <= tile_width / 3) {
      tile_location loc = w.make_tile_location(get_random_containing_tile_coordinates(bottom_middle, rng), FULL_REALIZATION);
      while (carrying_ > 0) {
        if (loc.stuff_at().contents() == AIR) {
          w.replace_substance(loc, AIR, RUBBLE);
          get_state(w.tile_physics()).altered_minerals_info.insert(std::make_pair(loc.coords(), carried_minerals.back()));
          carried_minerals.pop_back();
          --carrying_;
        }
        loc = loc.get_neighbor<zplus>(FULL_REALIZATION);
        if (loc.coords().z > get_max_containing_tile_coordinate(top_middle.z, Z)) break;
      }
    }
  }
  else {
    // Go forward and dig!
    velocity_.x = facing_.x * 15 / 4 / seconds;
    velocity_.y = facing_.y * 15 / 4 / seconds;
    
    //if (facing_.z >= 0 || direction_home_xymag >= 10*tile_width) {
      const vector3<distance> laser_start_point = top_middle + (facing_xy * 4 / 10) + vector3<distance>(0,0,(facing_.z > 0) ? tile_height : 1*fine_distance_units);
      /*const vector3<distance> top_middle(
        uniform_int_distribution<distance>(shape_bounds.min(X), shape_bounds.max(X))(rng),
        uniform_int_distribution<distance>(shape_bounds.min(Y), shape_bounds.max(Y))(rng),
        shape_bounds.max(Z));*/
      vector3<distance> beam_vector_1 = (facing_.z > 0) ? facing_xy : facing_;
      beam_vector_1[Z] -= tile_height / 2;
      const object_or_tile_identifier hit1 = laser_find(w, my_id, laser_start_point, beam_vector_1, true);
      if (hit1 != object_or_tile_identifier()) {
        if (tile_location const* locp = hit1.get_tile_location()) {
          if ((locp->stuff_at().contents() == ROCK || locp->stuff_at().contents() == RUBBLE)) {
            carried_minerals.push_back(w.get_minerals(locp->coords()));
            ++carrying_;
            w.replace_substance(*locp, locp->stuff_at().contents(), AIR);
          }
        }
      }
      else {
        const vector3<distance> beam_vector_2(facing_.x / -4, facing_.y / -4,
                                                1*fine_distance_units - shape_bounds.size(Z) - tile_height*2);
        const object_or_tile_identifier hit2 = laser_find(w, my_id, laser_start_point + beam_vector_1, beam_vector_2, true);
        if (hit2 != object_or_tile_identifier()) {
          if (tile_location const* locp = hit2.get_tile_location()) {
            tile_location const& loc = *locp;
            if ((loc.stuff_at().contents() == ROCK || loc.stuff_at().contents() == RUBBLE)) {
              // TODO probably have autorobots use cardinal directions in the first place
              tile_location backloc = loc.get_neighbor_by_variable(opposite_cardinal_direction(get_cdir()), CONTENTS_ONLY);
              if ( ((facing_.z  > 0) && (backloc.get_neighbor<zminus>(CONTENTS_ONLY).stuff_at().contents() == AIR))
                || ((facing_.z <= 0) && (backloc                                    .stuff_at().contents() == AIR))
                || ((facing_.z  < 0) && (direction_home_xymag >= 10*tile_width)
                                     && (backloc.get_neighbor<zplus >(CONTENTS_ONLY).stuff_at().contents() == AIR))
                ) {
                carried_minerals.push_back(w.get_minerals(loc.coords()));
                ++carrying_;
                w.replace_substance(*locp, loc.stuff_at().contents(), AIR);
              }
            }
          }
        }
      }
    //}
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

vector3<tile_coordinate> refinery::input_loc_coords()const { return initial_location_ - vector3<tile_coordinate_signed_type>(2, 0, 0); }
vector3<tile_coordinate> refinery::waste_rock_output_loc_coords()const { return initial_location_ + vector3<tile_coordinate_signed_type>(2, 0, 3); }
vector3<tile_coordinate> refinery::metal_output_loc_coords()const { return initial_location_ + vector3<tile_coordinate_signed_type>(0, 2, 3); }

void refinery::update(world& w, input_representation::input_news_t const&, object_identifier) {
  tile_location             input_loc =
      w.make_tile_location(input_loc_coords(), FULL_REALIZATION);
  tile_location waste_rock_output_loc =
      w.make_tile_location(waste_rock_output_loc_coords(), FULL_REALIZATION);
  tile_location      metal_output_loc =
      w.make_tile_location(metal_output_loc_coords(), FULL_REALIZATION);
  
  if ((input_loc.stuff_at().contents() == RUBBLE) && (waste_rock_inside_ < tile_volume) && (metal_inside_ < tile_volume)) {
    minerals m = w.get_minerals(input_loc.coords());
    metal_inside_ += m.metal;
    waste_rock_inside_ += tile_volume - m.metal;
    w.replace_substance(input_loc, RUBBLE, AIR);
  }
  if ((waste_rock_inside_ >= tile_volume) && (waste_rock_output_loc.stuff_at().contents() == AIR)) {
    w.replace_substance(waste_rock_output_loc, AIR, RUBBLE);
    get_state(w.tile_physics()).altered_minerals_info.insert(std::make_pair(waste_rock_output_loc.coords(), minerals(          0)));
    waste_rock_inside_ -= tile_volume;
  }
  if ((     metal_inside_ >= tile_volume) && (     metal_output_loc.stuff_at().contents() == AIR)) {
    w.replace_substance(     metal_output_loc, AIR, RUBBLE);
    get_state(w.tile_physics()).altered_minerals_info.insert(std::make_pair(     metal_output_loc.coords(), minerals(tile_volume)));
         metal_inside_ -= tile_volume;
  }
}





shape conveyor_belt::get_initial_personal_space_shape()const {
  return shape(bounding_box::min_and_max(lower_bound_in_fine_distance_units(initial_location_), upper_bound_in_fine_distance_units(initial_location_) - vector3<distance>(0, 0, tile_height * 4 / 5)));
}
shape conveyor_belt::get_initial_detail_shape()const {
  return get_initial_personal_space_shape();
}


void conveyor_belt::update(world& w, input_representation::input_news_t const&, object_identifier) {
  tile_location loc = w.make_tile_location(initial_location_, FULL_REALIZATION);
  if (loc.stuff_at().contents() == RUBBLE) {
    // hack way to speed the tile?
    vector3<sub_tile_velocity>& tile_vel = get_state(w.tile_physics()).active_fluids[loc].velocity;
    sub_tile_velocity target_vel((tile_width / seconds) * identity(tile_physics_sub_tile_distance_units / fine_distance_units) / identity(fixed_frame_lengths / seconds));
    sub_tile_velocity one_frame_acceleration((20 * meters / seconds / seconds) * identity(tile_physics_sub_tile_distance_units / meters) / identity(fixed_frame_lengths / seconds) / identity(fixed_frame_lengths / seconds) * fixed_frame_lengths);
    const which_dimension_type dim = which_dimension_is_cardinal_direction(direction_);
    if (is_a_positive_directional_cardinal_direction(direction_)) {
      if (tile_vel(dim) < target_vel) {
        tile_vel[dim] += one_frame_acceleration;
        if (tile_vel(dim) > target_vel) tile_vel[dim] = target_vel;
      }
    }
    else {
      if (-tile_vel(dim) < target_vel) {
        tile_vel[dim] -= one_frame_acceleration;
        if (-tile_vel(dim) > target_vel) tile_vel[dim] = -target_vel;
      }
    }
    const which_dimension_type dim2 = (dim == X) ? Y : X;
    if (abs(tile_vel(dim2)) <= one_frame_acceleration) tile_vel[dim2] = 0;
    else tile_vel[dim2] -= one_frame_acceleration * sign(tile_vel(dim2));
    tile_location next_loc = loc.get_neighbor_by_variable(direction_, CONTENTS_ONLY);
    sub_tile_velocity target_zvel((next_loc.stuff_at().contents() == AIR) ? 0 : ((tile_height / seconds) * identity(tile_physics_sub_tile_distance_units / fine_distance_units) / identity(fixed_frame_lengths / seconds)));
    if (tile_vel(Z) < target_zvel) {
      tile_vel[Z] += one_frame_acceleration;
      if (tile_vel(Z) > target_zvel) tile_vel[Z] = target_zvel;
    }
  }
}



