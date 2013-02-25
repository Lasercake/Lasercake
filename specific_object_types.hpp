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

#ifndef LASERCAKE_SPECIFIC_OBJECT_TYPES_HPP__
#define LASERCAKE_SPECIFIC_OBJECT_TYPES_HPP__

#include "world.hpp"


#if 0
// Should all views be objects in the world?
// (Dilemma: may, or may not, the view affect the simulation even a bit?)
// If views from no particular robot were objects in the world,
// they might look like this:

class dull_ghost : public autonomous_object {
public:
  virtual void update(world& w, input_representation::input_news_t const& mind_control, object_identifier my_id);
private:
  vector3<distance> location_;
  vector3<distance> facing_;//double view_direction;
};

class circling_surveillance_ghost : public autonomous_object {
public:
  virtual void update(world& w, input_representation::input_news_t const& mind_control, object_identifier my_id);
private:
  // circles a particular location at a distance
  // currently, the angle of view is a function of the current time, lol;
  // a trigonometric function, too, so we can't call it except in display code.
  vector3<distance> surveilled_location_;
  distance view_dist_;
};
#endif

// TODO: Decide if this is a unit we want to regularly use for volume,
// and if it is, put it in world_constants.hpp
typedef physical_quantity<lint64_t, units<dim::meter<3>>> cubic_meters;

enum click_action_type {
  NO_CLICK_ACTION,
  DIG_ROCK_TO_RUBBLE,
  THROW_RUBBLE,
  COLLECT_METAL,
  FIRE_ROCKETS,
  ROTATE_CONVEYOR,
  SHOOT_LASERS,
  BUILD_OBJECT,
  DECONSTRUCT_OBJECT
};
struct click_action {
  click_action():type(NO_CLICK_ACTION),fine_target_location(),which_affected(NO_OBJECT),object_built(),metal_spent(0){}
  click_action_type type;
  vector3<distance> fine_target_location;
  object_or_tile_identifier which_affected;
  shared_ptr<object> object_built;
  cubic_meters metal_spent;
};

class robot : public mobile_object, public autonomous_object, public object_with_eye_direction, public object_with_player_instructions {
public:
  robot(vector3<distance> location, vector3<distance> facing):location_(location),facing_(facing),metal_carried_(storage_volume()),mode_("digging"){}
  
  virtual shape get_initial_personal_space_shape()const override;
  virtual shape get_initial_detail_shape()const override;
  
  virtual void update(world& w, input_representation::input_news_t const& mind_control,
                      object_identifier my_id) override;
  vector3<distance> get_facing()const override { return facing_; }
  //vector3<tile_coordinate> get_building_tile(world& w, object_identifier my_id)const; // TODO: This use of world& should be able to be world const&

  std::string player_instructions()const override;
  
  click_action get_current_click_action(world& w, object_identifier my_id)const; // TODO: This use of world& should be able to be world const&
private:
  void perform_click_action(world& w, object_identifier my_id, click_action a);
  cubic_meters storage_volume()const;
  vector3<distance> location_;
  vector3<distance> facing_;
  cubic_meters metal_carried_;
  std::string mode_;
};

class laser_emitter : public mobile_object, public autonomous_object, public object_with_eye_direction {
public:
  laser_emitter(vector3<distance> location, vector3<distance> facing):location_(location),facing_(facing){}
  
  virtual shape get_initial_personal_space_shape()const override;
  virtual shape get_initial_detail_shape()const override;
  
  virtual void update(world& w, input_representation::input_news_t const& mind_control,
                      object_identifier id) override;
  vector3<distance> get_facing()const override { return facing_; }
private:
  vector3<distance> location_;
  vector3<distance> facing_;
};

class autorobot : public mobile_object, public autonomous_object, public object_with_eye_direction {
public:
  autorobot(vector3<tile_coordinate> location, vector3<distance> facing);

  virtual shape get_initial_personal_space_shape()const override;
  virtual shape get_initial_detail_shape()const override;

  virtual void update(world& w, input_representation::input_news_t const& mind_control,
                      object_identifier my_id) override;
  vector3<distance> get_facing()const override { return facing_; }
private:
  vector3<distance> location_;
  vector3<tile_coordinate> initial_location_;
  vector3<distance> facing_;
  int carrying_;
  std::vector<minerals> carried_minerals;
};

class random_walk_rocket : public mobile_object, public autonomous_object {
public:
  random_walk_rocket(vector3<distance> location, vector3<distance> facing);

  virtual shape get_initial_personal_space_shape()const override;
  virtual shape get_initial_detail_shape()const override;

  virtual void update(world& w, input_representation::input_news_t const& mind_control,
                      object_identifier my_id) override;
private:
  vector3<distance> initial_location_;
};


class solar_panel : public tile_aligned_object {
public:
  solar_panel(vector3<tile_coordinate> location):initial_location_(location){}
  virtual shape get_initial_personal_space_shape()const override;
  virtual shape get_initial_detail_shape()const override;
private:
  vector3<tile_coordinate> initial_location_;
};


// I don't know if "refinery" is a correct term for just extracting metal from ore.
// "smelter" might be correct (this device probably uses a more efficient process
//    than smelting per se, but there's precedent for using "smelter" generally)
class refinery : public tile_aligned_object, public autonomous_object {
public:
  refinery(vector3<tile_coordinate> location):initial_location_(location),waste_rock_inside_(0),metal_inside_(0){}
  virtual shape get_initial_personal_space_shape()const override;
  virtual shape get_initial_detail_shape()const override;
  
  vector3<tile_coordinate> input_loc_coords()const;
  vector3<tile_coordinate> waste_rock_output_loc_coords()const;
  vector3<tile_coordinate> metal_output_loc_coords()const;

  virtual void update(world& w, input_representation::input_news_t const& mind_control,
                      object_identifier my_id) override;
private:
  vector3<tile_coordinate> initial_location_;
  cubic_meters waste_rock_inside_;
  cubic_meters metal_inside_;
};


class conveyor_belt : public tile_aligned_object, public autonomous_object {
public:
  conveyor_belt(vector3<tile_coordinate> location, cardinal_direction dir = xplus):initial_location_(location),direction_(dir){}
  virtual shape get_initial_personal_space_shape()const override;
  virtual shape get_initial_detail_shape()const override;

  virtual void update(world& w, input_representation::input_news_t const& mind_control,
                      object_identifier my_id) override;

  cardinal_direction direction()const {return direction_;}
  void rotate() {
    direction_ =
            (direction_ == xplus ) ? yplus  :
            (direction_ == yplus ) ? xminus :
            (direction_ == xminus) ? yminus : xplus;
  }
private:
  vector3<tile_coordinate> initial_location_;
  cardinal_direction direction_;
};


#endif

