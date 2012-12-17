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

#ifndef LASERCAKE_SPECIFIC_OBJECT_TYPES_HPP__
#define LASERCAKE_SPECIFIC_OBJECT_TYPES_HPP__

#include "world.hpp"

class robot : public mobile_object, public autonomous_object, public object_with_eye_direction, public object_with_player_instructions {
public:
  robot(vector3<fine_scalar> location, vector3<fine_scalar> facing):location_(location),facing_(facing),carrying_(false){}
  
  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;
  
  virtual void update(world& w, input_representation::input_news_t const& mind_control, object_identifier my_id);
  vector3<fine_scalar> get_facing()const { return facing_; }
  vector3<tile_coordinate> get_building_tile(world& w)const; // TODO: This use of world& should be able to be world const&

  std::string player_instructions()const;
private:
  vector3<fine_scalar> location_;
  vector3<fine_scalar> facing_;
  int carrying_;
};

class laser_emitter : public mobile_object, public autonomous_object, public object_with_eye_direction {
public:
  laser_emitter(vector3<fine_scalar> location, vector3<fine_scalar> facing):location_(location),facing_(facing){}
  
  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;
  
  virtual void update(world& w, input_representation::input_news_t const& mind_control, object_identifier id);
  vector3<fine_scalar> get_facing()const { return facing_; }
private:
  vector3<fine_scalar> location_;
  vector3<fine_scalar> facing_;
};

class autorobot : public mobile_object, public autonomous_object, public object_with_eye_direction {
public:
  autorobot(vector3<fine_scalar> location, vector3<fine_scalar> facing);

  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;

  virtual void update(world& w, input_representation::input_news_t const& mind_control, object_identifier my_id);
  vector3<fine_scalar> get_facing()const { return facing_; }
private:
  vector3<fine_scalar> location_;
  vector3<fine_scalar> initial_location_;
  vector3<fine_scalar> facing_;
  int carrying_;
};

class solar_panel : public tile_aligned_object {
public:
  solar_panel(vector3<tile_coordinate> location):initial_location_(location){}
  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;
private:
  vector3<tile_coordinate> initial_location_;
};


#endif

