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

class robot : public mobile_object, public autonomous_object {
public:
  robot(vector3<fine_scalar> location, vector3<fine_scalar> facing):location(location),facing(facing),carrying(false){}
  
  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;
  
  virtual void update(world &w, object_identifier my_id);
  vector3<fine_scalar> get_facing()const { return facing; }
private:
  vector3<fine_scalar> location;
  vector3<fine_scalar> facing;
  bool carrying;
};

class laser_emitter : public mobile_object, public autonomous_object {
public:
  laser_emitter(vector3<fine_scalar> location, vector3<fine_scalar> facing):location(location),facing(facing){}
  
  virtual shape get_initial_personal_space_shape()const;
  virtual shape get_initial_detail_shape()const;
  
  virtual void update(world &w, object_identifier id);
private:
  vector3<fine_scalar> location;
  vector3<fine_scalar> facing;
};

#endif

