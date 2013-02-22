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

#ifndef LASERCAKE_GL_DATA_PREPARATION_HPP__
#define LASERCAKE_GL_DATA_PREPARATION_HPP__

#include "world.hpp"
#include "input_representation.hpp"
#include "gl_data_abstract.hpp"

struct gl_data_preparation_config {
  gl_data_preparation_config(distance view_radius, object_identifier view_from)
   : view_radius(view_radius), view_from(view_from) {}
  distance view_radius;
  object_identifier view_from;
  // TODO do we also want the config to be able to specify
  // a view *location* (that's not an object)?
  // view_from is used to look up location, facing, and avoiding
  // drawing the object we're viewing from.
  // It is also only used when view_on_the_world::view_type is ROBOT.
};
inline std::ostream& operator<<(std::ostream& os, gl_data_preparation_config const& c) {
  // TODO does the float precision we output here matter?
  return os << "{view_radius=" << c.view_radius << "; view_from=" << c.view_from << "}";
}

class view_on_the_world {
public:
  view_on_the_world(vector3<distance> approx_initial_center);

  // Call this every frame that you want user-input to have view-related effects.
  void input(input_representation::input_news_t const& input_news);

  // To call, default-construct the result and then pass it as a reference.
  void prepare_gl_data(world /*TODO const*/& w, gl_data_preparation_config c, abstract_gl_data& result);

  // TODO make private and/or trailing underscore
  vector3<distance> view_loc_for_local_display;
  enum { GLOBAL, LOCAL, ROBOT } view_type;
  double local_view_direction;
  vector3<distance> surveilled_by_global_display;
  distance global_view_dist;
  bool drawing_regular_stuff;
  bool drawing_debug_stuff;
};


#endif
