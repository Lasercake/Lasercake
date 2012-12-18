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

#ifndef LASERCAKE_OBJECT_AND_TILE_ITERATION_HPP__
#define LASERCAKE_OBJECT_AND_TILE_ITERATION_HPP__

#include "tile_iteration.hpp"
#include "data_structures/bbox_collision_detector_iteration.hpp"

// w.visit_collidable_tiles_and_objects(bbox, visitor)
//
// visitor e.g.:
// struct visitor {
//  octant_number octant()const { return octant_; }
//  octant_number octant_; //e.g. from vector_octant()
//
//  void found(tile_location const& loc);
//  void found(object_identifier oid);
// };

namespace visit_collidable_tiles_and_objects_impl {

typedef lasercake_int<int64_t>::type cost_type;
//typedef faux_optional<cost_type> optional_cost_type;

inline cost_type cost_for(octant_number octant, objects_collision_detector::bounding_box bbox) {
  return
    (LASERCAKE_OCTANT_X_POSITIVE(octant) ? cost_type(bbox.min(X)) : -cost_type(bbox.max(X))) +
    (LASERCAKE_OCTANT_Y_POSITIVE(octant) ? cost_type(bbox.min(Y)) : -cost_type(bbox.max(Y))) +
    (LASERCAKE_OCTANT_Z_POSITIVE(octant) ? cost_type(bbox.min(Z)) : -cost_type(bbox.max(Z)));
}
inline cost_type cost_for(octant_number octant, tile_location loc) {
  vector3<tile_coordinate> const& coords = loc.coords();
  return
    (LASERCAKE_OCTANT_X_POSITIVE(octant) ?  cost_type(lower_bound_in_fine_units(coords(X), X))
                                         : -cost_type(upper_bound_in_fine_units(coords(X), X))) +
    (LASERCAKE_OCTANT_Y_POSITIVE(octant) ?  cost_type(lower_bound_in_fine_units(coords(Y), Y))
                                         : -cost_type(upper_bound_in_fine_units(coords(Y), Y))) +
    (LASERCAKE_OCTANT_Z_POSITIVE(octant) ?  cost_type(lower_bound_in_fine_units(coords(Z), Z))
                                         : -cost_type(upper_bound_in_fine_units(coords(Z), Z)));
}

// for object iteration:
template<typename Visitor>
struct get_cost {
  typedef visit_collidable_tiles_and_objects_impl::cost_type cost_type;
  octant_number octant_;
  //Visitor& visitor_;
  // unnecessarily HACK-y
//  bounding_box bbox_;
  
  /*optional_*/cost_type min_cost(objects_collision_detector::bounding_box bbox) {
    // unnecessarily HACK-y
//    if(!overlaps(bbox_, bbox)) return boost::none;
    return cost_for(octant_, bbox);
  }
  // ignore the object's actual shape (part of the HACK)
  cost_type cost(object_identifier, objects_collision_detector::bounding_box bbox) {
    return cost_for(octant_, bbox);
  }
};

// for tile visitation (inside tile visitation we call object iteration):
template<typename Visitor>
struct tile_visitor {
  tribool look_here(power_of_two_bounding_cube<3, tile_coordinate> const& bbox) {
    return true;
    // unnecessarily HACK-y
//    if(subsumes(tile_bbox_, bbox)) return true;
//    if(overlaps(tile_bbox_, bbox)) return indeterminate;
//    return false;
  }
  bool collidable_tile(tile_location const& loc) {
    const cost_type cost = cost_for(octant_, loc);
    while(obj_i_ != obj_end_ && obj_i_->cost < cost) {
      visitor_.found(obj_i_->object);
    }
    visitor_.found_(loc);
    return true;
  }
  octant_number octant()const { return octant_; }
  octant_number octant_;
  //get_cost<Visitor> get_cost_;
  typedef decltype(
    static_cast<objects_collision_detector*>(nullptr)->iterate(
      *static_cast<get_cost<Visitor>*>(nullptr)
    ).begin()) objects_iterator;
  objects_iterator obj_i_;
  objects_iterator obj_end_;
  Visitor& visitor_;
  // unnecessarily HACK-y
//  tile_bounding_box tile_bbox_;
};


} // end namespace visit_collidable_tiles_and_objects_impl


template<typename Visitor>
void world::visit_collidable_tiles_and_objects(/*bounding_box bbox,*/ Visitor&& visitor) {
  const octant_number octant = visitor.octant();
  visit_collidable_tiles_and_objects_impl::get_cost<Visitor> cost{octant/*, bbox*/};
  auto objs_range = objects_exposed_to_collision_.iterate(cost);
  visit_collidable_tiles_and_objects_impl::tile_visitor<Visitor> tile{
    octant, objs_range.begin(), objs_range.end(),
    visitor/*,
    get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(bbox)*/
  };
  this->visit_collidable_tiles(tile);
  while(tile.obj_i_ != tile.obj_end_) {
    visitor.found(tile.obj_i_->object);
  }
}



#endif
