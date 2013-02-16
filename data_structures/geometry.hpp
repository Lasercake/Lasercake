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


#ifndef LASERCAKE_POLYGON_COLLISION_DETECTION_HPP__
#define LASERCAKE_POLYGON_COLLISION_DETECTION_HPP__

#include <cassert>
#include <vector>
#include <array>

#include "../utils.hpp"
#include "../units.hpp"
// possible TODO UNITS templatize instead of using fine_distance_units specifically;
#include "../world_constants.hpp"
#include "misc_structures.hpp"

// So that we can make implicit conversions between their and our bbox:
#include "bbox_collision_detector.hpp"


namespace geom {
// 64 bits though these ints are, you can't really do collision detection
// with them except for things with a max distance of about (???) bits between
// any of the parts of them.

typedef lint64_t geometry_int_type;
typedef geometry_int_type dimensionless;

typedef physical_quantity<non_normalized_rational<dimensionless>, seconds_t> rational_time;
typedef faux_optional<rational_time> optional_rational_time;

typedef physical_quantity<non_normalized_rational<dimensionless>, fine_distance_units_t> rational_coord;
typedef faux_optional<rational_coord> optional_rational_coord;

typedef non_normalized_rational<dimensionless> dimensionless_rational;
typedef faux_optional<dimensionless_rational> optional_dimensionless_rational;

typedef typename units_pow<fine_distance_units_t, 2>::type fine_distance_units2_t;
typedef typename units_pow<fine_distance_units_t, 3>::type fine_distance_units3_t;
typedef typename units_pow<fine_distance_units_t, 4>::type fine_distance_units4_t;
typedef typename units_pow<fine_distance_units_t, 5>::type fine_distance_units5_t;
typedef typename units_prod<fine_distance_units_t, pseudo_t>::type pseudo_fine_distance_units_t;
typedef typename units_prod<fine_distance_units2_t, pseudo_t>::type pseudo_fine_distance_units2_t;
typedef typename units_prod<fine_distance_units3_t, pseudo_t>::type pseudo_fine_distance_units3_t;
typedef typename units_prod<fine_distance_units4_t, pseudo_t>::type pseudo_fine_distance_units4_t;
typedef typename units_prod<fine_distance_units5_t, pseudo_t>::type pseudo_fine_distance_units5_t;

typedef physical_quantity<geometry_int_type, fine_distance_units_t> coord;
typedef physical_quantity<geometry_int_type, fine_distance_units2_t> coord2;
typedef physical_quantity<geometry_int_type, fine_distance_units3_t> coord3;
typedef physical_quantity<geometry_int_type, fine_distance_units4_t> coord4;
typedef physical_quantity<geometry_int_type, fine_distance_units5_t> coord5;
typedef physical_quantity<geometry_int_type, pseudo_fine_distance_units_t> pseudocoord;
typedef physical_quantity<geometry_int_type, pseudo_fine_distance_units2_t> pseudocoord2;
typedef physical_quantity<geometry_int_type, pseudo_fine_distance_units3_t> pseudocoord3;
typedef physical_quantity<geometry_int_type, pseudo_fine_distance_units4_t> pseudocoord4;
typedef physical_quantity<geometry_int_type, pseudo_fine_distance_units5_t> pseudocoord5;
typedef vector3<coord> vect;
typedef vector3<coord2> vect2;
typedef vector3<coord3> vect3;
typedef vector3<coord4> vect4;
typedef vector3<coord5> vect5;
typedef vector3<pseudocoord> pseudovect;
typedef vector3<pseudocoord2> pseudovect2;
typedef vector3<pseudocoord3> pseudovect3;
typedef vector3<pseudocoord4> pseudovect4;
typedef vector3<pseudocoord5> pseudovect5;

// TODO check if these ought be the same as the world_constants units
typedef typename units_prod<fine_distance_units_t, dim::second<(-1)>>::type geom_velocity_units_t;
typedef physical_quantity<geometry_int_type, geom_velocity_units_t> geom_velocity_scalar;
typedef vector3<geom_velocity_scalar> geom_velocity_vect;

struct polygon_collision_info_cache {
  polygon_collision_info_cache():is_valid(false){}
  bool is_valid;
  vect translation_amount;
  coord2 denom;
  coord2 a_times_denom;
  coord2 b_times_denom;
  int amount_twisted;
  std::vector<vect> adjusted_vertices;
};

class bounding_box {
public:
  bounding_box():is_anywhere_(false){}
  explicit bounding_box(vect loc):is_anywhere_(true),min_(loc),max_(loc){}
private:
  bounding_box(vect min, vect max):is_anywhere_(true),min_(min),max_(max){
    caller_correct_if(min(X) <= max(X) && min(Y) <= max(Y) && min(Z) <= max(Z),
        "Can't create zero-or-less-sized ::bounding_boxes (besides by default-construction)");
  }
  bool is_anywhere_;
  vect min_;
  vect max_;
public:

  // Named "constructors".
  static bounding_box min_and_max(vect min, vect max) {
    bounding_box result(min, max);
    return result;
  }
  static bounding_box spanning_two_points(vect v1, vect v2) {
    // when they're not already aligned nicely in min and max.
    bounding_box result;
    if(v1(X) < v2(X)) { result.min_.set(X, v1(X)); result.max_.set(X, v2(X)); }
    else              { result.min_.set(X, v2(X)); result.max_.set(X, v1(X)); }
    if(v1(Y) < v2(Y)) { result.min_.set(Y, v1(Y)); result.max_.set(Y, v2(Y)); }
    else              { result.min_.set(Y, v2(Y)); result.max_.set(Y, v1(Y)); }
    if(v1(Z) < v2(Z)) { result.min_.set(Z, v1(Z)); result.max_.set(Z, v2(Z)); }
    else              { result.min_.set(Z, v2(Z)); result.max_.set(Z, v1(Z)); }
    result.is_anywhere_ = true;
    return result;
  }
  static bounding_box min_and_size_minus_one(vect min, vect size_minus_one) {
    bounding_box result(min, min + size_minus_one);
    return result;
  }
  static bounding_box size_minus_one_and_max(vect size_minus_one, vect max) {
    bounding_box result(max - size_minus_one, max);
    return result;
  }
  static bounding_box min_and_size(vect min, vect size) {
    bounding_box result(min, min + (size - vect(1*fine_distance_units, 1*fine_distance_units, 1*fine_distance_units)));
    return result;
  }
  static bounding_box size_and_max(vect size, vect max) {
    bounding_box result(max - (size - vect(1*fine_distance_units, 1*fine_distance_units, 1*fine_distance_units)), max);
    return result;
  }

  vect min()const { return min_; }
  vect max()const { return max_; }
  vect size_minus_one()const { return max_-min_; }
  vect size()const { return size_minus_one() + vect(1*fine_distance_units, 1*fine_distance_units, 1*fine_distance_units); }
  
  coord min(which_dimension_type dim)const { return min_(dim); }
  coord max(which_dimension_type dim)const { return max_(dim); }
  coord size_minus_one(which_dimension_type dim)const { return max_(dim)-min_(dim); }
  coord size(which_dimension_type dim)const { return size_minus_one(dim) + 1*fine_distance_units; }

  void set_min_holding_max_constant(vect min) { min_ = min; }
  void set_max_holding_min_constant(vect max) { max_ = max; }
  void set_min_holding_max_constant(which_dimension_type dim, coord min) { min_.set(dim, min); }
  void set_max_holding_min_constant(which_dimension_type dim, coord max) { max_.set(dim, max); }
  //easy to write the rest as they become necessary

  bool is_anywhere()const { return is_anywhere_; }

  bool contains(vect const& v)const;
  bool overlaps(bounding_box const& o)const;
  bool volume_overlaps(bounding_box const& o)const;
  void combine_with(bounding_box const& o);
  void restrict_to(bounding_box const& o);
  void translate(vect t);

  // Implicit conversions to/from bbox_collision_detector bounding_box
  // for convenience interacting with it.
  // Note: the bbox_collision_detector bounding_box is unit-less,
  // so we sketchily remove the units from the values here.
  typedef collision_detector::bounding_box<64, 3> equivalent_collision_detector_bounding_box;
  operator equivalent_collision_detector_bounding_box()const {
    caller_correct_if(is_anywhere(), "Trying to pass nowhere-bounds to bbox_collision_detector, which has no such concept");
    
    typedef uint64_t uint_t; // a modulo type and modulo conversion
    typedef equivalent_collision_detector_bounding_box::coordinate_array uint_array;
    
    uint_array uint_min = {{
      uint_t(get_primitive_int(get(min(X), fine_distance_units))),
      uint_t(get_primitive_int(get(min(Y), fine_distance_units))),
      uint_t(get_primitive_int(get(min(Z), fine_distance_units)))
    }};
    uint_array uint_max = {{
      uint_t(get_primitive_int(get(max(X), fine_distance_units))),
      uint_t(get_primitive_int(get(max(Y), fine_distance_units))),
      uint_t(get_primitive_int(get(max(Z), fine_distance_units)))
    }};
    return equivalent_collision_detector_bounding_box::min_and_max(uint_min, uint_max);
  }
  bounding_box(equivalent_collision_detector_bounding_box const& b) {
    typedef int64_t int_t; // a modulo conversion
    min_ = vect(
      coord(int_t(b.min(X)), fine_distance_units),
      coord(int_t(b.min(Y)), fine_distance_units),
      coord(int_t(b.min(Z)), fine_distance_units)
    );
    max_ = vect(
      coord(int_t(b.max(X)), fine_distance_units),
      coord(int_t(b.max(Y)), fine_distance_units),
      coord(int_t(b.max(Z)), fine_distance_units)
    );
    is_anywhere_ = true;
  }
};
inline bool operator==(bounding_box const& b1, bounding_box const& b2) {
  return (b1.is_anywhere() == false && b2.is_anywhere() == false) ||
    (b1.is_anywhere() == b2.is_anywhere() && b1.min() == b2.min() && b1.max() == b2.max());
}
inline bool operator!=(bounding_box const& b1, bounding_box const& b2) {
  return !(b1 == b2);
}
inline std::ostream& operator<<(std::ostream& os, bounding_box const& bb) {
  return os << '[' << bb.min() << ", " << bb.max() << ']';
}

// TODO: rays and lines
struct line_segment {
  line_segment(std::array<vect, 2> ends):ends(ends){}
  line_segment(vect end1, vect end2):ends({{end1, end2}}){}
  std::array<vect, 2> ends;
  
  void translate(vect t);
  bounding_box bounds()const;
};
inline std::ostream& operator<<(std::ostream& os, line_segment const& l) {
  return os << '[' << l.ends[0] << "--" << l.ends[1] << ']';
}

struct convex_polygon {
  // The structure simply trusts that you will provide a convex, coplanar sequence of points. Failure to do so will result in undefined behavior.
  void setup_cache_if_needed()const;
  convex_polygon(std::vector<vect> const& vertices):vertices_(vertices){ caller_error_if(vertices_.size() < 3, "Trying to construct a polygon with fewer than three vertices"); } // And there's also something wrong with a polygon where all the points are collinear, but it's more complicated to check that.
  polygon_collision_info_cache const& get_cache()const { return cache_; }
  std::vector<vect> const& get_vertices()const { return vertices_; }
  void translate(vect t);
  bounding_box bounds()const;
private:
  std::vector<vect> vertices_;
  mutable polygon_collision_info_cache cache_;
};
inline std::ostream& operator<<(std::ostream& os, convex_polygon const& p) {
  os << '[';
  for(auto const& vertex : p.get_vertices()) {
    os << vertex << "--";
  }
  return os << ']';
}

struct convex_polyhedron {
public:
  struct edge {
    edge(uint8_t vert_1, uint8_t vert_2, uint8_t face_1, uint8_t face_2):vert_1(vert_1),vert_2(vert_2),face_1(face_1),face_2(face_2){}
    uint8_t vert_1;
    uint8_t vert_2;
    // these are "which numbered face", NOT "which index into face_info_"
    uint8_t face_1;
    uint8_t face_2;
  };
  
  // constructors are currently quadratic in the number of vertices
  convex_polyhedron(std::vector<vect> const& vs);
  convex_polyhedron(bounding_box const& bb);
  // construct as a sweep
  // Note that the signs of the max_error vector coordinates
  // MUST be equal to the signs of v coordinates, or 0.
  convex_polyhedron(convex_polyhedron const& start_shape, vect displacement, vect max_error);
  
  std::vector<vect> const& vertices()const { return vertices_; }
  std::vector<edge> const& edges()const { return edges_; }
  std::vector<uint8_t> const& face_info()const { return face_info_; }
  uint8_t const& num_faces()const { return num_faces_; }
  void translate(vect t);
  bounding_box bounds()const;
  
private:
  void init_other_info_from_vertices();
  uint8_t num_faces_;
  std::vector<vect> vertices_;
  std::vector<edge> edges_;
  std::vector<uint8_t> face_info_;
};
inline std::ostream& operator<<(std::ostream& os, convex_polyhedron const& p) {
  os << "polyhedron-vertices:[";
  for(auto const& vertex : p.vertices()) {
    os << vertex << ", ";
  }
  return os << ']';
}

class shape {
public:
  shape(                             ) : bounds_cache_is_valid_(false) {}
  shape(     line_segment const& init) : bounds_cache_is_valid_(false) { segments_ .push_back(init); }
  shape(   convex_polygon const& init) : bounds_cache_is_valid_(false) { polygons_ .push_back(init); }
  shape(convex_polyhedron const& init) : bounds_cache_is_valid_(false) { polyhedra_.push_back(init); }
  shape(     bounding_box const& init) : bounds_cache_is_valid_(false) { boxes_    .push_back(init); }
  shape(lasercake_vector<convex_polygon>::type const& init)
                                    : bounds_cache_is_valid_(false), polygons_(init) {}
  
  shape(shape const& o):bounds_cache_(o.bounds_cache_),bounds_cache_is_valid_(o.bounds_cache_is_valid_),segments_(o.segments_),polygons_(o.polygons_),polyhedra_(o.polyhedra_),boxes_(o.boxes_) {}
  
  void translate(vect t);
  
  bool intersects(shape const& other)const;
  bool intersects(bounding_box const& other)const;
  bool volume_intersects(shape const& other)const;
  bounding_box bounds()const;
  vect arbitrary_interior_point()const;
  
  lasercake_vector<     line_segment>::type const& get_segments ()const { return segments_ ; }
  lasercake_vector<   convex_polygon>::type const& get_polygons ()const { return polygons_ ; }
  lasercake_vector<convex_polyhedron>::type const& get_polyhedra()const { return polyhedra_; }
  lasercake_vector<     bounding_box>::type const& get_boxes    ()const { return boxes_    ; }
private:
  mutable bounding_box bounds_cache_;
  mutable bool bounds_cache_is_valid_;
  
  lasercake_vector<     line_segment>::type segments_;
  lasercake_vector<   convex_polygon>::type polygons_;
  lasercake_vector<convex_polyhedron>::type polyhedra_;
  lasercake_vector<     bounding_box>::type boxes_   ;
};
inline std::ostream& operator<<(std::ostream& os, shape const& sh) {
  os << "shape:{";
  if(sh.get_segments().size()) {
    os << " segments:";
    for(auto const& s : sh.get_segments()) {
      os << s << ';';
    }
  }
  if(sh.get_polygons().size()) {
    os << " polygons:";
    for(auto const& s : sh.get_polygons()) {
      os << s << ';';
    }
  }
  if(sh.get_polyhedra().size()) {
    os << " polyhedra:";
    for(auto const& s : sh.get_polyhedra()) {
      os << s << ';';
    }
  }
  if(sh.get_boxes().size()) {
    os << " boxes:";
    for(auto const& s : sh.get_boxes()) {
      os << s << ';';
    }
  }
  return os << '}';
}

// returns (was there an intersection?, what fraction of the length
//          of the line segment was the first)
optional_dimensionless_rational get_first_intersection(
                                  line_segment const& other, shape const& s);
optional_dimensionless_rational get_first_intersection(
                                  line_segment const& l, bounding_box const& bb);
optional_dimensionless_rational get_first_intersection(
                                  line_segment l, convex_polygon const& p);

// Polyhedron sweep intersection stuff... could possibly use some
// cleanup (here and in object_motion.cpp)
typedef std::pair<vect, vect2> base_point_and_outward_facing_normal;

struct polyhedron_planes_info_for_intersection {
  std::vector<base_point_and_outward_facing_normal> base_points_and_outward_facing_normals;
};

// These planes have a meaningful facing; they are isomorphic to half-spaces.
struct plane_as_base_point_and_normal {
  vect base_point;
  vect2 normal;
  
  plane_as_base_point_and_normal(vect base_point, vect2 normal):base_point(base_point),normal(normal){}
  plane_as_base_point_and_normal(){}
};


struct pair_of_parallel_supporting_planes {
  pair_of_parallel_supporting_planes(vect p1_base_point, vect p2_base_point, vect2 p1_to_p2_normal):p1_base_point(p1_base_point),p2_base_point(p2_base_point),p1_to_p2_normal(p1_to_p2_normal){}
  vect p1_base_point;
  vect p2_base_point;
  vect2 p1_to_p2_normal;
};
void populate_with_relating_planes(convex_polyhedron const& p1, convex_polyhedron const& p2, std::vector<pair_of_parallel_supporting_planes>& planes_collector);


struct potential_running_into_a_polyhedron_info {
  potential_running_into_a_polyhedron_info():is_anywhere(false){}
  bool is_anywhere;
  rational_time min;
  rational_time max;
  plane_as_base_point_and_normal arbitrary_plane_hit_first;
  plane_as_base_point_and_normal arbitrary_plane_of_closest_exclusion;
};

potential_running_into_a_polyhedron_info when_do_polyhedra_intersect(
      convex_polyhedron const& p1,
      convex_polyhedron const& p2,
      geom_velocity_vect velocity);


/*bool intersects(line_segment l, convex_polygon const& p);
bool intersects(convex_polygon const& p1, convex_polygon const& p2);
bool intersects(line_segment const& l, std::vector<convex_polygon> const& ps);
bool intersects(std::vector<convex_polygon> const& ps1, std::vector<convex_polygon> const& ps2);*/

} /* end namespace geom */

// TODO really? Also TODO rename this cpp/hpp?
using geom::bounding_box;
using geom::shape;

#endif

