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



/*

t1, t2, t3
vs.
l1, l2


Clockwiseness pointClockwisenessFromOrderedSegment(LineSegment ray, Point p) {
  const polygon_int_type temp = (((p.y - ray.e1.y) * (ray.e2.x - ray.e1.x)) - ((ray.e2.y - ray.e1.y) * (p.x - ray.e1.x)));
  if (temp > 0) return COUNTERCLOCKWISE;
  if (temp < 0) return CLOCKWISE;
  return COLLINEAR;
}

Interesting observation: Under a linear transformation, a line stays a line, and the topology stays the same. Here, we'll try to make the situation simpler by applying a skew:

move t3 to (0,0,0)

(0,0,0)(t1x t1y t1z)(t2x t2y t2z)
want t1z = 0 t2z = 0
addition to the z coordinate = a*x + b*y
EQ1) -t1z = a*t1x + b*t1y
EQ2) -t2z = a*t2x + b*t2y

t2x*EQ1 - t1x*EQ2) t2z*t1z-t1z*t2x = b(t1y*t2x - t2y*t1x)
b = (t2z*t1x-t1z*t2x) / (t1y*t2x - t2y*t1x)
since a and b are only different in an x vs y way, logically...
a = (t2z*t1y-t1z*t2y) / (t1x*t2y - t2x*t1y)
(denom == 0 iff t1 and t2 are collinear in the plane - just rotate the triangle 90 degrees until they're not.)

We could just do the computations at a factor of that denominator for a little while.
D = (t1x*t2y - t2x*t1y) = the denominator of a
Let A = (t2z*t1y-t1z*t2y) = a*D
Let B = -(t2z*t1x-t1z*t2x) = b*D
Then we'd get two transformed z values...

Ideally...
z1 = l1z + a*l1x + b*l1y
Multiply by D...
D*z1 = D*l1z + A*l1x + B*l1y

We only care about the *proportion* between z1 and z2, so Dz1/Dz2 are just as good, as long as we don't get rounding error.
Both negative or both positive -> no collision
Otherwise, ideally...
let l1xy = l1 projected onto the plane, etc
ltxy = the point where (l1, l2) intersects the plane
ltxy = ((0 - z1) / (z2 - z1)) l2xy + ((z2 - 0) / (z2 - z1)) l1xy
More practically...
ltxy = ((0 - D*z1) / (D*z2 - D*z1)) l2xy + ((D*z2 - 0) / (D*z2 - D*z1)) l1xy
ltxy = ((0 - D*z1) l2xy + (D*z2 - 0) l1xy) / (D*z2 - D*z1)

Now we can multiply everything by (D*z2 - D*z1)...
(D*z2 - D*z1)ltxy = D*z2*l1xy - D*z1*l2xy

And also multiply the triangle coordinates by (D*z2 - D*z1), and then do the clockwiseness check above!

This is a perfect accuracy collision check.
The downside is that it (in the worst case) has *four* factors of the distances between two points,
  (That's before the clockwiseness function, but the clockwiseness function can be implemented here without squaring the magnitude)
and so we couldn't use values that differ by more than the fourth log of our coordinate type,
namely... 64/4 bits, or 16 bits.
(D = diff^2, D(z2 - z1) = diff^3, D(z2 - z1)ltxy = diff^4, )

*/

/*

And for 2d line vs. line collisions...

skew the 'object' line onto an axis...
put ol1 at 0,0

skewing onto the x axis,
skew(x, y) = (x, y + ax)
a*ol2x + ol2y = 0
a = -(ol2y/ol2x)
D = ol2x
A = a*D = -ol2y

Dy1 = (D*skew(sl1))y = D*sl1y + A*sl1x
Dy2 = (D*skew(sl2))y = D*sl2y + A*sl2x

loc on x axis = lt = ((0 - y1) / (y2 - y1)) sl2x + ((y2 - 0) / (y2 - y1)) sl1x
lt*(Dy2 - Dy1) = sl1x * ol2x*y2 - sl2x * ol2x*y1

*/

#include <boost/type_traits.hpp>
#include <boost/range/iterator_range.hpp>

#include "polygon_collision_detection.hpp"

using boost::none;

typedef polygon_rational_type rational;




namespace {

vector3<polygon_int_type> cross_product(vector3<polygon_int_type> a, vector3<polygon_int_type> b) {
  return vector3<polygon_int_type>(
    (a(Y) * b(Z)) - (b(Y) * a(Z)),
    (a(Z) * b(X)) - (b(Z) * a(X)),
    (a(X) * b(Y)) - (b(X) * a(Y))
  );
}
vector3<polygon_int_type> plane_normal(vector3<polygon_int_type> p1, vector3<polygon_int_type> p2, vector3<polygon_int_type> p3) {
  return cross_product(p2 - p1, p3 - p1);
  /*return vector3<polygon_int_type>(
    (p1(Y) * (p2(Z) - p3(Z))) + (p2(Y) * (p3(Z) - p1(Z))) + (p3(Y) * (p1(Z) - p2(Z))),
    (p1(Z) * (p2(X) - p3(X))) + (p2(Z) * (p3(X) - p1(X))) + (p3(Z) * (p1(X) - p2(X))),
    (p1(X) * (p2(Y) - p3(Y))) + (p2(X) * (p3(Y) - p1(Y))) + (p3(X) * (p1(Y) - p2(Y)))
  );*/
}
vector3<polygon_int_type> forcedly_directed_plane_normal(vector3<polygon_int_type> p1, vector3<polygon_int_type> p2, vector3<polygon_int_type> p3, vector3<polygon_int_type> interior_direction) {
  const vector3<polygon_int_type> arbitrary_normal = plane_normal(p1, p2, p3);
  if (interior_direction.dot<polygon_int_type>(arbitrary_normal) > 0) {
    return -arbitrary_normal;
  }
  else {
    return arbitrary_normal;
  }
}

bool vectors_are_parallel(vector3<polygon_int_type> const& v1, vector3<polygon_int_type> const& v2) {
  return (v1(X) * v2(Y) == v2(X) * v1(Y)) && (v1(X) * v2(Z) == v2(X) * v1(Z)) && (v1(Y) * v2(Z) == v2(Y) * v1(Z));
}

faux_optional<std::pair<vector3<polygon_int_type>, vector3<polygon_int_type>>> get_excluding_face_onesided(std::vector<vector3<polygon_int_type>> const& vs1, polyhedron_planes_info_for_intersection ps2) {
  for (auto const& p : ps2.base_points_and_outward_facing_normals) {
    bool excludes_entire = true;
    for (vector3<polygon_int_type> v : vs1) {
      if (p.second.dot<polygon_int_type>(v - p.first) <= 0) {
        excludes_entire = false;
        break;
      }
    }
    if (excludes_entire) {
      return p;
    }
  }
  return none;
}

bool find_excluding_planes_onesided(std::vector<plane_as_base_point_and_normal> p1_planes, convex_polyhedron const& p2,
    std::vector<plane_as_base_point_and_normal>* planes_collector_1,
    std::vector<plane_as_base_point_and_normal>* planes_collector_2) {
  bool found_any_excluding_planes = false;
  for (auto const& plane : p1_planes) {
    bool excludes_entire = true;
    bool found_any_points = false;
    vector3<polygon_int_type> closest_point;
    polygon_int_type closest_dotprod;
    for (auto const& v : p2.vertices()) {
      const auto dotprod = plane.normal.dot<polygon_int_type>(v - plane.base_point);
      if (dotprod <= 0) {
        excludes_entire = false;
        break;
      }
      if (planes_collector_2 && ((!found_any_points) || (dotprod < closest_dotprod))) {
        found_any_points = true;
        closest_dotprod = dotprod;
        closest_point = v;
      }
    }
    if (excludes_entire) {
      if (!(planes_collector_1 || planes_collector_2)) return true;
      found_any_excluding_planes = true;
      if (planes_collector_1) {
        planes_collector_1->push_back(plane);
      }
      if (planes_collector_2) {
        assert(found_any_points);
        planes_collector_2->push_back(plane_as_base_point_and_normal(closest_point, -plane.normal));
      }
    }
  }
  return found_any_excluding_planes;
}

void populate_with_plane_info(convex_polyhedron const& p, std::vector<plane_as_base_point_and_normal>& collector) {
  for (uint8_t i = 0; i < p.face_info().size(); i += p.face_info()[i] + 1) {
    collector.push_back(plane_as_base_point_and_normal(
        p.vertices()[p.face_info()[i + 1]],
        // relying on the fact that the first three vertices of each face are in the proper order.
        plane_normal(p.vertices()[p.face_info()[i + 1]],
                     p.vertices()[p.face_info()[i + 2]],
                     p.vertices()[p.face_info()[i + 3]])
      ));
  }
}

bool planes_exclude_together_but_not_alone(plane_as_base_point_and_normal const& p1, plane_as_base_point_and_normal const& p2, std::vector<vector3<polygon_int_type>> const& verts) {
  bool excludes_entire1 = true;
  bool excludes_entire2 = true;
  for (auto const& v : verts) {
    const auto dotprod1 = p1.normal.dot<polygon_int_type>(v - p1.base_point);
    const auto dotprod2 = p2.normal.dot<polygon_int_type>(v - p2.base_point);
    if (dotprod1 <= 0) {
      excludes_entire1 = false;
      if (dotprod2 <= 0) {
        return false;
      }
    }
    if (dotprod2 <= 0) {
      excludes_entire2 = false;
    }
  }
  return !(excludes_entire1 || excludes_entire2);
}

} /* end anonymous namespace */
#if 0
struct pair_of_parallel_supporting_planes {
  pair_of_parallel_supporting_planes(vector3<polygon_int_type> p1_base_point, vector3<polygon_int_type> p2_base_point, vector3<polygon_int_type> p1_to_p2_normal):p1_base_point(p1_base_point),p2_base_point(p2_base_point),p1_to_p2_normal(p1_to_p2_normal){}
  vector3<polygon_int_type> p1_base_point;
  vector3<polygon_int_type> p2_base_point;
  vector3<polygon_int_type> p1_to_p2_normal;
};

void populate_with_relating_planes__faces_onesided(std::vector<plane_as_base_point_and_normal> pA_planes, convex_polyhedron const& pB, bool A_is_1, std::vector<pair_of_parallel_supporting_planes>& planes_collector) {
  for (auto const& plane : pA_planes) {
    bool found_any_points = false;
    vector3<polygon_int_type> closest_point;
    polygon_int_type closest_dotprod;
    for (auto const& v : pB.vertices()) {
      // We subtract base_point just to minimize integer size; the behavior should be the same either way.
      const auto dotprod = (v - plane.base_point).dot<polygon_int_type>(plane.normal);
      if ((!found_any_points) || (dotprod < closest_dotprod)) {
        found_any_points = true;
        closest_dotprod = dotprod;
        closest_point = v;
      }
    }
    if (A_is_1) planes_collector.push_back(pair_of_parallel_supporting_planes(
                  plane.base_point, closest_point   ,  plane.normal));
    else        planes_collector.push_back(pair_of_parallel_supporting_planes(
                  closest_point   , plane.base_point, -plane.normal));
  }
}

void populate_with_relating_planes(convex_polyhedron const& p1, convex_polyhedron const& p2, std::vector<pair_of_parallel_supporting_planes>& planes_collector) {
  std::vector<plane_as_base_point_and_normal> p1_planes; p1_planes.reserve(p1.num_faces());
  populate_with_plane_info(p1, p1_planes);
  std::vector<plane_as_base_point_and_normal> p2_planes; p2_planes.reserve(p2.num_faces());
  populate_with_plane_info(p2, p2_planes);
  planes_collector.reserve(p1_planes.size() + p2_planes.size());
  populate_with_relating_planes__faces_onesided(p1_planes, p2, true, planes_collector);
  populate_with_relating_planes__faces_onesided(p2_planes, p1, false, planes_collector);

  for (auto e1 : p1.edges()) {
    const auto e1v1 = p1.vertices()[e1.vert_1];
    const auto e1v2 = p1.vertices()[e1.vert_2];
    const auto e1d = e1v2 - e1v1;
    const auto e1f1 = p1_planes[e1.face_1];
    const auto e1f2 = p1_planes[e1.face_2];

    // a vector:
    // - perpendicular to e1d
    // - parallel to the plane of e1f1
    // - pointing towards e1
    // (the term 'discriminant' is just because it discriminates between cases, not any standard mathematical usage.)
    auto e1f1_discriminant = cross_product(e1d, e1f1.normal);
    if (e1f1_discriminant.dot<polygon_int_type>(e1f2.normal) < 0) e1f1_discriminant = -e1f1_discriminant;
    auto e1f2_discriminant = cross_product(e1d, e1f2.normal);
    if (e1f2_discriminant.dot<polygon_int_type>(e1f1.normal) < 0) e1f2_discriminant = -e1f2_discriminant;
    
    for (auto e2 : p2.edges()) {
      const auto e2v1 = p2.vertices()[e2.vert_1];
      const auto e2v2 = p2.vertices()[e2.vert_2];
      const auto e2d = e2v2 - e2v1;
      const auto e2f1 = p2_planes[e2.face_1];
      const auto e2f2 = p2_planes[e2.face_2];

      // TODO: perhaps only compute these once for each edge in p2
      auto e2f1_discriminant = cross_product(e2d, e2f1.normal);
      if (e2f1_discriminant.dot<polygon_int_type>(e2f2.normal) < 0) e2f1_discriminant = -e2f1_discriminant;
      auto e2f2_discriminant = cross_product(e2d, e2f2.normal);
      if (e2f2_discriminant.dot<polygon_int_type>(e2f1.normal) < 0) e2f2_discriminant = -e2f2_discriminant;

      vector3<polygon_int_type> proposed_normal;
      if (vectors_are_parallel(e1d, e2d)) {
        // a vector:
        // - perpendicular to the lines
        // - in the plane containing the two lines
        proposed_normal = cross_product(e1d, cross_product(e1d, e2v1 - e1v1));
      }
      else {
        proposed_normal = cross_product(e1d, e2d);
      }

      // The proposed normal has to be on the proper side of all four discriminants
      // in order to represent a meaningful plane that's not a face.
      // However it might just be in the wrong sense.
      {
        auto dotprod1 = proposed_normal.dot<polygon_int_type>(e1f1_discriminant);
        if (dotprod1 == 0) continue;
        if (dotprod1 < 0) {
          proposed_normal = -proposed_normal;
        }
      }
      if (proposed_normal.dot<polygon_int_type>(e1f2_discriminant) <= 0) continue;
      if (proposed_normal.dot<polygon_int_type>(e2f1_discriminant) >= 0) continue;
      if (proposed_normal.dot<polygon_int_type>(e2f2_discriminant) >= 0) continue;
      
      planes_collector.push_back(pair_of_parallel_supporting_planes(
                  e1v1, e2v1, proposed_normal));
    }
  }
}
#endif

struct potential_running_into_a_polyhedron_info {
  potential_running_into_a_polyhedron_info():min(none),max(none),is_anywhere(true){}
  potential_running_into_a_polyhedron_info(optional_rational min,optional_rational max):min(min),max(max),is_anywhere((!min) || (!max) || (*min <= *max)){}
  potential_running_into_a_polyhedron_info(boost::none_t):min(none),max(none),is_anywhere(false){}
  potential_running_into_a_polyhedron_info(rational loc, plane_as_base_point_and_normal plane):min(loc),max(loc){
    planes_hit_first.push_back(plane);
  }
  potential_running_into_a_polyhedron_info(rational min, boost::none_t, plane_as_base_point_and_normal plane):min(min),max(none){
    planes_hit_first.push_back(plane);
  }
  bool is_anywhere;
  optional_rational min;
  optional_rational max;
  std::unordered_map<plane_as_base_point_and_normal> planes_hit_first;
  plane_as_base_point_and_normal arbitrary_plane_of_closest_exclusion;
  void combine_with(potential_running_into_a_polyhedron_info const& other) {
    if (!is_anywhere) {
      *this = other;
    }
    else {
      if (other.min == min) {
        planes_hit_first.combin e(other.planes_hit_first);
      }
      else if (other.min < min) {
        planes_hit_first = other.planes_hit_first;
        min = other.min;
      }
      
      if (other.max > max) {
        max = other.max;
      }
    }
  }
  void intersect_with(potential_running_into_a_polyhedron_info const& other);
}

potential_running_into_a_polyhedron_info when_is_point_within_planes(vector3<polygon_int_type> point, vector3<polygon_int_type> velocity, std::vector<plane_as_base_point_and_normal> const& planes) {
  potential_running_into_a_polyhedron_info result;
  for (auto const& plane : planes) {
    // We subtract base_point just to minimize integer size; the behavior should be the same either way.
    const auto vel_dotprod = velocity.dot<polygon_int_type>(plane.normal);
    const auto disp_dotprod = (plane.base_point - point).dot<polygon_int_type>(plane.normal);
    if (vel_dotprod == 0) {
      if (disp_dotprod < 0) result = potential_running_into_a_polyhedron_info(none);
    }
    else {
      const rational moment(disp_dotprod, vel_dotprod);
      if (vel_dotprod > 0) {
        result.intersect_with(potential_running_into_a_polyhedron_info(none, moment));
      }
      else {
        result.intersect_with(potential_running_into_a_polyhedron_info(moment, none, plane));
      }
      if (!result.is_anywhere) break;
    }
  }
  return result;
}

potential_running_into_a_polyhedron_info when_do_polyhedra_intersect(convex_polyhedron const& p1, convex_polyhedron const& p2, vector3<polygon_int_type> velocity) {
  potential_running_into_a_polyhedron_info result(none);
  std::vector<plane_as_base_point_and_normal> p1_planes; p1_planes.reserve(p1.num_faces());
  populate_with_plane_info(p1, p1_planes);
  for (auto v : p2.vertices()) {
    result.combine_with(when_is_point_within_planes(v, velocity, p1_planes));
    if (!result.is_anywhere) return result;
  }
  std::vector<plane_as_base_point_and_normal> p2_planes; p2_planes.reserve(p2.num_faces());
  populate_with_plane_info(p2, p2_planes);
  for (auto v : p1.vertices()) {
    result.combine_with(when_is_point_within_planes(v, -velocity, p2_planes));
    if (!result.is_anywhere) return result;
  }

  // At a moment when two polyhedra start/stop intersecting,
  // either a vertex is touching a face, or an edge is touching an edge.
  // Handle the latter here.
  for (auto e1 : p1.edges()) {
    const auto e1v1 = p1.vertices()[e1.vert_1];
    const auto e1v2 = p1.vertices()[e1.vert_2];
    const auto e1d = e1v2 - e1v1;
    if (vectors_are_parallel(e1d, velocity)) continue;

    const auto e1_travel_plane_normal = cross_product(e1d, velocity);

    // a vector:
    // - perpendicular to e1d
    // - parallel to the plane of e1f1
    // - pointing towards e1 from any other point on e1f1
    // (the term 'discriminant' is just because it discriminates between cases, not any standard mathematical usage.)
    auto e1f1_discriminant = cross_product(e1d, e1f1.normal);
    if (e1f1_discriminant.dot<polygon_int_type>(e1f2.normal) < 0) e1f1_discriminant = -e1f1_discriminant;
    auto e1f2_discriminant = cross_product(e1d, e1f2.normal);
    if (e1f2_discriminant.dot<polygon_int_type>(e1f1.normal) < 0) e1f2_discriminant = -e1f2_discriminant;

    for (auto e2 : p2.edges()) {
      const auto e2v1 = p2.vertices()[e2.vert_1];
      const auto e2v2 = p2.vertices()[e2.vert_2];
      const auto e2v1side = sign((e2v1 - e1v1).dot<polygon_int_type>(e1_travel_plane_normal));
      // If one of the ends is on the same plane, that end (a vertex) was already caught where it entered the volume
      if (e2v1side == 0) continue;
      const auto e2v2side = sign((e2v2 - e1v1).dot<polygon_int_type>(e1_travel_plane_normal));
      if (e2v2side == 0) continue;
      if (e2v2side == e2v1side) continue;
      const auto e2d = e2v2 - e2v1;
      if (vectors_are_parallel(e1d, e2d)) continue;
      if (vectors_are_parallel(e2d, velocity)) continue;
      const auto e2_travel_plane_normal = cross_product(e2d, velocity);
      const auto e1v1side = sign((e2v1 - e1v1).dot<polygon_int_type>(e2_travel_plane_normal));
      if (e1v1side == 0) continue;
      const auto e1v2side = sign((e2v2 - e1v1).dot<polygon_int_type>(e2_travel_plane_normal));
      if (e1v2side == 0) continue;
      if (e1v2side == e1v1side) continue;

      // Now we know that the two lines intersect when they're coplanar.
      // But don't report this if there's already an 

      // TODO: perhaps only compute these once for each edge in p2
      auto e2f1_discriminant = cross_product(e2d, e2f1.normal);
      if (e2f1_discriminant.dot<polygon_int_type>(e2f2.normal) < 0) e2f1_discriminant = -e2f1_discriminant;
      auto e2f2_discriminant = cross_product(e2d, e2f2.normal);
      if (e2f2_discriminant.dot<polygon_int_type>(e2f1.normal) < 0) e2f2_discriminant = -e2f2_discriminant;

      vector3<polygon_int_type> proposed_normal = cross_product(e1d, e2d);

      // The proposed normal has to be on the proper side of all four discriminants
      // in order to represent a meaningful plane that's not a face.
      // However it might just be in the wrong sense.
      {
        auto dotprod1 = proposed_normal.dot<polygon_int_type>(e1f1_discriminant);
        if (dotprod1 == 0) continue;
        if (dotprod1 < 0) {
          proposed_normal = -proposed_normal;
        }
      }
      if (proposed_normal.dot<polygon_int_type>(e1f2_discriminant) <= 0) continue;
      if (proposed_normal.dot<polygon_int_type>(e2f1_discriminant) >= 0) continue;
      if (proposed_normal.dot<polygon_int_type>(e2f2_discriminant) >= 0) continue;


      const auto vel_dotprod = velocity.dot<polygon_int_type>(proposed_normal);
      if (vel_dotprod == 0) continue;
      const auto disp_dotprod = (e2v1 - e1v1).dot<polygon_int_type>(proposed_normal);
      const rational moment(disp_dotprod, vel_dotprod);
      if (vel_dotprod > 0) {
        result.combine_with(potential_running_into_a_polyhedron_info(moment));
      }
      else {
        result.combine_with(potential_running_into_a_polyhedron_info(moment, plane));
      }
    }
  }

  if (result.is_anywhere) {
    assert(!result.planes_hit_first.empty());
  }
  return result;
}

bool find_excluding_planes(convex_polyhedron const& p1, convex_polyhedron const& p2, std::vector<plane_as_base_point_and_normal>* planes_collector_1, std::vector<plane_as_base_point_and_normal>* planes_collector_2) {
  bool found_any_excluding_planes = false;
  std::vector<plane_as_base_point_and_normal> p1_planes; p1_planes.reserve(p1.num_faces());
  populate_with_plane_info(p1, p1_planes);
  if (find_excluding_planes_onesided(p1_planes, p2, planes_collector_1, planes_collector_2)) {
    if (!(planes_collector_1 || planes_collector_2)) return true;
    found_any_excluding_planes = true;
  }
  
  std::vector<plane_as_base_point_and_normal> p2_planes; p2_planes.reserve(p2.num_faces());
  populate_with_plane_info(p2, p2_planes);
  if (find_excluding_planes_onesided(p2_planes, p1, planes_collector_2, planes_collector_1)) {
    if (!(planes_collector_1 || planes_collector_2)) return true;
    found_any_excluding_planes = true;
  }
  /*
  for (auto const& e1 : p1.edges()) {
    for (auto const& e2 : p2.edges()) {
      const auto e1v1 = p1.vertices()[e1.vert_1];
      const auto e1v2 = p1.vertices()[e1.vert_2];
      const auto e2v1 = p2.vertices()[e2.vert_1];
      const auto e2v2 = p2.vertices()[e2.vert_2];
      if (planes_exclude_together_but_not_alone(
             p1_planes[e1.face_1], p1_planes[e1.face_2], p2.vertices()) &&
          planes_exclude_together_but_not_alone(
             p2_planes[e2.face_1], p2_planes[e2.face_2], p1.vertices())) {
        
        assert(!vectors_are_parallel(e2v2 - e2v1, e1v2 - e1v1));
        if (!(planes_collector_1 || planes_collector_2)) return true;
        found_any_excluding_planes = true;
        const auto normal = forcedly_directed_plane_normal(e1v1,e1v2,e1v1 + (e2v2-e2v1), e1v1 - e2v1);
        if (planes_collector_1) planes_collector_1->push_back(plane_as_base_point_and_normal(e1v1, normal));
        if (planes_collector_2) planes_collector_2->push_back(plane_as_base_point_and_normal(e2v1, -normal));
      }
    }
  }*/
  
  return found_any_excluding_planes;
}

faux_optional<std::pair<vector3<polygon_int_type>, vector3<polygon_int_type>>> get_excluding_face(std::vector<vector3<polygon_int_type>> const& vs1, polyhedron_planes_info_for_intersection ps1, std::vector<vector3<polygon_int_type>> const& vs2, polyhedron_planes_info_for_intersection ps2) {
  if (auto result = get_excluding_face_onesided(vs1, ps2)) return result;
  if (auto result = get_excluding_face_onesided(vs2, ps1)) return result;
  return none;
}

void compute_planes_info_for_intersection(convex_polyhedron const& ph, polyhedron_planes_info_for_intersection& collector) {
  for (uint8_t i = 0; i < ph.face_info().size(); i += ph.face_info()[i] + 1) {
    collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
        ph.vertices()[ph.face_info()[i + 1]],
        // relying on the fact that the first three vertices of each face are in the proper order.
        plane_normal(ph.vertices()[ph.face_info()[i + 1]], ph.vertices()[ph.face_info()[i + 2]], ph.vertices()[ph.face_info()[i + 3]])
      ));
  }
}

faux_optional<std::pair<vector3<polygon_int_type>, vector3<polygon_int_type>>> get_excluding_face(std::vector<vector3<polygon_int_type>> const& vs, polyhedron_planes_info_for_intersection ps, convex_polyhedron const& other) {
  polyhedron_planes_info_for_intersection other_ps;
  compute_planes_info_for_intersection(other, other_ps);
  return get_excluding_face(vs, ps, other.vertices(), other_ps);
}

// TODO do sweep_intersects(stuff, bounding_box) in a more efficient way
// (This was just the simplest implementation I could think of)
void compute_info_for_intersection(bounding_box const& bb, std::vector<vector3<polygon_int_type>>& vertex_collector, polyhedron_planes_info_for_intersection& plane_collector) {
  vertex_collector.push_back(bb.min());
  vertex_collector.push_back(vector3<polygon_int_type>(bb.max(X),bb.min(Y),bb.min(Z)));
  vertex_collector.push_back(vector3<polygon_int_type>(bb.min(X),bb.max(Y),bb.min(Z)));
  vertex_collector.push_back(vector3<polygon_int_type>(bb.max(X),bb.max(Y),bb.min(Z)));
  vertex_collector.push_back(vector3<polygon_int_type>(bb.min(X),bb.min(Y),bb.max(Z)));
  vertex_collector.push_back(vector3<polygon_int_type>(bb.max(X),bb.min(Y),bb.max(Z)));
  vertex_collector.push_back(vector3<polygon_int_type>(bb.min(X),bb.max(Y),bb.max(Z)));
  vertex_collector.push_back(bb.max());
  plane_collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
      bb.min(), vector3<polygon_int_type>(-1,0,0)));
  plane_collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
      bb.min(), vector3<polygon_int_type>(0,-1,0)));
  plane_collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
      bb.min(), vector3<polygon_int_type>(0,0,-1)));
  plane_collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
      bb.max(), vector3<polygon_int_type>(1,0,0)));
  plane_collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
      bb.max(), vector3<polygon_int_type>(0,1,0)));
  plane_collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
      bb.max(), vector3<polygon_int_type>(0,0,1)));
}
faux_optional<std::pair<vector3<polygon_int_type>, vector3<polygon_int_type>>> get_excluding_face(std::vector<vector3<polygon_int_type>> const& vs, polyhedron_planes_info_for_intersection ps, bounding_box const& other) {
  std::vector<vector3<polygon_int_type>> other_vs;
  polyhedron_planes_info_for_intersection other_ps;
  compute_info_for_intersection(other, other_vs, other_ps);
  return get_excluding_face(vs, ps, other_vs, other_ps);
}

faux_optional<std::pair<vector3<polygon_int_type>, vector3<polygon_int_type>>> get_excluding_face(convex_polyhedron const& p1, convex_polyhedron const& p2) {
  polyhedron_planes_info_for_intersection ps1;
  polyhedron_planes_info_for_intersection ps2;
  compute_planes_info_for_intersection(p1, ps1);
  compute_planes_info_for_intersection(p2, ps2);
  return get_excluding_face(p1.vertices(), ps1, p2.vertices(), ps2);
}

faux_optional<std::pair<vector3<polygon_int_type>, vector3<polygon_int_type>>> get_excluding_face(convex_polyhedron const& p, bounding_box const& bb) {
  polyhedron_planes_info_for_intersection ps1;
  std::vector<vector3<polygon_int_type>> vs2;
  polyhedron_planes_info_for_intersection ps2;
  compute_planes_info_for_intersection(p, ps1);
  compute_info_for_intersection(bb, vs2, ps2);
  return get_excluding_face(p.vertices(), ps1, vs2, ps2);
}

template<typename VectorType, int ArraySize> class arrayvector {
private:
  std::array<typename VectorType::value_type, ArraySize> first_n_values_;
  int size_;
  VectorType* real_vector_;
public:
  arrayvector():size_(0),real_vector_(NULL){}
  int size()const { return size_; }
  typename VectorType::value_type& operator[](int idx) {
    if (idx < ArraySize) {
      return first_n_values_[idx];
    }
    else {
      caller_correct_if(real_vector_, "out of bounds arrayvector access");
      return (*real_vector_)[idx - ArraySize];
    }
  }
  typename VectorType::value_type& back() {
    return (*this)[size_ - 1];
  }
  typename VectorType::value_type& front() {
    return first_n_values_[0];
  }
  void push_back(typename VectorType::value_type value) {
    if (size_ < ArraySize) {
      first_n_values_[size_] = value;
    }
    else {
      if (!real_vector_) {
        real_vector_ = new VectorType();
        //std::cerr << "arrayvector size exceeded: " << ArraySize << "\n";
      }
      real_vector_->push_back(value);
    }
    ++size_;
  }
  void pop_back() {
    if(size_ > ArraySize) {
      real_vector_->pop_back();
    }
    --size_;
  }
  bool empty()const { return size_ == 0; }
  ~arrayvector() {
    if (real_vector_) delete real_vector_;
  }
  class iterator : public boost::iterator_facade<iterator, typename VectorType::value_type, boost::bidirectional_traversal_tag> {
    public:
      iterator() : av_(NULL),which_(0) {}
      explicit iterator(arrayvector* av_, int which_) : av_(av_),which_(which_) {}
    private:
    friend class boost::iterator_core_access;
      void increment() {++which_;}
      void decrement() {--which_;}
      bool equal(iterator const& other)const { return which_ == other.which_; }
      typename VectorType::value_type& dereference()const { return (*av_)[which_]; }
      arrayvector* av_;
      int which_;
  };
  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, size_); }
};

void convex_polyhedron::init_other_info_from_vertices() {
  caller_correct_if(vertices_.size() <= 255, "You can't make a polyhedron with more than 255 points.");
  caller_correct_if(vertices_.size() >= 4, "You can't make a polyhedron with fewer than 4 points.");
  std::vector<vector3<polygon_int_type>> const& vs = vertices_;
  
  // Algorithm: For each point, expand the polyhedron to include that point.
  // When you're done, you'll have the correct answer.
  // Complexity: Quadratic in the number of points.
  struct face_building_info {
    face_building_info(){}
    arrayvector<std::vector<int>, 8> verts;
    vector3<polygon_int_type> normal;
    int final_idx;
  };
  typedef std::list<face_building_info>::iterator face_reference_t;
  struct line_building_info {
    face_reference_t face_1;
    face_reference_t face_2;
    int vert_1;
    int vert_2;
  };
  std::list<face_building_info> faces;
  std::list<line_building_info> lines;
  
  // Hack: Start by making a tetrahedron with the first four non-coplanar verts.
  std::array<int, 4> first_four_noncoplanar_verts{{0,-1,-1,-1}};
  for (int i = 1; i < (int)vs.size(); ++i) {
    if (vs[i] != vs[0]) {
      first_four_noncoplanar_verts[1] = i;
      break;
    }
  }
  assert(first_four_noncoplanar_verts[1] != -1);
  for (int i = first_four_noncoplanar_verts[1]+1; i < (int)vs.size(); ++i) {
    if (!vectors_are_parallel(vs[i] - vs[0], vs[first_four_noncoplanar_verts[1]] - vs[0])) {
      first_four_noncoplanar_verts[2] = i;
      break;
    }
  }
  assert(first_four_noncoplanar_verts[2] != -1);
  auto f3norm = plane_normal(vs[0], vs[first_four_noncoplanar_verts[1]], vs[first_four_noncoplanar_verts[2]]);
  for (int i = first_four_noncoplanar_verts[2]+1; i < (int)vs.size(); ++i) {
    if ((vs[i] - vs[0]).dot<polygon_int_type>(f3norm) != 0) {
      first_four_noncoplanar_verts[3] = i;
      break;
    }
  }
  assert(first_four_noncoplanar_verts[3] != -1);
  
  
  std::array<face_reference_t, 4> first_four_faces;
  for(int q = 0; q < 4; ++q) {
    faces.push_front(face_building_info());
    first_four_faces[q] = faces.begin();
    face_building_info& f = faces.front();
    const int i1 = first_four_noncoplanar_verts[q];
    const int i2 = first_four_noncoplanar_verts[(q+1) % 4];
    const int i3 = first_four_noncoplanar_verts[(q+2) % 4];
    const int i4 = first_four_noncoplanar_verts[(q+3) % 4];
    f.verts.push_back(i2); f.verts.push_back(i3); f.verts.push_back(i4);
    f.normal = plane_normal(vs[i2],vs[i3],vs[i4]);
    if ((vs[i2] - vs[i1]).dot<polygon_int_type>(f.normal) < 0) f.normal = -f.normal;
  }
  for(int q = 0; q < 4; ++q) {
    for(int j = q + 1; j < 4; ++j) {
      line_building_info l;
      l.vert_1 = first_four_noncoplanar_verts[q];
      l.vert_2 = first_four_noncoplanar_verts[j];
      l.face_1 = first_four_faces[((q != 0) && (j != 0)) ? 0 : ((q != 1) && (j != 1)) ? 1 : 2];
      l.face_2 = first_four_faces[((q != 3) && (j != 3)) ? 3 : ((q != 2) && (j != 2)) ? 2 : 1];
      lines.push_back(l);
      assert_if_ASSERT_EVERYTHING(vs[l.vert_1] != vs[l.vert_2]);
    }
  }
  
  for (int i = 0; i < (int)vs.size(); ++i) {
    // These four would be eliminated anyway, but we skip them for speed.
    if ((i == first_four_noncoplanar_verts[0]) ||
        (i == first_four_noncoplanar_verts[1]) ||
        (i == first_four_noncoplanar_verts[2]) ||
        (i == first_four_noncoplanar_verts[3])) continue;
    /*std::cerr << "==================  Step info:\n";
    std::cerr << "Verts: " << vs.size() << "\n";
    std::cerr << "Lines: " << lines.size() << "\n";
    std::cerr << "Faces: " << faces.size() << "\n";
    for (auto& f : faces) {
      std::cerr << "Face of size " << f.verts.size() << "\n";
    }*/
    bool exposed = false;
    for (auto& f : faces) {
      assert(!f.verts.empty());
      polygon_int_type dotprod = (vs[i] - vs[f.verts.front()]).dot<polygon_int_type>(f.normal);
      //std::cerr << i << "!" << dotprod << "!\n";
      if (dotprod > 0) {
        exposed = true;
        break;
      }
    }
    if (exposed) {
      arrayvector<std::vector<face_reference_t>, 16> new_and_old_faces;
      arrayvector<std::vector<face_reference_t>, 8> old_faces;
      arrayvector<std::vector<int>, 16> verts_of_potential_new_lines;
      arrayvector<std::vector<int>, 16> verts_of_preexisting_lines;
      for (auto l = lines.begin(); l != lines.end(); ) {
        assert(l->face_1 != faces.end());
        assert(l->face_2 != faces.end());
        auto& f1 = *l->face_1;
        auto& f2 = *l->face_2;
        assert(!f1.verts.empty());
        assert(!f2.verts.empty());
        const polygon_int_type dotprod1 = (vs[i] - vs[f1.verts.front()]).dot<polygon_int_type>(f1.normal);
        const polygon_int_type dotprod2 = (vs[i] - vs[f2.verts.front()]).dot<polygon_int_type>(f2.normal);
        //std::cerr << dotprod1 << ", " << dotprod2 << "\n";
        // Both <=0: we're inside both, the line is unaffected
        // Both >0: we're outside both, the line will just be purged
        if ((dotprod1 == 0) && (dotprod2 == 0)) {
          int deleted_vert = -1;
          for (int dim = 0; dim < num_dimensions; ++dim) {
            if (vs[l->vert_1](dim) != vs[l->vert_2](dim)) {
              if ((vs[l->vert_1](dim) < vs[l->vert_2](dim)) == (vs[l->vert_1](dim) < vs[i](dim))) {
                deleted_vert = l->vert_2;
                verts_of_preexisting_lines.push_back(l->vert_2);
                l->vert_2 = i;
                assert(vs[l->vert_1] != vs[l->vert_2]);
              }
              else {
                deleted_vert = l->vert_1;
                verts_of_preexisting_lines.push_back(l->vert_1);
                l->vert_1 = i;
                assert(vs[l->vert_1] != vs[l->vert_2]);
              }
              break;
            }
          }
          assert(deleted_vert != -1);
          
          // Go through and erase, but make sure we leave f1.verts.back() constant,
          // because we're counting on inserting at the end to avoid inserting duplicates.
          for (auto& eraser : f1.verts) {
            if (eraser == deleted_vert) {
              eraser = f1.verts[f1.verts.size() - 2];
              f1.verts[f1.verts.size() - 2] = f1.verts.back();
              f1.verts.pop_back();
              break; // it's unique
            }
          }
          for (auto& eraser : f2.verts) {
            if (eraser == deleted_vert) {
              eraser = f2.verts[f2.verts.size() - 2];
              f2.verts[f2.verts.size() - 2] = f2.verts.back();
              f2.verts.pop_back();
              break; // it's unique
            }
          }
          if (f1.verts.back() != i) f1.verts.push_back(i);
          if (f2.verts.back() != i) f2.verts.push_back(i);
          ++l;
        }
        else if ((dotprod1 > 0) != (dotprod2 > 0)) {
          verts_of_potential_new_lines.push_back(l->vert_1);
          verts_of_potential_new_lines.push_back(l->vert_2);
          if (dotprod1 == 0) {
            old_faces.push_back(l->face_1);
            new_and_old_faces.push_back(l->face_1);
            if (f1.verts.back() != i) f1.verts.push_back(i);
            lines.erase(l++);
          }
          else if (dotprod2 == 0) {
            old_faces.push_back(l->face_2);
            new_and_old_faces.push_back(l->face_2);
            if (f2.verts.back() != i) f2.verts.push_back(i);
            lines.erase(l++);
          }
          else {
            faces.push_front(face_building_info());
            face_building_info& new_face = faces.front();
            new_and_old_faces.push_back(faces.begin());
            if (dotprod1 > 0) l->face_1 = faces.begin();
            else              l->face_2 = faces.begin();
            new_face.verts.push_back(i);
            new_face.verts.push_back(l->vert_1);
            new_face.verts.push_back(l->vert_2);
            new_face.normal = plane_normal(vs[i], vs[l->vert_1], vs[l->vert_2]);
            for (int q = 0; q < 4; ++q) {
              const int idx = first_four_noncoplanar_verts[q];
              if ((i != idx) && (l->vert_1 != idx) && (l->vert_2 != idx)) {
                if ((vs[i] - vs[idx]).dot<polygon_int_type>(new_face.normal) < 0) {
                  new_face.normal = -new_face.normal;
                  break;
                }
              }
            }
            ++l;
          }
        }
        else if ((dotprod1 > 0) && (dotprod2 > 0)) lines.erase(l++);
        else ++l;
      }
      
      for (auto f = faces.begin(); f != faces.end(); ) {
        polygon_int_type dotprod = (vs[i] - vs[*f->verts.begin()]).dot<polygon_int_type>(f->normal);
        if (dotprod > 0) faces.erase(f++);
        else ++f;
      }
      for (int lv = 0; lv < (int)verts_of_potential_new_lines.size(); ) {
        bool eliminate = false;
        for (auto pv : verts_of_preexisting_lines) {
          if (pv == verts_of_potential_new_lines[lv]) {
            eliminate = true;
            break;
          }
        }
        if (eliminate) {
          verts_of_potential_new_lines[lv] = verts_of_potential_new_lines.back();
          verts_of_potential_new_lines.pop_back();
        }
        else {
          verts_of_preexisting_lines.push_back(verts_of_potential_new_lines[lv]);
          ++lv;
        }
      }
      //std::cerr << "EEEEEEEEE " << verts_of_potential_new_lines.size() << ", " << new_and_old_faces.size() << ", " << old_faces.size() << "\n";
      for (int lv : verts_of_potential_new_lines) {
        line_building_info l;
        int num_faces_including = 0;
        l.face_1 = faces.end();
        for (auto fr : new_and_old_faces) {
          if (fr != l.face_1) {
            const polygon_int_type dotprod = (vs[lv] - vs[*fr->verts.begin()]).dot<polygon_int_type>(fr->normal);
            //std::cerr<<dotprod<<"...\n";
            assert(dotprod <= 0);
            if (dotprod == 0) {
              if (num_faces_including == 0) {
                l.face_1 = fr;
              }
              else {
                l.face_2 = fr;
                assert(l.face_2 != l.face_1);
              }
              ++num_faces_including;
              if (num_faces_including == 2) break;
            }
          }
        }
        if (num_faces_including == 2) {
          l.vert_1 = i;
          l.vert_2 = lv;
          assert(vs[i] != vs[lv]);
          lines.push_back(l);
        }
        else {
          for (auto fr : old_faces) {
            for (auto& eraser : fr->verts) {
              if (eraser == lv) {
                eraser = fr->verts.back();
                fr->verts.pop_back();
                break; // it's unique
              }
            }
          }
        }
      }
    }
  }
  
  std::vector<bool> existences_of_points(vertices_.size(), false);
  for (auto& f : faces) {
    for (int v : f.verts) {
      existences_of_points[v] = true;
    }
  }
  std::vector<int> vertex_id_map(vertices_.size());
  int next_vert_id = 0;
  for (int i = 0; i < (int)vertices_.size(); ++i) {
    if (existences_of_points[i]) {
      vertices_[next_vert_id] = vertices_[i];
      vertex_id_map[i] = next_vert_id;
      ++next_vert_id;
    }
  }
  vertices_.erase(vertices_.begin() + next_vert_id, vertices_.end());
  num_faces_ = faces.size();
  int which_face = 0;
  for (auto& f : faces) {
    const int face_idx = face_info_.size();
    f.final_idx = which_face++;
    face_info_.push_back(f.verts.size());
    for (int vid : f.verts) {
      face_info_.push_back(vertex_id_map[vid]);
    }
    // Hack: we rely on the fact that this makes the *first three* points have a reliable outwards
    // normal vector. So sort them that way.
    // Really we should sort ALL OF THEM that way
    if (plane_normal(vertices_[face_info_[face_idx + 1]], vertices_[face_info_[face_idx + 2]], vertices_[face_info_[face_idx + 3]]).dot<polygon_int_type>(f.normal) < 0) {
      const int temp = face_info_[face_idx + 2];
      face_info_[face_idx + 2] = face_info_[face_idx + 3];
      face_info_[face_idx + 3] = temp;
    }
  }
  for (auto const& l : lines) {
    //std::cerr << l.vert_1 << l.vert_2 << vertices_.size();
    //if (vertex_id_map.find(l.vert_1) != vertex_id_map.end() && vertex_id_map.find(l.vert_2) != vertex_id_map.end())
    edges_.push_back(edge(
      vertex_id_map[l.vert_1], vertex_id_map[l.vert_2],
      l.face_1->final_idx, l.face_2->final_idx
      ));
  }
}
convex_polyhedron::convex_polyhedron(std::vector<vector3<polygon_int_type>> const& vs):vertices_(vs) {
  init_other_info_from_vertices();
}
convex_polyhedron::convex_polyhedron(bounding_box const& bb) {
  vertices_.push_back(bb.min());
  vertices_.push_back(vector3<polygon_int_type>(bb.max(X),bb.min(Y),bb.min(Z)));
  vertices_.push_back(vector3<polygon_int_type>(bb.min(X),bb.max(Y),bb.min(Z)));
  vertices_.push_back(vector3<polygon_int_type>(bb.max(X),bb.max(Y),bb.min(Z)));
  vertices_.push_back(vector3<polygon_int_type>(bb.min(X),bb.min(Y),bb.max(Z)));
  vertices_.push_back(vector3<polygon_int_type>(bb.max(X),bb.min(Y),bb.max(Z)));
  vertices_.push_back(vector3<polygon_int_type>(bb.min(X),bb.max(Y),bb.max(Z)));
  vertices_.push_back(bb.max());
  init_other_info_from_vertices();
}

convex_polyhedron::convex_polyhedron(convex_polyhedron const& start_shape, vector3<polygon_int_type> displacement, vector3<polygon_int_type> max_error) {
  // Compute a maximum sweep with possible rounding error.
  // The resulting polyhedron will represent the union of all
  // of p + a*dirs[0] + b*dirs[1] + c*dirs[2] + d.dirs[3]
  // for p \in start_shape, a,b,c,d \in [0,1]
  
  std::array<vector3<polygon_int_type>, 4> dirs = {{
    vector3<polygon_int_type>(max_error(X), 0, 0),
    vector3<polygon_int_type>(0, max_error(Y), 0),
    vector3<polygon_int_type>(0, 0, max_error(Z)),
    displacement
  }};
  // Collapse parallel dirs.
  for (int dim = 0; dim < num_dimensions; ++dim) {
    if ((displacement((dim+1) % num_dimensions) == 0) && (displacement((dim+2) % num_dimensions) == 0)) {
      dirs[3][dim] += dirs[dim](dim);
      dirs[dim][dim] = 0;
      //std::cerr << "eliminating parallel " << dir << "\n";
    }
  }
  std::array<bool, 4> dir_existences = {{
    dirs[0] != vector3<polygon_int_type>(0,0,0),
    dirs[1] != vector3<polygon_int_type>(0,0,0),
    dirs[2] != vector3<polygon_int_type>(0,0,0),
    dirs[3] != vector3<polygon_int_type>(0,0,0)
  }};
  
  // In order from "fewest to most components" to make things more efficient in one place.
  // Each bit (1 << n) represents whether the nth direction vector was used.
  /*static const std::array<uint8_t, 16> combinations_by_idx = {{
    0,
    0x1, 0x2, 0x4, 0x8,
    0x3, 0x5, 0x9, 0x6, 0xA, 0xC,
    0x7, 0xB, 0xD, 0xE,
    0xF
  }};*/
  const std::array<vector3<polygon_int_type>, 16> vectors_by_combo = {{
    /*0x0*/ vector3<polygon_int_type>(0,0,0),
    /*0x1*/ dirs[0],
    /*0x2*/           dirs[1],
    /*0x3*/ dirs[0] + dirs[1],
    /*0x4*/                     dirs[2],
    /*0x5*/ dirs[0]           + dirs[2],
    /*0x6*/           dirs[1] + dirs[2],
    /*0x7*/ dirs[0] + dirs[1] + dirs[2],
    /*0x8*/                               dirs[3],
    /*0x9*/ dirs[0]                     + dirs[3],
    /*0xA*/           dirs[1]           + dirs[3],
    /*0xB*/ dirs[0] + dirs[1]           + dirs[3],
    /*0xC*/                     dirs[2] + dirs[3],
    /*0xD*/ dirs[0]           + dirs[2] + dirs[3],
    /*0xE*/           dirs[1] + dirs[2] + dirs[3],
    /*0xF*/ dirs[0] + dirs[1] + dirs[2] + dirs[3]
  }};
  /*static const std::array<uint16_t, 3> B_and_not_v_combos = {{
    (1<<0x1) | (1<<0x3) | (1<<0x5) | (1<<0x7),
    (1<<0x2) | (1<<0x3) | (1<<0x6) | (1<<0x7),
    (1<<0x4) | (1<<0x5) | (1<<0x6) | (1<<0x7),
  }};
  static const std::array<uint16_t, 3> v_and_not_B_combos = {{
    (1<<0x8) | (1<<0xA) | (1<<0xC) | (1<<0xE),
    (1<<0x8) | (1<<0x9) | (1<<0xC) | (1<<0xD),
    (1<<0x8) | (1<<0x9) | (1<<0xA) | (1<<0xB),
  }};*/
  static const std::array<uint16_t, 4> combos_including_dir = {{
    (1<<0x1) | (1<<0x3) | (1<<0x5) | (1<<0x7) | (1<<0x9) | (1<<0xB) | (1<<0xD) | (1<<0xF),
    (1<<0x2) | (1<<0x3) | (1<<0x6) | (1<<0x7) | (1<<0xA) | (1<<0xB) | (1<<0xE) | (1<<0xF),
    (1<<0x4) | (1<<0x5) | (1<<0x6) | (1<<0x7) | (1<<0xC) | (1<<0xD) | (1<<0xE) | (1<<0xF),
    (1<<0x8) | (1<<0x9) | (1<<0xA) | (1<<0xB) | (1<<0xC) | (1<<0xD) | (1<<0xE) | (1<<0xF),
  }};
  
  // Eliminate whatever vertices we can eliminate with complete confidence;
  // We'll leave some incorrect (i.e. strictly interior) vertices around,
  //   but init_other_info_from_vertices() will catch them.
  std::vector<arrayvector<std::vector<vector3<polygon_int_type>>, 8>> normals_by_point_idx(start_shape.vertices().size());
  for (uint8_t i = 0; i < start_shape.face_info().size(); i += start_shape.face_info()[i] + 1) {
    for (uint8_t j = 0; j < start_shape.face_info()[i]; ++j) {
      normals_by_point_idx[start_shape.face_info()[i + j + 1]].push_back(
        // relying on the fact that the first three vertices of each face are in the proper order.
        plane_normal(
          start_shape.vertices()[start_shape.face_info()[i + 1]],
          start_shape.vertices()[start_shape.face_info()[i + 2]],
          start_shape.vertices()[start_shape.face_info()[i + 3]]));
    }
  }
  
  for (uint8_t vert_idx = 0; vert_idx < start_shape.vertices().size(); ++vert_idx) {
    uint16_t existences_of_translates = 0xffff;
    for (int dir = 0; dir < 4; ++dir) {
      if (dir_existences[dir]) {
        bool p_plus_vector_is_shadowed = true;
        bool p_minus_vector_is_shadowed = true;
        for (auto const& normal : normals_by_point_idx[vert_idx]) {
          const polygon_int_type dotprod = dirs[dir].dot<polygon_int_type>(normal);
          if (dotprod > 0) {
            p_plus_vector_is_shadowed = false;
            if (!p_minus_vector_is_shadowed) break;
          }
          if (dotprod < 0) {
            p_minus_vector_is_shadowed = false;
            if (!p_plus_vector_is_shadowed) break;
          }
        }
        assert(!(p_plus_vector_is_shadowed && p_minus_vector_is_shadowed));
        
        if (p_plus_vector_is_shadowed) {
          existences_of_translates &= ~combos_including_dir[dir];
        }
        if (p_minus_vector_is_shadowed) {
          existences_of_translates &= combos_including_dir[dir];
        }
      }
      else {
        existences_of_translates &= ~combos_including_dir[dir];
      }
    }
    
    for (int combo = 0; combo < 16; ++combo) {
      if (existences_of_translates & (1 << combo)) {
        vertices_.push_back(start_shape.vertices()[vert_idx]+vectors_by_combo[combo]);
      }
    }
  }
  
  init_other_info_from_vertices();
 
#if 0
  return;
  
  std::unordered_multimap<uint8_t, vector3<polygon_int_type>> normals_by_point_idx;
  for (uint8_t i = 0; i < ph.face_info().size(); i += ph.face_info()[i] + 1) {
    // relying on the fact that the first three vertices of each face are in the proper order.
    auto normal = plane_normal(ph.vertices()[ph.face_info()[i + 1]], ph.vertices()[ph.face_info()[i + 2]], ph.vertices()[ph.face_info()[i + 3]]);
    for (uint8_t j = 0; j < ph.face_info()[i]; ++j) {
      normals_by_point_idx.insert(std::make_pair(ph.face_info()[i + j + 1], normal));
    }
  }
  
  std::vector<uint16_t> existences_of_translates(ph.vertices().size(), 0xffff);
  for (uint8_t i = 0; i < ph.vertices().size(); ++i) {
    vector3<polygon_int_type> const& p = ph.vertices()[i];
    // There are sixteen possible vertices translated from P -
    // p, p+v, p+e1, p+v+e1, p+e2, p+e2+v, p+e2+e1, ... in all the possible combinations
    // Each of them would be a base point for p's planes (as well as for any extra planes
    // we generate, but that doesn't become relevant immediately).
    // Therefore, we can mount p's planes at each point, and if they obscure any of the
    // other points, eliminate those other points.
    // Of course, some of that information is redundant.
    // If, for instance, p obscures p+e1, then for any x, p+x obscures p+x+e1.
    // So we only have to make the following checks:
    /*for (int j = 1; j < 16; ++j) { // (the 0 vector is meaningless for this)
      const uint8_t combo = combinations_by_idx[j];
      //std::cerr << "grr, " << std::hex << j << ", " << (int)combo << ", " << existences_of_translates[i] << std::dec << "\n";
      // Wait, no, we can't skip already-eliminated vectors because they may be still relevant if
      // they don't have all the coordinates; e.g. if we test (e1) and it eliminates (e2) but
      // (e2+v) needs to eliminate (v) then we have to test (e2) to do that.
      // (Formerly:
      // Skip already-eliminated vectors
      //if (existences_of_translates[i] & (1 << combinations_by_idx[j])) {
        // If any of the directions is zero, we effectively ignore that direction.
        // (We aren't looking at the empty combo here.)
        if (vectors_by_combo[combo] == vector3<polygon_int_type>(0,0,0)) {
          existences_of_translates[i] &= ~combinations_with_only_shared_dirs_by_idx[j];
        }
        else {
          bool p_plus_vector_is_shadowed = true;
          bool p_minus_vector_is_shadowed = true;
          const auto range = normals_by_point_idx.equal_range(i);
          for (auto pair : boost::make_iterator_range(range.first, range.second)) {
            auto normal = pair.second;
            const polygon_int_type dotprod = vectors_by_combo[combo].dot<polygon_int_type>(normal);
            //std::cerr << p << normal << vectors_by_combo[combo] << "\n";
            if (dotprod > 0) {
              p_plus_vector_is_shadowed = false;
              if (!p_minus_vector_is_shadowed) break;
            }
            if (dotprod < 0) {
              p_minus_vector_is_shadowed = false;
              if (!p_plus_vector_is_shadowed) break;
            }
          }
          //std::cerr << "foo\n";
          if (p_plus_vector_is_shadowed) {
          //std::cerr << "BLAH\n";
            existences_of_translates[i] &= ~combinations_with_only_shared_dirs_by_idx[j];
          }
          if (p_minus_vector_is_shadowed) {
          //std::cerr << "DANWRWK\n";
            existences_of_translates[i] &= ~combinations_with_no_shared_dirs_by_idx[j];
          }
        }
      //}
    }*/
    
    // Three hacks: (1) We rely on the fact that the error is in the same direction in each dimension as v,
    //    and so (2) the combos (errorx + errory + errorz) and (v) are always concealed, so we eliminate them here,
    //    and (3) they wouldn't necessarily be eliminated anywhere else.
    // All other invalid vectors get eliminated properly though.
    if ((v(X) != 0) && (v(Y) != 0) && (v(Z) != 0)) {
      existences_of_translates[i] &= (~(1<<0x7)) & (~(1<<0x8));
    }
    
    // Eliminate all combos that include a zero vector.
    for (int dir = 0; dir < 4; ++dir) {
      if (dirs[dir] == vector3<polygon_int_type>(0,0,0)) existences_of_translates[i] &= ~combos_including_dir[dir];
    }
    
    // Eliminate all combos that are obscured by another combo.
    // I'm sure this could be optimized.
    std::array<int, 4> di1 = {{0,0,0,0}};
    std::array<int, 4> di2 = {{0,0,0,0}};
    for(di1[0] = 0; di1[0] < 2; ++di1[0]) {
      if (di1[0] && (dirs[0] == vector3<polygon_int_type>(0,0,0))) continue;
      for(di1[1] = 0; di1[1] < 2; ++di1[1]) {
        if (di1[1] && (dirs[1] == vector3<polygon_int_type>(0,0,0))) continue;
        for(di1[2] = 0; di1[2] < 2; ++di1[2]) {
          if (di1[2] && (dirs[2] == vector3<polygon_int_type>(0,0,0))) continue;
          for(di1[3] = 0; di1[3] < 2; ++di1[3]) {
            if (di1[3] && (dirs[3] == vector3<polygon_int_type>(0,0,0))) continue;
      for(di2[0] = 0; di2[0] < 2; ++di2[0]) {
      if (di2[0] && (dirs[0] == vector3<polygon_int_type>(0,0,0))) continue;
        for(di2[1] = 0; di2[1] < 2; ++di2[1]) {
        if (di2[1] && (dirs[1] == vector3<polygon_int_type>(0,0,0))) continue;
          for(di2[2] = 0; di2[2] < 2; ++di2[2]) {
          if (di2[2] && (dirs[2] == vector3<polygon_int_type>(0,0,0))) continue;
            for(di2[3] = 0; di2[3] < 2; ++di2[3]) {
            if (di2[3] && (dirs[3] == vector3<polygon_int_type>(0,0,0))) continue;
        vector3<polygon_int_type> diff_vector(0,0,0);
        for (int dir = 0; dir < 4; ++dir) {
          if (di2[dir]) diff_vector += dirs[dir];
          if (di1[dir]) diff_vector -= dirs[dir];
        }
        if (diff_vector != vector3<polygon_int_type>(0,0,0)) {
          bool point_2_is_shadowed = true;
          const auto range = normals_by_point_idx.equal_range(i);
          for (auto pair : boost::make_iterator_range(range.first, range.second)) {
            auto normal = pair.second;
            const polygon_int_type dotprod = diff_vector.dot<polygon_int_type>(normal);
            //std::cerr << p << normal << vectors_by_combo[combo] << "\n";
            if (dotprod > 0) {
              point_2_is_shadowed = false;
              break;
            }
          }
          if (point_2_is_shadowed) {
            existences_of_translates[i] &= ~(1 << (
              (di2[0] ? (1 << 0) : 0) |
              (di2[1] ? (1 << 1) : 0) |
              (di2[2] ? (1 << 2) : 0) |
              (di2[3] ? (1 << 3) : 0)
            ));
          }
        }
      }}}}
    }}}}
    
    // Special case: If v has exactly one zero-entry, it does some extra obscuring.
    for (int dim = 0; dim < num_dimensions; ++dim) {
      if (v(dim) == 0) {
        const int dim2 = (dim+1) % num_dimensions;
        const int dim3 = (dim+2) % num_dimensions;
        if ((v(dim2) != 0) && (v(dim3) != 0)) {
          // If you have eight coplanar points, two of them are subsumed by the others
          //std::cerr << (int)i << std::hex << ", " << existences_of_translates[i] << ", " << ((~(combos_including_dir[dim] | (1 << (1 << 3)))) & 0xffff) << "\n";
          if (existences_of_translates[i] == ((~(combos_including_dir[dim] | (1 << (1 << 3)))) & 0xffff)) {
          //std::cerr << "erjeajirjeairjiearearaerae";
            existences_of_translates[i] &= ~(1 << ((1 << dim2) | (1 << dim3)));
            //existences_of_translates[i] &= ~(1 << (1 << 3));
          }
        
          for (int j = 0; j < 2; ++j) {
            const int dimA = j ? dim2 : dim3;
            const int dimB = j ? dim3 : dim2;
            if ((existences_of_translates[i] & B_and_not_v_combos[dimB]) && (existences_of_translates[i] & v_and_not_B_combos[dimB])) {
              // One may eliminate the other.
              //vector3<polygon_int_type> const& B = dirs[dimB];
              optional_rational v_eliminating_min = none;
              optional_rational v_eliminating_max = none;
              optional_rational B_eliminating_min = none;
              optional_rational B_eliminating_max = none;
              if ((v(dimA) < 0) == (v(dimB) < 0)) {
                v_eliminating_max = rational(v(dimB), v(dimA));
                B_eliminating_max = rational(v(dimB), v(dimA));
              }
              else {
                v_eliminating_min = rational(v(dimB), v(dimA));
                B_eliminating_min = rational(v(dimB), v(dimA));
              }
              const auto range = normals_by_point_idx.equal_range(i);
              for (auto pair : boost::make_iterator_range(range.first, range.second)) {
                auto normal = pair.second;
                //  std::cerr << normal << dimA << dimB << "\n";
                if (normal(dimB) == 0) {
                  if (normal(dimA) == 0) {
                    // it's a wash - that normal represents the same plane we're considering,
                    // so it contains everything
                  }
                  else if ((normal(dimA) < 0) == (v(dimA) < 0)) {
                    // hack - completely eliminate v from being eliminated
                    v_eliminating_max = rational(0);
                    v_eliminating_min = rational(1);
                  }
                  else {
                    // hack - completely eliminate B from being eliminated
                    B_eliminating_max = rational(0);
                    B_eliminating_min = rational(1);
                  }
                }
                else {
                  const rational slope(-normal(dimA), normal(dimB));
                  if ((normal(dimB) < 0) == (v(dimA) < 0)) {
                    if (!v_eliminating_max || (*v_eliminating_max > slope)) v_eliminating_max = slope;
                    if (!B_eliminating_min || (*B_eliminating_min < slope)) B_eliminating_min = slope;
                  }
                  else {
                    if (!B_eliminating_max || (*B_eliminating_max > slope)) B_eliminating_max = slope;
                    if (!v_eliminating_min || (*v_eliminating_min < slope)) v_eliminating_min = slope;
                  }
                }
              }
              
              const bool keep_B = B_eliminating_min && B_eliminating_max && (*B_eliminating_min > *B_eliminating_max);
              const bool keep_v = v_eliminating_min && v_eliminating_max && (*v_eliminating_min > *v_eliminating_max);
              assert (keep_v || keep_B);
              if (!keep_B) existences_of_translates[i] &= ~B_and_not_v_combos[dimB];
              if (!keep_v) existences_of_translates[i] &= ~v_and_not_B_combos[dimB];
            }
          }
        }
      }
    }
    
    // The target structure will have all non-eliminated points as vertices
    for (int combo = 0; combo < 16; ++combo) {
      if (existences_of_translates[i] & (1 << combo)) {
        vertex_collector.push_back(p+vectors_by_combo[combo]);
      }
    }
    // If, for any two directions held constant, all four points that include those
    // two directions exist, then there's a quadrilateral face that we need
    // to take notes about. (If only three exist, it's a different situation.)
    // There are twenty-four possible faces of this nature.
    for (int j = 0; j < 4; ++j) {
      for (int k = j + 1; k < 4; ++k) {
        for (int jb = 0; jb < 2; ++jb) {
          for (int kb = 0; kb < 2; ++kb) {
            const int plane_dir_1 = (j != 0 && k != 0) ? 0 :
                                    (j != 1 && k != 1) ? 1 :
                                                         2;
            const int plane_dir_2 = (j != 3 && k != 3) ? 3 :
                                    (j != 2 && k != 2) ? 2 :
                                                         1;
            int base_combo = 0;
            if (jb) base_combo |= (1 << j);
            if (kb) base_combo |= (1 << k);
            uint16_t required_combos =
              (1 << base_combo) |
              (1 << (base_combo | (1 << plane_dir_1))) |
              (1 << (base_combo | (1 << plane_dir_2))) |
              (1 << (base_combo | (1 << plane_dir_1) | (1 << plane_dir_2)));
            if ((existences_of_translates[i] & required_combos) == required_combos) {
              const vector3<polygon_int_type> base_point = p+vectors_by_combo[base_combo];
              const vector3<polygon_int_type> point2 = base_point + dirs[plane_dir_1];
              const vector3<polygon_int_type> point3 = base_point + dirs[plane_dir_2];
              vector3<polygon_int_type> normal = plane_normal(base_point, point2, point3);
              // It might be in the wrong direction...
              for (uint8_t v = 0; v < ph.vertices().size(); ++v) {
                if (v != i) {
                  polygon_int_type dotprod = (ph.vertices()[v] - ph.vertices()[i]).dot<polygon_int_type>(normal);
                  if (dotprod < 0) {
                    break;
                  }
                  if (dotprod > 0) {
                    normal = -normal;
                    break;
                  }
                }
              }
              plane_collector.base_points_and_outward_facing_normals.push_back(
                 std::make_pair(base_point, normal));
            }
          }
        }
      }
    }
  }
  
  for (std::pair<uint8_t, uint8_t> l : ph.edges()) {
    // It can also create planes of motion in the directions of the rounding error or movement.
    // There are (theoretically) 12+12+8 = 32 possible lines that can spawn planes this way.
    // All the other directions have to be held constant,
    // except that if this is a line in the direction of one of the vectors, it's okay (and in fact, inevitable)
    // for the two endpoints to differ in that direction.
    //std::cerr << "hi " << ph.vertices()[l.first] << ph.vertices()[l.second] << "\n";
    
    // Special case: If we're parallel to one of the directions of movement, we need to ignore the difference between not-including-that-direction and including-that-direction.
    const vector3<polygon_int_type> line_vector = ph.vertices()[l.second] - ph.vertices()[l.first];
    int parallel_dir = -1;
    //    std::cerr << line_vector << "\n";
    for (int dir = 0; dir < 4; ++dir) {
      if (dirs[dir] != vector3<polygon_int_type>(0,0,0)) {
        if (vectors_are_parallel(line_vector, dirs[dir])) {
     //   std::cerr << dir << "\n";
          assert(parallel_dir == -1);
          parallel_dir = dir;
        }
      }
    }
    uint8_t parallel_dir_bit = (parallel_dir == -1) ? 0 : (1 << parallel_dir);
        
    for (int i = 0; i < 4; ++i) {
      if (i == parallel_dir) continue;
      
      // In certain cases, some bogus planes would pass through the other checks. Rule out here ones that are obscured even by the polyhedron moving along the line itself.
      bool clear_to_create = false;
      const auto range = normals_by_point_idx.equal_range(l.first);
      for (auto pair : boost::make_iterator_range(range.first, range.second)) {
        auto normal = pair.second;
        const auto range2 = normals_by_point_idx.equal_range(l.second);
        for (auto pair2 : boost::make_iterator_range(range2.first, range2.second)) {
          if (pair2.second == normal) {
            if (dirs[i].dot<polygon_int_type>(normal) < 0) {
              clear_to_create = true;
            }
          }
        }
      }
      if (!clear_to_create) continue;
      
      // Special case: If v has exactly one zero-entry, make sure to handle the hexagonal planes that can be generated.
      uint8_t ignore_coplanar_v_bit = 0;
      uint8_t non_coplanar_dimensions = 0;
      if (i < 3) {
        const int dim2 = (i+1) % num_dimensions;
        const int dim3 = (i+2) % num_dimensions;
        if (((v(dim2) == 0) && (line_vector(dim2) == 0)) || ((v(dim3) == 0) && (line_vector(dim3) == 0))) {
          ignore_coplanar_v_bit = (1 << 3);
          non_coplanar_dimensions = ((line_vector(dim2) == 0) ? (1 << dim2) : 0) | ((line_vector(dim3) == 0) ? (1 << dim3) : 0);
        }
      }
      
      //std::cerr << i << "lol\n";
      for (int combo = 0; combo < 15; ++combo) { // (this is "for each combo with less than four components")
        if (
                (!(combo & (1 << i))
             && (!(combo & parallel_dir_bit)))
            ) { // i.e. "for each combo without those directions in it"
          std::array<vector3<polygon_int_type>, 2> moved_line_ends;
          std::array<bool, 2> ends_found = {{false, false}};
          for (int j = 0; j < 2; ++j) {
            const auto v = j ? l.second : l.first;
            uint16_t combos_check1 =
              (1 << (combo)) |
              (1 << (combo | parallel_dir_bit));
            if (!(combo & (ignore_coplanar_v_bit | non_coplanar_dimensions))) {
              combos_check1 |= combos_check1 << ignore_coplanar_v_bit;
            }
            uint16_t combos_check2 =
              (1 << (combo | (1 << i))) |
              (1 << (combo | (1 << i) | parallel_dir_bit));
            if (!(combo & (ignore_coplanar_v_bit | non_coplanar_dimensions))) {
              combos_check2 |= combos_check2 << ignore_coplanar_v_bit;
            }
            if (existences_of_translates[v] & combos_check1) {
              if (existences_of_translates[v] & combos_check2) {
                ends_found[j] = true;
                moved_line_ends[j] = ph.vertices()[v]+vectors_by_combo[combo];
              }
            }
          }
          if (ends_found[0] && ends_found[1]) {
            /*const int dir_to_construct_interior_point_in =
              ((i != 0) && (parallel_dir != 0)) ? 0 :
              ((i != 1) && (parallel_dir != 1)) ? 1 : 2;
            const vector3<polygon_int_type> interior_direction =
               (combo & (1 << dir_to_construct_interior_point_in)) ?
               (-dirs[dir_to_construct_interior_point_in]) :
               dirs[dir_to_construct_interior_point_in];*/
            vector3<polygon_int_type> normal = plane_normal(moved_line_ends[0], moved_line_ends[1], moved_line_ends[0] + dirs[i]);
            // It might be in the wrong direction...
            for (uint8_t i = 0; i < ph.vertices().size(); ++i) {
              if ((i != l.first) && (i != l.second)) {
                polygon_int_type dotprod = (ph.vertices()[i] - moved_line_ends[0]).dot<polygon_int_type>(normal);
                if (dotprod < 0) {
                  break;
                }
                if (dotprod > 0) {
                  normal = -normal;
                  break;
                }
              }
            }
            // Quadratic hack: Eliminate cases where the plane is wroooong
            // We only actually generate wrong planes when...
            bool wroooong = false;
            for (auto v : vertex_collector) {
              if ((v - moved_line_ends[0]).dot<polygon_int_type>(normal) > 0) {
                wroooong = true;
                break;
              }
            }
            if (!wroooong) {
              plane_collector.base_points_and_outward_facing_normals.push_back(
                 std::make_pair(moved_line_ends[0], normal));
              // I used these redundant things to help me visualize this earlier.
              /*plane_collector.base_points_and_outward_facing_normals.push_back(
                 std::make_pair(moved_line_ends[1], normal));
              plane_collector.base_points_and_outward_facing_normals.push_back(
                 std::make_pair(moved_line_ends[0] + dirs[i], normal));
              plane_collector.base_points_and_outward_facing_normals.push_back(
                 std::make_pair(moved_line_ends[1] + dirs[i], normal));*/
            }
          }
        }
      }
    }
  }
  for (uint8_t i = 0; i < ph.face_info().size(); i += ph.face_info()[i] + 1) {
    // Each plane becomes the farthest-out plane that it can be.
    // relying on the fact that the first three vertices of each face are in the proper order.
    const vector3<polygon_int_type> normal = plane_normal(ph.vertices()[ph.face_info()[i + 1]], ph.vertices()[ph.face_info()[i + 2]], ph.vertices()[ph.face_info()[i + 3]]);
    vector3<polygon_int_type> farthest_out_point = ph.vertices()[ph.face_info()[i+1]];
    for (uint8_t j = 0; j < ph.face_info()[i]; ++j) {
      for (int combo = 0; combo < 16; ++combo) {
        if (existences_of_translates[ph.face_info()[i + 1 + j]] & (1 << combo)) {
          auto const& v = ph.vertices()[ph.face_info()[i + 1 + j]] + vectors_by_combo[combo];
          if ((v - farthest_out_point).dot<polygon_int_type>(normal) > 0) {
            farthest_out_point = v;
          }
        }
      }
    }
    plane_collector.base_points_and_outward_facing_normals.push_back(std::make_pair(
        farthest_out_point,
        normal
      ));
  }
#endif
}


bool bounding_box::contains(vector3<polygon_int_type> const& v)const {
  if (!is_anywhere_) return false;
  return (v.x >= min_.x && v.x <= max_.x &&
          v.y >= min_.y && v.y <= max_.y &&
          v.z >= min_.z && v.z <= max_.z);
}
bool bounding_box::overlaps(bounding_box const& o)const {
  return is_anywhere_ && o.is_anywhere_
     && min_.x <= o.max_.x && o.min_.x <= max_.x
     && min_.y <= o.max_.y && o.min_.y <= max_.y
     && min_.z <= o.max_.z && o.min_.z <= max_.z;
}
void bounding_box::combine_with(bounding_box const& o) {
       if (!o.is_anywhere_) {            return; }
  else if (!  is_anywhere_) { *this = o; return; }
  else {
    if (o.min_.x < min_.x) min_.x = o.min_.x;
    if (o.min_.y < min_.y) min_.y = o.min_.y;
    if (o.min_.z < min_.z) min_.z = o.min_.z;
    if (o.max_.x > max_.x) max_.x = o.max_.x;
    if (o.max_.y > max_.y) max_.y = o.max_.y;
    if (o.max_.z > max_.z) max_.z = o.max_.z;
  }
}
void bounding_box::restrict_to(bounding_box const& o) {
       if (!  is_anywhere_) {                      return; }
  else if (!o.is_anywhere_) { is_anywhere_ = false; return; }
  else {
    if (o.min_.x > min_.x) min_.x = o.min_.x;
    if (o.min_.y > min_.y) min_.y = o.min_.y;
    if (o.min_.z > min_.z) min_.z = o.min_.z;
    if (o.max_.x < max_.x) max_.x = o.max_.x;
    if (o.max_.y < max_.y) max_.y = o.max_.y;
    if (o.max_.z < max_.z) max_.z = o.max_.z;
    if (min_.x > max_.x || min_.y > max_.y || min_.z > max_.z) is_anywhere_ = false;
  }
}

/*
shape::shape(bounding_box const& init): bounds_cache_is_valid(true) {
  bounds_cache = init;
  if (!init.is_anywhere()) return;
  
  for (int i = 0; i < 3; ++i) {
    std::vector<vector3<polygon_int_type>> vertices1, vertices2;
    vector3<polygon_int_type> base2(init.min()); base2[i] = init.max(i);
    vector3<polygon_int_type> diff1(0,0,0), diff2(0,0,0);
    diff1[(i+1)%3] = init.max((i+1)%3) - init.min((i+1)%3);
    diff2[(i+2)%3] = init.max((i+2)%3) - init.min((i+2)%3);
    vertices1.push_back(init.min()                );
    vertices1.push_back(init.min() + diff1        );
    vertices1.push_back(init.min() + diff1 + diff2);
    vertices1.push_back(init.min()         + diff2);
    vertices2.push_back(     base2                );
    vertices2.push_back(     base2 + diff1        );
    vertices2.push_back(     base2 + diff1 + diff2);
    vertices2.push_back(     base2         + diff2);
    
    polygons.push_back(convex_polygon(vertices1));
    polygons.push_back(convex_polygon(vertices2));
  }
}
*/
  
void convex_polygon::setup_cache_if_needed()const {
  if (cache_.is_valid) return;
  
  cache_.adjusted_vertices = vertices_;
  
  // Translate everything to a more convenient location. Translations are linear and hence preserve everything we need.
  cache_.translation_amount = -vertices_[0];
  for (vector3<polygon_int_type>& v : cache_.adjusted_vertices) v += cache_.translation_amount;
  
  cache_.amount_twisted = 0;
  while (true) {
    cache_.denom = (cache_.adjusted_vertices[1].x*cache_.adjusted_vertices[2].y - cache_.adjusted_vertices[1].y*cache_.adjusted_vertices[2].x);
    if (cache_.denom != 0) break;
    
    // One of the three orientations must work.
    // These are rotations, which are linear and hence preserve everything we need.
    
    ++cache_.amount_twisted;
    for (vector3<polygon_int_type>& v : cache_.adjusted_vertices) v = vector3<polygon_int_type>(v.y, v.z, v.x);
    
    assert_if_ASSERT_EVERYTHING(cache_.amount_twisted <= 2);
  }
  
  // In the formula Skew(T) = I + (z unit vector)*(a(t.x) + b(t.y)) ...
  cache_.a_times_denom =  ((cache_.adjusted_vertices[2].z*cache_.adjusted_vertices[1].y)
                         - (cache_.adjusted_vertices[1].z*cache_.adjusted_vertices[2].y));
  cache_.b_times_denom = -((cache_.adjusted_vertices[2].z*cache_.adjusted_vertices[1].x)
                         - (cache_.adjusted_vertices[1].z*cache_.adjusted_vertices[2].x));
  
  // We don't actually need to skew the polygon, since (by definition) it just skews the z coordinate to zero,
  // and because of that, hereafter we just don't need to refer to the z coordinates at all.
}

void bounding_box::translate(vector3<polygon_int_type> t) {
  min_ += t; max_ += t;
}

void line_segment::translate(vector3<polygon_int_type> t) {
  for (vector3<polygon_int_type>& v : ends) v += t;
}

void convex_polygon::translate(vector3<polygon_int_type> t) {
  for (vector3<polygon_int_type>& v : vertices_) v += t;
  cache_.translation_amount -= t;
}

void convex_polyhedron::translate(vector3<polygon_int_type> t) {
  for (vector3<polygon_int_type>& v : vertices_) v += t;
}

void shape::translate(vector3<polygon_int_type> t) {
  for (     line_segment& l : segments_ ) l.translate(t);
  for (   convex_polygon& p : polygons_ ) p.translate(t);
  for (convex_polyhedron& p : polyhedra_) p.translate(t);
  for (     bounding_box& b : boxes_    ) b.translate(t);
  if (bounds_cache_is_valid_ && bounds_cache_.is_anywhere()) { bounds_cache_.translate(t); }
}

bounding_box line_segment::bounds()const {
  bounding_box result;
  for (vector3<polygon_int_type> const& v : ends) result.combine_with(bounding_box(v));
  return result;
}

bounding_box convex_polygon::bounds()const {
  bounding_box result;
  for (vector3<polygon_int_type> const& v : vertices_) result.combine_with(bounding_box(v));
  return result;
}

bounding_box convex_polyhedron::bounds()const {
  bounding_box result;
  for (vector3<polygon_int_type> const& v : vertices_) result.combine_with(bounding_box(v));
  return result;
}

bounding_box shape::bounds()const {
  if (bounds_cache_is_valid_) return bounds_cache_;
  bounds_cache_ = bounding_box();
  for (     line_segment const& l : segments_ ) bounds_cache_.combine_with(l.bounds());
  for (   convex_polygon const& p : polygons_ ) bounds_cache_.combine_with(p.bounds());
  for (convex_polyhedron const& p : polyhedra_) bounds_cache_.combine_with(p.bounds());
  for (     bounding_box const& b : boxes_    ) bounds_cache_.combine_with(b         );
  bounds_cache_is_valid_ = true;
  return bounds_cache_;
}

vector3<polygon_int_type> shape::arbitrary_interior_point()const {
  if (!segments_.empty()) return segments_[0].ends[0];
  if (!polygons_.empty()) return polygons_[0].get_vertices()[0];
  if (!polyhedra_.empty()) return polyhedra_[0].vertices()[0];
  for (bounding_box const& bb : boxes_) { if (bb.is_anywhere()) { return bb.min(); }}
  caller_error("Trying to get an arbitrary interior point of a shape that contains no points");
}

namespace /*anonymous*/ {
namespace get_intersection_line_segment_bounding_box_helper {
enum should_keep_going { RETURN_NONE_IMMEDIATELY, KEEP_GOING };
template<bool less>
should_keep_going check(which_dimension_type dim, rational& intersecting_min, rational& intersecting_max, line_segment const& l, bounding_box const& bb) {
  typename boost::conditional<less, std::less<rational>, std::greater<rational> >::type compare;
  vector3<polygon_int_type> const& bb_min_or_max = less ? bb.min() : bb.max();
  vector3<polygon_int_type> const& bb_max_or_min = less ? bb.max() : bb.min();
  rational& intersecting_min_or_max = less ? intersecting_min : intersecting_max;
  if (l.ends[0][dim] == l.ends[1][dim]) {
    if (compare(l.ends[0][dim], bb_min_or_max[dim])) return RETURN_NONE_IMMEDIATELY;
  }
  else {
    const rational checkval(
      (l.ends[1][dim] > l.ends[0][dim] ? bb_min_or_max[dim] : bb_max_or_min[dim]) - l.ends[0][dim],
      l.ends[1][dim] - l.ends[0][dim]
    );
    if (compare(intersecting_min_or_max, checkval)) {
      intersecting_min_or_max = checkval;
      if (intersecting_min > intersecting_max) return RETURN_NONE_IMMEDIATELY;
    }
  }
  return KEEP_GOING;
}
}
} /* end anonymous namespace */

optional_rational get_first_intersection(line_segment const& l, bounding_box const& bb) {
  if (!bb.is_anywhere()) return none;
  
  // Check for common, simple cases to save time.
  if (l.ends[0](X) < bb.min(X) && l.ends[1](X) < bb.min(X)) return none;
  if (l.ends[0](Y) < bb.min(Y) && l.ends[1](Y) < bb.min(Y)) return none;
  if (l.ends[0](Z) < bb.min(Z) && l.ends[1](Z) < bb.min(Z)) return none;
  if (l.ends[0](X) > bb.max(X) && l.ends[1](X) > bb.max(X)) return none;
  if (l.ends[0](Y) > bb.max(Y) && l.ends[1](Y) > bb.max(Y)) return none;
  if (l.ends[0](Z) > bb.max(Z) && l.ends[1](Z) > bb.max(Z)) return none;
  if (bb.contains(l.ends[0])) return rational(0);
  
  rational intersecting_min(0);
  rational intersecting_max(1);

  using namespace get_intersection_line_segment_bounding_box_helper;

  if(check<true>(X, intersecting_min, intersecting_max, l, bb) == RETURN_NONE_IMMEDIATELY) return none;
  if(check<true>(Y, intersecting_min, intersecting_max, l, bb) == RETURN_NONE_IMMEDIATELY) return none;
  if(check<true>(Z, intersecting_min, intersecting_max, l, bb) == RETURN_NONE_IMMEDIATELY) return none;
  if(check<false>(X, intersecting_min, intersecting_max, l, bb) == RETURN_NONE_IMMEDIATELY) return none;
  if(check<false>(Y, intersecting_min, intersecting_max, l, bb) == RETURN_NONE_IMMEDIATELY) return none;
  if(check<false>(Z, intersecting_min, intersecting_max, l, bb) == RETURN_NONE_IMMEDIATELY) return none;
  
  return intersecting_min;
}

namespace /*anonymous*/ {
namespace get_intersection_line_segment_convex_polygon_helper {
optional_rational planar_get_first_intersection(
              polygon_int_type sl1x, polygon_int_type sl1y,
              polygon_int_type sl2x, polygon_int_type sl2y,
              polygon_int_type ol1x, polygon_int_type ol1y,
              polygon_int_type ol2x, polygon_int_type ol2y) {
  // assume ol1 is (0, 0)
  ol2x -= ol1x;
  sl1x -= ol1x;
  sl2x -= ol1x;
  ol2y -= ol1y;
  sl1y -= ol1y;
  sl2y -= ol1y;
  if (ol2x == 0) {
    return planar_get_first_intersection(sl1y, sl1x, sl2y, sl2x, 0, 0, ol2y, ol2x);
  }
  const polygon_int_type D = ol2x;
  const polygon_int_type A = -ol2y;
  const polygon_int_type Dy1 = D*sl1y + A*sl1x;
  const polygon_int_type Dy2 = D*sl2y + A*sl2x;
  const polygon_int_type Dy2mDy1 = Dy2 - Dy1;
  const polygon_int_type ltx_Dy2mDy1 = sl1x * Dy2 - sl2x * Dy1;
  if (ltx_Dy2mDy1 < 0 || ltx_Dy2mDy1 > ol2x*Dy2mDy1) return none;
  else                                               return rational(Dy1, Dy1 - Dy2);
}
}
} /* end anonymous namespace */

optional_rational get_first_intersection(line_segment l, convex_polygon const& p) {
  using namespace get_intersection_line_segment_convex_polygon_helper;
  
  p.setup_cache_if_needed();
  polygon_collision_info_cache const& c = p.get_cache();
  
  // Translate and twist, as we did with the polygon.
  for (vector3<polygon_int_type>& v : l.ends) v += c.translation_amount;
  for (vector3<polygon_int_type>& v : l.ends) v = vector3<polygon_int_type>(v[(0 + c.amount_twisted) % 3], v[(1 + c.amount_twisted) % 3], v[(2 + c.amount_twisted) % 3]);
  // Now skew the z values. Skews are linear and hence preserve everything we need.
  // The line's z values are scaled up as well as skewed.
  for (vector3<polygon_int_type>& v : l.ends) { v.z = v.z * c.denom + (c.a_times_denom * v.x + c.b_times_denom * v.y); }
  
  if (sign(l.ends[0].z) == sign(l.ends[1].z)) {
    if (l.ends[0].z != 0) {
      // If the endpoints are on the same side, they're not colliding, obviously!
      return none;
    }
    else {
      // Now, we need to do 2D line vs. polygon collisions, which are just a bunch of 2D line vs. line collisions.
      // We just ignore the z values for these purposes.
      optional_rational result;
      for (size_t i = 0; i < c.adjusted_vertices.size(); ++i) {
        const int next_i = (i + 1) % c.adjusted_vertices.size();
        optional_rational here = planar_get_first_intersection(l.ends[0].x, l.ends[0].y, l.ends[1].x, l.ends[1].y, c.adjusted_vertices[i].x, c.adjusted_vertices[i].y, c.adjusted_vertices[next_i].x, c.adjusted_vertices[next_i].y);
        if (here && (!result || *here < *result)) {
          result = *here;
        }
      }
      return result;
    }
  }
  
  const polygon_int_type denom2 = l.ends[1].z - l.ends[0].z;
  // Find the point in the plane (scaled up by denom2, which was scaled up by denom...)
  
  const vector3<polygon_int_type> point_in_plane_times_denom2(
    l.ends[1].z*l.ends[0].x - l.ends[0].z*l.ends[1].x,
    l.ends[1].z*l.ends[0].y - l.ends[0].z*l.ends[1].y,
    0
  );
  
  // Don't assume which clockwiseness the polygon is - but the point can never be on the same side of all the lines if it's outside the polygon, and always will be if it's inside.
  polygon_int_type previous_clockwiseness = 0;
  for (size_t i = 0; i < c.adjusted_vertices.size(); ++i) {
    const size_t next_i = (i + 1) % c.adjusted_vertices.size();
    const polygon_int_type clockwiseness = sign(
        ((point_in_plane_times_denom2.y - c.adjusted_vertices[i].y*denom2) * (c.adjusted_vertices[next_i].x - c.adjusted_vertices[i].x))
      - ((point_in_plane_times_denom2.x - c.adjusted_vertices[i].x*denom2) * (c.adjusted_vertices[next_i].y - c.adjusted_vertices[i].y))
    );
    if (clockwiseness != 0) {
      if (previous_clockwiseness == 0) {
        previous_clockwiseness = clockwiseness;
      }
      else {
        if (clockwiseness != previous_clockwiseness) return none;
      }
    }
  }
  
  return rational(l.ends[0].z, l.ends[0].z - l.ends[1].z);
}

optional_rational get_first_intersection(line_segment l, convex_polyhedron const& p) {
  rational min_intersecting(0);
  rational max_intersecting(1);
  
  polyhedron_planes_info_for_intersection planes_info;
  compute_planes_info_for_intersection(p, planes_info);
  
  for (auto const& base_point_and_normal : planes_info.base_points_and_outward_facing_normals) {
    /*
      Relative to the base point:
        the plane is defined by the normal vector
        (x, y, z) dot normal = 0
      We want the r such that
        (l0 + r(lv)) dot normal = 0
        r (lv dot normal) + (l0 dot normal) = 0
        r = -(l0 dot normal) / (lv dot normal)
    */
    
    // if lv dot normal is greater than zero, they're pointing in the same direction, so we're going OUT of this face.
    // if lv dot normal is less than zero, they're pointing in opposite directions, so we're going INTO this face.
    // if it's zero, then we're parallel to the surface, which means we're either totally cool or need to be eliminated completely.
    polygon_int_type l0_dot_normal = (l.ends[0] - base_point_and_normal.first).dot<polygon_int_type>(base_point_and_normal.second);
    polygon_int_type lv_dot_normal = (l.ends[1] - l.ends[0]).dot<polygon_int_type>(base_point_and_normal.second);
    if (lv_dot_normal == 0) {
      // This determines whether we started inside the plane or outside of it.
      if (l0_dot_normal > 0) {
        return none;
      }
    }
    else {
      rational intersection_point(-l0_dot_normal, lv_dot_normal);
      if (lv_dot_normal > 0) {
        if (intersection_point < max_intersecting) {
          max_intersecting = intersection_point;
          if (min_intersecting > max_intersecting) return none;
        }
      }
      else {
        if (intersection_point > min_intersecting) {
          min_intersecting = intersection_point;
          if (min_intersecting > max_intersecting) return none;
        }
      }
    }
  }
  return min_intersecting;
}

namespace /*anonymous*/ {
std::array<line_segment, 12> edges_of_bounding_box_as_line_segments(bounding_box b) {
  typedef vector3<polygon_int_type> vect;
  std::array<line_segment, 12> result = {{
    line_segment(vect(b.min(X), b.min(Y), b.min(Z)),
                 vect(b.max(X), b.min(Y), b.min(Z))),
    line_segment(vect(b.min(X), b.max(Y), b.min(Z)),
                 vect(b.max(X), b.max(Y), b.min(Z))),
    line_segment(vect(b.min(X), b.max(Y), b.max(Z)),
                 vect(b.max(X), b.max(Y), b.max(Z))),
    line_segment(vect(b.min(X), b.min(Y), b.max(Z)),
                 vect(b.max(X), b.min(Y), b.max(Z))),

    line_segment(vect(b.min(X), b.min(Y), b.min(Z)),
                 vect(b.min(X), b.max(Y), b.min(Z))),
    line_segment(vect(b.max(X), b.min(Y), b.min(Z)),
                 vect(b.max(X), b.max(Y), b.min(Z))),
    line_segment(vect(b.max(X), b.min(Y), b.max(Z)),
                 vect(b.max(X), b.max(Y), b.max(Z))),
    line_segment(vect(b.min(X), b.min(Y), b.max(Z)),
                 vect(b.min(X), b.max(Y), b.max(Z))),

    line_segment(vect(b.min(X), b.min(Y), b.min(Z)),
                 vect(b.min(X), b.min(Y), b.max(Z))),
    line_segment(vect(b.max(X), b.min(Y), b.min(Z)),
                 vect(b.max(X), b.min(Y), b.max(Z))),
    line_segment(vect(b.max(X), b.max(Y), b.min(Z)),
                 vect(b.max(X), b.max(Y), b.max(Z))),
    line_segment(vect(b.min(X), b.max(Y), b.min(Z)),
                 vect(b.min(X), b.max(Y), b.max(Z))),
  }};
  return result;
}
  
bool nonshape_intersects_onesided(convex_polygon const& p1, convex_polygon const& p2) {
  std::vector<vector3<polygon_int_type>> const& vs = p1.get_vertices();
  for (size_t i = 0; i < vs.size(); ++i) {
    const int next_i = (i + 1) % vs.size();
    if (get_first_intersection(line_segment(vs[i], vs[next_i]), p2)) return true;
  }
  return false;
}

bool nonshape_intersects(convex_polygon const& p1, convex_polygon const& p2) {
  return (nonshape_intersects_onesided(p1,p2) || nonshape_intersects_onesided(p2,p1));
}

bool nonshape_intersects(convex_polygon const& p1, convex_polyhedron const& p2) {
  // Wait. What if the polygon is a giant slice through the polyhedron?
  std::cerr << "Warning: Polygon-polyhedron collisions not fully implemented yet!";
  std::vector<vector3<polygon_int_type>> const& vs = p1.get_vertices();
  for (size_t i = 0; i < vs.size(); ++i) {
    const int next_i = (i + 1) % vs.size();
    if (get_first_intersection(line_segment(vs[i], vs[next_i]), p2)) return true;
  }
  return false;
}

bool nonshape_intersects(convex_polygon const& p, bounding_box const& bb) {
  if (!bb.is_anywhere()) return false;
  
  std::vector<vector3<polygon_int_type>> const& vs = p.get_vertices();
  if (bb.contains(vs[0])) return true;
  for (size_t i = 0; i < vs.size(); ++i) {
    const int next_i = (i + 1) % vs.size();
    if (get_first_intersection(line_segment(vs[i], vs[next_i]), bb)) return true;
  }
  for(line_segment edge : edges_of_bounding_box_as_line_segments(bb)) {
    if (get_first_intersection(edge, p)) return true;
  }
  return false;
}

} /* end anonymous namespace */

bool shape::intersects(shape const& other)const {
  if (!bounds().overlaps(other.bounds())) return false;
  
  caller_error_if((!segments_.empty()) && (!other.segments_.empty()), "You tried to check whether two shapes that both contain 'segment' parts were intersecting. That shouldn't be allowed to happen, because of the somewhat arbitrary nature of whether two 1D objects intersect in 3D space.");
  
  for (line_segment const& l : segments_) {
    for (convex_polygon const& p2 : other.polygons_) {
      if (get_first_intersection(l, p2)) return true;
    }
    for (convex_polyhedron const& p2 : other.polyhedra_) {
      if (get_first_intersection(l, p2)) return true;
    }
    for (bounding_box const& b2 : other.boxes_) {
      if (get_first_intersection(l, b2)) return true;
    }
  }

  for (convex_polygon const& p1 : polygons_) {
    for (line_segment const& l : other.segments_) {
      if (get_first_intersection(l, p1)) return true;
    }
    for (convex_polygon const& p2 : other.polygons_) {
      if (nonshape_intersects(p1, p2)) return true;
    }
    for (convex_polyhedron const& p2 : other.polyhedra_) {
      if (nonshape_intersects(p1, p2)) return true;
    }
    for (bounding_box const& b2 : other.boxes_) {
      if (nonshape_intersects(p1, b2)) return true;
    }
  }

  for (convex_polyhedron const& p1 : polyhedra_) {
    for (line_segment const& l : other.segments_) {
      if (get_first_intersection(l, p1)) return true;
    }
    for (convex_polygon const& p2 : other.polygons_) {
      if (nonshape_intersects(p2, p1)) return true;
    }
    for (convex_polyhedron const& p2 : other.polyhedra_) {
      if (!get_excluding_face(p1, p2)) return true;
    }
    for (bounding_box const& b2 : other.boxes_) {
      if (!get_excluding_face(p1, b2)) return true;
    }
  }

  for (bounding_box const& b1 : boxes_) {
    for (line_segment const& l : other.segments_) {
      if (get_first_intersection(l, b1)) return true;
    }
    for (convex_polygon const& p2 : other.polygons_) {
      if (nonshape_intersects(p2, b1)) return true;
    }
    for (convex_polyhedron const& p2 : other.polyhedra_) {
      if (!get_excluding_face(p2, b1)) return true;
    }
    for (bounding_box const& b2 : other.boxes_) {
      if (b1.overlaps(b2)) return true;
    }
  }
  return false;
}

bool shape::intersects(bounding_box const& other)const {
  if (!bounds().overlaps(other)) return false;

  for (line_segment const& l : segments_) {
    if (get_first_intersection(l, other)) return true;
  }

  for (convex_polygon const& p1 : polygons_) {
    if (nonshape_intersects(p1, other)) return true;
  }

  for (bounding_box const& b1 : boxes_) {
    if (b1.overlaps(other)) return true;
  }
  return false;
}

optional_rational get_first_intersection(line_segment const& l, shape const& s) {
  optional_rational result;
  for (convex_polygon const& p : s.get_polygons()) {
    const optional_rational here = get_first_intersection(l, p);
    if (here && (!result || *here < *result)) {
      result = *here;
    }
  }
  for (convex_polyhedron const& p : s.get_polyhedra()) {
    const optional_rational here = get_first_intersection(l, p);
    if (here && (!result || *here < *result)) {
      result = *here;
    }
  }
  for (bounding_box const& bb : s.get_boxes()) {
    const optional_rational here = get_first_intersection(l, bb);
    if (here && (!result || *here < *result)) {
      result = *here;
    }
  }
  return result;
}

