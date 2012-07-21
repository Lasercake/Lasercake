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




#if 0

// Utility functions for wtfwtf
inline polygon_int_type plane_D_value(std::array<vector3<polygon_int_type>, 3> const& vs) {
  // Outputs have max height 3n + 1 + log2(3).
  return
    vs[1](X) * ((vs[3](Y) * vs[2](Z)) - (vs[2](Y) * vs[3](Z))) +
    vs[2](X) * ((vs[1](Y) * vs[3](Z)) - (vs[3](Y) * vs[1](Z))) +
    vs[3](X) * ((vs[2](Y) * vs[1](Z)) - (vs[1](Y) * vs[2](Z)));
}

// "true" = return 'none' immediately
// "false" = keep going
bool restrict_answer_to_planes(rational& min_intersecting, rational& max_intersecting, vector3<polygon_int_type> const& line_start, vector3<polygon_int_type> const& line_vector, vector3<polygon_int_type> const& normal, polygon_int_type D1, polygon_int_type D2, std::array<vector3<polygon_int_type>, 3> const& D1_vs) {
  // Plugging the line equation
  //     (x, y, z) = r(dx, dy, dz) + (x1, x2, x3)
  // into the plane equation
  //     Ax + By + Cz + D = 0
  // yields
  //     r(Adx + Bdy + Cdz) + Ax1 + Bx2 + Cx3 + D = 0
  //     r = -(Ax1 + Bx2 + Cx3 + D) / (Adx + Bdy + Cdz)
  
  // denominator has max height 3n + 1 + 2*log2(3).
  const polygon_int_type rdenom = line_vector.dot<polygon_int_type>(normal);
  if (rdenom == 0) {
    // we're perpendicular to the normal, i.e. parallel to the planes.
    // either we're between them (totally fine) or outside (fail instantly).
    std::array<vector3<polygon_int_type>, 3> line_plane_hack = {
      line_start,
      line_start + D1_vs[1] - D1_vs[0],
      line_start + D1_vs[2] - D1_vs[0],
    };
    const polygon_int_type D3 = plane_D_value(line_plane_hack);
    if (((D3 < D1) != (D3 < D2)) || (D3 == D1) || (D3 == D2)) return false;
    else return true;
  }
  const polygon_int_type start_dot_normal = line_start.dot<polygon_int_type>(normal);
  // numerators have max height 3n + 3 + log2(3).
  const polygon_int_type rnum_D1 = -(start_dot_normal + D1);
  const polygon_int_type rnum_D2 = -(start_dot_normal + D2);
  const rational r1(rnum_D1, rdenom);
  const rational r2(rnum_D2, rdenom);
  
  /*
  if ((rnum_minD > rdenom) && (rnum_maxD > rdenom)) {
    // The line is entirely before the planes.
    return false;
  }
  if ((rnum_minD < 0) && (rnum_maxD < 0)) {
    // The line is entirely beyond the planes.
    return false;
  }*/
  
  if (r1 > r2) {
    if (r1 < max_intersecting) { max_intersecting = r1; }
    if (r2 > min_intersecting) { min_intersecting = r2; }
    return (min_intersecting > max_intersecting);
  }
  else {
    if (r2 < max_intersecting) { max_intersecting = r2; }
    if (r1 > min_intersecting) { min_intersecting = r1; }
    return (min_intersecting > max_intersecting);
  }
}


/*
bool wtfwtf (convex_polygon const& p, line_segment const& l, vector3<polygon_int_type> possible_independent_offsets, vector3<polygon_int_type> possible_vector_offset) {
  rational min_intersecting(0);
  rational max_intersecting(1);
 
  // Translate everything to near the origin in order to minimize the numbers' heights.
  // The first point of the polygon is fine.
  // I believe this function cannot overflow if all the inputs are
  // within a 961005 x 961005 x 961005 box.
  // (That's floor(2^(128 - 4 - 3*log2(3))).)
  std::vector<vector3<polygon_int_type>> vs(p.get_vertices());
  const vector3<polygon_int_type> line_vector = l.ends[1] - l.ends[0];
  const vector3<polygon_int_type> line_start = l.ends[0] - vs[0];
  for (int i = vs.size() - 1; i > 0; --i) {
    vs[i] -= vs[0];
  }
  
  // First - figure out where/if the line segment passes through the plane of the polygon.
  // Or rather, find the min and max distance along the line you'd have to reach to pass through the plane,
  // by plugging the line vector into the plane equation
  //     Ax + By + Cz + D = 0
  // (A, B, C) is the plane's normal vector, and doesn't change when we translate the whole polygon
  //     by up to 1 (or really any amount) in each of the three dimensions.
  
  // Each coordinate of the normal vector has max height 2n + 1 + log2(3).
  const vector3<polygon_int_type> normal(
    (vs[1](X) * (vs[2](Z) - vs[3](Z))) + (vs[2](Y) * (vs[3](Z) - vs[1](Z))) + (vs[3](Y) * (vs[1](Z) - vs[2](Z))),
    (vs[1](Z) * (vs[2](X) - vs[3](X))) + (vs[2](Z) * (vs[3](X) - vs[1](X))) + (vs[3](Z) * (vs[1](X) - vs[2](X))),
    (vs[1](X) * (vs[2](Y) - vs[3](Y))) + (vs[2](X) * (vs[3](Y) - vs[1](Y))) + (vs[3](X) * (vs[1](Y) - vs[2](Y)))
  );
  
  // But D varies. We want the min and max.
  vector3<polygon_int_type> minD_offsets(0,0,0);
  vector3<polygon_int_type> maxD_offsets(0,0,0);
  for (which_dimension_type d = 0; d < num_dimensions; ++d) {
    if ((normal(d) != 0) && (possible_independent_offsets(d) != 0)) {
      if ((normal(d) > 0) == (possible_independent_offsets(d) > 0)) {
        minD_offsets(d) = possible_independent_offsets(d);
      }
      else {
        maxD_offsets(d) = possible_independent_offsets(d);
      }
    }
  }
  const polygon_int_type offs_dot_normal = possible_vector_offset.dot<polygon_int_type>(normal);
  if (offs_dot_normal > 0) {
    minD_offsets += possible_vector_offset;
  }
  else if (offs_dot_normal < 0) {
    maxD_offsets += possible_vector_offset;
  }
  
  const std::array<vector3<polygon_int_type>, 3> minD_vs = { vs[1] + minD_offsets, vs[2] + minD_offsets, vs[3] + minD_offsets };
  const std::array<vector3<polygon_int_type>, 3> maxD_vs = { vs[1] + maxD_offsets, vs[2] + maxD_offsets, vs[3] + maxD_offsets };
  
  const polygon_int_type minD = plane_D_value(minD_vs);
  const polygon_int_type maxD = plane_D_value(maxD_vs);
  
  
  if (restrict_answer_to_planes(min_intersecting, max_intersecting,
       line_start, line_vector,
       normal, minD, maxD, minD_vs)) return false;
}*/

#endif

namespace {

vector3<polygon_int_type> plane_normal(vector3<polygon_int_type> p1, vector3<polygon_int_type> p2, vector3<polygon_int_type> p3) {
  return vector3<polygon_int_type>(
    (p1(Y) * (p2(Z) - p3(Z))) + (p2(Y) * (p3(Z) - p1(Z))) + (p3(Y) * (p1(Z) - p2(Z))),
    (p1(Z) * (p2(X) - p3(X))) + (p2(Z) * (p3(X) - p1(X))) + (p3(Z) * (p1(X) - p2(X))),
    (p1(X) * (p2(Y) - p3(Y))) + (p2(X) * (p3(Y) - p1(Y))) + (p3(X) * (p1(Y) - p2(Y)))
  );
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

} /* end anonymous namespace */

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

bool vectors_are_parallel(vector3<polygon_int_type> const& v1, vector3<polygon_int_type> const& v2) {
  return (v1(X) * v2(Y) == v2(X) * v1(Y)) && (v1(X) * v2(Z) == v2(X) * v1(Z)) && (v1(Y) * v2(Z) == v2(Y) * v1(Z));
}

void convex_polyhedron::init_other_info_from_vertices() {
  caller_correct_if(vertices_.size() <= 255, "You can't make a polyhedron with more than 255 points.");
  caller_correct_if(vertices_.size() >= 4, "You can't make a polyhedron with fewer than 4 points.");
  std::vector<vector3<polygon_int_type>> const& vs = vertices_;
  
  // Algorithm: For each point, expand the polyhedron to include that point.
  // When you're done, you'll have the correct answer.
  // Complexity: Quadratic in the number of points.
  struct face_building_info {
    face_building_info(){}
    std::unordered_set<int> verts;
    vector3<polygon_int_type> normal;
  };
  struct line_building_info {
    int face_1;
    int face_2;
    int vert_1;
    int vert_2;
  };
  std::unordered_map<int, face_building_info> faces;
  std::list<line_building_info> lines;
  
  // Hack: Start by making a tetrahedron with the first four non-coplanar verts.
  std::array<int, 4> first_four_noncoplanar_verts{{0,1,-1,-1}};
  for (int i = 2; i < (int)vs.size(); ++i) {
    if (!vectors_are_parallel(vs[i] - vs[0], vs[1] - vs[0])) {
      first_four_noncoplanar_verts[2] = i;
      break;
    }
  }
  assert(first_four_noncoplanar_verts[2] != -1);
  auto f3norm = plane_normal(vs[0], vs[1], vs[first_four_noncoplanar_verts[2]]);
  for (int i = first_four_noncoplanar_verts[2]+1; i < (int)vs.size(); ++i) {
    if ((vs[i] - vs[0]).dot<polygon_int_type>(f3norm) != 0) {
      first_four_noncoplanar_verts[3] = i;
      break;
    }
  }
  assert(first_four_noncoplanar_verts[3] != -1);
  
  for(int q = 0; q < 4; ++q) {
    face_building_info& f = faces[q];
    const int i1 = first_four_noncoplanar_verts[q];
    const int i2 = first_four_noncoplanar_verts[(q+1) % 4];
    const int i3 = first_four_noncoplanar_verts[(q+2) % 4];
    const int i4 = first_four_noncoplanar_verts[(q+3) % 4];
    f.verts.insert(i2); f.verts.insert(i3); f.verts.insert(i4);
    f.normal = plane_normal(vs[i2],vs[i3],vs[i4]);
    if ((vs[i2] - vs[i1]).dot<polygon_int_type>(f.normal) < 0) f.normal = -f.normal;
    for(int j = q + 1; j < 4; ++j) {
      line_building_info l;
      l.vert_1 = first_four_noncoplanar_verts[q];
      l.vert_2 = first_four_noncoplanar_verts[j];
      l.face_1 = ((q != 0) && (j != 0)) ? 0 : ((q != 1) && (j != 1)) ? 1 : 2;
      l.face_2 = ((q != 3) && (j != 3)) ? 3 : ((q != 2) && (j != 2)) ? 2 : 1;
      lines.push_back(l);
      assert(vs[l.vert_1] != vs[l.vert_2]);
    }
  }
  int next_face_id = 4;
  for (int i = 0; i < (int)vs.size(); ++i) {
    bool exposed = false;
    for (auto const& f : faces) {
      assert(!f.second.verts.empty());
      polygon_int_type dotprod = (vs[i] - vs[*f.second.verts.begin()]).dot<polygon_int_type>(f.second.normal);
      std::cerr << i << "!" << dotprod << "!\n";
      if (dotprod > 0) {
        exposed = true;
        break;
      }
    }
    if (exposed) {
      std::unordered_set<int> new_and_old_faces;
      std::unordered_set<int> old_faces;
      std::unordered_set<int> verts_of_potential_new_lines;
      for (auto l = lines.begin(); l != lines.end(); ) {
        auto fit1 = faces.find(l->face_1);
        auto fit2 = faces.find(l->face_2);
        assert(fit1 != faces.end());
        assert(fit2 != faces.end());
        auto& f1 = fit1->second;
        auto& f2 = fit2->second;
        assert(!f1.verts.empty());
        assert(!f2.verts.empty());
        const polygon_int_type dotprod1 = (vs[i] - vs[*f1.verts.begin()]).dot<polygon_int_type>(f1.normal);
        const polygon_int_type dotprod2 = (vs[i] - vs[*f2.verts.begin()]).dot<polygon_int_type>(f2.normal);
        std::cerr << dotprod1 << ", " << dotprod2 << "\n";
        // Both <=0: we're inside both, the line is unaffected
        // Both >0: we're outside both, the line will just be purged
        if ((dotprod1 == 0) && (dotprod2 == 0)) {
          f1.verts.insert(i);
          f2.verts.insert(i);
          for (int dim = 0; dim < num_dimensions; ++dim) {
            if (vs[l->vert_1](dim) != vs[l->vert_2](dim)) {
              if ((vs[l->vert_1](dim) < vs[l->vert_2](dim)) == (vs[l->vert_1](dim) < vs[i](dim))) {
                f1.verts.erase(l->vert_2);
                f2.verts.erase(l->vert_2);
                l->vert_2 = i;
                assert(vs[l->vert_1] != vs[l->vert_2]);
              }
              else {
                f1.verts.erase(l->vert_1);
                f2.verts.erase(l->vert_1);
                l->vert_1 = i;
                assert(vs[l->vert_1] != vs[l->vert_2]);
              }
              break;
            }
          }
          ++l;
        }
        else if ((dotprod1 > 0) != (dotprod2 > 0)) {
          verts_of_potential_new_lines.insert(l->vert_1);
          verts_of_potential_new_lines.insert(l->vert_2);
          if (dotprod1 == 0) {
            old_faces.insert(l->face_1);
            new_and_old_faces.insert(l->face_1);
            f1.verts.insert(i);
            lines.erase(l++);
          }
          else if (dotprod2 == 0) {
            old_faces.insert(l->face_2);
            new_and_old_faces.insert(l->face_2);
            f2.verts.insert(i);
            lines.erase(l++);
          }
          else {
            face_building_info new_face;
            const int new_face_id = next_face_id++;
            new_and_old_faces.insert(new_face_id);
            if (dotprod1 > 0) l->face_1 = new_face_id;
            else              l->face_2 = new_face_id;
            new_face.verts.insert(i);
            new_face.verts.insert(l->vert_1);
            new_face.verts.insert(l->vert_2);
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
            faces.insert(std::make_pair(new_face_id, new_face));
            ++l;
          }
        }
        else if ((dotprod1 > 0) && (dotprod2 > 0)) lines.erase(l++);
        else ++l;
      }
      for (auto f = faces.begin(); f != faces.end(); ) {
        polygon_int_type dotprod = (vs[i] - vs[*f->second.verts.begin()]).dot<polygon_int_type>(f->second.normal);
        if (dotprod > 0) faces.erase(f++);
        else ++f;
      }
      std::cerr << "EEEEEEEEE" << verts_of_potential_new_lines.size() << ", " << new_and_old_faces.size() << ", " << old_faces.size() << "\n";
      for (int lv : verts_of_potential_new_lines) {
        line_building_info l;
        int num_faces_including = 0;
        for (int fid : new_and_old_faces) {
          polygon_int_type dotprod = (vs[lv] - vs[*faces[fid].verts.begin()]).dot<polygon_int_type>(faces[fid].normal);
          std::cerr<<dotprod<<"...\n";
          assert(dotprod <= 0);
          if (dotprod == 0) {
            if (num_faces_including == 0) l.face_1 = fid;
            else                          l.face_2 = fid;
            ++num_faces_including;
            if (num_faces_including == 2) break;
          }
        }
        if (num_faces_including == 2) {
          l.vert_1 = i;
          l.vert_2 = lv;
          assert(vs[i] != vs[lv]);
          lines.push_back(l);
        }
        else {
          for (int fid : old_faces) {
            faces[fid].verts.erase(lv);
          }
        }
      }
    }
  }
  
  std::vector<bool> existences_of_points(vertices_.size(), false);
  for (auto const& f : faces) {
    for (int v : f.second.verts) {
      existences_of_points[v] = true;
    }
  }
  std::unordered_map<int,int> vertex_id_map;
  int next_vert_id = 0;
  for (int i = 0; i < (int)vertices_.size(); ++i) {
    if (existences_of_points[i]) {
      vertices_[next_vert_id] = vertices_[i];
      vertex_id_map[i] = next_vert_id;
      ++next_vert_id;
    }
  }
  vertices_.erase(vertices_.begin() + next_vert_id, vertices_.end());
  for (auto const& l : lines) {
    std::cerr << l.vert_1 << l.vert_2 << vertices_.size();
    edges_.push_back(std::make_pair(
       vertex_id_map.find(l.vert_1)->second,
       vertex_id_map.find(l.vert_2)->second));
  }
  for (auto const& f : faces) {
    const int face_idx = face_info_.size();
    face_info_.push_back(f.second.verts.size());
    for (int vid : f.second.verts) {
      face_info_.push_back(vertex_id_map.find(vid)->second);
    }
    // Hack: we rely on the fact that this makes the *first three* points have a reliable outwards
    // normal vector. So sort them that way.
    // Really we should sort ALL OF THEM that way
    if (plane_normal(vertices_[face_info_[face_idx + 1]], vertices_[face_info_[face_idx + 2]], vertices_[face_info_[face_idx + 3]]).dot<polygon_int_type>(f.second.normal) < 0) {
      const int temp = face_info_[face_idx + 2];
      face_info_[face_idx + 2] = face_info_[face_idx + 3];
      face_info_[face_idx + 3] = temp;
    }
  }

#if 0  
  // lol quartic algorihtm TODO i can haz less stupid
  std::unordered_multimap<uint8_t, uint8_t> indexes_of_already_created_polygons_by_index_of_point;
  for (uint8_t i = 0; i < vs.size(); ++i) {
    for (uint8_t j = i + 1; j < vs.size(); ++j) {
      for (uint8_t k = j + 1; k < vs.size(); ++k) {
        // Check if we've already collected this plane
        bool already_collected = false;
        for (int l = i; l != -1; l = (l == i) ? j : (l == j) ? k : -1) {
          const auto range = indexes_of_already_created_polygons_by_index_of_point.equal_range(l);
          for (auto pair : boost::make_iterator_range(range.first, range.second)) {
            assert(pair.first == l);
            const uint8_t poly_start_idx = pair.second;
            int number_of_same_points = 0;
            assert(poly_start_idx < face_info_.size());
            for (uint8_t m = poly_start_idx + 1; m < poly_start_idx + 1 + face_info_[poly_start_idx]; ++m) {
              assert(m < face_info_.size());
              if ((face_info_[m] == i) || (face_info_[m] == j) || (face_info_[m] == k)) {
                ++number_of_same_points;
                if (number_of_same_points == 3) {
                  already_collected = true;
                  goto triplebreak;
                }
              }
            }
          }
        }
        triplebreak:
        
        if (!already_collected) {
          vector3<polygon_int_type> normal = plane_normal(vs[i],vs[j],vs[k]);
          caller_error_if(normal == vector3<polygon_int_type>(0,0,0), "Trying to create a strictly convex polyhedron with three collinear points");
          bool positive_used = false;
          bool negative_used = false;
          std::vector<uint8_t> additional_points;
          for (uint8_t l = 0; l < vs.size(); ++l) {
            if ((l == i) || (l == j) || (l == k)) continue;
            
            polygon_int_type dotprod = (vs[l] - vs[i]).dot<polygon_int_type>(normal);
            if (dotprod == 0) {
              additional_points.push_back(l);
            }
            else if (dotprod > 0) {
              positive_used = true;
              if (negative_used) break;
            }
            else if (dotprod < 0) {
              negative_used = true;
              if (positive_used) break;
            }
            else assert(false);
          }
          caller_correct_if(positive_used || negative_used, "You can't create a polyhedron with all coplanar points!");
          if (positive_used != negative_used) {
            uint8_t poly_idx = face_info_.size();
            face_info_.push_back(3 + additional_points.size());
            indexes_of_already_created_polygons_by_index_of_point.insert(std::make_pair(i, poly_idx));
            indexes_of_already_created_polygons_by_index_of_point.insert(std::make_pair(j, poly_idx));
            indexes_of_already_created_polygons_by_index_of_point.insert(std::make_pair(k, poly_idx));
            
            // We rely on the fact that this makes the *first three* points have a reliable outwards
            // normal vector.
            if (negative_used) {
              face_info_.push_back(i);
              face_info_.push_back(j);
              face_info_.push_back(k);
            }
            else {
              face_info_.push_back(i);
              face_info_.push_back(k);
              face_info_.push_back(j);
            }
            for (uint8_t l : additional_points) {
              face_info_.push_back(l);
              indexes_of_already_created_polygons_by_index_of_point.insert(std::make_pair(l, poly_idx));
            }
          }
        }
      }
    }
  }
  
  for (uint8_t i = 0; i < vs.size(); ++i) {
    for (uint8_t j = i + 1; j < vs.size(); ++j) {
      int shared_polys_found = 0;
      const auto range = indexes_of_already_created_polygons_by_index_of_point.equal_range(i);
      for (auto pair : boost::make_iterator_range(range.first, range.second)) {
        assert(pair.first == i);
        uint8_t poly_start_idx = pair.second;
        assert(poly_start_idx < face_info_.size());
        for (uint8_t m = poly_start_idx + 1; m < poly_start_idx + 1 + face_info_[poly_start_idx]; ++m) {
          assert(m < face_info_.size());
          if (face_info_[m] == j) {
            ++shared_polys_found;
            break;
          }
        }
      }
      if (shared_polys_found > 1) {
        edges_.push_back(std::make_pair(i,j));
      }
    }
  }
#endif
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

void compute_sweep_allowing_rounding_error(convex_polyhedron const& ph, vector3<polygon_int_type> const& v, vector3<polygon_int_type> max_error, std::vector<vector3<polygon_int_type>>& vertex_collector, polyhedron_planes_info_for_intersection& plane_collector) {
  // Essentially, the polyhedron plus the entire space the polyhedron can reach by moving
  // in any combination of 0-to-1 of each of the following vectors:
  std::array<vector3<polygon_int_type>, 4> dirs = {{
    vector3<polygon_int_type>(max_error(X), 0, 0),
    vector3<polygon_int_type>(0, max_error(Y), 0),
    vector3<polygon_int_type>(0, 0, max_error(Z)),
    v
  }};
  // Collapse parallel dirs.
  // This is mostly unnecessary, but needed for the plane-generated-by-line code.
  for (int dim = 0; dim < num_dimensions; ++dim) {
    if ((v((dim+1) % num_dimensions) == 0) && (v((dim+2) % num_dimensions) == 0)) {
      dirs[3][dim] += dirs[dim](dim);
      dirs[dim][dim] = 0;
      //std::cerr << "eliminating parallel " << dir << "\n";
    }
  }
  
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
    vector3<polygon_int_type>(0,0,0),
    dirs[0],
    dirs[1],
    dirs[0] + dirs[1],
    dirs[2],
    dirs[0] + dirs[2],
    dirs[1] + dirs[2],
    dirs[0] + dirs[1] + dirs[2],
    dirs[3],
    dirs[0] + dirs[3],
    dirs[1] + dirs[3],
    dirs[0] + dirs[1] + dirs[3],
    dirs[2] + dirs[3],
    dirs[0] + dirs[2] + dirs[3],
    dirs[1] + dirs[2] + dirs[3],
    dirs[0] + dirs[1] + dirs[2] + dirs[3]
  }};
  static const std::array<uint16_t, 3> B_and_not_v_combos = {{
    (1<<0x1) | (1<<0x3) | (1<<0x5) | (1<<0x7),
    (1<<0x2) | (1<<0x3) | (1<<0x6) | (1<<0x7),
    (1<<0x4) | (1<<0x5) | (1<<0x6) | (1<<0x7),
  }};
  static const std::array<uint16_t, 3> v_and_not_B_combos = {{
    (1<<0x8) | (1<<0xA) | (1<<0xC) | (1<<0xE),
    (1<<0x8) | (1<<0x9) | (1<<0xC) | (1<<0xD),
    (1<<0x8) | (1<<0x9) | (1<<0xA) | (1<<0xB),
  }};
  static const std::array<uint16_t, 4> combos_including_dir = {{
    (1<<0x1) | (1<<0x3) | (1<<0x5) | (1<<0x7) | (1<<0x9) | (1<<0xB) | (1<<0xD) | (1<<0xF),
    (1<<0x2) | (1<<0x3) | (1<<0x6) | (1<<0x7) | (1<<0xA) | (1<<0xB) | (1<<0xE) | (1<<0xF),
    (1<<0x4) | (1<<0x5) | (1<<0x6) | (1<<0x7) | (1<<0xC) | (1<<0xD) | (1<<0xE) | (1<<0xF),
    (1<<0x8) | (1<<0x9) | (1<<0xA) | (1<<0xB) | (1<<0xC) | (1<<0xD) | (1<<0xE) | (1<<0xF),
  }};
  
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
  for (bounding_box const& bb : s.get_boxes()) {
    const optional_rational here = get_first_intersection(l, bb);
    if (here && (!result || *here < *result)) {
      result = *here;
    }
  }
  return result;
}

