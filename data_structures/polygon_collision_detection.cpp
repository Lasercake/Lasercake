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

#include "polygon_collision_detection.hpp"

using boost::none;

typedef polygon_rational_type rational;

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

void shape::translate(vector3<polygon_int_type> t) {
  for (  line_segment& l : segments_) l.translate(t);
  for (convex_polygon& p : polygons_) p.translate(t);
  for (  bounding_box& b : boxes_   ) b.translate(t);
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

bounding_box shape::bounds()const {
  if (bounds_cache_is_valid_) return bounds_cache_;
  bounds_cache_ = bounding_box();
  for (  line_segment const& l : segments_) bounds_cache_.combine_with(l.bounds());
  for (convex_polygon const& p : polygons_) bounds_cache_.combine_with(p.bounds());
  for (  bounding_box const& b : boxes_   ) bounds_cache_.combine_with(b         );
  bounds_cache_is_valid_ = true;
  return bounds_cache_;
}

vector3<polygon_int_type> shape::arbitrary_interior_point()const {
  if (!segments_.empty()) return segments_[0].ends[0];
  if (!polygons_.empty()) return polygons_[0].get_vertices()[0];
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
    for (bounding_box const& b2 : other.boxes_) {
      if (nonshape_intersects(p1, b2)) return true;
    }
  }

  for (bounding_box const& b1 : boxes_) {
    for (line_segment const& l : other.segments_) {
      if (get_first_intersection(l, b1)) return true;
    }
    for (convex_polygon const& p2 : other.polygons_) {
      if (nonshape_intersects(p2, b1)) return true;
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

