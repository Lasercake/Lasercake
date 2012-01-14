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
  const int64_t temp = (((p.y - ray.e1.y) * (ray.e2.x - ray.e1.x)) - ((ray.e2.y - ray.e1.y) * (p.x - ray.e1.x)));
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

#include "polygon_collision_detection.hpp"

bool bounding_box::contains(vector3<int64_t> const& v)const {
  if (!is_anywhere) return false;
  return (v.x >= min.x && v.x <= max.x &&
          v.y >= min.y && v.y <= max.y &&
          v.z >= min.z && v.z <= max.z);
}
bool bounding_box::overlaps(bounding_box const& o)const {
  return is_anywhere && o.is_anywhere
     && min.x <= o.max.x && o.min.x <= max.x
     && min.y <= o.max.y && o.min.y <= max.y
     && min.z <= o.max.z && o.min.z <= max.z;
}
void bounding_box::combine_with(bounding_box const& o) {
       if (!o.is_anywhere) {            return; }
  else if (!  is_anywhere) { *this = o; return; }
  else {
    if (o.min.x < min.x) min.x = o.min.x;
    if (o.min.y < min.y) min.y = o.min.y;
    if (o.min.z < min.z) min.z = o.min.z;
    if (o.max.x > max.x) max.x = o.max.x;
    if (o.max.y > max.y) max.y = o.max.y;
    if (o.max.z > max.z) max.z = o.max.z;
  }
}
void bounding_box::restrict_to(bounding_box const& o) {
       if (!  is_anywhere) {                      return; }
  else if (!o.is_anywhere) { is_anywhere = false; return; }
  else {
    if (o.min.x > min.x) min.x = o.min.x;
    if (o.min.y > min.y) min.y = o.min.y;
    if (o.min.z > min.z) min.z = o.min.z;
    if (o.max.x < max.x) max.x = o.max.x;
    if (o.max.y < max.y) max.y = o.max.y;
    if (o.max.z < max.z) max.z = o.max.z;
    if (min.x > max.x || min.y > max.y || min.z > max.z) is_anywhere = false;
  }
}

/*
shape::shape(bounding_box const& init): bounds_cache_is_valid(true) {
  bounds_cache = init;
  if (!init.is_anywhere) return;
  
  for (int i = 0; i < 3; ++i) {
    std::vector<vector3<int64_t>> vertices1, vertices2;
    vector3<int64_t> base2(init.min); base2[i] = init.max[i];
    vector3<int64_t> diff1(0,0,0), diff2(0,0,0);
    diff1[(i+1)%3] = init.max[(i+1)%3] - init.min[(i+1)%3];
    diff2[(i+2)%3] = init.max[(i+2)%3] - init.min[(i+2)%3];
    vertices1.push_back(init.min                );
    vertices1.push_back(init.min + diff1        );
    vertices1.push_back(init.min + diff1 + diff2);
    vertices1.push_back(init.min         + diff2);
    vertices2.push_back(   base2                );
    vertices2.push_back(   base2 + diff1        );
    vertices2.push_back(   base2 + diff1 + diff2);
    vertices2.push_back(   base2         + diff2);
    
    polygons.push_back(convex_polygon(vertices1));
    polygons.push_back(convex_polygon(vertices2));
  }
}
*/
  
void convex_polygon::setup_cache_if_needed()const {
  if (cache.is_valid) return;
  
  cache.adjusted_vertices = vertices;
  
  // Translate everything to a more convenient location. Translations are linear and hence preserve everything we need.
  cache.translation_amount = -vertices[0];
  for (vector3<int64_t> &v : cache.adjusted_vertices) v += cache.translation_amount;
  
  cache.amount_twisted = 0;
  while (true) {
    cache.denom = (cache.adjusted_vertices[1].x*cache.adjusted_vertices[2].y - cache.adjusted_vertices[1].y*cache.adjusted_vertices[2].x);
    if (cache.denom != 0) break;
    
    // One of the three orientations must work.
    // These are rotations, which are linear and hence preserve everything we need.
    
    ++cache.amount_twisted;
    for (vector3<int64_t> &v : cache.adjusted_vertices) v = vector3<int64_t>(v.y, v.z, v.x);
    assert(cache.amount_twisted <= 2);
  }
  
  // In the formula Skew(T) = I + (z unit vector)*(a(t.x) + b(t.y)) ...
  cache.a_times_denom =  ((cache.adjusted_vertices[2].z*cache.adjusted_vertices[1].y)
                        - (cache.adjusted_vertices[1].z*cache.adjusted_vertices[2].y));
  cache.b_times_denom = -((cache.adjusted_vertices[2].z*cache.adjusted_vertices[1].x)
                        - (cache.adjusted_vertices[1].z*cache.adjusted_vertices[2].x));
  
  // We don't actually need to skew the polygon, since (by definition) it just skews the z coordinate to zero,
  // and because of that, hereafter we just don't need to refer to the z coordinates at all.
}

void bounding_box::translate(vector3<int64_t> t) {
  min += t; max += t;
}

void line_segment::translate(vector3<int64_t> t) {
  for (vector3<int64_t> &v : ends) v += t;
}

void convex_polygon::translate(vector3<int64_t> t) {
  for (vector3<int64_t> &v : vertices) v += t;
  cache.translation_amount -= t;
}

void shape::translate(vector3<int64_t> t) {
  for (  line_segment &l : segments) l.translate(t);
  for (convex_polygon &p : polygons) p.translate(t);
  for (  bounding_box &b : boxes   ) b.translate(t);
  if (bounds_cache_is_valid && bounds_cache.is_anywhere) {
    bounds_cache.min += t;
    bounds_cache.max += t;
    // now still valid!
  }
}

bounding_box line_segment::bounds()const {
  bounding_box result;
  for (vector3<int64_t> const& v : ends) result.combine_with(bounding_box(v));
  return result;
}

bounding_box convex_polygon::bounds()const {
  bounding_box result;
  for (vector3<int64_t> const& v : vertices) result.combine_with(bounding_box(v));
  return result;
}

bounding_box shape::bounds()const {
  if (bounds_cache_is_valid) return bounds_cache;
  bounds_cache = bounding_box();
  for (  line_segment const& l : segments) bounds_cache.combine_with(l.bounds());
  for (convex_polygon const& p : polygons) bounds_cache.combine_with(p.bounds());
  for (  bounding_box const& b : boxes   ) bounds_cache.combine_with(b         );
  bounds_cache_is_valid = true;
  return bounds_cache;
}

vector3<int64_t> shape::arbitrary_interior_point()const {
  if (!segments.empty()) return segments[0].ends[0];
  if (!polygons.empty()) return polygons[0].get_vertices()[0];
  for (bounding_box const& bb : boxes) { if (bb.is_anywhere) { return bb.min; }}
  caller_error("Trying to get an arbitrary interior point of a shape that contains no points");
}

std::pair<bool, boost::rational<int64_t>> get_intersection(int64_t sl1x, int64_t sl1y, int64_t sl2x, int64_t sl2y, int64_t ol1x, int64_t ol1y, int64_t ol2x, int64_t ol2y) {
  // assume ol1 is (0, 0)
  ol2x -= ol1x;
  sl1x -= ol1x;
  sl2x -= ol1x;
  ol2y -= ol1y;
  sl1y -= ol1y;
  sl2y -= ol1y;
  if (ol2x == 0) {
    return get_intersection(sl1y, sl1x, sl2y, sl2x, 0, 0, ol2y, ol2x);
  }
  const int64_t D = ol2x;
  const int64_t A = -ol2y;
  const int64_t Dy1 = D*sl1y + A*sl1x;
  const int64_t Dy2 = D*sl2y + A*sl2x;
  const int64_t Dy2mDy1 = Dy2 - Dy1;
  const int64_t ltx_Dy2mDy1 = sl1x * Dy2 - sl2x * Dy1;
  if (ltx_Dy2mDy1 < 0 || ltx_Dy2mDy1 > ol2x*Dy2mDy1) return std::make_pair(false, 1);
  else                                               return std::make_pair(true, boost::rational<int64_t>(Dy1, Dy1 - Dy2));
}

std::pair<bool, boost::rational<int64_t>> get_intersection(line_segment const& l, bounding_box const& bb) {
  if (!bb.is_anywhere) return std::make_pair(false, 1);
  if (bb.contains(l.ends[0])) return std::make_pair(true, 0);
  
  boost::rational<int64_t> intersecting_min(0);
  boost::rational<int64_t> intersecting_max(1);
  
#define CHECK(minormax, maxormin, compare, dimension) \
  if (l.ends[0].dimension == l.ends[1].dimension) { \
    if (l.ends[0].dimension compare bb.minormax.dimension) return std::make_pair(false, 1); \
  } \
  else { \
    const boost::rational<int64_t> checkval((l.ends[1].dimension > l.ends[0].dimension ? bb.minormax.dimension : bb.maxormin.dimension) - l.ends[0].dimension, l.ends[1].dimension - l.ends[0].dimension); \
    if (intersecting_##minormax compare checkval) { \
      intersecting_##minormax = checkval; \
      if (intersecting_min > intersecting_max) return std::make_pair(false, 1); \
    } \
  }
  
  CHECK(min, max, <, x)
  CHECK(min, max, <, y)
  CHECK(min, max, <, z)
  CHECK(max, min, >, x)
  CHECK(max, min, >, y)
  CHECK(max, min, >, z)
#undef CHECK
  
  return std::make_pair(true, intersecting_min);
}

std::pair<bool, boost::rational<int64_t>> get_intersection(line_segment l, convex_polygon const& p) {
  p.setup_cache_if_needed();
  polygon_collision_info_cache const& c = p.get_cache();
  
  // Translate and twist, as we did with the polygon.
  for (vector3<int64_t> &v : l.ends) v += c.translation_amount;
  for (vector3<int64_t> &v : l.ends) v = vector3<int64_t>(v[(0 + c.amount_twisted) % 3], v[(1 + c.amount_twisted) % 3], v[(2 + c.amount_twisted) % 3]);
  // Now skew the z values. Skews are linear and hence preserve everything we need.
  // The line's z values are scaled up as well as skewed.
  for (vector3<int64_t> &v : l.ends) { v.z = v.z * c.denom + (c.a_times_denom * v.x + c.b_times_denom * v.y); }
  
  if (sign(l.ends[0].z) == sign(l.ends[1].z)) {
    if (l.ends[0].z != 0) {
      // If the endpoints are on the same side, they're not colliding, obviously!
      return std::make_pair(false, 1);
    }
    else {
      // Now, we need to do 2D line vs. polygon collisions, which are just a bunch of 2D line vs. line collisions.
      // We just ignore the z values for these purposes.
      std::pair<bool, boost::rational<int64_t>> result(false, 1);
      for (size_t i = 0; i < c.adjusted_vertices.size(); ++i) {
        const int next_i = (i + 1) % c.adjusted_vertices.size();
        std::pair<bool, boost::rational<int64_t>> here = get_intersection(l.ends[0].x, l.ends[0].y, l.ends[1].x, l.ends[1].y, c.adjusted_vertices[i].x, c.adjusted_vertices[i].y, c.adjusted_vertices[next_i].x, c.adjusted_vertices[next_i].y);
        if (here.first && (!result.first || here.second < result.second)) {
          result = here;
        }
      }
      return result;
    }
  }
  
  const int64_t denom2 = l.ends[1].z - l.ends[0].z;
  // Find the point in the plane (scaled up by denom2, which was scaled up by denom...)
  
  const vector3<int64_t> point_in_plane_times_denom2(
    l.ends[1].z*l.ends[0].x - l.ends[0].z*l.ends[1].x,
    l.ends[1].z*l.ends[0].y - l.ends[0].z*l.ends[1].y,
    0
  );
  
  // Don't assume which clockwiseness the polygon is - but the point can never be on the same side of all the lines if it's outside the polygon, and always will be if it's inside.
  int previous_clockwiseness = 0;
  for (size_t i = 0; i < c.adjusted_vertices.size(); ++i) {
    const int next_i = (i + 1) % c.adjusted_vertices.size();
    const int64_t clockwiseness = sign(
        ((point_in_plane_times_denom2.y - c.adjusted_vertices[i].y*denom2) * (c.adjusted_vertices[next_i].x - c.adjusted_vertices[i].x))
      - ((point_in_plane_times_denom2.x - c.adjusted_vertices[i].x*denom2) * (c.adjusted_vertices[next_i].y - c.adjusted_vertices[i].y))
    );
    if (clockwiseness != 0) {
      if (previous_clockwiseness == 0) {
        previous_clockwiseness = clockwiseness;
      }
      else {
        if (clockwiseness != previous_clockwiseness) return std::make_pair(false, boost::rational<int64_t>(1));
      }
    }
  }
  
  return std::make_pair(true, boost::rational<int64_t>(l.ends[0].z, l.ends[0].z - l.ends[1].z));
}

bool nonshape_intersects_onesided(convex_polygon const& p1, convex_polygon const& p2) {
  std::vector<vector3<int64_t>> const& vs = p1.get_vertices();
  for (size_t i = 0; i < vs.size(); ++i) {
    const int next_i = (i + 1) % vs.size();
    if (get_intersection(line_segment(vs[i], vs[next_i]), p2).first) return true;
  }
  return false;
}

bool nonshape_intersects(convex_polygon const& p1, convex_polygon const& p2) {
  return (nonshape_intersects_onesided(p1,p2) || nonshape_intersects_onesided(p2,p1));
}

bool nonshape_intersects(convex_polygon const& p, bounding_box const& bb) {
  if (!bb.is_anywhere) return false;
  
  std::vector<vector3<int64_t>> const& vs = p.get_vertices();
  if (bb.contains(vs[0])) return true;
  for (size_t i = 0; i < vs.size(); ++i) {
    const int next_i = (i + 1) % vs.size();
    if (get_intersection(line_segment(vs[i], vs[next_i]), bb).first) return true;
  }
  // TODO: come up with a nicer, generalizable way to do something for every edge of a bounding box
  for (int x = 0; x < 2; ++x) { for (int y = 0; y < 2; ++y) { for (int z = 0; z < 2; ++z) {
    vector3<int64_t> base(x ? bb.min.x : bb.max.x, y ? bb.min.y : bb.max.y, z ? bb.min.z : bb.max.z);
    if (x == 0) {
      vector3<int64_t> other = base; other.x = bb.max.x;
      if (get_intersection(line_segment(base, other), p).first) return true;
    }
    if (y == 0) {
      vector3<int64_t> other = base; other.y = bb.max.y;
      if (get_intersection(line_segment(base, other), p).first) return true;
    }
    if (z == 0) {
      vector3<int64_t> other = base; other.z = bb.max.z;
      if (get_intersection(line_segment(base, other), p).first) return true;
    }
  }}}
  return false;
}

bool shape::intersects(shape const& other)const {
  if (!bounds().overlaps(other.bounds())) return false;
  
  for (line_segment const& l : segments) {
    for (convex_polygon const& p2 : other.polygons) {
      if (get_intersection(l, p2).first) return true;
    }
    for (bounding_box const& b2 : other.boxes) {
      if (get_intersection(l, b2).first) return true;
    }
  }

  for (convex_polygon const& p1 : polygons) {
    for (line_segment const& l : other.segments) {
      if (get_intersection(l, p1).first) return true;
    }
    for (convex_polygon const& p2 : other.polygons) {
      if (nonshape_intersects(p1, p2)) return true;
    }
    for (bounding_box const& b2 : other.boxes) {
      if (nonshape_intersects(p1, b2)) return true;
    }
  }

  for (bounding_box const& b1 : boxes) {
    for (line_segment const& l : other.segments) {
      if (get_intersection(l, b1).first) return true;
    }
    for (convex_polygon const& p2 : other.polygons) {
      if (nonshape_intersects(p2, b1)) return true;
    }
    for (bounding_box const& b2 : other.boxes) {
      if (b1.overlaps(b2)) return true;
    }
  }
  return false;
}

std::pair<bool, boost::rational<int64_t>> shape::first_intersection(line_segment const& other)const {
  std::pair<bool, boost::rational<int64_t>> result(false, 1);
  for (convex_polygon const& p : polygons) {
    std::pair<bool, boost::rational<int64_t>> here = get_intersection(other, p);
    if (here.first && (!result.first || here.second < result.second)) {
      result = here;
    }
  }
  for (bounding_box const& bb : boxes) {
    std::pair<bool, boost::rational<int64_t>> here = get_intersection(other, bb);
    if (here.first && (!result.first || here.second < result.second)) {
      result = here;
    }
  }
  return result;
}

