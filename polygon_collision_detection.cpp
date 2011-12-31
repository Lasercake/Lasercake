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
addition to the z coordinate = ax + by
EQ1) -t1z = at1x + bt1y
EQ2) -t2z = at2x + bt2y

t2x*EQ1 - t1x*EQ2) t2zt1z-t1zt2x = b(t1yt2x - t2yt1x)
b = (t2zt1x-t1zt2x) / (t1yt2x - t2yt1x)
since a and b are only different in an x vs y way, logically...
a = (t2zt1y-t1zt2y) / (t1xt2y - t2xt1y)
(denom == 0 iff t1 and t2 are collinear in the plane - just rotate the triangle 90 degrees until they're not.)

We could just do the computations at a factor of that denominator for a little while.
D = (t1xt2y - t2xt1y) = the denominator of a
Let A = (t2zt1y-t1zt2y) = a*D
Let B = -(t2zt1x-t1zt2x) = b*D
Then we'd get two transformed z values...

Ideally...
z1 = l1z + al1x + bl1y
Multiply by D...
Dz1 = Dl1z + Al1x + Bl1y

We only care about the *proportion* between z1 and z2, so Dz1/Dz2 are just as good, as long as we don't get rounding error.
Both negative or both positive -> no collision
Otherwise, ideally...
let l1xy = l1 projected onto the plane, etc
ltxy = the point where (l1, l2) intersects the plane
ltxy = ((0 - z1) / (z2 - z1)) l2xy + ((z2 - 0) / (z2 - z1)) l1xy
More practically...
ltxy = ((0 - Dz1) / (Dz2 - Dz1)) l2xy + ((Dz2 - 0) / (Dz2 - Dz1)) l1xy
ltxy = ((0 - Dz1) l2xy + (Dz2 - 0) l1xy) / (Dz2 - Dz1)

Now we can multiply everything by (Dz2 - Dz1)...
(Dz2 - Dz1)ltxy = Dz2*l1xy - Dz1*l2xy

And also multiply the triangle coordinates by (Dz2 - Dz1), and then do the clockwiseness check above!

This is a perfect accuracy collision check.
The downside is that it (in the worst case) has *four* factors of the distances between two points,
  (That's before the clockwiseness function, but the clockwiseness function can be implemented here without squaring the magnitude)
and so we couldn't use values that differ by more than the fourth log of our coordinate type,
namely... 64/4 bits, or 16 bits.
(D = diff^2, D(z2 - z1) = diff^3, D(z2 - z1)ltxy = diff^4, )

*/

#include "polygon_collision_detection.hpp"

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

bool intersects(line_segment l, convex_polygon const& p) {
  p.setup_cache_if_needed();
  polygon_collision_info_cache const& c = p.get_cache();
  
  // Translate and twist, as we did with the polygon.
  for (vector3<int64_t> &v : l.ends) v += c.translation_amount;
  for (vector3<int64_t> &v : l.ends) v = vector3<int64_t>(v[(0 + c.amount_twisted) % 3], v[(1 + c.amount_twisted) % 3], v[(2 + c.amount_twisted) % 3]);
  // Now skew the line. Skews are linear and hence preserve everything we need.
  // The line is scaled up as well as skewed.
  for (vector3<int64_t> &v : l.ends) { v *= c.denom; v.z += (c.a_times_denom * v.x + c.b_times_denom * v.y); }
  
  // If the endpoints are on the same side, they're not colliding, obviously!
  if (sign(l.ends[0].z) * sign(l.ends[1].z) == 1) return false;
  
  const int64_t denom2 = l.ends[1].z - l.ends[0].z;
  // Find the point in the plane (scaled up by denom2, which was scaled up by denom...)
  
  const vector3<int64_t> point_in_plane_times_denom2(
    l.ends[1].z*l.ends[0].x - l.ends[0].z*l.ends[1].x,
    l.ends[0].z*l.ends[1].y - l.ends[1].z*l.ends[0].y,
    0
  );
  
  // Don't assume which clockwiseness the polygon is - but the point can never be on the same side of all the lines if it's outside the polygon, and always will be if it's inside.
  int previous_clockwiseness = 0;
  for (size_t i = 0; i < c.adjusted_vertices.size(); ++i) {
    const int next_i = (i + 1) % c.adjusted_vertices.size();
    const int64_t clockwiseness = (
        ((point_in_plane_times_denom2.y - c.adjusted_vertices[i].y*denom2) * (c.adjusted_vertices[next_i].x - c.adjusted_vertices[i].x))
      - ((point_in_plane_times_denom2.x - c.adjusted_vertices[i].x*denom2) * (c.adjusted_vertices[next_i].y - c.adjusted_vertices[i].y))
    );
    if (clockwiseness != 0) {
      if (previous_clockwiseness == 0) {
        previous_clockwiseness = clockwiseness;
      }
      else {
        if (clockwiseness != previous_clockwiseness) return false;
      }
    }
  }
  
  return true;
}

bool intersects(convex_polygon const& p1, convex_polygon const& p2) {
  std::vector<vector3<int64_t>> const& vs = p1.get_vertices();
  for (size_t i = 0; i < vs.size(); ++i) {
    const int next_i = (i + 1) % vs.size();
    if (intersects(line_segment(vs[i], vs[next_i]), p2)) return true;
  }
  return false;
}

bool intersects(line_segment const& l, std::vector<convex_polygon> const& ps) {
  for (convex_polygon const& p : ps) {
    if (intersects(l, p)) return true;
  }
  return false;
}

bool intersects(std::vector<convex_polygon> const& ps1, std::vector<convex_polygon> const& ps2) {
  for (convex_polygon const& p1 : ps1) {
    for (convex_polygon const& p2 : ps2) {
      if (intersects(p1, p2)) return true;
    }
  }
  return false;
}

