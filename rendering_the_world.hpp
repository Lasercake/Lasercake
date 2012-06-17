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

#ifndef LASERCAKE_RENDERING_THE_WORLD_HPP__
#define LASERCAKE_RENDERING_THE_WORLD_HPP__

#include <vector>
#include <unordered_map>

// for types e.g. GLfloat, GLubyte
#if defined(__APPLE__) || defined(__MACOSX__)
#include "OpenGL/gl.h"
#else
#include "GL/gl.h"
#endif

#include "world.hpp"
#include "input_representation.hpp"


namespace world_rendering {

// These are to be passed as part of arrays to OpenGL.
// Thus their data format/layout makes a difference.
// TODO maybe make vector3<GLfloat> and vertex the same thing?
// Is vector3<GLfloat>'s layout required to be the same as that of
// this struct? I believe so.  typedef vector3<GLfloat> vertex; ?
struct vertex {
  vertex(GLfloat x, GLfloat y, GLfloat z) : x(x), y(y), z(z) {}
  /* implicit conversion */ vertex(vector3<GLfloat> const& v) : x(v.x), y(v.y), z(v.z) {}

  GLfloat x, y, z;
};
static_assert(sizeof(vertex) == 3*sizeof(GLfloat), "OpenGL needs this data layout.");

struct color {

  // Use hex RGBA values as familiar from e.g. CSS.
  // e.g. 0x00ff0077 is pure green at 50% opacity.
  explicit color(uint32_t rgba)
   : r(GLubyte(rgba >> 24)), g(GLubyte(rgba >> 16)), b(GLubyte(rgba >> 8)), a(GLubyte(rgba)) {}

  color(GLubyte r, GLubyte g, GLubyte b, GLubyte a) : r(r), g(g), b(b), a(a) {}

  // random Web forums thought a factor of 255.0 was correct, but is it exactly the right conversion?
  color(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
   : r(GLubyte(r*255)), g(GLubyte(g*255)), b(GLubyte(b*255)), a(GLubyte(a*255)) {}

  GLubyte r, g, b, a;
};
static_assert(sizeof(color) == 4*sizeof(GLubyte), "OpenGL needs this data layout.");


struct gl_call_data {
  typedef uint32_t size_type;
  size_type count;
  size_type alloced;
  vertex* vertices;
  color* colors;
  //vertex* vertices_end;
  //color* colors_end;
  static const size_type default_size = 5;
  static const size_type expand_multiplier = 4;
  gl_call_data()
    : count(0),
      alloced(default_size),
      vertices((vertex*)malloc(default_size*sizeof(vertex))),
      colors((color*)malloc(default_size*sizeof(color))) {
    assert(vertices);
    assert(colors);
  }
  gl_call_data(gl_call_data const& other)
    : count(other.count),
      alloced(other.alloced),
      vertices((vertex*)malloc(other.alloced*sizeof(vertex))),
      colors((color*)malloc(other.alloced*sizeof(color))) {
    assert(vertices);
    assert(colors);
    memcpy(vertices, other.vertices, count*sizeof(vertex));
    memcpy(colors, other.colors, count*sizeof(color));
  }
  gl_call_data& operator=(gl_call_data const& other) = delete;
  ~gl_call_data() { free(vertices); free(colors); }
  void push_vertex(vertex const& v, color const& c) {
    if(count == alloced) do_realloc();
    vertices[count] = v;
    colors[count] = c;
    ++count;
  }
  size_type size() const { return count; }
  void do_realloc() {
    const size_type new_size = alloced * expand_multiplier;
    assert(new_size > alloced);
    vertex* new_vertices = (vertex*)malloc(new_size * sizeof(vertex));
    color* new_colors = (color*)malloc(new_size * sizeof(color));
    assert(new_vertices);
    assert(new_colors);
    memcpy(new_vertices, vertices, count*sizeof(vertex));
    memcpy(new_colors, colors, count*sizeof(color));
    free(vertices);
    free(colors);
    vertices = new_vertices;
    colors = new_colors;
    alloced = new_size;
  }
};

struct gl_collection {
  // Points and lines are for debugging, because they don't change size at distance,
  // and TODO can be represented reasonably by triangles.
  // Triangles are legit.
  // Quads are TODO OpenGL prefers you to use triangles (esp. OpenGL ES) and
  // they'll be converted at some point.
  gl_call_data points;
  gl_call_data lines;
  gl_call_data triangles;
  gl_call_data quads;
};

//The gl_collection:s with higher indices here are intended to be
//further away and rendered first (therefore covered up most
//by everything else that's closer).
typedef std::unordered_map<size_t, gl_collection> gl_collectionplex;

struct gl_all_data {
  gl_collectionplex stuff_to_draw_as_gl_collections_by_distance;
  vector3<GLfloat> facing;
  vector3<GLfloat> facing_up;
};

} // end namespace world_rendering


struct world_rendering_config {
  world_rendering_config(fine_scalar view_radius) : view_radius(view_radius) {}
  fine_scalar view_radius;
};

class view_on_the_world {
public:
  view_on_the_world(object_identifier robot_id, vector3<fine_scalar> approx_initial_center);

  // Call this every frame that you want user-input to have view-related effects.
  void input(input_representation::input_news_t const& input_news);

  // To call, default-construct the result and then pass it as a reference.
  void render(world /*TODO const*/& w, world_rendering_config c, world_rendering::gl_all_data& result);

  // TODO make private and/or trailing underscore
  object_identifier robot_id;
  vector3<fine_scalar> view_loc_for_local_display;
  enum { GLOBAL, LOCAL, ROBOT } view_type;
  double view_direction;
  vector3<fine_scalar> surveilled_by_global_display;
  fine_scalar globallocal_view_dist;
  bool drawing_regular_stuff;
  bool drawing_debug_stuff;
};


#endif
