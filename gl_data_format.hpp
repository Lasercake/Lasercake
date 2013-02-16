/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

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

#ifndef LASERCAKE_GL_DATA_FORMAT_HPP__
#define LASERCAKE_GL_DATA_FORMAT_HPP__

// This header does not use GL types (GLfloat, GLubyte) in order to
// be agnostic between GLEW and Qt's opinions of OpenGL headers.
// gl_data_preparation.cpp static_asserts that they're the types we
// expect.
// (This might not be necessary currently, but doesn't hurt anything
// (that I know of).)

#include <string>
#include <vector>
#include <ostream>
#include <stdlib.h> //malloc
#include <string.h> //memcpy

#include "world_constants.hpp"


namespace gl_data_format {

typedef float header_GLfloat;
typedef unsigned char header_GLubyte;

// These are to be passed as part of arrays to OpenGL.
// Thus their data format/layout makes a difference.
// TODO maybe make vector3<GLfloat> and vertex the same thing?
// Is vector3<GLfloat>'s layout required to be the same as that of
// this struct? I believe so.  typedef vector3<GLfloat> vertex; ?
struct vertex {
  vertex() {}

  vertex(header_GLfloat x, header_GLfloat y, header_GLfloat z) : x(x), y(y), z(z) {}
  /* implicit conversion */ vertex(vector3<header_GLfloat> const& v) : x(v.x), y(v.y), z(v.z) {}

  header_GLfloat x, y, z;
};
static_assert(sizeof(vertex) == 3*sizeof(header_GLfloat), "OpenGL needs this data layout.");
inline std::ostream& operator<<(std::ostream& os, vertex const& v) {
  // TODO does the float precision we output here matter?
  return os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
}

struct color {
  color() {}

  // Use hex RGBA values as familiar from e.g. CSS.
  // e.g. 0x00ff0077 is pure green at 50% opacity.
  explicit color(uint32_t rgba)
    : r(header_GLubyte(rgba >> 24)), g(header_GLubyte(rgba >> 16)),
      b(header_GLubyte(rgba >>  8)), a(header_GLubyte(rgba)) {}

  color(header_GLubyte r, header_GLubyte g, header_GLubyte b, header_GLubyte a)
    : r(r), g(g), b(b), a(a) {}

  // random Web forums thought a factor of 255.0 was correct, but is it exactly the right conversion?
  color(header_GLfloat r, header_GLfloat g, header_GLfloat b, header_GLfloat a)
   : r(header_GLubyte(r*255)), g(header_GLubyte(g*255)), b(header_GLubyte(b*255)), a(header_GLubyte(a*255)) {}

  header_GLubyte r, g, b, a;
};
static_assert(sizeof(color) == 4*sizeof(header_GLubyte), "OpenGL needs this data layout.");
inline std::ostream& operator<<(std::ostream& os, color const& c) {
  // TODO does the float precision we output here matter?
  return os << "rgba(" << std::hex << int(c.r) << int(c.g) << int(c.b) << int(c.a) << std::dec << ')';
}

struct vertex_with_color {
  vertex_with_color() {}

  vertex_with_color(vertex v, color c) : c(c), v(v) {}
  vertex_with_color(header_GLfloat x, header_GLfloat y, header_GLfloat z, color c)
    : c(c), v(x,y,z) {}
  color c;
  vertex v;
};
static_assert(sizeof(vertex_with_color) == 16, "OpenGL needs this data layout.");
inline std::ostream& operator<<(std::ostream& os, vertex_with_color const& vc) {
  // TODO does the float precision we output here matter?
  return os << vc.c << '~' << vc.v;
}

struct gl_call_data {
  typedef uint32_t size_type;
  size_type count;
  size_type alloced;
  vertex_with_color* vertices;
  //vertex_with_color* vertices_end;
  static const size_type default_size = 5;
  static const size_type expand_multiplier = 4;
  gl_call_data()
    : count(0),
      alloced(default_size),
      vertices((vertex_with_color*)malloc(default_size*sizeof(vertex_with_color))) {
    assert(vertices);
  }
  gl_call_data(gl_call_data const& other)
    : count(other.count),
      alloced(other.alloced),
      vertices((vertex_with_color*)malloc(other.alloced*sizeof(vertex_with_color))) {
    assert(vertices);
    memcpy(vertices, other.vertices, count*sizeof(vertex_with_color));
  }
  gl_call_data& operator=(gl_call_data const& other) = delete;
  ~gl_call_data() { free(vertices); }
  void push_vertex(vertex_with_color const& v) {
    if(count == alloced) do_realloc(alloced * expand_multiplier);
    vertices[count] = v;
    ++count;
  }
  void push_vertex(vertex const& v, color const& c) {
    if(count == alloced) do_realloc(alloced * expand_multiplier);
    vertices[count].c = c;
    vertices[count].v = v;
    ++count;
  }
  void reserve_new_slots(size_type num_slots) {
    const size_type new_count = count + num_slots;
    if(new_count > alloced) do_realloc(new_count * expand_multiplier);
    count = new_count;
  }
  size_type size() const { return count; }
  void do_realloc(size_type new_size) {
    assert(new_size > alloced);
    vertex_with_color* new_vertices = (vertex_with_color*)malloc(new_size * sizeof(vertex_with_color));
    assert(new_vertices);
    memcpy(new_vertices, vertices, count*sizeof(vertex_with_color));
    free(vertices);
    vertices = new_vertices;
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
typedef std::vector<gl_collection> gl_collectionplex;

struct heads_up_display_text {
  // text may contain newlines, and will also
  // be soft-wrapped to fit on the screen.
  std::string text;
  color c;
  std::string font_name;
  int point_size;
  int horizontal_margin_in_pixels;
  int vertical_margin_in_pixels;
};

struct gl_all_data {
  gl_collectionplex stuff_to_draw_as_gl_collections_by_distance;
  color tint_everything_with_this_color;
  heads_up_display_text hud_text;
  vector3<header_GLfloat> facing;
  vector3<header_GLfloat> facing_up;
};

} // end namespace gl_data_format

#endif
