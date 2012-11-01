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

#define GL_GLEXT_PROTOTYPES 1

#if defined(__APPLE__) || defined(__MACOSX__)
#include "OpenGL/gl.h"
#include "OpenGL/glu.h"
#else
#include "GL/gl.h"
#include "GL/glu.h"
#endif

#include "gl_rendering.hpp"

void output_gl_data_to_OpenGL(
    gl_data_preparation::gl_all_data const& gl_data,
    viewport_dimension viewport_width,
    viewport_dimension viewport_height,
    LasercakeGLWidget& gl_widget
) {
  using namespace gl_data_preparation;
  #define BUFFER_OFFSET(i) ((void*)(i))
  const GLuint INVALID_BUFFER_ID = 0;

  // This code is intended to have no operations in it that can possibly
  // throw exceptions, to simplify dealing with OpenGL context state.
  // When allocating, e.g. via std::vector, wrap it in a try/catch
  // and do something sensible if there's an exception.

  // TODO perhaps these static data should be member variables and this
  // function should be a class instead.
  static bool gl_inited = false;
  static GLuint rect_VBO_name = INVALID_BUFFER_ID;

  // by_distance_VBO_* indexed by distance*DISTANCE_IDX_FACTOR + this enum:
  enum by_distance_idx_adjustment_enum {
    QUADS_IDX = 0, TRIANGLES_IDX, LINES_IDX, POINTS_IDX,
    DISTANCE_IDX_FACTOR
  };
  static std::vector<GLuint> by_distance_VBO_names;
  static std::vector<size_t> by_distance_VBO_sizes;
  typedef std::array<vertex_with_color, 4> rect_type;
  if(!gl_inited) {
    glGenBuffers(1, &rect_VBO_name);
    glBindBuffer(GL_ARRAY_BUFFER, rect_VBO_name);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rect_type), nullptr, GL_STREAM_DRAW);
    gl_inited = true;
  }
  // TODO Apparently atexit we should glDeleteBuffers, glDisable, and stuff?

  glViewport(0, 0, viewport_width, viewport_height);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  // Depth func LEQUAL not LESS.  We try to draw objects in back-to-front
  // order, so rounding error means the front (later) one should win.
  //glEnable(GL_DEPTH_TEST)
  //glDepthFunc(GL_LEQUAL);
  glClear(GL_COLOR_BUFFER_BIT/* | GL_DEPTH_BUFFER_BIT*/);
  glLoadIdentity();

  // TODO convert these GLU calls to plain GL calls?
  gluPerspective(80, (double(viewport_width) / viewport_height), 0.1*get_primitive_double(tile_width), 300.0*get_primitive_double(tile_width));
  gluLookAt(0, 0, 0,
            gl_data.facing.x, gl_data.facing.y, gl_data.facing.z,
            gl_data.facing_up.x, gl_data.facing_up.y, gl_data.facing_up.z);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  if(gl_data.stuff_to_draw_as_gl_collections_by_distance.size() > by_distance_VBO_names.size()) {
    const size_t new_buffers_base = by_distance_VBO_names.size()*DISTANCE_IDX_FACTOR;
    const size_t num_new_buffers = gl_data.stuff_to_draw_as_gl_collections_by_distance.size()*DISTANCE_IDX_FACTOR - new_buffers_base;
    try {
      by_distance_VBO_names.resize(new_buffers_base + num_new_buffers);
      by_distance_VBO_sizes.resize(new_buffers_base + num_new_buffers);
    }
    catch(std::bad_alloc const&) {
      return;
    }
    glGenBuffers(num_new_buffers, &by_distance_VBO_names[new_buffers_base]);
    for(size_t i = 0; i != num_new_buffers; ++i) {
      by_distance_VBO_sizes[new_buffers_base + i] = 0;
    }
  }

  for(size_t dist_plus_one = gl_data.stuff_to_draw_as_gl_collections_by_distance.size(); dist_plus_one != 0; --dist_plus_one) {
    const size_t dist = dist_plus_one - 1;
    gl_collection const& coll = gl_data.stuff_to_draw_as_gl_collections_by_distance[dist];
    struct polygon_type {
      GLenum gl_type;
      by_distance_idx_adjustment_enum our_idx_adj;
      gl_call_data gl_collection::* gl_data_container_ptr_to_member;
    };
    const std::array<polygon_type, DISTANCE_IDX_FACTOR> types = {{
      { GL_QUADS, QUADS_IDX, &gl_collection::quads },
      { GL_TRIANGLES, TRIANGLES_IDX, &gl_collection::triangles },
      { GL_LINES, LINES_IDX, &gl_collection::lines },
      { GL_POINTS, POINTS_IDX, &gl_collection::points },
    }};
    for (polygon_type type : types) {
      gl_call_data const& data = coll.*(type.gl_data_container_ptr_to_member);
      if(const size_t count = data.size()) {
        const size_t buf_name_idx = dist*DISTANCE_IDX_FACTOR + type.our_idx_adj;
        glBindBuffer(GL_ARRAY_BUFFER, by_distance_VBO_names[buf_name_idx]);
        if(by_distance_VBO_sizes[buf_name_idx] < count) {
          glBufferData(GL_ARRAY_BUFFER, count*sizeof(vertex_with_color), data.vertices, GL_STREAM_DRAW);
          by_distance_VBO_sizes[buf_name_idx] = count;
        }
        else {
          glBufferSubData(GL_ARRAY_BUFFER, 0, count*sizeof(vertex_with_color), data.vertices);
        }
        glInterleavedArrays(GL_C4UB_V3F, 0, BUFFER_OFFSET(0));
        glDrawArrays(type.gl_type, 0, count);
      }
    }
  }

  // Is there a simpler way to tint the whole screen a color?
  const color tint = gl_data.tint_everything_with_this_color;
  const rect_type rect = {{
    vertex_with_color(0, 0, 0, tint),
    vertex_with_color(0, 1, 0, tint),
    vertex_with_color(1, 1, 0, tint),
    vertex_with_color(1, 0, 0, tint)
  }};
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1);
  glBindBuffer(GL_ARRAY_BUFFER, rect_VBO_name);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(rect), &rect[0]);
  glInterleavedArrays(GL_C4UB_V3F, 0, BUFFER_OFFSET(0));
  glDrawArrays(GL_QUADS, 0, 4);
  glBindBuffer(GL_ARRAY_BUFFER, INVALID_BUFFER_ID);

  render_2d_text_overlay(gl_data, viewport_width, viewport_height, gl_widget);

  #undef BUFFER_OFFSET
}
