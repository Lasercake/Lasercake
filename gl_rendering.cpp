/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <vector>
#include <array>

#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>

#include "gl_rendering.hpp"
#include "gl_data_format.hpp"

using namespace gl_data_format;

#define BUFFER_OFFSET(i) ((void*)(i))
const GLuint INVALID_BUFFER_ID = 0;

// by_distance_VBO_* are indexed by distance*DISTANCE_IDX_FACTOR + this enum:
enum by_distance_idx_adjustment_enum {
  QUADS_IDX = 0, TRIANGLES_IDX, LINES_IDX, POINTS_IDX,
  DISTANCE_IDX_FACTOR
};

typedef std::array<vertex_with_color, 4> rect_type;

struct gl_renderer::state_t_ {
  GLuint rect_VBO_name;
  std::vector<GLuint> by_distance_VBO_names;
  std::vector<size_t> by_distance_VBO_sizes;
};

gl_renderer::gl_renderer() {}
gl_renderer::~gl_renderer() {}

void gl_renderer::output_gl_data_to_OpenGL(
    abstract_gl_data const& abstract_gl_data,
    viewport_dimension viewport_width,
    viewport_dimension viewport_height,
    LasercakeGLWidget& gl_widget,
    // volatile: Ensure that every load is in fact done,
    // even though they look redundant.
    std::atomic_bool const volatile& interrupt
) {
  // This code is intended to have no operations in it that can possibly
  // throw exceptions, to simplify dealing with OpenGL context state.
  // When allocating, e.g. via std::vector, wrap it in a try/catch
  // and do something sensible if there's an exception.

  gl_all_data const& gl_data = abstract_gl_data.data();

  if(interrupt.load(std::memory_order_relaxed)) {return;};

  if(!state_) {
    try {
      state_.reset(new state_t_);
    }
    catch(std::bad_alloc const&) {
      return;
    }

    const GLenum glew_init_err = glewInit();
    if(glew_init_err != GLEW_OK)
    {
      // give up
      state_.reset();
      return;
    }

    glGenBuffersARB(1, &state_->rect_VBO_name);
    glBindBufferARB(GL_ARRAY_BUFFER, state_->rect_VBO_name);
    glBufferDataARB(GL_ARRAY_BUFFER, sizeof(rect_type), nullptr, GL_STREAM_DRAW);

    if(interrupt.load(std::memory_order_relaxed)) {return;};
  }

  glViewport(0, 0, viewport_width, viewport_height);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  // Depth func LEQUAL not LESS.  We try to draw objects in back-to-front
  // order, so rounding error means the front (later) one should win.
  //glEnable(GL_DEPTH_TEST)
  //glDepthFunc(GL_LEQUAL);
  glClear(GL_COLOR_BUFFER_BIT/* | GL_DEPTH_BUFFER_BIT*/);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  const glm::mat4 projection_matrix =
    make_projection_matrix(float(viewport_width) / float(viewport_height));
  glLoadMatrixf(glm::value_ptr(projection_matrix));

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  const glm::mat4 view_matrix =
    make_view_matrix(gl_data.facing, gl_data.facing_up);
  glLoadMatrixf(glm::value_ptr(view_matrix));

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  if(interrupt.load(std::memory_order_relaxed)) {return;};

  if(gl_data.stuff_to_draw_as_gl_collections_by_distance.size() > state_->by_distance_VBO_names.size()) {
    const size_t new_buffers_base = state_->by_distance_VBO_names.size()*DISTANCE_IDX_FACTOR;
    const size_t num_new_buffers = gl_data.stuff_to_draw_as_gl_collections_by_distance.size()*DISTANCE_IDX_FACTOR - new_buffers_base;
    try {
      state_->by_distance_VBO_names.resize(new_buffers_base + num_new_buffers);
      state_->by_distance_VBO_sizes.resize(new_buffers_base + num_new_buffers);
    }
    catch(std::bad_alloc const&) {
      return;
    }
    glGenBuffersARB(num_new_buffers, &state_->by_distance_VBO_names[new_buffers_base]);
    for(size_t i = 0; i != num_new_buffers; ++i) {
      if(interrupt.load(std::memory_order_relaxed)) {return;};
      state_->by_distance_VBO_sizes[new_buffers_base + i] = 0;
    }
  }

  for(size_t dist_plus_one = gl_data.stuff_to_draw_as_gl_collections_by_distance.size(); dist_plus_one != 0; --dist_plus_one) {
    if(interrupt.load(std::memory_order_relaxed)) {return;};
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
      if(interrupt.load(std::memory_order_relaxed)) {return;};
      gl_call_data const& data = coll.*(type.gl_data_container_ptr_to_member);
      if(const size_t count = data.size()) {
        const size_t buf_name_idx = dist*DISTANCE_IDX_FACTOR + type.our_idx_adj;
        glBindBufferARB(GL_ARRAY_BUFFER, state_->by_distance_VBO_names[buf_name_idx]);
        if(state_->by_distance_VBO_sizes[buf_name_idx] < count) {
          glBufferDataARB(GL_ARRAY_BUFFER, count*sizeof(vertex_with_color), data.vertices, GL_STREAM_DRAW);
          state_->by_distance_VBO_sizes[buf_name_idx] = count;
        }
        else {
          glBufferSubDataARB(GL_ARRAY_BUFFER, 0, count*sizeof(vertex_with_color), data.vertices);
        }
        glInterleavedArrays(GL_C4UB_V3F, 0, BUFFER_OFFSET(0));
        glDrawArrays(type.gl_type, 0, count);
      }
    }
  }
  if(interrupt.load(std::memory_order_relaxed)) {return;};

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
  glBindBufferARB(GL_ARRAY_BUFFER, state_->rect_VBO_name);
  glBufferSubDataARB(GL_ARRAY_BUFFER, 0, sizeof(rect), &rect[0]);
  glInterleavedArrays(GL_C4UB_V3F, 0, BUFFER_OFFSET(0));
  glDrawArrays(GL_QUADS, 0, 4);
  glBindBufferARB(GL_ARRAY_BUFFER, INVALID_BUFFER_ID);

  if(interrupt.load(std::memory_order_relaxed)) {return;};

  render_2d_text_overlay_(abstract_gl_data, viewport_width, viewport_height, gl_widget);
}

void gl_renderer::fini() {
  if(state_) {
    glDeleteBuffersARB(1, &state_->rect_VBO_name);
    glDeleteBuffersARB(state_->by_distance_VBO_names.size(),
                       &state_->by_distance_VBO_names[0]);
    // by_distance_VBO_sizes does not contain OpenGL-owned data
    state_.reset();
  }
}

