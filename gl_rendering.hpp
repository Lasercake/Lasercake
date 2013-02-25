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

#ifndef LASERCAKE_GL_RENDERING_HPP__
#define LASERCAKE_GL_RENDERING_HPP__

#include <boost/scoped_ptr.hpp>
#include "cxx11/atomic.hpp"
#include "gl_data_abstract.hpp"

// Avoid including any Qt headers because Qt headers and GLEW
// headers do not get along.
class LasercakeGLWidget;

typedef int viewport_dimension; // Qt uses 'int' for sizes.

// One instance of gl_renderer should exist per GL context.
// This ensures that its initing and state correspond with
// GL state correctly.
//
// gl_renderer is not guaranteed that its constructor or destructor
// are called with its GL context active.  It inits itself
// the first time output_gl_data_to_OpenGL() is called.
// To clean up those GL resources, call fini(), which also resets
// gl_renderer to a clean state.
class gl_renderer {
public:
  // These functions must be called with the relevant GL context active.
  void output_gl_data_to_OpenGL(
      abstract_gl_data const& gl_data,
      viewport_dimension viewport_width,
      viewport_dimension viewport_height,
      LasercakeGLWidget& gl_widget,
      // This allows the caller to interrupt the GL rendering,
      // in case it's taking a long time.
      // It's polled between GL calls.
      // When polled and found to be 'true', this function
      // returns before drawing everything.
      atomic::atomic_bool const volatile& interrupt
  );
  void fini();

  // The constructor and destructor needn't be.
  // explicit destructor for private-implementation idiom.
  gl_renderer();
  ~gl_renderer();

private:
  // implemented in gl_qt_usage.cpp using Qt:
  void render_2d_text_overlay_(
      abstract_gl_data const& gl_data,
      viewport_dimension viewport_width,
      viewport_dimension viewport_height,
      LasercakeGLWidget& gl_widget
  );

  struct state_t_;
  boost::scoped_ptr<state_t_> state_;
};

#endif
