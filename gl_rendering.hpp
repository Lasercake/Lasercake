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

#ifndef LASERCAKE_GL_RENDERING_HPP__
#define LASERCAKE_GL_RENDERING_HPP__

#include "gl_data_preparation.hpp"

// Avoid including any Qt headers because Qt headers and GLEW
// headers do not get along.
class LasercakeGLWidget;

typedef int viewport_dimension; // Qt uses 'int' for sizes.

void output_gl_data_to_OpenGL(
    gl_data_preparation::gl_all_data const& gl_data,
    viewport_dimension viewport_width,
    viewport_dimension viewport_height,
    LasercakeGLWidget& gl_widget
);

// implemented in main.cpp using Qt:
void render_2d_text_overlay(
    gl_data_preparation::gl_all_data const& gl_data,
    viewport_dimension viewport_width,
    viewport_dimension viewport_height,
    LasercakeGLWidget& gl_widget
);

#endif
