/*

    Copyright Eli Dupree and Isaac Dupree, 2013

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

#ifndef LASERCAKE_GL_DATA_ABSTRACT_HPP__
#define LASERCAKE_GL_DATA_ABSTRACT_HPP__

namespace gl_data_format {
  struct gl_all_data;
}

// This lets you create and pass around GL data while it's
// an incomplete type.  It doesn't (currently) let you copy it though.
struct abstract_gl_data {
  abstract_gl_data();
  ~abstract_gl_data();
  abstract_gl_data(abstract_gl_data const&) = delete;
  abstract_gl_data& operator=(abstract_gl_data const&) = delete;

  gl_data_format::gl_all_data& data() { return data_; }
  gl_data_format::gl_all_data const& data()const { return data_; }
private:
  gl_data_format::gl_all_data& data_;
};

#endif
