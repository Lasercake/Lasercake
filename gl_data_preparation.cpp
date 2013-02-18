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

#include <GL/glew.h>

#include "gl_data_preparation.hpp"
#include "gl_data_format.hpp"
#include "world.hpp"

#include "specific_object_types.hpp"
#include "tile_physics.hpp" // to access internals for debugging-displaying...

#include "tile_iteration.hpp"

using namespace gl_data_format;

static_assert(boost::is_same<header_GLfloat, GLfloat>::value, "consistent GL types");
static_assert(boost::is_same<header_GLubyte, GLubyte>::value, "consistent GL types");

abstract_gl_data::abstract_gl_data() : data_(*new gl_all_data()) {}
abstract_gl_data::~abstract_gl_data() { delete &data_; }

std::ostream& operator<<(std::ostream& os, glm::vec4 v) {
  return os << '(' << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ')';
}

// what about shifting the bounding-cube to near the main thing, or float rounding error...
template<typename Bbox>
inline bool overlaps(frustum const& f, Bbox const& bbox) {
  //std::cerr << "frustoverlaps?\n";
  for(glm::vec4 const& half_space : f.half_spaces) {
    const glm::vec4 extremity(
      get_primitive_int(bbox.min(X)) + (-int(half_space.x >= 0) & get_primitive_int(bbox.size(X))),
      get_primitive_int(bbox.min(Y)) + (-int(half_space.y >= 0) & get_primitive_int(bbox.size(Y))),
      get_primitive_int(bbox.min(Z)) + (-int(half_space.z >= 0) & get_primitive_int(bbox.size(Z))),
      1
    );
    //std::cerr << "frustoverlaps  " << half_space << "  " << extremity << "\n";
    if(glm::dot(half_space, extremity) < 0) {
      //std::cerr << "frustoverlaps=no\n";
      return false;
    }
  }
  //std::cerr << "frustoverlaps!!\n";
  return true;
}
template<typename Bbox>
inline bool subsumes(frustum const& f, Bbox const& bbox) {
  //std::cerr << "frustsubsumes?\n";
  for(glm::vec4 const& half_space : f.half_spaces) {
    const glm::vec4 extremity(
      get_primitive_int(bbox.min(X)) + (-int(half_space.x < 0) & get_primitive_int(bbox.size(X))),
      get_primitive_int(bbox.min(Y)) + (-int(half_space.y < 0) & get_primitive_int(bbox.size(Y))),
      get_primitive_int(bbox.min(Z)) + (-int(half_space.z < 0) & get_primitive_int(bbox.size(Z))),
      1
    );
    //std::cerr << "frustsubsumes  " << half_space << "  " << extremity << "\n";
    if(glm::dot(half_space, extremity) < 0) {
      //std::cerr << "frustsubsumes=no\n";
      return false;
    }
  }
  //std::cerr << "frustsubsumes!!\n";
  return true;
}

// there's not really any need for it to be normalized, though we easily could
// half_space = glm::normalize(half_space)
frustum convert_frustum_from_fine_distance_units_to_tile_count_units(frustum f) {
  for(glm::vec4& half_space : f.half_spaces) {
    half_space.x *= get_primitive_int(tile_width);
    half_space.y *= get_primitive_int(tile_width);
    half_space.z *= get_primitive_int(tile_height);
    // w unchanged
  }
  return f;
}

namespace /* anonymous */ {

// Units: we *could* use units with the floating-point numbers,
// but we'd have to cast them away with every GL call, which would
// get tedious.  Hmm.

template<typename Int>
inline vector3<GLfloat> cast_vector3_to_float(vector3<Int> v) {
  return vector3<GLfloat>(get_primitive_float(v.x), get_primitive_float(v.y), get_primitive_float(v.z));
}
template<typename Int>
inline vector3<double> cast_vector3_to_double(vector3<Int> v) {
  return vector3<double>(get_primitive_double(v.x), get_primitive_double(v.y), get_primitive_double(v.z));
}

inline GLfloat convert_distance_to_GL(distance distance) {
  return get_primitive_float(distance / fine_distance_units);
}
inline vector3<GLfloat> convert_displacement_to_GL(vector3<distance> distance) {
  return cast_vector3_to_float(distance / fine_distance_units);
}

inline double convert_distance_to_double(distance distance) {
  return get_primitive_double(distance / fine_distance_units);
}
inline vector3<double> convert_displacement_to_double(vector3<distance> distance) {
  return cast_vector3_to_double(distance / fine_distance_units);
}

static const vector3<GLfloat> tile_size_float = convert_displacement_to_GL(tile_size);
static const vector3<double> tile_size_double = convert_displacement_to_double(tile_size);

inline vector3<GLfloat> convert_coordinates_to_GL(vector3<distance> view_center, vector3<distance> input) {
  return cast_vector3_to_float((input - view_center) / fine_distance_units);
}

inline vector3<GLfloat> convert_tile_coordinates_to_GL(vector3<double> view_center_double, vector3<tile_coordinate> input) {
  // (Floats don't have enough precision to represent tile_coordinates exactly,
  // which before subtraction they must do. Doubles do.)
  vector3<double> tile_distance_min = cast_vector3_to_double(input).multiply_piecewise_by(tile_size_double);
  vector3<GLfloat> tile_gl_min(tile_distance_min - view_center_double);
  return tile_gl_min;
}


void push_vertex(gl_call_data& data, vertex const& v, color const& c) {
  data.push_vertex(v, c);
}

// There are versions of these with one color passed for all vertices to share (for convenience),
// and with the ability to specify one color per vertex,

void push_point(gl_collection& coll,
                vertex const& v, color const& c) {
  push_vertex(coll.points, v, c);
}

void push_line(gl_collection& coll,
               vertex const& v1,
               vertex const& v2, color const& c) {
  push_vertex(coll.lines, v1, c);
  push_vertex(coll.lines, v2, c);
}
void push_line(gl_collection& coll,
               vertex const& v1, color const& c1,
               vertex const& v2, color const& c2) {
  push_vertex(coll.lines, v1, c1);
  push_vertex(coll.lines, v2, c2);
}

void push_triangle(gl_collection& coll,
               vertex const& v1,
               vertex const& v2,
               vertex const& v3, color const& c) {
  push_vertex(coll.triangles, v1, c);
  push_vertex(coll.triangles, v2, c);
  push_vertex(coll.triangles, v3, c);
}
void push_triangle(gl_collection& coll,
               vertex const& v1, color const& c1,
               vertex const& v2, color const& c2,
               vertex const& v3, color const& c3) {
  push_vertex(coll.triangles, v1, c1);
  push_vertex(coll.triangles, v2, c2);
  push_vertex(coll.triangles, v3, c3);
}

// TODO make push_quad push two triangles
void push_quad(gl_collection& coll,
               vertex const& v1,
               vertex const& v2,
               vertex const& v3,
               vertex const& v4, color const& c) {
  push_vertex(coll.quads, v1, c);
  push_vertex(coll.quads, v2, c);
  push_vertex(coll.quads, v3, c);
  push_vertex(coll.quads, v4, c);
}
void push_quad(gl_collection& coll,
               vertex const& v1, color const& c1,
               vertex const& v2, color const& c2,
               vertex const& v3, color const& c3,
               vertex const& v4, color const& c4) {
  push_vertex(coll.quads, v1, c1);
  push_vertex(coll.quads, v2, c2);
  push_vertex(coll.quads, v3, c3);
  push_vertex(coll.quads, v4, c4);
}

// TODO allow choosing each polygon vertex's color?
void push_convex_polygon(vector3<distance> const& view_loc,
                         gl_collection& coll,
                         std::vector<geom::vect> const& vertices,
                         color const& c) {
  if(vertices.size() >= 3) {
    // draw convex polygon via (sides - 2) triangles
    std::vector<geom::vect>::const_iterator vertices_i = vertices.begin();
    const std::vector<geom::vect>::const_iterator vertices_end = vertices.end();
    const vertex first_vertex(convert_coordinates_to_GL(view_loc, *vertices_i));
    ++vertices_i;
    vertex prev_vertex(convert_coordinates_to_GL(view_loc, *vertices_i));
    ++vertices_i;
    for (; vertices_i != vertices_end; ++vertices_i) {
      const vertex this_vertex(convert_coordinates_to_GL(view_loc, *vertices_i));
      push_triangle(coll, first_vertex, prev_vertex, this_vertex, c);
      prev_vertex = this_vertex;
    }
  }
}


distance manhattan_distance_to_bounding_box(bounding_box b, vector3<distance> const& v) {
  const distance xdist = (v(X) < b.min(X)) ? (b.min(X) - v(X)) : (v(X) > b.max(X)) ? (v(X) - b.max(X)) : 0;
  const distance ydist = (v(Y) < b.min(Y)) ? (b.min(Y) - v(Y)) : (v(Y) > b.max(Y)) ? (v(Y) - b.max(Y)) : 0;
  const distance zdist = (v(Z) < b.min(Z)) ? (b.min(Z) - v(Z)) : (v(Z) > b.max(Z)) ? (v(Z) - b.max(Z)) : 0;
  return xdist + ydist + zdist;
}

tile_coordinate tile_manhattan_distance_to_bounding_box_rounding_down(bounding_box b, vector3<distance> const& v) {
  const distance xdist = (v(X) < b.min(X)) ? (b.min(X) - v(X)) : (v(X) > b.max(X)) ? (v(X) - b.max(X)) : 0;
  const distance ydist = (v(Y) < b.min(Y)) ? (b.min(Y) - v(Y)) : (v(Y) > b.max(Y)) ? (v(Y) - b.max(Y)) : 0;
  const distance zdist = (v(Z) < b.min(Z)) ? (b.min(Z) - v(Z)) : (v(Z) > b.max(Z)) ? (v(Z) - b.max(Z)) : 0;
  // Like (xdist / tile_width + ydist / tile_width + zdist / tile_height) but more precise:
  return (xdist + ydist + (zdist * tile_width / tile_height)) / tile_width;
}

// When you're on two tiles, can this be correct? Is it a problem if it isn't? I think it's alright.
// (Comparing to converting b to fine units and calling tile_manhattan_distance_to_bounding_box_rounding_down.)
tile_coordinate tile_manhattan_distance_to_tile_bounding_box(tile_bounding_box b, vector3<tile_coordinate> const& v) {
  const tile_coordinate xdist = (v(X) < b.min(X)) ? (b.min(X) - v(X)) : (v(X) > b.max(X)) ? (v(X) - b.max(X)) : 0;
  const tile_coordinate ydist = (v(Y) < b.min(Y)) ? (b.min(Y) - v(Y)) : (v(Y) > b.max(Y)) ? (v(Y) - b.max(Y)) : 0;
  const tile_coordinate zdist = (v(Z) < b.min(Z)) ? (b.min(Z) - v(Z)) : (v(Z) > b.max(Z)) ? (v(Z) - b.max(Z)) : 0;
  return xdist + ydist + zdist;
}


} // end anonymous namespace



view_on_the_world::view_on_the_world(vector3<distance> approx_initial_center)
: view_loc_for_local_display(approx_initial_center),
  view_type(ROBOT),
  local_view_direction(0),
  surveilled_by_global_display(approx_initial_center + vector3<distance>(5*tile_width, 5*tile_width, 5*tile_width)),
  global_view_dist(20*tile_width),
  drawing_regular_stuff(true),
  drawing_debug_stuff(true)
{}

void view_on_the_world::input(input_representation::input_news_t const& input_news) {
  using namespace input_representation;
  for(key_change_t const& c : input_news.key_activity_since_last_frame()) {
    if(c.second == PRESSED) {
      key_type const& k = c.first;
      if(k == "1") view_type = view_on_the_world::ROBOT;
      if(k == "2") view_type = view_on_the_world::LOCAL;
      if(k == "3") view_type = view_on_the_world::GLOBAL;
      if(k == "8") drawing_regular_stuff = !drawing_regular_stuff;
      if(k == "9") drawing_debug_stuff = !drawing_debug_stuff;
    }
  }
  if (view_type == LOCAL) {
    const bool fwdback = input_news.is_currently_pressed("5") || input_news.is_currently_pressed("s");
    const bool fwd = fwdback && !input_news.is_currently_pressed("shift");
    const bool back = fwdback && input_news.is_currently_pressed("shift");
    const bool left = input_news.is_currently_pressed("left") || input_news.is_currently_pressed("a");
    const bool right = input_news.is_currently_pressed("right") || input_news.is_currently_pressed("d");
    const bool up = input_news.is_currently_pressed("up") || input_news.is_currently_pressed("w");
    const bool down = input_news.is_currently_pressed("down") || input_news.is_currently_pressed("x");

    if (fwd) {
      view_loc_for_local_display += vector3<distance>(
        distance(numeric_representation_cast<double>(tile_width) * std::cos(local_view_direction)) / 10,
        distance(numeric_representation_cast<double>(tile_width) * std::sin(local_view_direction)) / 10,
        0
      );
    }
    if (back) {
      view_loc_for_local_display -= vector3<distance>(
        distance(numeric_representation_cast<double>(tile_width) * std::cos(local_view_direction)) / 10,
        distance(numeric_representation_cast<double>(tile_width) * std::sin(local_view_direction)) / 10,
        0
      );
    }
    if (left) { local_view_direction += 0.06; }
    if (right) { local_view_direction -= 0.06; }
    if (up) { view_loc_for_local_display.z += tile_width / 10; }
    if (down) { view_loc_for_local_display.z -= tile_width / 10; }
  }
  if (view_type == view_on_the_world::GLOBAL) {
    if (input_news.is_currently_pressed("q")) { surveilled_by_global_display.x += tile_width; }
    if (input_news.is_currently_pressed("a")) { surveilled_by_global_display.x -= tile_width; }
    if (input_news.is_currently_pressed("w")) { surveilled_by_global_display.y += tile_width; }
    if (input_news.is_currently_pressed("s")) { surveilled_by_global_display.y -= tile_width; }
    if (input_news.is_currently_pressed("e")) { surveilled_by_global_display.z += tile_width; }
    if (input_news.is_currently_pressed("d")) { surveilled_by_global_display.z -= tile_width; }
    if (input_news.is_currently_pressed("r")) { global_view_dist += tile_width; }
    if (input_news.is_currently_pressed("f")) { global_view_dist -= tile_width; }
  }
}

// Inlining into prepare_tile() is useful, so specify 'inline'.
inline color compute_tile_color(world const& w, tile_location const& loc) {
  vector3<tile_coordinate> const& coords = loc.coords();
  int illumination = 0;
  auto foo = w.tile_litnesses_.find(coords);
  if(foo != w.tile_litnesses_.end()) illumination = foo->second;
  illumination += 24;
  if (true || illumination > 128) illumination = 128;
  uint32_t r = 0;
  uint32_t g = 0;
  uint32_t b = 0;
  uint32_t a = 0;
  const uint32_t pattern = ((get_primitive<uint32_t>(coords.x) + get_primitive<uint32_t>(coords.y) + get_primitive<uint32_t>(coords.z)) % 3);
  uint32_t metal = 0;
  switch (loc.stuff_at().contents()) {
    //prepare_tile() doesn't need this case, so omit it:
    //case AIR: return color(0x00000000);
    case ROCK: {
      r = g = b = 0x33 * pattern + 0x55;
      a = 0xff;
      metal = initial_minerals(loc.coords()).metal / meters / meters / meters;
    } break;
    case RUBBLE: {
      r = 0xff * (2 + pattern) / 4; g = 0xbb * (2 + pattern) / 4; b = 0x55 * (2 + pattern) / 4; a = 0xcc;
      metal = w.get_minerals(loc.coords()).metal / meters / meters / meters;
    } break;
    case GROUPABLE_WATER: r = 0x00; g = 0x00; b = 0xff; a = 0x77; break;
    case UNGROUPABLE_WATER: r = 0x66; g = 0x66; b = 0xff; a = 0x77; break;
    default: assert(false);
  }
  g = g + metal;
  if (g > 0xff) g = 0xff;
  r = r * illumination / 128;
  g = g * illumination / 128;
  b = b * illumination / 128;
  if (illumination == 24) {
    r = 0;
  }
  return color((r << 24) + (g << 16) + (b << 8) + a);
}


// We only draw faces of the tile that (1) are not material-interior
// and (2) that face towards the viewer.
//
// (For opaque materials, the viewer could not have seen the other faces
// anyway, if we drew them correctly.)
//
// Condition 2 means that the up-to-three faces that we draw
// will not visually overlap, so it doesn't matter which order OpenGL
// draws them in
// (not even in the hardest case, in which they're translucent so the
// OpenGL depth buffer can't be used, and different colors).
void prepare_tile(world const& w, gl_collection& coll, tile_location const& loc, vector3<double> const& view_loc_double, vector3<tile_coordinate> view_tile_loc_rounded_down) {
  vector3<tile_coordinate> const& coords = loc.coords();
  tile const& t = loc.stuff_at();
  const tile_contents contents = t.contents();

  // If the viewer is exactly aligned with the edge of a tile, then this code
  // will interpret one of the two tile coordinates the viewer is aligned with
  // as the same coord as the viewer, and one not.  That's acceptable:
  // the worst that can happen is we draw (or don't draw) a face that
  // is exactly aligned with the viewer's line-of-sight, which can't
  // be seen anyway.
  const int x_close_side = (view_tile_loc_rounded_down.x < coords.x) ? 0 : 1;
  const int y_close_side = (view_tile_loc_rounded_down.y < coords.y) ? 0 : 1;
  const int z_close_side = (view_tile_loc_rounded_down.z < coords.z) ? 0 : 1;
  const bool is_same_x_coord_as_viewer = (view_tile_loc_rounded_down.x == coords.x);
  const bool is_same_y_coord_as_viewer = (view_tile_loc_rounded_down.y == coords.y);
  const bool is_same_z_coord_as_viewer = (view_tile_loc_rounded_down.z == coords.z);

  // Only output the faces that are not interior to a single kind of material.
  const std::array<tile, num_cardinal_directions> neighbor_tiles = loc.get_all_neighbor_tiles(CONTENTS_ONLY);
  // Also, for tiles aligned in a dimension with the viewer,
  // neither face in that dimension is facing towards the viewer.
  //->near
  bool draw_x_close_side;
  bool draw_y_close_side;
  bool draw_z_close_side;

  const vector3<double> tile_distance_min = cast_vector3_to_double(coords).multiply_piecewise_by(tile_size_double);
  const vector3<GLfloat> tile_gl_min = vector3<GLfloat>(tile_distance_min - view_loc_double);
  const vector3<GLfloat> tile_gl_max = tile_gl_min + tile_size_float;
  vector3<GLfloat> tile_gl_near;
  vector3<GLfloat> tile_gl_far;
  if(x_close_side == 0) {
    tile_gl_near.x = tile_gl_min.x;
    tile_gl_far.x = tile_gl_max.x;
    draw_x_close_side = (neighbor_tiles[xminus].contents() != contents
                                            && !is_same_x_coord_as_viewer);
  }
  else {
    tile_gl_far.x = tile_gl_min.x;
    tile_gl_near.x = tile_gl_max.x;
    draw_x_close_side = (neighbor_tiles[xplus].contents() != contents
                                            && !is_same_x_coord_as_viewer);
  }
  if(y_close_side == 0) {
    tile_gl_near.y = tile_gl_min.y;
    tile_gl_far.y = tile_gl_max.y;
    draw_y_close_side = (neighbor_tiles[yminus].contents() != contents
                                            && !is_same_y_coord_as_viewer);
  }
  else {
    tile_gl_far.y = tile_gl_min.y;
    tile_gl_near.y = tile_gl_max.y;
    draw_y_close_side = (neighbor_tiles[yplus].contents() != contents
                                            && !is_same_y_coord_as_viewer);
  }
  if(z_close_side == 0) {
    tile_gl_near.z = tile_gl_min.z;
    tile_gl_far.z = tile_gl_max.z;
    draw_z_close_side = (neighbor_tiles[zminus].contents() != contents
                                            && !is_same_z_coord_as_viewer);
  }
  else {
    tile_gl_far.z = tile_gl_min.z;
    tile_gl_near.z = tile_gl_max.z;
    draw_z_close_side = (neighbor_tiles[zplus].contents() != contents
                                            && !is_same_z_coord_as_viewer);
  }
  const gl_call_data::size_type original_count = coll.quads.count;
  coll.quads.reserve_new_slots(4 * (draw_x_close_side + draw_y_close_side + draw_z_close_side));
  vertex_with_color* base = coll.quads.vertices + original_count;

  const color tile_color = compute_tile_color(w, loc);

  const vertex_with_color gl_vertices[2][2][2] =
    { { { vertex_with_color(tile_gl_near.x, tile_gl_near.y, tile_gl_near.z, tile_color),
          vertex_with_color(tile_gl_near.x, tile_gl_near.y, tile_gl_far.z,  tile_color) },
        { vertex_with_color(tile_gl_near.x, tile_gl_far.y,  tile_gl_near.z, tile_color),
          vertex_with_color(tile_gl_near.x, tile_gl_far.y,  tile_gl_far.z,  tile_color) }, },
      { { vertex_with_color(tile_gl_far.x,  tile_gl_near.y, tile_gl_near.z, tile_color),
          vertex_with_color(tile_gl_far.x,  tile_gl_near.y, tile_gl_far.z,  tile_color) },
        { vertex_with_color(tile_gl_far.x,  tile_gl_far.y,  tile_gl_near.z, tile_color),
          vertex_with_color(tile_gl_far.x,  tile_gl_far.y,  tile_gl_far.z,  tile_color) } } };

  //TODO what if you are close enough to a wall or lake-surface that
  //this falls inside your near clipping plane?
  if (draw_x_close_side) {
    base[0] = gl_vertices[0][0][0];
    base[1] = gl_vertices[0][1][0];
    base[2] = gl_vertices[0][1][1];
    base[3] = gl_vertices[0][0][1];
    base += 4;
  }
  if (draw_y_close_side) {
    base[0] = gl_vertices[0][0][0];
    base[1] = gl_vertices[0][0][1];
    base[2] = gl_vertices[1][0][1];
    base[3] = gl_vertices[1][0][0];
    base += 4;
  }
  if (draw_z_close_side) {
    base[0] = gl_vertices[0][0][0];
    base[1] = gl_vertices[1][0][0];
    base[2] = gl_vertices[1][1][0];
    base[3] = gl_vertices[0][1][0];
    base += 4;
  }
}

// 0 seemed to be the best values speed wise: which means,
// of course, that it's as if we deleted the code that
// implements these numbers.
const num_bits_type tile_exponent_below_which_we_dont_filter = 0;
// Don't waste time checking against frustums for too-small benefit.
const num_bits_type tile_exponent_below_which_we_dont_filter_frustums = 0;

struct bbox_tile_prep_visitor {
  tribool look_here(power_of_two_bounding_cube<3, tile_coordinate> const& bbox) {
    const num_bits_type exp = bbox.size_exponent_in_each_dimension();
    if(exp < tile_exponent_below_which_we_dont_filter) return true;
    if(!overlaps(bounds_, bbox)) return false;
    power_of_two_bounding_cube<3, tile_coordinate_signed_type> mostly_centred_bbox(
      vector3<tile_coordinate>(bbox.min()) - view_tile_loc_rounded_down, exp);
    if((exp >= tile_exponent_below_which_we_dont_filter_frustums)
        && !overlaps(tile_view_frustum_, mostly_centred_bbox)) return false;
    if(subsumes(bounds_, bbox) &&
      ((exp < tile_exponent_below_which_we_dont_filter_frustums)
        || subsumes(tile_view_frustum_, mostly_centred_bbox))
    ) {
      return true;
    }
    return indeterminate;
    //maybe have "tribool within()" like overlaps()/subsumes()?
  }
  bool collidable_tile(tile_location const& loc) {
    gl_collection& coll = gl_collections_by_distance.at(
      get_primitive_int(tile_manhattan_distance_to_tile_bounding_box(loc.coords(), view_tile_loc_rounded_down))
    );
    prepare_tile(w, coll, loc, view_loc_double, view_tile_loc_rounded_down);

    if (view.drawing_debug_stuff && is_fluid(loc.stuff_at().contents())) {
      const vector3<GLfloat> locv = convert_tile_coordinates_to_GL(view_loc_double, loc.coords());
      if (tile_physics_impl::active_fluid_tile_info const* fluid =
            find_as_pointer(tile_physics_impl::get_state(w.tile_physics()).active_fluids, loc)) {
        const vector3<GLfloat> line_base = locv + tile_size_float / 2;
        push_line(coll,
                  line_base,
                  line_base + cast_vector3_to_float(fluid->velocity / tile_physics_sub_tile_velocity_units),
                  color(0x00ff0077));

        for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
          const sub_tile_distance prog = fluid->progress[dir];
          if (prog > 0) {
            const vector3<GLfloat> directed_prog(
              (vector3<double>(cardinal_direction_vectors[dir]) * numeric_representation_cast<double>(prog)) /
              numeric_representation_cast<double>(tile_physics_impl::progress_necessary(dir)));

            push_line(coll,
                        vertex(locv.x + 0.51, locv.y + 0.5, locv.z + 0.1),
                        vertex(
                          locv.x + 0.51 + directed_prog.x,
                          locv.y + 0.5 + directed_prog.y,
                          locv.z + 0.1 + directed_prog.z),
                        color(0x0000ff77));
          }
        }
      }
      else {
        push_point(coll, vertex(locv.x + 0.5, locv.y + 0.5, locv.z + 0.1), color(0x00000077));
      }
    }

    return true;
  }
  octant_number octant()const { return 7; }

  gl_collectionplex& gl_collections_by_distance;
  vector3<double> view_loc_double;
  vector3<tile_coordinate> view_tile_loc_rounded_down;
  tile_bounding_box bounds_;
  frustum tile_view_frustum_;
  view_on_the_world& view;
  world& w;
};

void prepare_shape(vector3<distance> view_loc, gl_collection& coll,
                   shape const& object_shape, color shape_color) {
  lasercake_vector<bounding_box>::type const& obj_bboxes = object_shape.get_boxes();
  for (bounding_box const& bbox : obj_bboxes) {
    const vector3<GLfloat> bmin = convert_coordinates_to_GL(view_loc, bbox.min());
    const vector3<GLfloat> bmax = convert_coordinates_to_GL(view_loc, bbox.max());
    push_quad(coll,
              vertex(bmin.x, bmin.y, bmin.z),
              vertex(bmax.x, bmin.y, bmin.z),
              vertex(bmax.x, bmax.y, bmin.z),
              vertex(bmin.x, bmax.y, bmin.z),
              shape_color);
    push_quad(coll,
              vertex(bmin.x, bmin.y, bmin.z),
              vertex(bmax.x, bmin.y, bmin.z),
              vertex(bmax.x, bmin.y, bmax.z),
              vertex(bmin.x, bmin.y, bmax.z),
              shape_color);
    push_quad(coll,
              vertex(bmin.x, bmin.y, bmin.z),
              vertex(bmin.x, bmin.y, bmax.z),
              vertex(bmin.x, bmax.y, bmax.z),
              vertex(bmin.x, bmax.y, bmin.z),
              shape_color);
    push_quad(coll,
              vertex(bmin.x, bmin.y, bmax.z),
              vertex(bmax.x, bmin.y, bmax.z),
              vertex(bmax.x, bmax.y, bmax.z),
              vertex(bmin.x, bmax.y, bmax.z),
              shape_color);
    push_quad(coll,
              vertex(bmin.x, bmax.y, bmin.z),
              vertex(bmax.x, bmax.y, bmin.z),
              vertex(bmax.x, bmax.y, bmax.z),
              vertex(bmin.x, bmax.y, bmax.z),
              shape_color);
    push_quad(coll,
              vertex(bmax.x, bmin.y, bmin.z),
              vertex(bmax.x, bmin.y, bmax.z),
              vertex(bmax.x, bmax.y, bmax.z),
              vertex(bmax.x, bmax.y, bmin.z),
              shape_color);
  }

  lasercake_vector<geom::convex_polygon>::type const& obj_polygons = object_shape.get_polygons();
  for (geom::convex_polygon const& polygon : obj_polygons) {
    push_convex_polygon(view_loc, coll, polygon.get_vertices(), shape_color);
#if 0
    // shared_ptr<mobile_object> objp = boost::dynamic_pointer_cast<mobile_object>(*(w.get_object(id)));
    // TODO so many redundant velocity vectors!!
    for(auto const& this_vertex : polygon.get_vertices()) {
      const vector3<GLfloat> locv = convert_coordinates_to_GL(view_loc, this_vertex);
      push_line(coll,
                locv,
                locv + cast_vector3_to_float(objp->velocity()),
                shape_color);
    }
#endif
  }
  lasercake_vector<geom::convex_polyhedron>::type const& obj_polyhedra = object_shape.get_polyhedra();
  for (geom::convex_polyhedron const& ph : obj_polyhedra) {
    for (uint8_t i = 0; i < ph.face_info().size(); i += ph.face_info()[i] + 1) {
      std::vector<geom::vect> poly;
      for (uint8_t j = 0; j < ph.face_info()[i]; ++j) {
        poly.push_back(ph.vertices()[ph.face_info()[i + j + 1]]);
      }
      push_convex_polygon(view_loc, coll, poly, shape_color);
    }
  }
}

void view_on_the_world::prepare_gl_data(
  world /*TODO const*/& w,
  gl_data_preparation_config config,
  abstract_gl_data& abstract_gl_data //result
) {
  gl_all_data& gl_data = abstract_gl_data.data();
  //for short
  gl_collectionplex& gl_collections_by_distance = gl_data.stuff_to_draw_as_gl_collections_by_distance;
  gl_collections_by_distance.clear();
  // Calculate a bit conservatively; no harm doing so:
  const size_t max_gl_collection = get_primitive<size_t>(
    2*(config.view_radius / tile_width) + (config.view_radius / tile_height)
    + 3*((1<<tile_exponent_below_which_we_dont_filter)-1)
    + 5);
  gl_collections_by_distance.resize(max_gl_collection);

  //These values are computed every GL-preparation-frame.
  vector3<distance> view_loc;
  vector3<distance> view_towards;

  if (view_type == ROBOT) {
    assert(config.view_from != NO_OBJECT);
    bounding_box b = w.get_object_personal_space_shapes().find(config.view_from)->second.bounds();
    view_loc = ((b.min() + b.max()) / 2);
    vector3<distance> facing = boost::dynamic_pointer_cast<object_with_eye_direction>(w.get_objects().find(config.view_from)->second)->get_facing();
    view_towards = view_loc + facing;
  }
  else if (view_type == LOCAL) {
    view_loc = view_loc_for_local_display;
    view_towards = view_loc + vector3<distance>(
      distance(100*numeric_representation_cast<double>(tile_width) * std::cos(local_view_direction)),
      distance(100*numeric_representation_cast<double>(tile_width) * std::sin(local_view_direction)),
      0
    );
  }
  else if (view_type == GLOBAL) {
    const double game_time_in_seconds =
      numeric_representation_cast<double>(w.game_time_elapsed())
        / identity(time_units / seconds) / seconds;
    view_towards = surveilled_by_global_display;
    const auto global_view_dist_double = numeric_representation_cast<double>(global_view_dist);
    view_loc = surveilled_by_global_display + vector3<distance>(
      distance(global_view_dist_double * std::cos(game_time_in_seconds * 3 / 4)),
      distance(global_view_dist_double * std::sin(game_time_in_seconds * 3 / 4)),
      distance(global_view_dist_double / 2 + global_view_dist_double / 4 * std::sin(game_time_in_seconds / 2))
    );
  }

  const vector3<double> view_loc_double(cast_vector3_to_double(view_loc / fine_distance_units));
  const vector3<tile_coordinate> view_tile_loc_rounded_down(get_min_containing_tile_coordinates(view_loc));

  const vector3<GLfloat> facing = convert_displacement_to_GL(view_towards - view_loc);
  const vector3<GLfloat> facing_up(0, 0, 1);
  gl_data.facing = facing;
  gl_data.facing_up = facing_up;
  frustum view_frustum_in_fine_distance_units = make_frustum_from_matrix(
    make_projection_matrix(float(pretend_aspect_ratio_value_when_culling))
    * make_view_matrix(facing, facing_up)
  );
  // be conservative: don't cull things too near the edge of the frustum;
  // fight rounding error!
  // The 1000 was empirically more useful than just 10 or 0.00001.
  // It probably should be a multiple of tile_width, though I'm not sure.
  for(glm::vec4& half_space : view_frustum_in_fine_distance_units.half_spaces) {
    half_space.w += 1000.00001;
  }
  // We use tile units comparisons when the view center has been rounded
  // to a tile; so adjust that.  TODO: are all the signs correct?
  frustum view_frustum_in_fine_distance_units_for_view_tile_loc
    = view_frustum_in_fine_distance_units;
  const vector3<float> view_tile_loc_rounding_error_in_fine_distance_units
    = cast_vector3_to_float(view_loc - lower_bound_in_fine_distance_units(view_tile_loc_rounded_down));
  for(glm::vec4& half_space : view_frustum_in_fine_distance_units_for_view_tile_loc.half_spaces) {
    half_space.w -= vector3<float>(half_space.x, half_space.y, half_space.z)
                      .dot<float>(view_tile_loc_rounding_error_in_fine_distance_units);
  }
  const frustum view_frustum_in_tile_count_units =
    convert_frustum_from_fine_distance_units_to_tile_count_units(view_frustum_in_fine_distance_units_for_view_tile_loc);
  {
    const heads_up_display_text everywhere_hud_text = {
      "Esc: quit; Tab: switch robot; "
      // F11 and [cmd|ctrl]-shift-F both work on all platforms, but
      //save space by showing the typical one for the platform
      #if defined(__APPLE__) || defined(__MACOSX__)
      "cmd-shift-F"
      #else
      "F11"
      #endif
      ": fullscreen; "
      "p: pause; g: single-step; "
      "1, 2, 3: overview, local, robot view; "
      "8: regular drawing; 9: debug drawing\n",
      color(0xffcc33cc),
      "Granger_ch8plus",
      24,
      36, 18
    };
    auto const robot_has_instructions =
      boost::dynamic_pointer_cast<object_with_player_instructions>(w.get_objects().find(config.view_from)->second);
    if (view_type == ROBOT && robot_has_instructions) {
      heads_up_display_text useful_hud_text = everywhere_hud_text;
      useful_hud_text.text = robot_has_instructions->player_instructions()
          + '\n' + useful_hud_text.text;
      gl_data.hud_text = useful_hud_text;
    }
    else if (view_type == GLOBAL) {
      heads_up_display_text useful_hud_text = everywhere_hud_text;
      useful_hud_text.text = "qawsedrf: navigate overview view\n\n"
          + useful_hud_text.text;
      gl_data.hud_text = useful_hud_text;
    }
    else if (view_type == LOCAL) {
      heads_up_display_text useful_hud_text = everywhere_hud_text;
      useful_hud_text.text = "s/S/5 left/right/up/down a/w/d/x: navigate local view\n\n"
          + useful_hud_text.text;
      gl_data.hud_text = useful_hud_text;
    }
    else {
      heads_up_display_text useful_hud_text = everywhere_hud_text;
      useful_hud_text.text = "(this robot can't be psychically controlled)\n\n"
          + useful_hud_text.text;
      gl_data.hud_text = useful_hud_text;
    }
  }

  const auto tiles_here = get_all_containing_tile_coordinates(view_loc);
  // Average the color. Take the max opacity, so that you can't see through rock ever.
  //
  // TODO I think the correct thing to do is, if, say, you are half in water, half air,
  // to make the part of the screen that's in water have blue added, and the air
  // part have air-color added (i.e., in this case, do nothing :-P).
  uint32_t total_r = 0;
  uint32_t total_g = 0;
  uint32_t total_b = 0;
  GLubyte max_a = 0;
  for(auto coords : tiles_here) {
    const tile_location here = w.make_tile_location(coords, CONTENTS_AND_LOCAL_CACHES_ONLY);
    const color color_here = (here.stuff_at().contents() == AIR ?
                                color(0x00000000) : compute_tile_color(w,here));
    total_r += color_here.r;
    total_g += color_here.g;
    total_b += color_here.b;
    max_a = std::max(max_a, color_here.a);
  }
  gl_data.tint_everything_with_this_color = color(
    GLubyte(total_r / tiles_here.size()),
    GLubyte(total_g / tiles_here.size()),
    GLubyte(total_b / tiles_here.size()),
    max_a);

  // Optimization:
  if(gl_data.tint_everything_with_this_color.a == 0xff) { return; }


  const distance view_dist = config.view_radius;
  const bounding_box fine_view_bounds = bounding_box::min_and_max(
      view_loc - vector3<distance>(view_dist,view_dist,view_dist),
      view_loc + vector3<distance>(view_dist,view_dist,view_dist)
    );
  const tile_bounding_box tile_view_bounds_pre = get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(fine_view_bounds);
  // This is perhaps suboptimal
  const tile_coordinate tshift = (tile_coordinate(1) << tile_exponent_below_which_we_dont_filter) - tile_coordinate(1);
  const vector3<tile_coordinate> tshift3(tshift, tshift, tshift);
  const tile_bounding_box tile_view_bounds = tile_bounding_box::min_and_size(
    tile_view_bounds_pre.min() - tshift3, tile_view_bounds_pre.size() + tshift3*2);
  
  //Uncomment this 'if' to see how inefficient the current (Dec 2012) implementation of
  //world::ensure_realization_of_space() is for large spaces (e.g. lasercake -v200).
  //if(w.game_time_elapsed() % (time_units_per_second/3) == 0)
  w.ensure_realization_of_space(tile_view_bounds, CONTENTS_AND_LOCAL_CACHES_ONLY);

  if (this->drawing_debug_stuff) {
    // Issue: this doesn't limit to nearby tile state; it iterates everything.
    for (auto const& p : tile_physics_impl::get_state(w.tile_physics()).persistent_water_groups) {
      tile_physics_impl::persistent_water_group_info const& g = p.second;

      for (auto const& suckable_tiles : g.suckable_tiles_by_height.as_map()) {
        for(tile_location const& suckable_tile : suckable_tiles.second) {
          const bounding_box tile_fine_bbox = fine_bounding_box_of_tile(suckable_tile.coords());
          if(manhattan_distance_to_bounding_box(tile_fine_bbox, view_loc) > config.view_radius) {
            continue;
          }
          vector3<GLfloat> locv = convert_tile_coordinates_to_GL(view_loc_double, suckable_tile.coords());
          gl_collection& coll = gl_collections_by_distance.at(
            get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(tile_fine_bbox, view_loc))
          );
          push_point(coll, vertex(locv.x + 0.5, locv.y + 0.5, locv.z + 0.15), color(0xff00ff77));
        }
      }
      for (auto const& pushable_tiles : g.pushable_tiles_by_height.as_map()) {
        for(tile_location const& pushable_tile : pushable_tiles.second) {
          const bounding_box tile_fine_bbox = fine_bounding_box_of_tile(pushable_tile.coords());
          if(manhattan_distance_to_bounding_box(tile_fine_bbox, view_loc) > config.view_radius) {
            continue;
          }
          vector3<GLfloat> locv = convert_tile_coordinates_to_GL(view_loc_double, pushable_tile.coords());
          gl_collection& coll = gl_collections_by_distance.at(
            get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(tile_fine_bbox, view_loc))
          );
          push_point(coll, vertex(locv.x + 0.5, locv.y + 0.5, locv.z + 0.15), color(0xff770077));
        }
      }
    }
  }

  for (auto p : w.get_laser_sfxes()) {
    const vector3<GLfloat> locvf1 = convert_coordinates_to_GL(view_loc, p.first);
    const vector3<GLfloat> locvf2 = convert_coordinates_to_GL(view_loc, p.first + p.second);
    const vector3<GLfloat> dlocvf = locvf2 - locvf1;
    const int length = 50; //(locvf2 - locvf1).magnitude_within_32_bits();
    const vector3<GLfloat> dlocvf_per_step = dlocvf / length;
    vector3<GLfloat> locvf = locvf1;

    for (int i = 0; i <= length; ++i) {
      const bounding_box segment_bbox = bounding_box::spanning_two_points(
        p.first + (p.second * i / length), p.first + (p.second * (i+1) / length));
      if(manhattan_distance_to_bounding_box(segment_bbox, view_loc) > config.view_radius) {
        continue;
      }
      gl_collection& coll = gl_collections_by_distance.at(
        get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(segment_bbox, view_loc))
      );
      const vector3<GLfloat> locvf_next = locvf1 + dlocvf_per_step * (i+1);
      const GLfloat apparent_laser_height = convert_distance_to_GL(tile_height) / 2;
      push_quad(coll,
                vertex(locvf.x, locvf.y, locvf.z),
                vertex(locvf.x, locvf.y, locvf.z + apparent_laser_height),
                vertex(locvf_next.x, locvf_next.y, locvf_next.z + apparent_laser_height),
                vertex(locvf_next.x, locvf_next.y, locvf_next.z),
                color(0x00ff0077));
      locvf = locvf_next;
    }
  }

  if (this->drawing_regular_stuff) {
    vector<object_identifier> objects_to_draw;
    w.objects_exposed_to_collision().get_objects_overlapping(objects_to_draw, fine_view_bounds);
    for (object_identifier const& id : objects_to_draw) {
      gl_collection& coll = gl_collections_by_distance.at(
        get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(w.get_bounding_box_of_object_or_tile(id), view_loc))
      );
      shape const* maybe_obj_shape = find_as_pointer(w.get_object_personal_space_shapes(), id);
      assert(maybe_obj_shape);
      shape const& obj_shape = *maybe_obj_shape;

      shared_ptr<object>* maybe_objp = w.get_object(id);
      assert(maybe_objp);
      shared_ptr<object> objp = *maybe_objp;
      if ((view_type != ROBOT) || (id != config.view_from)) {
        if(dynamic_pointer_cast<solar_panel>(objp)) {
          prepare_shape(view_loc, coll, obj_shape, color(0xffff00aa));
        }
        else if(dynamic_pointer_cast<robot>(objp)) {
          prepare_shape(view_loc, coll, obj_shape, color(0x00ffffaa));
        }
        else if(dynamic_pointer_cast<autorobot>(objp)) {
          prepare_shape(view_loc, coll, obj_shape, color(0x00ffffaa));
        }
        else if(dynamic_pointer_cast<laser_emitter>(objp)) {
          prepare_shape(view_loc, coll, obj_shape, color(0xff7755aa));
        }
        else if(shared_ptr<conveyor_belt> belt = dynamic_pointer_cast<conveyor_belt>(objp)) {
          prepare_shape(view_loc, coll, obj_shape, color(0xffffffaa));
          vector3<distance> foo = vector3<lint64_t>(cardinal_direction_vectors[belt->direction()]) * tile_width / 3;
          vector3<distance> bar(foo.y, foo.x, 0);
          vector3<distance> center = (obj_shape.bounds().min() + obj_shape.bounds().max()) / 2;
          vector3<distance> up(0, 0, tile_height / 5);
          push_quad(coll,
                    convert_coordinates_to_GL(view_loc, center - foo),
                    convert_coordinates_to_GL(view_loc, center - foo + up),
                    convert_coordinates_to_GL(view_loc, center + foo + up),
                    convert_coordinates_to_GL(view_loc, center + foo),
                    color(0xff0000aa));
          push_quad(coll,
                    convert_coordinates_to_GL(view_loc, center + foo),
                    convert_coordinates_to_GL(view_loc, center + foo + up),
                    convert_coordinates_to_GL(view_loc, center + bar + up),
                    convert_coordinates_to_GL(view_loc, center + bar),
                    color(0xff0000aa));
          push_quad(coll,
                    convert_coordinates_to_GL(view_loc, center + foo),
                    convert_coordinates_to_GL(view_loc, center + foo + up),
                    convert_coordinates_to_GL(view_loc, center - bar + up),
                    convert_coordinates_to_GL(view_loc, center - bar),
                    color(0xff0000aa));
        }
        else {
          // just in case.
          prepare_shape(view_loc, coll, obj_shape, color(0xffffffaa));
        }
      }
      else {
        if(shared_ptr<robot> rob = dynamic_pointer_cast<robot>(objp)) {
          prepare_shape(view_loc, coll, tile_shape(rob->get_building_tile(w, id)), color(0xffff0033));
        }
      }
    }
  }

  if (this->drawing_regular_stuff) {
    w.visit_collidable_tiles(bbox_tile_prep_visitor{
      gl_collections_by_distance, view_loc_double, view_tile_loc_rounded_down,
      tile_view_bounds, view_frustum_in_tile_count_units, *this, w
    });
  }
}




