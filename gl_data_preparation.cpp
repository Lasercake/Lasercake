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

#include "gl_data_preparation.hpp"
#include "world.hpp"

#include "specific_object_types.hpp"
#include "tile_physics.hpp" // to access internals for debugging-displaying...

using namespace gl_data_preparation;

namespace /* anonymous */ {

template<typename Float, typename Int>
vector3<Float> cast_vector3_to_floating(vector3<Int> v) {
  return vector3<Float>(get_primitive_double(v.x), get_primitive_double(v.y), get_primitive_double(v.z));
}

vector3<GLfloat> convert_coordinates_to_GL(vector3<fine_scalar> view_center, vector3<fine_scalar> input) {
  return cast_vector3_to_floating<GLfloat>(input - view_center) / get_primitive_double(tile_width);
}

vector3<GLfloat> convert_tile_coordinates_to_GL(vector3<double> view_center_double, vector3<tile_coordinate> input) {
  // (Floats don't have enough precision to represent tile_coordinates exactly,
  // which before subtraction they must do. Doubles do.)
  const vector3<double> input_as_tile_width_scale_double(
    get_primitive_double(input.x),
    get_primitive_double(input.y),
    get_primitive_double(input.z) * (get_primitive_double(tile_height) / get_primitive_double(tile_width)));
  vector3<GLfloat> result(input_as_tile_width_scale_double - view_center_double);
  return result;
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
void push_convex_polygon(vector3<fine_scalar> const& view_loc,
                         gl_collection& coll,
                         std::vector<vector3<polygon_int_type> > const& vertices,
                         color const& c) {
  if(vertices.size() >= 3) {
    // draw convex polygon via (sides - 2) triangles
    std::vector< vector3<polygon_int_type> >::const_iterator vertices_i = vertices.begin();
    const std::vector< vector3<polygon_int_type> >::const_iterator vertices_end = vertices.end();
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


tile_coordinate tile_manhattan_distance_to_bounding_box_rounding_down(bounding_box b, vector3<fine_scalar> const& v) {
  const fine_scalar xdist = (v(X) < b.min(X)) ? (b.min(X) - v(X)) : (v(X) > b.max(X)) ? (v(X) - b.max(X)) : 0;
  const fine_scalar ydist = (v(Y) < b.min(Y)) ? (b.min(Y) - v(Y)) : (v(Y) > b.max(Y)) ? (v(Y) - b.max(Y)) : 0;
  const fine_scalar zdist = (v(Z) < b.min(Z)) ? (b.min(Z) - v(Z)) : (v(Z) > b.max(Z)) ? (v(Z) - b.max(Z)) : 0;
  // Like (xdist / tile_width + ydist / tile_width + zdist / tile_height) but more precise:
  return (xdist + ydist + (zdist * tile_width / tile_height)) / tile_width;
}


} // end anonymous namespace



view_on_the_world::view_on_the_world(object_identifier robot_id, vector3<fine_scalar> approx_initial_center)
: robot_id(robot_id),
  view_loc_for_local_display(approx_initial_center),
  view_type(GLOBAL),
  view_direction(0),
  surveilled_by_global_display(approx_initial_center + vector3<fine_scalar>(5*tile_width, 5*tile_width, 5*tile_width)),
  globallocal_view_dist(20*tile_width),
  drawing_regular_stuff(true),
  drawing_debug_stuff(true)
{}

void view_on_the_world::input(input_representation::input_news_t const& input_news) {
  using namespace input_representation;
  for(key_change_t const& c : input_news.key_activity_since_last_frame()) {
    if(c.second == PRESSED) {
      key_type const& k = c.first;
      if(k == "z") drawing_regular_stuff = !drawing_regular_stuff;
      if(k == "t") drawing_debug_stuff = !drawing_debug_stuff;
      if(k == "q") surveilled_by_global_display.x += tile_width;
      if(k == "a") surveilled_by_global_display.x -= tile_width;
      if(k == "w") surveilled_by_global_display.y += tile_width;
      if(k == "s") surveilled_by_global_display.y -= tile_width;
      if(k == "e") surveilled_by_global_display.z += tile_width;
      if(k == "d") surveilled_by_global_display.z -= tile_width;
      if(k == "r") globallocal_view_dist += tile_width;
      if(k == "f") globallocal_view_dist -= tile_width;
      if(k == "l") view_type = view_on_the_world::LOCAL;
      if(k == "o") view_type = view_on_the_world::GLOBAL;
      if(k == "i") view_type = view_on_the_world::ROBOT;
    }
  }
  if (view_type == LOCAL) {
    if (input_news.is_currently_pressed("u")) {
      view_loc_for_local_display += vector3<fine_scalar>(
        fine_scalar(get_primitive_double(tile_width) * std::cos(view_direction)) / 10,
        fine_scalar(get_primitive_double(tile_width) * std::sin(view_direction)) / 10,
        0
      );
    }
    if (input_news.is_currently_pressed("j")) {
      view_loc_for_local_display -= vector3<fine_scalar>(
        fine_scalar(get_primitive_double(tile_width) * std::cos(view_direction)) / 10,
        fine_scalar(get_primitive_double(tile_width) * std::sin(view_direction)) / 10,
        0
      );
    }
    if (input_news.is_currently_pressed("h")) { view_direction += 0.06; }
    if (input_news.is_currently_pressed("k")) { view_direction -= 0.06; }
    if (input_news.is_currently_pressed("y")) { view_loc_for_local_display.z += tile_width / 10; }
    if (input_news.is_currently_pressed("n")) { view_loc_for_local_display.z -= tile_width / 10; }
  }
}

void prepare_tile(gl_collection& coll, tile_location const& loc, vector3<double> const& view_loc_double, vector3<tile_coordinate> view_tile_loc_rounded_down) {
  vector3<tile_coordinate> const& coords = loc.coords();
  tile const& t = loc.stuff_at();
  
  const color tile_color =
    t.contents() ==              ROCK ? color(((((get_primitive<uint32_t>(coords.x) + get_primitive<uint32_t>(coords.y) + get_primitive<uint32_t>(coords.z)) % 3)
                                                * 0x222222u + 0x333333u) << 8) + 0xffu) :
    t.contents() ==            RUBBLE ? color(0xffff0077) :
    t.contents() ==   GROUPABLE_WATER ? color(0x0000ff77) :
    t.contents() == UNGROUPABLE_WATER ? color(0x6666ff77) :
    (assert(false), (/*hack to make this compile*/0?color(0):throw 0xdeadbeef));

  // If we make one of the 'glb' members the closest corner of the tile to the player,
  // and the other the farthest, then we can draw the faces in a correct order
  // efficiently.  (Previously this code just used the lower bound for one corner
  // and the upper bound for the other corner.)
  //
  // It doesn't matter what part of the tile we compare against -- if the
  // viewer is aligned with the tile in a dimension, then which close corner
  // is picked won't change the order of any faces that are actually going to
  // overlap in the display.
  const int x_close_side = (view_tile_loc_rounded_down.x < coords.x) ? 0 : 1;
  const int y_close_side = (view_tile_loc_rounded_down.y < coords.y) ? 0 : 1;
  const int z_close_side = (view_tile_loc_rounded_down.z < coords.z) ? 0 : 1;
  const bool is_same_x_coord_as_viewer = (view_tile_loc_rounded_down.x == coords.x);
  const bool is_same_y_coord_as_viewer = (view_tile_loc_rounded_down.y == coords.y);
  const bool is_same_z_coord_as_viewer = (view_tile_loc_rounded_down.z == coords.z);
  // "is_same_loc_as_viewer" - there is at most one tile with this true!
  // So we *could* draw that tile specially once instead of doing this bit of
  // boolean work for every tile - but it's not much work.
  const bool is_same_loc_as_viewer = is_same_x_coord_as_viewer && is_same_y_coord_as_viewer && is_same_z_coord_as_viewer;

  const std::array<vector3<GLfloat>, 2> glb = {{
    convert_tile_coordinates_to_GL(view_loc_double, vector3<tile_coordinate>(
        coords.x +      x_close_side , coords.y +      y_close_side , coords.z +      z_close_side)),
    convert_tile_coordinates_to_GL(view_loc_double, vector3<tile_coordinate>(
        coords.x + int(!x_close_side), coords.y + int(!y_close_side), coords.z + int(!z_close_side)))
  }};

  const vertex_with_color gl_vertices[2][2][2] =
    { { { vertex_with_color(glb[0].x, glb[0].y, glb[0].z, tile_color),
          vertex_with_color(glb[0].x, glb[0].y, glb[1].z, tile_color) },
        { vertex_with_color(glb[0].x, glb[1].y, glb[0].z, tile_color),
          vertex_with_color(glb[0].x, glb[1].y, glb[1].z, tile_color) }, },
      { { vertex_with_color(glb[1].x, glb[0].y, glb[0].z, tile_color),
          vertex_with_color(glb[1].x, glb[0].y, glb[1].z, tile_color) },
        { vertex_with_color(glb[1].x, glb[1].y, glb[0].z, tile_color),
          vertex_with_color(glb[1].x, glb[1].y, glb[1].z, tile_color) } } };

  // Draw the farther faces first so that the closer faces will be drawn
  // after -- on top of -- the farther faces.  The closer faces are the ones
  // that have 0,0,0 as a vertex and the farther faces are the ones that have
  // 1,1,1 as a vertex.

  // Only output the faces that are not interior to a single kind of material.

  const tile_contents contents = t.contents();
  const std::array<bool, 2> x_neighbors_differ = {{
    loc.get_neighbor<xminus>(CONTENTS_ONLY).stuff_at().contents() != contents,
    loc.get_neighbor<xplus>(CONTENTS_ONLY).stuff_at().contents() != contents
  }};
  const std::array<bool, 2> y_neighbors_differ = {{
    loc.get_neighbor<yminus>(CONTENTS_ONLY).stuff_at().contents() != contents,
    loc.get_neighbor<yplus>(CONTENTS_ONLY).stuff_at().contents() != contents
  }};
  const std::array<bool, 2> z_neighbors_differ = {{
    loc.get_neighbor<zminus>(CONTENTS_ONLY).stuff_at().contents() != contents,
    loc.get_neighbor<zplus>(CONTENTS_ONLY).stuff_at().contents() != contents
  }};

  const bool draw_x_close_side = x_neighbors_differ[x_close_side]
    && (!is_same_x_coord_as_viewer || is_same_loc_as_viewer);
  const bool draw_y_close_side = y_neighbors_differ[y_close_side]
    && (!is_same_y_coord_as_viewer || is_same_loc_as_viewer);
  const bool draw_z_close_side = z_neighbors_differ[z_close_side]
    && (!is_same_z_coord_as_viewer || is_same_loc_as_viewer);

  const gl_call_data::size_type original_count = coll.quads.count;
  coll.quads.reserve_new_slots(4 * (draw_x_close_side + draw_y_close_side + draw_z_close_side));
  vertex_with_color* base = coll.quads.vertices + original_count;

  if (draw_z_close_side) {
    base[0] = gl_vertices[0][0][0];
    base[1] = gl_vertices[1][0][0];
    base[2] = gl_vertices[1][1][0];
    base[3] = gl_vertices[0][1][0];
    base += 4;
  }
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
}


void view_on_the_world::prepare_gl_data(
  world /*TODO const*/& w,
  gl_data_preparation_config config,
  gl_data_preparation::gl_all_data& gl_data //result
) {
    //for short
    gl_collectionplex& gl_collections_by_distance = gl_data.stuff_to_draw_as_gl_collections_by_distance;
    gl_collections_by_distance.clear();

    //These values are computed every GL-preparation-frame.
    vector3<fine_scalar> view_loc;
    vector3<fine_scalar> view_towards;

    if (view_type == LOCAL) {
      view_loc = view_loc_for_local_display;
      view_towards = view_loc + vector3<fine_scalar>(
        get_primitive_double(globallocal_view_dist) * std::cos(view_direction),
        get_primitive_double(globallocal_view_dist) * std::sin(view_direction),
        0
      );
    }
    else if (view_type == ROBOT) {
      bounding_box b = w.get_object_personal_space_shapes().find(robot_id)->second.bounds();
      view_loc = ((b.min() + b.max()) / 2);
      vector3<fine_scalar> facing = boost::dynamic_pointer_cast<robot>(w.get_objects().find(robot_id)->second)->get_facing();
      view_towards = view_loc + facing;
    }
    else {
      double game_time_in_seconds = get_primitive_double(w.game_time_elapsed()) / get_primitive_double(time_units_per_second);
      view_towards = surveilled_by_global_display;
      view_loc = surveilled_by_global_display + vector3<fine_scalar>(
        get_primitive_double(globallocal_view_dist) * std::cos(game_time_in_seconds * 3 / 4),
        get_primitive_double(globallocal_view_dist) * std::sin(game_time_in_seconds * 3 / 4),
        get_primitive_double(globallocal_view_dist / 2) + get_primitive_double(globallocal_view_dist / 4) * std::sin(game_time_in_seconds / 2)
      );
    }

    const vector3<double> view_loc_double(cast_vector3_to_floating<double>(view_loc) / get_primitive_double(tile_width));
    const vector3<tile_coordinate> view_tile_loc_rounded_down(get_min_containing_tile_coordinates(view_loc));

    vector<object_or_tile_identifier> tiles_to_draw;

    if (this->drawing_regular_stuff) {
      const fine_scalar view_dist = config.view_radius;
      w.collect_things_exposed_to_collision_intersecting(tiles_to_draw, bounding_box::min_and_max(
        view_loc - vector3<fine_scalar>(view_dist,view_dist,view_dist),
        view_loc + vector3<fine_scalar>(view_dist,view_dist,view_dist)
      ));
    }

    if (this->drawing_debug_stuff) {
      // Issue: this doesn't limit to nearby tile state; it iterates everything.
      for (auto const& p : tile_physics_impl::get_state(w.tile_physics()).persistent_water_groups) {
        tile_physics_impl::persistent_water_group_info const& g = p.second;

        for (auto const& foo : g.suckable_tiles_by_height.as_map()) {
          for(tile_location const& bar : foo.second) {
            vector3<GLfloat> locv = convert_tile_coordinates_to_GL(view_loc_double, bar.coords());
            gl_collection& coll = gl_collections_by_distance[
              get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(fine_bounding_box_of_tile(bar.coords()), view_loc))
            ];
            push_point(coll, vertex(locv.x + 0.5, locv.y + 0.5, locv.z + 0.15), color(0xff00ff77));
          }
        }
        for (auto const& foo : g.pushable_tiles_by_height.as_map()) {
          for(tile_location const& bar : foo.second) {
            vector3<GLfloat> locv = convert_tile_coordinates_to_GL(view_loc_double, bar.coords());
            gl_collection& coll = gl_collections_by_distance[
              get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(fine_bounding_box_of_tile(bar.coords()), view_loc))
            ];
            push_point(coll, vertex(locv.x + 0.5, locv.y + 0.5, locv.z + 0.15), color(0xff770077));
          }
        }
      }
    }

    for (auto p : w.get_laser_sfxes()) {
      const vector3<GLfloat> locvf1 = convert_coordinates_to_GL(view_loc, p.first);
      const vector3<GLfloat> locvf2 = convert_coordinates_to_GL(view_loc, p.first + p.second);
      const vector3<GLfloat> dlocvf = locvf2 - locvf1;
      GLfloat length = 50; //(locvf2 - locvf1).magnitude_within_32_bits();
      const vector3<GLfloat> dlocvf_per_step = dlocvf / length;
      vector3<GLfloat> locvf = locvf1;

      for (int i = 0; i <= length; ++i) {
        gl_collection& coll = gl_collections_by_distance[
          get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(
            bounding_box::spanning_two_points(p.first + (p.second * i / length), p.first + (p.second * (i+1) / length)),
            view_loc))
        ];
        const vector3<GLfloat> locvf_next = locvf1 + dlocvf_per_step * (i+1);
        push_quad(coll,
                  vertex(locvf.x, locvf.y, locvf.z),
                  vertex(locvf.x, locvf.y, locvf.z + 0.1),
                  vertex(locvf_next.x, locvf_next.y, locvf_next.z + 0.1),
                  vertex(locvf_next.x, locvf_next.y, locvf_next.z),
                  color(0x00ff0077));
        locvf = locvf_next;
      }
    }

    for (object_or_tile_identifier const& id : tiles_to_draw) {
      gl_collection& coll = gl_collections_by_distance[
        get_primitive_int(tile_manhattan_distance_to_bounding_box_rounding_down(w.get_bounding_box_of_object_or_tile(id), view_loc))
      ];
      if (object_identifier const* mid = id.get_object_identifier()) {
        shared_ptr<mobile_object> objp = boost::dynamic_pointer_cast<mobile_object>(*(w.get_object(*mid)));
        const object_shapes_t::const_iterator obj_shape = w.get_object_personal_space_shapes().find(*mid);

        const color objects_color(0xff00ffaa); //something bright&visible

        lasercake_vector<bounding_box>::type const& obj_bboxes = obj_shape->second.get_boxes();
        for (bounding_box const& bbox : obj_bboxes) {
          const vector3<GLfloat> bmin = convert_coordinates_to_GL(view_loc, bbox.min());
          const vector3<GLfloat> bmax = convert_coordinates_to_GL(view_loc, bbox.max());
          push_quad(coll,
                    vertex(bmin.x, bmin.y, bmin.z),
                    vertex(bmax.x, bmin.y, bmin.z),
                    vertex(bmax.x, bmax.y, bmin.z),
                    vertex(bmin.x, bmax.y, bmin.z),
                    objects_color);
          push_quad(coll,
                    vertex(bmin.x, bmin.y, bmin.z),
                    vertex(bmax.x, bmin.y, bmin.z),
                    vertex(bmax.x, bmin.y, bmax.z),
                    vertex(bmin.x, bmin.y, bmax.z),
                    objects_color);
          push_quad(coll,
                    vertex(bmin.x, bmin.y, bmin.z),
                    vertex(bmin.x, bmin.y, bmax.z),
                    vertex(bmin.x, bmax.y, bmax.z),
                    vertex(bmin.x, bmax.y, bmin.z),
                    objects_color);
          push_quad(coll,
                    vertex(bmin.x, bmin.y, bmax.z),
                    vertex(bmax.x, bmin.y, bmax.z),
                    vertex(bmax.x, bmax.y, bmax.z),
                    vertex(bmin.x, bmax.y, bmax.z),
                    objects_color);
          push_quad(coll,
                    vertex(bmin.x, bmax.y, bmin.z),
                    vertex(bmax.x, bmax.y, bmin.z),
                    vertex(bmax.x, bmax.y, bmax.z),
                    vertex(bmin.x, bmax.y, bmax.z),
                    objects_color);
          push_quad(coll,
                    vertex(bmax.x, bmin.y, bmin.z),
                    vertex(bmax.x, bmin.y, bmax.z),
                    vertex(bmax.x, bmax.y, bmax.z),
                    vertex(bmax.x, bmax.y, bmin.z),
                    objects_color);
        }

        lasercake_vector<convex_polygon>::type const& obj_polygons = obj_shape->second.get_polygons();
        for (convex_polygon const& polygon : obj_polygons) {
          push_convex_polygon(view_loc, coll, polygon.get_vertices(), color(0x77777777));

          // TODO so many redundant velocity vectors!!
          for(auto const& this_vertex : polygon.get_vertices()) {
            const vector3<GLfloat> locv = convert_coordinates_to_GL(view_loc, this_vertex);
            push_line(coll,
                      vertex(locv.x, locv.y, locv.z),
                      vertex(
                        locv.x + (get_primitive_double(objp->velocity().x) / get_primitive_double(tile_width)),
                        locv.y + (get_primitive_double(objp->velocity().y) / get_primitive_double(tile_width)),
                        locv.z + (get_primitive_double(objp->velocity().z) / get_primitive_double(tile_width))),
                      objects_color);
          }
        }
      }
      if (tile_location const* locp = id.get_tile_location()) {
        tile_location const& loc = *locp;
        tile const& t = loc.stuff_at();

        prepare_tile(coll, loc, view_loc_double, view_tile_loc_rounded_down);

        vector3<GLfloat> locv = convert_tile_coordinates_to_GL(view_loc_double, loc.coords());

        if (this->drawing_debug_stuff && is_fluid(t.contents())) {
          if (tile_physics_impl::active_fluid_tile_info const* fluid =
                find_as_pointer(tile_physics_impl::get_state(w.tile_physics()).active_fluids, loc)) {
            push_line(coll,
                      vertex(locv.x+0.5, locv.y+0.5, locv.z + 0.1),
                      vertex(
                        locv.x + 0.5 + (get_primitive_double(fluid->velocity.x) / get_primitive_double(tile_width)),
                        locv.y + 0.5 + (get_primitive_double(fluid->velocity.y) / get_primitive_double(tile_width)),
                        locv.z + 0.1 + (get_primitive_double(fluid->velocity.z) / get_primitive_double(tile_width))),
                      color(0x00ff0077));

            for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
              const sub_tile_distance prog = fluid->progress[dir];
              if (prog > 0) {
                vector3<GLfloat> directed_prog =
                  (vector3<GLfloat>(cardinal_direction_vectors[dir]) * get_primitive_double(prog)) /
                  get_primitive_double(tile_physics_impl::progress_necessary(dir));

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
      }
    }

    gl_data.facing = cast_vector3_to_floating<GLfloat>(view_towards - view_loc) / get_primitive_double(tile_width);
    gl_data.facing_up = vector3<GLfloat>(0, 0, 1);

}




