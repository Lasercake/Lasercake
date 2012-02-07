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

#include "rendering_the_world.hpp"
#include "world.hpp"

#include "specific_object_types.hpp"
#include "tile_physics.hpp" // to access internals for debugging-displaying...

#include "GL/gl.h" //TODO remove from here?
#include "GL/glu.h" //TODO remove from here?
#include "SDL.h" //TODO remove from here!

using namespace world_rendering;

namespace /* anonymous */ {

vector3<GLfloat> convert_coordinates_to_GL(vector3<fine_scalar> view_center, vector3<fine_scalar> input) {
  return vector3<GLfloat>(input - view_center) / tile_width;
}



void push_vertex(gl_call_data& data, vertex const& v, color const& c) {
  data.vertices.push_back(v);
  data.colors.push_back(c);
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
                         std::vector<vector3<int64_t> > const& vertices,
                         color const& c) {
  if(vertices.size() >= 3) {
    // draw convex polygon via (sides - 2) triangles
    std::vector< vector3<int64_t> >::const_iterator vertices_i = vertices.begin();
    const std::vector< vector3<int64_t> >::const_iterator vertices_end = vertices.end();
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
  const fine_scalar xdist = (v.x < b.min.x) ? (b.min.x - v.x) : (v.x > b.max.x) ? (v.x - b.max.x) : 0;
  const fine_scalar ydist = (v.y < b.min.y) ? (b.min.y - v.y) : (v.y > b.max.y) ? (v.y - b.max.y) : 0;
  const fine_scalar zdist = (v.z < b.min.z) ? (b.min.z - v.z) : (v.z > b.max.z) ? (v.z - b.max.z) : 0;
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
  globallocal_view_dist(20*tile_width)
{}


void view_on_the_world::render(
  world /*TODO const*/& w,
  world_rendering_config rendering_config,
  world_rendering::gl_all_data& gl_data //result
) {
    //for short
    gl_collectionplex& gl_collections_by_distance = gl_data.stuff_to_draw_as_gl_collections_by_distance;
    gl_collections_by_distance.clear();

    //These values are computed every rendering-frame.
    vector3<fine_scalar> view_loc;
    vector3<fine_scalar> view_towards;
    
    if (view_type == LOCAL) {
      view_loc = view_loc_for_local_display;
      view_towards = view_loc + vector3<fine_scalar>(
        globallocal_view_dist * std::cos(view_direction),
        globallocal_view_dist * std::sin(view_direction),
        0
      );
      if (rendering_config.keystate[SDLK_u]) {
        view_loc_for_local_display += vector3<fine_scalar>(
        fine_scalar(double(tile_width) * std::cos(view_direction)) / 10,
        fine_scalar(double(tile_width) * std::sin(view_direction)) / 10,
        0);
      }
      if (rendering_config.keystate[SDLK_j]) {
        view_loc_for_local_display -= vector3<fine_scalar>(
          fine_scalar(double(tile_width) * std::cos(view_direction)) / 10,
          fine_scalar(double(tile_width) * std::sin(view_direction)) / 10,
        0);
      }
      if (rendering_config.keystate[SDLK_h]) { view_direction += 0.06; }
      if (rendering_config.keystate[SDLK_k]) { view_direction -= 0.06; }
      if (rendering_config.keystate[SDLK_y]) { view_loc_for_local_display.z += tile_width / 10; }
      if (rendering_config.keystate[SDLK_n]) { view_loc_for_local_display.z -= tile_width / 10; }
    }
    else if (view_type == ROBOT) {
      bounding_box b = w.get_object_personal_space_shapes().find(robot_id)->second.bounds();
      view_loc = ((b.min + b.max) / 2);
      vector3<fine_scalar> facing = boost::dynamic_pointer_cast<robot>(w.get_objects().find(robot_id)->second)->get_facing();
      view_towards = view_loc + facing;
    }
    else {
      double game_time_in_seconds = double(w.game_time_elapsed()) / time_units_per_second;
      view_towards = surveilled_by_global_display;
      view_loc = surveilled_by_global_display + vector3<fine_scalar>(
        globallocal_view_dist * std::cos(game_time_in_seconds * 3 / 4),
        globallocal_view_dist * std::sin(game_time_in_seconds * 3 / 4),
        (globallocal_view_dist / 2) + (globallocal_view_dist / 4) * std::sin(game_time_in_seconds / 2)
      );
    }

    unordered_set<object_or_tile_identifier> tiles_to_draw;
    /*w.collect_things_exposed_to_collision_intersecting(tiles_to_draw, tile_bounding_box(
      vector3<tile_coordinate>(world_center_coord + view_x - 50, world_center_coord + view_y - 50, world_center_coord + view_z - 50),
      vector3<tile_coordinate>(101,101,101)
    ));*/
    // this is a bloody stupid hack, TODO do something different
    if (rendering_config.drawing_regular_stuff) {
    w.collect_things_exposed_to_collision_intersecting(tiles_to_draw, bounding_box(
      view_loc - vector3<fine_scalar>(tile_width*50,tile_width*50,tile_width*50),
      view_loc + vector3<fine_scalar>(tile_width*50,tile_width*50,tile_width*50)
    ));
    }

    // this is a bloody stupid hack, TODO do something different
    if (rendering_config.drawing_debug_stuff) {
    for (auto const& p : tile_physics_impl::get_state(w.tile_physics()).persistent_water_groups) {
      tile_physics_impl::persistent_water_group_info const& g = p.second;

      for (auto const& foo : g.suckable_tiles_by_height.as_map()) {
        for(tile_location const& bar : foo.second.as_unordered_set()) {
          vector3<GLfloat> locv = convert_coordinates_to_GL(view_loc, lower_bound_in_fine_units(bar.coords()));
          gl_collection& coll = gl_collections_by_distance[
            tile_manhattan_distance_to_bounding_box_rounding_down(fine_bounding_box_of_tile(bar.coords()), view_loc)
          ];
          push_point(coll, vertex(locv.x + 0.5, locv.y + 0.5, locv.z + 0.15), color(0xff00ff77));
        }
      }
      for (auto const& foo : g.pushable_tiles_by_height.as_map()) {
        for(tile_location const& bar : foo.second.as_unordered_set()) {
          vector3<GLfloat> locv = convert_coordinates_to_GL(view_loc, lower_bound_in_fine_units(bar.coords()));
          gl_collection& coll = gl_collections_by_distance[
            tile_manhattan_distance_to_bounding_box_rounding_down(fine_bounding_box_of_tile(bar.coords()), view_loc)
          ];
          push_point(coll, vertex(locv.x + 0.5, locv.y + 0.5, locv.z + 0.15), color(0xff770077));
        }
      }
    }
    }

    for (auto p : w.laser_sfxes) {
      const vector3<GLfloat> locvf1 = convert_coordinates_to_GL(view_loc, p.first);
      const vector3<GLfloat> locvf2 = convert_coordinates_to_GL(view_loc, p.first + p.second);
      const vector3<GLfloat> dlocvf = locvf2 - locvf1;
      GLfloat length = 50; //(locvf2 - locvf1).magnitude_within_32_bits();
      const vector3<GLfloat> dlocvf_per_step = dlocvf / length;
      vector3<GLfloat> locvf = locvf1;

      for (int i = 0; i <= length; ++i) {
        gl_collection& coll = gl_collections_by_distance[
          tile_manhattan_distance_to_bounding_box_rounding_down(
            bounding_box(p.first + (p.second * i / length), p.first + (p.second * (i+1) / length)),
            view_loc)
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
        tile_manhattan_distance_to_bounding_box_rounding_down(w.get_bounding_box_of_object_or_tile(id), view_loc)
      ];
      if (object_identifier const* mid = id.get_object_identifier()) {
        shared_ptr<mobile_object> objp = boost::dynamic_pointer_cast<mobile_object>(*(w.get_object(*mid)));
        const object_shapes_t::const_iterator blah = w.get_object_personal_space_shapes().find(*mid);
        std::vector<convex_polygon> const& foo = blah->second.get_polygons();
        for (convex_polygon const& bar : foo) {
          push_convex_polygon(view_loc, coll, bar.get_vertices(), color(0x77777777));

          // TODO so many redundant velocity vectors!!
          for(auto const& this_vertex : bar.get_vertices()) {
            const vector3<GLfloat> locv = convert_coordinates_to_GL(view_loc, this_vertex);
            push_line(coll,
                      vertex(locv.x, locv.y, locv.z),
                      vertex(
                        locv.x + ((GLfloat)objp->velocity.x / (tile_width)),
                        locv.y + ((GLfloat)objp->velocity.y / (tile_width)),
                        locv.z + ((GLfloat)objp->velocity.z / (tile_width))),
                      color(0x00ff0077));
          }
        }
      }
      if (tile_location const* locp = id.get_tile_location()) {
        tile_location const& loc = *locp;
        vector3<tile_coordinate> const& coords = loc.coords();
        tile const& t = loc.stuff_at();
        vector3<GLfloat> locv = convert_coordinates_to_GL(view_loc, lower_bound_in_fine_units(loc.coords()));

        {
          const color tile_color =
            t.contents() ==              ROCK ? color(((((coords.x + coords.y + coords.z) % 3)
                                                        * 0x222222u + 0x333333u) << 8) + 0xff) :
            t.contents() ==            RUBBLE ? color(0xffff0077) :
            t.contents() ==   GROUPABLE_WATER ? color(0x0000ff77) :
            t.contents() == UNGROUPABLE_WATER ? color(0x6666ff77) :
            (assert(false), (/*hack to make this compile*/0?color(0):throw 0xdeadbeef));

          // If we make one of the 'glb' members the closest corner of the tile to the player,
          // and the other the farthest, then we can draw the faces in a correct order
          // efficiently.  (Previously this code just used the lower bound for one corner
          // and the upper bound for the other corner.)
          const std::array<vector3<fine_scalar>, 2> fine = {{
            lower_bound_in_fine_units(loc.coords()),
            upper_bound_in_fine_units(loc.coords())
          }};
          // It doesn't matter what part of the tile we compare against -- if the
          // viewer is aligned with the tile in a dimension, then which close corner
          // is picked won't change the order of any faces that are actually going to
          // overlap in the display.
          // Actually, TODO, if you're aligned in one or two of the dimensions,
          // how will this make the closest tile to you be drawn first and the farthest
          // drawn last?  This code can't do that currently. Hmm.
          const int x_close_idx = (view_loc.x < fine[0].x) ? 0 : 1;
          const int y_close_idx = (view_loc.y < fine[0].y) ? 0 : 1;
          const int z_close_idx = (view_loc.z < fine[0].z) ? 0 : 1;

          const std::array<vector3<GLfloat>, 2> glb = {{
            convert_coordinates_to_GL(view_loc, vector3<fine_scalar>(
                fine[x_close_idx].x, fine[y_close_idx].y, fine[z_close_idx].z)),
            convert_coordinates_to_GL(view_loc, vector3<fine_scalar>(
                fine[!x_close_idx].x, fine[!y_close_idx].y, fine[!z_close_idx].z))
          }};

          // Draw the farther faces first so that the closer faces will be drawn
          // after -- on top of -- the farther faces.  The closer faces are the ones
          // that have 0,0,0 as a vertex and the farther faces are the ones that have
          // 1,1,1 as a vertex.

          // Only output the faces that are not interior to a single kind of material.
          if(((z_close_idx == 0) ? loc.get_neighbor<zplus>(CONTENTS_ONLY) : loc.get_neighbor<zminus>(CONTENTS_ONLY))
                    .stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[1].y, glb[1].z),
                      vertex(glb[0].x, glb[1].y, glb[1].z),
                      tile_color);
          }
          if(((x_close_idx == 0) ? loc.get_neighbor<xplus>(CONTENTS_ONLY) : loc.get_neighbor<xminus>(CONTENTS_ONLY))
                    .stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[1].x, glb[0].y, glb[0].z),
                      vertex(glb[1].x, glb[1].y, glb[0].z),
                      vertex(glb[1].x, glb[1].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[1].z),
                      tile_color);
          }
          if(((y_close_idx == 0) ? loc.get_neighbor<yplus>(CONTENTS_ONLY) : loc.get_neighbor<yminus>(CONTENTS_ONLY))
                    .stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[1].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[1].z),
                      vertex(glb[1].x, glb[1].y, glb[1].z),
                      vertex(glb[1].x, glb[1].y, glb[0].z),
                      tile_color);
          }
          if(((z_close_idx == 0) ? loc.get_neighbor<zminus>(CONTENTS_ONLY) : loc.get_neighbor<zplus>(CONTENTS_ONLY))
                    .stuff_at().contents() != t.contents()) {
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[0].z),
                      vertex(glb[1].x, glb[0].y, glb[0].z),
                      vertex(glb[1].x, glb[1].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[0].z),
                      tile_color);
          }
          if(((x_close_idx == 0) ? loc.get_neighbor<xminus>(CONTENTS_ONLY) : loc.get_neighbor<xplus>(CONTENTS_ONLY))
                    .stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[1].z),
                      vertex(glb[0].x, glb[0].y, glb[1].z),
                      tile_color);
          }
          if(((y_close_idx == 0) ? loc.get_neighbor<yminus>(CONTENTS_ONLY) : loc.get_neighbor<yplus>(CONTENTS_ONLY))
                    .stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[0].z),
                      vertex(glb[0].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[0].z),
                      tile_color);
          }
        }

        if (is_fluid(t.contents()) && rendering_config.drawing_debug_stuff) {
          if (tile_physics_impl::active_fluid_tile_info const* fluid =
                find_as_pointer(tile_physics_impl::get_state(w.tile_physics()).active_fluids, loc)) {
            push_line(coll,
                      vertex(locv.x+0.5, locv.y+0.5, locv.z + 0.1),
                      vertex(
                        locv.x + 0.5 + ((GLfloat)fluid->velocity.x / (tile_width)),
                        locv.y + 0.5 + ((GLfloat)fluid->velocity.y / (tile_width)),
                        locv.z + 0.1 + ((GLfloat)fluid->velocity.z / (tile_width))),
                      color(0x00ff0077));

            for (cardinal_direction dir = 0; dir < num_cardinal_directions; ++dir) {
              const sub_tile_distance prog = fluid->progress[dir];
              if (prog > 0) {
                vector3<GLfloat> directed_prog =
                  (vector3<GLfloat>(cardinal_direction_vectors[dir]) * prog) /
                  tile_physics_impl::progress_necessary(dir);

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

    gl_data.facing = vector3<GLfloat>(view_towards - view_loc) / tile_width;
    gl_data.facing_up = vector3<GLfloat>(0, 0, 1);

}



