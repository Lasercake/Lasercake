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

#include <sys/resource.h>
#include <time.h>
   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "SDL.h"
#include "GL/gl.h"
#include "GL/glu.h"

#include <iostream>

#include "world.hpp"

namespace /* anonymous */ {

typedef int64_t microseconds_t;
/* didn't have enough precision, at least on this Linux (3.3333 millisecond resolution)
 * microseconds_t rusage_to_microseconds(struct rusage const& r) {
  return (microseconds_t)r.ru_utime.tv_sec * 1000000 + (microseconds_t)r.ru_utime.tv_usec
       + (microseconds_t)r.ru_stime.tv_sec * 1000000 + (microseconds_t)r.ru_stime.tv_usec;
}*/
// TODO these functions are probably not portable but are useful to help
// avoid introducing performance regressions.
struct rusage get_this_process_rusage() {
  struct rusage ret;
  getrusage(RUSAGE_SELF, &ret);
  return ret;
}
microseconds_t get_this_process_microseconds() {
  struct timespec ts;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  return (microseconds_t)ts.tv_sec * 1000000 + (microseconds_t)ts.tv_nsec / 1000;
}
microseconds_t get_monotonic_microseconds() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (microseconds_t)ts.tv_sec * 1000000 + (microseconds_t)ts.tv_nsec / 1000;
}
void debug_print_microseconds(microseconds_t us) {
  std::cerr << (us / 1000) << '.' << (us / 100 % 10);
}

const int max_simple_hill_width = 20;

}
namespace std {
  template<> struct hash<pair<tile_coordinate, tile_coordinate>> {
    inline size_t operator()(pair<tile_coordinate, tile_coordinate> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.first);
      boost::hash_combine(seed, v.second);
      return seed;
    }
  };
}
namespace {

int get_hill(unordered_map<std::pair<tile_coordinate, tile_coordinate>, int> &hills_map, pair<tile_coordinate, tile_coordinate> loc) {
  auto iter = hills_map.find(loc);
  if (iter == hills_map.end()) {
    int hill = (rand()&255) ? 0 : (1 + (rand()%max_simple_hill_width));
    hills_map.insert(make_pair(loc, hill));
    return hill;
  }
  else return iter->second;
}
tile_coordinate get_height(unordered_map<std::pair<tile_coordinate, tile_coordinate>, tile_coordinate> &height_map, unordered_map<std::pair<tile_coordinate, tile_coordinate>, int> &hills_map, pair<tile_coordinate, tile_coordinate> loc) {
  auto iter = height_map.find(loc);
  if (iter == height_map.end()) {
    tile_coordinate height = world_center_coord - 100;
    const tile_coordinate x = loc.first;
    const tile_coordinate y = loc.second;
    for (tile_coordinate x2 = x - max_simple_hill_width; x2 <= x + max_simple_hill_width; ++x2) {
      for (tile_coordinate y2 = y - max_simple_hill_width; y2 <= y + max_simple_hill_width; ++y2) {
        height += std::max(0, get_hill(hills_map, make_pair(x2, y2)) - (int)i64sqrt((x2-x)*(x2-x) + (y2-y)*(y2-y)));
      }
    }
    height_map.insert(make_pair(loc, height));
    return height;
  }
  else return iter->second;
}

struct world_building_func {
  world_building_func(std::string scenario):scenario(scenario){}
  std::string scenario;
  unordered_map<std::pair<tile_coordinate, tile_coordinate>, int> hills_map;
  unordered_map<std::pair<tile_coordinate, tile_coordinate>, tile_coordinate> height_map;
  
  void operator()(world_building_gun make, tile_bounding_box bounds) {
    const tile_coordinate wc = world_center_coord;
    if (scenario == "simple_hills") {
      for (tile_coordinate x = bounds.min.x; x < bounds.min.x + bounds.size.x; ++x) {
        for (tile_coordinate y = bounds.min.y; y < bounds.min.y + bounds.size.y; ++y) {
          tile_coordinate height = get_height(height_map, hills_map, make_pair(x, y));
          for (tile_coordinate z = bounds.min.z; z < std::min(height, bounds.min.z + bounds.size.z); ++z) {
            vector3<tile_coordinate> l(x,y,z);
            make(ROCK, l);
          }
        }
      }
      return;
    }
    if (scenario.substr(0,15) == "pressure_tunnel") {
      for(vector3<tile_coordinate> l : bounds) {
        const tile_coordinate tower_lower_coord = wc;
        const tile_coordinate tower_upper_coord = wc+10;
        const tile_coordinate tower_height = 200;
        if (l.x < tower_lower_coord && l.y >= tower_lower_coord && l.y <= tower_lower_coord && l.z >= wc && l.z <= wc) {}
        else if (l.x < tower_lower_coord && l.y >= tower_lower_coord-1 && l.y <= tower_lower_coord+1 && l.z >= wc-1 && l.z <= wc+1)
          make(ROCK, l);
        else if (l.x >= tower_lower_coord && l.x < tower_upper_coord && l.y >= tower_lower_coord && l.y < tower_upper_coord && l.z >= wc && l.z < wc + tower_height)
          make(GROUPABLE_WATER, l);
        else if (l.x >= tower_lower_coord-1 && l.x < tower_upper_coord+1 && l.y >= tower_lower_coord-1 && l.y < tower_upper_coord+1 && l.z >= wc-1 && l.z < wc + tower_height+1)
          make(ROCK, l);
      }
      return;
    }
    for (tile_coordinate x = std::max(world_center_coord-1, bounds.min.x); x < std::min(world_center_coord+21, bounds.min.x + bounds.size.x); ++x) {
      for (tile_coordinate y = std::max(world_center_coord-1, bounds.min.y); y < std::min(world_center_coord+21, bounds.min.y + bounds.size.y); ++y) {
        for (tile_coordinate z = std::max(world_center_coord-1, bounds.min.z); z < std::min(world_center_coord+21, bounds.min.z + bounds.size.z); ++z) {
          vector3<tile_coordinate> l(x,y,z);
          if (x == world_center_coord-1 || x == world_center_coord+20 || y == world_center_coord-1 || y == world_center_coord+20 || z == world_center_coord-1 /*|| z == world_center_coord+20*/) {
            make(ROCK, l);
          }
          else {
            if ((scenario.substr(0,5) == "tower") &&
                  x >= wc+4 && x <= wc+6 &&
                  y >= wc+4 && y <= wc+6 &&
                  z >= wc+1) make(GROUPABLE_WATER, l);
            else if ((scenario == "tower2" || scenario == "tower3") &&
                  x >= wc+3 && x <= wc+7 &&
                  y >= wc+3 && y <= wc+7 &&
                  z >= wc+1) make(ROCK, l);
            else if (scenario == "tower3" && z < wc+3 && (
                  x == wc+1 || x == wc+9 || y == wc+1 || wc+y == 9)) make(ROCK, l);
                  
            if (scenario == "shallow") {
                   if (z == wc+0 && x >= wc+5) make(ROCK, l);
              else if (z == wc+1 && x >= wc+10) make(ROCK, l);
              else if (z == wc+2 && x >= wc+15) make(ROCK, l);
              else if (x == wc+19) make(GROUPABLE_WATER, l);
            }
            if (scenario == "steep") {
              if (z < 20 - x) make(ROCK, l);
              else if (z >= wc+15 && (wc + 20 - x) >= 15) make(GROUPABLE_WATER, l);
            }
            if (scenario == "tank") {
              if (z < wc+8 && x > wc+10) make(ROCK, l);
              else if (x >= wc+12 && y <= wc+13 && y >= wc+7) make(GROUPABLE_WATER, l);
              else if (y != wc+10 && x > wc+10) make(ROCK, l);
              else if (z > wc+8 && x > wc+10 && x < wc+18) make(ROCK, l);
              else if (x > wc+10) make(GROUPABLE_WATER, l);
            }
            if (scenario == "tank2") {
              if (z < wc+8 && x > wc+10) make(ROCK, l);
              else if (x >= wc+12 && y <= wc+13 && y >= wc+7) make(GROUPABLE_WATER, l);
              else if (y != wc+9 && y != wc+10 && x > wc+10) make(ROCK, l);
              else if (z > wc+9 && x > wc+10 && x < wc+18) make(ROCK, l);
              else if (x > wc+10) make(GROUPABLE_WATER, l);
            }
            if (scenario.substr(0,6) == "twisty") {
              if (x == wc+0) make(GROUPABLE_WATER, l);
              else if (x == wc+1 && z > wc+0) make(ROCK, l);
              else if (x == wc+5) make(scenario == "twistyrubble" ? RUBBLE : ROCK, l);
              else if (x == wc+2 && (z % 4) == 1) make(ROCK, l);
              else if (x == wc+3 && (z % 2) == 1) make(ROCK, l);
              else if (x == wc+4 && (z % 4) == 3) make(ROCK, l);
            }
          }
        }
      }
    }
  }
};

// These are to be passed as part of arrays to OpenGL.
// Thus their data format/layout makes a difference.
struct vertex {
  vertex(GLfloat x, GLfloat y, GLfloat z) : x(x), y(y), z(z) {}
  /* implicit conversion */ vertex(vector3<GLfloat> const& v) : x(v.x), y(v.y), z(v.z) {}
  
  GLfloat x, y, z;
};
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
struct gl_call_data {
  vector<vertex> vertices;
  vector<color> colors;
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

// TODO maybe write push_convex_polygon instead of implementing it below
// specifically for 'object's?


const vector3<fine_scalar> wc = lower_bound_in_fine_units(world_center_coords);

vector3<GLfloat> convert_coordinates_to_GL(vector3<fine_scalar> view_center, vector3<fine_scalar> input) {
  return vector3<GLfloat>(input - view_center) / tile_width;
}

tile_coordinate tile_manhattan_distance_to_bounding_box_rounding_down(bounding_box b, vector3<fine_scalar> const& v) {
  const fine_scalar xdist = (v.x < b.min.x) ? (b.min.x - v.x) : (v.x > b.max.x) ? (v.x - b.max.x) : 0;
  const fine_scalar ydist = (v.y < b.min.y) ? (b.min.y - v.y) : (v.y > b.max.y) ? (v.y - b.max.y) : 0;
  const fine_scalar zdist = (v.z < b.min.z) ? (b.min.z - v.z) : (v.z > b.max.z) ? (v.z - b.max.z) : 0;
  // Like (xdist / tile_width + ydist / tile_width + zdist / tile_height) but more precise:
  return (xdist + ydist + (zdist * tile_width / tile_height)) / tile_width;
}

void mainLoop (std::string scenario)
{
  SDL_Event event;
  int done = 0;
  int frame = 0;
  int p_mode = 0;
srand(time(NULL));

  world w( (worldgen_function_t(world_building_func(scenario))) );
  vector3<fine_scalar> laser_loc = wc + vector3<fine_scalar>(10ULL << 10, 10ULL << 10, 10ULL << 10);
  shared_ptr<robot> baz (new robot(laser_loc - vector3<fine_scalar>(0,0,tile_width*2), vector3<fine_scalar>(5<<9,3<<9,0)));
  object_identifier robot_id = w.try_create_object(baz); // we just assume that this works
  shared_ptr<laser_emitter> foo (new laser_emitter(laser_loc, vector3<fine_scalar>(5,3,1)));
  shared_ptr<laser_emitter> bar (new laser_emitter(laser_loc + vector3<fine_scalar>(0,0,tile_width*2), vector3<fine_scalar>(5,4,-1)));
  w.try_create_object(foo);
  w.try_create_object(bar);
  
  vector3<fine_scalar> view_loc_for_local_display = wc;
  enum { GLOBAL, LOCAL, ROBOT } view_type = GLOBAL;
  double view_direction = 0;

  vector3<fine_scalar> surveilled_by_global_display =
        wc + vector3<fine_scalar>(5*tile_width, 5*tile_width, 5*tile_width);
  fine_scalar globallocal_view_dist = 20*tile_width;
    
  while ( !done ) {
    microseconds_t begin_frame_monotonic_microseconds = get_monotonic_microseconds();

    /* Check for events */
    while ( SDL_PollEvent (&event) ) {
      switch (event.type) {
        case SDL_MOUSEMOTION:
          break;
          
        case SDL_MOUSEBUTTONDOWN:
          break;
          
        case SDL_KEYDOWN:
          if(event.key.keysym.sym == SDLK_p) ++p_mode;
          if(event.key.keysym.sym == SDLK_q) surveilled_by_global_display.x += tile_width;
          if(event.key.keysym.sym == SDLK_a) surveilled_by_global_display.x -= tile_width;
          if(event.key.keysym.sym == SDLK_w) surveilled_by_global_display.y += tile_width;
          if(event.key.keysym.sym == SDLK_s) surveilled_by_global_display.y -= tile_width;
          if(event.key.keysym.sym == SDLK_e) surveilled_by_global_display.z += tile_width;
          if(event.key.keysym.sym == SDLK_d) surveilled_by_global_display.z -= tile_width;
          if(event.key.keysym.sym == SDLK_r) globallocal_view_dist += tile_width;
          if(event.key.keysym.sym == SDLK_f) globallocal_view_dist -= tile_width;
          if(event.key.keysym.sym == SDLK_l) view_type = LOCAL;
          if(event.key.keysym.sym == SDLK_o) view_type = GLOBAL;
          if(event.key.keysym.sym == SDLK_i) view_type = ROBOT;
          if(event.key.keysym.sym != SDLK_ESCAPE)break;
          
        case SDL_QUIT:
          done = 1;
          break;
          
        default:
          break;
      }
    }
    
    Uint8 *keystate = SDL_GetKeyState(NULL);
    
    bool pd_this_time = (p_mode == 1);
    if(p_mode > 1)--p_mode;

    // microseconds_this_program_has_used_so_far
    microseconds_t microseconds_before_drawing = get_this_process_microseconds();

    //drawing code here

    vector3<fine_scalar> view_loc;
    vector3<fine_scalar> view_towards;

    //The gl_collection:s with higher indices here are intended to be
    //further away and rendered first (therefore covered up most
    //by everything else that's closer).
    std::unordered_map<size_t, gl_collection> gl_collections_by_distance;
    
    if (view_type == LOCAL) {
      view_loc = view_loc_for_local_display;
      view_towards = view_loc + vector3<fine_scalar>(
        globallocal_view_dist * std::cos(view_direction),
        globallocal_view_dist * std::sin(view_direction),
        0
      );
      if (keystate[SDLK_u]) {
        view_loc_for_local_display += vector3<fine_scalar>(
        fine_scalar(double(tile_width) * std::cos(view_direction)) / 10,
        fine_scalar(double(tile_width) * std::sin(view_direction)) / 10,
        0);
      }
      if (keystate[SDLK_j]) {
        view_loc_for_local_display -= vector3<fine_scalar>(
          fine_scalar(double(tile_width) * std::cos(view_direction)) / 10,
          fine_scalar(double(tile_width) * std::sin(view_direction)) / 10,
        0);
      }
      if (keystate[SDLK_h]) { view_direction += 0.06; }
      if (keystate[SDLK_k]) { view_direction -= 0.06; }
      if (keystate[SDLK_y]) { view_loc_for_local_display.z += tile_width / 10; }
      if (keystate[SDLK_n]) { view_loc_for_local_display.z -= tile_width / 10; }
    }
    else if (view_type == ROBOT) {
      bounding_box b = w.get_object_personal_space_shapes().find(robot_id)->second.bounds();
      view_loc = ((b.min + b.max) / 2);
      vector3<fine_scalar> facing = boost::dynamic_pointer_cast<robot>(w.get_objects().find(robot_id)->second)->get_facing();
      view_towards = view_loc + facing;
    }
    else {
      view_towards = surveilled_by_global_display;
      view_loc = surveilled_by_global_display + vector3<fine_scalar>(
        globallocal_view_dist * std::cos((double)frame / 40.0),
        globallocal_view_dist * std::sin((double)frame / 40.0),
        (globallocal_view_dist / 2) + (globallocal_view_dist / 4) * std::sin((double)frame / 60.0)
      );
    }
    
    unordered_set<object_or_tile_identifier> tiles_to_draw;
    /*w.collect_things_exposed_to_collision_intersecting(tiles_to_draw, tile_bounding_box(
      vector3<tile_coordinate>(world_center_coord + view_x - 50, world_center_coord + view_y - 50, world_center_coord + view_z - 50),
      vector3<tile_coordinate>(101,101,101)
    ));*/
    w.collect_things_exposed_to_collision_intersecting(tiles_to_draw, bounding_box(
      view_loc - vector3<fine_scalar>(tile_width*50,tile_width*50,tile_width*50),
      view_loc + vector3<fine_scalar>(tile_width*50,tile_width*50,tile_width*50)
    ));
    
    for (auto const& p : w.get_persistent_water_groups()) {
      persistent_water_group_info const& g = p.second;
      
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
          assert(bar.get_vertices().size() >= 3);
          // draw convex polygon via (sides - 2) triangles
          std::vector< vector3<int64_t> >::const_iterator vertices_i = bar.get_vertices().begin();
          const std::vector< vector3<int64_t> >::const_iterator vertices_end = bar.get_vertices().end();
          const vector3<GLfloat> first_vertex_locv = convert_coordinates_to_GL(view_loc, *vertices_i);
          ++vertices_i;
          vector3<GLfloat> prev_vertex_locv = convert_coordinates_to_GL(view_loc, *vertices_i);
          ++vertices_i;
          for (; vertices_i != vertices_end; ++vertices_i) {
            const vector3<GLfloat> this_vertex_locv = convert_coordinates_to_GL(view_loc, *vertices_i);
            push_triangle(coll, first_vertex_locv, prev_vertex_locv, this_vertex_locv, color(0x77777777));
            prev_vertex_locv = this_vertex_locv;
          }
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
        tile const& t = loc.stuff_at();
        vector3<GLfloat> locv = convert_coordinates_to_GL(view_loc, lower_bound_in_fine_units(loc.coords()));

        // Hack - TODO remove
        if (frame == 0 && t.contents() == GROUPABLE_WATER) {
          w.replace_substance(loc, GROUPABLE_WATER, UNGROUPABLE_WATER);
        }

        {
          const color tile_color =
            t.contents() ==              ROCK ? color(0x77000077) :
            t.contents() ==            RUBBLE ? color(0xffff0077) :
            t.contents() ==   GROUPABLE_WATER ? color(0x0000ff77) :
            t.contents() == UNGROUPABLE_WATER ? color(0x6666ff77) :
            (assert(false), (/*hack to make this compile*/0?color(0):throw 0xdeadbeef));

          const std::array<vector3<GLfloat>, 2> glb = {{
            convert_coordinates_to_GL(view_loc, lower_bound_in_fine_units(loc.coords())),
            convert_coordinates_to_GL(view_loc, upper_bound_in_fine_units(loc.coords()))
          }};

          // Only output the faces that are not interior to a single kind of material.
          if(loc.get_neighbor(cdir_zminus, CONTENTS_ONLY).stuff_at().contents() != t.contents()) {
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[0].z),
                      vertex(glb[1].x, glb[0].y, glb[0].z),
                      vertex(glb[1].x, glb[1].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[0].z),
                      tile_color);
          }
          if(loc.get_neighbor(cdir_zplus, CONTENTS_ONLY).stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[1].y, glb[1].z),
                      vertex(glb[0].x, glb[1].y, glb[1].z),
                      tile_color);
          }
          if(loc.get_neighbor(cdir_xminus, CONTENTS_ONLY).stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[1].z),
                      vertex(glb[0].x, glb[0].y, glb[1].z),
                      tile_color);
          }
          if(loc.get_neighbor(cdir_xplus, CONTENTS_ONLY).stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[1].x, glb[0].y, glb[0].z),
                      vertex(glb[1].x, glb[1].y, glb[0].z),
                      vertex(glb[1].x, glb[1].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[1].z),
                      tile_color);
          }
          if(loc.get_neighbor(cdir_yminus, CONTENTS_ONLY).stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[0].y, glb[0].z),
                      vertex(glb[0].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[1].z),
                      vertex(glb[1].x, glb[0].y, glb[0].z),
                      tile_color);
          }
          if(loc.get_neighbor(cdir_yplus, CONTENTS_ONLY).stuff_at().contents() != t.contents()){
            push_quad(coll,
                      vertex(glb[0].x, glb[1].y, glb[0].z),
                      vertex(glb[0].x, glb[1].y, glb[1].z),
                      vertex(glb[1].x, glb[1].y, glb[1].z),
                      vertex(glb[1].x, glb[1].y, glb[0].z),
                      tile_color);
          }
        }

        if (is_fluid(t.contents())) {
          if (active_fluid_tile_info const* fluid = w.get_active_fluid_info(loc)) {
            push_line(coll,
                      vertex(locv.x+0.5, locv.y+0.5, locv.z + 0.1),
                      vertex(
                        locv.x + 0.5 + ((GLfloat)fluid->velocity.x / (tile_width)),
                        locv.y + 0.5 + ((GLfloat)fluid->velocity.y / (tile_width)),
                        locv.z + 0.1 + ((GLfloat)fluid->velocity.z / (tile_width))),
                      color(0x00ff0077));

            for (EACH_CARDINAL_DIRECTION(dir)) {
              const sub_tile_distance prog = fluid->progress[dir];
              if (prog > 0) {
                vector3<GLfloat> directed_prog = (vector3<GLfloat>(dir.v) * prog) / progress_necessary(dir);

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

    microseconds_t microseconds_before_GL = get_this_process_microseconds();
    microseconds_t monotonic_microseconds_before_GL = get_monotonic_microseconds();

    //glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT/* | GL_DEPTH_BUFFER_BIT*/);
    frame += 1;
    glLoadIdentity();
    gluPerspective(80, 1, 0.1, 100);
    vector3<GLfloat> facing(view_towards - view_loc); facing /= tile_width;
    gluLookAt(0,0,0, facing.x,facing.y,facing.z, 0,0,1);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    std::vector<size_t> gl_collections_by_distance_order;
    for(auto const& p : gl_collections_by_distance) {
      gl_collections_by_distance_order.push_back(p.first);
    }
    //sort in descending order
    std::sort(gl_collections_by_distance_order.rbegin(), gl_collections_by_distance_order.rend());
    for(size_t i : gl_collections_by_distance_order) {
      gl_collection const& coll = gl_collections_by_distance[i];
      if(const size_t count = coll.quads.vertices.size()) {
        glVertexPointer(3, GL_FLOAT, 0, &coll.quads.vertices[0]);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, &coll.quads.colors[0]);
        glDrawArrays(GL_QUADS, 0, count);
      }
      if(const size_t count = coll.triangles.vertices.size()) {
        glVertexPointer(3, GL_FLOAT, 0, &coll.triangles.vertices[0]);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, &coll.triangles.colors[0]);
        glDrawArrays(GL_TRIANGLES, 0, count);
      }
      if(const size_t count = coll.lines.vertices.size()) {
        glVertexPointer(3, GL_FLOAT, 0, &coll.lines.vertices[0]);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, &coll.lines.colors[0]);
        glDrawArrays(GL_LINES, 0, count);
      }
      if(const size_t count = coll.points.vertices.size()) {
        glVertexPointer(3, GL_FLOAT, 0, &coll.points.vertices[0]);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, &coll.points.colors[0]);
        glDrawArrays(GL_POINTS, 0, count);
      }
    }
    
    
    glFinish();	
    SDL_GL_SwapBuffers();

    microseconds_t monotonic_microseconds_after_GL = get_monotonic_microseconds();
    microseconds_t microseconds_before_processing = get_this_process_microseconds();
    
    //doing stuff code here
    if (!pd_this_time) w.update();

    microseconds_t microseconds_after = get_this_process_microseconds();
    microseconds_t end_frame_monotonic_microseconds = get_monotonic_microseconds();

    // TODO does the GL code use up "CPU time"? Maybe measure both monotonic and CPU?
    microseconds_t microseconds_for_processing = microseconds_after - microseconds_before_processing;
    microseconds_t microseconds_for_drawing = microseconds_before_GL - microseconds_before_drawing;
    microseconds_t microseconds_for_GL = microseconds_before_processing - microseconds_before_GL;
    microseconds_t monotonic_microseconds_for_GL = monotonic_microseconds_after_GL - monotonic_microseconds_before_GL;
    microseconds_t monotonic_microseconds_for_frame = end_frame_monotonic_microseconds - begin_frame_monotonic_microseconds;
    double fps = 1000000.0 / monotonic_microseconds_for_frame;

    debug_print_microseconds(microseconds_for_processing);
    std::cerr << ", ";
    debug_print_microseconds(microseconds_for_drawing);
    std::cerr << ", ";
    debug_print_microseconds(microseconds_for_GL);
    std::cerr << "–";
    debug_print_microseconds(monotonic_microseconds_for_GL);
    std::cerr << " ms; " << fps << " fps; " << get_this_process_rusage().ru_maxrss / 1024 << "MiB\n";

//    SDL_Delay(50);
  }
}



static SDL_Surface *gScreen;

static void initAttributes ()
{
    // Setup attributes we want for the OpenGL context

    int value;

    // Don't set color bit sizes (SDL_GL_RED_SIZE, etc)
    //    Mac OS X will always use 8-8-8-8 ARGB for 32-bit screens and
    //    5-5-5 RGB for 16-bit screens

    // Request a 16-bit depth buffer (without this, there is no depth buffer)
    value = 16;
    SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, value);


    // Request double-buffered OpenGL
    //     The fact that windows are double-buffered on Mac OS X has no effect
    //     on OpenGL double buffering.
    value = 1;
    SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, value);
}

static void printAttributes ()
{
    // Print out attributes of the context we created
    int nAttr;
    int i;

    int  attr[] = { SDL_GL_RED_SIZE, SDL_GL_BLUE_SIZE, SDL_GL_GREEN_SIZE,
                    SDL_GL_ALPHA_SIZE, SDL_GL_BUFFER_SIZE, SDL_GL_DEPTH_SIZE };

    const char *desc[] = { "Red size: %d bits\n", "Blue size: %d bits\n", "Green size: %d bits\n",
                     "Alpha size: %d bits\n", "Color buffer size: %d bits\n",
                     "Depth bufer size: %d bits\n" };

    nAttr = sizeof(attr) / sizeof(int);

    for (i = 0; i < nAttr; i++) {

        int value;
        SDL_GL_GetAttribute ((SDL_GLattr)attr[i], &value);
        printf (desc[i], value);
    }
}

static void createSurface (int fullscreen)
{
    Uint32 flags = 0;

    flags = SDL_OPENGL;
    if (fullscreen)
        flags |= SDL_FULLSCREEN;

    // Create window
    gScreen = SDL_SetVideoMode (640, 640, 0, flags);
    if (gScreen == NULL) {

        fprintf (stderr, "Couldn't set 640x640 OpenGL video mode: %s\n",
                 SDL_GetError());
                SDL_Quit();
                exit(2);
        }
}



} /* end anonymous namespace */


int main(int argc, char *argv[])
{
	// Init SDL video subsystem
	if ( SDL_Init (SDL_INIT_VIDEO) < 0 ) {
		
        fprintf(stderr, "Couldn't initialize SDL: %s\n",
			SDL_GetError());
		exit(1);
	}

    // Set GL context attributes
    initAttributes ();
    
    // Create GL context
    createSurface (0);
    
    // Get GL context attributes
    printAttributes ();
    
    // Init GL state
	gluPerspective(90, 1, 1, 100);
	gluLookAt(20,20,20,0,0,0,0,0,1);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    
    // Draw, get events...
    if (argc < 2) {
      std::cerr << "You didn't give an argument saying which scenario to use! Using default value...";
      mainLoop ("default");
    }
    else mainLoop (argv[1]);
    
    // Cleanup
	SDL_Quit();
	
    return 0;
}
