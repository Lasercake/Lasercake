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
#include "specific_worlds.hpp"
#include "specific_object_types.hpp"

#include "rendering_the_world.hpp"

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




void output_gl_data_to_OpenGL(world_rendering::gl_all_data const& gl_data) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  //glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT/* | GL_DEPTH_BUFFER_BIT*/);
  glLoadIdentity();
  gluPerspective(80, 1, 0.1, 100);
  gluLookAt(0, 0, 0,
            gl_data.facing.x, gl_data.facing.y, gl_data.facing.z,
            gl_data.facing_up.x, gl_data.facing_up.y, gl_data.facing_up.z);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  std::vector<size_t> gl_collections_by_distance_order;
  for(auto const& p : gl_data.stuff_to_draw_as_gl_collections_by_distance) {
    gl_collections_by_distance_order.push_back(p.first);
  }
  //sort in descending order
  std::sort(gl_collections_by_distance_order.rbegin(), gl_collections_by_distance_order.rend());
  for(size_t i : gl_collections_by_distance_order) {
    world_rendering::gl_collection const& coll = gl_data.stuff_to_draw_as_gl_collections_by_distance.find(i)->second;
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
}



const vector3<fine_scalar> wc = lower_bound_in_fine_units(world_center_coords);

void mainLoop (std::string scenario)
{
  SDL_Event event;
  int done = 0;
  int frame = 0;
  int p_mode = 0;
  bool drawing_regular_stuff = true;
  bool drawing_debug_stuff = true;
srand(time(NULL));

  world w(make_world_building_func(scenario));
  vector3<fine_scalar> laser_loc = wc + vector3<fine_scalar>(10ULL << 10, 10ULL << 10, 10ULL << 10);
  shared_ptr<robot> baz (new robot(laser_loc - vector3<fine_scalar>(0,0,tile_width*2), vector3<fine_scalar>(5<<9,3<<9,0)));
  object_identifier robot_id = w.try_create_object(baz); // we just assume that this works
  shared_ptr<laser_emitter> foo (new laser_emitter(laser_loc, vector3<fine_scalar>(5,3,1)));
  shared_ptr<laser_emitter> bar (new laser_emitter(laser_loc + vector3<fine_scalar>(0,0,tile_width*2), vector3<fine_scalar>(5,4,-1)));
  w.try_create_object(foo);
  w.try_create_object(bar);

  // HACK - TODO remove at least the second effect
  // This hack has two effects:
  // Generating all the worldblocks near the start right away
  // and
  // (BIG HACK)
  // making water (that's near enough to the start) start out ungroupable
  // which helps us e.g. start with a pillar of water in the air that's about to fall,
  // so we can test how that physics works.
  {
    std::cerr << "\nInit: ";
    const microseconds_t microseconds_before_init = get_this_process_microseconds();

    unordered_set<object_or_tile_identifier> tiles_near_start;
    // I choose these distances big enough that, as of the time of writing this comment,
    // the GLOBAL view won't have to realize any new tiles in order to make a complete cycle
    // around wc.  This is an interim way to get rid of that annoying lag, at the cost
    // of a bit more of annoying startup time.
    w.collect_things_exposed_to_collision_intersecting(tiles_near_start, bounding_box(
      wc - vector3<fine_scalar>(tile_width*80,tile_width*80,tile_width*80),
      wc + vector3<fine_scalar>(tile_width*80,tile_width*80,tile_width*80)
    ));
    for (object_or_tile_identifier const& id : tiles_near_start) {
      if (tile_location const* locp = id.get_tile_location()) {
        tile_location const& loc = *locp;
        if (loc.stuff_at().contents() == GROUPABLE_WATER) {
          w.replace_substance(loc, GROUPABLE_WATER, UNGROUPABLE_WATER);
        }
      }
    }

    const microseconds_t microseconds_after_init = get_this_process_microseconds();
    const microseconds_t microseconds_for_init = microseconds_after_init - microseconds_before_init;
    debug_print_microseconds(microseconds_for_init);
    std::cerr << " ms\n";
  }

  view_on_the_world view(robot_id, wc);

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
          if(event.key.keysym.sym == SDLK_z) drawing_regular_stuff = !drawing_regular_stuff;
          if(event.key.keysym.sym == SDLK_t) drawing_debug_stuff = !drawing_debug_stuff;
          if(event.key.keysym.sym == SDLK_q) view.surveilled_by_global_display.x += tile_width;
          if(event.key.keysym.sym == SDLK_a) view.surveilled_by_global_display.x -= tile_width;
          if(event.key.keysym.sym == SDLK_w) view.surveilled_by_global_display.y += tile_width;
          if(event.key.keysym.sym == SDLK_s) view.surveilled_by_global_display.y -= tile_width;
          if(event.key.keysym.sym == SDLK_e) view.surveilled_by_global_display.z += tile_width;
          if(event.key.keysym.sym == SDLK_d) view.surveilled_by_global_display.z -= tile_width;
          if(event.key.keysym.sym == SDLK_r) view.globallocal_view_dist += tile_width;
          if(event.key.keysym.sym == SDLK_f) view.globallocal_view_dist -= tile_width;
          if(event.key.keysym.sym == SDLK_l) view.view_type = view_on_the_world::LOCAL;
          if(event.key.keysym.sym == SDLK_o) view.view_type = view_on_the_world::GLOBAL;
          if(event.key.keysym.sym == SDLK_i) view.view_type = view_on_the_world::ROBOT;
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

    world_rendering::gl_all_data gl_data;
    world_rendering_config rendering_config;
    rendering_config.drawing_regular_stuff = drawing_regular_stuff;
    rendering_config.drawing_debug_stuff = drawing_debug_stuff;
    rendering_config.keystate = keystate;
    view.render(w, rendering_config, gl_data);

    microseconds_t microseconds_before_GL = get_this_process_microseconds();
    microseconds_t monotonic_microseconds_before_GL = get_monotonic_microseconds();

    output_gl_data_to_OpenGL(gl_data);
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

    frame += 1;
    std::cerr << "Frame " << frame << ": ";
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




void print_SDL_GL_attributes()
{
    // Print out attributes of the context we created
    int nAttr;
    int i;

    int  attr[] = { SDL_GL_RED_SIZE, SDL_GL_BLUE_SIZE, SDL_GL_GREEN_SIZE,
                    SDL_GL_ALPHA_SIZE, SDL_GL_BUFFER_SIZE, SDL_GL_DEPTH_SIZE };

    const char *desc[] = { "Red size: %d bits\n", "Blue size: %d bits\n", "Green size: %d bits\n",
                     "Alpha size: %d bits\n", "Color buffer size: %d bits\n",
                     "Depth buffer size: %d bits\n" };

    nAttr = sizeof(attr) / sizeof(int);

    for (i = 0; i < nAttr; i++) {

        int value;
        SDL_GL_GetAttribute ((SDL_GLattr)attr[i], &value);
        printf (desc[i], value);
    }
}

SDL_Surface* create_SDL_GL_surface(bool fullscreen)
{
  SDL_Surface* result;

  // Don't set color bit sizes (SDL_GL_RED_SIZE, etc)
  //    Mac OS X will always use 8-8-8-8 ARGB for 32-bit screens and
  //    5-5-5 RGB for 16-bit screens
  // TODO but is that accurate for the other OSes?

  // Request a 16-bit depth buffer (without this, there is no depth buffer)
  SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, 16);


  // Request double-buffered OpenGL
  //     The fact that windows are double-buffered on Mac OS X has no effect
  //     on OpenGL double buffering.
  SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, 1);

  Uint32 sdl_video_mode_flags = SDL_OPENGL;
  if (fullscreen) {
    sdl_video_mode_flags |= SDL_FULLSCREEN;
  }

  // Create window
  result = SDL_SetVideoMode (640, 640, 0, sdl_video_mode_flags);
  if (result == NULL) {
    fprintf (stderr, "Couldn't set 640x640 OpenGL video mode: %s\n", SDL_GetError());
    SDL_Quit();
    exit(2);
  }

  return result;
}



} /* end anonymous namespace */


int main(int argc, char *argv[])
{
    // Init SDL video subsystem
    if ( SDL_Init (SDL_INIT_VIDEO) < 0 ) {
      fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
      exit(1);
    }
    
    // Create GL context
    create_SDL_GL_surface(false);
    
    // Get GL context attributes
    print_SDL_GL_attributes();
    
    // Draw, get events...
    std::string scenario;
    if (argc < 2) {
      std::cerr << "You didn't give an argument saying which scenario to use! Using default value...";
      scenario = "default";
    }
    else {
      scenario = argv[1];
    }
    
    mainLoop(scenario);
    
    // Cleanup
    SDL_Quit();
    
    return 0;
}
