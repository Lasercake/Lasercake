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
#include <iomanip>
#include <sstream>
#include <locale>

#include "world.hpp"
#include "specific_worlds.hpp"
#include "specific_object_types.hpp"

#include "rendering_the_world.hpp"

#include "input_representation.hpp"

#include "concurrency_utils.hpp"

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
microseconds_t get_this_thread_microseconds() {
  struct timespec ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return (microseconds_t)ts.tv_sec * 1000000 + (microseconds_t)ts.tv_nsec / 1000;
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

// Usage example:
// std::cerr << std::setw(6) << (ostream_bundle() << "foo" << 564) << std::endl;
struct ostream_bundle {
  template<typename T> ostream_bundle& operator<<(T const& t) { ss_ << t; return *this; }
  std::string str() { return ss_.str(); }
private:
  std::stringstream ss_;
};
std::ostream& operator<<(std::ostream& os, ostream_bundle& b) {
  return os << b.str();
}

// show_decimal(1234567, 1000, 10) --> "1234.5"
template<typename Integral, typename Integral2>
std::string show_decimal(Integral us, Integral2 divisor, int places, std::locale const& locale = std::locale("")) {
  Integral divisordivisor = 1;
  for(int i = 0; i < places; ++i) { divisordivisor *= 10; }
  
  return (ostream_bundle()
    << (us / divisor)
    << std::use_facet< std::numpunct<char> >(locale).decimal_point()
    << std::setfill('0') << std::setw(places) << std::abs(us / (divisor / divisordivisor) % divisordivisor)
  ).str();
}

std::string show_microseconds(microseconds_t us) {
  return show_decimal(us, 1000, 1);
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

object_identifier init_test_world_and_return_our_robot(world& w) {
  const vector3<fine_scalar> laser_loc = world_center_fine_coords + vector3<fine_scalar>(10ULL << 10, 10ULL << 10, 10ULL << 10);
  const shared_ptr<robot> baz (new robot(laser_loc - vector3<fine_scalar>(0,0,tile_width*2), vector3<fine_scalar>(5<<9,3<<9,0)));
  const object_identifier robot_id = w.try_create_object(baz); // we just assume that this works
  const shared_ptr<laser_emitter> foo (new laser_emitter(laser_loc, vector3<fine_scalar>(5,3,1)));
  const shared_ptr<laser_emitter> bar (new laser_emitter(laser_loc + vector3<fine_scalar>(0,0,tile_width*2), vector3<fine_scalar>(5,4,-1)));
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
    // around world-center.  This is an interim way to get rid of that annoying lag, at the cost
    // of a bit more of annoying startup time.
    w.collect_things_exposed_to_collision_intersecting(tiles_near_start, bounding_box(
      world_center_fine_coords - vector3<fine_scalar>(tile_width*80,tile_width*80,tile_width*80),
      world_center_fine_coords + vector3<fine_scalar>(tile_width*80,tile_width*80,tile_width*80)
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
    std::cerr << show_microseconds(microseconds_for_init) << " ms\n";
  }

  return robot_id;
}

typedef world_rendering::gl_all_data gl_data_t;

//is a pointer to avoid copying around all that data
typedef shared_ptr<gl_data_t> gl_data_ptr_t;

struct frame_output_t {
  gl_data_ptr_t gl_data_ptr;
  microseconds_t microseconds_for_drawing;
  microseconds_t microseconds_for_simulation;
};

using input_representation::input_news_t;

void sim_thread_step(
          world& w,
          view_on_the_world& view,
          concurrent::m_var<input_news_t>* take_input, //nullptr to not take input
          bool simulate,
          bool draw,
          concurrent::m_var<frame_output_t>* put_output //nullptr to not put output
                    ) {
  input_news_t input_news;
  if(take_input) {input_news = take_input->take();}

  const microseconds_t microseconds_before_simulating = get_this_thread_microseconds();
  if(simulate) {w.update(input_news);}
  const microseconds_t microseconds_after_simulating = get_this_thread_microseconds();
  const microseconds_t microseconds_for_simulation = microseconds_after_simulating - microseconds_before_simulating;

  const microseconds_t microseconds_before_drawing = get_this_thread_microseconds();
  if(take_input) {view.input(input_news);}
  gl_data_ptr_t gl_data_ptr(new gl_data_t());
  if(draw) {view.render(w, world_rendering_config(), *gl_data_ptr);}
  const microseconds_t microseconds_after_drawing = get_this_thread_microseconds();
  const microseconds_t microseconds_for_drawing = microseconds_after_drawing - microseconds_before_drawing;

  const frame_output_t output = {
    gl_data_ptr,
    microseconds_for_drawing,
    microseconds_for_simulation
  };

  if(put_output) {put_output->put(output);}
}

void mainLoop (std::string scenario)
{
  using namespace input_representation;
  
  concurrent::m_var<input_news_t> input_news_pipe;
  concurrent::m_var<frame_output_t> frame_output_pipe;
  
  const worldgen_function_t worldgen = make_world_building_func(scenario);
  if(!worldgen) {
    std::cerr << "Scenario name given that doesn't exist!: \'" << scenario << "\'\n";
    exit(4);
  }

#if !LASERCAKE_NO_THREADS
  concurrent::thread simulation_thread([&input_news_pipe, &frame_output_pipe, worldgen]() {
#endif
    world w(worldgen);
    const object_identifier robot_id = init_test_world_and_return_our_robot(w);
    view_on_the_world view(robot_id, world_center_fine_coords);
    sim_thread_step(w, view, nullptr, false, true, &frame_output_pipe);
#if !LASERCAKE_NO_THREADS
    while(true) {
      sim_thread_step(w, view, &input_news_pipe, true, true, &frame_output_pipe);
    }
  });
#endif

  const bool exploit_parallelism = true;
  if(!exploit_parallelism) {
    frame_output_pipe.take(); //ignore the result
  }

  frame_output_t last_frame_output;
  last_frame_output.gl_data_ptr.reset(new gl_data_t); //init in case necessary
  int done = 0;
  int frame = 0;
  bool paused = false;
  int steps_queued_to_do_while_paused = 0;

  microseconds_t microseconds_for_GL = 0;
  microseconds_t monotonic_microseconds_for_GL = 0;
  microseconds_t monotonic_microseconds_for_frame = 0;
  microseconds_t microseconds_for_simulation = 0;
  microseconds_t microseconds_for_drawing = 0;

  while ( !done ) {
    const microseconds_t begin_frame_monotonic_microseconds = get_monotonic_microseconds();

#if LASERCAKE_NO_THREADS
    if(frame != 0) {sim_thread_step(w, view, &input_news_pipe, true, true, &frame_output_pipe);}
#endif
    
    key_activity_t key_activity_since_last_frame;
    
    // TODO use SDL's TextInput API (and/or switch toolkits)
    // if that is appropriate for our interface somewhere and/or everywhere.

    // TODO I'd like at least arrow keys to have a different name
    // - to be Unicode arrow characters rather than e.g. "down".
    
    /* Check for events */
    SDL_Event event;
    while ( SDL_PollEvent (&event) ) {
      switch (event.type) {
        case SDL_MOUSEMOTION:
          break;
          
        case SDL_MOUSEBUTTONDOWN:
          break;
          
        case SDL_KEYDOWN:
          key_activity_since_last_frame.push_back(
            key_change_t(SDL_GetKeyName(event.key.keysym.sym), PRESSED));
          break;
          
        case SDL_KEYUP:
          key_activity_since_last_frame.push_back(
            key_change_t(SDL_GetKeyName(event.key.keysym.sym), RELEASED));
          break;
          
        case SDL_QUIT:
          done = 1;
          break;
          
        default:
          break;
      }
    }
    
    Uint8 const* const keystate = SDL_GetKeyState(NULL);
    keys_currently_pressed_t keys_currently_pressed;
    for(size_t i = SDLK_FIRST; i <= SDLK_LAST; ++i) {
      if(keystate[i]) {
        keys_currently_pressed.insert(SDL_GetKeyName((SDLKey)i));
      }
    }

    input_news_t input_news(keys_currently_pressed, key_activity_since_last_frame);

    for(key_change_t const& c : key_activity_since_last_frame) {
      if(c.second == PRESSED) {
        key_type const& k = c.first;
        std::cerr << k << '\n';
        if(k == "p") { paused = !paused; steps_queued_to_do_while_paused = 0;}
        if(k == "g") { if(paused) { ++steps_queued_to_do_while_paused; } }
        if(k == "escape") done = 1;
      }
    }
    
    const bool paused_this_time = paused && steps_queued_to_do_while_paused == 0;
    if(steps_queued_to_do_while_paused > 0) {
      --steps_queued_to_do_while_paused;
    }

    if(!paused_this_time) {
      input_news_pipe.put(input_news);
      last_frame_output = frame_output_pipe.take();
      if(last_frame_output.gl_data_ptr == nullptr) {
        last_frame_output.gl_data_ptr.reset(new gl_data_t);
      }
    }

    const microseconds_t microseconds_before_GL = get_this_process_microseconds();
    const microseconds_t monotonic_microseconds_before_GL = get_monotonic_microseconds();

    output_gl_data_to_OpenGL(*last_frame_output.gl_data_ptr);
    glFinish();
    SDL_GL_SwapBuffers();

    const microseconds_t monotonic_microseconds_after_GL = get_monotonic_microseconds();
    const microseconds_t microseconds_after_GL = get_this_process_microseconds();

    frame += 1;

    microseconds_for_simulation = last_frame_output.microseconds_for_simulation;
    microseconds_for_drawing = last_frame_output.microseconds_for_drawing;
    microseconds_for_GL = microseconds_after_GL - microseconds_before_GL;
    monotonic_microseconds_for_GL = monotonic_microseconds_after_GL - monotonic_microseconds_before_GL;
    
    const microseconds_t end_frame_monotonic_microseconds = get_monotonic_microseconds();

    monotonic_microseconds_for_frame = end_frame_monotonic_microseconds - begin_frame_monotonic_microseconds;
    
    std::string frames_per_second_str = " inf ";
    if(monotonic_microseconds_for_frame > 0) {
      const int64_t frames_per_kilosecond = 1000000000 / monotonic_microseconds_for_frame;
      frames_per_second_str = show_decimal(frames_per_kilosecond, 1000, 2);
    }
    std::cerr
    << "Frame " << std::left << std::setw(4) << frame << std::right << ":"
    //TODO bugreport: with fps as double, this produced incorrect results for me-- like multiplying output by 10
    // -- may be a libstdc++ bug (or maybe possibly me misunderstanding the library)
    //<< std::ios::fixed << std::setprecision(4) << fps << " fps; "
    << std::setw(4) << get_this_process_rusage().ru_maxrss / 1024 << "MiB; "
    << std::setw(6) << frames_per_second_str << "fps"
    << std::setw(6) << show_microseconds(monotonic_microseconds_for_frame) << "ms"
    << ":"
    << std::setw(7) << show_microseconds(microseconds_for_simulation) << "sim"
    << std::setw(6) << show_microseconds(microseconds_for_drawing) << "draw"
    << ":" << std::setw(6) << show_microseconds(microseconds_for_simulation + microseconds_for_drawing) << "sd"
    << " -> " << (ostream_bundle()
                            << ((microseconds_for_GL < monotonic_microseconds_for_GL) ? (show_microseconds(microseconds_for_GL) + "–") : std::string())
                            << show_microseconds(monotonic_microseconds_for_GL)
                        )
    << "gl\n";
  }

#if !LASERCAKE_NO_THREADS
  simulation_thread.interrupt();
  simulation_thread.join();
#endif
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
    std::cerr.imbue(std::locale(""));

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
