/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

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

   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "SDL.h"
#include "GL/gl.h"
#include "GL/glu.h"

#include <iostream>
#include <cmath>

#define ZTREE_TESTING
#include "../data_structures/geometry.hpp"

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

typedef lint64_t time_int_type;
typedef non_normalized_rational<time_int_type> time_type;
typedef faux_optional<time_type> optional_time;

void draw_vertex(vector3<geometry_int_type> v) {
  glVertex3f((GLfloat)v(X), (GLfloat)v(Y), (GLfloat)v(Z));
}
void draw_polyhedron(convex_polyhedron foo) {

          for (uint8_t i = 0; i < foo.face_info().size(); i += foo.face_info()[i] + 1) {
           glBegin(GL_LINE_LOOP);
            for (uint8_t j = 0; j < foo.face_info()[i]; ++j) {
              draw_vertex(foo.vertices()[foo.face_info()[i + j + 1]]);
            }
            glEnd();
          }
}

extern void compute_planes_info_for_intersection(convex_polyhedron const& ph, polyhedron_planes_info_for_intersection& collector);

static void mainLoop (std::string /*scenario*/)
{
  SDL_Event event;
  int done = 0;
  int p_mode = 0;
large_fast_noncrypto_rng rng(time(NULL));
typedef boost::random::uniform_int_distribution<int64_t> uniform_random;
  
  
  vector3<geometry_int_type> velocity(1,2,3);
int frame = 0;

  std::vector<vector3<geometry_int_type>> verts;
  verts.push_back(vector3<geometry_int_type>(3, 3, 3));
  verts.push_back(vector3<geometry_int_type>(3, -3, 3));
  verts.push_back(vector3<geometry_int_type>(-3, -3, 3));
  verts.push_back(vector3<geometry_int_type>(-3, 3, 3));
  verts.push_back(vector3<geometry_int_type>(0, 0, -3));
  convex_polyhedron foo1(verts);
  convex_polyhedron foo2(bounding_box::min_and_max(-vector3<geometry_int_type>(3, 3, 3), vector3<geometry_int_type>(3, 3, 3)));
  convex_polyhedron obstacle(bounding_box::min_and_max(-vector3<geometry_int_type>(3, 3, 1), vector3<geometry_int_type>(3, 3, 1)));
  
  bool draw_endp = false;
  bool draw_coll_stuff = false;
  bool draw_normals = false;
  bool draw_poly = true;
  bool use_foo1 = false;
    
  while ( !done ) {

    /* Check for events */
    while ( SDL_PollEvent (&event) ) {
      switch (event.type) {
        case SDL_MOUSEMOTION:
          break;
          
        case SDL_MOUSEBUTTONDOWN:
          break;
          
        case SDL_KEYDOWN:
          if(event.key.keysym.sym == SDLK_p) ++p_mode;
          if(event.key.keysym.sym == SDLK_z) draw_poly = !draw_poly;
          if(event.key.keysym.sym == SDLK_x) draw_normals = !draw_normals;
          if(event.key.keysym.sym == SDLK_c) draw_endp = !draw_endp;
          if(event.key.keysym.sym == SDLK_v) draw_coll_stuff = !draw_coll_stuff;
          if(event.key.keysym.sym == SDLK_b) use_foo1 = !use_foo1;
          if(event.key.keysym.sym == SDLK_q) ++velocity[X];
          if(event.key.keysym.sym == SDLK_a) --velocity[X];
          if(event.key.keysym.sym == SDLK_w) ++velocity[Y];
          if(event.key.keysym.sym == SDLK_s) --velocity[Y];
          if(event.key.keysym.sym == SDLK_e) ++velocity[Z];
          if(event.key.keysym.sym == SDLK_d) --velocity[Z];
          if(event.key.keysym.sym == SDLK_r) obstacle.translate(vector3<geometry_int_type>(1,0,0));
          if(event.key.keysym.sym == SDLK_f) obstacle.translate(vector3<geometry_int_type>(-1,0,0));
          if(event.key.keysym.sym == SDLK_t) obstacle.translate(vector3<geometry_int_type>(0,1,0));
          if(event.key.keysym.sym == SDLK_g) obstacle.translate(vector3<geometry_int_type>(0,-1,0));
          if(event.key.keysym.sym == SDLK_y) obstacle.translate(vector3<geometry_int_type>(0,0,1));
          if(event.key.keysym.sym == SDLK_h) obstacle.translate(vector3<geometry_int_type>(0,0,-1));
          //if(event.key.keysym.sym == SDLK_r) ++view_dist;
          //if(event.key.keysym.sym == SDLK_f) --view_dist;
          if(event.key.keysym.sym != SDLK_ESCAPE)break;
          
        case SDL_QUIT:
          done = 1;
          break;
          
        default:
          break;
      }
    }
    if(p_mode == 1)continue;
    if(p_mode > 1)--p_mode;
    int before_drawing = SDL_GetTicks();

    //drawing code here
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    gluPerspective(80, 1, 1, 100);
    gluLookAt(5+15*std::cos(double(frame) / 200),5+15*std::sin(double(frame) / 200),10,5,5,0,0,0,1);
  
  auto const& foo = use_foo1 ? foo1 : foo2;
  polyhedron_planes_info_for_intersection bar;
  compute_planes_info_for_intersection(foo, bar);
  
  if (draw_coll_stuff) {
  std::vector<pair_of_parallel_supporting_planes> relating_planes;
  populate_with_relating_planes(foo, obstacle, relating_planes);
  
      auto coll_info = when_do_polyhedra_intersect(foo, obstacle, velocity);
      if ((coll_info.is_anywhere) && (coll_info.max >= 0) && (coll_info.min <= (time_type(1)))) {
        if ((coll_info.min >= 0) || (coll_info.arbitrary_plane_of_closest_exclusion.normal.dot<geometry_int_type>(velocity) > 0)) {
  glColor4f(0.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.6);
          
        }
        else {
  glColor4f(0.5 + GLfloat(rand()%255) / 512.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.0, 0.6);
        }
      }
      else {


  glColor4f(0.5 + GLfloat(rand()%255) / 512.0, 0.0, 0.5 + GLfloat(rand()%255) / 512.0,  0.6);
      }

    for (auto const& p : relating_planes) {
            glBegin(GL_LINES);
              draw_vertex(p.p1_base_point);
              draw_vertex(p.p1_base_point + p.p1_to_p2_normal);
            glEnd();
      /*      glBegin(GL_LINES);
              draw_vertex(p.p2_base_point);
              draw_vertex(p.p2_base_point - p.p1_to_p2_normal);
            glEnd();*/
    }
    
      if (coll_info.is_anywhere) {
          for (uint8_t i = 0; i < foo.face_info().size(); i += foo.face_info()[i] + 1) {
           glBegin(GL_LINE_LOOP);
            for (uint8_t j = 0; j < foo.face_info()[i]; ++j) {
              auto v = foo.vertices()[foo.face_info()[i + j + 1]];
  glVertex3f((GLfloat)v(X) + (GLfloat)velocity(X) * (GLfloat)coll_info.min.numerator / (GLfloat)coll_info.min.denominator,
             (GLfloat)v(Y) + (GLfloat)velocity(Y) * (GLfloat)coll_info.min.numerator / (GLfloat)coll_info.min.denominator,
             (GLfloat)v(Z) + (GLfloat)velocity(Z) * (GLfloat)coll_info.min.numerator / (GLfloat)coll_info.min.denominator);
            }
            glEnd();
          }

  glColor4f(0.5 + GLfloat(rand()%255) / 512.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.5 + GLfloat(rand()%255) / 512.0,  0.6);
            glBegin(GL_LINES);
              draw_vertex(coll_info.arbitrary_plane_of_closest_exclusion.base_point);
              draw_vertex(coll_info.arbitrary_plane_of_closest_exclusion.base_point + coll_info.arbitrary_plane_of_closest_exclusion.normal);
            glEnd();
      }

  }

  glColor4f(0.5 + GLfloat(rand()%255) / 512.0, 0.0, 0.0, 0.6);
  draw_polyhedron(obstacle);
    if (draw_poly) {
  glColor4f(0.5 + GLfloat(rand()%255) / 512.0, 0.0, 0.0, 0.6);
  draw_polyhedron(foo);
        }
    if (draw_endp) { 
  glColor4f(0.5 + GLfloat(rand()%255) / 512.0, 0.0, 0.0, 0.6);
  convex_polyhedron foot(foo); foot.translate(velocity); draw_polyhedron(foot);
        }
          if (draw_normals) {
            glBegin(GL_LINES);
            for (auto const& pair : bar.base_points_and_outward_facing_normals) {
  glColor4f(0.0, 0.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.6);
              draw_vertex(pair.first);
              draw_vertex(pair.first + pair.second);
            }
            glEnd();
          }
          /*if (draw_sweep_normals) {
            glBegin(GL_LINES);
            for (auto const& pair : sweep_bar.base_points_and_outward_facing_normals) {
  glColor4f(0.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.6);
              draw_vertex(pair.first);
              draw_vertex(pair.first + pair.second);
            }
            glEnd();
          }
          if (draw_sweep_verts) {
          glPointSize(3);
            glBegin(GL_POINTS); 
            for (auto const& v : sweep_verts) {
  glColor4f(0.5 + GLfloat(rand()%255) / 512.0, 0.5 + GLfloat(rand()%255) / 512.0, 0.0, 0.6);
              draw_vertex(v);
            }
            glEnd();
          }*/
          
    
    int before_GL = SDL_GetTicks();
    
    glFinish();	
    SDL_GL_SwapBuffers();
   
    int before_processing = SDL_GetTicks();
    
    //doing stuff code here
	++frame;
    
    
    int after = SDL_GetTicks();
    std::cerr << (after - before_processing) << ", " << (before_GL - before_drawing) << ", " << (before_processing - before_GL) << "\n";

//    SDL_Delay(50);
  }
}

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
