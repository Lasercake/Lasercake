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
#include "../data_structures/bbox_collision_detector.hpp"
#define BBOX_COLLISION_DETECTOR_IMPL_TREAT_AS_HEADER 1
#include "../data_structures/bbox_collision_detector.cpp"

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


namespace collision_detector {
namespace impl {
struct zbox_debug_visualizer {
  zbox_debug_visualizer():frame(0){}
  typedef bbox_collision_detector<int, 64, 3> collision_detector_t;
  collision_detector_t collision_detector;
  typedef collision_detector_t::bounding_box bounding_box;
  typedef bounding_box::coordinate_array coordinate_array;
  collision_detector_t::bounding_box floating_bbox;
  int frame;
  void update() {
    frame++;
    floating_bbox = bounding_box::min_and_size_minus_one(
      coordinate_array({{
        static_cast<uint64_t>((1ULL<<63) + (10LL<<32)*std::sin(double(frame) / 600)),
        static_cast<uint64_t>((1ULL<<63) + (10LL<<32)*std::sin(double(frame) / 670)),
        static_cast<uint64_t>((1ULL<<63) + (10LL<<32)*std::sin(double(frame) / 720)) }}),
      coordinate_array({{
        static_cast<uint64_t>((1ULL<<32) + (1LL<<31)*std::sin(double(frame) / 750)),
        static_cast<uint64_t>((1ULL<<32) + (1LL<<31)*std::sin(double(frame) / 780)),
        static_cast<uint64_t>((1ULL<<32) + (1LL<<31)*std::sin(double(frame) / 830)) }}));
    if (frame > 1)
      collision_detector.erase(0);
    collision_detector.insert(0, floating_bbox);
  }
  void stupid_draw(double xmin, double ymin, double zmin, double xsiz, double ysiz, double zsiz)const {
    glBegin(GL_LINES);
      glVertex3f(xmin,     ymin,     zmin     );
      glVertex3f(xmin+xsiz,ymin,     zmin     );
      glVertex3f(xmin,     ymin,     zmin     );
      glVertex3f(xmin,     ymin+ysiz,zmin     );
      glVertex3f(xmin,     ymin,     zmin     );
      glVertex3f(xmin,     ymin,     zmin+zsiz);
      glVertex3f(xmin+xsiz,ymin,     zmin     );
      glVertex3f(xmin+xsiz,ymin+ysiz,zmin     );
      glVertex3f(xmin+xsiz,ymin,     zmin     );
      glVertex3f(xmin+xsiz,ymin,     zmin+zsiz);
      glVertex3f(xmin,     ymin+ysiz,zmin     );
      glVertex3f(xmin,     ymin+ysiz,zmin+zsiz);
      glVertex3f(xmin,     ymin+ysiz,zmin     );
      glVertex3f(xmin+xsiz,ymin+ysiz,zmin     );
      glVertex3f(xmin,     ymin,     zmin+zsiz);
      glVertex3f(xmin+xsiz,ymin,     zmin+zsiz);
      glVertex3f(xmin,     ymin,     zmin+zsiz);
      glVertex3f(xmin,     ymin+ysiz,zmin+zsiz);
      glVertex3f(xmin+xsiz,ymin+ysiz,zmin     );
      glVertex3f(xmin+xsiz,ymin+ysiz,zmin+zsiz);
      glVertex3f(xmin+xsiz,ymin,     zmin+zsiz);
      glVertex3f(xmin+xsiz,ymin+ysiz,zmin+zsiz);
      glVertex3f(xmin,     ymin+ysiz,zmin+zsiz);
      glVertex3f(xmin+xsiz,ymin+ysiz,zmin+zsiz);
    glEnd();
  }
  void stupid_draw(collision_detector_t::bounding_box drawn_bbox)const {
    double xmin = int64_t(drawn_bbox.min(0) - (1ULL << 63)) / double(1ULL << 32);
    double ymin = int64_t(drawn_bbox.min(1) - (1ULL << 63)) / double(1ULL << 32);
    double zmin = int64_t(drawn_bbox.min(2) - (1ULL << 63)) / double(1ULL << 32);
    double xsiz = (double(drawn_bbox.size_minus_one(0)) + 1) / double(1ULL << 32);
    double ysiz = (double(drawn_bbox.size_minus_one(1)) + 1) / double(1ULL << 32);
    double zsiz = (double(drawn_bbox.size_minus_one(2)) + 1) / double(1ULL << 32);
    stupid_draw(xmin,ymin,zmin,xsiz,ysiz,zsiz);
  }
  void draw_node(collision_detector_t::ztree_node_ptr const& tree)const {
    if (tree) {
      if (tree->objects_here.empty()) glColor3f(0.5,0.0,0.0);
      else glColor3f(1.0,0.0,0.0);
      stupid_draw(tree->here.get_bbox());
      draw_node(tree->child0);
      draw_node(tree->child1);
    }
  }
  void draw()const {
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    gluPerspective(80, 1, 1, 100);
    gluLookAt(30*std::cos(double(frame) / 200),30*std::sin(double(frame) / 200),15,0,0,0,0,0,1);
    draw_node(collision_detector.objects_tree_);
    glColor3f(0.0,0.0,1.0);
    stupid_draw(floating_bbox);
  }
};
}
}


static void mainLoop (std::string /*scenario*/)
{
  SDL_Event event;
  int done = 0;
  int p_mode = 0;
srand(time(NULL));

  collision_detector::impl::zbox_debug_visualizer w;
  
  double view_x = 5, view_y = 5, view_z = 5, view_dist = 20;
    
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
          if(event.key.keysym.sym == SDLK_q) ++view_x;
          if(event.key.keysym.sym == SDLK_a) --view_x;
          if(event.key.keysym.sym == SDLK_w) ++view_y;
          if(event.key.keysym.sym == SDLK_s) --view_y;
          if(event.key.keysym.sym == SDLK_e) ++view_z;
          if(event.key.keysym.sym == SDLK_d) --view_z;
          if(event.key.keysym.sym == SDLK_r) ++view_dist;
          if(event.key.keysym.sym == SDLK_f) --view_dist;
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
    w.draw();
    
    int before_GL = SDL_GetTicks();
    
    glFinish();	
    SDL_GL_SwapBuffers();
   
    int before_processing = SDL_GetTicks();
    
    //doing stuff code here
    w.update();
    
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
