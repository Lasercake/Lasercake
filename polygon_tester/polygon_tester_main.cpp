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
#include "../polygon_collision_detection.hpp"

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

struct floating_line {
  std::array<vector3<int64_t>, 2> ends;
  bool collided;
};

struct floating_poly {
  std::array<vector3<int64_t>, 2> ends;
  std::vector<vector3<int64_t>> vertices;
  bool collided;
};

static void mainLoop (std::string /*scenario*/)
{
  SDL_Event event;
  int done = 0;
  int p_mode = 0;
srand(time(NULL));
  
  double view_x = 5, view_y = 5, view_z = 5, view_dist = 20;
int frame = 0;
std::vector<floating_line> lines;
std::vector<floating_poly> polys;

for (int i = 0; i < 50; ++i) {
  floating_line foo;
  vector3<int64_t> first(rand()%1024, rand()%1024, 300 + rand()%100);
  vector3<int64_t> diff((rand()%200) - 100, (rand()%200) - 100, 10 + rand()%100);
  foo.ends[0] = first;
  foo.ends[1] = first + diff;
  lines.push_back(foo);
}
for (int i = 0; i < 50; ++i) {
  floating_poly foo;
  vector3<int64_t> first(rand()%1024, rand()%1024, -300 + rand()%100);
  vector3<int64_t> diff1((rand()%200), (rand()%200), 5 + rand()%20);
  vector3<int64_t> diff2((rand()%200) - 250, (rand()%200) - 250, 5 + rand()%20);
  foo.vertices.push_back(first);
  foo.vertices.push_back(first+diff1);
  foo.vertices.push_back(first+diff1+diff2);
  polys.push_back(foo);
}
  {
    /*std::vector<vector3<int64_t>> laer;
    laer.push_back(vector3<int64_t>(-20, 20, 20));
    laer.push_back(vector3<int64_t>(20, 20, 0));
    laer.push_back(vector3<int64_t>(0, -20, 0));
    convex_polygon oafe(laer);
    line_segment flwj(vector3<int64_t>(2, 4, 50));
    line_segment flwj(vector3<int64_t>(-2, -3, -50));*/
    std::vector<vector3<int64_t>> laer;
    laer.push_back(vector3<int64_t>(-20, 20, 0));
    laer.push_back(vector3<int64_t>(20, 20, 0));
    laer.push_back(vector3<int64_t>(0, -20, 0));
    convex_polygon oafe(laer);
    line_segment flwj(vector3<int64_t>(0, 0, 50), vector3<int64_t>(0, 0, -50));
    assert(shape(flwj).intersects(oafe));
 }
    
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
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    gluPerspective(80, 1, 1, 100);
    gluLookAt(5+15*std::cos(double(frame) / 200),5+15*std::sin(double(frame) / 200),10,5,5,0,0,0,1);
    for (floating_line& foo : lines) {
      if (foo.collided) glColor3f(1.0,0.0,1.0);
      else              glColor3f(0.0,0.0,1.0);
      glBegin(GL_LINES);
        glVertex3f(0.01*foo.ends[0].x, 0.01*foo.ends[0].y, 0.01*foo.ends[0].z);
        glVertex3f(0.01*foo.ends[1].x, 0.01*foo.ends[1].y, 0.01*foo.ends[1].z);
      glEnd();
    }
    for (floating_poly& foo : polys) {
      if (foo.collided) glColor3f(1.0,1.0,0.0);
      else              glColor3f(0.0,1.0,0.0);
      glBegin(GL_POLYGON);
        for (vector3<int64_t>& v : foo.vertices)
          glVertex3f(0.01*v.x, 0.01*v.y, 0.01*v.z);
      glEnd();
    }
    
    int before_GL = SDL_GetTicks();
    
    glFinish();	
    SDL_GL_SwapBuffers();
   
    int before_processing = SDL_GetTicks();
    
    //doing stuff code here
	++frame;
    for (floating_line& foo : lines) {
      for (vector3<int64_t>& v : foo.ends) --v.z;
      foo.collided = false;
    }
    for (floating_poly& foo : polys) {
      for (vector3<int64_t>& v : foo.vertices) ++v.z;
      foo.collided = false;
    }
    for (floating_line& foo : lines) {
      for (floating_poly& bar : polys) {
        if (shape(line_segment(foo.ends)).intersects(convex_polygon(bar.vertices))) {
          foo.collided = true;
          bar.collided = true;
      std::cerr << "GRONK\n";
        }
      }
    }
    
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
