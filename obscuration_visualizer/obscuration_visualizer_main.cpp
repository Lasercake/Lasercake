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

#define USE_BOUNDS_CHECKED_INTS 1
#include "../data_structures/collection_of_obscured_area_boundaries.hpp"
// purely for a convenient bounding box type:
#include "../data_structures/polygon_collision_detection.hpp"

using namespace collection_of_obscured_area_boundaries_impl;

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

void draw_vertex(coord_type x, coord_type y) {
  glVertex3f((GLfloat)x.numerator.get() / (GLfloat)x.denominator.get(), (GLfloat)y.numerator.get() / (GLfloat)y.denominator.get(), 0.0);
}

void draw_lines(logical_node_lines_collection const& lc) {
  bounds_2d const& b = lc.bounds;
  for (auto const& l : lc.hlines) {
    glBegin(GL_LINES);
      draw_vertex(l.b1 > b.min_x ? l.b1 : b.min_x,l.l);
      draw_vertex(l.b2 < b.max_x ? l.b2 : b.max_x,l.l);
    glEnd();
  }
  for (auto const& l : lc.vlines) {
    glBegin(GL_LINES);
      draw_vertex(l.l, l.b1 > b.min_y ? l.b1 : b.min_y);
      draw_vertex(l.l, l.b2 < b.max_y ? l.b2 : b.max_y);
    glEnd();
  }
  for (auto const& l : lc.plines) {
  glColor4f(GLfloat(rand()%255) / 255.0, GLfloat(rand()%255) / 255.0, GLfloat(rand()%255) / 255.0, 0.8);
    coord_type startx;
    coord_type starty;
    coord_type endx;
    coord_type endy;
    if (l.b1.is_x_boundary) {
      startx = l.b1.b;
      starty = l.y_intercept(startx);
    }
    else {
      starty = l.b1.b;
      startx = l.x_intercept(starty);
    }
    if (l.b2.is_x_boundary) {
      endx = l.b2.b;
      endy = l.y_intercept(endx);
    }
    else {
      endy = l.b2.b;
      endx = l.x_intercept(endy);
    }
    
    assert(startx <= b.max_x);
    assert(endx >= b.min_x);
    if (l.slope > coord_type(0)) {
      assert(starty <= b.max_y);
      assert(endy >= b.min_y);
    }
    else {
      assert(endy <= b.max_y);
      assert(starty >= b.min_y);
    }
    
    if (startx < b.min_x) {
      startx = b.min_x;
      starty = l.y_intercept(startx);
    }
    if (endx > b.max_x) {
      endx = b.max_x;
      endy = l.y_intercept(endx);
    }
    if (l.slope > coord_type(0)) {
      if (starty < b.min_y) {
        starty = b.min_y;
        startx = l.x_intercept(starty);
      }
      if (endy > b.max_y) {
        endy = b.max_y;
        endx = l.x_intercept(endy);
      }
    }
    else {
      if (endy < b.min_y) {
        endy = b.min_y;
        endx = l.x_intercept(endy);
      }
      if (starty > b.max_y) {
        starty = b.max_y;
        startx = l.x_intercept(starty);
      }
    }
    glBegin(GL_LINES);
      draw_vertex(startx,starty);
      draw_vertex(endx,endy);
    glEnd();
  }
}

void draw_node(logical_node n) {
  bounds_2d b = n.bounds();
  glColor4f(1.0,1.0,1.0, 0.2);
  glBegin((n.contents_type == ALL_BLOCKED) ? GL_QUADS : GL_LINE_LOOP);
    draw_vertex(b.min_x, b.min_y);
    draw_vertex(b.max_x, b.min_y);
    draw_vertex(b.max_x, b.max_y);
    draw_vertex(b.min_x, b.max_y);
  glEnd();
  if (n.contents_type == MIXED_AS_CHILDREN) {
    for(int i = 0; i < 4; ++i) {
      draw_node(n.child(i));
    }
  }
  if (n.contents_type == MIXED_AS_LINES) {
    glColor4f(1.0,0.5,0.5, 0.6);
    draw_lines(*(n.lines_pointer_reference()));
  }
}

static void mainLoop (std::string /*scenario*/)
{
  SDL_Event event;
  int done = 0;
  int p_mode = 0;
large_fast_noncrypto_rng rng(time(NULL));
typedef boost::random::uniform_int_distribution<int64_t> uniform_random;
  
  double view_x = 5, view_y = 5, view_z = 5, view_dist = 20;
int frame = 0;
  obscured_areas_tree foo;
  
  bounding_box box_to_draw;
  logical_node_lines_collection lc_to_draw;
  bool last_was_visible = true;
  int m_mode = 0;
  int total_process_milliseconds = 0;
  
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
          if(event.key.keysym.sym == SDLK_m) ++m_mode;
          if(event.key.keysym.sym == SDLK_u) m_mode += 100;
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
    glLineWidth(2);
    glLoadIdentity();
    gluPerspective(80, 1, 1, 100);
    gluLookAt(0.0+1.0*std::cos(double(frame) / 200),0.0+1.0*std::sin(double(frame) / 200),2.0,0,0,0,0,0,1);
    
    glColor4f(0.0,last_was_visible ? 1.0 : 0.0, last_was_visible ? 0.0 : 1.0, 0.3);
    glBegin(GL_QUADS);
      glVertex3f(-1.0, -1.0, 0.0);
      glVertex3f(1.0, -1.0, 0.0);
      glVertex3f(1.0, 1.0, 0.0);
      glVertex3f(-1.0, 1.0, 0.0);
    glEnd();
    glBegin(GL_QUADS);
      glVertex3f(-1.2, -1.1, 0.0);
      glVertex3f(-1.0, -1.1, 0.0);
      glVertex3f(-1.0, -1.0, 0.0);
      glVertex3f(-1.2, -1.0, 0.0);
    glEnd();
    
    draw_node(foo.top_logical_node());
    
    glColor4f(1.0,0.0,0.0, 0.5);
    draw_lines(lc_to_draw);
    
    int before_GL = SDL_GetTicks();
    
    glFinish();	
    SDL_GL_SwapBuffers();
   
    int before_processing = SDL_GetTicks();
    
    //doing stuff code here
	++frame;
	
    if(m_mode && lc_to_draw.empty()) {
      --m_mode;
      int min_z = (rand()%60000) + 1000;
      bounding_box::vect min(rand()%(2*min_z - 1000) - min_z, rand()%(2*min_z  - 1000) - min_z, min_z);
      bounding_box::vect size(1000,1000,1000);
      // Hack - avoid having axis-aligned perspective lines
      if (min.x == 0) ++min.x;
      if (min.x + size.x == 0) ++min.x;
      if (min.y == 0) ++min.y;
      if (min.y + size.y == 0) ++min.y;
      bounding_box::vect max(min + size);
      box_to_draw = bounding_box::min_and_max(min, max);
      lc_to_draw.insert_vline(axis_aligned_line(
        coord_type(min.x, (min.x > 0) ? max.z : min.z),
        coord_type(min.y, (min.x > 0) ? max.z : min.z),
        coord_type(max.y, (min.x > 0) ? max.z : min.z), true));
      lc_to_draw.insert_hline(axis_aligned_line(
        coord_type(min.y, (min.y > 0) ? max.z : min.z),
        coord_type(min.x, (min.y > 0) ? max.z : min.z),
        coord_type(max.x, (min.y > 0) ? max.z : min.z), true));
      lc_to_draw.insert_vline(axis_aligned_line(
        coord_type(max.x, (max.x < 0) ? max.z : min.z),
        coord_type(min.y, (max.x < 0) ? max.z : min.z),
        coord_type(max.y, (max.x < 0) ? max.z : min.z), false));
      lc_to_draw.insert_hline(axis_aligned_line(
        coord_type(max.y, (max.y < 0) ? max.z : min.z),
        coord_type(min.x, (max.y < 0) ? max.z : min.z),
        coord_type(max.x, (max.y < 0) ? max.z : min.z), false));
      if ((min.x > 0) != (min.y > 0)) {
        assert((min.x > 0) == (((min.x > 0) ? max.z : min.z) > ((min.y > 0) ? max.z : min.z)));
        lc_to_draw.insert_pline(perspective_line(coord_type(min.y, min.x),
            x_or_y_boundary(coord_type(min.x, (min.x > 0) ? max.z : min.z), true),
            x_or_y_boundary(coord_type(min.y, (min.y > 0) ? max.z : min.z), false),
            true
          ));
      }
      if ((max.x > 0) != (max.y > 0)) {
        assert((max.x > 0) == (((max.y > 0) ? min.z : max.z) > ((max.x > 0) ? min.z : max.z)));
        lc_to_draw.insert_pline(perspective_line(coord_type(max.y, max.x),
            x_or_y_boundary(coord_type(max.y, (max.y > 0) ? min.z : max.z), false),
            x_or_y_boundary(coord_type(max.x, (max.x > 0) ? min.z : max.z), true),
            false
          ));
      }
      if ((min.x > 0) == (max.y > 0)) {
        assert((min.x > 0) == (((min.x > 0) ? max.z : min.z) > ((max.y > 0) ? min.z : max.z)));
        lc_to_draw.insert_pline(perspective_line(coord_type(max.y, min.x),
            x_or_y_boundary(coord_type(min.x, (min.x > 0) ? max.z : min.z), true),
            x_or_y_boundary(coord_type(max.y, (max.y > 0) ? min.z : max.z), false),
            true
          ));
      }
      if ((max.x > 0) == (min.y > 0)) {
        assert((max.x > 0) == (((min.y > 0) ? max.z : min.z) > ((max.x > 0) ? min.z : max.z)));
        lc_to_draw.insert_pline(perspective_line(coord_type(min.y, max.x),
            x_or_y_boundary(coord_type(min.y, (min.y > 0) ? max.z : min.z), false),
            x_or_y_boundary(coord_type(max.x, (max.x > 0) ? min.z : max.z), true),
            false
          ));
      }
    }
    if(m_mode && !lc_to_draw.empty()) {
      --m_mode;
      last_was_visible = foo.insert_from_lines_collection(lc_to_draw);
      lc_to_draw.clear();
    }
    
    int after = SDL_GetTicks();
    total_process_milliseconds += (after - before_processing);
    std::cerr << frame << ": " << total_process_milliseconds << ", " << (after - before_processing) << ", " << (before_GL - before_drawing) << ", " << (before_processing - before_GL) << "\n";

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
