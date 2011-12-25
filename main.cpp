
/* Simple program:  Create a blank window, wait for keypress, quit.

   Please see the SDL documentation for details on using the SDL API:
   /Developer/Documentation/SDL/docs.html
*/
   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "SDL.h"
#include "GL/gl.h"
#include "GL/glu.h"

#include "water.cpp"
#include <iostream>

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

// Hack - situation building functions
void build_midair_water_tower(int midairness) {
  for (int x = 4; x <= 6; ++x) { for (int y = 4; y <= 6; ++y) { for(int z = midairness; z < MAX_Z; ++z) {
    tiles[location(x, y, z)].contents = WATER;
  }}}
}
void wall_midair_water_tower(int midairness) {
  for (int x = 3; x <= 7; ++x) { for (int y = 3; y <= 7; ++y) { for(int z = midairness; z < MAX_Z; ++z) {
    if (x == 3 || x == 7 || y == 3 || y == 7) tiles[location(x, y, z)].contents = ROCK;
  }}}
}
void build_extra_ground_walls() {
  for (int x = 0; x < MAX_X; ++x) { for(int z = 0; z < 3; ++z) {
    tiles[location(x, 1, z)].contents = ROCK;
    tiles[location(x, 9, z)].contents = ROCK;
  }}
  for (int y = 0; y < MAX_Y; ++y) { for(int z = 0; z < 3; ++z) {
    tiles[location(1, y, z)].contents = ROCK;
    tiles[location(9, y, z)].contents = ROCK;
  }}
}
void build_water_sheet_and_shallow_slope() {
  for (EACH_LOCATION(loc)) {
    if (loc.z == 0 && loc.x >= 5) tiles[loc].contents = ROCK;
    else if (loc.z == 1 && loc.x >= 10) tiles[loc].contents = ROCK;
    else if (loc.z == 2 && loc.x >= 15) tiles[loc].contents = ROCK;
    else if (loc.x == 19)tiles[loc].contents = WATER;
  }
}
void build_water_mass_and_steep_slope() {
  for (EACH_LOCATION(loc)) {
    if (loc.z < 20 - loc.x) tiles[loc].contents = ROCK;
    else if (loc.z >= 15 && (20 - loc.x) >= 15) tiles[loc].contents = WATER;
  }
}
void build_punctured_tank() {
  for (EACH_LOCATION(loc)) {
    if (loc.z < 8 && loc.x > 10) tiles[loc].contents = ROCK;
    else if (loc.x >= 12 && loc.y <= 13 && loc.y >= 7) { tiles[loc].contents = WATER; }
    else if (loc.y != 10 && loc.x > 10) tiles[loc].contents = ROCK;
    else if (loc.z > 8 && loc.x > 10 && loc.x < 18) tiles[loc].contents = ROCK;
    else if (loc.x > 10) { tiles[loc].contents = WATER; }
  }
}
void build_punctured_tank2() {
  for (EACH_LOCATION(loc)) {
    if (loc.z < 8 && loc.x > 10) tiles[loc].contents = ROCK;
    else if (loc.x >= 12 && loc.y <= 13 && loc.y >= 7) { tiles[loc].contents = WATER; }
    else if (loc.y != 9 && loc.y != 10 && loc.x > 10) tiles[loc].contents = ROCK;
    else if (loc.z > 9 && loc.x > 10 && loc.x < 18) tiles[loc].contents = ROCK;
    else if (loc.x > 10) { tiles[loc].contents = WATER; }
  }
}
void build_annoying_twisty_passageways() {
  for (EACH_LOCATION(loc)) {
    if (loc.x == 0)tiles[loc].contents = WATER;
    else if (loc.x == 1 && loc.z > 0) tiles[loc].contents = ROCK;
    else if (loc.x == 5) tiles[loc].contents = ROCK;
    else if (loc.x == 2 && (loc.z % 4) == 1) tiles[loc].contents = ROCK;
    else if (loc.x == 3 && (loc.z % 2) == 1) tiles[loc].contents = ROCK;
    else if (loc.x == 4 && (loc.z % 4) == 3) tiles[loc].contents = ROCK;
  }
}
void build_a_little_water_on_the_ground() {
  for (int x = 5; x <= 11; ++x) { for (int y = 5; y <= 11; ++y) {
    tiles[location(x, y, 0)].contents = WATER;
  }}
}

static void mainLoop (std::string scenario)
{
    SDL_Event event;
    int done = 0;
    int frame = 0;
    int p_mode = 0;
srand(time(NULL));
  for (EACH_LOCATION(loc))
  {
    tiles[loc].contents = AIR;
  }
  if (scenario == "tower1") { build_midair_water_tower(1); }
  else if (scenario == "tower2") { build_midair_water_tower(1); wall_midair_water_tower(1); }
  else if (scenario == "tower3") { build_midair_water_tower(1); wall_midair_water_tower(1); build_extra_ground_walls(); }
  else if (scenario == "shallow") { build_water_sheet_and_shallow_slope(); }
  else if (scenario == "steep") { build_water_mass_and_steep_slope(); }
  else if (scenario == "tank") { build_punctured_tank(); }
  else if (scenario == "tank2") { build_punctured_tank2(); }
  else if (scenario == "twisty") { build_annoying_twisty_passageways(); }
  else if (scenario == "droplet") { tiles[location(10,10,10)].contents = WATER; }
  else if (scenario == "droplets") { tiles[location(10,10,10)].contents = WATER; tiles[location(10,10,19)].contents = WATER; }
  else build_a_little_water_on_the_ground();


    
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
		
		glClear(GL_COLOR_BUFFER_BIT);
		
		//drawing code here
		frame += 1;
	glLoadIdentity();
	gluPerspective(80, 1, 1, 100);
	gluLookAt(5.0 + 20.0 * std::cos((double)frame / 40.0),5.0 + 20.0 * std::sin((double)frame / 40.0),15.0 + 5.0 * std::sin((double)frame / 60.0),5,5,5,0,0,1);
		int total_amount = 0;
		int max_pressure = 0;
		int total_force = 0;
		int max_velmgsq = 0;
	for (int x = 0; x < MAX_X; ++x) { for (int y = 0; y < MAX_Y; ++y) {
		glColor4f(0.2,0.4,0.0,1.0);

		glBegin(GL_POLYGON);
			glVertex3f(x, y, 0);
			glVertex3f(x + 1, y, 0);
			glVertex3f(x + 1, y +1, 0);
			glVertex3f(x, y+1, 0);
		glEnd();
	}}
	for (EACH_LOCATION(loc))
	{
		if (tiles[loc].contents != AIR)
		{
		  if (tiles[loc].contents == ROCK) {
		glColor4f(0.5,0.0,0.0,0.5);
		}
		else {
		  if(is_sticky_water(loc))glColor4f(0.0, 0.0, 1.0, 0.5);
		  else glColor4f(0.4, 0.4, 1.0, 0.5);
		}
		glBegin(GL_POLYGON);
			glVertex3f(loc.x, loc.y, (double)loc.z + 0.5);
			glVertex3f(loc.x + 1, loc.y, (double)loc.z + 0.5);
			glVertex3f(loc.x + 1, loc.y +1, (double)loc.z + 0.5);
			glVertex3f(loc.x, loc.y+1, (double)loc.z + 0.5);
		glEnd();
		if (tiles[loc].contents == WATER) {
		  water_movement_info &water = tiles.active_tiles[loc]; // TODO GET RID OF THIS INCREDIBLE HACK THAT ACTIVATES ALL THE WATER CONSTANTLY
		  //if (!can_be_exit_tile(loc)){
		  glColor4f(0.0, 1.0, 0.0, 0.5);
		  glBegin(GL_LINES);
			glVertex3f((double)loc.x+0.5, (double)loc.y+0.5, (double)loc.z + 0.5);
			glVertex3f((double)loc.x+0.5+((double)water.velocity.x / (250 * precision_factor)),
			  (double)loc.y+0.5+((double)water.velocity.y / (250 * precision_factor)),
			  (double)loc.z + 0.5+((double)water.velocity.z / (250 * precision_factor)));
		  glEnd();
		  //}
		  glColor4f(0.0, 0.0, 1.0, 0.5);
		  for (EACH_CARDINAL_DIRECTION(dir)) {
		    vector3 vect = (dir * water.progress[dir]);
		    glBegin(GL_LINES);
			glVertex3f((double)loc.x+0.5, (double)loc.y+0.5, (double)loc.z + 0.5);
			glVertex3f((double)loc.x+0.5+((double)vect.x / progress_necessary),
			  (double)loc.y+0.5+((double)vect.y / progress_necessary),
			  (double)loc.z + 0.5+((double)vect.z / progress_necessary));
		    glEnd();
		  
		  }
		}
		}
	}
		glFinish();	
        SDL_GL_SwapBuffers();
		int before_processing = SDL_GetTicks();
		
		//doing stuff code here
		update_water();
		int after = SDL_GetTicks();
std::cerr << (after - before_processing) << ", " << (before_processing - before_drawing) << "\n";

//	SDL_Delay(50);
		
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
      std::cerr << "You didn't give an argument saying which scenario to use!";
    }
    mainLoop (argv[1]);
    
    // Cleanup
	SDL_Quit();
	
    return 0;
}
