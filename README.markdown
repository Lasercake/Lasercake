
Welcome to Lasercake!
=====================

Lasercake is an open-world game about the environment.

Website: [lasercake.net](http://www.lasercake.net/)

We appreciate feedback.
Any way is good to contact us: [email](mailto:lasercake@googlegroups.com), [Google Group](https://groups.google.com/d/forum/lasercake), [IRC](https://webchat.freenode.net/?channels=lasercake) or [GitHub](https://github.com/Lasercake/Lasercake/issues).


Project blurb
=============

What is energy? Where does the energy we use come from? How do we gather and process materials, and what effects does that have on the planet we live on?

The Lasercake project aims to help people understand these things through the powerful medium of computer games.

**In Lasercake (the game), the player can use robots to build industrial projects – but unlike in similar games, every part of the world is based on real-life science.** Mine waste has to be dumped somewhere and causes pollution. Energy is conserved. Solar panels, wind turbines, and so on, harvest realistic amounts of energy. In short, we plan to include any and all scientific concepts that we can include while still keeping the game fun and engaging.

Most of those things don't exist yet – the game is far from complete – but we have a prototype showing some of the things we've done already.

We want to make awesome things free to everyone. Anyone with an Internet connection may download Lasercake and its source code without charge, and anyone with the ability may create and distribute modified versions under the terms of the GNU AGPL.

The current project team is [Eli Dupree](http://www.elidupree.com/) and [Isaac Dupree](http://www.idupree.com/). We began this project in December 2011 as an experiment in simulating water physics, and it's only kept expanding since.

We want your cool skills! The two of us could do this project on our own, but **it will be more awesome if you help us out.** There's a lot of different things that go into a big project like Lasercake – art, sustainable design, computer programming, geology, physics, sound design, gender studies, and many other things besides. If you want to help, any way is good to contact us: [email](mailto:lasercake@googlegroups.com), [Google Group](https://groups.google.com/d/forum/lasercake), [IRC](https://webchat.freenode.net/?channels=lasercake) or [GitHub](https://github.com/Lasercake/Lasercake/issues).


Game instructions
=================

When you start the game, click in the game window – then you'll be able to look around in-game using the mouse. You'll be looking from the point of view of a robot who can dig, shoot lasers, and build stuff. **All of the controls are written in-game** – you can probably learn most of the game by just trying all the controls that are listed, and reading the text that pops up. In fact, why don't you do that right now? You can come back and read the rest of this README if you get stuck.

The main controls are:

* Mouse motion: Look around.
* WASD: Move in the four horizontal directions, relative to the way you're looking.
* Space: Move upwards.
* (left) mouse button: Do whichever action you have selected.
* ZXCVB: Select different actions (they show descriptions when selected).

Your robot also levitates a little off the ground, so you can climb up shallow inclines with just the WASD keys.

You start out looking at a "refinery". Right now, it's a white box, but eventually we will make it look cooler. The resource flow currently goes like this:

You need metal to build things. You start with some metal, but to get more, you need to refine it. The simplest way to do that is...

1. Be in digging mode. (You start in digging mode, and you can switch back with "z").
2. Point at a nearby rock tile and click it turn turn it to rubble, then click it again to shove it.
3. Keep shoving the rock until it gets to the refinery input.
4. Repeat until you've gathered enough to make one tile of metal. (The refinery splits rubble into metal and waste rock – each rubble tile only has a little metal in it. The orangeish waste rock comes out of the refinery opposite the input, and the yellow metal comes out the side. Greener rock has more metal in it.)
5. Go near the yellow metal tile and click it to pick it up (note that you can't do this if you're still full of metal – each tile is 200 cubic meters and you can only carry a total of 421 cubic meters).

That's a pretty slow process. There's a lot of ways to do it faster. You can use lasers ("x") to disintegrate more rock at once. You can build more conveyors ("c") to move rubble automatically. You can build other robots ("b") to automatically dig for you. And you can build extra refineries ("v"), although there currently isn't any real advantage to using more than one.

You can also deconstruct buildings and retrieve their metal ("z" mode and click a nearby building), as long as you have enough room to store the extra metal.


Source code
===========

### Subsystems

#### GUI

Lasercake uses Qt and OpenGL to create its user interface.  The simulation, however, does *not* use these.
* `main.hpp` and `main.cpp` use Qt directly; `main.hpp` is processed with Qt's `moc` tool.
* `fonts.qrc` is a Qt-formatted resource.  Additional resource files are in `resources/`.

In order to access all OpenGL calls cross-platform, we use GLEW.  GLEW is compatible at runtime with Qt but their headers cannot both be included in the same source file.

* `gl_data_format.hpp` declares a format for data is passed to OpenGL.
* `gl_data_abstract.hpp` is a wrapper to treat that format as an abstract data type.
* `gl_data_preparation.[ch]pp` turns the simulation-data into this format, but does not use any OpenGL or Qt calls.
* `gl_rendering.[ch]pp` turns this format into OpenGL calls.
* `gl_qt_usage.cpp` is used by `gl_rendering.cpp`.  It exists only because (1) we're currently using Qt's text rendering since OpenGL doesn't come with a way to render text and (2) because of the GLEW/Qt incompatibility, the two have to be used in different source files.

User input is converted into `input_representation.hpp` to be sent into the simulation code in a hopefully-not-toolkit-dependent way.

#### Simulation

The simulation can use concurrency and logging, but should not have any other side-effects neither by syscall nor global variable.

The world is composed of "objects", which have positions measured in "fine distance units", and "tiles", which are grid-aligned and a fixed number of "fine distance units" wide and tall (on the order of a thousand, and they are less tall than wide).

* Objects move and collide with objects and tiles in `object_motion.cpp`
* Tiles move and interact with each other in `tile_physics.[ch]pp`
* Objects and tiles can be efficiently iterated in a meaningful order using `tile_iteration.hpp` or `object_and_tile_iteration.hpp`
* Specific types of objects have behaviors defined in `specific_object_types.[ch]pp`
* Player-controlled object types receive player input in the `input_representation.hpp` format
* The initial world state is infinite and randomly generated according to a world-generator choice from `specific_worlds.[ch]pp` that uses the interface defined in `worldgen.hpp`
* The tile system is introduced in `tiles.hpp`
* Tile contents are stored in a memory-efficient way in `the_decomposition_of_the_world_into_blocks.[ch]pp`
* The world is shown to the user using `gl_data_preparation.[ch]pp` which generates data to send to the GUI thread (which that thread will pass to OpenGL).
* The world state as a whole is defined in `world.hpp`
* Physical units and constants such as time and the force of gravity are defined in `world_constants.hpp` using unit-checking integral types defined in `units.hpp`
* `sunlight.cpp` is unused (a simulation of sunlight that was too slow)
* `disorganized_stuff.cpp` contains a few implementations that didn't neatly fit anywhere else and were too few in number to each deserve their own file

#### Helpers

* `config.hpp`: Every Lasercake C++ source file includes this, directly or indirectly.  It contains compiler/platform feature macros.  It defines a few variants on assert().  It contains a logging macro to be used instead of `std::cerr` (see Logging).
* `utils.hpp` and `data_structures/*` contain a variety of helper code useful mostly to the simulation.
* `cxx11/` is a compatibility layer for OSes that can have a C++11 compiler but no C++11 std libs (Mac OS X 10.6); see `cxx11/README.txt`
* `bundled_libs/`, `generate-boostbcp.py`: We bundle copies of some libraries we use with Lasercake source so that it's easier for people to build from source.  We are committed to Lasercake remaining easy to build from system versions of these libraries as well.  We have CMake options `USE_BUNDLED_*` to explicitly control whether system libraries or bundled copies are used.
* The build system is CMake, configured in `CMakeLists.txt` and `cmake/*`.  `dev-build.py` is a wrapper that might be more convenient for development; for example, it automatically runs the unit tests after building.
* `attic/` contains code that used to be used, and is still potentially useful but is not used for anything in Lasercake anymore.

#### Testing

* `tests/*` contains automated unit tests that are run by running `./Lasercake --run-self-tests`.
* `*_visualizer/` are miscellaneous 3D visualizer mini-programs to help understand complicated simulation data structures.  (Unfortunately they currently require the SDL library and are not integrated with the main executable.)
* `gdb-lasercake` and `callgrind-lasercake` are shorthands to debug or profile Lasercake.

#### Logging

Log messages are not sent to `std::cerr`, but rather to `LOG`, a macro defined in `config.hpp` with helper implementation in `log.cpp`.  It has the following benefits:

* `#include <iostream>` creates a static variable for each .cpp file to make extra sure that `std::cerr` is initialized before being used.  `LOG` does not do this.
* `LOG` works completely on the current thread's stack, with no shared state or locks (e.g. buffers shared between threads).  It writes using a single `write()` syscall.  (`write()` exists in mingw as well as Unixes.)  In theory, threads that use `LOG` are still compatible with being terminated by e.g. `pthread_cancel()`.
* `LOG` could be changed to write to a destination other than `std::cerr` easily.

#### Determinacy checking

The Lasercake simulation, from a given initial world state, must simulate exactly the same thing across different runs of Lasercake and different platforms.  If it doesn't, it is a bug leading to out-of-sync or worse.  We made a system to help test determinacy.  To use it, one must instrument the code by running `instrument_code_for_determinacy_checking.py`.  This makes Lasercake write to stdout, at most function calls, the function name and argument.  Run this Lasercake single-threadedly while writing its output to a file.  Run it again, or again on another platform with identical source code.  If the output traces are identical, this is a good sign.  If they are different, you can find the first line where they differ to track down the cause of out-of-sync.  Printing code is in `debug_print_deterministically.hpp`, included by `config.hpp` when DEBUG_PRINT_DETERMINISTICALLY is #defined to 1.


