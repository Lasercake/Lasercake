
Building Lasercake
==================

   Dependencies
------------------

### libraries ###

Lasercake currently depends on the following libraries:
(You'll need the development headers.)

- Qt (at least 4.8) -- http://qt-project.org/
- OpenGL
- GLM -- a small header-only C++ library that
         we should (TODO) include a copy of for your
         convenience -- http://glm.g-truc.net/

- optionally, Glib (for a small speed improvement. To
                   disable, pass -DGLIB=OFF to cmake)
- optionally, Boost ( http://www.boost.org/ . Lasercake
              comes with all the Boost code it needs,
              but if you're developing you might need
              it; ./generate-boostbcp.py has relevant
              comments too.)
- optionally, GLEW (we bundle a copy of GLEW - just a
              few C files - so you don't need it installed.
              We haven't yet implemented a CMake option to use
              system GLEW; if you don't want to use the
              bundled GLEW just ask us to implement such
              an option.)

### tools ###

CMake
A recent C++ compiler (e.g. GCC >= 4.6, CLang >= 3.1)
    that supports the C++0x features Lasercake uses.

optionally, Python (>= 2.6, including >= 3) for misc
    development scripts.
optionally, a shell (POSIX/Bourne-compatible) for misc
    development scripts.


To build Lasercake
==================

To build, run 'cmake .' then 'make' (or use your favorite CMake
interface).  (If developing, using ./dev-build.py might
float your boat better.)
It will give you an executable 'lasercake'.
Then you can run that and/or put it anywhere you like on
your computer.

These build instructions have been tested on:

* Linux [Arch Linux as well as Debian Testing, x86_64, Feb 2013].

* Mac OS X 10.6
      [using MacPorts qt4-mac and gcc47;
      cmake -DCMAKE_C_COMPILER=gcc-mp-4.7 -DCMAKE_CXX_COMPILER=g++-mp-4.7
      Run Lasercake and then click on it in the dock to bring the
      window to the front... our Mac integration needs improvement!
      2012-12-17: I'm getting crashes that I'm having trouble debugging.
      2013-02-14: It's working better now, but perhaps it's because of
        https://trac.macports.org/ticket/35770
        also see
          https://trac.macports.org/ticket/34288
      ..newer versions in MacPorts apparently don't include the new libstdc++,
      and so system C++11 library features on Mac reliably would require
      a new system std lib, namely (so I hear) libc++ in OS X 10.7.
      ]

* MinGW [Cross-compiling for Windows from Linux;
      I currently have to manually copy all needed .dlls to be
      next to lasercake.exe (in order to run it in Wine, at least)]

Please tell us if you try to build it elsewhere, no matter how
successful you are!

