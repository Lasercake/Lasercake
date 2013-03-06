
Building Lasercake
==================

   Dependencies
------------------

### libraries ###

Lasercake currently depends on the following libraries:
(You'll need the development headers.)

- Qt (at least 4.8) -- http://qt-project.org/
- OpenGL

And optionally, small libraries that we bundle in the repo.
By default CMake will build using the bundled versions so you
don't have to install anything, but you can use system versions
with -DUSE_BUNDLED_*=NO for * = BOOST or GLEW or GLM.

- optionally, Boost -- http://www.boost.org/ . Lasercake comes with all the
                Boost code it needs, but if you're developing you might need
                it; ./generate-boostbcp.py has relevant comments.
- optionally, GLEW, an OpenGL portability wrapper library.  We bundle this.
- optionally, GLM -- a small header-only C++ library for graphics-related
                maths -- http://glm.g-truc.net/.  We bundle this.

- optionally, Glib (for its allocator, which might be slightly faster or
                    slower than system malloc.  We default to not using it
                    because I measure no benefit -- it was just an experiment.)

### tools ###

CMake
A recent C++ compiler (e.g. GCC >= 4.6, CLang >= 3.2)
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
It will give you an executable 'Lasercake'.
Then you can run that and/or put it anywhere you like on
your computer.

These build instructions have been tested on:

* Linux [Arch Linux as well as Debian Testing, x86 and x86_64, Feb 2013].

* Mac OS X 10.6
      [using either qt-project.org Qt 4.8 or MacPorts qt4-mac,
      and MacPorts clang-3.2 (which is in /opt/local/bin/);
      cmake -DCMAKE_C_COMPILER=clang-mp-3.2 -DCMAKE_CXX_COMPILER=clang++-mp-3.2
            -DUSE_BOOST_CXX11_LIBS=ON
      If you have 10.7+ and a new enough XCode, you should only
      need to install Qt and everything should "just work".  We don't
      have this so we can't test it.  Let us know how you fare!
      ]

* MinGW [Cross-compiling for Windows from Linux;
      I currently have to manually copy all needed .dlls to be
      next to lasercake.exe (in order to run it in Wine, at least)]

Please tell us if you try to build it elsewhere, no matter how
successful you are!

