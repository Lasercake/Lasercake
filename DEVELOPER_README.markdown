

Fonts
-----

To add (or remove) a font, you need to make the following changes:
* Add a .ttf font file to resources/
* Remember that Lasercake is GPLed so you shall also:
  Include your most-preferred format for editing the font next to the .ttf.
* List this file in fonts.qrc
* Add another QFontDatabase::addApplicationFont line in main.cpp
* Find out the "font name" of the font so you can refer to it in your code
* Rebuild Lasercake (these fonts are embedded in the executable)


Cross-compiling
---------------

Notes: from Linux to Windows via MinGW:

This is what I have to do as of 2012-11-01 on Arch Linux,
after installing several mingw32 packages:

- Extra CMake arguments:
    -DCMAKE_TOOLCHAIN_FILE=cmake/Toolchain-ArchLinux-mingw32.cmake

    (use/make a different toolchain file if your distro puts mingw files in
    a different place, or if you want to use a different toolchain)

- CMake might say

        CMake Warning: Manually-specified variables were not used by the project:
            CMAKE_TOOLCHAIN_FILE

  (it does for me); ignore that warning, it's incorrect.

- Then copy
/usr/i486-mingw32/{bin/{zlib1.dll,libpng15-15.dll},lib/{libgcc_s_sjlj-1.dll,libstdc++-6.dll,QtCore4.dll,QtGui4.dll,QtOpenGL4.dll}}
or
/usr/i686-w64-mingw32/{bin/{QtCore4.dll,QtGui4.dll,QtOpenGL4.dll,zlib1.dll,libpng15-15.dll},lib/{libgcc_s_sjlj-1.dll,libstdc++-6.dll}}
or
/usr/x86_64-w64-mingw32/{bin/{QtCore4.dll,QtGui4.dll,QtOpenGL4.dll,zlib1.dll,libpng15-15.dll},lib/{libgcc_s_sjlj-1.dll,libstdc++-6.dll}}
next to lasercake.exe
(the path for those dlls will depend on your distro)
(Anyone: do you know whether you're supposed to do it this way,
 or whether CMake or Windows or such has a better way to deal with
 pulling in these DLLs?)

Then run
`wine lasercake.exe`
and see if it runs!


Releasing
---------

### build flags ###

#### LTO (link-time optimization)

Using link-time optimization (tested with GCC 4.7, Feb 2013) cuts the
Lasercake binary size in half, though it doesn't make the program run
significantly faster (probably because we already put the performance
critical code in templates in the headers).  Linking with -flto takes
about a minute on my modern CPU so you probably don't want to do this
while developing.
-DLTO=ON (which is shorthand for
    -DCMAKE_CXX_FLAGS=-flto -DCMAKE_C_FLAGS=-flto -DCMAKE_EXE_LINKER_FLAGS=-fwhole-program
)

#### PGO (profile-guided optimization)

Profile-guided optimization did not currently appear to profitable enough
to bother with, but it has been before, so perhaps it will in the future. How to:
% cmake path/to/lasercake -DCMAKE_CXX_FLAGS=-fprofile-generate -DCMAKE_C_FLAGS=-fprofile-generate -DCMAKE_EXE_LINKER_FLAGS=-fprofile-generate
% make -j3
run the generated lasercake in various ways until you've exercised its various
performance-critical code paths.  This generates .gcda profiles next to the .o
binaries in CMakeFiles/lasercake.dir; each run adds to the existing .gcda files.
You probably want to keep this binary in case you didn't like the profile and want
to run it some more, so rename it.
% cmake path/to/lasercake -DCMAKE_CXX_FLAGS='-fprofile-use -fprofile-correction' -DCMAKE_C_FLAGS='-fprofile-use -fprofile-correction' -DCMAKE_EXE_LINKER_FLAGS='-fprofile-use -fprofile-correction'
% make -j3
enjoy your new binary.
(-fprofile-correction is necessary because Lasercake is multi-threaded.
Alternatively, you can run the profile-generating Lasercake with --no-threads.)

#### Mac OS X

-DCMAKE_C_COMPILER=/opt/local/bin/clang-mp-3.2
-DCMAKE_CXX_COMPILER=/opt/local/bin/clang++-mp-3.2
-DOSX_TARGET_10_5=ON
-DUSE_BOOST_CXX11_LIBS=ON
[plus above flags]

Compilers: Those paths are right if using Macports Clang.  If you have a
new enough XCode, it should have a new enough Clang already. (I don't have the
latest OS X to check.  Apple keep including snapshot Clang versions, so
I don't know which one is the first to fix all the bugs that make Clang
crash when compiling Lasercake.  Upstream Clang 3.1 is too old and upstream
Clang 3.2 is new enough.)  Recent GCC from Macports probably works too, but
Apple are moving towards Clang so it's probably better to prefer Clang on Mac
(all else equal).

Targeting 10.5: it's valuable not to exclude too many users! If you have
too new an OS X / XCode setup, you might not have the 10.5 SDK, or even 10.6.
(A good fraction of OS X users are still on 10.6! -Isaac, Feb 2013)

USE_BOOST_CXX11_LIBS: OS X before 10.7 uses GCC 4.2's libstdc++, which doesn't
support any C++11 library features.  This flag makes us use equivalent Boost
libraries instead.  Alternatives: OS X 10.7 has a copy of libc++ (C++ library
created by LLVM/Clang folks).  I hear 10.7's copy supports (some? all?) C++11
library features, but I have not tested their version (I don't have 10.7, and
in any case using it for releases would prevent the binary from running on
10.6).  It might also work to build your own recent lib(std)c++ and link it
into the binary, although it is problematic (in theory, and in practice if you
are using system Boost) to have more than one C++ runtime in the same
executable.

### packaging ###

TODO: look into CPack

### process ###

1: Git tag a release candidate.
2: Build binaries; upload them; get them tested on several platforms.
3: If there are any problems, repeat starting at step 1
4: Otherwise git tag and rebuild with the non -rc version number, upload that,
   update the website, etc.
