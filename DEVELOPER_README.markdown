

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
    -DCMAKE_TOOLCHAIN_FILE=cmake/Toolchain-ArchLinux-mingw32.cmake -DGLIB=NO

    (use/make a different toolchain file if your distro puts mingw files in
    a different place)

    (GLIB=NO because it's only an ineffective performance thing currently
    and it's easier to have less deps here)

- CMake might say

        CMake Warning: Manually-specified variables were not used by the project:
            CMAKE_TOOLCHAIN_FILE

  (it does for me); ignore that warning, it's incorrect.

- Then copy
/usr/i486-mingw32/{bin/{zlib1.dll,libpng15-15.dll},lib/{libgcc_s_sjlj-1.dll,libstdc++-6.dll,QtCore4.dll,QtGui4.dll,QtOpenGL4.dll}}
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

Using link-time optimization (LTO) (tested with GCC 4.7, Feb 2013) cuts
the Lasercake binary size in half, though it doesn't make the program run
significantly faster (probably because we already put the performance
critical code in templates in the headers).  Linking with -flto takes
about a minute on my modern CPU so you probably don't want to do this
while developing.
-DCMAKE_CXX_FLAGS=-flto -DCMAKE_C_FLAGS=-flto -DCMAKE_EXE_LINKER_FLAGS=-fwhole-program

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

### packaging ###

TODO: look into CPack

### process ###

1: Git tag a release candidate.
2: Build binaries; upload them; get them tested on several platforms.
3: If there are any problems, repeat starting at step 1
4: Otherwise git tag and rebuild with the non -rc version number, upload that,
   update the website, etc.
