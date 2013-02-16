

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

