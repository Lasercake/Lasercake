

Localization
------------

Currently we use Qt Linguist.
(TODO: mark strings with tr(), and check that it works.)
We could switch to gettext if you prefer.

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

### scripts ###

release-build.py is suitable for building for Linux and Windows.
Notes and rationale are below.

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

git clone https://github.com/Lasercake/Lasercake.git Lasercake-0.23-clean
mkdir Lasercake-0.23-build
cd Lasercake-0.23-build
cmake ../Lasercake-0.23-clean
-DCMAKE_C_COMPILER=/opt/local/bin/clang-mp-3.2
-DCMAKE_CXX_COMPILER=/opt/local/bin/clang++-mp-3.2
-DCMAKE_OSX_DEPLOYMENT_TARGET=10.5
-DCMAKE_OSX_SYSROOT=/Developer/SDKs/MacOSX10.5.sdk
-DUSE_BOOST_CXX11_LIBS=ON
[plus above flags]

make -j3

Then `cpack -G DragNDrop` to create a DMG containing the .app (in a version
that should start on other users' systems), a ReadMe, and
an alias to /Applications for the user.

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
But does this actually make 10.5 work?  Our testing indicates otherwise.

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

#### Linux

sudo debootstrap --arch=i386 wheezy ./debootstrap-x86-wheezy http://ftp.us.debian.org/debian
sudo debootstrap --arch=amd64 wheezy ./debootstrap-amd64-wheezy http://ftp.us.debian.org/debian
# set up bind-mounts etc. for a functioning chroot
# https://github.com/idupree/scripts/blob/master/superchroot
sudo superchroot ./debootstrap-x86-wheezy
aptitude update
aptitude install build-essential libqt4-dev cmake git python
OR
aptitude update; aptitude full-upgrade
For a user that's on your main system:
addgroup --gid xxxx name; adduser --uid xxxx --gid xxxx name
su - name
git clone https://github.com/Lasercake/Lasercake.git Lasercake-0.23-clean
mkdir Lasercake-0.23-build
cd Lasercake-0.23-build
cmake ../Lasercake-0.23-clean [plus above optimization flags]
make -j3
mkdir Lasercake-0.23-linux-[arch]-dynamic
mv Lasercake Lasercake-0.23-linux-[arch]-dynamic
# What is the best file format for Linux ReadMe:s?
# Does the ReadMe contain any instruction to install Qt?
cp ../Lasercake-0.23-clean/README.markdown Lasercake-0.23-linux-[arch]-dynamic
tar -czf Lasercake-0.23-linux-[arch]-dynamic.tar.gz Lasercake-0.23-linux-[arch]-dynamic


#### Windows (cross-compiled from Linux)

git clone https://github.com/Lasercake/Lasercake.git Lasercake-0.23-clean
mkdir Lasercake-0.23-build-win32
cd Lasercake-0.23-build-win32
cmake ../Lasercake-0.23-clean
  -DCMAKE_TOOLCHAIN_FILE=cmake/Toolchain-ArchLinux-mingw32.cmake
  [plus above optimization flags]
make -j3
mkdir Lasercake-0.23-win32
mv Lasercake.exe Lasercake-0.23-win32
cp /usr/i486-mingw32/{bin/{zlib1.dll,libpng15-15.dll},lib/{libgcc_s_sjlj-1.dll,libstdc++-6.dll,QtCore4.dll,QtGui4.dll,QtOpenGL4.dll}} Lasercake-0.23-win32
cp ../Lasercake-0.23-clean/resources/ReadMe.rtf Lasercake-0.23-win32
zip -r Lasercake-0.23-win32.zip Lasercake-0.23-win32


#### Source

git clone https://github.com/Lasercake/Lasercake.git Lasercake-0.23-source

# TODO: figure something out regarding Windows line endings.
zip -r Lasercake-0.23-source.zip Lasercake-0.23-source
tar -czf Lasercake-0.23-source.tar.gz Lasercake-0.23-source
tar -cJf Lasercake-0.23-source.tar.xz Lasercake-0.23-source

cp -a Lasercake-0.23-source Lasercake-0.23-source-minimal
rm -rf Lasercake-0.23-source-minimal/{.git,bundled-libs}

zip -r Lasercake-0.23-source-minimal.zip Lasercake-0.23-source-minimal
tar -czf Lasercake-0.23-source-minimal.tar.gz Lasercake-0.23-source-minimal
tar -cJf Lasercake-0.23-source-minimal.tar.xz Lasercake-0.23-source-minimal


### process ###

1: Git tag a release candidate.
git tag -u 17062391 Lasercake-[version]-rcN -m'Lasercake-[version]-rcN'
git push --tags
2: Build binaries; upload them; get them tested on several platforms.
3: If there are any problems, repeat starting at step 1
4: Otherwise:
5: Update the minor version from odd to even (even minor == release).
6: git tag and rebuild with this new non-rc version number,
7: upload that, update the website, etc.
8. Update the version number in git to the next odd number to indicate dev
   version; even numbers are releases.
