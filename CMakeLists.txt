cmake_minimum_required (VERSION 2.6)
project (Lasercake)

# TODO #define these and emit them on --help/--version.
set (Lasercake_VERSION_MAJOR 0)
set (Lasercake_VERSION_MINOR 22)
set (Lasercake_VERSION_PATCH 0)
# TODO if in git, add part of the git hash to the version desc?
# TODO is it useful to include the year in any version-related
# strings?
set (Lasercake_version_desc ${Lasercake_VERSION_MAJOR}.${Lasercake_VERSION_MINOR}.${Lasercake_VERSION_PATCH})


set (OPTIONS_DEFINES )
option (USE_BUNDLED_BOOST "On: Use Boost from ./bundled_libs/; off: use system Boost." ON)
option (USE_BUNDLED_GLM "On: Use glm from ./bundled_libs/; off: use system glm." ON)
option (USE_BUNDLED_GLEW "On: Use GLEW from ./bundled_libs/; off: use system GLEW." ON)
option (USE_BOOST_CXX11_LIBS "On: Use Boost's implementation of C++11 standard library features; off: use the standard library or autodetect that using Boost's is necessary." OFF)
option (GLIB "Use GLib slices for some memory allocation (requires glib)." OFF)
option (USE_BOUNDS_CHECKED_INTS "Check for overflow of most int computations at runtime (slow)." OFF)
option (PARANOID "Turn on various code paranoia checks (slow)." OFF)
option (_GLIBCXX_DEBUG "Defines _GLIBCXX_DEBUG iterator-validity debugging (slow)." OFF)
option (BUILD_SELF_TESTS "Build Lasercake's self-testing code component (recommended)." ON)

# These are merely shorthand for CMake options that I use for
# releases on various systems. -Isaac
option (LTO "Use link-time optimization (roughly halves the size of the binary)" OFF)
option (OSX_TARGET_10_5 "OS X: make binary run on 10.5 even if building on later OS X" OFF)


include (FindPkgConfig)
set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_SOURCE_DIR}/cmake")

include(CheckCXXSourceCompiles)


check_cxx_source_compiles("#include<utility>\nint main(){return 0;}" COMPILER_SUPPORTS_CXX)
set(CMAKE_REQUIRED_FLAGS -std=c++0x)
# constexpr is a not perfect but pretty good proxy for supporting our code...
# if this doesn't compile, our code definitely won't.
check_cxx_source_compiles("constexpr int i = 3; int main(){int arr[i]={};return arr[1];}" COMPILER_SUPPORTS_CXX11)
check_cxx_source_compiles("#include<array>\n#include<atomic>\nint main(){std::array<int,1> arr = {{}};std::atomic<int> atom(0); return atom.load();}" COMPILER_SUPPORTS_CXX11_STDLIB)
check_cxx_source_compiles("thread_local void* tls; int main(){return(tls==0);}" HAS_THREAD_LOCAL)
check_cxx_source_compiles("__thread void* tls; int main(){return(tls==0);}" HAS___THREAD)
set(CMAKE_REQUIRED_FLAGS )

# check COMPILER_SUPPORTS_CXX just so that we don't give the C++11 error message
# if they actually just have a completely broken C++ compiler.
if(NOT COMPILER_SUPPORTS_CXX)
	message(FATAL_ERROR "Your C++ compiler is broken.")
endif()
if(NOT COMPILER_SUPPORTS_CXX11)
	message(FATAL_ERROR "Your compiler doesn't support the C++11 features Lasercake requires.\n(Or it was released since we wrote this message in Feb 2013 and has a way to enable C++11 features that we haven't implemented yet; if so, tell us)")
endif()

if(HAS___THREAD AND NOT HAS_THREAD_LOCAL)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -Dthread_local=__thread)
endif()
if(NOT HAS___THREAD AND NOT HAS_THREAD_LOCAL)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DNO_COMPILER_SUPPORTED_TLS=1)
endif()

if (USE_BOOST_CXX11_LIBS OR NOT COMPILER_SUPPORTS_CXX11_STDLIB)
	# We undef __GXX_EXPERIMENTAL_CXX0X__ so as to more effectively
	# test USE_BOOST_CXX11_LIBS on our dev machines.
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DUSE_BOOST_CXX11_LIBS=1 -U__GXX_EXPERIMENTAL_CXX0X__ -include "${CMAKE_SOURCE_DIR}/cxx11/cxx11_utils.hpp")
endif()
set(CXX11_LIB_WRAPPERS_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/bundled_libs/cxx11/")

if (OSX_TARGET_10_5)
	find_file(OSX_10_5_SDK MacOSX10.5.sdk /Developer/SDKs/)
	if("${OSX_10_5_SDK}" STREQUAL "OSX_10_5_SDK-NOTFOUND")
		message(FATAL_ERROR "You asked to target OS X 10.5 but we couldn't find the MacOSX 10.5 SDK.")
	endif()
	set(CMAKE_OSX_DEPLOYMENT_TARGET 10.5)
	set(CMAKE_OSX_SYSROOT "${OSX_10_5_SDK}")
endif()

if (_GLIBCXX_DEBUG)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -D_GLIBCXX_DEBUG)
endif()
if (GLIB)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DLASERCAKE_USE_GLIB=1)
	find_package(Glib REQUIRED)
endif()

find_package(Qt4 4.8 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
find_package(OpenGL REQUIRED)

if (USE_BUNDLED_GLM)
	set(GLM_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/bundled_libs/glm/")
else()
	find_package(GLM REQUIRED)
endif()
if (USE_BUNDLED_GLEW)
	set(GLEW_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/bundled_libs/glm/")
	set(GLEW_LIBRARIES )
	set(LASERCAKE_GLEW_SOURCES ${CMAKE_SOURCE_DIR}/bundled_libs/glew/src/glew.c)
	set(GLEW_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/bundled_libs/glew/include/)
	set(OPTIONS_DEFINES ${OPTIONS_DEFINES} -DGLEW_STATIC)
else()
	find_package(GLEW REQUIRED)
	set(LASERCAKE_GLEW_SOURCES )
endif()

# BOOST_SYSTEM_NO_DEPRECATED prevents some deprecated members with global constructors.
# BOOST_CHRONO_HEADER_ONLY means we don't need the lib components that we're
#   not interested in anyway.
set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DBOOST_SYSTEM_NO_DEPRECATED -DBOOST_CHRONO_HEADER_ONLY)
if (USE_BUNDLED_BOOST)
	set (BOOST_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/bundled_libs/boostbcp/")
	set (BOOST_LIBRARIES )
	file (GLOB_RECURSE LASERCAKE_BOOST_SOURCES ${CMAKE_SOURCE_DIR}/bundled_libs/boostbcp/libs/*/src/*.cpp)
	set_source_files_properties (${LASERCAKE_BOOST_SOURCES} PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter")
else()
	set(BOOST_COMPONENTS_WE_NEED program_options)
	find_package(Boost 1.50 COMPONENTS ${BOOST_COMPONENTS_WE_NEED} REQUIRED)
	set(LASERCAKE_BOOST_SOURCES )
endif()
if (NOT BUILD_SELF_TESTS)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DLASERCAKE_NO_SELF_TESTS=1)
endif()
if (USE_BOUNDS_CHECKED_INTS OR PARANOID)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DUSE_BOUNDS_CHECKED_INTS=1)
endif()
if (PARANOID)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DBBOX_COLLISION_DETECTOR_DEBUG=1 -DASSERT_EVERYTHING=1)
endif()


macro(semicolons_to_spaces output_var_name)
	string(REPLACE ";" " " "${output_var_name}" "${ARGN}")
endmacro()

#Usage: make_into_valid_varname(prefix_${some_text})
#Then use: ${prefix_${some_text}} where you need the validized value.
# Currently it just turns non-ASCII-alnum chars into underscore.
# We need this because in compile tests CMake passes -Dvarname
# to the compiler which, with gcc -Werror, errors for invalidly named
# varnames.
macro(make_into_valid_varname text)
	# Don't use [a-z] in case the locale is different than "C";
	# don't use [[:alnum:]] because non-ASCII letters aren't
	# valid in all identifiers; don't rely on the CMake regex
	# engine flouting any of the above.
	string(REGEX REPLACE "[^_abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789]" "_" "${text}" "${text}")
endmacro()
macro(show_paths output_var_name)
	semicolons_to_spaces(SHOW_PATHS_white_paths "${ARGN}")
	string(REPLACE "${CMAKE_SOURCE_DIR}/" "" "${output_var_name}" "${SHOW_PATHS_white_paths}")
endmacro()

set(OPT_FLAGS -O3)
set(OPT_LINK_FLAGS )
if (LTO)
	set(OPT_FLAGS ${OPT_FLAGS} -flto)
	set(OPT_LINK_FLAGS ${OPT_LINK_FLAGS} -fwhole-program)
endif()
set(WARNING_FLAGS )
include(CheckCXXCompilerFlag)
foreach(flag -g -Wall -Wextra -Wglobal-constructors -Wno-unknown-pragmas -Wno-unused-function -fmax-errors=5 -ferror-limit=5 -fdiagnostics-show-option)
	make_into_valid_varname("has_flag_${flag}")
	check_cxx_compiler_flag("-Werror ${flag}" ${has_flag_${flag}})
	if(${has_flag_${flag}})
		set(WARNING_FLAGS ${WARNING_FLAGS} ${flag})
	endif()
endforeach(flag)
if(WIN32)
	# MinGW has various problems with -std=c++0x.  For example,
	# the compiler with -std=c++0x defines __STRICT_ANSI__
	# and thus Windows-related headers don't define everything
	# that Boost headers expect them to.
	set(CMAKE_CXX_FLAGS "-std=gnu++0x ${CMAKE_CXX_FLAGS}")
	# Sadly my MinGW cross-compile leads to linking errors with
	# -fstack-protector.  TODO try things with the compiler rather
	# than assuming based on the target platform.  TODO can we
	# use -fstack-protector here somehow?  (it would be nice.)
	set(SECURITY_FLAGS -fwrapv -D_FORTIFY_SOURCE=2)
else()
	# Clang has trouble parsing GNU libstdc++ headers with -std=gnu++0x
	# because the 'gnu' (by leaving __STRICT_ANSI__ undefined)
	# enables various GCC extensions in headers like __float128 that
	# Clang doesn't understand.  (For some versions of clang and
	# libstdc++.)  Also, we don't need these GNU extensions.
	set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
	set(SECURITY_FLAGS -fwrapv -D_FORTIFY_SOURCE=2 -fstack-protector --param=ssp-buffer-size=4)
endif()
set(OUR_CXX_FLAGS ${OPT_FLAGS} ${WARNING_FLAGS} ${SECURITY_FLAGS} ${OPTIONS_DEFINES} ${ESSENTIAL_FLAGS})
semicolons_to_spaces(OUR_CXX_FLAGS_spaces ${OUR_CXX_FLAGS})

set(OUR_LINK_FLAGS
	${OPT_FLAGS} ${WARNING_FLAGS} ${SECURITY_FLAGS} ${OPTIONS_DEFINES} ${ESSENTIAL_FLAGS}
	${OPT_LINK_FLAGS}
)
semicolons_to_spaces(OUR_LINK_FLAGS_spaces ${OUR_LINK_FLAGS})

message(STATUS "compiler flags: ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_FLAGS}")

set(INCLUDE_DIRS ${BOOST_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${GLIB_INCLUDE_DIR} ${GLIB_CONFIG_INCLUDE_DIR} ${CXX11_LIB_WRAPPERS_INCLUDE_DIRS})
message(STATUS "qt defs: ${QT_DEFINITIONS}")
add_definitions(${QT_DEFINITIONS})
semicolons_to_spaces(INCLUDE_DIRS_spaces ${INCLUDE_DIRS})
message(STATUS "include dirs: ${INCLUDE_DIRS_spaces}")
include_directories(${INCLUDE_DIRS})

file (GLOB LASERCAKE_SOURCES *.cpp *.hpp data_structures/*.[ch]pp)
file (GLOB LASERCAKE_TEST_SOURCES tests/*.cpp)
if(BUILD_SELF_TESTS)
	set(LASERCAKE_SOURCES ${LASERCAKE_SOURCES} ${LASERCAKE_TEST_SOURCES})
endif()
file (GLOB LASERCAKE_HEADERS *.hpp data_structures/*.hpp tests/*.hpp)
qt4_wrap_cpp(LASERCAKE_MOC main.hpp)
qt4_add_resources(LASERCAKE_RESOURCES_RCC fonts.qrc)
set(LASERCAKE_LINK_FLAGS ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${Boost_LIBRARIES} ${GLIB_LIBRARIES})
semicolons_to_spaces(LASERCAKE_LINK_FLAGS_spaces ${LASERCAKE_LINK_FLAGS})
show_paths(LASERCAKE_SOURCES_pretty ${LASERCAKE_SOURCES})
message(STATUS "Lasercake link flags: ${LASERCAKE_LINK_FLAGS_spaces}")
message(STATUS "Lasercake sources: ${LASERCAKE_SOURCES_pretty}")
add_executable (Lasercake ${LASERCAKE_GLEW_SOURCES} ${LASERCAKE_BOOST_SOURCES} ${LASERCAKE_SOURCES} ${LASERCAKE_MOC} ${LASERCAKE_RESOURCES_RCC})
target_link_libraries(Lasercake ${LASERCAKE_LINK_FLAGS})
set_target_properties(Lasercake PROPERTIES
	COMPILE_FLAGS "${OUR_CXX_FLAGS_spaces}"
	LINK_FLAGS "${OUR_LINK_FLAGS_spaces}"

	# On Windows, add WinMain() to make this a GUI program:
	WIN32_EXECUTABLE ON

	# On Mac, GUI programs are put into a directory called a "bundle"
	# that behaves like a file to users (it contains an executable
	# and metadata like an icon).
	MACOSX_BUNDLE ON
	# What is the difference between these version keys?
	# Consult /usr/share/cmake-2.8/Modules/MacOSXBundleInfo.plist.in
	# to get the Apple versions of these keys (we can copy that file
	# and reference it in a MACOSX_BUNDLE_INFO_PLIST if we need to
	# change the template).
	MACOSX_BUNDLE_BUNDLE_VERSION "${Lasercake_version_desc}"
	MACOSX_BUNDLE_SHORT_VERSION_STRING "${Lasercake_version_desc}"
	#MACOSX_BUNDLE_LONG_VERSION_STRING "${Lasercake_version_desc}"
	MACOSX_BUNDLE_GUI_IDENTIFIER "net.lasercake.Lasercake"
	MACOSX_BUNDLE_BUNDLE_NAME "Lasercake"
	MACOSX_BUNDLE_INFO_STRING "Lasercake ${Lasercake_version_desc}, a free/libre/open-source game, http://lasercake.net/"
	# OS X Finder "Get Info" seems to display the info string
	# as "Version: ..." *or* the copyright as "Copyright: ...",
	# not both, so specify the one with the friendlier prefix
	# ("Version", i.e. the "info-string" one).
	#MACOSX_BUNDLE_COPYRIGHT "Free/libre/open-source AGPLv3 by Eli Dupree, Isaac Dupree, et al."
	# TODO: do we want to set <key>NSHighResolutionCapable</key> <true/> ?
	# Probably once we ensure that displayed text scales sensibly in
	# relation to window size.
	# Why does the template set LSRequiresCarbon? Aren't we using a Qt
	# build that is Cocoa-based?  Yet that setting doesn't seem to be
	# causing any harm?
)

if(APPLE)
	install (TARGETS Lasercake BUNDLE DESTINATION .)
	install (CODE "
		include(BundleUtilities)
		fixup_bundle(\"\${CMAKE_INSTALL_PREFIX}/Lasercake.app\" \"\" \"\")
	")
	install (FILES README.markdown DESTINATION . RENAME ReadMe.txt)
endif()
# Note: This only makes sense when doing out-of-tree builds;
# otherwise it might try to include the build contents and any other
# temporary files or uncommitted changes.
#Also, do we want to exclude bundled_libs/ and .git/ for binary
#distribution builds? Or how to decrease the download size?  We could include
#a URL to the source code in the README (perhaps a ReadMe.txt.in which
#fills in the version number automatically) which could point directly
#to a download as well as online browsing on a GitHub
#tree-specific-to-this-tag view, or to an online page that gives the option.
#install (DIRECTORY ${CMAKE_SOURCE_DIR}/ DESTINATION source-code)

if(BUILD_SELF_TESTS)
	enable_testing()
	add_test(TestPasses Lasercake --run-self-tests)
endif()


# This CPack stuff may or may not be functional.
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
set (CPACK_PACKAGE_VENDOR "re:calib - Eli Dupree and Isaac Dupree")
# TODO: auto-convert these to HTML or RTF or at least a .txt extension
# so they're opened in the right editors?
set (CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.markdown") #TODO: intro file shown by NSIS Windows installer, I think
set (CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.markdown") #TODO: a longer(?) readme shown by some installers
set (CPACK_RESOURCE_FILE_WELCOME "${CMAKE_SOURCE_DIR}/README.markdown") #TODO: what is the difference between all these readme-ish files?
#set (CPACK_MONOLITHIC_INSTALL 1) #since we just have one thing to package; do we need to say this?
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "Lasercake is an open-world game about the environment")
set (CPACK_PACKAGE_EXECUTABLES Lasercake;Lasercake)
# are these redundant/necessary?
set (CPACK_PACKAGE_VERSION_MAJOR ${Lasercake_VERSION_MAJOR})
set (CPACK_PACKAGE_VERSION_MINOR ${Lasercake_VERSION_MINOR})
set (CPACK_PACKAGE_VERSION_PATCH ${Lasercake_VERSION_PATCH})

# why show a license; it's not required by law or custom with FOSS software
# set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING.txt")
set (CPACK_PACKAGE_INSTALL_DIRECTORY "Lasercake-${Lasercake_version_desc}")

#set (CPACK_BUNDLE_NAME "Lasercake")

#file(READ .gitignore gitignore_contents)
#string(REPLACE "." "\\\\." gitignore_contents_regexified1 ${gitignore_contents})
#string(REPLACE "*" ".*" gitignore_contents_regexified2 ${gitignore_contents_regexified1})
#string(REPLACE "\n" ";" gitignore_contents_list ${gitignore_contents_regexified2})
#set(CPACK_SOURCE_IGNORE_FILES ${gitignore_contents_list})
#Never mind, just make source from a completely clean tree.

#set(CPACK_GENERATOR "TGZ")
set(CPACK_SOURCE_GENERATOR "ZIP;TGZ")

include (CPack)

