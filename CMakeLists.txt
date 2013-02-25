cmake_minimum_required (VERSION 2.6)
project (Lasercake)

set (Lasercake_VERSION_MAJOR 0)
set (Lasercake_VERSION_MINOR 1)

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
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DUSE_BOOST_CXX11_LIBS=1 -U__GXX_EXPERIMENTAL_CXX0X__)
endif()
set(CXX11_LIB_WRAPPERS_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/bundled_libs/cxx11/")


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
	set(ESSENTIAL_FLAGS -std=gnu++0x)
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
	set(ESSENTIAL_FLAGS -std=c++0x)
	set(SECURITY_FLAGS -fwrapv -D_FORTIFY_SOURCE=2 -fstack-protector --param=ssp-buffer-size=4)
endif()
set(OUR_CXX_FLAGS ${OPT_FLAGS} ${WARNING_FLAGS} ${SECURITY_FLAGS} ${OPTIONS_DEFINES} ${ESSENTIAL_FLAGS})
set(THEIR_CXX_FLAGS_spaces ${CMAKE_CXX_FLAGS})
semicolons_to_spaces(OUR_CXX_FLAGS_spaces ${OUR_CXX_FLAGS})
set(CMAKE_CXX_FLAGS "${OUR_CXX_FLAGS_spaces} ${THEIR_CXX_FLAGS_spaces}")

# certain bundled libs are C files so we need to set their C flags,
# not just assume the whole world uses C++ flags
set(OUR_C_FLAGS ${OPT_FLAGS} ${WARNING_FLAGS} ${SECURITY_FLAGS} ${OPTIONS_DEFINES})
set(THEIR_C_FLAGS_spaces ${CMAKE_C_FLAGS})
semicolons_to_spaces(OUR_C_FLAGS_spaces ${OUR_C_FLAGS})
set(CMAKE_C_FLAGS "${OUR_C_FLAGS_spaces} ${THEIR_C_FLAGS_spaces}")

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
message(STATUS "lasercake link flags: ${LASERCAKE_LINK_FLAGS_spaces}")
message(STATUS "lasercake sources: ${LASERCAKE_SOURCES_pretty}")
add_executable (lasercake WIN32 ${LASERCAKE_GLEW_SOURCES} ${LASERCAKE_BOOST_SOURCES} ${LASERCAKE_SOURCES} ${LASERCAKE_MOC} ${LASERCAKE_RESOURCES_RCC})
target_link_libraries(lasercake ${LASERCAKE_LINK_FLAGS})

if(BUILD_SELF_TESTS)
	enable_testing()
	add_test(TestPasses lasercake --run-self-tests)
endif()

