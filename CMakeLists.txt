cmake_minimum_required (VERSION 2.6)
project (Lasercake)

set (Lasercake_VERSION_MAJOR 0)
set (Lasercake_VERSION_MINOR 1)

set (OPTIONS_DEFINES )
# (option THREADS TODO: does anyone need this to compile against a Qt that has threads disabled?
# Could be permitted, with a bit of work.  Do you have a way to test compiling that configuration?)
option (USE_BOOSTBCP "On: Use Boost from ./boostbcp/; off: use system Boost." ON)
option (THREADS "On: Use Qt multithreading for parallelism; off: don't link/use threads." ON)
option (TIMING "On: Use Boost.Chrono to count performance (e.g. FPS); off: don't count perf." ON)
option (GLIB "Use GLib slices for some memory allocation (faster; requires glib)." ON)
option (USE_BOUNDS_CHECKED_INTS "Check for overflow of most int computations at runtime (slow)." OFF)
option (PARANOID "Turn on various code paranoia checks (slow)." OFF)

if (GLIB)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DLASERCAKE_USE_GLIB=1)
	include (FindPkgConfig)
	set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_SOURCE_DIR}/cmake")
	find_package(Glib)
endif()

find_package(Qt4 4.8 COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})
find_package(OpenGL)

if (USE_BOOSTBCP)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DLASERCAKE_USE_BOOSTBCP=1)
	# because Boost.Chrono requires -lrt on Linux; sighs:
	# (TODO use a compile-test instead.)
	if(WIN32 OR APPLE)
		set (Boost_LIBRARIES )
	else()
		set (Boost_LIBRARIES rt)
	endif()
	set (BOOST_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/boostbcp/")
	file (GLOB_RECURSE LASERCAKE_BOOST_SOURCES ${CMAKE_SOURCE_DIR}/boostbcp/libs/*/src/*.cpp)
	set_source_files_properties (${LASERCAKE_BOOST_SOURCES} PROPERTIES COMPILE_FLAGS "-w")
else()
	find_package(Boost COMPONENTS program_options)
	set(LASERCAKE_BOOST_SOURCES )
endif()
if (NOT THREADS)
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DLASERCAKE_NO_THREADS=1)
endif()
if (TIMING)
	if (NOT USE_BOOSTBCP)
		find_package(Boost COMPONENTS system chrono)
	endif()
else()
	set (OPTIONS_DEFINES ${OPTIONS_DEFINES} -DLASERCAKE_NO_TIMING=1)
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
macro(show_paths output_var_name)
	semicolons_to_spaces(SHOW_PATHS_white_paths "${ARGN}")
	string(REPLACE "${CMAKE_SOURCE_DIR}/" "" "${output_var_name}" "${SHOW_PATHS_white_paths}")
endmacro()

set(OPT_FLAGS -O3)
set(WARNING_FLAGS -Wall -Wextra -Wno-unknown-pragmas -fmax-errors=15 -fdiagnostics-show-option)
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

message(STATUS "compiler flags: ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_FLAGS}")

set(INCLUDE_DIRS ${BOOST_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLIB_INCLUDE_DIR} ${GLIB_CONFIG_INCLUDE_DIR})
message(STATUS "qt defs: ${QT_DEFINITIONS}")
add_definitions(${QT_DEFINITIONS})
semicolons_to_spaces(INCLUDE_DIRS_spaces ${INCLUDE_DIRS})
message(STATUS "include dirs: ${INCLUDE_DIRS_spaces}")
include_directories(${INCLUDE_DIRS})

file (GLOB LASERCAKE_SOURCES *.cpp *.hpp data_structures/*.[ch]pp)
file (GLOB LASERCAKE_HEADERS *.hpp data_structures/*.hpp)
qt4_wrap_cpp(LASERCAKE_MOC main.hpp) #${LASERCAKE_HEADERS})
set(LASERCAKE_LINK_FLAGS ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${GLIB_LIBRARIES})
semicolons_to_spaces(LASERCAKE_LINK_FLAGS_spaces ${LASERCAKE_LINK_FLAGS})
show_paths(LASERCAKE_SOURCES_pretty ${LASERCAKE_SOURCES})
message(STATUS "lasercake link flags: ${LASERCAKE_LINK_FLAGS_spaces}")
message(STATUS "lasercake sources: ${LASERCAKE_SOURCES_pretty}")
add_executable (lasercake WIN32 ${LASERCAKE_BOOST_SOURCES} ${LASERCAKE_SOURCES} ${LASERCAKE_MOC})
target_link_libraries(lasercake ${LASERCAKE_LINK_FLAGS})

if (NOT USE_BOOSTBCP)
	find_package(Boost COMPONENTS unit_test_framework)
endif()
file (GLOB TEST_SOURCES data_structures/borrowed_bitset.cpp tests/*.cpp)
set(TEST_LINK_FLAGS ${Boost_LIBRARIES})
semicolons_to_spaces(TEST_LINK_FLAGS_spaces ${TEST_LINK_FLAGS})
show_paths(TEST_SOURCES_pretty ${TEST_SOURCES})
message(STATUS "test link flags: ${TEST_LINK_FLAGS_spaces}")
message(STATUS "test sources: ${TEST_SOURCES_pretty}")
add_executable (test-lasercake ${LASERCAKE_BOOST_SOURCES} ${TEST_SOURCES})
target_link_libraries(test-lasercake ${TEST_LINK_FLAGS})

enable_testing()
add_test(TestPasses test-lasercake)

