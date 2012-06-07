cmake_minimum_required (VERSION 2.6)
project (Lasercake)

set (Lasercake_VERSION_MAJOR 0)
set (Lasercake_VERSION_MINOR 1)

find_package(Boost COMPONENTS thread)
find_package(SDL)
find_package(OpenGL)

include_directories(${SDL_INCLUDE_DIR} ${BOOST_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})

set(OPT_FLAGS "-O3")
set(WARNING_FLAGS "-Wall -Wextra -Wno-unknown-pragmas -fmax-errors=15")
set(SECURITY_FLAGS "-fwrapv -fstack-protector --param=ssp-buffer-size=4 -D_FORTIFY_SOURCE=2")
set(CMAKE_CXX_FLAGS "${OPT_FLAGS} ${WARNING_FLAGS} ${SECURITY_FLAGS} -std=c++0x ${CMAKE_CXX_FLAGS}")

file (GLOB LASERCAKE_SOURCES *.cpp *.hpp)
add_executable (lasercake ${LASERCAKE_SOURCES})
target_link_libraries(lasercake ${SDL_LIBRARY} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} rt)

find_package(Boost COMPONENTS unit_test_framework)
file (GLOB_RECURSE TEST_SOURCES *.hpp tests/*.cpp)
add_executable (test-lasercake ${TEST_SOURCES})
target_link_libraries(test-lasercake ${Boost_LIBRARIES} rt)

enable_testing()
add_test(TestPasses test-lasercake)

