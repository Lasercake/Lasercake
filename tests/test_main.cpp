/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_USE_BOOSTBCP
#define BOOST_TEST_DYN_LINK
#endif
#include "test_header.hpp"
#include "test_main.hpp"

// boost::unit_test::unit_test_main's first argument is one of two different
// possible function pointer types, depending on compile options.  Luckily
// C++ allows overloading of &overloaded_function when the result is passed
// to something that requires a specific function pointer type.
bool init_unit_test_suite() {
  return true;
}
boost::unit_test::test_suite* init_unit_test_suite(int, char**) {
  return nullptr;
}

int lasercake_test_main(int argc, char *argv[]) {
  return boost::unit_test::unit_test_main(&init_unit_test_suite, argc, argv);
}
