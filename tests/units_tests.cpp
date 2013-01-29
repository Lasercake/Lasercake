/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

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

#include "test_header.hpp"

#include "../units.hpp"

typedef units<boost::ratio<1>, 0, 1, 0, 0, 0, 0> meter;
constexpr meter meters = meter();
typedef units<boost::ratio<1>, 0, 0, 1, 0, 0, 0> gram;
typedef units<boost::ratio<1000>, 0, 0, 1, 0, 0, 0> kilogram;
typedef units<boost::ratio<1>, 0, 0, 0, 1, 0, 0> second;
typedef units<boost::ratio<1, 360>, 1, 0, 0, 0, 0, 0> degree;

BOOST_AUTO_TEST_CASE( unitses ) {
  unit<int32_t, meter> foo = 1 * meter();
  unit<int32_t, meter> foo1 = 3 * meter();
  unit<int32_t, meter> foo2 = 5 * meters;
  foo + foo;
  auto bfoo = foo * foo;
  bfoo = bfoo * 3;

  // Deliberately avoid get_primitive_int() to make sure
  // that it is returning the correct type in both cases.
#if USE_BOUNDS_CHECKED_INTS
  int yay = (bfoo / foo / foo).get();
#else
  int yay = (bfoo / foo / foo);
#endif

  volatile auto what = bfoo;

  BOOST_CHECK_EQUAL(foo2 + foo1, 8*meters);
  BOOST_CHECK_EQUAL(foo2 - foo1, 2*meters);
  BOOST_CHECK_EQUAL(foo2 / foo1, 1);
  BOOST_CHECK_EQUAL(foo2 * foo1, 15*meters*meters);
}
