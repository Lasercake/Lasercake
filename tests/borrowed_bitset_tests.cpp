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

#include "test_header.hpp"

#include "../data_structures/borrowed_bitset.hpp"

BOOST_AUTO_TEST_CASE( test_borrowed_bitset ) {
  {
    borrowed_bitset b700(700);
    BOOST_CHECK_EQUAL(b700.test(699), false);
    BOOST_CHECK_NO_THROW(b700.set(699));
    BOOST_CHECK_EQUAL(b700.test(699), true);
    BOOST_CHECK_EQUAL(b700.test(698), false);
    BOOST_CHECK_EQUAL(b700.test(0), false);
    BOOST_CHECK_EQUAL(b700.test(1), false);
    BOOST_CHECK_NO_THROW(b700.set(699));
    BOOST_CHECK_EQUAL(b700.test(699), true);
    BOOST_CHECK_EQUAL(b700.test(698), false);
    BOOST_CHECK_EQUAL(b700.test(0), false);
    BOOST_CHECK_EQUAL(b700.test(1), false);
    BOOST_CHECK_NO_THROW(b700.set(0));
    BOOST_CHECK_EQUAL(b700.test(699), true);
    BOOST_CHECK_EQUAL(b700.test(698), false);
    BOOST_CHECK_EQUAL(b700.test(0), true);
    BOOST_CHECK_EQUAL(b700.test(1), false);
  }
  {
    borrowed_bitset b700(700);
    BOOST_CHECK_EQUAL(b700.test(699), false);
    BOOST_CHECK_EQUAL(b700.test(698), false);
    BOOST_CHECK_EQUAL(b700.test(0), false);
    BOOST_CHECK_EQUAL(b700.test(1), false);
  }
  {
    borrowed_bitset b700_1(700);
    borrowed_bitset b700_2(700);
    BOOST_CHECK_NO_THROW(b700_1.set(120));
    BOOST_CHECK_NO_THROW(b700_2.set(121));
    BOOST_CHECK_EQUAL(b700_1.test(120), true);
    BOOST_CHECK_EQUAL(b700_2.test(120), false);
    BOOST_CHECK_EQUAL(b700_1.test(121), false);
    BOOST_CHECK_EQUAL(b700_2.test(121), true);
    borrowed_bitset b0(0);
    BOOST_CHECK_EQUAL(b700_2.test(121), true);
    borrowed_bitset b1(1);
    BOOST_CHECK_EQUAL(b1.test(0), false);
    BOOST_CHECK_NO_THROW(b1.set(0));
    BOOST_CHECK_EQUAL(b1.test(0), true);
  }
  // no multi-threading tests at the moment.
  // the best they could do is check for race conditions anyway
  // (well, which one of the valgrind tools could check for)
}
