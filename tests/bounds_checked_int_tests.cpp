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

#include "../bounds_checked_int.hpp"

//#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_MODULE bounds_checked_int test
#include <boost/test/unit_test.hpp>

typedef bounds_checked_int<uint32_t> u32;
typedef bounds_checked_int<int32_t> i32;
typedef bounds_checked_int<uint64_t> u64;
typedef bounds_checked_int<int64_t> i64;

BOOST_AUTO_TEST_CASE( boundses ) {
  {
    bounds_checked_int<uint32_t> i(0);
    BOOST_CHECK_THROW(--i, std::logic_error);
    BOOST_CHECK_THROW(i + u32(1) + u32(0xffffffff), std::logic_error);
    BOOST_CHECK_EQUAL(i + u32(0xffffffff), u32(0xffffffff));
    BOOST_CHECK_EQUAL(-- ++i, u32(0));
    BOOST_CHECK(!i);
    BOOST_CHECK_EQUAL(i += u32(7), u32(7));
    BOOST_CHECK(i);
  }

  {
    bounds_checked_int<int64_t> i = i64(std::numeric_limits<int64_t>::min());
    BOOST_CHECK_THROW(--i, std::logic_error);
    BOOST_CHECK_THROW(-i, std::logic_error);
    BOOST_CHECK_THROW(i / i64(-1), std::logic_error);

//    BOOST_CHECK_EQUAL(u32(1) + u64(2), u64(3));
//    BOOST_CHECK_EQUAL(u64(1) + u32(2), u64(3));
    BOOST_CHECK_EQUAL(u64(0xffffffffull) * u64(0x100000001ull), u64(std::numeric_limits<uint64_t>::max()));
    BOOST_CHECK_THROW(u64(0xffffffffull) * u64(0x100000002ull), std::logic_error);
    BOOST_CHECK_THROW(u64(0xffffffffull) * u64(0x100000002ull), std::logic_error);

    typedef bounds_checked_int<int32_t, -10, 10> tenz;
    tenz p(9);
    BOOST_CHECK_THROW(p*tenz(2), std::logic_error);
    BOOST_CHECK_EQUAL(p/tenz(-1), tenz(-9));
    BOOST_CHECK_THROW(p/tenz(-1) - tenz(2), std::logic_error);
  }
  
  //test not convertable signed/unsigned or wrong way somehow? boost is_convertible?
}