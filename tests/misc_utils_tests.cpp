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

#include "../utils.hpp"
#include <limits>
#include <array>
#include <algorithm>

//#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_MODULE utils test
#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>

namespace /* anonymous */ {

void i64sqrt_test(uint64_t radicand) {
  uint64_t sqrt_result = i64sqrt(radicand); //implicit cast the result to 64 bits so we can square it
  BOOST_CHECK(sqrt_result * sqrt_result <= radicand);
  if(sqrt_result != std::numeric_limits<uint32_t>::max()) {
    BOOST_CHECK((sqrt_result+1) * (sqrt_result+1) > radicand);
  }
}


BOOST_AUTO_TEST_CASE( my_sqrt ) {
  std::array<uint64_t, 19> numbers_to_test = {{ 0, 1, 2, 3, 4, 5, 17, 232, 500,
    78978978, 8948954789789349789ull, 0xfffffffful, 0x100000000ull,
    0x100000001ull, 0xffffffffffffffffull, 0xfffffffffffffffeull,
    0xeeeeeeeeeeeeeeeeull, 0xfffffffe00000001ull, 0xfffffffe00000000ull
  }};
  std::for_each(numbers_to_test.begin(), numbers_to_test.end(), &i64sqrt_test);
}

BOOST_AUTO_TEST_CASE( divide_rounding_towards_zero_test ) {
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(3,1), 3);
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(3,2), 1);
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(0,3), 0);
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(0,-3), 0);
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(-3,3), -1);
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(-3,2), -1);
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(3,-2), -1);
  BOOST_CHECK_EQUAL(divide_rounding_towards_zero(-3,-2), 1);
  BOOST_CHECK_THROW(divide_rounding_towards_zero(-3,0), std::logic_error);
}

BOOST_AUTO_TEST_CASE( non_normalized_rational_test ) {
  typedef non_normalized_rational<int32_t> rational;
  BOOST_CHECK_EQUAL(rational(3,6), rational(2,4));
  BOOST_CHECK(rational(3,6) != rational(3,5));
  BOOST_CHECK(rational(3,6) < rational(3,5));
  BOOST_CHECK(rational(3,6) <= rational(3,5));
  BOOST_CHECK(rational(3,5) > rational(3,6));
  BOOST_CHECK(rational(3,5) >= rational(3,6));
  BOOST_CHECK(rational(3,6) <= rational(2,4));
  BOOST_CHECK(rational(3,6) >= rational(2,4));
  
  BOOST_CHECK_EQUAL(rational(0,1), rational(0,2));
  BOOST_CHECK_EQUAL(rational(0,2), rational(0,-1));
  BOOST_CHECK_EQUAL(rational(-3,-6), rational(2,4));
  BOOST_CHECK_EQUAL(rational(3,-6), rational(2,-4));
  BOOST_CHECK_EQUAL(rational(3,-6), rational(-2,4));
  BOOST_CHECK(rational(3,6) != rational(3,5));
  BOOST_CHECK(rational(3,-6) < rational(3,5));
  BOOST_CHECK(rational(3,6) >= rational(-3,5));
  BOOST_CHECK(rational(3,-5) < rational(3,-6));
  BOOST_CHECK(rational(-100,3) >= rational(-1000,4));
  BOOST_CHECK(rational(-3,-6) <= rational(2,4));
  BOOST_CHECK(rational(-3,-6) >= rational(-2,4));
}

BOOST_AUTO_TEST_CASE( signs_test ) {
  BOOST_CHECK_EQUAL(sign(2), 1);
  BOOST_CHECK_EQUAL(sign(-2), -1);
  BOOST_CHECK_EQUAL(sign(0), 0);
  BOOST_CHECK_EQUAL(sign(std::numeric_limits<int32_t>::min()), -1);

  BOOST_CHECK_EQUAL(is_negative(2), false);
  BOOST_CHECK_EQUAL(is_negative(-2), true);
  BOOST_CHECK_EQUAL(is_negative(0), false);
  BOOST_CHECK_EQUAL(is_negative(std::numeric_limits<int32_t>::min()), true);
}

}  /* end anonymous namespace */
