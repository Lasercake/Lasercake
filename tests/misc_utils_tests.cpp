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
#include <boost/test/parameterized_test.hpp>

#include "../utils.hpp"
#include <limits>
#include <array>
#include <algorithm>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>

namespace /* anonymous */ {

// Test case from https://svn.boost.org/trac/boost/ticket/6189
class MGen {
public:
    typedef boost::mt19937::result_type result_type;
    result_type min() const { return impl_.min(); }
    result_type max() const { return impl_.max(); }
    result_type operator()() { return 2114502989; }
private:
    boost::mt19937 impl_;
};
BOOST_AUTO_TEST_CASE( boost_before_1_50_bug ) {
  MGen gen;
  const int64_t val = boost::uniform_int<int64_t>(-50, 50)(gen);
  BOOST_CHECK_GE(val, -50);
  BOOST_CHECK_LE(val, 50);
}

int32_t popcount_obvious(uint64_t number, size_t bits) {
  size_t count = 0;
  for(size_t i = 0; i != bits; ++i) {
    count += bool(number & (uint64_t(1) << i));
  }
  return count;
}

void popcount_test(uint64_t number) {
  BOOST_CHECK_EQUAL(popcount(uint64_t(number)), popcount_obvious(number, 64));
  BOOST_CHECK_EQUAL(popcount(uint32_t(number)), popcount_obvious(number, 32));
  BOOST_CHECK_EQUAL(popcount(uint16_t(number)), popcount_obvious(number, 16));
  BOOST_CHECK_EQUAL(popcount(uint8_t(number)), popcount_obvious(number, 8));
}

BOOST_AUTO_TEST_CASE( my_popcount ) {
  BOOST_CHECK_EQUAL(popcount(uint64_t(0xef)), 7);
  BOOST_CHECK_EQUAL(popcount(uint32_t(0xef)), 7);
  BOOST_CHECK_EQUAL(popcount(uint16_t(0xef)), 7);
  BOOST_CHECK_EQUAL(popcount(uint8_t(0xef)), 7);

  std::array<uint64_t, 20> numbers_to_test = {{ 0, 1, 2, 3, 4, 5, 17, 232, 500,
    78978978, 8948954789789349789ull, 0xfffffffful, 0x100000000ull,
    0x100000001ull, 0xffffffffffffffffull, 0xfffffffffffffffeull,
    0xeeeeeeeeeeeeeeeeull, 0xfffffffe00000001ull, 0xfffffffe00000000ull,
    0xffffffffffffffffull
  }};
  std::for_each(numbers_to_test.begin(), numbers_to_test.end(), &popcount_test);
}

void i64sqrt_test(uint64_t radicand) {
  uint64_t sqrt_result = i64sqrt(radicand); //implicit cast the result to 64 bits so we can square it
  BOOST_CHECK_LE(sqrt_result * sqrt_result, radicand);
  if(sqrt_result != std::numeric_limits<uint32_t>::max()) {
    BOOST_CHECK_GT((sqrt_result+1) * (sqrt_result+1), radicand);
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


void i64log2_test(uint64_t argument) {
  uint64_t log_result = ilog2(argument);
  BOOST_CHECK_LE(1ull << log_result, argument);
  if(log_result != 63) {
    BOOST_CHECK_GT(1ull << (log_result+1), argument);
  }
}

BOOST_AUTO_TEST_CASE( my_log ) {
  BOOST_CHECK_THROW(ilog2(0u), std::logic_error);
  std::array<uint64_t, 18> numbers_to_test = {{ 1, 2, 3, 4, 5, 17, 232, 500,
    78978978, 8948954789789349789ull, 0xfffffffful, 0x100000000ull,
    0x100000001ull, 0xffffffffffffffffull, 0xfffffffffffffffeull,
    0xeeeeeeeeeeeeeeeeull, 0xfffffffe00000001ull, 0xfffffffe00000000ull
  }};
  std::for_each(numbers_to_test.begin(), numbers_to_test.end(), &i64log2_test);
}

BOOST_AUTO_TEST_CASE( my_ctz ) {
  BOOST_CHECK_THROW(count_trailing_zeroes_64(0), std::logic_error);
  BOOST_CHECK_EQUAL(count_trailing_zeroes_64(0xe), 1);
  BOOST_CHECK_EQUAL(count_trailing_zeroes_64(0xf), 0);
  BOOST_CHECK_EQUAL(count_trailing_zeroes_64(0xf0), 4);
  BOOST_CHECK_EQUAL(count_trailing_zeroes_64(0xe0), 5);
  BOOST_CHECK_EQUAL(count_trailing_zeroes_64(0x8000000000000000ull), 63);
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

BOOST_AUTO_TEST_CASE( vector3_tests ) {
  // more would be better

  {
    std::stringstream ss;
    ss << vector3<int>(-99,0,100);
    BOOST_CHECK_EQUAL(ss.str(), "(-99, 0, 100)");
  }
  {
    std::stringstream ss;
    ss << vector3<bounds_checked_int<int>>(-99,0,100);
    BOOST_CHECK_EQUAL(ss.str(), "(-99, 0, 100)");
  }
}

static_assert(static_pow_nonnegative_integer<9,7>::value == 4782969, "bug");
static_assert(static_pow_nonnegative_integer<9,0>::value == 1, "bug");
static_assert(static_pow_nonnegative_integer<0,1>::value == 0, "bug");
static_assert(static_pow_nonnegative_integer<0,0>::value == 1, "this is generally useful");
static_assert(static_pow_nonnegative_integer<1,999999999>::value == 1, "bug");
static_assert(static_pow_nonnegative_integer<999999999,1>::value == 999999999, "bug");
static_assert(static_pow_nonnegative_integer<2,32>::value == 0x100000000ull, "bug");

static_assert(static_root_nonnegative_integer<9,2>::value == 3, "bug");
static_assert(static_root_nonnegative_integer<9,2>::remainder == 0, "bug");
static_assert(static_root_nonnegative_integer<10,2>::value == 3, "bug");
static_assert(static_root_nonnegative_integer<10,2>::remainder == 1, "bug");
static_assert(static_root_nonnegative_integer<8,2>::value == 2, "bug");
static_assert(static_root_nonnegative_integer<8,2>::remainder == 4, "bug");
static_assert(static_root_nonnegative_integer<27,3>::value == 3, "bug");
static_assert(static_root_nonnegative_integer<243,5>::value == 3, "bug");
static_assert(static_root_nonnegative_integer<500,5>::remainder == 500-243, "bug");
static_assert(static_root_nonnegative_integer<999999999,1>::value == 999999999, "bug");
//static_assert(static_root_nonnegative_integer<1,999999999>::value == 1, "bug");
static_assert(static_root_nonnegative_integer<0,3>::value == 0, "bug");


}  /* end anonymous namespace */
