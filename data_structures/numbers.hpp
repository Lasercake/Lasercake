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

#ifndef LASERCAKE_NUMBER_STRUCTURES_HPP__
#define LASERCAKE_NUMBER_STRUCTURES_HPP__

#include "../config.hpp"
#include <numeric>
#include <boost/integer/static_log2.hpp>

// returns the signum (-1, 0, or 1)
template <typename Number>
inline Number sign(Number n) {
  return (n > 0) - (n < 0);
}

template <typename Number>
inline bool is_negative(Number n) {
  // this is (very slightly) the fastest comparison-with-zero among < 0, <= 0, > 0, >= 0:
  // in signed two's complement, it's the value of the highest bit:
  // which leads to one less x86 instruction (small code = more fits in cache).
  // (Once inlined and optimized in context, the chosen comparison might make no difference.)
  return (n < 0);
}

template<typename ScalarType1, typename ScalarType2>
auto divide_rounding_towards_zero(ScalarType1 dividend, ScalarType2 divisor)
  -> decltype(dividend/divisor) /*C++11 syntax for return type that lets it refer to argument names*/
{
  caller_correct_if(divisor != 0, "divisor must be nonzero");
  using namespace std;
  const auto abs_result = abs(dividend) / abs(divisor);
  if (is_negative(dividend) == is_negative(divisor)) return abs_result;
  else return -abs_result;
}

inline int32_t ilog2(uint64_t argument) {
  caller_error_if(argument == 0, "the logarithm of zero is undefined");
#if defined(DETECTED_builtin_clz64)
  return 63 - DETECTED_builtin_clz64(argument);
#else
  int32_t shift
         = argument &  (((1ULL << 32) - 1) << 32)           ? 32 : 0;
  shift += argument & ((((1ULL << 16) - 1) << 16) << shift) ? 16 : 0;
  shift += argument & ((((1ULL <<  8) - 1) <<  8) << shift) ?  8 : 0;
  shift += argument & ((((1ULL <<  4) - 1) <<  4) << shift) ?  4 : 0;
  shift += argument & ((((1ULL <<  2) - 1) <<  2) << shift) ?  2 : 0;
  shift += argument & ((((1ULL <<  1) - 1) <<  1) << shift) ?  1 : 0;
  return shift;
#endif
}
inline int32_t ilog2(uint32_t argument) {
  caller_error_if(argument == 0, "the logarithm of zero is undefined");
#if defined(DETECTED_builtin_clz64)
  return 31 - DETECTED_builtin_clz32(argument);
#else
  int32_t shift
         = argument & ((((1ULL << 16) - 1) << 16) << shift) ? 16 : 0;
  shift += argument & ((((1ULL <<  8) - 1) <<  8) << shift) ?  8 : 0;
  shift += argument & ((((1ULL <<  4) - 1) <<  4) << shift) ?  4 : 0;
  shift += argument & ((((1ULL <<  2) - 1) <<  2) << shift) ?  2 : 0;
  shift += argument & ((((1ULL <<  1) - 1) <<  1) << shift) ?  1 : 0;
  return shift;
#endif
}
inline int32_t num_bits_in_integer_that_are_not_leading_zeroes(uint64_t i) {
  if(i == 0) return 0;
  else return ilog2(i) + 1;
}
inline int32_t num_bits_in_integer_that_are_not_leading_zeroes(uint32_t i) {
  if(i == 0) return 0;
  else return ilog2(i) + 1;
}
template<uint64_t Bits>
struct static_num_bits_in_integer_that_are_not_leading_zeroes {
  static const uint64_t value = boost::static_log2<Bits>::value + 1;
};
template<>
struct static_num_bits_in_integer_that_are_not_leading_zeroes<0> {
  static const uint64_t value = 0;
};

inline int32_t count_trailing_zeroes_64(uint64_t argument) {
  caller_error_if(argument == 0, "the number of trailing zeroes of zero is undefined");
#if defined(DETECTED_builtin_ctz64)
  return DETECTED_builtin_ctz64(argument);
#else
  int32_t shift
         = argument &  ((1ULL << 32) - 1)           ? 0 : 32;
  shift += argument & (((1ULL << 16) - 1) << shift) ? 0 : 16;
  shift += argument & (((1ULL <<  8) - 1) << shift) ? 0 : 8;
  shift += argument & (((1ULL <<  4) - 1) << shift) ? 0 : 4;
  shift += argument & (((1ULL <<  2) - 1) << shift) ? 0 : 2;
  shift += argument & (((1ULL <<  1) - 1) << shift) ? 0 : 1;
  return shift;
#endif
}

inline uint32_t i64sqrt(uint64_t radicand)
{
  typedef uint64_t full_t;
  typedef uint32_t half_t;

  // log2(0) doesn't exist, but sqrt(0) does, so we have to check for it here.
  if(radicand == 0)return 0;

  //shift is the log base 2 of radicand, rounded down.
  int shift = ilog2(radicand);

  //bounds are [lower_bound, upper_bound), a half-open range.
  //lower_bound is guaranteed to be less than or equal to the answer.
  //upper_bound is guaranteed to be greater than the answer.
  half_t lower_bound = half_t(1) << (shift >> 1);

  //upper_bound is twice the original lower_bound;
  //upper_bound is    2**(floor(log2(radicand) / 2)+1)
  //which is equal to 2**ceil((log2(radicand)+1) / 2)
  full_t upper_bound = full_t(lower_bound) << 1;

#ifdef DETECTED_uint128_t
  typedef DETECTED_uint128_t twice_t;
  assert_if_ASSERT_EVERYTHING(full_t(lower_bound)*lower_bound <= radicand);
  assert_if_ASSERT_EVERYTHING(twice_t(upper_bound)*upper_bound > radicand);
#endif

  while(lower_bound < upper_bound - 1)
  {
    const half_t mid = half_t((upper_bound + lower_bound) >> 1);
    if(full_t(mid) * mid > radicand) {
      upper_bound = mid;
    }
    else {
      lower_bound = mid;
    }
  }

  return lower_bound;
}

template<typename IntType> struct non_normalized_rational {
  IntType numerator;
  IntType denominator;
  non_normalized_rational(IntType n, IntType d):numerator(n),denominator(d){ caller_error_if(denominator == 0, "Constructing a rational with denominator zero..."); }
  non_normalized_rational(IntType n):numerator(n),denominator(1){}
  non_normalized_rational():numerator(0),denominator(1){}
  bool operator< (non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator <  o.numerator*denominator;
    else
      return numerator*o.denominator >  o.numerator*denominator;
  }
  bool operator> (non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator >  o.numerator*denominator;
    else
      return numerator*o.denominator <  o.numerator*denominator;
  }
  bool operator<=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator <= o.numerator*denominator;
    else
      return numerator*o.denominator >= o.numerator*denominator;
  }
  bool operator>=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator >= o.numerator*denominator;
    else
      return numerator*o.denominator <= o.numerator*denominator;
  }
  bool operator==(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    return numerator*o.denominator == o.numerator*denominator; }
  bool operator!=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    return numerator*o.denominator != o.numerator*denominator; }
  
  non_normalized_rational reciprocal()const {
    return non_normalized_rational(denominator, numerator);
  }
  non_normalized_rational operator+(non_normalized_rational const& o)const {
    // Simplify only the stupidest case.
    if (denominator == o.denominator) return non_normalized_rational(numerator + numerator, denominator);
    return non_normalized_rational(numerator * o.denominator + denominator * o.numerator, denominator * o.denominator);
  }
  void operator+=(non_normalized_rational const& o) {
    // Simplify only the stupidest case.
    if (denominator == o.denominator) {
      numerator += o.numerator;
    }
    else {
      numerator = numerator * o.denominator + denominator * o.numerator;
      denominator *= o.denominator;
    }
  }
  non_normalized_rational operator-(non_normalized_rational const& o)const {
    // Simplify only the stupidest case.
    if (denominator == o.denominator) return non_normalized_rational(numerator - numerator, denominator);
    return non_normalized_rational(numerator * o.denominator - denominator * o.numerator, denominator * o.denominator);
  }
  void operator-=(non_normalized_rational const& o) {
    // Simplify only the stupidest case.
    if (denominator == o.denominator) {
      numerator -= o.numerator;
    }
    else {
      numerator = numerator * o.denominator - denominator * o.numerator;
      denominator *= o.denominator;
    }
  }
  non_normalized_rational operator*(non_normalized_rational const& o)const {
    // Simplify the stupidest cases.
    if (numerator == o.denominator) return non_normalized_rational(o.numerator, denominator);
    if (o.numerator == denominator) return non_normalized_rational(numerator, o.denominator);
    return non_normalized_rational(numerator * o.numerator, denominator * o.denominator);
  }
  void operator*=(non_normalized_rational const& o) {
    // Simplify the stupidest cases.
    if (numerator == o.denominator) numerator = o.numerator;
    else if (o.numerator == denominator) denominator = o.denominator;
    else {
      numerator *= o.numerator;
      denominator *= o.denominator;
    }
  }
  non_normalized_rational operator/(non_normalized_rational const& o)const {
    return *this * o.reciprocal();
  }
  void operator/=(non_normalized_rational const& o) {
    *this *= o.reciprocal();
  }
};
template<typename IntType> inline std::ostream& operator<<(std::ostream& os, non_normalized_rational<IntType>const& r) {
  return os << r.numerator << '/' << r.denominator;
}

template<typename IntType, typename Num>
inline auto multiply_rational_into(Num n, non_normalized_rational<IntType> rat)
-> decltype(n * rat.numerator / rat.denominator) {
  return n * rat.numerator / rat.denominator;
}

#endif
