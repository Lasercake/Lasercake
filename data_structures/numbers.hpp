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
#include <boost/type_traits/make_signed.hpp>
#include <boost/type_traits/make_unsigned.hpp>

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

template<typename Int>
typename boost::make_signed<Int>::type to_signed_type(Int i) {
  typename boost::make_signed<Int>::type result(i);
  caller_correct_if(result >= 0, "to_signed_type overflow");
  return result;
}
template<typename Int>
typename boost::make_unsigned<Int>::type to_unsigned_type(Int i) {
  caller_correct_if(i >= 0, "to_unsigned_type underflow");
  typename boost::make_unsigned<Int>::type result(i);
  return result;
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
#if defined(DETECTED_builtin_clz32)
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


// I wish this machinery for shift_value_is_safe_for_type<>()
// didn't have to be so complicated.
template<typename Int, Int Min, Int Max> class bounds_checked_int;
template<typename Target, typename AnyInt>
Target get_primitive(AnyInt a);

template<typename ShiftedType, bool HasNumericLimits>
struct shift_value_is_safe_impl_2;
template<typename ShiftedType>
struct shift_value_is_safe_impl_2<ShiftedType, true> {
  template<typename ShiftValueType>
  static bool impl(ShiftValueType const& shift_value) {
    const auto too_high_shift_amount =
      std::numeric_limits<ShiftedType>::digits + std::numeric_limits<ShiftedType>::is_signed;
    return shift_value >= ShiftValueType(0)
        && shift_value < ShiftValueType(too_high_shift_amount);
  };
};
template<typename ShiftedType>
struct shift_value_is_safe_impl_2<ShiftedType, false> {
  template<typename ShiftValueType>
  static bool impl(ShiftValueType const&) {
    return true;
  }
};
template<typename ShiftedType>
struct shift_value_is_safe_impl_1
  : shift_value_is_safe_impl_2<ShiftedType, std::numeric_limits<ShiftedType>::is_specialized> {};
template<typename BaseInt, BaseInt Min, BaseInt Max>
struct shift_value_is_safe_impl_1< bounds_checked_int<BaseInt, Min, Max> >
  : shift_value_is_safe_impl_2<BaseInt, std::numeric_limits<BaseInt>::is_specialized> {};

template<typename ShiftedType, typename ShiftValueType>
inline bool shift_value_is_safe_for_type(ShiftValueType const& shift_value) {
  return shift_value_is_safe_impl_1<ShiftedType>::impl(shift_value);
}

template<typename ShiftedType, typename ShiftValueType>
inline ShiftedType safe_left_shift(ShiftedType const& a, ShiftValueType const& shift) {
  if (shift_value_is_safe_for_type<ShiftedType>(shift)) {
    return a << shift;
  }
  else {
    return 0;
  }
}



template<typename Int>
inline int32_t ilog2(Int argument) {
  static_assert(sizeof(Int) <= 64, "not implemented");
  if(std::numeric_limits<Int>::is_signed) caller_error_if(argument <= Int(0), "logarithm is only defined on positive numbers");
  if(sizeof(Int) <= 32) return ilog2(get_primitive<uint32_t>(argument));
  else return ilog2(get_primitive<uint64_t>(argument));
}

template<typename Int>
inline int32_t num_bits_in_integer_that_are_not_leading_zeroes(Int i) {
  static_assert(!std::numeric_limits<Int>::is_signed, "Don't confuse yourself by calling this on a signed type.");
  if(i == Int(0)) return 0;
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


#ifdef DETECTED_uint128_t
typedef DETECTED_uint128_t uint128;
#else
struct uint128 {
  uint64_t low;
  uint64_t high;

  bool operator==(uint128 other)const {
    return high == other.high && low == other.low;
  }
  bool operator!=(uint128 other)const {
    return high != other.high || low != other.low;
  }
  bool operator<(uint128 other)const {
    return high < other.high || (high == other.high && low < other.low);
  }
  bool operator>(uint128 other)const {
    return other < *this;
  }
  bool operator<=(uint128 other)const {
    return high < other.high || (high == other.high && low <= other.low);
  }
  bool operator>=(uint128 other)const {
    return other <= *this;
  }
};
#endif

#ifdef DETECTED_int128_t
typedef DETECTED_int128_t int128;
#else
struct int128 {
  //store these in the same bit-pattern as a real int128 would be
  uint64_t low;
  uint64_t high;

  bool operator==(int128 other)const {
    return high == other.high && low == other.low;
  }
  bool operator!=(int128 other)const {
    return high != other.high || low != other.low;
  }
  bool operator<(int128 other)const {
    const uint64_t sign_bit = uint64_t(1) << 63;
    return (high^sign_bit) < (other.high^sign_bit) || (high == other.high && low < other.low);
  }
  bool operator>(int128 other)const {
    return other < *this;
  }
  bool operator<=(int128 other)const {
    const uint64_t sign_bit = uint64_t(1) << 63;
    return (high^sign_bit) < (other.high^sign_bit) || (high == other.high && low <= other.low);
  }
  bool operator>=(int128 other)const {
    return other <= *this;
  }
};
#endif

inline uint16_t width_doubling_multiply(uint8_t a1, uint8_t a2) { return (uint16_t)a1 * a2; }
inline int16_t width_doubling_multiply(int8_t a1, int8_t a2) { return (int16_t)a1 * a2; }

inline uint32_t width_doubling_multiply(uint16_t a1, uint16_t a2) { return (uint32_t)a1 * a2; }
inline int32_t width_doubling_multiply(int16_t a1, int16_t a2) { return (int32_t)a1 * a2; }

inline uint64_t width_doubling_multiply(uint32_t a1, uint32_t a2) { return (uint64_t)a1 * a2; }
inline int64_t width_doubling_multiply(int32_t a1, int32_t a2) { return (int64_t)a1 * a2; }

inline uint128 width_doubling_multiply(uint64_t a1, uint64_t a2) {
#ifdef DETECTED_uint128_t
  return (DETECTED_uint128_t)a1 * a2;
#else
  typedef uint128 twice_t;
  typedef uint64_t full_t;
  typedef uint32_t half_t;

  twice_t result;
  if (uint32_t(a1) == a1 && uint32_t(a2) == a2) {
    result.high = 0;
    result.low = (uint64_t)uint32_t(a1) * uint32_t(a2);
  }
  else {
    const half_t half_max = std::numeric_limits<half_t>::max();
    const int half_digits = std::numeric_limits<half_t>::digits;//or sizeof * 8
    const half_t low1 = a1 & half_max;
    const half_t low2 = a2 & half_max;
    const half_t high1 = a1 >> half_digits;
    const half_t high2 = a2 >> half_digits;
    const full_t highhigh = (full_t)high1 * high2;
    const full_t highlow = (full_t)high1 * low2;
    const half_t highlow_low = highlow & half_max;
    const half_t highlow_high = highlow >> half_digits;
    const full_t lowhigh = (full_t)low1 * high2;
    const half_t lowhigh_low = lowhigh & half_max;
    const half_t lowhigh_high = lowhigh >> half_digits;
    const full_t lowlow = (full_t)low1 * low2;

    // Won't overflow:
    result.high = (full_t)highhigh + highlow_high + lowhigh_high;

    result.low = lowlow;
    const full_t overflow_check_A = result.low;
    result.low += ((full_t)highlow_low << half_digits);
    result.high += (result.low < overflow_check_A);

    const full_t overflow_check_B = result.low;
    result.low += ((full_t)lowhigh_low << half_digits);
    result.high += (result.low < overflow_check_B);
  }

  //assert(result.low == (((DETECTED_uint128_t)a1 * a2) & std::numeric_limits<full_t>::max()));
  //assert(result.high == (((DETECTED_uint128_t)a1 * a2) >> std::numeric_limits<full_t>::digits));

  return result;
#endif
}
inline int128 width_doubling_multiply(int64_t a1, int64_t a2) {
#ifdef DETECTED_int128_t
  return (DETECTED_int128_t)a1 * a2;
#else
  int128 result;
  typedef int64_t full_t;
  if (int32_t(a1) == a1 && int32_t(a2) == a2) {
    const int64_t result64 = (int64_t)int32_t(a1) * int32_t(a2);
    result.high = result64 >> 63; //sign-extend
    result.low = result64;
  }
  else if (a1 == 0 || a2 == 0) {
    result.high = 0;
    result.low = 0;
  }
  else if (a1 == std::numeric_limits<full_t>::min()) {
    result.high = -a2;
    result.low = 0;
  }
  else if (a2 == std::numeric_limits<full_t>::min()) {
    result.high = -a1;
    result.low = 0;
  }
  else {
    const uint64_t abs_a1 = std::abs(a1);
    const uint64_t abs_a2 = std::abs(a2);
    const uint128 unsigned_result = width_doubling_multiply(abs_a1, abs_a2);
    if ((a1 < 0) == (a2 < 0)) {
      result.high = unsigned_result.high;
      result.low = unsigned_result.low;
    }
    else {
      result.high = ~unsigned_result.high;
      result.low = -unsigned_result.low;
    }
  }
  //assert(full_t(result.low) == full_t((DETECTED_int128_t)a1 * a2));
  //assert(full_t(result.high) == full_t(((DETECTED_int128_t)a1 * a2) >> 64));
  return result;
#endif
}

template<typename IntType> struct non_normalized_rational {
  IntType numerator;
  IntType denominator;
  non_normalized_rational(IntType n, IntType d):numerator(n),denominator(d){ caller_error_if(denominator == 0, "Constructing a rational with denominator zero..."); }
  non_normalized_rational(IntType n):numerator(n),denominator(1){}
  non_normalized_rational():numerator(0),denominator(1){}
  bool operator< (non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    const auto prod1 = width_doubling_multiply(numerator, o.denominator);
    const auto prod2 = width_doubling_multiply(o.numerator, denominator);
    if (is_negative(denominator) == is_negative(o.denominator))
      return prod1 < prod2;
    else
      return prod1 > prod2;
  }
  bool operator> (non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    const auto prod1 = width_doubling_multiply(numerator, o.denominator);
    const auto prod2 = width_doubling_multiply(o.numerator, denominator);
    if (is_negative(denominator) == is_negative(o.denominator))
      return prod1 > prod2;
    else
      return prod1 < prod2;
  }
  bool operator<=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    const auto prod1 = width_doubling_multiply(numerator, o.denominator);
    const auto prod2 = width_doubling_multiply(o.numerator, denominator);
    if (is_negative(denominator) == is_negative(o.denominator))
      return prod1 <= prod2;
    else
      return prod1 >= prod2;
  }
  bool operator>=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    const auto prod1 = width_doubling_multiply(numerator, o.denominator);
    const auto prod2 = width_doubling_multiply(o.numerator, denominator);
    if (is_negative(denominator) == is_negative(o.denominator))
      return prod1 >= prod2;
    else
      return prod1 <= prod2;
  }
  bool operator==(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    const auto prod1 = width_doubling_multiply(numerator, o.denominator);
    const auto prod2 = width_doubling_multiply(o.numerator, denominator);
    return prod1 == prod2;
  }
  bool operator!=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    const auto prod1 = width_doubling_multiply(numerator, o.denominator);
    const auto prod2 = width_doubling_multiply(o.numerator, denominator);
    return prod1 != prod2;
  }
  
  non_normalized_rational reciprocal()const {
    return non_normalized_rational(denominator, numerator);
  }
  non_normalized_rational operator+(non_normalized_rational const& o)const {
    // Simplify only the stupidest case.
    if (denominator == o.denominator) return non_normalized_rational(numerator + o.numerator, denominator);
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
    if (denominator == o.denominator) return non_normalized_rational(numerator - o.numerator, denominator);
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
