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


// Zero divided by something is always zero.
// Any other division result is always (before rounding)
// positive or negative.
// We specify a division strategy in terms of positive
// numbers and an indication of how negative rounding is
// related to positive rounding.
enum rounding_strategy_for_positive_numbers {
  round_down, round_up,
  round_to_nearest_with_ties_rounding_up,
  round_to_nearest_with_ties_rounding_down,
  round_to_nearest_with_ties_rounding_to_even,
  round_to_nearest_with_ties_rounding_to_odd
};
enum rounding_strategy_for_negative_numbers {
  // "doesn't make a difference" is true for unsigned arguments
  // and for round-to-even and round-to-odd.  It is a compile
  // error to claim "doesn't make a difference" when it might
  // in fact make a difference.
  negative_variant_doesnt_make_a_difference,
  
  // result invariant under negation; roughly,
  // -divide(-x, y, strat) == divide(x, y, strat)
  negative_mirrors_positive,
  
  // result invariant under addition of a constant; roughly,
  // divide(x + C*y, y, strat) - C == divide(x, y, strat)
  negative_continuous_with_positive,
  
  // these just assert that the numerator and denominator is nonnegative.
  negative_is_forbidden
};
template<
  rounding_strategy_for_positive_numbers PosStrategy,
  rounding_strategy_for_negative_numbers NegStrategy
    = negative_variant_doesnt_make_a_difference>
struct rounding_strategy {
  static const rounding_strategy_for_positive_numbers positive_strategy = PosStrategy;
  static const rounding_strategy_for_negative_numbers negative_strategy = NegStrategy;
};
namespace rounding_strategies_impl {
// Otherwise we get warnings when instantiated with unsigned
// types for (val < 0):
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#pragma clang diagnostic ignored "-Wtautological-compare"

// C++11 specifies that int '/' is round-to-zero.
// Test this at compile time and runtime (misc_utils_tests.cpp
// test "standard_rounding_behavior").
// It's simpler to implement division algorithms when we can
// count on something (anything), and this is the thing
// that everyone seems to have agreed on.
static_assert(8/3 == 2, "we use the typical, standard round-towards-zero");
static_assert(-8/3 == -2, "we use the typical, standard round-towards-zero");
static_assert(-8/-3 == 2, "we use the typical, standard round-towards-zero");
static_assert(8/-3 == -2, "we use the typical, standard round-towards-zero");
static_assert(8%3 == 2, "we use the typical, standard round-towards-zero");
static_assert(-8%3 == -2, "we use the typical, standard round-towards-zero");
static_assert(-8%-3 == -2, "we use the typical, standard round-towards-zero");
static_assert(8%-3 == 2, "we use the typical, standard round-towards-zero");

// TODO boost::enable_if<std::numeric_limits<T1 & T2>::is_specialized
// or get_primitive_int_type? or.

// Division is a slow instruction, so we want to start it as soon
// as possible and finish as soon as possible after we have the
// result.  Most division instructions also give modulus for free.
// If the divisor is a compile-time constant, however, the compiler
// converts it to a multiplication-by-constant and a few shifts,
// and the remainder sometimes costs a bit.
// We also have to consider, if modifying 'divisor' before dividing,
// whether the possibility of overflow is acceptable.
template<typename T>
inline T divide_impl(T dividend, T divisor,
      rounding_strategy<round_down, negative_mirrors_positive>) {
  caller_correct_if(divisor != T(0), "divisor must be nonzero");
  return dividend / divisor;
}
template<typename T>
inline T divide_impl(T dividend, T divisor,
      rounding_strategy<round_down, negative_continuous_with_positive>) {
  caller_correct_if(divisor != T(0), "divisor must be nonzero");
  const T rounded_to_zero = dividend / divisor;
  const T remainder = dividend % divisor;
  //'adjust' is probably branch-predictable
  // even if the sign of the numbers aren't.
  // (The compiler seems to be smart and transforms
  //  either to something reasonable, given that either
  //  has a reasonable way to run it.)
  //const T adjust =
  //  ((dividend < 0) != (divisor < 0)) && (remainder != 0);
  //return rounded_to_zero - adjust;
  if(((dividend < T(0)) != (divisor < T(0))) && (remainder != T(0))) {
    return rounded_to_zero - T(1);
  }
  else {
    return rounded_to_zero;
  }
}
template<typename T>
inline T divide_impl(T dividend, T divisor,
      rounding_strategy<round_up, negative_mirrors_positive>) {
  caller_correct_if(divisor != T(0), "divisor must be nonzero");
  const T rounded_to_zero = dividend / divisor;
  const T remainder = dividend % divisor;
  const T adjust = ((dividend < T(0)) == (divisor < T(0)) ? T(1) : T(-1));
  if(remainder != 0) {
    return rounded_to_zero + adjust;
  }
  else {
    return rounded_to_zero;
  }
}
template<typename T>
inline T divide_impl(T dividend, T divisor,
      rounding_strategy<round_up, negative_continuous_with_positive>) {
  caller_correct_if(divisor != T(0), "divisor must be nonzero");
  const T rounded_to_zero = dividend / divisor;
  const T remainder = dividend % divisor;
  if(((dividend < T(0)) == (divisor < T(0))) && (remainder != T(0))) {
    return rounded_to_zero + T(1);
  }
  else {
    return rounded_to_zero;
  }
}

// this is too tricky to allow two different int type arguments for
template<typename T,
  rounding_strategy_for_positive_numbers PosStrategy,
  rounding_strategy_for_negative_numbers NegStrategy>
inline T divide_impl(T dividend, T divisor,
      rounding_strategy<PosStrategy, NegStrategy>) {
  caller_correct_if(divisor != T(0), "divisor must be nonzero");
  const T rounded_to_zero = dividend / divisor;
  const T remainder = dividend % divisor;
  // result_positive is a slight misnomer.
  // If dividend is zero, then the result and remainder are zero
  // and all is fine regardless of this bool's value.
  // If dividend is nonzero, this bool indicates (as it must)
  // the sign of the conceptual, unrounded quotient.
  const bool result_positive = (dividend < T(0)) == (divisor < T(0));
  const T adjust = (result_positive ? T(1) : T(-1));

  // compare(abs(divisor)/2, abs(remainder)) without rounding error
  // or overflow for any argument.
  T tweak;
  T nonnegative_divisor_ish;
  if(divisor < T(0)) {
    tweak = T(1);
    nonnegative_divisor_ish = ~divisor;
  }
  else {
    tweak = T(0);
    nonnegative_divisor_ish = divisor;
  }
  // remainder cannot be -MIN_INT
  // remainder's sign is dividend's sign (the CPU knows the latter sooner)
  const T nonnegative_remainder = ((dividend < T(0)) ? -remainder : remainder);
  const T left = nonnegative_divisor_ish - nonnegative_remainder;
  const T right = nonnegative_remainder - tweak;
  if(left < right) {
    // remainder's magnitude is large; round quotient's magnitude up
    return rounded_to_zero + adjust;
  }
  else if(left > right) {
    // remainder's magnitude is small; round quotient's magnitude down
    return rounded_to_zero;
  }
  else {
    // tie!
    // partially compile-time ifs here.
    if(PosStrategy == round_to_nearest_with_ties_rounding_to_even) {
      if(rounded_to_zero & 1) { return rounded_to_zero + adjust; }
      else                    { return rounded_to_zero; }
    }
    if(PosStrategy == round_to_nearest_with_ties_rounding_to_odd) {
      if(rounded_to_zero & 1) { return rounded_to_zero; }
      else                    { return rounded_to_zero + adjust; }
    }
    // else it is one of the four biased rounding tiebreakers.
    if((result_positive || NegStrategy == negative_mirrors_positive)
        == (PosStrategy == round_to_nearest_with_ties_rounding_up)) {
      return rounded_to_zero + adjust;
    }
    else {
      return rounded_to_zero;
    }
  }
}

template<typename T, rounding_strategy_for_positive_numbers PosStrategy>
inline T divide_impl(T dividend, T divisor,
      rounding_strategy<PosStrategy, negative_is_forbidden>) {
  caller_error_if(dividend < 0 || divisor < 0, "Negative number forbidden in this division!");
  // Use unsigned types so that the divide_impl optimizes better
  // even if it's not inlined.
  typedef typename boost::make_unsigned<T>::type UnsignedT;
  return T(divide_impl<UnsignedT>(
    UnsignedT(dividend),
    UnsignedT(divisor),
    rounding_strategy<PosStrategy, negative_mirrors_positive>()));
}
#pragma GCC diagnostic pop
} /* end namespace rounding_strategies */

// Avoid specifying rounding_strategy<> in last argument's structure so that
// overloads of divide() will be picked before this one.
template<typename T1, typename T2, typename RoundingStrategy,
  rounding_strategy_for_positive_numbers PosStrategy = RoundingStrategy::positive_strategy,
  rounding_strategy_for_negative_numbers NegStrategy = RoundingStrategy::negative_strategy,
  typename = typename boost::enable_if_c<
    (std::numeric_limits<T1>::is_specialized && std::numeric_limits<T2>::is_specialized
      // we don't currently implement IEEE754 floating-point rounding modes
      && std::numeric_limits<T1>::is_integer && std::numeric_limits<T2>::is_integer
    )>::type>
inline auto divide(T1 dividend, T2 divisor, RoundingStrategy strat)
-> decltype(dividend/divisor) {
  static_assert(std::numeric_limits<T1>::is_signed == std::numeric_limits<T2>::is_signed,
                "Dividing two numbers of mixed sign is probably a bad idea.");
  typedef decltype(dividend / divisor) operation_type;
  static_assert(
       NegStrategy != negative_variant_doesnt_make_a_difference
    || PosStrategy == round_to_nearest_with_ties_rounding_to_even
    || PosStrategy == round_to_nearest_with_ties_rounding_to_odd
    || (!std::numeric_limits<T1>::is_signed && !std::numeric_limits<T2>::is_signed),
    "You lied! The negative variant does make a difference.");
  return rounding_strategies_impl::divide_impl(
    operation_type(dividend), operation_type(divisor), strat);
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


inline int32_t popcount(uint64_t argument)  {
  static const uint64_t mask0 = 0x5555555555555555;
  static const uint64_t mask1 = 0x3333333333333333;
  static const uint64_t mask2 = 0x0f0f0f0f0f0f0f0f;
  static const uint64_t mask3 = 0x00ff00ff00ff00ff;
  static const uint64_t mask4 = 0x0000ffff0000ffff;
  static const uint64_t mask5 = 0x00000000ffffffff;
  uint64_t n = argument;
  n = (n & mask0) + ((n >> (1<<0)) & mask0);
  n = (n & mask1) + ((n >> (1<<1)) & mask1);
  n = (n & mask2) + ((n >> (1<<2)) & mask2);
  n = (n & mask3) + ((n >> (1<<3)) & mask3);
  n = (n & mask4) + ((n >> (1<<4)) & mask4);
  n = (n & mask5) + ((n >> (1<<5)) & mask5);
  return int32_t(n);
}
inline int32_t popcount(uint32_t argument)  {
  static const uint32_t mask0 = 0x55555555;
  static const uint32_t mask1 = 0x33333333;
  static const uint32_t mask2 = 0x0f0f0f0f;
  static const uint32_t mask3 = 0x00ff00ff;
  static const uint32_t mask4 = 0x0000ffff;
  uint32_t n = argument;
  n = (n & mask0) + ((n >> (1<<0)) & mask0);
  n = (n & mask1) + ((n >> (1<<1)) & mask1);
  n = (n & mask2) + ((n >> (1<<2)) & mask2);
  n = (n & mask3) + ((n >> (1<<3)) & mask3);
  n = (n & mask4) + ((n >> (1<<4)) & mask4);
  return int32_t(n);
}
inline int32_t popcount(uint16_t argument)  {
  static const uint32_t mask0 = 0x5555;
  static const uint32_t mask1 = 0x3333;
  static const uint32_t mask2 = 0x0f0f;
  static const uint32_t mask3 = 0x00ff;
  uint32_t n = argument;
  n = (n & mask0) + ((n >> (1<<0)) & mask0);
  n = (n & mask1) + ((n >> (1<<1)) & mask1);
  n = (n & mask2) + ((n >> (1<<2)) & mask2);
  n = (n & mask3) + ((n >> (1<<3)) & mask3);
  return int32_t(n);
}
inline int32_t popcount(uint8_t argument)  {
  static const uint32_t mask0 = 0x55;
  static const uint32_t mask1 = 0x33;
  static const uint32_t mask2 = 0x0f;
  uint32_t n = argument;
  n = (n & mask0) + ((n >> (1<<0)) & mask0);
  n = (n & mask1) + ((n >> (1<<1)) & mask1);
  n = (n & mask2) + ((n >> (1<<2)) & mask2);
  return int32_t(n);
}


template<typename Int>
inline int32_t ilog2(Int argument) {
  static_assert(std::numeric_limits<Int>::digits <= 64, "not implemented");
  if(std::numeric_limits<Int>::is_signed) caller_error_if(argument <= Int(0), "logarithm is only defined on positive numbers");
  if(std::numeric_limits<Int>::digits <= 32) return ilog2(get_primitive<uint32_t>(argument));
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


template<uintmax_t Result>
struct static_pow_nonnegative_integer_answer {
  static const uintmax_t value = Result; // result or max uintmax_t for overflow
  static const uintmax_t value_minus_one = value - 1; //result-1 or max uintmax_t for overflow
  static const uintmax_t modulo_value = Result; // result
  static const bool overflow = false;
};
template<uintmax_t A, uintmax_t B, bool AlreadyOverflowing = false>
struct static_multiply_nonnegative_integer {
  static const uintmax_t modulo_value = A * B;
  static const bool overflow = AlreadyOverflowing
    || modulo_value / (B?B:1) != (B?A:0)  //if B!=0, check
    || modulo_value / (A?A:1) != (A?B:0); //if A!=0, check
  static const uintmax_t value = (overflow ? uintmax_t(-1) : modulo_value);
  static const uintmax_t value_minus_one = (overflow ? uintmax_t(-1) : modulo_value - 1);
};

template<typename A, typename B>
struct static_re_multiply_nonnegative_integer
  : static_multiply_nonnegative_integer<A::modulo_value, B::modulo_value,
                                          (A::overflow || B::overflow)> {};



template<typename Base, uintmax_t Exponent, bool Odd = (Exponent % 2 == 1)>
struct static_pow_nonnegative_integer_impl;
template<typename B> struct static_pow_nonnegative_integer_impl<B, 0, false>
  : static_pow_nonnegative_integer_answer<1> {};
template<typename B> struct static_pow_nonnegative_integer_impl<B, 1, true>
  : B {};
template<typename B> struct static_pow_nonnegative_integer_impl<B, 2, false>
  : static_re_multiply_nonnegative_integer<B, B> {};
template<typename B, uintmax_t EvenExponent> struct static_pow_nonnegative_integer_impl<B, EvenExponent, false>
  : static_re_multiply_nonnegative_integer<
      static_pow_nonnegative_integer_impl<B, EvenExponent/2>,
      static_pow_nonnegative_integer_impl<B, EvenExponent/2>
      > {};
template<typename B, uintmax_t OddExponent> struct static_pow_nonnegative_integer_impl<B, OddExponent, true>
  : static_re_multiply_nonnegative_integer<
      B,
      static_re_multiply_nonnegative_integer<
        static_pow_nonnegative_integer_impl<B, OddExponent/2>,
        static_pow_nonnegative_integer_impl<B, OddExponent/2>
      > > {};

template<uintmax_t Base, uintmax_t Exponent, bool AllowOverflow = false>
struct static_pow_nonnegative_integer
  : static_pow_nonnegative_integer_impl<
      static_pow_nonnegative_integer_answer<Base>, Exponent> {
  static_assert(AllowOverflow ||
    !static_pow_nonnegative_integer_impl<
      static_pow_nonnegative_integer_answer<Base>, Exponent>::overflow,
    "exponentiation overflow");
};

template<uintmax_t Radicand, uintmax_t Root = 2>
struct static_root_nonnegative_integer {
  // Newton-Raphson method.
  template<uintmax_t LowerBound, uintmax_t UpperBound, bool Done>
  struct recur {
    static const uintmax_t mid = ((LowerBound + UpperBound) >> 1);
    static const bool mid_is_new_upper_bound =
      static_pow_nonnegative_integer<mid, Root, true>::value_minus_one > Radicand - 1;
    static const uintmax_t new_lower_bound = mid_is_new_upper_bound ? LowerBound : mid;
    static const uintmax_t new_upper_bound = mid_is_new_upper_bound ? mid : UpperBound;
    static const bool done = new_lower_bound >= new_upper_bound - 1;
    static const uintmax_t value = recur<new_lower_bound, new_upper_bound, done>::value;
  };
  template<uintmax_t LowerBound, uintmax_t UpperBound>
  struct recur<LowerBound, UpperBound, true> {
    static const uintmax_t value = LowerBound;
  };
  static const uintmax_t shift = boost::static_log2<Radicand>::value;
  static const uintmax_t initial_lower_bound = uintmax_t(1) << (shift / Root);
  static const uintmax_t initial_upper_bound = initial_lower_bound * Root;
  static const uintmax_t value = recur<initial_lower_bound, initial_upper_bound, false>::value;
  static const uintmax_t remainder = Radicand - static_pow_nonnegative_integer<value, Root>::value;
};
template<uintmax_t Radicand>
struct static_root_nonnegative_integer<Radicand, 1> {
  static const uintmax_t value = Radicand;
  static const uintmax_t remainder = 0;
};
template<uintmax_t Root>
struct static_root_nonnegative_integer<0, Root> {
  static const uintmax_t value = 0;
  static const uintmax_t remainder = 0;
};
// zeroth roots are meaningless:
template<uintmax_t Radicand>
struct static_root_nonnegative_integer<Radicand, 0> {};
template<>
struct static_root_nonnegative_integer<0, 0> {};


static const uintmax_t safe_uintmax_t_to_square =
  (uintmax_t(1)<<(std::numeric_limits<uintmax_t>::digits/2)) - 1u;
template<uintmax_t Factor, uintmax_t Factoree,
  int Difficulty = (
    ((Factoree % Factor) == 0)
    + (Factor <= safe_uintmax_t_to_square &&
        ((Factoree % ((Factor <= safe_uintmax_t_to_square)*Factor*Factor)) == 0)))>
struct extract_factor_impl;
template<uintmax_t Factor, uintmax_t Factoree>
struct extract_factor_impl<Factor, Factoree, 0> {
  static const int factor_exponent = 0;
  static const uintmax_t factored_out_value = 1;
  static const uintmax_t rest_of_factoree = Factoree;
};
template<uintmax_t Factor, uintmax_t Factoree>
struct extract_factor_impl<Factor, Factoree, 1> {
  static const int factor_exponent = (Factoree % Factor) == 0;
  static const uintmax_t factored_out_value = (factor_exponent ? Factor : 1);
  static const uintmax_t rest_of_factoree = (factor_exponent ? Factoree/Factor : Factoree);
};
template<uintmax_t Factor, uintmax_t Factoree>
struct extract_factor_impl<Factor, Factoree, 2> {
private:
  static_assert(std::numeric_limits<uintmax_t>::digits <= 128, "unimplemented");
  static const uintmax_t f1 = Factor;
  static const uintmax_t f2 = (f1<=safe_uintmax_t_to_square)*f1*f1;
  static const uintmax_t f4 = (f2<=safe_uintmax_t_to_square)*f2*f2;
  static const uintmax_t f8 = (f4<=safe_uintmax_t_to_square)*f4*f4;
  static const uintmax_t f16 = (f8<=safe_uintmax_t_to_square)*f8*f8;
  static const uintmax_t f32 = (f16<=safe_uintmax_t_to_square)*f16*f16;
  static const uintmax_t f64 = (f32<=safe_uintmax_t_to_square)*f32*f32;
  static const bool e64 = f64 && ((Factoree % (    (f64?f64:1))) == 0);
  static const uintmax_t p64 =     (e64?f64:1);
  static const bool e32 = f32 && ((Factoree % (p64*(f32?f32:1))) == 0);
  static const uintmax_t p32 = p64*(e32?f32:1);
  static const bool e16 = f16 && ((Factoree % (p32*(f16?f16:1))) == 0);
  static const uintmax_t p16 = p32*(e16?f16:1);
  static const bool e8 = f8 && ((Factoree % (p16*(f8?f8:1))) == 0);
  static const uintmax_t p8 = p16*(e8?f8:1);
  static const bool e4 = f4 && ((Factoree % (p8*(f4?f4:1))) == 0);
  static const uintmax_t p4 = p8*(e4?f4:1);
  static const bool e2 = f2 && ((Factoree % (p4*(f2?f2:1))) == 0);
  static const uintmax_t p2 = p4*(e2?f2:1);
  static const bool e1 = f1 && ((Factoree % (p2*(f1?f1:1))) == 0);
  static const uintmax_t p1 = p2*(e1?f1:1);
public:
  static const int factor_exponent = e64*64 + e32*32 + e16*16 + e8*8
                                              + e4*4 + e2*2 + e1*1;
  static const uintmax_t factored_out_value = p1;
  static const uintmax_t rest_of_factoree = Factoree / factored_out_value;
};

// extract_factor<F, N> divides out F from N as many times
// as it goes in evenly.  It tells you how many times it
// divided (factor_exponent), the amount divided out
// (factors_value, which is F to the factor_exponent),
// and N sans the F parts (rest_of_factoree).
template<uintmax_t Factor, uintmax_t Factoree>
struct extract_factor : extract_factor_impl<Factor, Factoree> {
  static const int factor_base = Factor;
  static const int factoree = Factoree;
};
// These are not meaningful:
template<uintmax_t Factoree> struct extract_factor<0, Factoree>;
template<uintmax_t Factoree> struct extract_factor<1, Factoree>;
template<uintmax_t Factor> struct extract_factor<Factor, 0>;





// TODO rename to isqrt or similar?
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

template<typename Target, typename Num>
struct numeric_representation_cast_impl {
  typedef Target target_type;
};

template<typename Target, typename Num>
inline typename numeric_representation_cast_impl<Target, Num>::target_type
numeric_representation_cast(Num const& num) {
  return typename numeric_representation_cast_impl<Target, Num>::target_type(num);
}

#endif
