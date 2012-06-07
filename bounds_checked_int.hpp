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

#include "utils.hpp"

#ifndef LASERCAKE_BOUNDS_CHECKED_INT_HPP__
#define LASERCAKE_BOUNDS_CHECKED_INT_HPP__

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/make_signed.hpp>
#include <boost/type_traits/make_unsigned.hpp>
#include <boost/integer_traits.hpp>
#include <boost/integer/static_min_max.hpp>
#include <boost/mpl/if.hpp>

#include <ostream>

namespace bounds_checked_int_impl {
  template<int bits, bool is_signed> struct int_types;
  template<> struct int_types< 8, true > { typedef  int8_t type; typedef int8_t signed_type; typedef uint8_t unsigned_type; };
  template<> struct int_types< 8, false> { typedef uint8_t type; typedef int8_t signed_type; typedef uint8_t unsigned_type; };
  template<> struct int_types<16, true > { typedef  int16_t type; typedef int16_t signed_type; typedef uint16_t unsigned_type; };
  template<> struct int_types<16, false> { typedef uint16_t type; typedef int16_t signed_type; typedef uint16_t unsigned_type; };
  template<> struct int_types<32, true > { typedef  int32_t type; typedef int32_t signed_type; typedef uint32_t unsigned_type; };
  template<> struct int_types<32, false> { typedef uint32_t type; typedef int32_t signed_type; typedef uint32_t unsigned_type; };
  template<> struct int_types<64, true > { typedef  int64_t type; typedef int64_t signed_type; typedef uint64_t unsigned_type; };
  template<> struct int_types<64, false> { typedef uint64_t type; typedef int64_t signed_type; typedef uint64_t unsigned_type; };
#if defined(DETECTED_int128_t) && defined(DETECTED_uint128_t)
  template<> struct int_types<128, true > { typedef  DETECTED_int128_t type; typedef DETECTED_int128_t signed_type; typedef DETECTED_uint128_t unsigned_type; };
  template<> struct int_types<128, false> { typedef DETECTED_uint128_t type; typedef DETECTED_int128_t signed_type; typedef DETECTED_uint128_t unsigned_type; };
#endif
  struct cannot_compare_types_of_different_signs_correctly {};
  // for checking int1 OP int2 (or unary ops on int1 if you take the default value)
  template<typename Int1, typename Int2 = Int1>
  struct checking : int_types<2 * boost::static_signed_max<sizeof(Int1)*8, sizeof(Int2)*8>::value,
                              std::numeric_limits<Int1>::is_signed || std::numeric_limits<Int2>::is_signed> {
    struct bit_math : int_types<boost::static_signed_max<sizeof(Int1)*8, sizeof(Int2)*8>::value,
                                std::numeric_limits<Int1>::is_signed && std::numeric_limits<Int2>::is_signed> {};
    typedef typename boost::mpl::if_c<std::numeric_limits<Int1>::is_signed == std::numeric_limits<Int2>::is_signed,
                                         int_types<boost::static_signed_max<sizeof(Int1)*8, sizeof(Int2)*8>::value, std::numeric_limits<Int1>::is_signed>,
                                         cannot_compare_types_of_different_signs_correctly>::type comparison;
  };
  
  template<typename Int1, typename Int2>
  struct superior_to {
    static const bool value =
          std::numeric_limits<Int1>::is_signed  == std::numeric_limits<Int2>::is_signed &&
          std::numeric_limits<Int1>::radix      == std::numeric_limits<Int2>::radix &&
          std::numeric_limits<Int1>::is_integer <= std::numeric_limits<Int2>::is_integer &&
          std::numeric_limits<Int1>::digits     >= std::numeric_limits<Int2>::digits;
  };
  
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wtype-limits"
  #pragma clang diagnostic ignored "-Wtautological-compare" /*Clang uses GCC's warning pragma stack*/
  template<typename Int, typename ShiftAmountType>
  inline void check_shift_amount_valid(ShiftAmountType shift) {
    // Such shifts do not work as you'd mathematically expect on some CPUs' bare metal,
    // and bounds_checked_int is meant to be used in a correctly removable way so we cannot
    // emulate them.  (Also, it would make bounds checking more work.)
    caller_error_if(shift >= ShiftAmountType(std::numeric_limits<Int>::digits), "bounds_checked_int invalid too-large shift");
    caller_error_if(shift < ShiftAmountType(0), "bounds_checked_int invalid negative-amount shift");
  }
  #pragma GCC diagnostic pop
}

template<
  typename Int,
  Int Min,
  Int Max
>
class bounds_checked_int;
//overloaded: a.get() or a
template<typename Int, Int Min, Int Max>
inline Int get_primitive_int(bounds_checked_int<Int,Min,Max> a) { return a.get(); }
template<typename Int>
inline Int get_primitive_int(Int a) { return a; }

template<typename AnyInt>
inline double get_primitive_double(AnyInt a) { return get_primitive_int(a); }

template<
  typename Int,
  Int Min = boost::integer_traits<Int>::const_min,
  Int Max = boost::integer_traits<Int>::const_max
>
class bounds_checked_int {
private:
  struct unspecified_bool_{int member; private:unspecified_bool_();};
  typedef int unspecified_bool_::* unspecified_bool_type;
public:
  static_assert(Min <= Max, "bounds_checked_int min less than max (sanity check)");
  static_assert(Min >= boost::integer_traits<Int>::const_min, "bounds_checked_int min fits in type");
  static_assert(Max <= boost::integer_traits<Int>::const_max, "bounds_checked_int max fits in type");
  typedef Int int_type;

  //default-constructible iff 0 is a member of the valid range.
  //Alternatives: use Min if 0 is invalid?
  //For compilers that can't handle this, use:
  //bounds_checked_int() : v_(check_valid_(0)) {}
  template<typename = typename boost::enable_if_c<(Min <= 0 && Max >= 0)>::type*>
  bounds_checked_int() : val_(0) {}

  //implicit conversion from builtin int types and other bounds_checked_ints
  template<typename Int2>
  bounds_checked_int(Int2 val) : val_(check_valid_(get_primitive_int(val))) {}

  //no implicit truncating conversion from floats to ints! aah! (But what if you want to request that?)
  explicit bounds_checked_int(float val) : val_(check_valid_(val)) {}
  explicit bounds_checked_int(double val) : val_(check_valid_(val)) {}
  explicit bounds_checked_int(long double val) : val_(check_valid_(val)) {}

  //(the implicitly generated copy constructor is implicit.)


  //unary operators
  bounds_checked_int operator+() const { return *this; }
  bounds_checked_int operator-() const { return bounds_checked_int(-typename bounds_checked_int_impl::checking<Int>::signed_type(val_)); }
  bounds_checked_int operator~() const { return bounds_checked_int(~val_); }

  //as an optimization, use a special check rather than the constructor which calls check_valid_.
  bounds_checked_int& operator++() { caller_error_if(val_ == Max, "bounds_checked_int overflow" ); ++val_; return *this; }
  bounds_checked_int& operator--() { caller_error_if(val_ == Min, "bounds_checked_int underflow"); --val_; return *this; }

  bounds_checked_int operator++(int) { bounds_checked_int p = *this; ++*this; return p; }
  bounds_checked_int operator--(int) { bounds_checked_int p = *this; --*this; return p; }

  operator unspecified_bool_type() const { return val_ ? &unspecified_bool_::member : nullptr; }

  int_type get() const { return val_; }

  friend inline std::ostream& operator<<(std::ostream& os, bounds_checked_int i) {
    return os << i.get();
  }
  friend inline size_t hash_value(bounds_checked_int i) {
    return std::hash<Int>()(i.get());
  }
private:
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wtype-limits"
  #pragma clang diagnostic ignored "-Wtautological-compare"
  template<typename OtherType>
  int_type check_valid_(OtherType val) {
    //If Max or Min doesn't fit into OtherType, then clearly val cannot exceed that boundary
    //(and the check code would be incorrect, so we can and *must* not do it).
    static const bool other_is_floating_point = !std::numeric_limits<OtherType>::is_exact;
    caller_correct_if(other_is_floating_point
                        ? (val <= OtherType(Max) && Int(val) <= Max)
                        : (Int(OtherType(Max)) == Max && (Max >= Int(0)) == (OtherType(Max) >= OtherType(0)))
                          ? (val <= OtherType(Max))
                          : (Max >= Int(0)),
                      "bounds_checked_int overflow");
    caller_correct_if(other_is_floating_point
                        ? (val >= OtherType(Min) && Int(val) >= Min)
                        : (Int(OtherType(Min)) == Min && (Min >= Int(0)) == (OtherType(Min) >= OtherType(0)))
                          ? (val >= OtherType(Min))
                          : (Min < Int(0)),
                      "bounds_checked_int underflow");
    return static_cast<int_type>(val);
  }
  #pragma GCC diagnostic pop
  int_type val_;
};

namespace std {
  template<typename Int, Int Min, Int Max>
  struct hash< bounds_checked_int<Int,Min,Max> > {
    inline size_t operator()(bounds_checked_int<Int,Min,Max> const& i) const {
      return hash_value(i);
    }
  };
}

// How to implement numeric_limits most correctly with non-default bounds?
// Especially as something that is supposed to only check code, never change
// its behaviour.
// Leaving unimplemented for now.
#if 0
namespace std {
  // is_modulo: always false?
  // min(), max(), digits, digits10
  template<typename Int> struct numeric_limits< bounds_checked_int<Int> > : numeric_limits<Int> {};
}
namespace boost {
  template<typename Int> struct   make_signed< bounds_checked_int<Int> > { typedef bounds_checked_int<typename   make_signed<Int>::type> type; };
  template<typename Int> struct make_unsigned< bounds_checked_int<Int> > { typedef bounds_checked_int<typename make_unsigned<Int>::type> type; };
}
#endif

// Checks that the value fits into the target type.
template<typename Target, typename AnyInt>
inline Target get_primitive(AnyInt a) {
  return bounds_checked_int<Target>(a).get();
}

template<typename Int>
struct get_primitive_int_type {
  typedef Int type;
};
template<typename Int, Int Min, Int Max>
struct get_primitive_int_type< bounds_checked_int<Int,Min,Max> > {
  typedef Int type;
};


template<typename Int1, Int1 Min1, Int1 Max1, typename Int2, Int2 Min2, Int2 Max2>
struct common_bounds_checked_int :
  boost::mpl::if_c<
    (std::numeric_limits<Int1>::is_signed == std::numeric_limits<Int2>::is_signed),
    bounds_checked_int<
      typename boost::mpl::if_c<(std::numeric_limits<Int1>::digits > std::numeric_limits<Int2>::digits), Int1, Int2>::type,
      (Min1 < Min2 ? Min1 : Min2),
      (Max1 > Max2 ? Max1 : Max2)>,
    void> {};


// Binary operations.
// Binary operations on two bounds_checked_ints must be the same signedness
// and convert to the larger of the two int types and more generous of their min and max.
// Binary operations on a bounds_checked_int and another type require the other type
// to be the same sign as and no bigger than the bounds_checked_int.

#define INT(...) __VA_ARGS__
#define BOOL(...) bool

#define BOUNDS_CHECKED_INT_BIN_OP(opname, checking_type_name, return_type_macro) \
template<typename Int1, Int1 Min1, Int1 Max1, typename Int2, Int2 Min2, Int2 Max2> \
inline return_type_macro(typename common_bounds_checked_int<Int1,Min1,Max1,Int2,Min2,Max2>::type) \
operator opname(bounds_checked_int<Int1,Min1,Max1> a, bounds_checked_int<Int2,Min2,Max2> b) { \
  typedef typename common_bounds_checked_int<Int1,Min1,Max1,Int2,Min2,Max2>::type result_type; \
  typedef typename bounds_checked_int_impl::checking<Int1,Int2>::checking_type_name checking_type; \
  return return_type_macro(result_type)(checking_type(a.get()) opname checking_type(b.get())); } \
template<typename Int, Int Min, Int Max, typename Int2> \
inline typename boost::enable_if<bounds_checked_int_impl::superior_to<Int, Int2>, return_type_macro(bounds_checked_int<Int,Min,Max>) >::type \
operator opname(bounds_checked_int<Int,Min,Max> a, Int2 b) { \
  typedef bounds_checked_int<Int,Min,Max> result_type; \
  typedef typename bounds_checked_int_impl::checking<Int,Int2>::checking_type_name checking_type; \
  return return_type_macro(result_type)(checking_type(a.get()) opname checking_type(b)); } \
template<typename Int, Int Min, Int Max, typename Int2> \
inline typename boost::enable_if<bounds_checked_int_impl::superior_to<Int, Int2>, return_type_macro(bounds_checked_int<Int,Min,Max>) >::type \
operator opname(Int2 a, bounds_checked_int<Int,Min,Max> b) { \
  typedef bounds_checked_int<Int,Min,Max> result_type; \
  typedef typename bounds_checked_int_impl::checking<Int2,Int>::checking_type_name checking_type; \
  return return_type_macro(result_type)(checking_type(a) opname checking_type(b.get())); }

#define BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(opname, checking_type_name) \
BOUNDS_CHECKED_INT_BIN_OP(opname, checking_type_name, INT) \
template<typename Int1, Int1 Min1, Int1 Max1, typename Int2, Int2 Min2, Int2 Max2> \
inline typename boost::enable_if<bounds_checked_int_impl::superior_to<Int1, Int2>, bounds_checked_int<Int1,Min1,Max1> >::type& \
operator opname##=(bounds_checked_int<Int1,Min1,Max1>& a, bounds_checked_int<Int2,Min2,Max2> b) { \
  return a = a opname b; } \
template<typename Int, Int Min, Int Max, typename Int2> \
inline typename boost::enable_if<bounds_checked_int_impl::superior_to<Int, Int2>, bounds_checked_int<Int,Min,Max> >::type \
operator opname##=(bounds_checked_int<Int,Min,Max>& a, Int2 b) { \
  return a = a opname b; } \

BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(+, type)
BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(-, signed_type)
BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(*, type)
BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(/, type)
BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(%, type)
BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(&, bit_math::type)
BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(|, bit_math::type)
BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN(^, bit_math::type)

BOUNDS_CHECKED_INT_BIN_OP(==, comparison::type, BOOL)
BOUNDS_CHECKED_INT_BIN_OP(!=, comparison::type, BOOL)
BOUNDS_CHECKED_INT_BIN_OP(> , comparison::type, BOOL)
BOUNDS_CHECKED_INT_BIN_OP(< , comparison::type, BOOL)
BOUNDS_CHECKED_INT_BIN_OP(<=, comparison::type, BOOL)
BOUNDS_CHECKED_INT_BIN_OP(>=, comparison::type, BOOL)

#undef INT
#undef BOOL
#undef BOUNDS_CHECKED_INT_BIN_OP
#undef BOUNDS_CHECKED_INT_BIN_OP_WITH_ASSIGN

template<typename Int, Int Min, Int Max, typename IntAny>
inline bounds_checked_int<Int,Min,Max>
operator<<(bounds_checked_int<Int,Min,Max> a, IntAny shift) {
  typedef bounds_checked_int<Int,Min,Max> result_type;
  bounds_checked_int_impl::check_shift_amount_valid<Int>(shift);
  return result_type(typename bounds_checked_int_impl::checking<Int>::type(a.get()) << get_primitive_int(shift));
}
template<typename Int, Int Min, Int Max, typename IntAny>
inline bounds_checked_int<Int,Min,Max>
operator>>(bounds_checked_int<Int,Min,Max> a, IntAny shift) {
  typedef bounds_checked_int<Int,Min,Max> result_type;
  bounds_checked_int_impl::check_shift_amount_valid<Int>(shift);
  return result_type(a.get() >> get_primitive_int(shift));
}
template<typename Int, Int Min, Int Max, typename IntAny>
inline bounds_checked_int<Int,Min,Max>&
operator<<=(bounds_checked_int<Int,Min,Max>& a, IntAny shift) {
  return a = a << shift;
}
template<typename Int, Int Min, Int Max, typename IntAny>
inline bounds_checked_int<Int,Min,Max>&
operator>>=(bounds_checked_int<Int,Min,Max>& a, IntAny shift) {
  return a = a >> shift;
}

// overloading and calling the utils.hpp one
uint32_t i64sqrt(uint64_t i);
inline bounds_checked_int<uint32_t> i64sqrt(bounds_checked_int<uint64_t> i) {
  return i64sqrt(i.get());
}

namespace std {
template<typename Int, Int Min, Int Max>
inline bounds_checked_int<Int,Min,Max>
abs(bounds_checked_int<Int,Min,Max> i) {
  return (i < 0) ? -i : i;
}
}


#endif
