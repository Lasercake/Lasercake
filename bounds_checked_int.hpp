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

#ifndef LASERCAKE_BOUNDS_CHECKED_INT_HPP__
#define LASERCAKE_BOUNDS_CHECKED_INT_HPP__

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/make_signed.hpp>
#include <boost/type_traits/make_unsigned.hpp>
#include <boost/integer_traits.hpp>

#include <ostream>

#include "utils.hpp"

namespace bounds_checked_int_impl {
template<typename Int> struct twice_as_big_int;
template<> struct twice_as_big_int<int32_t> { typedef int64_t type; typedef int64_t signed_type; typedef uint64_t unsigned_type; };
template<> struct twice_as_big_int<uint32_t> { typedef uint64_t type; typedef int64_t signed_type; typedef uint64_t unsigned_type; };
template<> struct twice_as_big_int<int64_t> { typedef __int128_t type; typedef __int128_t signed_type; typedef __uint128_t unsigned_type; };
template<> struct twice_as_big_int<uint64_t> { typedef __uint128_t type; typedef __int128_t signed_type; typedef __uint128_t unsigned_type; };
}

template<
  typename Int,
  Int Min = boost::integer_traits<Int>::const_min,
  Int Max = boost::integer_traits<Int>::const_max
>
class bounds_checked_int {
private:
  static_assert(Min <= Max, "bounds_checked_int min less than max (sanity check)");
  typedef typename bounds_checked_int_impl::twice_as_big_int<Int>::type checking_type;
  typedef typename bounds_checked_int_impl::twice_as_big_int<Int>::signed_type signed_ch_typ;
  typedef typename bounds_checked_int_impl::twice_as_big_int<Int>::unsigned_type unsigned_ch_typ;
  typedef typename boost::make_signed<Int>::type signed_type;
  typedef typename boost::make_unsigned<Int>::type unsigned_type;
  struct unspecified_bool_{int member;};
  typedef int unspecified_bool_::* unspecified_bool_type;
  
  template<typename Int2>
  struct strictly_superior_than {
    static const bool value =
          std::numeric_limits<Int>::is_signed  == std::numeric_limits<Int2>::is_signed &&
          std::numeric_limits<Int>::radix      == std::numeric_limits<Int2>::radix &&
          std::numeric_limits<Int>::is_integer <= std::numeric_limits<Int2>::is_integer &&
          std::numeric_limits<Int>::digits     >= std::numeric_limits<Int2>::digits;
  };
public:
  typedef Int int_type;

  //implicit conversions
  //or should conversions from larger-sized types be a compile-time failure?
  //no, that'll just make people static_cast and not get the bounds checking
  //unless we make those 'explicit'...hmm...is that possible? no, the smaller ones would still be implicit conversions aaah. warnings??
// bounds_checked_int(signed_type val)   : v_(check_valid_(signed_ch_typ(val))) {}
//  bounds_checked_int(unsigned_type val) : v_(check_valid_(checking_type(val))) {}
  //explicit bounds_checked_int(int_type val) : v_(check_valid_(val)) {}
  explicit bounds_checked_int(checking_type val) : v_(check_valid_(val)) {}

  //(the implicitly generated copy constructor is implicit.)

  template<typename Int2, Int2 Min2, Int2 Max2>
  //implicit conversion
  bounds_checked_int(bounds_checked_int<Int2, Min2, Max2> o,
                     typename boost::enable_if< strictly_superior_than<Int2> >::type* = 0)
  : v_(check_valid_(o.get())) {}

  template<typename Int2, Int2 Min2, Int2 Max2>
  explicit
  bounds_checked_int(bounds_checked_int<Int2, Min2, Max2> o,
                     typename boost::disable_if< strictly_superior_than<Int2> >::type* = 0)
  : v_(check_valid_(o.get())) {}

  bounds_checked_int operator+(bounds_checked_int o) const { return bounds_checked_int(checking_type(v_) + checking_type(o.v_)); }
  bounds_checked_int operator-(bounds_checked_int o) const { return bounds_checked_int(signed_ch_typ(v_) - signed_ch_typ(o.v_)); }
  bounds_checked_int operator*(bounds_checked_int o) const { return bounds_checked_int(checking_type(v_) * checking_type(o.v_)); }
  bounds_checked_int operator/(bounds_checked_int o) const { return bounds_checked_int(checking_type(v_) / checking_type(o.v_)); }
  bounds_checked_int operator%(bounds_checked_int o) const { return bounds_checked_int(checking_type(v_) % checking_type(o.v_)); }
  
  bounds_checked_int operator&(bounds_checked_int o) const { return bounds_checked_int(checking_type(v_) & checking_type(o.v_)); }
  bounds_checked_int operator|(bounds_checked_int o) const { return bounds_checked_int(checking_type(v_) | checking_type(o.v_)); }
  bounds_checked_int operator^(bounds_checked_int o) const { return bounds_checked_int(checking_type(v_) ^ checking_type(o.v_)); }

  bounds_checked_int operator+() const { return *this; }
  bounds_checked_int operator-() const { return bounds_checked_int(-signed_ch_typ(v_)); }
  bounds_checked_int operator~() const { return bounds_checked_int(~v_); }

  //as an optimization,
  bounds_checked_int& operator++() { caller_error_if(v_ == Max, "bounds_checked_int overflow"); ++v_; return *this; }
  bounds_checked_int& operator--() { caller_error_if(v_ == Min, "bounds_checked_int overflow"); --v_; return *this; }
  
  bounds_checked_int operator<<(    signed_type    o) const { check_shift_amount_valid_(o); return bounds_checked_int(checking_type(v_) << o); }
  bounds_checked_int operator<<(  unsigned_type    o) const { check_shift_amount_valid_(o); return bounds_checked_int(checking_type(v_) << o); }
  bounds_checked_int operator<<(    signed_ch_typ  o) const { check_shift_amount_valid_(o); return bounds_checked_int(checking_type(v_) << o); }
  bounds_checked_int operator<<(  unsigned_ch_typ  o) const { check_shift_amount_valid_(o); return bounds_checked_int(checking_type(v_) << o); }
  bounds_checked_int operator<<(bounds_checked_int o) const { check_shift_amount_valid_(o); return bounds_checked_int(checking_type(v_) << o.v_); }
  
  bounds_checked_int operator>>(    signed_type    o) const { check_shift_amount_valid_(o); return bounds_checked_int(v_ >> o); }
  bounds_checked_int operator>>(  unsigned_type    o) const { check_shift_amount_valid_(o); return bounds_checked_int(v_ >> o); }
  bounds_checked_int operator>>(    signed_ch_typ  o) const { check_shift_amount_valid_(o); return bounds_checked_int(v_ >> o); }
  bounds_checked_int operator>>(  unsigned_ch_typ  o) const { check_shift_amount_valid_(o); return bounds_checked_int(v_ >> o); }
  bounds_checked_int operator>>(bounds_checked_int o) const { check_shift_amount_valid_(o); return bounds_checked_int(v_ >> o.v_); }

  bool operator==(bounds_checked_int o) const { return v_ == o.v_; }
  bool operator!=(bounds_checked_int o) const { return v_ != o.v_; }
  bool operator> (bounds_checked_int o) const { return v_ >  o.v_; }
  bool operator< (bounds_checked_int o) const { return v_ <  o.v_; }
  bool operator>=(bounds_checked_int o) const { return v_ >= o.v_; }
  bool operator<=(bounds_checked_int o) const { return v_ <= o.v_; }

  bounds_checked_int& operator+=(bounds_checked_int o) { *this = operator+(o); return *this; }
  bounds_checked_int& operator-=(bounds_checked_int o) { *this = operator-(o); return *this; }
  bounds_checked_int& operator*=(bounds_checked_int o) { *this = operator*(o); return *this; }
  bounds_checked_int& operator/=(bounds_checked_int o) { *this = operator/(o); return *this; }
  bounds_checked_int& operator%=(bounds_checked_int o) { *this = operator%(o); return *this; }
  bounds_checked_int& operator&=(bounds_checked_int o) { *this = operator&(o); return *this; }
  bounds_checked_int& operator|=(bounds_checked_int o) { *this = operator|(o); return *this; }
  bounds_checked_int& operator^=(bounds_checked_int o) { *this = operator^(o); return *this; }
  
  bounds_checked_int& operator<<=(    signed_type    o) { *this = operator<<(o); return *this; }
  bounds_checked_int& operator<<=(  unsigned_type    o) { *this = operator<<(o); return *this; }
  bounds_checked_int& operator<<=(    signed_ch_typ  o) { *this = operator<<(o); return *this; }
  bounds_checked_int& operator<<=(  unsigned_ch_typ  o) { *this = operator<<(o); return *this; }
  bounds_checked_int& operator<<=(bounds_checked_int o) { *this = operator<<(o); return *this; }
  
  bounds_checked_int& operator>>=(    signed_type    o) { *this = operator>>(o); return *this; }
  bounds_checked_int& operator>>=(  unsigned_type    o) { *this = operator>>(o); return *this; }
  bounds_checked_int& operator>>=(    signed_ch_typ  o) { *this = operator>>(o); return *this; }
  bounds_checked_int& operator>>=(  unsigned_ch_typ  o) { *this = operator>>(o); return *this; }
  bounds_checked_int& operator>>=(bounds_checked_int o) { *this = operator>>(o); return *this; }

  bounds_checked_int operator++(int) { bounds_checked_int p = *this; operator++(); return p; }
  bounds_checked_int operator--(int) { bounds_checked_int p = *this; operator--(); return p; }

  operator unspecified_bool_type() const { return v_ ? &unspecified_bool_::member : nullptr; }
  
  int_type get() const { return v_; }
private:
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wtype-limits"
  //These templates are only instantiated a fixed few number of times for each instantiation of bounds_checked_int,
  //so it shouldn't slow down compilation too much (fingers crossed!).
  template<typename OtherType>
  int_type check_valid_(OtherType val) {
    if(int_type(OtherType(Max)) == Max) {caller_error_if(val > OtherType(Max), "bounds_checked_int overflow");}
    if(int_type(OtherType(Min)) == Min) {caller_error_if(val < OtherType(Min), "bounds_checked_int underflow");}
    return static_cast<int_type>(val);
  }
  template<typename ShiftAmountType>
  void check_shift_amount_valid_(ShiftAmountType shift) {
    // Such shifts do not work as you'd mathematically expect on some CPUs' bare metal,
    // and bounds_checked_int is meant to be used in a correctly removable way so we cannot
    // emulate them.  (Also, it would make bounds checking more work.)
    caller_error_if(shift >= ShiftAmountType(std::numeric_limits<Int>::digits), "bounds_checked_int invalid too-large shift");
    caller_error_if(shift < ShiftAmountType(0), "bounds_checked_int invalid negative-amount shift");
  }
  #pragma GCC diagnostic pop
  int_type v_;
};

template<typename Int, Int Min, Int Max>
std::ostream& operator<<(std::ostream& os, bounds_checked_int<Int,Min,Max> i) {
  return os << i.get();
}

#endif
