/*

    Copyright Eli Dupree and Isaac Dupree, 2013

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

#ifndef LASERCAKE_UNITS_HPP__
#define LASERCAKE_UNITS_HPP__

#include <ostream>
#include <boost/utility/enable_if.hpp>
#include <boost/ratio/ratio.hpp>
#include <boost/ratio/ratio_io.hpp>

#include "utils.hpp"

// Principle: all units replaced with int(1) and identity plain int types should
// get the same results; this is just a checking mechanism.
// Overflow/underflow and undefined behavior are forbidden.
// Oh hmm neat I can rely on bounds_checked_int to enforce signed/unsigned
// although the error messages will be ugly.

typedef int32_t unit_exponent_type;

template<
  typename Ratio, // a boost::ratio
  // or should we name the template parameters m, g, s, A, K, for nicer error msgs?
  // Is there any need to support non-integer exponents?
  unit_exponent_type Tau, // 2pi, listed here because
  // it can't be expressed as a ratio and because the mathematically natural
  // unit of angle is the radian yet it's more important to us to be able to
  // represent 1 full circle as an exact number.
  unit_exponent_type Meter,
  unit_exponent_type Gram,
  unit_exponent_type Second,
  unit_exponent_type Ampere,
  unit_exponent_type Kelvin
>
struct units {
  typedef units<boost::ratio<Ratio::den, Ratio::num>,
    -Tau, -Meter, -Gram, -Second, -Ampere, -Kelvin> reciprocal_type;

  typedef Ratio ratio;
  static const unit_exponent_type tau = Tau;
  static const unit_exponent_type meter = Meter;
  static const unit_exponent_type gram = Gram;
  static const unit_exponent_type second = Second;
  static const unit_exponent_type ampere = Ampere;
  static const unit_exponent_type kelvin = Kelvin;
};
typedef units<boost::ratio<1>, 0, 0, 0, 0, 0, 0> trivial_units;

template<typename UnitsA, typename UnitsB>
struct multiply_units {
  typedef units<
            typename boost::ratio_multiply<typename UnitsA::ratio,
                                           typename UnitsB::ratio>::type,
            UnitsA::tau    + UnitsB::tau,
            UnitsA::meter  + UnitsB::meter,
            UnitsA::gram   + UnitsB::gram,
            UnitsA::second + UnitsB::second,
            UnitsA::ampere + UnitsB::ampere,
            UnitsA::kelvin + UnitsB::kelvin>
          type;
};
template<typename UnitsA, typename UnitsB>
struct divide_units {
  typedef units<
            typename boost::ratio_divide<typename UnitsA::ratio,
                                         typename UnitsB::ratio>::type,
            UnitsA::tau    - UnitsB::tau,
            UnitsA::meter  - UnitsB::meter,
            UnitsA::gram   - UnitsB::gram,
            UnitsA::second - UnitsB::second,
            UnitsA::ampere - UnitsB::ampere,
            UnitsA::kelvin - UnitsB::kelvin>
          type;
};

template<
  typename Int, //the base type that this mimics.
  // Imagine multiplying the numeric value of that int by all of the below
  // in order to get the conceptual value of the contained number.
  typename Units //a 'units'
> 
class unit {
private:
  struct unspecified_bool_{int member; private:unspecified_bool_();};
  typedef int unspecified_bool_::* unspecified_bool_type;
public:
  typedef typename lasercake_int<Int>::type base_type;
private:
  base_type val_;
public:

#if 0
  // trivial: dimensionless and unscaled; conceptually equivalent
  // (and implicitly convertable with) the underlying type.
  static const bool has_physical_dimension =
    !(Meter == 0 && Gram == 0 && Second == 0
      && Ampere == 0 && Kelvin == 0);
  static const bool is_trivial =
    boost::ratio_equal<Ratio, boost::ratio<1>>::value
    && Tau == 0 && !has_physical_dimension;
 //TODO static_assert ratio is normalized? do i need that currently? nope.
#endif

  typedef unit<Int, typename Units::reciprocal_type> reciprocal_type;

  template<typename OtherInt>
  struct rebase {
    typedef unit<typename get_primitive_int_type<OtherInt>::type, Units> type;
  };
  // This partial specialization allows SFINAE to exclude the
  // operator*(unit, AnyInt) and similar overloads when their "AnyInt"
  // is actually not an int but a unit.
  template<typename AnyInt, typename AnyUnits>
  struct rebase<unit<AnyInt, AnyUnits>>;

  // Default-construction is the same as the base type.
  unit() = default;

  // (Implicit copy and move construction and assignment.)

  // Implicit conversion from plain ints to the dimensionless, unscaled
  // 'unit' type, and vice versa.
  // (oh hmm the vice versa will make comparisons/ops with plain ints ambiguous, darn)
//  template<typename = typename boost::enable_if_c<is_trivial>::type*>
//  unit(Int i) : val_(i) {}
//  template<typename = typename boost::enable_if_c<is_trivial>::type*>
//  operator Int() { return val_; }
#if 0
  // Implicit conversion from plain ints to the dimensionless, unscaled
  // 'unit' type,
  template<typename AnInt>
  unit(AnInt i,
       typename boost::enable_if_c<is_trivial>::type* = 0
  ) : val_(i) {}
  // and vice versa.
  operator
  //will implicit conversions work??
  //friend inline auto operator+(unit a, unit b) -> decltype(a+b)
#endif
#if 0
  // the only time it's useful, with * and /, compiler can't
  // work out *which* unit<> instantiation to try!
  // so i think this means overloading * and /.
  template<typename AnInt>
  unit(AnInt i//,
       //typename boost::enable_if_c<is_trivial, AnInt>::type* = 0
  ) : val_(i) {}
#endif

  // Implicit conversion from unit with same dimensions but smaller
  // representation type.
  template<typename SmallerInt>
  unit(unit<SmallerInt, Units> a,
       typename boost::enable_if_c<
           bounds_checked_int_impl::superior_to<Int, SmallerInt>::value
         >::type* = 0) : val_(a.val_) {}

  operator unspecified_bool_type() const { return val_ ? &unspecified_bool_::member : nullptr; }

  friend inline std::ostream& operator<<(std::ostream& os, unit a) {
    os << a.val_;
    //also nvm utf8 for now although tau and exponents would benefit
    // oh micro- prefix hmm. (mu)
    if(!boost::ratio_equal<typename Units::ratio, boost::ratio<1>>::value) {
      os << '*' << boost::ratio_string<typename Units::ratio, char>::short_name();
    }
    if(Units::tau) { os << '*' << "tau"; if(Units::tau != 1) { os << '^' << Units::tau; } }
    if(Units::gram) { os << '*' << 'g'; if(Units::gram != 1) { os << '^' << Units::gram; } }
    if(Units::meter) { os << '*' << 'm'; if(Units::meter != 1) { os << '^' << Units::meter; } }
    if(Units::kelvin) { os << '*' << 'K'; if(Units::kelvin != 1) { os << '^' << Units::kelvin; } }
    if(Units::ampere) { os << '*' << 'A'; if(Units::ampere != 1) { os << '^' << Units::ampere; } }
#if 0
    // short or programmable output? hm. i'll go with short
    // Note that e.g. k(m^2) and (km)^2 are different
    // and I am outputing the former!! hm so:
    if(!boost::ratio_equal<Ratio, boost::ratio<1>>::value) {
      os << boost::ratio_string<Ratio, char>::short_name();
    }
#endif
#if 0
    if(!is_trivial) {
      os << boost::ratio_string<Ratio, char>::long_name();
      if(Tau) {
        *tau
      }
    }
#endif
    return os;
  }

  // No ++ or -- since they reference the non-dimensional constant 1.
  // No bitwise ops currently; I'm not sure if they're meaningful here.
  // Of the many operators, only * and / have the ability to modify
  // dimensions.
  friend inline unit operator+(unit a) { return a; }
  friend inline unit operator-(unit a) { return construct_(-a.val_); }
  friend inline unit abs(unit a) { return (a.val_ < 0) ? -a : a; }
  friend inline size_t hash_value(unit a) { return std::hash<base_type>()(a.val_); }
  friend inline unit operator+(unit a, unit b) { return construct_(a.val_ + b.val_); }
  friend inline unit operator-(unit a, unit b) { return construct_(a.val_ - b.val_); }
  friend inline unit operator%(unit a, unit b) { return construct_(a.val_ % b.val_); }
  friend inline bool operator==(unit a, unit b) { return construct_(a.val_ == b.val_); }
  friend inline bool operator!=(unit a, unit b) { return construct_(a.val_ != b.val_); }
  friend inline bool operator>(unit a, unit b) { return construct_(a.val_ > b.val_); }
  friend inline bool operator<(unit a, unit b) { return construct_(a.val_ < b.val_); }
  friend inline bool operator<=(unit a, unit b) { return construct_(a.val_ <= b.val_); }
  friend inline bool operator>=(unit a, unit b) { return construct_(a.val_ >= b.val_); }
  template<typename AnyInt>
  friend inline unit operator<<(unit a, AnyInt shift) { return construct_(a.val_ << shift); }
  template<typename AnyInt>
  friend inline unit operator>>(unit a, AnyInt shift) { return construct_(a.val_ >> shift); }

  friend inline unit& operator+=(unit& a, unit b) { a.val_ += b.val_; return a; }
  friend inline unit& operator-=(unit& a, unit b) { a.val_ -= b.val_; return a; }
  friend inline unit& operator%=(unit& a, unit b) { a.val_ %= b.val_; return a; }
  template<typename AnyInt>
  friend inline unit& operator<<=(unit& a, AnyInt shift) { a.val_ <<= shift; return a; }
  template<typename AnyInt>
  friend inline unit& operator>>=(unit& a, AnyInt shift) { a.val_ >>= shift; return a; }

  // Hmm, esp. for * and / members below, what if AnyInt is a bigger int than the current type.
  // TODO.
  template<typename AnyInt>
  friend inline unit& operator*=(unit& a, AnyInt factor) { a.val_ *= factor; return a; }
  template<typename AnyInt>
  friend inline unit& operator/=(unit& a, AnyInt divisor) { a.val_ /= divisor; return a; }

  // These signatures are complicated so that the result type will
  // take the larger representation size of the two argument types,
  // as it would (via implicit conversions) when combining two unit<>
  // types.
  template<typename AnyInt>
  friend inline auto operator*(unit a, AnyInt factor)
  -> typename rebase<decltype(base_type() * factor)>::type
  { return rebase<decltype(a.val_ * factor)>::type::construct_(a.val_ * factor); }
  template<typename AnyInt>
  friend inline auto operator*(AnyInt factor, unit b)
  -> typename rebase<decltype(factor * base_type())>::type
  { return rebase<decltype(factor * b.val_)>::type::construct_(factor * b.val_); }
  template<typename AnyInt>
  friend inline auto operator/(unit a, AnyInt divisor)
  -> typename rebase<decltype(base_type() / divisor)>::type
  { return rebase<decltype(a.val_ / divisor)>::type::construct_(a.val_ / divisor); }
  template<typename AnyInt>
  friend inline auto operator/(AnyInt dividend, unit b)
  -> typename rebase<decltype(dividend / base_type())>::type::reciprocal_type
  { return rebase<decltype(dividend / b.val_)>::type::reciprocal_type::construct_(dividend / b.val_); }


  friend struct unit_muldiv;

private:
  static inline unit construct_(base_type i) {
    unit result;
    result.val_ = i;
    return result;
  }
};

// impl:
struct unit_muldiv {
  template<typename Unit>
  struct reduce {
    typedef Unit type;
    static inline type construct(typename type::base_type i) { return type::construct_(i); }
  };
  template<typename Int>
  struct reduce<unit<Int, trivial_units>> {
    typedef typename unit<Int, trivial_units>::base_type type;
    static inline type construct(type i) { return i; }
  };

  template<typename Int, typename UnitsA, typename UnitsB>
  struct mul {
    typedef unit<Int, typename multiply_units<UnitsA, UnitsB>::type> unit_type;
    typedef typename reduce<unit_type>::type result_type;
    static inline result_type multiply(unit<Int, UnitsA> a, unit<Int, UnitsB> b) {
      return reduce<unit_type>::construct(a.val_ * b.val_);
    }
    static inline result_type be(unit<Int, UnitsA> a) {
      return reduce<unit_type>::construct(a.val_);
    }
    template<typename T> static inline result_type be(T a) {
      return reduce<unit_type>::construct(a);
    }
  };

  template<typename Int, typename UnitsA, typename UnitsB>
  struct div {
    typedef unit<Int, typename divide_units<UnitsA, UnitsB>::type> unit_type;
    typedef typename reduce<unit_type>::type result_type;
    static inline result_type divide(unit<Int, UnitsA> a, unit<Int, UnitsB> b) {
      return reduce<unit_type>::construct(a.val_ / b.val_);
    }
    static inline result_type be(unit<Int, UnitsA> a) {
      return reduce<unit_type>::construct(a.val_);
    }
    template<typename T> static inline result_type be(T a) {
      return reduce<unit_type>::construct(a);
    }
  };
};

template<typename Int, typename UnitsA, typename UnitsB>
inline typename unit_muldiv::mul<Int, UnitsA, UnitsB>::result_type
operator*(unit<Int, UnitsA> a, unit<Int, UnitsB> b) {
  return unit_muldiv::mul<Int, UnitsA, UnitsB>::multiply(a, b);
}


template<typename Int, typename UnitsA, typename UnitsB>
inline typename unit_muldiv::div<Int, UnitsA, UnitsB>::result_type
operator/(unit<Int, UnitsA> a, unit<Int, UnitsB> b) {
  return unit_muldiv::div<Int, UnitsA, UnitsB>::divide(a, b);
}


template<typename Int, typename Units>
struct get_primitive_int_type< unit<Int, Units> > { typedef Int type; };

template<typename T>
struct get_units {
  typedef trivial_units type;
};
template<typename Int, typename Units>
struct get_units< unit<Int, Units> > {
  typedef Units type;
};

template<
  typename Ratio,
  unit_exponent_type Tau,
  unit_exponent_type Meter,
  unit_exponent_type Gram,
  unit_exponent_type Second,
  unit_exponent_type Ampere,
  unit_exponent_type Kelvin
>
struct get_units< units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin> > {
  typedef units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin> type;
};

template<
  typename T,
  typename Ratio,
  unit_exponent_type Tau,
  unit_exponent_type Meter,
  unit_exponent_type Gram,
  unit_exponent_type Second,
  unit_exponent_type Ampere,
  unit_exponent_type Kelvin
>
inline typename unit_muldiv::mul<
  typename get_primitive_int_type<T>::type,
  typename get_units<T>::type,
  units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin>
>::result_type
operator*(T a, units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin>) {
  return unit_muldiv::mul<
    typename get_primitive_int_type<T>::type,
    typename get_units<T>::type,
    units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin>
  >::be(a);
}

template<
  typename T,
  typename Ratio,
  unit_exponent_type Tau,
  unit_exponent_type Meter,
  unit_exponent_type Gram,
  unit_exponent_type Second,
  unit_exponent_type Ampere,
  unit_exponent_type Kelvin
>
inline typename unit_muldiv::div<
  typename get_primitive_int_type<T>::type,
  typename get_units<T>::type,
  units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin>
>::result_type
operator/(T a, units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin>) {
  return unit_muldiv::div<
    typename get_primitive_int_type<T>::type,
    typename get_units<T>::type,
    units<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin>
  >::be(a);
}

//class coordinate 
#endif
