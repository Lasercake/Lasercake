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
#include <sstream>
#include <utility>
#include <boost/utility/enable_if.hpp>
#include <boost/ratio/ratio.hpp>
#include <boost/integer.hpp>
#include <boost/type_traits/conditional.hpp>
#include "utils.hpp"

// === units ===
//
// This module is a mechanism you can use to cause yourself type errors
// when your code has variables with certain physical dimensions or constant
// factors and you try something nonsensical (like
// adding meters to seconds).[1]
//
// Instead of using, say, int32_t, you can use
//   physical_quantity<int32_t, meters_t>.
// Most operations on a physical_quantity type require the dimensions to
// remain the same (the numeric type follows the numeric promotions of the
// base type[2]).  Multiplication and division multiply or divide the quantities'
// unit types; e.g. dividing meters by seconds gives you a quantity
// with unit type meters per second.
//
// == creation and extraction ==
// You create a variable of a unit type by calling make(42, meters) or
// multiplying (42 * meters).  You retrieve a value 'distance' for use with
// code that can't handle physical_quantity types by calling
// get(distance, meters) or dividing (distance / meters).
//
// == unit types ==
//
// The units argument of physical_quantity<> is an instantiation of units<>.
// units<> is a variadic template taking a sequence of dimension kinds
// from the "dim" namespace (see documentation below if you want to add a
// dimension kind or use these directly).  An example unit is
//   typedef units<dim::ratio<1000>, dim::kilogram<1>, dim::meter<2>,
//                                            dim::second<-2>>
//       kilonewtons_t;
// A selection of common units is provided near the end of this file;
// look for "[FRIENDLY UNITS]".
//
// [1] Principle: replacing "units<>" variables with "1" and physical_quantity
// types with their underlying numeric type should not change code behavior.
// This is just a checking mechanism.
// Overflow/underflow and undefined behavior are forbidden.
//
// [2] For convenience, because you probably don't want modulo
// behavior in units, physical_quantity wraps the numeric type in
// lasercake_int which may bounds_checked_int the contained type.
// We may make this configurable in the future.



// Oh hmm neat I can rely on bounds_checked_int to enforce signed/unsigned
// although the error messages will be ugly.

// units<> * T not supported, but T * units<> is.  physical_quantity<> can be on either side.

// TODO document more

namespace dim {
  enum dimension_kind_tag {
    ratio_tag = 1,

    // === Irrational constant factors ===
    // tau: 2pi, listed here because it can't be expressed as a ratio
    // and because the mathematically natural unit of angle is the radian
    // yet it's more important to us to be able to represent 1 full circle
    // as an exact number.
    // https://en.wikipedia.org/wiki/Turn_%28geometry%29
    tau_tag,

    // === Basic physical dimensions ===
    // are ordered roughly in the order that they
    // commonly appear when written out (e.g. kg m/s^2 orders them kg, m, s).

    // Why kg not g? Derived units like Newton are based on it,
    // so it seemed more readable in that way, but it's a tradeoff
    // that could be made in either direction.
    kilogram_tag,

    meter_tag,
    second_tag,
    ampere_tag,
    kelvin_tag,

    // === Other ===

    // pseudovectors, pseudoscalars: sign dependent on space's arbitrary chirality
    pseudo_tag
  };

  // A dimension-kind is a mathematical group.
  // A sensible one, for example, is {meter^N | N <- integers}.
  // It is usually Abelian (symmetric).
  // It preferably has roots defined for meaningful (element, root) combinations;
  // e.g. some physical quantities can meaningfully have a square root
  // of them taken.
  //
  //
  // To make a new basic physical dimension:
  //
  // You must add to the dimension_kind_tag enumeration.  Example: your_tag.
  //
  // You must create a struct your_dim_kind.  Example:
  //   template<intmax_t Arg> struct your_dim_kind
  //     : dimension_kind_base<your_dim_kind<Arg>, your_tag> { ... };
  //
  // If your dimension resembles a physical dimension (something
  // that is not like anything else and can be multiplied by itself
  // any number of times with no modulo effects etc),
  // consider creating your_dim_kind like this instead:
  //   template<intmax_t Exponent>
  //   struct your_dim_kind : basic_physical_dimension<your_dim_kind, your_tag, Exponent> {
  //     static const char* symbol() { return "Y"; }
  //   };
  //
  // In the former case:
  //     Your struct must contain:
  //        typedef ... identity; //the group's identity element
  //        typedef ... inverse; //the inverse of this element
  //     You must either implement
  //        template<typename AGroupMember> struct combine {
  //          typedef ... type; //group operation on this and AGroupMember
  //        };
  //     or, equivalently, specialize dim::combine<GroupMember1, GroupMember2>
  //     for your group.
  //
  //     If you wish to define roots, your struct must contain:
  //        template<intmax_t Root> struct root { typedef ... type; };
  //     It need not have a valid instantiation for all arguments of your type.
  //     If A::root<N>::type is B, then B::pow<N>::type shall be A.
  //
  //     (pow<A, N> denotes combining A with itself N times, or its inverse -N times,
  //      and is default-implemented based on combine, inverse and identity.
  //      You can override it if you want, but there's probably no reason to.
  //      Note that you should handle pow<A, Num, Den> if you handle roots, as
  //      the default implementation intelligently delegates to root<> if Den is non-one.)
  //
  //     If you wish your dimension to be writable to ostreams, you must
  //     implement that.  Example:
  //        friend inline std::ostream& operator<<(std::ostream& os, your_dim_kind) {
  //          return os << "Y" << '^' << Arg;
  //        }
  //
  // Finally, you must specialize identity<your_tag>.  Example:
  //  template<> struct identity<your_tag>  { typedef typename your_dim_kind<0>::identity type; };

  template<dimension_kind_tag Tag> struct identity;

  template<typename A, typename B>
  struct combine : A::template combine<B> {};

  template<typename A, intmax_t Exponent, bool Odd = (Exponent % 2 == 1)>
  struct default_pow_impl;
  template<typename A> struct default_pow_impl<A, 0, false>;
  template<typename A> struct default_pow_impl<A, 1, true> { typedef A type; };
  template<typename A> struct default_pow_impl<A, 2, false> : combine<A, A> {};
  template<typename A, intmax_t EvenExponent> struct default_pow_impl<A, EvenExponent, false> {
    typedef typename default_pow_impl<A, EvenExponent/2>::type intermediate;
    typedef typename combine<intermediate, intermediate>::type type;
  };
  template<typename A, intmax_t OddExponent> struct default_pow_impl<A, OddExponent, true> {
    typedef typename default_pow_impl<A, OddExponent/2>::type intermediate;
    typedef typename combine<A, typename combine<intermediate, intermediate>::type>::type type;
  };

  template<typename A, intmax_t Exponent,
    typename Identity = void, typename Inverse = void,
    bool NonnegativeExp = (Exponent >= 0)>
  struct default_pow : default_pow_impl<A, Exponent> {};
  
  template<typename A, typename Identity, typename Inverse>
  struct default_pow<A, 0, Identity, Inverse, true> { typedef Identity type; };

  template<typename A, intmax_t Exponent, typename Identity, typename Inverse>
  struct default_pow<A, Exponent, Identity, Inverse, false> :
    default_pow<Inverse, -Exponent, Identity, A> {};

  template<typename A>
  struct is_identity : boost::is_same<A, typename A::identity> {};

  template<typename Derived, dimension_kind_tag Tag>
  struct dimension_kind_base {
    static const dimension_kind_tag tag = Tag;
    //typedef typename Derived::template combine<typename Derived::inverse>::type identity;
    //static const bool is_identity = boost::is_same<Derived, identity>::value;
    template<intmax_t Exponent, intmax_t Root = 1> struct pow {
      typedef typename Derived::template root<Root>::type intermediate;
      typedef typename default_pow<intermediate, Exponent,
        typename intermediate::identity, typename intermediate::inverse>::type type;
    };
    template<intmax_t Exponent> struct pow<Exponent, 1> :
      default_pow<Derived, Exponent, typename Derived::identity, typename Derived::inverse> {};
//    template<intmax_t Exponent> struct pow : default_pow<Derived, Exponent,
//      typename Derived::identity, typename Derived::inverse> {};
    template<typename Other> struct combine : dim::combine<Derived, Other> {};
  };

  //fails in GCC < 4.7:
  //template<char...C> struct str { static constexpr char value[] = { C..., '\0' }; };

  // The strings are notionally a compile-time constant value;
  // can we make them be one?

  // UTF-8?  Tau and exponents would benefit.
  // We might also get a non-ASCII character if ratio
  // is 1/1000000 (micro-, mu).  Do our ostreams and terminals
  // get the encoding right?
  // Conciseness?
  // Note that e.g. k(m^2) and (km)^2 are different
  // and this output represents the former!

  static const intmax_t max_intmax_t = boost::integer_traits<intmax_t>::const_max;
  static const intmax_t min_intmax_t = boost::integer_traits<intmax_t>::const_min;

  // Is there any need to support non-integer exponents?
  template<template<intmax_t> class Derived, dimension_kind_tag Tag, intmax_t Exponent>
  struct basic_physical_dimension : dimension_kind_base<Derived<Exponent>, Tag> {
    static const intmax_t exponent = Exponent;

    typedef Derived<0> identity;
    typedef Derived<-Exponent> inverse;
    template<typename Other> struct combine;
    template<intmax_t OtherExponent> struct combine<Derived<OtherExponent>> {
      static_assert(OtherExponent <= 0 ||
        max_intmax_t - (OtherExponent <= 0 ? 0 : OtherExponent) >= Exponent, "overflow");
      static_assert(OtherExponent >= 0 ||
        min_intmax_t - (OtherExponent >= 0 ? 0 : OtherExponent) <= Exponent, "underflow");

      typedef Derived<Exponent + OtherExponent> type;
    };
    template<intmax_t Root> struct root {
      static_assert(Exponent % Root == 0,
        "physical dimensions do not presently support fractional powers of base units");
      typedef Derived<Exponent/Root> type;
    };
    friend inline std::ostream& operator<<(std::ostream& os, basic_physical_dimension) {
      return os << Derived<Exponent>::symbol() << '^' << Exponent;
    }
  };
  // or should we name the template parameters m, g, s, A, K, for nicer error msgs?
  template<intmax_t Exponent> struct meter : basic_physical_dimension<
    meter, meter_tag, Exponent>   { static const char* symbol() { return "m"; } };
  template<intmax_t Exponent> struct kilogram : basic_physical_dimension<
    kilogram, kilogram_tag, Exponent>{ static const char* symbol() { return "kg"; } };
  template<intmax_t Exponent> struct second : basic_physical_dimension<
    second, second_tag, Exponent> { static const char* symbol() { return "a"; } };
  template<intmax_t Exponent> struct ampere : basic_physical_dimension<
    ampere, ampere_tag, Exponent> { static const char* symbol() { return "A"; } };
  template<intmax_t Exponent> struct kelvin : basic_physical_dimension<
    kelvin, kelvin_tag, Exponent> { static const char* symbol() { return "K"; } };
  template<intmax_t Exponent> struct tau : basic_physical_dimension<
    tau, tau_tag, Exponent>       { static const char* symbol() { return "tau"; } };

  template<bool Pseudo>
  struct pseudo : dimension_kind_base<pseudo<Pseudo>, pseudo_tag> {
    static const bool pseudoness = Pseudo;

    typedef pseudo<false> identity;
    typedef pseudo inverse;
    template<typename Other> struct combine;
    template<bool OtherPseudo> struct combine<pseudo<OtherPseudo>> {
      typedef pseudo<Pseudo ^ OtherPseudo> type;
    };
    template<intmax_t Root> struct root {
      static_assert(Root % 2 == 1 || !Pseudo,
                    "Even roots of possibly-negative numbers may be imaginary.");
      typedef pseudo type;
    };
    friend inline std::ostream& operator<<(std::ostream& os, pseudo) {
      if(Pseudo) { os << "[pseudo]"; } else { os << "[nonpseudo]"; }
      return os;
    }
  };

  template<intmax_t Num, intmax_t Den>
  inline std::ostream& show_ratio(std::ostream& os) {
    typedef boost::ratio<Num, Den> ratio;
    static const bool negative = (ratio::num < 0);
    static const intmax_t positive_num = (negative ? -ratio::num : ratio::num);
    typedef extract_factor<10, positive_num> num_tens;
    typedef extract_factor<10, ratio::den> den_tens;
    static const bool do_pow10 = (num_tens::factor_exponent + den_tens::factor_exponent) >= 4;
    static const int pow10_exp = num_tens::factor_exponent - den_tens::factor_exponent;
    static const intmax_t numB = (do_pow10 ? num_tens::rest_of_factoree : positive_num);
    static const intmax_t denB = (do_pow10 ? den_tens::rest_of_factoree : ratio::den);
    typedef extract_factor<2, numB> num_twos;
    typedef extract_factor<2, denB> den_twos;
    static const bool do_pow2 = (num_twos::factor_exponent + den_twos::factor_exponent) >= 11;
    static const int pow2_exp = num_twos::factor_exponent - den_twos::factor_exponent;
    static const intmax_t numC = (do_pow2 ? num_twos::rest_of_factoree : numB);
    static const intmax_t denC = (do_pow2 ? den_twos::rest_of_factoree : denB);
    static const bool brackets = // all the time except a specific case
      !(!negative && numC == 1 && denC == 1 && (do_pow10 + do_pow2 == 1));
    bool star = false;
    if(brackets) { os << '['; }
    if(negative) { os << '-'; }
    if(numC != 1 || denC != 1) { if(star) {os << '*';}; os << numC; star = true; }
    if(denC != 1) { os << '/' << denC; star = true; }
    if(do_pow2) { if(star) {os << '*';}; os << "2^" << pow2_exp; star = true;}
    if(do_pow10) { if(star) {os << '*';}; os << "10^" << pow10_exp; star = true;}
    if(!star) { os << '1'; }
    if(brackets) { os << ']'; }
    return os;
  }
  //ratio? factor? rational_factor?
  template<intmax_t Num, intmax_t Den = 1>
  struct ratio : dimension_kind_base<ratio<Num, Den>, ratio_tag> {
    static const intmax_t num = Num;
    static const intmax_t den = Den;
    typedef boost::ratio<Num, Den> boost_ratio;
    static_assert(Num == boost_ratio::num && Den == boost_ratio::den, "not normalized");

    typedef ratio<1> identity;
    typedef ratio<Den, Num> inverse;
    template<typename Other> struct combine;
    template<intmax_t OtherNum, intmax_t OtherDen> struct combine<ratio<OtherNum, OtherDen>> {
      typedef typename boost::ratio_multiply<
          boost_ratio, typename ratio<OtherNum, OtherDen>::boost_ratio
        >::type combined_boost_ratio;
      typedef ratio<combined_boost_ratio::num, combined_boost_ratio::den> type;
    };
    template<intmax_t Root> struct root {
      static_assert(Root % 2 == 1 || num >= 0,
                    "Even roots of negative numbers are imaginary.");
      typedef static_root_nonnegative_integer<(num >= 0 ? num : -num), Root> num_root;
      typedef static_root_nonnegative_integer<den, Root> den_root;
      static_assert(num_root::remainder == 0, "non-exact dim::ratio exponentiation");
      static_assert(den_root::remainder == 0, "non-exact dim::ratio exponentiation");
      typedef ratio<(num >= 0 ? num_root::value : -num_root::value), den_root::value> type;
    };

    friend inline std::ostream& operator<<(std::ostream& os, ratio) {
      show_ratio<Num, Den>(os);
      return os;
    }
  };
  // make_ratio normalizes rather than requiring you to.
  template<intmax_t Num, intmax_t Den>
  struct make_ratio {
    typedef boost::ratio<Num, Den> boost_ratio;
    typedef ratio<boost_ratio::num, boost_ratio::den> type;
  };

  template<> struct identity<ratio_tag>  { typedef typename  ratio<0>::identity type; };
  template<> struct identity<tau_tag>    { typedef typename    tau<0>::identity type; };
  template<> struct identity<kilogram_tag>{ typedef typename kilogram<0>::identity type; };
  template<> struct identity<meter_tag>  { typedef typename  meter<0>::identity type; };
  template<> struct identity<second_tag> { typedef typename second<0>::identity type; };
  template<> struct identity<ampere_tag> { typedef typename ampere<0>::identity type; };
  template<> struct identity<kelvin_tag> { typedef typename kelvin<0>::identity type; };
  template<> struct identity<pseudo_tag> { typedef typename pseudo<0>::identity type; };

} /* end namespace dim */





// TODO UNITS make sure all the abs/sign/imbue_sign have consistent
// result units with each other.

// Skip down a page for more API; mostly tedious forward declarations here.
template<typename...DimensionKind> struct units;
template<typename Num, typename Units> class physical_quantity;

template<typename Num>
struct physical_quantity_representation_type {
  typedef typename lasercake_int<Num>::type type;
};

typedef units<> trivial_units;
template<typename Units> struct is_trivial_units : boost::false_type {};
template<> struct is_trivial_units<trivial_units> : boost::true_type {};


template<typename T> struct get_units;
template<dim::dimension_kind_tag Tag, typename T> struct get_dimension_kind;
template<typename... T> struct units_prod;

namespace units_impl {
  template<typename Units> struct show_units_impl;
  template<typename T> struct get_units_impl;
  template<dim::dimension_kind_tag Tag, typename Units> struct get_dimension_kind_impl2;
  template<typename UnitsA, typename UnitsB> struct multiply_units;
  template<typename UnitsA, typename UnitsB> struct divide_units;

  template<typename Units> struct verify_units;
  template<> struct verify_units<units<>> { static const bool value = true; };
  template<typename DimKind>
  struct verify_units<units<DimKind>> {
    static const bool value = !dim::is_identity<DimKind>::value;
  };
  template<typename DimKind1, typename DimKind2, typename...DimKinds>
  struct verify_units<units<DimKind1, DimKind2, DimKinds...>> {
    static const bool value =
      !dim::is_identity<DimKind1>::value
      && DimKind1::tag < DimKind2::tag
      && verify_units<units<DimKind2, DimKinds...>>::value;
  };
  template<typename Units> struct is_units : boost::false_type {};
  template<typename...U> struct is_units<units<U...>> : boost::true_type {};
  // The implicit conversion to Units isn't useful on the type level, but
  // if you're constructing one in an expression it means you can skip the
  // "typename" "::type" parts.
  template<typename Units>
  struct units_result {
    typedef Units type;
    operator Units()const { return Units(); }
  };
} /* end namespace units_impl */

// get_units has
// ::type   the units<> type affiliated with the given type
//           (affiliations include being a dimension-kind, a units<>,
//            a physical_quantity's unit, a meta-function with ::type
//            having to do with units, and a scalar non-unit type)
// ::representation_type
//          the numeric type representing this (if any)
// ::is_nonunit_type      boolean
// ::is_nonunit_scalar    boolean
template<typename T>
struct get_units : units_impl::get_units_impl<T> {};

// Operations that transform one units type to a related units type:
// units_pow<units, pow[, root]>
// units_root<units, root>
// units_recip<units>
// get_sign_components_of_units<units>
// get_non_sign_components_of_units<units>
// units_prod<units...>
//    ::type
//
// units_prod is a swiss army knife of unit combination.
// Provide any number of unit-like types and its ::type is the
// units<> that is the product of all the units they represent.

template<typename... Unitses> struct units_prod;

template<typename Units, intmax_t Num, intmax_t Den = 1> struct units_pow;
template<typename... DimKind, intmax_t Num, intmax_t Den>
struct units_pow<units<DimKind...>, Num, Den> : units_impl::units_result<
  units<typename DimKind::template pow<Num, Den>::type...>> {};
template<typename... DimKind, intmax_t Den>
struct units_pow<units<DimKind...>, 0, Den> : units_impl::units_result<units<>> {};

template<typename Units, intmax_t Root = 2> struct units_root;
template<typename... DimKind, intmax_t Root>
struct units_root<units<DimKind...>, Root> : units_impl::units_result<
  units<typename DimKind::template root<Root>::type...>> {};

template<typename Units> struct units_recip;
template<typename... DimKind>
struct units_recip<units<DimKind...>> : units_impl::units_result<
  units<typename DimKind::inverse...>> {};

template<typename Units>
struct get_sign_components_of_units : units_impl::units_result<
  typename units_prod<
    dim::pseudo<get_dimension_kind<dim::pseudo_tag, Units>::type::pseudoness>,
    dim::ratio<(get_dimension_kind<dim::ratio_tag, Units>::type::num < 0 ? -1 : 1)>
  >::type> {};

template<typename Units>
struct get_non_sign_components_of_units : units_impl::units_result<
  typename units_prod<
    Units,
    typename units_recip<typename get_sign_components_of_units<Units>
  ::type>::type>::type> {};

// value-level units<> reciprocal function:
template<typename... DimKind>
inline constexpr units<typename DimKind::inverse...>
reciprocal(units<DimKind...>) { return units<typename DimKind::inverse...>(); }

template<typename...DimensionKind>
struct units {
  static_assert(units_impl::verify_units<units>::value,
    "Type arguments of units are in the wrong order and/or contain an identity element and/or duplicate dimension kinds.");

  // value-level units<> pow function.
  // The only reason pow is a member function is that the exponent
  // has to be a template argument, and as a non-member function
  // the template arguments (the exponent) would have to come before
  // the units, which is a confusing order for an exponent.
  template<intmax_t Num, intmax_t Den = 1>
  static constexpr
  typename units_pow<units, Num, Den>::type
  pow() {
    return typename units_pow<units, Num, Den>::type();
  }

  // A units is positive and equal to itself.
  friend inline bool operator==(units, units) { return true; }
  friend inline bool operator!=(units, units) { return false; }

  friend inline bool operator==(units, decltype(nullptr)) { return false; }
  friend inline bool operator!=(units, decltype(nullptr)) { return true; }
  friend inline bool operator==(decltype(nullptr), units) { return false; }
  friend inline bool operator!=(decltype(nullptr), units) { return true; }

  friend inline bool operator>(units, decltype(nullptr)) { return true; }
  friend inline bool operator>=(units, decltype(nullptr)) { return true; }
  friend inline bool operator<(decltype(nullptr), units) { return true; }
  friend inline bool operator<=(decltype(nullptr), units) { return true; }
  friend inline bool operator<(units, decltype(nullptr)) { return false; }
  friend inline bool operator<=(units, decltype(nullptr)) { return false; }
  friend inline bool operator>(decltype(nullptr), units) { return false; }
  friend inline bool operator>=(decltype(nullptr), units) { return false; }

  friend inline units abs(units a) { return a; }
  friend inline int sign(units) { return 1; }

  friend inline std::ostream& operator<<(std::ostream& os, units) {
    units_impl::show_units_impl<units>::show(os);
    return os;
  }
};

// Given a kind of dimension you're interested in,
// returns a dim::* item describing that kind of dimension
// in this type.
template<dim::dimension_kind_tag Tag, typename Units>
struct get_dimension_kind :
  units_impl::get_dimension_kind_impl2<Tag, typename get_units<Units>::type> {};

// We specialize this template that's from bounds_checked_int:
template<typename Num, typename Units>
struct get_primitive_int_type< physical_quantity<Num, Units> > { typedef Num type; };
template<typename...U>
struct get_primitive_int_type< units<U...> > { typedef void type; };

// Using this template lets you provide a non-primitive Num type
// (it unwraps the type) and makes dimensionless quantities simply
// be the base numeric type (which is what we intend to do).
template<typename Num, typename Units>
struct make_physical_quantity_type {
  typedef physical_quantity<typename get_primitive_int_type<Num>::type, Units> type;
  static inline type construct(typename type::base_type i) { return type(i, Units()); }
};
template<typename Num>
struct make_physical_quantity_type<Num, units<>> {
  typedef typename physical_quantity_representation_type<typename get_primitive_int_type<Num>::type>::type type;
  static inline type construct(type i) { return i; }
};

// The suggested way to make and retrieve physical quantity values.
template<typename Num>
inline Num make(Num i, units<>) { return i; }
template<typename Num>
inline Num get(Num i, units<>) { return i; }
template<typename Num, typename...U>
inline physical_quantity<typename get_primitive_int_type<Num>::type, units<U...>>
make(Num i, units<U...> u) {
  return physical_quantity<typename get_primitive_int_type<Num>::type, units<U...>>(i, u);
}
// (get(physical_quantity<>) defined in-class)

template<
  typename Num, //the base type that this mimics.
  // Imagine multiplying the numeric value of that int by all of the below
  // in order to get the conceptual value of the contained number.
  typename Units //a 'units'
> 
class physical_quantity {
public:
  // helpful typedefs
  typedef typename physical_quantity_representation_type<Num>::type base_type;
  typedef Units units;

  template<typename OtherNum>
  struct rebase {
    typedef physical_quantity<typename get_primitive_int_type<OtherNum>::type, Units> type;
  };

  typedef physical_quantity<Num, typename units_recip<Units>::type> reciprocal_type;

private:
  base_type val_;
  typedef physical_quantity this_t;
  static_assert(units_impl::is_units<Units>::value, "Units must be of type units<..>");
public:

  // Default-construction is the same as the base type.
  physical_quantity() = default;

  // Implicit conversion from literal 0,
  // because zero is meaningful at every unit.
  physical_quantity(decltype(nullptr)) : val_(0) {}

  // If you provide the correct units, you're free to construct one
  // out of a number.
  physical_quantity(base_type i, Units) : val_(i) {}
  // Or to retrieve one.
  base_type get(Units)const { return val_; }
  friend inline base_type get(physical_quantity a, Units u) { return a.get(u); }

  // (Implicit copy and move construction and assignment.)


  // Implicit conversion from physical_quantity with same dimensions but
  // smaller representation type.
  template<typename SmallerNum>
  physical_quantity(physical_quantity<SmallerNum, Units> a,
       typename boost::enable_if_c<
           bounds_checked_int_impl::superior_to<Num, SmallerNum>::value
         >::type* = 0) : val_(a.get(Units())) {}

  // Explicit conversion from physical_quantity with same dimensions but
  // representation type that does not convert losslessly.
  template<typename BiggerNum>
  explicit physical_quantity(physical_quantity<BiggerNum, Units> a,
       typename boost::disable_if_c<
           bounds_checked_int_impl::superior_to<Num, BiggerNum>::value
         >::type* = 0) : val_(a.get(Units())) {}


  explicit operator bool() const { return bool(val_); }

  friend inline std::ostream& operator<<(std::ostream& os, physical_quantity a) {
    os << a.val_ << '*' << Units();
    return os;
  }

  // No ++ or -- since they reference the non-dimensional constant 1.
  // No bitwise ops currently; I'm not sure if they're meaningful here.
  // Of the many operators, only * and / have the ability to modify
  // dimensions.
  friend inline this_t operator+(this_t a) { return a; }
  friend inline this_t operator-(this_t a) { return this_t(-a.val_, Units()); }
  friend inline this_t abs(this_t a) { return (a.val_ < 0) ? -a : a; }
  friend inline
  typename make_physical_quantity_type<Num, typename units_prod<dim::pseudo<
    get_dimension_kind<dim::pseudo_tag, Units>::type::pseudoness>>::type>::type
  sign(this_t a) { return sign(a.val_) * typename units_prod<dim::pseudo<
    get_dimension_kind<dim::pseudo_tag, Units>::type::pseudoness>>::type(); }
  friend inline size_t hash_value(this_t a) { return std::hash<base_type>()(a.val_); }
  friend inline this_t operator+(this_t a, this_t b) { return this_t(a.val_ + b.val_, Units()); }
  friend inline this_t operator-(this_t a, this_t b) { return this_t(a.val_ - b.val_, Units()); }
  friend inline this_t operator%(this_t a, this_t b) { return this_t(a.val_ % b.val_, Units()); }
  friend inline bool operator==(this_t a, this_t b) { return a.val_ == b.val_; }
  friend inline bool operator!=(this_t a, this_t b) { return a.val_ != b.val_; }
  friend inline bool operator>(this_t a, this_t b) { return a.val_ > b.val_; }
  friend inline bool operator<(this_t a, this_t b) { return a.val_ < b.val_; }
  friend inline bool operator<=(this_t a, this_t b) { return a.val_ <= b.val_; }
  friend inline bool operator>=(this_t a, this_t b) { return a.val_ >= b.val_; }
  template<typename AnyInt>
  friend inline this_t operator<<(this_t a, AnyInt shift) { return this_t(a.val_ << shift, Units()); }
  template<typename AnyInt>
  friend inline this_t operator>>(this_t a, AnyInt shift) { return this_t(a.val_ >> shift, Units()); }

  friend inline this_t& operator+=(this_t& a, this_t b) { a.val_ += b.val_; return a; }
  friend inline this_t& operator-=(this_t& a, this_t b) { a.val_ -= b.val_; return a; }
  friend inline this_t& operator%=(this_t& a, this_t b) { a.val_ %= b.val_; return a; }
  template<typename AnyInt>
  friend inline this_t& operator<<=(this_t& a, AnyInt shift) { a.val_ <<= shift; return a; }
  template<typename AnyInt>
  friend inline this_t& operator>>=(this_t& a, AnyInt shift) { a.val_ >>= shift; return a; }

  // Hmm, esp. for * and / members below, what if AnyNum is a bigger type than the current type.
  // TODO.
  template<typename AnyNum>
  friend inline this_t& operator*=(this_t& a, AnyNum factor) { a.val_ *= factor; return a; }
  template<typename AnyNum>
  friend inline this_t& operator/=(this_t& a, AnyNum divisor) { a.val_ /= divisor; return a; }

  // These signatures are complicated so that the result type will
  // take the larger representation size of the two argument types,
  // as it would (via implicit conversions) when combining two
  // physical_quantity<> types.
  template<typename AnyNum>
  friend inline
  typename rebase<decltype(
    std::declval<base_type>() *
     std::declval<typename boost::enable_if_c<get_units<AnyNum>::is_nonunit_scalar, AnyNum>::type>()
  )>::type
  operator*(this_t a, AnyNum factor)
  { return typename rebase<decltype(a.val_ * factor)>::type(a.val_ * factor, Units()); }

  template<typename AnyNum>
  friend inline
  typename rebase<decltype(
    std::declval<typename boost::enable_if_c<get_units<AnyNum>::is_nonunit_scalar, AnyNum>::type>()
     * std::declval<base_type>()
  )>::type
  operator*(AnyNum factor, this_t b)
  { return typename rebase<decltype(factor * b.val_)>::type(factor * b.val_, Units()); }

  template<typename AnyNum>
  friend inline
  typename rebase<decltype(
    std::declval<base_type>()
     / std::declval<typename boost::enable_if_c<get_units<AnyNum>::is_nonunit_scalar, AnyNum>::type>()
  )>::type
  operator/(this_t a, AnyNum divisor)
  { return typename rebase<decltype(a.val_ / divisor)>::type(a.val_ / divisor, Units()); }

  template<typename AnyNum>
  friend inline
  typename rebase<decltype(
    std::declval<typename boost::enable_if_c<get_units<AnyNum>::is_nonunit_scalar, AnyNum>::type>()
     / std::declval<base_type>()
  )>::type::reciprocal_type
  operator/(AnyNum dividend, this_t b)
  { typedef typename rebase<decltype(dividend / b.val_)>::type::reciprocal_type result_type;
    return result_type(dividend / b.val_, typename result_type::units()); }
};

// More multiplication and division defined later; the signatures
// are very tedious.


// imbue_sign() multiplies the sign of arg 1 into the (signed) value of arg 2.
template<typename T1, typename T2>
inline //TODO write physical_quantity_prod<>?
typename make_physical_quantity_type<
  typename get_units<T2>::representation_type,
  typename units_prod<
    get_units<T2>,
    dim::pseudo<get_dimension_kind<dim::pseudo_tag, T1>::type::pseudoness>,
    dim::ratio<(get_dimension_kind<dim::ratio_tag, T1>::type::num < 0 ? -1 : 1)>
  >::type
>::type
imbue_sign(T1 signum, T2 base_val) {
  static_assert(get_dimension_kind<dim::ratio_tag, T1>::type::num != 0, "imbuing an ambiguous sign");
  caller_error_if(signum == 0, "imbuing an ambiguous sign");
  return
    ((signum < 0) ? -base_val : base_val)
    * typename units_prod<
        dim::pseudo<get_dimension_kind<dim::pseudo_tag, T1>::type::pseudoness>,
        dim::ratio<(get_dimension_kind<dim::ratio_tag, T1>::type::num < 0 ? -1 : 1)>
      >::type();
}

// helper
template<intmax_t N>
struct identity_units {
  typedef physical_quantity<
      typename boost::int_max_value_t<(N >= 0 ? N : ~N)>::least,
      units<dim::ratio<1, N>>
    > type;
};

// For converting between magnitudes of the same general dimension of quantity;
// typical usage: (some quantity in kilograms) * identity(grams / kilograms)
template<intmax_t N>
inline typename identity_units<N>::type
identity(units<dim::ratio<1, N>> u) {
  return typename identity_units<N>::type(N, u);
}

// Sqrt'ing a quantity sqrts its units.
template<typename Int, typename Units>
inline physical_quantity<Int, typename units_root<Units, 2>::type>
i64sqrt(physical_quantity<Int, Units> const& radicand) {
  typedef typename units_root<Units, 2>::type sqrt_units;
  return physical_quantity<Int, sqrt_units>(i64sqrt(get(radicand, Units())), sqrt_units());
}

// helper
template<typename TypeIfWeDivided>
struct make_non_normalized_rational_physical_quantity_info {
  typedef typename get_units<TypeIfWeDivided>::representation_type int_type;
  typedef non_normalized_rational<int_type> rational_number_type;
  typedef typename get_units<TypeIfWeDivided>::type units;
  typedef typename make_physical_quantity_type<rational_number_type, units>::type type;
};

// make_non_normalized_rational_physical_quantity(numerator, denominator)
// takes a num and/or denom that are physical quantities, and produces a
//     physical_quantity<non_normalized_rational<base numeric type>, units>
// with the correct units (or just a non_normalized_rational if those units
// are dimensionless).
template<typename Num, typename Den>
inline typename
make_non_normalized_rational_physical_quantity_info<decltype(std::declval<Num>() / std::declval<Den>())>::type
make_non_normalized_rational_physical_quantity(Num num, Den den) {
  typedef make_non_normalized_rational_physical_quantity_info<decltype(std::declval<Num>() / std::declval<Den>())> info;
  return make(
    typename info::rational_number_type(
      get(num, typename get_units<Num>::type()),
      get(den, typename get_units<Den>::type())),
    typename info::units());
}
template<typename Num>
inline typename
make_non_normalized_rational_physical_quantity_info<Num>::type
make_non_normalized_rational_physical_quantity(Num num) {
  typedef make_non_normalized_rational_physical_quantity_info<Num> info;
  return make(
    typename info::rational_number_type(
      get(num, typename get_units<Num>::type()),
      typename info::int_type(1)),
    typename info::units());
}

// support std::abs
namespace std {
template<typename Num, typename Units>
inline physical_quantity<Num, Units>
abs(physical_quantity<Num, Units> a) {
  return ::abs(a);
}
}

//class coordinate?

// =========[FRIENDLY UNITS]==========
//
// === Basic units ===
typedef units<> radians_t; // the mathematically natural unit of angle
typedef units<dim::tau<1>> full_circles_t; // an often-convenient unit of angle
typedef units<dim::ratio<1, 360>, dim::tau<1>> degrees_t; // a unit of angle

typedef units<dim::kilogram<1>> kilograms_t;
typedef units<dim::ratio<1, 1000>, dim::kilogram<1>> grams_t;
typedef units<dim::meter<1>> meters_t;
typedef units<dim::second<1>> seconds_t;
typedef units<dim::ampere<1>> amperes_t;
typedef units<dim::kelvin<1>> kelvins_t;
typedef units<dim::pseudo<true>> pseudo_t;

constexpr auto full_circles = full_circles_t();
constexpr auto kilograms    = kilograms_t();
constexpr auto grams        = grams_t();
constexpr auto meters       = meters_t();
constexpr auto seconds      = seconds_t();
constexpr auto amperes      = amperes_t();
constexpr auto kelvins      = kelvins_t();
constexpr auto pseudo       = pseudo_t();
constexpr auto degrees      = degrees_t();

// === Derived units ===
// For consistency in coding style, we do not capitalize unit
// names (such as Newton) that SI conventionally capitalizes.

// Parentheses around negative exponents are solely to make KDevelop
// understand better.

typedef units<dim::second<(-1)>> hertz_t;
typedef units<dim::kilogram<1>, dim::meter<1>, dim::second<(-2)>> newtons_t;
typedef units<dim::kilogram<1>, dim::meter<2>, dim::second<(-2)>> joules_t;
typedef units<dim::kilogram<1>, dim::meter<(-1)>, dim::second<(-2)>> pascals_t;
typedef units<dim::kilogram<1>, dim::meter<2>, dim::second<(-3)>> watts_t;
typedef units<dim::second<1>, dim::ampere<1>> coulombs_t;
typedef units<dim::kilogram<1>, dim::meter<2>, dim::second<(-3)>, dim::ampere<(-1)>> volts_t;
typedef units<dim::kilogram<1>, dim::meter<2>, dim::second<(-3)>, dim::ampere<(-2)>> ohms_t;


// === SI prefixes ===

typedef units<dim::ratio<1000>> kilo_t;
typedef units<dim::ratio<1000000>> mega_t;
typedef units<dim::ratio<1000000000>> giga_t;
typedef units<dim::ratio<1000000000000>> tera_t;
typedef units<dim::ratio<1000000000000000>> peta_t;
typedef units<dim::ratio<1000000000000000000>> exa_t;

typedef units<dim::ratio<1, 1000>> milli_t;
typedef units<dim::ratio<1, 1000000>> micro_t;
typedef units<dim::ratio<1, 1000000000>> nano_t;
typedef units<dim::ratio<1, 1000000000000>> pico_t;
typedef units<dim::ratio<1, 1000000000000000>> femto_t;
typedef units<dim::ratio<1, 1000000000000000000>> atto_t;

constexpr auto kilo = kilo_t();
constexpr auto mega = mega_t();
constexpr auto giga = giga_t();
constexpr auto tera = tera_t();
constexpr auto peta = peta_t();
constexpr auto exa  = exa_t();

constexpr auto milli = milli_t();
constexpr auto micro = micro_t();
constexpr auto nano  = nano_t();
constexpr auto pico  = pico_t();
constexpr auto femto = femto_t();
constexpr auto atto  = atto_t();

// === Factors of your choice ===

template<typename Ratio>
struct units_ratio_t {
  typedef units<typename dim::make_ratio<Ratio::num, Ratio::den>::type> type;
};
template<intmax_t Num, intmax_t Den = 1>
struct units_factor_t {
  typedef units<typename dim::make_ratio<Num, Den>::type> type;
};

template<intmax_t Num, intmax_t Den = 1>
constexpr inline typename units_factor_t<Num, Den>::type units_factor() {
  return typename units_factor_t<Num, Den>::type();
}
template<typename Ratio>
constexpr inline typename units_ratio_t<Ratio>::type units_factor() {
  return typename units_ratio_t<Ratio>::type();
}




///////////////////////////////////////////////////////////////////////////
//////////// EXTRA-BORING IMPLEMENTATION DETAILS BELOW HERE ///////////////
///////////////////////////////////////////////////////////////////////////


#if 0 /*TODO*/
#ifdef SKIP_UNIT_CHECKING
template<typename Num, typename Units> using units<Num, Units> = Num;
#endif
#endif


// Specialize numbers.hpp's numeric_representation_cast<>
// for the sake of e.g. vector3's dot product implementation.
template<typename Target, typename Units, typename Num>
struct numeric_representation_cast_impl<physical_quantity<Target, Units>, Num> {
  typedef typename make_physical_quantity_type<Target, typename get_units<Num>::type>::type
      target_type;
};
template<typename Target, typename Units, typename Num>
struct numeric_representation_cast_impl<Target, physical_quantity<Num, Units>> {
  typedef typename make_physical_quantity_type<typename get_units<Target>::representation_type, Units>::type
      target_type;
};
template<typename Target, typename Units, typename Num, typename Units1>
struct numeric_representation_cast_impl<physical_quantity<Target, Units1>, physical_quantity<Num, Units>> {
  typedef typename make_physical_quantity_type<Target, Units>::type target_type;
};

namespace units_impl {


template<> struct show_units_impl<units<>> {
  static void show(std::ostream& os) {
    os << "[1]"; } };
template<typename DimKind> struct show_units_impl<units<DimKind>> {
  static void show(std::ostream& os) {
    os << DimKind(); } };
template<typename DimKind, typename...DimKinds>
struct show_units_impl<units<DimKind, DimKinds...>> {
  static void show(std::ostream& os) {
    os << DimKind() << '*';
    show_units_impl<units<DimKinds...>>::show(os); } };


template<typename Units, typename Impl, bool Nonunit, bool NonunitScalar>
struct result_of_get_units : units_impl::units_result<Units> {
  typedef Impl representation_type;
  static const bool is_nonunit_type = Nonunit;
  static const bool is_nonunit_scalar = NonunitScalar;
};

// * numeric non-dimensional types
template<typename T, typename = typename boost::enable_if_c<
      std::numeric_limits<T>::is_specialized>::type>
result_of_get_units<units<>, T, true, true> overload_find_units_type(T); //unimplemented

// * metafunction containing a unit or related type:
template<typename T, typename = typename boost::enable_if_c<
      !get_units_impl<typename T::type>::is_nonunit_type>::type>
get_units_impl<typename T::type> overload_find_units_type(T); //unimplemented

// * dimension kind:
template<typename T, dim::dimension_kind_tag = T::tag, typename = typename boost::enable_if_c<
  dim::is_identity<T>::value>::type>
result_of_get_units<units<>, void, false, false> overload_find_units_type(T); //unimplemented

template<typename T, dim::dimension_kind_tag = T::tag, typename = typename boost::disable_if_c<
  dim::is_identity<T>::value>::type>
result_of_get_units<units<T>, void, false, false> overload_find_units_type(T); //unimplemented

template<typename Num, typename Units>
struct get_units_impl<physical_quantity<Num, Units>> : units_impl::units_result<Units> {
  typedef typename physical_quantity_representation_type<Num>::type representation_type;
  static const bool is_nonunit_type = false;
  static const bool is_nonunit_scalar = false;
};
template<typename...U>
struct get_units_impl<units<U...>> : units_impl::units_result<units<U...>> {
  typedef void representation_type;
  static const bool is_nonunit_type = false;
  static const bool is_nonunit_scalar = false;
};
template<typename T>
struct get_units_impl<bounds_checked_int<T>> : get_units_impl<T> {};

template<typename T>
struct get_units_impl : decltype(overload_find_units_type(std::declval<T>())) {};



template<dim::dimension_kind_tag Tag, typename DimKindQ, typename Units,
  bool Correct = (DimKindQ::tag == Tag)> struct get_dimension_kind_impl;
template<dim::dimension_kind_tag Tag, typename DimKindQ>
struct get_dimension_kind_impl<Tag, DimKindQ, units<>, false> :
  dim::identity<Tag> {};
template<dim::dimension_kind_tag Tag, typename DimKindQ, typename...DimKinds>
struct get_dimension_kind_impl<Tag, DimKindQ, units<DimKinds...>, true> {
  typedef DimKindQ type;
};
template<dim::dimension_kind_tag Tag, typename DimKindQ, typename DimKind, typename...DimKinds>
struct get_dimension_kind_impl<Tag, DimKindQ, units<DimKind, DimKinds...>, false> :
  get_dimension_kind_impl<Tag, DimKind, units<DimKinds...>> {};

template<dim::dimension_kind_tag Tag>
struct get_dimension_kind_impl2<Tag, units<>> : dim::identity<Tag> {};
template<dim::dimension_kind_tag Tag, typename DimKind, typename...DimKinds>
struct get_dimension_kind_impl2<Tag, units<DimKind, DimKinds...>> :
  get_dimension_kind_impl<Tag, DimKind, units<DimKinds...>> {};



// private; does not check for correct dim-kind ordering:
template<typename DimKind, typename Units, bool IsIdentity = dim::is_identity<DimKind>::value>
struct units_cons;
template<typename DimKind, typename...DimKinds>
struct units_cons<DimKind, units<DimKinds...>, false> {
  typedef units<DimKind, DimKinds...> type;
};
template<typename DimKind, typename...DimKinds>
struct units_cons<DimKind, units<DimKinds...>, true> {
  typedef units<DimKinds...> type;
};
// "Compare": -1 if left < right; 0 if left == right; 1 if left > right.
// DimKinds are sorted with lesser tags first.
template<int Compare, typename UnitsA, typename UnitsB> struct units_zip;
template<typename DimKind1A, typename...DimKindsA, typename DimKind1B, typename...DimKindsB>
struct units_zip<0, units<DimKind1A, DimKindsA...>, units<DimKind1B, DimKindsB...>> :
  units_cons<typename dim::combine<DimKind1A, DimKind1B>::type,
    typename multiply_units<units<DimKindsA...>, units<DimKindsB...>>::type> {};
template<typename DimKind1A, typename...DimKindsA, typename DimKind1B, typename...DimKindsB>
struct units_zip<-1, units<DimKind1A, DimKindsA...>, units<DimKind1B, DimKindsB...>> :
  units_cons<DimKind1A,
    typename multiply_units<units<DimKindsA...>, units<DimKind1B, DimKindsB...>>::type> {};
template<typename DimKind1A, typename...DimKindsA, typename DimKind1B, typename...DimKindsB>
struct units_zip<1, units<DimKind1A, DimKindsA...>, units<DimKind1B, DimKindsB...>> :
  units_cons<DimKind1B,
    typename multiply_units<units<DimKind1A, DimKindsA...>, units<DimKindsB...>>::type> {};

template<typename DimKind1A, typename...DimKindsA, typename DimKind1B, typename...DimKindsB>
struct multiply_units<units<DimKind1A, DimKindsA...>, units<DimKind1B, DimKindsB...> > :
  units_impl::units_zip<
    ((DimKind1A::tag > DimKind1B::tag) - (DimKind1A::tag < DimKind1B::tag)),
    units<DimKind1A, DimKindsA...>,  units<DimKind1B, DimKindsB...> > {};
template<typename Units> struct multiply_units<Units, units<> > { typedef Units type; };
template<typename Units> struct multiply_units<units<>, Units> { typedef Units type; };
template<> struct multiply_units<units<>, units<>> { typedef units<> type; };

template<typename UnitsA, typename UnitsB>
struct divide_units : multiply_units<UnitsA, typename units_recip<UnitsB>::type> {};

}

template<> struct units_prod<> { typedef units<> type; };
template<typename...U> struct units_prod<units<U...>> { typedef units<U...> type; };
template<typename...U, typename... Unitses> struct units_prod<units<U...>, Unitses...> {
  typedef typename units_impl::multiply_units<units<U...>, typename units_prod<Unitses...>::type>::type type;
};
template<typename UnitLike, typename... Unitses>
struct units_prod<UnitLike, Unitses...>
  : units_prod<typename get_units<UnitLike>::type, Unitses...> {};


// And now, all those multiplication and division operators!

template<typename...UA, typename...UB>
inline constexpr
typename units_impl::multiply_units<units<UA...>, units<UB...>>::type
operator*(units<UA...>, units<UB...>) {
  return typename units_impl::multiply_units<units<UA...>, units<UB...>>::type();
}

template<typename...UA, typename...UB>
inline constexpr typename units_impl::divide_units<units<UA...>, units<UB...>>::type
operator/(units<UA...>, units<UB...>) {
  return typename units_impl::divide_units<units<UA...>, units<UB...>>::type();
}



template<typename NumA, typename NumB, typename UnitsA, typename UnitsB>
inline typename
make_physical_quantity_type<
    decltype(std::declval<NumA>() * std::declval<NumB>()),
    typename units_impl::multiply_units<UnitsA, UnitsB>::type
  >::type
operator*(physical_quantity<NumA, UnitsA> a, physical_quantity<NumB, UnitsB> b) {
  return make_physical_quantity_type<
          decltype(std::declval<NumA>() * std::declval<NumB>()),
          typename units_impl::multiply_units<UnitsA, UnitsB>::type
      >::construct(a.get(UnitsA()) * b.get(UnitsB()));
}

template<typename NumA, typename NumB, typename UnitsA, typename UnitsB>
inline typename
make_physical_quantity_type<
    decltype(std::declval<NumA>() / std::declval<NumB>()),
    typename units_impl::divide_units<UnitsA, UnitsB>::type
  >::type
operator/(physical_quantity<NumA, UnitsA> a, physical_quantity<NumB, UnitsB> b) {
  return make_physical_quantity_type<
          decltype(std::declval<NumA>() / std::declval<NumB>()),
          typename units_impl::divide_units<UnitsA, UnitsB>::type
      >::construct(a.get(UnitsA()) / b.get(UnitsB()));
}




template<typename Num, typename...U>
inline
typename make_physical_quantity_type<
  typename boost::enable_if_c<get_units<Num>::is_nonunit_scalar, Num>::type,
  units<U...>
>::type
operator*(Num a, units<U...>) {
  return make_physical_quantity_type<Num, units<U...>>::construct(a);
}

template<typename Num, typename UnitsA, typename...UB>
inline typename
make_physical_quantity_type<Num, typename units_impl::multiply_units<UnitsA, units<UB...>>::type>::type
operator*(physical_quantity<Num, UnitsA> a, units<UB...>) {
  return make_physical_quantity_type<
            Num, typename units_impl::multiply_units<UnitsA, units<UB...>>::type
    >::construct(a.get(UnitsA()));
}





template<typename Num, typename...U>
inline
typename make_physical_quantity_type<
  typename boost::enable_if_c<get_units<Num>::is_nonunit_scalar, Num>::type,
  typename units_recip<units<U...>>::type
>::type
operator/(Num a, units<U...>) {
  return make_physical_quantity_type<
            Num, typename units_recip<units<U...>>::type
    >::construct(a);
}

template<typename Num, typename UnitsA, typename...UB>
inline
typename make_physical_quantity_type<
  Num, typename units_impl::divide_units<UnitsA, units<UB...>>::type
>::type
operator/(physical_quantity<Num, UnitsA> a, units<UB...>) {
  return make_physical_quantity_type<
            Num, typename units_impl::divide_units<UnitsA, units<UB...>>::type
    >::construct(a.get(UnitsA()));
}


///////////////////////////////////////////////////////////////////////////
//////////// EXTRA-BORING IMPLEMENTATION DETAILS ABOVE HERE ///////////////
///////////////////////////////////////////////////////////////////////////


#endif
