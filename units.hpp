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

#define BOOST_RATIO_EXTENSIONS

#include <ostream>
#include <sstream>
#include <utility>
#include <boost/utility/enable_if.hpp>
#include <boost/ratio/ratio.hpp>
#include <boost/ratio/ratio_io.hpp>
#include <boost/integer.hpp>
#include <boost/type_traits/conditional.hpp>

#include "utils.hpp"

// Principle: all units replaced with int(1) and identity plain int types should
// get the same results; this is just a checking mechanism.
// Overflow/underflow and undefined behavior are forbidden.
// Oh hmm neat I can rely on bounds_checked_int to enforce signed/unsigned
// although the error messages will be ugly.

// units<> * T not supported, but T * units<> is.  unit<> can be on either side.

typedef int32_t unit_exponent_type;


template<
  typename Ratio = boost::ratio<1>, // a boost::ratio
  // or should we name the template parameters m, g, s, A, K, for nicer error msgs?
  // Is there any need to support non-integer exponents?
  unit_exponent_type Tau = 0, // 2pi, listed here because
  // it can't be expressed as a ratio and because the mathematically natural
  // unit of angle is the radian yet it's more important to us to be able to
  // represent 1 full circle as an exact number.
  unit_exponent_type Meter = 0,
  unit_exponent_type Gram = 0,
  unit_exponent_type Second = 0,
  unit_exponent_type Ampere = 0,
  unit_exponent_type Kelvin = 0,
  bool Pseudo = false // pseudovectors, pseudoscalars
  //when mul/div they add mod 2, aka they xor.
  //unit_with_Pseudo pseudo_subtract(unit_without_Pseudo, unit_without_Pseudo)
  // it should be generally flipping the Pseudo, as deduced from
  // https://en.wikipedia.org/wiki/Pseudovector#Behavior_under_cross_products
  // and the crossproduct definition
  // (& what about abs()?)
> struct u_v_t {}; //units_vector_type (compile-time vector)
template<typename U_V_T> struct units;

typedef units<u_v_t<>> trivial_units;
template<typename Units> struct is_trivial_units : boost::false_type {};
template<> struct is_trivial_units<trivial_units> : boost::true_type {};


template<typename BaseRatio, typename ExponentRatio>
struct static_ratio_pow_nonnegative {
  typedef static_root_nonnegative_integer<BaseRatio::num, ExponentRatio::den> num_root;
  typedef static_root_nonnegative_integer<BaseRatio::den, ExponentRatio::den> den_root;
  static_assert(num_root::remainder == 0, "non-exact unit exponentiation");
  static_assert(den_root::remainder == 0, "non-exact unit exponentiation");
  static const uint64_t num = static_pow_nonnegative_integer<num_root::value, ExponentRatio::num>::value;
  static const uint64_t den = static_pow_nonnegative_integer<den_root::value, ExponentRatio::num>::value;
  typedef boost::ratio<num, den> type;
};


template<typename BaseRatio, typename ExponentRatio>
struct static_ratio_pow {
  typedef typename static_ratio_pow_nonnegative<
    typename boost::ratio_abs<BaseRatio>::type,
    typename boost::ratio_abs<ExponentRatio>::type>::type type1;
  static_assert(ExponentRatio::den % 2 == 1 || BaseRatio::num >= 0,
                "Even roots of negative numbers are imaginary.");
  typedef typename boost::conditional<(ExponentRatio::num >= 0),
    type1, boost::ratio<type1::den, type1::num> >::type type2;
  typedef typename boost::conditional<(BaseRatio::num >= 0 || ExponentRatio::num % 2 == 0),
    type2, typename boost::ratio_negate<type2>::type>::type type3;
  typedef type3 type;
};


template<
  typename Ratio,
  unit_exponent_type Tau,
  unit_exponent_type Meter,
  unit_exponent_type Gram,
  unit_exponent_type Second,
  unit_exponent_type Ampere,
  unit_exponent_type Kelvin,
  bool Pseudo
>
struct units<u_v_t<Ratio, Tau, Meter, Gram, Second, Ampere, Kelvin, Pseudo> > {
  typedef Ratio ratio;
  static const unit_exponent_type tau = Tau;
  static const unit_exponent_type meter = Meter;
  static const unit_exponent_type gram = Gram;
  static const unit_exponent_type second = Second;
  static const unit_exponent_type ampere = Ampere;
  static const unit_exponent_type kelvin = Kelvin;
  static const bool pseudo = Pseudo;

  static const bool nonidentity_ratio = !boost::ratio_equal<ratio, boost::ratio<1>>::value;
  static const bool nontrivial_units = !is_trivial_units<units>::value;

  typedef units<u_v_t<boost::ratio<ratio::den, ratio::num>,
    -tau, -meter, -gram, -second, -ampere, -kelvin, pseudo> > reciprocal_type;
  static constexpr reciprocal_type reciprocal() { return reciprocal_type(); }

  template<intmax_t Num, intmax_t Den = 1>
  struct units_pow {
    typedef boost::ratio<Num, Den> exponent;
    static const intmax_t num = exponent::num;
    static const intmax_t den = exponent::den;
    static_assert(tau    % den == 0, "units<> does not presently support fractional powers of base units");
    static_assert(meter  % den == 0, "units<> does not presently support fractional powers of base units");
    static_assert(gram   % den == 0, "units<> does not presently support fractional powers of base units");
    static_assert(second % den == 0, "units<> does not presently support fractional powers of base units");
    static_assert(ampere % den == 0, "units<> does not presently support fractional powers of base units");
    static_assert(kelvin % den == 0, "units<> does not presently support fractional powers of base units");
    static_assert(pseudo % den == 0, "units<> does not presently support fractional powers of base units");
    typedef units<u_v_t<
        typename static_ratio_pow<ratio, exponent>::type,
        tau*num/den, meter*num/den, gram*num/den,
        second*num/den, ampere*num/den, kelvin*num/den,
        (pseudo*num/den) & 1> >
      type;
  };
  template<intmax_t Num, intmax_t Den = 1>
  static constexpr
  typename units::template units_pow<Num, Den>::type
  pow() {
    return typename units::template units_pow<Num, Den>::type();
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

  // The string is notionally a compile-time constant value;
  // can we make it be one?

  // UTF-8?  Tau and exponents would benefit.
  // We might also get a non-ASCII character if ratio
  // is 1/1000000 (micro-, mu).  Do our ostreams and terminals
  // get the encoding right?
  // Conciseness?
  // Note that e.g. k(m^2) and (km)^2 are different
  // and this output represents the former!
  static std::string suffix_repr() {
    std::stringstream os;
    if(nonidentity_ratio) {
      os << '*' << boost::ratio_string<ratio, char>::short_name();
    }
    if(tau) { os << '*' << "tau"; if(tau != 1) { os << '^' << tau; } }
    if(gram) { os << '*' << 'g'; if(gram != 1) { os << '^' << gram; } }
    if(meter) { os << '*' << 'm'; if(meter != 1) { os << '^' << meter; } }
    if(kelvin) { os << '*' << 'K'; if(kelvin != 1) { os << '^' << kelvin; } }
    if(ampere) { os << '*' << 'A'; if(ampere != 1) { os << '^' << ampere; } }
    if(pseudo) { os << "[pseudo]"; }
    return os.str();
  }
  static std::string repr() {
    if(nontrivial_units) {
      return suffix_repr().substr(1);
    }
    else {
      return "[1]";
    }
  }
  friend inline std::ostream& operator<<(std::ostream& os, units) {
    return os << repr();
  }
};

template<typename UnitsA, typename UnitsB>
struct multiply_units {
  typedef units<u_v_t<
            typename boost::ratio_multiply<typename UnitsA::ratio,
                                           typename UnitsB::ratio>::type,
            UnitsA::tau    + UnitsB::tau,
            UnitsA::meter  + UnitsB::meter,
            UnitsA::gram   + UnitsB::gram,
            UnitsA::second + UnitsB::second,
            UnitsA::ampere + UnitsB::ampere,
            UnitsA::kelvin + UnitsB::kelvin,
            UnitsA::pseudo ^ UnitsB::pseudo> >
          type;
};
template<typename UnitsA, typename UnitsB>
struct divide_units {
  typedef units<u_v_t<
            typename boost::ratio_divide<typename UnitsA::ratio,
                                         typename UnitsB::ratio>::type,
            UnitsA::tau    - UnitsB::tau,
            UnitsA::meter  - UnitsB::meter,
            UnitsA::gram   - UnitsB::gram,
            UnitsA::second - UnitsB::second,
            UnitsA::ampere - UnitsB::ampere,
            UnitsA::kelvin - UnitsB::kelvin,
            UnitsA::pseudo ^ UnitsB::pseudo> >
          type;
};

template<typename Int, typename Units> class unit;

template<typename Int, typename Units>
struct get_primitive_int_type< unit<Int, Units> > { typedef Int type; };

template<typename U>
struct get_primitive_int_type< units<U> > { typedef void type; };

template<typename Int>
struct unit_representation_type {
  typedef typename lasercake_int<Int>::type type;
};

template<typename T>
struct get_units {
  typedef trivial_units type;
  typedef T representation_type;
  static const bool is_nonunit_type = true;
  // This is an approximation...
  static const bool is_nonunit_scalar = std::numeric_limits<T>::is_specialized;
};
template<typename T>
struct get_units<bounds_checked_int<T>> : get_units<T> {};

template<typename Int, typename Units>
struct get_units< unit<Int, Units> > {
  typedef Units type;
  typedef typename unit_representation_type<Int>::type representation_type;
  static const bool is_nonunit_type = false;
  static const bool is_nonunit_scalar = false;
};
template<typename U>
struct get_units< units<U> > {
  typedef units<U> type;
  typedef void representation_type;
  static const bool is_nonunit_type = false;
  static const bool is_nonunit_scalar = false;
};

template<typename... Unitses> struct units_prod;
template<> struct units_prod<> { typedef trivial_units type; };
template<typename Units> struct units_prod<Units> { typedef Units type; };
template<typename Units, typename... Unitses> struct units_prod<Units, Unitses...> {
  typedef typename multiply_units<Units, typename units_prod<Unitses...>::type>::type type;
};


template<typename Int, typename Units>
struct make_unit_type {
  typedef unit<typename get_primitive_int_type<Int>::type, Units> type;
  static inline type construct(typename type::base_type i) { return type(i, Units()); }
};
template<typename Int>
struct make_unit_type<Int, trivial_units> {
  typedef typename unit_representation_type<typename get_primitive_int_type<Int>::type>::type type;
  static inline type construct(type i) { return i; }
};



template<typename UA, typename UB>
inline constexpr
typename multiply_units<units<UA>, units<UB>>::type
operator*(units<UA>, units<UB>) {
  return typename multiply_units<units<UA>, units<UB>>::type();
}

template<typename UA, typename UB>
inline constexpr typename divide_units<units<UA>, units<UB>>::type
operator/(units<UA>, units<UB>) {
  return typename divide_units<units<UA>, units<UB>>::type();
}

template<typename T> T imaginary_copy(T arg); // unimplemented
#define UNITS(units_val) decltype(::imaginary_copy((units_val)))


template<typename Ratio>
struct units_ratio_t {
  // Re-make the ratio here to ensure that the units<> will be structurally
  // equal to all other equal units<>es (i.e. reduced to lowest terms).
  typedef units<u_v_t<boost::ratio<Ratio::num, Ratio::den>>> type;
};
template<intmax_t Num, intmax_t Den = 1>
struct units_factor_t : units_ratio_t<boost::ratio<Num, Den>> {};

template<intmax_t Num, intmax_t Den = 1>
constexpr inline typename units_factor_t<Num, Den>::type units_factor() {
  return typename units_factor_t<Num, Den>::type();
}
template<typename Ratio>
constexpr inline typename units_ratio_t<Ratio>::type units_factor() {
  return typename units_ratio_t<Ratio>::type();
}

// https://en.wikipedia.org/wiki/Turn_%28geometry%29
constexpr auto full_circles = units<u_v_t<boost::ratio<1>, 1> >();
constexpr auto meters       = units<u_v_t<boost::ratio<1>, 0, 1> >();
constexpr auto grams        = units<u_v_t<boost::ratio<1>, 0, 0, 1> >();
constexpr auto seconds      = units<u_v_t<boost::ratio<1>, 0, 0, 0, 1> >();
constexpr auto amperes      = units<u_v_t<boost::ratio<1>, 0, 0, 0, 0, 1> >();
constexpr auto kelvins      = units<u_v_t<boost::ratio<1>, 0, 0, 0, 0, 0, 1> >();

// for pseudovectors, pseudoscalars, etc:
constexpr auto pseudo       = units<u_v_t<boost::ratio<1>, 0, 0, 0, 0, 0, 0, true> >();

constexpr auto degrees      = full_circles / units_factor<1, 360>();

constexpr auto kilo = units_factor<boost::kilo>();
constexpr auto mega = units_factor<boost::mega>();
constexpr auto giga = units_factor<boost::giga>();
constexpr auto tera = units_factor<boost::tera>();
constexpr auto peta = units_factor<boost::peta>();
constexpr auto exa  = units_factor<boost::exa>();

constexpr auto milli = units_factor<boost::milli>();
constexpr auto micro = units_factor<boost::micro>();
constexpr auto nano  = units_factor<boost::nano>();
constexpr auto pico  = units_factor<boost::pico>();
constexpr auto femto = units_factor<boost::femto>();
constexpr auto atto  = units_factor<boost::atto>();


// Avoid using the macro here because it confuses my IDE (KDevelop).
typedef decltype(imaginary_copy(full_circles)) full_circles_t;
typedef decltype(imaginary_copy(meters)) meters_t;
typedef decltype(imaginary_copy(grams)) grams_t;
typedef decltype(imaginary_copy(seconds)) seconds_t;
typedef decltype(imaginary_copy(amperes)) amperes_t;
typedef decltype(imaginary_copy(kelvins)) kelvins_t;
typedef decltype(imaginary_copy(pseudo)) pseudo_t;
typedef decltype(imaginary_copy(degrees)) degrees_t;

typedef decltype(imaginary_copy(kilo)) kilo_t;
typedef decltype(imaginary_copy(mega)) mega_t;
typedef decltype(imaginary_copy(giga)) giga_t;
typedef decltype(imaginary_copy(tera)) tera_t;
typedef decltype(imaginary_copy(peta)) peta_t;
typedef decltype(imaginary_copy(exa )) exa_t;

typedef decltype(imaginary_copy(milli)) milli_t;
typedef decltype(imaginary_copy(micro)) micro_t;
typedef decltype(imaginary_copy(nano )) nano_t;
typedef decltype(imaginary_copy(pico )) pico_t;
typedef decltype(imaginary_copy(femto)) femto_t;
typedef decltype(imaginary_copy(atto )) atto_t;



template<typename Int>
inline Int make(Int i, trivial_units) { return i; }
template<typename Int>
inline Int get(Int i, trivial_units) { return i; }

template<typename Int, typename U>
inline unit<typename get_primitive_int_type<Int>::type, units<U>>
make(Int i, units<U> u) {
  return unit<typename get_primitive_int_type<Int>::type, units<U>>(i, u);
}

template<
  typename Int, //the base type that this mimics.
  // Imagine multiplying the numeric value of that int by all of the below
  // in order to get the conceptual value of the contained number.
  typename Units //a 'units'
> 
class unit {
public:
  typedef typename unit_representation_type<Int>::type base_type;
private:
  base_type val_;
public:

  typedef unit<Int, typename Units::reciprocal_type> reciprocal_type;

  template<typename OtherInt>
  struct rebase {
    typedef unit<typename get_primitive_int_type<OtherInt>::type, Units> type;
  };

  // Default-construction is the same as the base type.
  unit() = default;

  // Implicit conversion from literal 0,
  // because zero is meaningful at every unit.
  unit(decltype(nullptr)) : val_(0) {}

  // If you provide the correct units, you're free to construct one
  // out of a number.
  unit(base_type i, Units) : val_(i) {}
  // Or to retrieve one.
  base_type get(Units)const { return val_; }
  friend inline base_type get(unit a, Units u) { return a.get(u); }

  // (Implicit copy and move construction and assignment.)


  // Implicit conversion from unit with same dimensions but smaller
  // representation type.
  template<typename SmallerInt>
  unit(unit<SmallerInt, Units> a,
       typename boost::enable_if_c<
           bounds_checked_int_impl::superior_to<Int, SmallerInt>::value
         >::type* = 0) : val_(a.get(Units())) {}

  explicit operator bool() const { return bool(val_); }

  friend inline std::ostream& operator<<(std::ostream& os, unit a) {
    os << a.val_ << Units::suffix_repr();
    return os;
  }

  // No ++ or -- since they reference the non-dimensional constant 1.
  // No bitwise ops currently; I'm not sure if they're meaningful here.
  // Of the many operators, only * and / have the ability to modify
  // dimensions.
  friend inline unit operator+(unit a) { return a; }
  friend inline unit operator-(unit a) { return unit(-a.val_, Units()); }
  friend inline unit abs(unit a) { return (a.val_ < 0) ? -a : a; }
  friend inline
  typename make_unit_type<Int, typename pseudo_t::units_pow<Units::pseudo>::type>::type
  sign(unit a) { return sign(a.val_) * pseudo.pow<Units::pseudo>(); }
  friend inline size_t hash_value(unit a) { return std::hash<base_type>()(a.val_); }
  friend inline unit operator+(unit a, unit b) { return unit(a.val_ + b.val_, Units()); }
  friend inline unit operator-(unit a, unit b) { return unit(a.val_ - b.val_, Units()); }
  friend inline unit operator%(unit a, unit b) { return unit(a.val_ % b.val_, Units()); }
  friend inline bool operator==(unit a, unit b) { return a.val_ == b.val_; }
  friend inline bool operator!=(unit a, unit b) { return a.val_ != b.val_; }
  friend inline bool operator>(unit a, unit b) { return a.val_ > b.val_; }
  friend inline bool operator<(unit a, unit b) { return a.val_ < b.val_; }
  friend inline bool operator<=(unit a, unit b) { return a.val_ <= b.val_; }
  friend inline bool operator>=(unit a, unit b) { return a.val_ >= b.val_; }
  template<typename AnyInt>
  friend inline unit operator<<(unit a, AnyInt shift) { return unit(a.val_ << shift, Units()); }
  template<typename AnyInt>
  friend inline unit operator>>(unit a, AnyInt shift) { return unit(a.val_ >> shift, Units()); }

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
  friend inline
  typename rebase<decltype(
    std::declval<base_type>() *
     std::declval<typename boost::enable_if_c<get_units<AnyInt>::is_nonunit_scalar, AnyInt>::type>()
  )>::type
  operator*(unit a, AnyInt factor)
  { return typename rebase<decltype(a.val_ * factor)>::type(a.val_ * factor, Units()); }

  template<typename AnyInt>
  friend inline
  typename rebase<decltype(
    std::declval<typename boost::enable_if_c<get_units<AnyInt>::is_nonunit_scalar, AnyInt>::type>()
     * std::declval<base_type>()
  )>::type
  operator*(AnyInt factor, unit b)
  { return typename rebase<decltype(factor * b.val_)>::type(factor * b.val_, Units()); }

  template<typename AnyInt>
  friend inline
  typename rebase<decltype(
    std::declval<base_type>()
     / std::declval<typename boost::enable_if_c<get_units<AnyInt>::is_nonunit_scalar, AnyInt>::type>()
  )>::type
  operator/(unit a, AnyInt divisor)
  { return typename rebase<decltype(a.val_ / divisor)>::type(a.val_ / divisor, Units()); }

  template<typename AnyInt>
  friend inline
  typename rebase<decltype(
    std::declval<typename boost::enable_if_c<get_units<AnyInt>::is_nonunit_scalar, AnyInt>::type>()
     / std::declval<base_type>()
  )>::type::reciprocal_type
  operator/(AnyInt dividend, unit b)
  { return typename rebase<decltype(dividend / b.val_)>::type::reciprocal_type(
      dividend / b.val_, Units::reciprocal()); }
};


template<typename IntA, typename IntB, typename UnitsA, typename UnitsB>
inline typename
make_unit_type<
    decltype(std::declval<IntA>() * std::declval<IntB>()),
    typename multiply_units<UnitsA, UnitsB>::type
  >::type
operator*(unit<IntA, UnitsA> a, unit<IntB, UnitsB> b) {
  return make_unit_type<
          decltype(std::declval<IntA>() * std::declval<IntB>()),
          typename multiply_units<UnitsA, UnitsB>::type
      >::construct(a.get(UnitsA()) * b.get(UnitsB()));
}

template<typename IntA, typename IntB, typename UnitsA, typename UnitsB>
inline typename
make_unit_type<
    decltype(std::declval<IntA>() / std::declval<IntB>()),
    typename divide_units<UnitsA, UnitsB>::type
  >::type
operator/(unit<IntA, UnitsA> a, unit<IntB, UnitsB> b) {
  return make_unit_type<
          decltype(std::declval<IntA>() / std::declval<IntB>()),
          typename divide_units<UnitsA, UnitsB>::type
      >::construct(a.get(UnitsA()) / b.get(UnitsB()));
}




template<typename Int, typename U>
inline
typename make_unit_type<
  typename boost::enable_if_c<get_units<Int>::is_nonunit_scalar, Int>::type,
  units<U>
>::type
operator*(Int a, units<U>) {
  return make_unit_type<Int, units<U>>::construct(a);
}

template<typename Int, typename UnitsA, typename UB>
inline typename
make_unit_type<Int, typename multiply_units<UnitsA, units<UB>>::type>::type
operator*(unit<Int, UnitsA> a, units<UB>) {
  return make_unit_type<
            Int, typename multiply_units<UnitsA, units<UB>>::type
    >::construct(a.get(UnitsA()));
}





template<typename Int, typename U>
inline
typename make_unit_type<
  typename boost::enable_if_c<get_units<Int>::is_nonunit_scalar, Int>::type,
  typename units<U>::reciprocal_type
>::type
operator/(Int a, units<U>) {
  return make_unit_type<
            Int, typename units<U>::reciprocal_type
    >::construct(a);
}

template<typename Int, typename UnitsA, typename UB>
inline
typename make_unit_type<
  Int, typename divide_units<UnitsA, units<UB>>::type
>::type
operator/(unit<Int, UnitsA> a, units<UB>) {
  return make_unit_type<
            Int, typename divide_units<UnitsA, units<UB>>::type
    >::construct(a.get(UnitsA()));
}


// Multiplies the sign of arg 1 into the (signed) value of arg 2.
template<typename T1, typename T2>
inline auto
imbue_sign(T1 signum, T2 base_val)
-> decltype(base_val
    * pseudo.pow<get_units<T1>::type::pseudo>()
    * units_factor<(get_units<T1>::type::ratio::num < 0 ? -1 : 1)>())
{
  static_assert(get_units<T1>::type::ratio::num != 0, "imbuing an ambiguous sign");
  caller_error_if(signum == 0, "imbuing an ambiguous sign");
  return
    ((signum < 0) ? -base_val : base_val)
    * pseudo.pow<get_units<T1>::type::pseudo>()
    * units_factor<(get_units<T1>::type::ratio::num < 0 ? -1 : 1)>();
}


template<intmax_t N>
struct identity_units {
  typedef unit<
      typename boost::int_max_value_t<(N >= 0 ? N : ~N)>::least,
      units<u_v_t<boost::ratio<1, N>>>
    > type;
};

template<intmax_t N>
inline typename identity_units<N>::type
identity(units<u_v_t<boost::ratio<1, N>>> u) {
  return typename identity_units<N>::type(N, u);
}

template<typename Target, typename Int, typename Units>
inline unit<typename get_primitive_int_type<Target>::type, Units>
numeric_representation_cast(unit<Int, Units> const& num) {
  return num;
}

template<typename Int, typename Units>
inline unit<Int, typename Units::template units_pow<1, 2>::type>
i64sqrt(unit<Int, Units> const& radicand) {
  typedef typename Units::template units_pow<1, 2>::type sqrt_units;
  return unit<Int, sqrt_units>(i64sqrt(get(radicand, Units())), sqrt_units());
}


template<typename TypeIfWeDivided>
struct make_non_normalized_rational_unit_info {
  typedef typename get_units<TypeIfWeDivided>::representation_type int_type;
  typedef non_normalized_rational<int_type> rational_number_type;
  typedef typename get_units<TypeIfWeDivided>::type units;
  typedef typename make_unit_type<rational_number_type, units>::type type;
};

template<typename Num, typename Den>
inline typename
make_non_normalized_rational_unit_info<decltype(std::declval<Num>() / std::declval<Den>())>::type
make_non_normalized_rational_unit(Num num, Den den) {
  typedef make_non_normalized_rational_unit_info<decltype(std::declval<Num>() / std::declval<Den>())> info;
  return make(
    typename info::rational_number_type(
      get(num, typename get_units<Num>::type()),
      get(den, typename get_units<Den>::type())),
    typename info::units());
}

template<typename Num>
inline typename
make_non_normalized_rational_unit_info<Num>::type
make_non_normalized_rational_unit(Num num) {
  typedef make_non_normalized_rational_unit_info<Num> info;
  return make(
    typename info::rational_number_type(
      get(num, typename get_units<Num>::type()),
      typename info::int_type(1)),
    typename info::units());
}

//class coordinate 
#endif
