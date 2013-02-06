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

#include "test_header.hpp"

#include "../units.hpp"

#include <iostream>
#include <boost/random/mersenne_twister.hpp>

typedef meters_t meter;
//constexpr auto kilograms = kilo*grams;

//typedef typename units_prod<kilo_t, grams_t>::type kilograms_t;
typedef typename units_prod<kilograms_t, meters_t,
  typename units_pow<seconds_t, -2>::type>::type newtons_t_A;
typedef typename units_prod<kilograms_t, meters_t,
  units_pow<seconds_t, -2>>::type newtons_t_B;

typedef typename units_prod<kilo_t, meters_t>::type kilometers_t;
typedef typename units_pow<kilometers_t, 6>::type kilometers6_t;
typedef typename units_pow<kilometers6_t, -2, 3>::type inverse_kilometers4_t;
typedef typename units_pow<meters_t, -4>::type inverse_meters4_t;

constexpr auto kilometers = kilometers_t();

BOOST_AUTO_TEST_CASE( unitses ) {
  const physical_quantity<int32_t, meter> foo = 1 * meter();
  const physical_quantity<int64_t, meter> foo64 = foo;
  const physical_quantity<int32_t, meter> foo3 = 3 * meter();
  const physical_quantity<int32_t, meter> foo5 = 5 * meters;
  foo + foo;
  auto bfoo = foo * foo;
  bfoo = bfoo * 3;

  foo64 + foo;

  // TODO consider whether to relax bounds_checked_int to make this work,
  // or provide conversion functions, or such.
  // const physical_quantity<double, meter> foofloating = foo;
  // Also this doesn't work because physical_quantity<> tries to wrap
  // its data type in a bounds checked int, heh.  That'd
  // be fairly easy to fix for floating point.
  // const physical_quantity<double, meter> foofloating = 1.0*meters;

  // Deliberately avoid get_primitive_int() to make sure
  // that it is returning the correct type in both cases.
#if USE_BOUNDS_CHECKED_INTS
  int yay = (bfoo / foo / foo).get();
#else
  int yay = (bfoo / foo / foo);
#endif
  const typename lasercake_int<int32_t>::type okayy1 = get(bfoo, meters*meters);
  const typename lasercake_int<int32_t>::type okayy2 = bfoo.get(meters*meters);
  BOOST_CHECK_EQUAL(okayy1, okayy2);

  meters*meters;
  units<dim::meter<2>> foofoo = meters*meters;
  meters/seconds;
  units<> dimless = meters/meters;
  2*dimless;
  2*(meters/meters);
  2*(meters*meters);
  ((2*meters)*meters) + (2*(meters*meters));

  1*meters*meters + 1*meters.pow<2>();

  // 1000*1000*1000*1000 doesn't fit into 32 bits, so this identity()
  // returns a 64 bit quantity.
  const physical_quantity<int64_t, inverse_kilometers4_t> invfoo4
    = 1/foo/foo/foo/foo
    * identity(inverse_kilometers4_t() / inverse_meters4_t());
  // This fits within fewer bits:
  const physical_quantity<int32_t, meters_t> thirtytwobits
    = 1 * kilometers * identity(meters / kilometers);

  const physical_quantity<int, newtons_t_A> newtonz =
    7 * kilograms * meters / seconds / seconds;
  newtonz + 1*newtons_t_B();

  auto scalar = 4*meters;
  auto pseudoscalar = scalar*pseudo;
  BOOST_CHECK_EQUAL(pseudoscalar, abs(pseudoscalar) * sign(pseudoscalar));
  scalar = pseudoscalar * pseudo;
  scalar = pseudoscalar / pseudo;
  pseudoscalar = scalar * pseudo;
  pseudoscalar = scalar / pseudo;
  pseudoscalar = pseudoscalar + pseudoscalar;

  1*pseudo + sign(pseudoscalar);
  auto useded = 1 + sign(scalar);

  scalar = imbue_sign(scalar, scalar);
  pseudoscalar = imbue_sign(scalar, pseudoscalar);
  pseudoscalar = imbue_sign(pseudoscalar, scalar);
  scalar = imbue_sign(pseudoscalar, pseudoscalar);

  std::cerr << (yay * foofoo * invfoo4 * thirtytwobits * useded) << '\n';

  rounding_strategy<
    rounding_strategies::round_down,
    rounding_strategies::negative_mirrors_positive> strat;
  divide(divide(foo, bfoo, strat), 3, strat) + divide(3, foo, strat);

  const auto ratty = make_non_normalized_rational_physical_quantity(bfoo, foo);
  const auto derat = make_units_split_rational(ratty);
  derat.numerator + foo;
  (3 + derat.denominator) * seconds;
  const auto rerat = make_non_normalized_rational_physical_quantity(
    derat.numerator, derat.denominator);
  BOOST_CHECK_EQUAL(ratty, rerat);
  derat + ratty;
  
  boost::mt19937 rng;
  const uniform_int_distribution<bounds_checked_int<int> > dist1(7, 11);
  bounds_checked_int<int> val1 = dist1(rng);
  BOOST_CHECK_GE(val1, 7);
  BOOST_CHECK_LE(val1, 11);
  const uniform_int_distribution<physical_quantity<int32_t, meter> > dist2(foo3, foo5);
  physical_quantity<int32_t, meter> val2 = dist2(rng);
  BOOST_CHECK_GE(val2, 3*meters);
  BOOST_CHECK_LE(val2, 5*meters);

  BOOST_CHECK_EQUAL(foo5 + foo3, 8*meters);
  BOOST_CHECK_EQUAL(foo5 - foo3, 2*meters);
  BOOST_CHECK_EQUAL(foo5 % foo3, 2*meters);
  BOOST_CHECK_EQUAL(foo3 % foo5, 3*meters);
  BOOST_CHECK_EQUAL((13*meters) % (5*meters), 3*meters);
  BOOST_CHECK_EQUAL(!!foo3, true);
  BOOST_CHECK_EQUAL(+foo3, 3*meters);
  BOOST_CHECK_EQUAL(-foo3, -3*meters);
  BOOST_CHECK_EQUAL(abs(foo3), 3*meters);
  BOOST_CHECK_EQUAL(abs(-foo3), 3*meters);
  BOOST_CHECK_EQUAL(foo5 == foo3, false);
  BOOST_CHECK_EQUAL(foo5 != foo3, true);
  BOOST_CHECK_EQUAL(foo5 > foo3, true);
  BOOST_CHECK_EQUAL(foo5 < foo3, false);
  BOOST_CHECK_EQUAL(foo5 <= foo3, false);
  BOOST_CHECK_EQUAL(foo5 >= foo3, true);
  BOOST_CHECK_EQUAL(foo5 << 3, 40*meters);
  BOOST_CHECK_EQUAL(foo5 >> 1, 2*meters);

  constexpr auto recipmeters = reciprocal(meters);
  const auto recipfoo3 = 9 / foo3;
  
  BOOST_CHECK_EQUAL(foo5 * foo3, 15*meters*meters);
  BOOST_CHECK_EQUAL(foo5 * 3, 15*meters);
  BOOST_CHECK_EQUAL(3 * foo5, 15*meters);
  BOOST_CHECK_EQUAL(foo5 * meters, 5*meters*meters);
  BOOST_CHECK_EQUAL(7 * meters, 7*meters);
  BOOST_CHECK_EQUAL(kilograms * meters, meters*kilograms);

  BOOST_CHECK_EQUAL(foo5 / foo3, 1);
  BOOST_CHECK_EQUAL(foo5 / 3, 1*meters);
  BOOST_CHECK_EQUAL(13 / foo5, 2/meters);
  BOOST_CHECK_EQUAL(foo5 / meters, 5);
  BOOST_CHECK_EQUAL(7 / meters, 7/meters);
  BOOST_CHECK_EQUAL(kilograms / meters, reciprocal(meters)*kilograms);

  BOOST_CHECK_EQUAL(foo5 * recipfoo3, 15);
  BOOST_CHECK_EQUAL(foo5 * recipmeters, 5);
  BOOST_CHECK_EQUAL(meters * recipmeters, trivial_units());

  BOOST_CHECK_EQUAL(foo5 / recipfoo3, 1*meters*meters);
  BOOST_CHECK_EQUAL(foo5 / recipmeters, 5*meters*meters);
  BOOST_CHECK_EQUAL(seconds / seconds, trivial_units());


  physical_quantity<int32_t, meter> mutfoo = 0;
  BOOST_CHECK_EQUAL(mutfoo, 0*meters);
  BOOST_CHECK_EQUAL(mutfoo += foo3, 3*meters);
  BOOST_CHECK_EQUAL(mutfoo += foo3, 6*meters);
  BOOST_CHECK_EQUAL(mutfoo -= foo3, 3*meters);
  BOOST_CHECK_EQUAL(mutfoo *= 7, 21*meters);
  BOOST_CHECK_EQUAL(mutfoo /= 2, 10*meters);
  BOOST_CHECK_EQUAL(mutfoo <<= 2, 40*meters);
  BOOST_CHECK_EQUAL(mutfoo >>= 2, 10*meters);
  BOOST_CHECK_EQUAL(mutfoo %= foo3, 1*meters);

  BOOST_CHECK_EQUAL(0*foo, 0*meters);
  BOOST_CHECK_EQUAL(foo*0, 0*meters);
  BOOST_CHECK(0*foo == 0);
  BOOST_CHECK(foo*0 == 0);
  BOOST_CHECK(!(foo == 0));
  BOOST_CHECK(foo != 0);
  BOOST_CHECK(-foo != 0);
  BOOST_CHECK(foo > 0);
  BOOST_CHECK(foo >= 0);
  BOOST_CHECK(-foo < 0);
  BOOST_CHECK(-foo <= 0);
  BOOST_CHECK(!(-foo > 0));
  BOOST_CHECK(0*foo >= 0);
  BOOST_CHECK(0*foo <= 0);
  BOOST_CHECK(!(0*foo > 0));
  BOOST_CHECK(!(0*foo < 0));
  BOOST_CHECK(0 != foo);
  BOOST_CHECK(0 != -foo);
  BOOST_CHECK(0 < foo);
  BOOST_CHECK(0 <= foo);
  BOOST_CHECK(0 > -foo);
  BOOST_CHECK(0 >= -foo);

  BOOST_CHECK_EQUAL(identity(grams / kilograms), (1000*units_factor<1, 1000>()));
  BOOST_CHECK_EQUAL(identity(grams / kilograms)*units_factor<1000>(), short(1000));
}
