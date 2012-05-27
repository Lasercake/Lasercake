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

#include "../utils.hpp"
#include <limits>
#include <array>

//#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_MODULE utils test
#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>

namespace /* anonymous */ {

void i64sqrt_test(uint64_t radicand) {
  uint64_t sqrt_result = i64sqrt(radicand); //implicit cast the result to 64 bits so we can square it
  BOOST_CHECK(sqrt_result * sqrt_result <= radicand);
  if(sqrt_result != std::numeric_limits<uint32_t>::max()) {
    BOOST_CHECK((sqrt_result+1) * (sqrt_result+1) > radicand);
  }
}


BOOST_AUTO_TEST_CASE( my_sqrt ) {
  std::array<uint64_t, 19> numbers_to_test = {{ 0, 1, 2, 3, 4, 5, 17, 232, 500,
    78978978, 8948954789789349789ull, 0xfffffffful, 0x100000000ull,
    0x100000001ull, 0xffffffffffffffffull, 0xfffffffffffffffeull,
    0xeeeeeeeeeeeeeeeeull, 0xfffffffe00000001ull, 0xfffffffe00000000ull
  }};
  BOOST_PARAM_TEST_CASE(i64sqrt_test, numbers_to_test.begin(), numbers_to_test.end());
}

}  /* end anonymous namespace */
