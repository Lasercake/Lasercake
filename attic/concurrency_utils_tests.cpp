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

#include "../concurrency_utils.hpp"

//#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_MODULE m_var test
#include <boost/test/unit_test.hpp>

using namespace concurrent;

BOOST_AUTO_TEST_CASE( test_m_var_single_threadedly )
{
  m_var<int> v;
  BOOST_CHECK(!v.try_take());
  v.put(3);
  BOOST_CHECK(!v.try_put(4));
  BOOST_CHECK_EQUAL(v.take(), 3);
  v.try_put(5);
  BOOST_CHECK_EQUAL(v.try_take(), boost::optional<int>(5));
}

#if !defined(LASERCAKE_NO_THREADS)
BOOST_AUTO_TEST_CASE( test_m_var_concurrent )
{
  m_var<int> v;
  boost::thread b([&v]() {
    boost::this_thread::sleep(boost::posix_time::milliseconds(300));
    v.put(1);
    v.put(2);
    v.put(3);
  });
  BOOST_CHECK_EQUAL(v.take(), 1);
  BOOST_CHECK_EQUAL(v.take(), 2);
  BOOST_CHECK_EQUAL(v.take(), 3);
}
#endif

struct something {
};
struct restricted {
  // no default-constructor.
  explicit restricted(something){}
  bool operator==(restricted const& /*other*/) const { return true; }
};
BOOST_AUTO_TEST_CASE( test_m_var_does_not_require_default_constructible )
{
  // partly just a compile-test
  m_var<restricted> v;
  BOOST_CHECK_EQUAL(v.try_take(), boost::none);
  v.try_put(restricted(something()));
}
