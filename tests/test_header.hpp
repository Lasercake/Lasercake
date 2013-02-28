/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_TEST_HEADER_HPP__
#define LASERCAKE_TEST_HEADER_HPP__

// At the top of your tests file,
// #define TESTS_FILE whichever_tests_file_you_are
// e.g. #define TESTS_FILE misc_utils_tests
// then include "test_header.hpp".
// At the bottom of your file, put
// REGISTER_TESTS // This must come last in the file.
//
// Also, add your test file identifier below to the list of
// DECLARE_TESTS_FILE(identifier).

// This emulates a part of the Boost.Test API but is significantly
// more lightweight.  Switching to this alone reduced the binary size
// by 1/3, the linking time by somewhat, and got rid of the last of
// the global constructors/destructors (save perhaps in Qt and
// separately-compiled libc / c++ std lib code that I haven't discovered).
// This also solves the problem that Clang-compiled Lasercake got into
// a mysterious infinite loop when trying to run the tests and that GDB
// had internal errors when trying to open that binary (Clang 3.2, GDB 7.5.1,
// Arch Linux x86_64, Jan 2013).

#include <boost/preprocessor/stringize.hpp>
#include "../config.hpp"

typedef int tests_file_id;
template<tests_file_id File> struct register_test_file;
#define DECLARE_TESTS_FILES_BEGIN \
  static const tests_file_id first_test_file = __COUNTER__ + 1;
#define DECLARE_TESTS_FILE(name) \
  static const tests_file_id name = __COUNTER__; \
  template<> struct register_test_file<name> { static void go(); };
#define DECLARE_TESTS_FILES_END \
  template<> struct register_test_file<__COUNTER__> { static void go() {} };

DECLARE_TESTS_FILES_BEGIN

DECLARE_TESTS_FILE(bbox_collision_detector_tests)
DECLARE_TESTS_FILE(borrowed_bitset_tests)
DECLARE_TESTS_FILE(bounds_checked_int_tests)
DECLARE_TESTS_FILE(misc_utils_tests)
DECLARE_TESTS_FILE(patricia_trie_tests)
DECLARE_TESTS_FILE(units_tests)

DECLARE_TESTS_FILES_END


//////////////////////////////////////////////////////////////////////////
//////// Test-registration code that uses no global constructors /////////
//////////////////////////////////////////////////////////////////////////

// The testing world is composed of test files, which are composed of test
// cases, which are composed of code and tests.  Test files are enumerated
// first-to-last in the above DECLARE_TESTS_FILES order
// via register_test_file<>'s recursion.  Within each file, test cases are
// run first-to-last (by enumerating them last-to-first with recursion)
// via register_test_file<>'s recursion.
// __COUNTER__ makes the whole thing work.  (__LINE__ could too, in a much
// more inefficient way, if it were necessary.)
template<tests_file_id File, int N>
struct register_test_case;

// If adapting this for Boost.Test, pass this as the 'testcase' argument instead:
// boost::unit_test::framework::master_test_suite().add(BOOST_TEST_CASE( &name ));
#define TEST_CASE_IMPL(counter, signature, testcase) \
  signature; \
  template<> struct register_test_case<TESTS_FILE, counter> { \
    static void go() { \
      register_test_case<TESTS_FILE, counter - 1>::go(); \
      testcase \
    } \
  }; \
  signature

#undef BOOST_AUTO_TEST_CASE
#define BOOST_AUTO_TEST_CASE(name) \
  TEST_CASE_IMPL( \
    __COUNTER__, static void name(), \
    test_cases_state.test_function = BOOST_PP_STRINGIZE(name); \
    name(); \
  )

#define REGISTER_TESTS \
  void register_test_file<TESTS_FILE>::go() { \
    test_cases_state.test_file = BOOST_PP_STRINGIZE(TESTS_FILE); \
    register_test_case<TESTS_FILE, __COUNTER__ - 1>::go(); \
    register_test_file<TESTS_FILE + 1>::go(); \
  }

#ifndef LASERCAKE_TEST_MAIN
template<> struct register_test_case<TESTS_FILE, __COUNTER__> { static void go() {} };
#endif



//////////////////////////////////////////////////////////////////////////
////////                   Individual-test code                  /////////
//////////////////////////////////////////////////////////////////////////


struct test_cases_state_t {
  const char* test_file;
  const char* test_function;
  const char* checkpoint;
  const char* step;
  int64_t num_checks_run;
  int64_t num_checks_failed;
  std::ostream* error_log;
  bool finished_without_crashing;
};

// Not threadsafe to run tests in more than one thread at once,
// because the Boost.Test design doesn't make that easy.
extern test_cases_state_t test_cases_state;

struct uninteresting{};
inline uninteresting make_uninteresting() { return uninteresting(); }
inline std::ostream& operator<<(std::ostream& os, uninteresting){return os;}
inline bool is_interesting(uninteresting) {return false;}
template<typename T> inline bool is_interesting(T const&) {return true;}

struct msg { bool good; const char* contents; explicit operator bool()const{return good;}};
inline std::ostream& operator<<(std::ostream& os, msg m){
  if(m.contents){os<<m.contents;}
  return os;
}

template<typename AF, typename BF, typename Predicate>
inline void do_test(AF&& af, BF&& bf, Predicate&& p, const char* desc) {
  test_cases_state_t& state = test_cases_state;
  const char* step = "beginning";
  bool success = false;
  state.checkpoint = desc;
  try {
    step = "evaluating A";
    auto&& a(af());
    step = "evaluating B";
    auto&& b(bf());
    step = "evaluating comparison";
    success = p(a, b);
    if(!success && state.error_log) {
      *state.error_log << "Failed: " << desc << ":\n    A = " << std::flush;
      step = "evaluating ostream << A";
      *state.error_log << a;
      if(is_interesting(b)) {
        *state.error_log << "; B = " << std::flush;
        step = "evaluating ostream << B";
        *state.error_log << b;
      }
    }
  }
  catch(std::exception& e) {
    success = false;
    if(state.error_log) {
      *state.error_log << desc << ":\n  caught unexpected exception when " << step << ":\n    " << std::flush;
      step = "caught unexpected exception; evaluating e.what()";
      *state.error_log << e.what();
    }
  }
  if(!success && state.error_log) {*state.error_log << std::endl;}
  ++state.num_checks_run;

  if(!success) {++state.num_checks_failed;}
}

// Just in case Boost.Test headers got included somehow
#undef BOOST_CHECK_EQUAL
#undef BOOST_CHECK_GT
#undef BOOST_CHECK_LT
#undef BOOST_CHECK_GE
#undef BOOST_CHECK_LT
#undef BOOST_CHECK_NE
#undef BOOST_CHECK
#undef BOOST_CHECK_NO_THROW
#undef BOOST_CHECK_THROW
// Don't parenthesize 'a' and 'b' expressions in return statements in these
// because it makes Clang warn (perhaps correctly?)
// "reference to stack memory associated with local variable '...' returned
//      [-Wreturn-stack-address]
// and because they are used as complete statements so the usual
// parentheses-for-correct-grouping-in-macros reason isn't a risk here.
#define BINARY_CHECK_IMPL(a, b, comparator_type, comparator_str) \
  do_test([&]()->decltype(a){return a;}, [&]()->decltype(b){return b;}, comparator_type(), \
    BOOST_PP_STRINGIZE(TESTS_FILE) ":" BOOST_PP_STRINGIZE(__LINE__) ": `" BOOST_PP_STRINGIZE(a) "` " comparator_str " `" BOOST_PP_STRINGIZE(b) "`" \
  )
#define BOOST_CHECK_EQUAL(a, b) BINARY_CHECK_IMPL(a, b, comparators::equal_to, "==")
#define BOOST_CHECK_GT(a, b)    BINARY_CHECK_IMPL(a, b, comparators::greater, ">")
#define BOOST_CHECK_LT(a, b)    BINARY_CHECK_IMPL(a, b, comparators::less, "<")
#define BOOST_CHECK_GE(a, b)    BINARY_CHECK_IMPL(a, b, comparators::greater_equal, ">=")
#define BOOST_CHECK_LE(a, b)    BINARY_CHECK_IMPL(a, b, comparators::less_equal, "<=")
#define BOOST_CHECK_NE(a, b)    BINARY_CHECK_IMPL(a, b, comparators::not_equal_to, "!=")
#define BOOST_CHECK(a) \
  do_test([&]()->decltype(a){return a;}, &make_uninteresting, comparators::first_is_true(), \
    BOOST_PP_STRINGIZE(TESTS_FILE) ":" BOOST_PP_STRINGIZE(__LINE__) ": `" BOOST_PP_STRINGIZE(a) "`" \
  )
#define BOOST_CHECK_NO_THROW(a) \
  do_test([&]{a; return true;}, &make_uninteresting, comparators::first_is_true(), \
    BOOST_PP_STRINGIZE(TESTS_FILE) ":" BOOST_PP_STRINGIZE(__LINE__) ": `" BOOST_PP_STRINGIZE(a) "`" \
  )
#define BOOST_CHECK_THROW(a, e) \
  do_test([&]() -> msg { \
      try{ a; } catch(e const& test_suite_expected_exception){return msg{true, test_suite_expected_exception.what()};} \
      return msg{false, "(something)"}; \
    }, &make_uninteresting, comparators::first_is_true(), \
    BOOST_PP_STRINGIZE(TESTS_FILE) ":" BOOST_PP_STRINGIZE(__LINE__) ": `" BOOST_PP_STRINGIZE(a) "` throws `" BOOST_PP_STRINGIZE(e) "`" \
  )

#endif

 