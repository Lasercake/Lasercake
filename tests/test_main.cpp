/*

    Copyright Eli Dupree and Isaac Dupree, 2013

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

#define LASERCAKE_TEST_MAIN 1

#include "test_header.hpp"
#include "test_main.hpp"

#include <boost/scope_exit.hpp>

// Not threadsafe to run tests in more than one thread at once,
// because the Boost.Test design doesn't make that easy.
test_cases_state_t test_cases_state;

int lasercake_test_main(int, char**) {
  test_cases_state = test_cases_state_t();
  BOOST_SCOPE_EXIT(void) {
    if(test_cases_state.finished_without_crashing && test_cases_state.num_checks_failed == 0) {
      LOG << "*** No errors detected in " << test_cases_state.num_checks_run << " checks\n";;
    }
    if(test_cases_state.num_checks_failed != 0) {
      LOG << test_cases_state.num_checks_failed << " test(s) failed ("
        << (test_cases_state.num_checks_run - test_cases_state.num_checks_failed)
        << " succeeded)\n";
    }
    if(!test_cases_state.finished_without_crashing) {
      LOG << "Tests exited unexpectedly with an exception\n";
    }
    test_cases_state = test_cases_state_t();
  } BOOST_SCOPE_EXIT_END
  logger_impl::log log; // HACK
  std::ostream& os = log << "Running tests..." << std::endl;
  test_cases_state.error_log = &os;
#if 0
  try {    // Don't try to catch exceptions, because the run-time system
           // does a better job of reporting them than we can.
#endif
    register_test_file<first_test_file>::go();
    test_cases_state.finished_without_crashing = true;
    return (test_cases_state.num_checks_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
#if 0
  } catch(std::exception e) {
    try {
      LOG << "Exception " << std::flush << e.what();
    } catch(...) {
    }
  } catch(...) {
    LOG << "Unknown exception " << std::flush;
  }
  // use the checkpoint information
#endif
}
