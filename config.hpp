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

#ifndef LASERCAKE_CONFIG_HPP__
#define LASERCAKE_CONFIG_HPP__

#include <inttypes.h>
#include <assert.h>

#include <stdexcept>
#include <boost/throw_exception.hpp>


// Some asserts take too much runtime to turn on by default.
// So write them "assert_if_ASSERT_EVERYTHING(x);"
// Build with -DASSERT_EVERYTHING to turn them on.
#ifdef ASSERT_EVERYTHING
  // By not using parentheses (e.g. not "#define assert_if_ASSERT_EVERYTHING(x) assert(x)"),
  // we prevent any additional unintended macro-expansion,
  // which in the case of assert would affect the string
  // that is printed when the assert fails.
  #define assert_if_ASSERT_EVERYTHING assert
  const bool assert_everything = true;
#else
  // Make sure the code takes no runtime (the compiler will optimize it out)
  // but that it still compiles to a boolean expression (so that turning on
  // ASSERT_EVERYTHING is sure to compile even if we didn't test with it on
  // recently).
  #define assert_if_ASSERT_EVERYTHING(x) ((true) ? (void)0 : ((x) ? (void)0 : (void)0))
  const bool assert_everything = false;
#endif

#ifndef LASERCAKE_BUILD_SYSTEM_DID_FEATURE_DETECTION

  #ifndef ATTRIBUTE_NORETURN
    // from http://www.boost.org/doc/libs/1_48_0/boost/exception/detail/attribute_noreturn.hpp
    #if defined(_MSC_VER)
      #define ATTRIBUTE_NORETURN __declspec(noreturn)
    #elif defined(__GNUC__)
      #define ATTRIBUTE_NORETURN __attribute__((noreturn))
    #else
      #define ATTRIBUTE_NORETURN
    #endif
  #endif

  #if defined(__GNUC__)
    inline int DETECTED_builtin_clz64(uint64_t arg) {
      static_assert(sizeof(unsigned int) == 8 || sizeof(unsigned long) == 8 || sizeof(unsigned long long) == 8, "no 64-bit builtin uint type?!");
      return
        (sizeof(unsigned int) == 8) ? __builtin_clz(arg) :
        (sizeof(unsigned long) == 8) ? __builtin_clzl(arg) :
        __builtin_clzll(arg);
    }
    inline int DETECTED_builtin_clz32(uint32_t arg) {
      static_assert(sizeof(unsigned int) == 4 || sizeof(unsigned long) == 4 || sizeof(unsigned long long) == 4, "no 32-bit builtin uint type?!");
      return
        (sizeof(unsigned int) == 4) ? __builtin_clz(arg) :
        (sizeof(unsigned long) == 4) ? __builtin_clzl(arg) :
        __builtin_clzll(arg);
    }
    inline int DETECTED_builtin_ctz64(uint64_t arg) {
      static_assert(sizeof(unsigned int) == 8 || sizeof(unsigned long) == 8 || sizeof(unsigned long long) == 8, "no 64-bit builtin uint type?!");
      return
        (sizeof(unsigned int) == 8) ? __builtin_ctz(arg) :
        (sizeof(unsigned long) == 8) ? __builtin_ctzl(arg) :
        __builtin_ctzll(arg);
    }
    inline int DETECTED_builtin_ctz32(uint32_t arg) {
      static_assert(sizeof(unsigned int) == 4 || sizeof(unsigned long) == 4 || sizeof(unsigned long long) == 4, "no 32-bit builtin uint type?!");
      return
        (sizeof(unsigned int) == 4) ? __builtin_ctz(arg) :
        (sizeof(unsigned long) == 4) ? __builtin_ctzl(arg) :
        __builtin_ctzll(arg);
    }
    #define DETECTED_builtin_clz64 DETECTED_builtin_clz64
    #define DETECTED_builtin_clz32 DETECTED_builtin_clz32
    #define DETECTED_builtin_ctz64 DETECTED_builtin_ctz64
    #define DETECTED_builtin_ctz32 DETECTED_builtin_ctz32
  #endif

  // This is a conservative poor way of guessing, that probably works for
  // most current machines/systems (but not for all compilers!)
  #if defined(__GNUC__) && (__LP64__ || __x86_64__)
    #define DETECTED_int128_t __int128_t
    #define DETECTED_uint128_t __uint128_t
  #endif
#endif

#if !LASERCAKE_NO_THREADS
  #if defined(__GNUC__)
    // current compilers (e.g. GCC 4.7, Clang 3.1) don't seem to support the 'thread_local' C++11 name
    #define thread_local __thread
  #endif
  const bool LASERCAKE_NO_THREADS = false;
#else
  #define thread_local
#endif

// (not enabled unless you enable it) #define USE_BOUNDS_CHECKED_INTS 1


// It's not polite for library functions to assert() because the library's users
// misused a correct library; use these for that case.
inline ATTRIBUTE_NORETURN void caller_error(const char* error) {
  // If exceptions prove worse for debugging than asserts/segfaults,
  // feel free to comment this out and use asserts/segfaults/breakpoints.
  boost::throw_exception(std::logic_error(error));
}
// You must provide an explanatory string so that the user of the library
// will know what *they* did wrong, and not have to interpret an assert() expression
// to find out.
inline void caller_error_if(bool cond, const char* error) {
  if(cond) {
    caller_error(error);
  }
}
inline void caller_correct_if(bool cond, const char* error) {
  if(!cond) {
    caller_error(error);
  }
}

#endif
