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

#include <streambuf>
#include <ostream>

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

  #if (!__clang__ && !BOOST_INTEL && ((__GNUC__ < 4) || (__GNUC__ == 4 && __GNUC_MINOR__ < 7)))
    #define LASERCAKE_GCC_LESS_THAN_4_7 1
  #endif

  #if LASERCAKE_GCC_LESS_THAN_4_7
    #define override
  #endif

  // Don't use LASERCAKE_IT_SEEMS_TO_BE_WINDOWS outside this fake feature
  // detection section if you can help it! Ask about specific features
  // instead.
  #define LASERCAKE_IT_SEEMS_TO_BE_WINDOWS \
    ((defined(_WIN32) || defined(__WIN32__) || defined(WIN32)) && !defined(__CYGWIN__))

  #if !LASERCAKE_IT_SEEMS_TO_BE_WINDOWS
    // TODO use GetProcessMemoryInfo or the like on Windows, if we care enough
    #define LASERCAKE_HAVE_SYS_RESOURCE_H 1
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

#define constexpr_require_and_return(cond, str, ret) ((cond) ? (ret) : throw std::logic_error((str)))

#if DEBUG_PRINT_DETERMINISTICALLY
#include "debug_print_deterministically.hpp"
#endif


namespace logger_impl {
  class log_buf : public std::streambuf {
    static const std::streamsize bufsize_ = 200;
    char buf_[bufsize_];
    void write_buf_();
  public:
    log_buf() { setp(buf_, buf_ + bufsize_); }
    ~log_buf() { if(pptr() != pbase()) { write_buf_(); }}
    std::streamsize xsputn(const char* s, std::streamsize n) override;
    int_type overflow(int_type ch) override;
    int sync() override { write_buf_(); return 0; }
  };
  class log {
    log_buf streambuf_;
    std::ostream os_;
  public:
    // The constructor and destructor generate too much code
    // (including multiple function calls) to be worth inlining.
    log();
    ~log();
    template<typename SomethingOutputted>
    inline std::ostream& operator<<(SomethingOutputted&& output) { return os_ << output; }

    // Apparently the compiler can't deduce SomethingOutputted when it's
    // a function (such as std::endl), so we have to list these overloads too.
    inline std::ostream& operator<<(std::ios_base& (*output)(std::ios_base&)) { return os_ << output; }
    inline std::ostream& operator<<(std::ios& (*output)(std::ios&)) { return os_ << output; }
    inline std::ostream& operator<<(std::ostream& (*output)(std::ostream&)) { return os_ << output; }

    // it's not meant to be copied/moved/anything
    log(log const&) = delete;
    log& operator=(log const&) = delete;
    log(log&&) = delete;
    log& operator=(log&&) = delete;
  };
}

// LOG << this << that << std::hex << the other thing << "\n";
// Constructs a temporary buffer on the stack, used to log the
// data to stderr.  Flushes data when destroyed (typically at the
// next sequence point: the end of the statement) and/or, when
// you send it std::flush or std::endl, and/or whenever its buffer
// fills up.
// On output I/O errors, this simply gives up and pretends to have been
// successful.
#define LOG (::logger_impl::log())

#endif
