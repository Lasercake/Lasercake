
Rationale
---------

Mac OS X 10.6 doesn't have a C++11-compatible C++ library, and using
newer compilers works okay but mixing C++ standard library
implementations in the same executable doesn't.

Details
-------

`#define USE_BOOST_CXX11_LIBS` if you want to use the non-standard-library
versions; otherwise these headers include the standard library versions.

Boost doesn't have unique_ptr<>, and we use it, and we have better access
to a C++11 compiler than a C++11 standard library on OS X (Feb 2013).

Therefore, basing this off libc++ r176006 unique_ptr.

Legalese
--------

Any copyright in this directory that is mine, I release freely under the
Boost Software License, the libc++ licenses, and the public domain.
