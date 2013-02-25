
#ifndef LASERCAKE_CXX11_ATOMIC_HPP__
#define LASERCAKE_CXX11_ATOMIC_HPP__

// There are too many identifiers related to atomicity
// for us to quickly 'using' each one individually; we put
// them in a new namespace.
#ifndef USE_BOOST_CXX11_LIBS
#include <atomic>
namespace atomic {
  using namespace std;
}
#else
#include <boost/atomic.hpp>
namespace atomic {
  using namespace boost;
}
#endif

#endif