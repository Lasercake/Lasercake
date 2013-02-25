
#ifndef LASERCAKE_CXX11_HASH_HPP__
#define LASERCAKE_CXX11_HASH_HPP__

// We need to specialize hash in its own namespace,
// so we define HASH_NAMESPACE
#ifndef USE_BOOST_CXX11_LIBS
#include <functional>
#define HASH_NAMESPACE std
using std::hash;
#else
#include <boost/functional/hash.hpp>
#define HASH_NAMESPACE boost
using boost::hash;
#endif

#endif