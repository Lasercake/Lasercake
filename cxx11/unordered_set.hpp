
#ifndef LASERCAKE_CXX11_UNORDERED_SET_HPP__
#define LASERCAKE_CXX11_UNORDERED_SET_HPP__

#ifndef USE_BOOST_CXX11_LIBS
#include <unordered_set>
using std::unordered_set;
using std::unordered_multiset;
#else
#include <boost/unordered_set.hpp>
using boost::unordered_set;
using boost::unordered_multiset;
#endif

#endif

