
#ifndef LASERCAKE_CXX11_UNORDERED_MAP_HPP__
#define LASERCAKE_CXX11_UNORDERED_MAP_HPP__

#include "hash.hpp"

#ifndef USE_BOOST_CXX11_LIBS
#include <unordered_map>
using std::unordered_map;
using std::unordered_multimap;
#else
#include <boost/unordered_map.hpp>
using boost::unordered_map;
using boost::unordered_multimap;
#endif

#endif

