
#ifndef LASERCAKE_CXX11_TUPLE_HPP__
#define LASERCAKE_CXX11_TUPLE_HPP__

#ifndef USE_BOOST_CXX11_LIBS
#include <tuple>
using std::tuple;
using std::tuple_size;
using std::get;
#else
#include <boost/tuple/tuple.hpp>
#include "boost/tuple/tuple_comparison.hpp"
using boost::tuples::tuple;
template<typename Tuple> struct tuple_size : boost::tuples::length<Tuple> {};
using boost::tuples::get;
#endif

#endif

