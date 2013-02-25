
#ifndef LASERCAKE_CXX11_FUNCTION_HPP__
#define LASERCAKE_CXX11_FUNCTION_HPP__

#ifndef USE_BOOST_CXX11_LIBS
#include <functional>
using std::function;
#else
#include <boost/function.hpp>
using boost::function;
#endif

#endif

