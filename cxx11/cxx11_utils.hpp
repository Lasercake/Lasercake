
#ifndef LASERCAKE_CXX11_UTILS_HPP__
#define LASERCAKE_CXX11_UTILS_HPP__

#ifndef USE_BOOST_CXX11_LIBS
#include <utility>
using std::move;
using std::forward;
using std::declval;

#else

#include <boost/utility/declval.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/is_lvalue_reference.hpp>

namespace cxx11_utils_impl {
using boost::remove_reference;
using boost::is_lvalue_reference;
template<typename T>
constexpr T&& forward(typename remove_reference<T>::type& t) noexcept
{ return static_cast<T&&>(t); }
template<typename T>
constexpr T&& forward(typename remove_reference<T>::type&& t) noexcept
{ static_assert(!is_lvalue_reference<T>::value, "incorrectly lvalue ref");
  return static_cast<T&&>(t); }
template<typename T>
constexpr typename remove_reference<T>::type&& move(T&& t) noexcept
{ return static_cast<typename remove_reference<T>::type&&>(t); }
}
using cxx11_utils_impl::forward;
using cxx11_utils_impl::move;
using boost::declval;

#endif

#endif