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

#ifndef LASERCAKE_DEBUG_PRINT_DETERMINISTICALLY_HPP__
#define LASERCAKE_DEBUG_PRINT_DETERMINISTICALLY_HPP__

#include <iostream>
#include <unordered_map>
inline std::ostream& debug_print_ostream() {return std::cout;}
template<typename T>
inline void debug_print_val_deterministically(T const& t) {
  debug_print_ostream() << t;
}

inline void debug_print_ptr_deterministically(void const* p) {
  //TODO prevent reuse of pointers from malloc/free: prevent freeing!?
  typedef std::unordered_map<void const*, size_t> ptrmap;
  typedef typename ptrmap::value_type ptrmapelem;
  static ptrmap ptr_names;
  static size_t next_ptr_name = 0;
  //const auto i = ptr_names.find(
  const auto pair = ptr_names.insert(ptrmapelem(p, next_ptr_name));
  next_ptr_name += pair.second;
  debug_print_ostream() << "ptr:" << pair.first->second;
}
template<typename T>
inline void debug_print_val_deterministically(T const* p) {
  debug_print_ptr_deterministically(p);
}
template<typename T>
inline void debug_print_val_deterministically(T* p) {
  debug_print_ptr_deterministically(p);
}
inline void debug_print_val_deterministically(char const* s) {
  debug_print_ostream() << s;
}
#include <boost/shared_ptr.hpp>
template<typename T>
inline void debug_print_val_deterministically(boost::shared_ptr<T> const& p) {
  debug_print_ptr_deterministically(p.get());
}
template<typename T1, typename T2>
inline void debug_print_val_deterministically(std::pair<T1,T2> const& pair) {
  debug_print_ostream() << '(';
  debug_print_val_deterministically(pair.first);
  debug_print_ostream() << ',';
  debug_print_val_deterministically(pair.second);
  debug_print_ostream() << ')';
}
#include <vector>
template<typename T>
inline void debug_print_val_deterministically(std::vector<T> const& v) {
  const size_t MAX_SHOW = 8;
  size_t max_show = MAX_SHOW;
  if(max_show > v.size()) max_show = v.size();
  debug_print_ostream() << '[';
  for(size_t n = 0; n != max_show; ++n) {
    if(n != 0) {debug_print_ostream() << ',';}
    debug_print_ostream() << v[n];
  }
  if(v.size() > max_show) {debug_print_ostream() << ",...";}
  debug_print_ostream() << ']';
}
/*
class world;
inline void debug_print_val_deterministically(world const& w) {
  debug_print_val_deterministically(&w);
}
namespace gl_data_preparation { struct gl_all_data; }
inline void debug_print_val_deterministically(gl_data_preparation::gl_all_data const& d) {
  debug_print_val_deterministically(&d);
}
#include <functional>
template<typename F>
inline void debug_print_val_deterministically(std::function<F> const&) {
  debug_print_ostream() << "<std::function>"; //oh well
}*/

#endif
