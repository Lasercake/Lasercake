/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_UNUSED_STRUCTURES_HPP__
#define LASERCAKE_UNUSED_STRUCTURES_HPP__

// Move these to data_structures/misc_structures.hpp or such if you want to
// use them in Lasercake.

#include "../data_structures/misc_structures.hpp"

#include <boost/iterator/filter_iterator.hpp>

template<typename T1, typename T2>
struct reference_pair {
  //or should it require T1 and T2 to be ref types already instead?
  typedef T1& first_type;
  typedef T2& second_type;
  typedef std::pair<typename boost::remove_cv<T1>::type,
                    typename boost::remove_cv<T2>::type> std_pair_type;
  //typedef std::pair<typename boost::remove_cv<T1>::type const,
  //                  typename boost::remove_cv<T2>::type> std_pair_type_const_mutable;
  // etc operators to allow adding to std containers ?
  first_type first;
  second_type second;
  reference_pair(first_type first, second_type second) : first(first), second(second) {}
  //operator std_pair_type() const { return std_pair_type(first, second); }
};


// a simple functor
struct bool_convertible_is_true {
  typedef bool result_type;
  template<typename T>
  bool operator()(T const& t)const { return t; }
};

// Has N or fewer elements.
template<typename T, size_t N, typename Predicate = bool_convertible_is_true >
struct filtered_array {
  T data_[N];

  typedef T                  value_type;
  typedef value_type*        pointer;
  typedef const value_type*  const_pointer;
  typedef value_type&        reference;
  typedef const value_type&  const_reference;
  typedef std::size_t        size_type;
  typedef std::ptrdiff_t     difference_type;
  typedef boost::filter_iterator<Predicate, value_type*>        iterator;
  typedef boost::filter_iterator<Predicate, const value_type*>  const_iterator;
  typedef std::reverse_iterator<iterator>                       reverse_iterator;
  typedef std::reverse_iterator<const_iterator>                 const_reverse_iterator;

  iterator begin() { return iterator(data_ + 0); }
  iterator end() { return iterator(data_ + N); }
  const_iterator begin() const { return const_iterator(data_ + 0); }
  const_iterator end() const { return const_iterator(data_ + N); }

  bool empty() const { return begin() == end(); }
  size_type size() const { return std::distance(begin(), end()); }
  size_type max_size() const { return N; }

  void fill(T const& t) { for(size_type i = 0; i != N; ++i) data_[i] = t; }
  void swap(filtered_array& other) { for(size_type i = 0; i != N; ++i) { using std::swap; swap(data_[i], other.data_[i]); } }
};

#endif

