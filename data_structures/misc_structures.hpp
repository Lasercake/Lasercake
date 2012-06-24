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

#ifndef LASERCAKE_MISC_STRUCTURES_HPP__
#define LASERCAKE_MISC_STRUCTURES_HPP__

#include <boost/utility.hpp>
#include <boost/optional.hpp>

// These are too small to all get their own files.

template<typename value_type, typename reference = value_type&, typename pointer = value_type*>
class value_as_ptr {
public:
  reference operator*() { return v_; }
  pointer operator->() { return boost::addressof(v_); }
  value_as_ptr(reference v) : v_(v) {}
private:
  value_type v_;
};

// Like boost::optional, but there is always a constructed T in a faux_optional,
// it just might be invalid/inaccessible.  Using boost::optional in polygon-collision-
// detection was a slight performance hit compared to std::pair<bool, rational> but
// faux_optional is no performance hit while still making its code clearer.
template<class T>
class faux_optional {
private:
  struct unspecified_bool_{int member; private:unspecified_bool_();};
  typedef int unspecified_bool_::* unspecified_bool_type;

public:
  typedef T value_type;
  faux_optional() : exists_(false) {}
  faux_optional(boost::none_t) : exists_(false) {}
  faux_optional(value_type const& value) : exists_(true), value_(value) {}
  faux_optional& operator=(boost::none_t) { exists_ = false; return *this; }
  faux_optional& operator=(value_type const& value) { exists_ = true; value_ = value; return *this; }
  operator unspecified_bool_type()const { return exists_ ? &unspecified_bool_::member : nullptr; }
  operator boost::optional<value_type>()const { if(exists_) return value_; else return boost::none; }
  value_type& operator*() { caller_correct_if(exists_, "deref of empty optional"); return value_; }
  value_type const& operator*()const { caller_correct_if(exists_, "deref of empty optional"); return value_; }
  value_type* operator->() { return boost::addressof(**this); }
  value_type const* operator->()const { return boost::addressof(**this); }

private:
  bool exists_;
  value_type value_;
};


#endif
