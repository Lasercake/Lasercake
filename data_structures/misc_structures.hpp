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


template<typename T>
struct literally_random_access_set {
public:
  void insert(T const& stuff) {
    if ((stuffs_set.insert(stuff)).second) {
      stuffs_superset_vector.push_back(stuff);
    }
  }
  bool erase(T const& which) {
    if (stuffs_set.erase(which)) {
      if (stuffs_set.size() * 2 <= stuffs_superset_vector.size()) {
        purge_nonexistent_stuffs();
      }
      return true;
    }
    return false;
  }
  template<typename RNG>
  T const& get_random(RNG& rng)const {
    caller_error_if(stuffs_set.empty(), "Trying to get a random element of an empty literally_random_access_set");
    size_t idx;
    const boost::random::uniform_int_distribution<size_t> random_item_idx(0, stuffs_superset_vector.size()-1);
    do {
      idx = random_item_idx(rng);
    } while (stuffs_set.find(stuffs_superset_vector[idx]) == stuffs_set.end());
    return stuffs_superset_vector[idx];
  }
  bool empty()const { return stuffs_set.empty(); }
  std::unordered_set<T> const& as_unordered_set()const { return stuffs_set; }
private:
  std::vector<T> stuffs_superset_vector;
  std::unordered_set<T> stuffs_set;
  void purge_nonexistent_stuffs() {
    size_t next_insert_idx = 0;
    for (T const& st : stuffs_set) {
      stuffs_superset_vector[next_insert_idx] = st;
      ++next_insert_idx;
    }
    stuffs_superset_vector.erase(stuffs_superset_vector.begin() + next_insert_idx, stuffs_superset_vector.end());
  }
};

#endif
