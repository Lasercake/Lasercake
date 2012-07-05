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

#include <utility>
#include <functional>
#include <vector>
#include <unordered_map>
#include <boost/utility.hpp>
#include <boost/iterator/transform_iterator.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <boost/optional.hpp>
#pragma GCC diagnostic pop


#include "../config.hpp"

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
class literally_random_access_set {
public:
  typedef size_t size_type;
private:
  typedef std::unordered_map<T, size_type> contents_type_;
  typedef typename contents_type_::value_type contents_value_type_;
  typedef typename contents_type_::iterator contents_iterator_;
  typedef typename contents_type_::const_iterator contents_const_iterator_;
  // In contents_type_, the user is only permitted to change the keys
  // (i.e. literally_random_access_set members),
  // and keys can't be changed because they're being used as keys, so only
  // provide a const iterator.
  struct select1st_ {
    typedef T const& result_type;
    T const& operator()(contents_value_type_ const& v)const {return v.first;}
  };
public:
  typedef boost::transform_iterator<select1st_, contents_const_iterator_> iterator;
  typedef iterator const_iterator;

  bool insert(T const& t) {
    std::pair<contents_iterator_, bool> iter_and_did_anything_change =
        members_to_indices_.insert(contents_value_type_(t, size()));
    if (iter_and_did_anything_change.second) {
      try {
        pointers_vector_.push_back(&*iter_and_did_anything_change.first);
      }
      catch(...) {
        members_to_indices_.erase(iter_and_did_anything_change.first);
        throw;
      }
    }
    return iter_and_did_anything_change.second;
  }
  bool erase(T const& t) {
    contents_iterator_ i = members_to_indices_.find(t);
    if (i != members_to_indices_.end()) {
      const size_type idx = i->second;
      assert(idx < pointers_vector_.size());
      contents_value_type_*const moved_ptr = pointers_vector_.back();
      moved_ptr->second = idx;
      pointers_vector_[idx] = moved_ptr;
      pointers_vector_.pop_back();
      members_to_indices_.erase(i);
      return true;
    }
    return false;
  }
  template<typename RNG>
  T const& get_random(RNG& rng)const {
    caller_error_if(members_to_indices_.empty(), "Trying to get a random element of an empty literally_random_access_set");
    const boost::random::uniform_int_distribution<size_type> random_item_idx(0, pointers_vector_.size()-1);
    const size_type idx = random_item_idx(rng);
    assert(idx < pointers_vector_.size());
    return pointers_vector_[idx]->first;
  }
  bool empty()const { return members_to_indices_.empty(); }
  size_type size()const { return members_to_indices_.size(); }

  const_iterator begin()const { return const_iterator(members_to_indices_.begin(), select1st_()); }
  const_iterator end()const { return const_iterator(members_to_indices_.end(), select1st_()); }

private:
  std::vector<contents_value_type_*> pointers_vector_;
  contents_type_ members_to_indices_;
};

#endif
