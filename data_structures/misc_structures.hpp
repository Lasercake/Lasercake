/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

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

#ifndef LASERCAKE_MISC_STRUCTURES_HPP__
#define LASERCAKE_MISC_STRUCTURES_HPP__

#include <utility>
#include <functional>
#include <vector>
#include "../cxx11/unordered_map.hpp"
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
class randomized_vector {
private:
  typedef std::vector<T> contents_type_;
public:
  typedef typename contents_type_::iterator iterator;
  typedef typename contents_type_::const_iterator const_iterator;

  template<typename RNG>
  void insert(T const& t, RNG& rng) {
    size_t new_max_idx = contents_.size();
    contents_.push_back(t);
    const boost::random::uniform_int_distribution<size_t> random_item_idx(0, new_max_idx);
    std::swap(contents_[new_max_idx], contents_[random_item_idx(rng)]);
  }
  void clear() { contents_.clear(); }
  bool empty()const { return contents_.empty(); }
  size_t size()const { return contents_.size(); }
  iterator begin() { return contents_.begin(); }
  iterator end() { return contents_.end(); }
  const_iterator begin()const { return contents_.begin(); }
  const_iterator end()const { return contents_.end(); }

private:
  contents_type_ contents_;
};

template<typename T>
class literally_random_access_set {
public:
  typedef size_t size_type;
private:
  typedef unordered_map<T, size_type> contents_type_;
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


// The current representation can happily be changed.
union small_string {
public:
  struct info {
    static constexpr size_t max_length = 23;
    static constexpr size_t max_len_incl_nul = max_length + 1;
    static constexpr size_t eightbytelen = (max_len_incl_nul + 7) / 8;
    static constexpr size_t data_len = eightbytelen * 8;
    static_assert(data_len >= max_len_incl_nul, "bug");
    static_assert(data_len > max_length, "bug");
  };

  template<size_t N>
  /*implicit*/ constexpr small_string(const char(&lit)[N])
  : buf_{
    at_(lit, 0), at_(lit, 1), at_(lit, 2), at_(lit, 3), at_(lit, 4),
    at_(lit, 5), at_(lit, 6), at_(lit, 7), at_(lit, 8), at_(lit, 9),
    at_(lit, 10), at_(lit, 11), at_(lit, 12), at_(lit, 13), at_(lit, 14),
    at_(lit, 15), at_(lit, 16), at_(lit, 17), at_(lit, 18), at_(lit, 19),
    at_(lit, 20), at_(lit, 21), at_(lit, 22), at_(lit, 23)
  }
  {
    static_assert(N <= info::max_length, "String literal too long for small_string.");
    static_assert(N > 0, "Non-null-terminated string literal.");
  }
  constexpr char operator[](size_t i) {
    return constexpr_require_and_return(i < info::data_len, "bounds overflow", buf_[i]);
  }

  explicit small_string(const char* s) {
    size_t i = 0;
    for(; s[i] != '\0'; ++i) {
      caller_error_if(i >= info::max_length, "string too large for small_string");
      buf_[i] = s[i];
    }
    for(; i < info::data_len; ++i) {
      buf_[i] = '\0';
    }
  }

  explicit small_string(const std::string s) {
    caller_correct_if(s.size() <= info::max_length, "string too large for small_string");
    size_t i = 0;
    for(; i < s.size(); ++i) {
      buf_[i] = s[i];
    }
    for(; i < info::data_len; ++i) {
      buf_[i] = '\0';
    }
  }

  friend inline bool operator==(small_string a, small_string b) {
    return a.buf64_[0] == b.buf64_[0] && a.buf64_[1] == b.buf64_[1] && a.buf64_[2] == b.buf64_[2];
  }
  friend inline bool operator!=(small_string a, small_string b) {
    return a.buf64_[0] != b.buf64_[0] || a.buf64_[1] != b.buf64_[1] || a.buf64_[2] != b.buf64_[2];
  }
  // these comparison operators if using buf64 would have to think about
  // endianness, which there is not a standardized way to do.
  friend inline bool operator<(small_string a, small_string b) {
    for(size_t i = 0; i != small_string::info::data_len; ++i) {
      if(a[i] < b[i]){return true;}
      if(a[i] > b[i]){return false;}
    }
    return false;
  }
  friend inline bool operator>(small_string a, small_string b) {
    return b < a;
  }
  friend inline bool operator<=(small_string a, small_string b) {
    return !(b < a);
  }
  friend inline bool operator>=(small_string a, small_string b) {
    return !(a < b);
  }
  friend inline std::ostream& operator<<(std::ostream& os, small_string a) {
    return os << a.buf_;
  }
private:
  char buf_[info::data_len];
  uint64_t buf64_[info::eightbytelen];

  template<size_t N>
  static constexpr char at_(const char(&literal)[N], size_t i) {
    return (i < N) ? (literal[i]) : '\0';
  }
};


#endif
