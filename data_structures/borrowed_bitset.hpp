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

#ifndef LASERCAKE_BORROWED_BITSET_HPP__
#define LASERCAKE_BORROWED_BITSET_HPP__

#include <list>
#include <stack>
#include <boost/utility.hpp>
#include <boost/dynamic_bitset.hpp>

#include "../utils.hpp"
#if 0 && !LASERCAKE_NO_THREADS
#include <boost/thread/thread.hpp>
#endif

typedef uint64_t bit_index_type;

namespace borrowed_bitset_impl {

// Noncopying singly linked list that owns its members unless you pop them.
template<typename T>
struct liststack_node : boost::noncopyable {
  liststack_node* next;
  T here;

  liststack_node() : next(nullptr), here() {}
  template<typename Arg>
  explicit liststack_node(Arg const& arg, liststack_node* next = nullptr)
   : next(next), here(arg) {}
  ~liststack_node() { delete next; }
};

template<typename T>
struct liststack : boost::noncopyable {
  typedef borrowed_bitset_impl::liststack_node<T> liststack_node;
  liststack_node* next;

  liststack() : next(nullptr) {}
  explicit liststack(liststack_node* next) : next(next) {}
  ~liststack() { delete next; }

  liststack_node* pop() {
    liststack_node* result = next;
    next = result->next;
    // for clarity:
    result->next = nullptr;
    return result;
  }
  void push(liststack_node* add) {
    add->next = next;
    next = add;
  }
  bool empty() {
    return next == nullptr;
  }
};

  
struct zeroable_bitset {
  // until this implementation becomes profile-measurable, it's good enough.
  bit_index_type num_bits;
  boost::dynamic_bitset<> bits;
  std::vector<bit_index_type> bits_to_clear;

  zeroable_bitset(bit_index_type num_bits)
   : num_bits(num_bits), bits(num_bits), bits_to_clear() {}
};

typedef liststack<zeroable_bitset> zeroable_bitset_list;
// node/iterator/such
typedef liststack_node<zeroable_bitset> zeroable_bitset_node;

static const size_t array_of_bitset_lists_len = 40;
static const size_t bit_to_min_size_exponent = 6;
// if we track 4-8byteses individually, this exponent would be smaller.
static const size_t bit_exponent_below_which_its_worth_tracking_bits_individually = 8;

// array of array_of_bitset_lists_len sizes, from 2**bit_to_min_size_exponent bits up.
typedef zeroable_bitset_list* zeroable_bitset_array;

extern thread_local zeroable_bitset_array array_of_bitset_lists;

inline
void delete_array_of_bitset_lists() { delete[] array_of_bitset_lists; }

inline
zeroable_bitset_array get_this_thread_array_of_bitset_lists() {
  if(array_of_bitset_lists == nullptr) {
    array_of_bitset_lists = new zeroable_bitset_list[array_of_bitset_lists_len];
    #if 0 && !LASERCAKE_NO_THREADS
    // No Boost.Thread for now; fewer deps.
    // Currently, the borrowed_bitset won't leak memory in Lasercake
    // because we only create a small, finite number of threads.
    // We'll use some better solution if that changes (TODO).
      try {
        // (note, if we use forcible thread cancelation, this won't run)
        // Also, donating them to another thread might be more useful
        // than deleting them.
        boost::this_thread::at_thread_exit(delete_array_of_bitset_lists);
      }
      catch(...) {
        delete array_of_bitset_lists;
        array_of_bitset_lists = nullptr;
        throw;
      }
    #endif
  }
  return array_of_bitset_lists;
};

inline size_t which_bitset_index_is_size(bit_index_type num_bits) {
  if(num_bits == 0) return 0;
  const bit_index_type max_bit_index = num_bits - 1;
  const bit_index_type max_8byte_index = max_bit_index >> bit_to_min_size_exponent;
  caller_error_if(max_8byte_index != size_t(max_8byte_index), "More bits requested than your architecture can handle!");
  const size_t which_bitset_size_index = num_bits_in_integer_that_are_not_leading_zeroes(max_8byte_index);
  caller_error_if(which_bitset_size_index >= array_of_bitset_lists_len, "More bits requested than we support!");
  return which_bitset_size_index;
}

inline bit_index_type how_many_bits_at_bitset_index(size_t bitset_index) {
  return bit_index_type(1) << (bitset_index + bit_to_min_size_exponent);
}

inline
zeroable_bitset_node* borrow_bitset(bit_index_type num_bits_desired) {
  zeroable_bitset_array array_of_bitset_lists = get_this_thread_array_of_bitset_lists();
  const size_t which_bitset_size_index = which_bitset_index_is_size(num_bits_desired);
  const bit_index_type actual_bits = how_many_bits_at_bitset_index(which_bitset_size_index);
  assert(actual_bits >= num_bits_desired);
  assert(actual_bits < num_bits_desired*2 || actual_bits == (1<<bit_to_min_size_exponent));
  zeroable_bitset_list& result_list = array_of_bitset_lists[which_bitset_size_index];
  if(!result_list.empty()) {
    zeroable_bitset_node* result = result_list.pop();
    assert(result->here.num_bits == actual_bits);
    return result;
  }
  else {
    return new zeroable_bitset_node(actual_bits);
  }
}

inline void return_bitset(zeroable_bitset_node* node) BOOST_NOEXCEPT {
  zeroable_bitset_array array_of_bitset_lists;
  try {
    array_of_bitset_lists = get_this_thread_array_of_bitset_lists();
  }
  catch(std::exception const&) {
    // This function is called from destructors so it must be nothrow.
    // In particular, this may happen if the destructor is called in a different
    // thread than the constructor was (std::bad_alloc or similar).
    delete node;
    return;
  }
  const size_t which_bitset_size_index = which_bitset_index_is_size(node->here.num_bits);
  array_of_bitset_lists[which_bitset_size_index].push(node);
}




// borrowed_bitset(n) borrows an array of zero bits of size n (rounded
// up to something), and when destructed it restores the zeroes and returns
// that array.  (There's a dynamically allocated pool of arrays of zeroes
// waiting to be borrowed; if none are available, borrowed_bitset allocates
// a new one, which it will return to the pool when destructed.)
//
// If there aren't too many borrowed_bitsets at once and program runs for
// a while, borrowed_bitset() effectively has O(1) construction, and
// destruction time <= the number of operations performed on it (so no worse
// than a constant factor). This is pretty good for something this much
// faster at uniquing than an unordered_set<uint>.
class borrowed_bitset : boost::noncopyable {
public:
  explicit borrowed_bitset(bit_index_type num_bits_desired) : bs_(borrow_bitset(num_bits_desired)) {}
  borrowed_bitset() : bs_(nullptr) {} //default-constructing invalid bitsets is okay
  borrowed_bitset(borrowed_bitset&& other) { bs_ = other.bs_; other.bs_ = nullptr; }
  bool test(bit_index_type which)const {
    caller_correct_if(which < size(), "borrowed_bitset bounds overflow");
    return bs_->here.bits.test(which);
  }
  bool set(bit_index_type which) {
    bool was_already_set = test(which);
    if(!was_already_set && tracking_bits_individually_()) {
      bs_->here.bits_to_clear.push_back(which);
    }
    bs_->here.bits.set(which);
    return was_already_set;
  }
  bit_index_type size()const {
    // Implementation detail: this number might be greater than the number
    // requested by the constructor.  We could store the requested number,
    // but is it important to?
    return bs_->here.num_bits;
  }
  ~borrowed_bitset() {
    if(!bs_) return;
    if(tracking_bits_individually_()) {
      for(bit_index_type which : bs_->here.bits_to_clear) {
        bs_->here.bits.reset(which);
      }
    }
    else {
      bs_->here.bits.reset();
    }
    bs_->here.bits_to_clear.clear();
    return_bitset(bs_);
  }
private:
  bool tracking_bits_individually_() const {
    return bs_->here.bits_to_clear.size()
         < (bs_->here.num_bits >> bit_exponent_below_which_its_worth_tracking_bits_individually);
  }
  zeroable_bitset_node* bs_;
};


// If you know ahead of time that you're going to set most of the bits,
// this will give a small constant-factor speed improvement over
// borrowed_bitset.
class borrowed_bitset_that_always_clears_using_memset : boost::noncopyable {
public:
  explicit borrowed_bitset_that_always_clears_using_memset(bit_index_type num_bits_desired)
   : bs_(borrow_bitset(num_bits_desired)) {}
  borrowed_bitset_that_always_clears_using_memset()
   : bs_(nullptr) {} //default-constructing invalid bitsets is okay
  borrowed_bitset_that_always_clears_using_memset(borrowed_bitset_that_always_clears_using_memset&& other)
    { bs_ = other.bs_; other.bs_ = nullptr; }
  bool test(bit_index_type which)const {
    caller_correct_if(which < size(), "borrowed_bitset bounds overflow");
    return bs_->here.bits.test(which);
  }
  bool set(bit_index_type which) {
    bool was_already_set = test(which);
    bs_->here.bits.set(which);
    return was_already_set;
  }
  bit_index_type size()const {
    // Implementation detail: this number might be greater than the number
    // requested by the constructor.  We could store the requested number,
    // but is it important to?
    return bs_->here.num_bits;
  }
  ~borrowed_bitset_that_always_clears_using_memset() {
    if(!bs_) return;
    bs_->here.bits.reset();
    return_bitset(bs_);
  }
private:
  zeroable_bitset_node* bs_;
};


}

using borrowed_bitset_impl::borrowed_bitset;
using borrowed_bitset_impl::borrowed_bitset_that_always_clears_using_memset;

#endif

