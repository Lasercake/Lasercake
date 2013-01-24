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

#ifndef LASERCAKE_DEQUE_HPP__
#define LASERCAKE_DEQUE_HPP__

#include <boost/iterator/iterator_facade.hpp>
#include "numbers.hpp"

// This is a double-ended queue that
// (1) is like std::vector in memory-efficiency and insertion complexity
//       (amortized constant time via exponentially larger reallocation)
// (2) iterators are invalidated by almost every mutating operation
//       (which is its main semantic disadvantage compared to std::deque).

// It is based on a circular buffer of power-of-two size.

// All mutating operations invalidate all iterators, except for
// pop_back(), which invalidates no iterators (though it obviously
// makes the iterator to the popped element non-dereferenceable).

template<typename T, bool IsReverse = false>
class deque_iterator : public boost::iterator_facade<
  deque_iterator<T, IsReverse>, T, std::random_access_iterator_tag
> {
  static const int reverseness_factor = (IsReverse ? -1 : 1);
  typedef uint32_t size_type;
public:

  template<typename SrcT, bool SrcIsReverse>
  deque_iterator(deque_iterator<SrcT, SrcIsReverse> const& i)
    : base_(i.base_), alloc_(i.alloc_), idx_(i.idx_), which_piece_(i.which_piece_) {}

  //for deque to use:
  deque_iterator(T* base, size_type alloc, size_type idx, bool which_piece)
    : base_(base), alloc_(alloc), idx_(idx), which_piece_(which_piece) {}

private:
  T* dereference()const {
    if(IsReverse) {return base_ + ((idx_-1)&alloc_);}
    else {return base_ + idx_;}
  }
  bool equal(deque_iterator const& other)const {
    return idx_ == other.idx_ && which_piece_ == other.which_piece_;
  }
  void increment() {
    if(IsReverse) {decrement_impl();} else {increment_impl();}
  }
  void decrement() {
    if(IsReverse) {increment_impl();} else {decrement_impl();}
  }
  void increment_impl() {
    which_piece_ = which_piece_ ^ (idx_ == alloc_);
    idx_ = (idx_ + 1) & alloc_;
  }
  void decrement_impl() {
    which_piece_ = which_piece_ ^ (idx_ == 0);
    idx_ = (idx_ - 1) & alloc_;
  }
  void advance(ptrdiff_t n) {
    n *= reverseness_factor;
    bool overflow = size_type(idx_ + n) > alloc_;
    which_piece_ = which_piece_ ^ overflow;
    idx_ = size_type(idx_ + n) & alloc_;
  }
  ptrdiff_t distance_to(deque_iterator const& i) {
    ptrdiff_t i_logical_idx = i.idx_ + i.which_piece_*(alloc_+1);
    ptrdiff_t logical_idx = idx_ + which_piece_*(alloc_+1);
    return reverseness_factor * (i_logical_idx - logical_idx);
  }
  friend class boost::iterator_core_access;
private:
  T* base_;
  size_type alloc_;
  size_type idx_;
  // which_piece_ is false at begin() and true as soon (imagining
  // incrementing repeatedly) as you wrap around from max idx_ to
  // min idx_.
  bool which_piece_;
};

template<typename T, typename Alloc = std::allocator<T> >
class deque {
public:
  typedef T value_type;
  typedef T& reference;
  typedef T* pointer;
  typedef const T& const_reference;
  typedef const T* const_pointer;
  typedef uint32_t size_type;
  typedef ptrdiff_t difference_type;

  typedef deque_iterator<T> iterator;
  typedef deque_iterator<const T> const_iterator;
  typedef deque_iterator<T, true> reverse_iterator;
  typedef deque_iterator<const T, true> const_reverse_iterator;

  deque() : alloc_begin_(0), alloc_(0), contents_size_(0), contents_first_idx_(0), contents_last_idx_(0) {}
  ~deque() { if(alloc_begin_) { clear(); Alloc().deallocate(alloc_begin_, alloc_+1); } }

  iterator begin()             { return       iterator(alloc_begin_, alloc_, contents_first_idx_, false); }
  const_iterator begin()const  { return const_iterator(alloc_begin_, alloc_, contents_first_idx_, false); }
  const_iterator cbegin()const { return const_iterator(alloc_begin_, alloc_, contents_first_idx_, false); }
  iterator end()               { size_type idx = (contents_last_idx_+1)&alloc_; return       iterator(alloc_begin_, alloc_, idx, contents_size_ && idx <= contents_first_idx_); }
  const_iterator end()const    { size_type idx = (contents_last_idx_+1)&alloc_; return const_iterator(alloc_begin_, alloc_, idx, contents_size_ && idx <= contents_first_idx_); }
  const_iterator cend()const   { size_type idx = (contents_last_idx_+1)&alloc_; return const_iterator(alloc_begin_, alloc_, idx, contents_size_ && idx <= contents_first_idx_); }
  reverse_iterator rbegin()             { return       reverse_iterator(end()); }
  const_reverse_iterator rbegin()const  { return const_reverse_iterator(end()); }
  const_reverse_iterator crbegin()const { return const_reverse_iterator(end()); }
  reverse_iterator rend()             { return       reverse_iterator(begin()); }
  const_reverse_iterator rend()const  { return const_reverse_iterator(begin()); }
  const_reverse_iterator crend()const { return const_reverse_iterator(begin()); }

  //hmph the empty space is only for the sake of iterators, and we can represent end somehow separately,
  //and often don't *need* to iterate queues! so. not doin'.
  //ohh intersting and need a separate 'empty' repr. ok.
  void push_front(T const& t) {
    size_type dest_idx = (contents_first_idx_ - 1) & alloc_;
    if (dest_idx == contents_last_idx_) {
      do_realloc_(false); //'false' puts the empty space at the beginning
      dest_idx = contents_first_idx_ - 1; //so (& alloc_) not needed here
    }
    Alloc().construct(alloc_begin_ + dest_idx, t);
    contents_first_idx_ = dest_idx;
    ++contents_size_;
  }
  void push_back(T const& t) {
    size_type dest_idx = (contents_last_idx_ + 1) & alloc_;
    if (dest_idx == contents_first_idx_) {
      do_realloc_(true); //'true' puts the empty space at the end
      dest_idx = contents_last_idx_ + 1; //so (& alloc_) not needed here
    }
    Alloc().construct(alloc_begin_ + dest_idx, t);
    contents_last_idx_ = dest_idx;
    ++contents_size_;
  }
  void pop_front() {
    caller_error_if(empty(), "pop_front() from empty queue");
    Alloc().destroy(alloc_begin_ + contents_first_idx_);
    contents_first_idx_ = (contents_first_idx_ + 1) & alloc_;
    --contents_size_;
  }
  void pop_back() {
    caller_error_if(empty(), "pop_back() from empty queue");
    Alloc().destroy(alloc_begin_ + contents_last_idx_);
    contents_last_idx_ = (contents_last_idx_ - 1) & alloc_;
    --contents_size_;
  }
  T& front() { return *(alloc_begin_ + contents_first_idx_); }
  T const& front()const { return *(alloc_begin_ + contents_first_idx_); }
  T& back() { return *(alloc_begin_ + contents_last_idx_); }
  T const& back()const { return *(alloc_begin_ + contents_last_idx_); }

  // bias default chooses push_back in line with how std::queue uses its member collection
  void reserve(size_type size, bool bias_towards_push_back = true) {
    size_type old_alloc_size = alloc_ + !!alloc_begin_;
    if(size <= old_alloc_size) return;
    // 'size' now >= 1.
    // Round up to a power of two:
    const size_type new_alloc_size = size_type(1) << ilog2(size);
    do_realloc_(old_alloc_size, new_alloc_size, bias_towards_push_back);
  }
  bool empty()const {return contents_size_ == 0;}
  size_type size()const {
    return contents_size_;
  }
  size_type max_size()const {
    // If you increase this to 2**32, beware the fact that
    // 2**32 is not even a valid uint32_t, and change do_realloc_(), reserve(),
    // and functions that can increase contents_size_ correspondingly.
    return size_type(1) << 31;
  }
  void clear() {
    if(alloc_begin_) {
      if(contents_first_idx_ <= contents_last_idx_) {
        T* range_begin = alloc_begin_+contents_first_idx_;
        T* range_end = alloc_begin_+contents_last_idx_+1;
        for(T* i = range_begin; i != range_end; ++i) { Alloc().destroy(i); }
      }
      else {
        T* range1_begin = alloc_begin_+contents_first_idx_;
        T* range1_end = alloc_begin_+alloc_+1;
        T* range2_begin = alloc_begin_;
        T* range2_end = alloc_begin_+contents_last_idx_+1;
        for(T* i = range1_begin; i != range1_end; ++i) { Alloc().destroy(i); }
        for(T* i = range2_begin; i != range2_end; ++i) { Alloc().destroy(i); }
      }
      contents_size_ = 0;
      contents_first_idx_ = 0;
      contents_last_idx_ = alloc_;
    }
  }
  void swap(deque& other){
    std::swap(alloc_begin_, other.alloc_begin_);
    std::swap(alloc_, other.alloc_);
    std::swap(contents_size_, other.contents_size_);
    std::swap(contents_first_idx_, other.contents_first_idx_);
    std::swap(contents_last_idx_, other.contents_last_idx_);
  }

private:
  void do_realloc_(bool bias_towards_push_back) {
    if(alloc_ == ((size_type(1)<<31) - 1)) {
      throw std::bad_alloc(); //"attempt to exceed queue max_size()");
    }
    // Double the size, or allocate 4 elements if it was unallocated.
    const size_type old_alloc_size = alloc_ + !!alloc_begin_;
    const size_type new_alloc_size = (old_alloc_size << 1) + (!alloc_begin_ << 2);
    do_realloc_(old_alloc_size, new_alloc_size, bias_towards_push_back);
  }
  void do_realloc_(size_type old_alloc_size, size_type new_alloc_size, bool bias_towards_push_back) {
    T* new_alloc_begin = Alloc().allocate(new_alloc_size);
    size_type new_contents_first_idx = (bias_towards_push_back ? 0 : new_alloc_size - contents_size_);
    size_type new_contents_last_idx = new_contents_first_idx + contents_size_ - 1;
    T* dest_begin = new_alloc_begin + new_contents_first_idx;
    if(alloc_begin_) {
      // TODO member-copy-constructor exception safety?
      // This implementation provides the 'basic' guarantee,
      // which isn't that useful, or the 'strong' guarantee if T
      // is nothrow movable.
      try {
        if(contents_first_idx_ <= contents_last_idx_) {
          T* range_begin = alloc_begin_+contents_first_idx_;
          T* range_end = alloc_begin_+contents_last_idx_+1;
          std::move(range_begin, range_end, dest_begin);
          for(T* i = range_begin; i != range_end; ++i) { Alloc().destroy(i); }
        }
        else {
          T* range1_begin = alloc_begin_+contents_first_idx_;
          T* range1_end = alloc_begin_+old_alloc_size;
          T* range2_begin = alloc_begin_;
          T* range2_end = alloc_begin_+contents_last_idx_+1;
          T* dest_begin2 = new_alloc_begin + (old_alloc_size - contents_first_idx_);
          std::move(range1_begin, range1_end, dest_begin);
          std::move(range2_begin, range2_end, dest_begin2);
          for(T* i = range1_begin; i != range1_end; ++i) { Alloc().destroy(i); }
          for(T* i = range2_begin; i != range2_end; ++i) { Alloc().destroy(i); }
        }
        Alloc().deallocate(alloc_begin_, old_alloc_size);
      } catch(...) {
        Alloc().deallocate(new_alloc_begin, new_alloc_size);
        throw;
      }
    }
    alloc_begin_ = new_alloc_begin;
    alloc_ = new_alloc_size - 1;
    contents_first_idx_ = new_contents_first_idx;
    contents_last_idx_ = new_contents_last_idx;
  }
  //either both nullptr, or first and last valid pointer ("last" is *not* a past-the-end pointer).
  T* alloc_begin_;
  // alloc_ is (2**n - 1) for some n, so it can be used with & for mod,
  // and also 0 when alloc_begin_ == nullptr.
  size_type alloc_;
  size_type contents_size_;
  size_type contents_first_idx_;
  size_type contents_last_idx_;
};


#endif
