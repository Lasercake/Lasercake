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

#ifndef LASERCAKE_CONCURRENCY_UTILS_HPP__
#define LASERCAKE_CONCURRENCY_UTILS_HPP__

#include "config.hpp" //for LASERCAKE_NO_THREADS

#ifndef LASERCAKE_NO_THREADS
#include <boost/thread.hpp>
#endif
#include <boost/optional.hpp>

// I wish there was a better library out there building on boost::thread/std::thread,
// but I didn't find a well-developed one. Is there?

namespace concurrent {

#ifndef LASERCAKE_NO_THREADS
using boost::thread;
using boost::condition_variable;
using boost::mutex;
using boost::unique_lock;
using boost::lock_guard;
#endif

// m_var mimicing this API:
// http://www.haskell.org/ghc/docs/7.4.1/html/libraries/base-4.5.0.0/Control-Concurrent-MVar.html
// It's a box that can contain zero or one of the contained type, a.k.a. a queue of size one.
// type Contained must be CopyConstructible and Assignable.
template<typename Contained>
class m_var {
public:
  typedef Contained value_type;

  m_var() {}
  m_var(value_type const& initial) : contents_(initial) {}
  
  // Blocks until the m_var contains something:
  
  // Removes and returns the contained value.
  // (Blocks until there is such a value.)
  // Postcondition: m_var is empty.
  value_type take() {
#if !LASERCAKE_NO_THREADS
    unique_lock<mutex> lock(mutex_);
    while(!contents_) {
      readers_wait_on_.wait(lock);
    }
#endif
    value_type result = preconditions_hold_take_();
    return result;
  }

  // Puts a value in the container.
  // (Blocks until there is space in the container.)
  // Postcondition: m_var is full.
  void put(value_type const& v) {
#if !LASERCAKE_NO_THREADS
    unique_lock<mutex> lock(mutex_);
    while(contents_) {
      writers_wait_on_.wait(lock);
    }
#endif
    preconditions_hold_put_(v);
  }

  
  // Nonblocking:
  
  // Removes and returns the contained value, or nothing.
  // (Nonblocking.)
  // Postcondition: m_var is empty.
  boost::optional<value_type> try_take() {
#if !LASERCAKE_NO_THREADS
    lock_guard<mutex> lock(mutex_);
#endif
    boost::optional<value_type> result;
    if(contents_) {
      result.reset(preconditions_hold_take_());
    }
    return result;
  }
  
  // Attempts to put a value in the container.
  // Returns true if successful, false if the container was already full.
  // Postcondition: m_var is full.
  bool try_put(value_type const& v) {
#if !LASERCAKE_NO_THREADS
    lock_guard<mutex> lock(mutex_);
#endif
    if(!contents_) {
      preconditions_hold_put_(v);
      return true;
    }
    else {
      return false;
    }
  }
  
private:
  // Is this the fastest and most correct implementation?
  boost::optional<value_type> contents_;
#if !LASERCAKE_NO_THREADS
  mutex mutex_;
  condition_variable readers_wait_on_;
  condition_variable writers_wait_on_;
#endif
  // utility functions; require a lock to already be taken
  // and the container to already be full/empty as applicable.
  // Use these functions everywhere that modifies contents_.
  void preconditions_hold_put_(value_type const& v) {
    if(LASERCAKE_NO_THREADS) {
      caller_error_if(contents_, "'put' into a full m_var in single-threaded mode blocks.");
    }
    contents_.reset(v);
#if !LASERCAKE_NO_THREADS
    readers_wait_on_.notify_one();
#endif
  }
  value_type preconditions_hold_take_() {
    if(LASERCAKE_NO_THREADS) {
      caller_error_if(!contents_, "'take' from an empty m_var in single-threaded mode blocks.");
    }
    value_type result = contents_.get();
    contents_.reset();
#if !LASERCAKE_NO_THREADS
    writers_wait_on_.notify_one();
#endif
    return result;
  }
};

}




#endif
