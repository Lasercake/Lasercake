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
#include <cstring> // for memset

#include "../utils.hpp"

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

// reminiscent of boost::dynamic_bitset
// should Block be a union instead?
// There is no checking for invalid indexes.
// In fact, after construction, it doesn't even *know* how large it is.
// It's for the other parts of the code to keep track of which parts of that
// they want to.
//
// For bitset operations, on x86_64,
// uint32_t appeared a bit faster than uint64_t as block-size for this.
//
// Block must be an unsigned integral type
//
// This struct is just used as a type-parametrized namespace;
// don't instantiate it.
template <typename Block = uint32_t>
struct bitbuffer_ops {
  typedef Block block_type;
  typedef ::bit_index_type bit_index_type;
  typedef size_t block_index_type;
  typedef int32_t block_width_type;
  static const block_width_type bits_per_block = std::numeric_limits<Block>::digits;

  static block_type* new_zeroed_buffer_with_num_blocks(block_index_type num_blocks) {
    block_type* buffer = new block_type[num_blocks];
    reset_block_range(buffer, 0, num_blocks);
    return buffer;
  }
  static void delete_buffer(block_type*& buffer) {
    if(buffer) {delete[] buffer;}
    buffer = nullptr;
  }

  static void reset_block_range(block_type* buffer, block_index_type begin, block_index_type count) {
    memset(buffer + begin, 0, count * sizeof(block_type));
  }

  static void set_bit(block_type* buffer, bit_index_type pos) {
    buffer[block_index(pos)] |= bit_mask(pos);
  }
  static void reset_bit(block_type* buffer, bit_index_type pos) {
    buffer[block_index(pos)] &= ~bit_mask(pos);
  }
  static void set_bit(block_type* buffer, bit_index_type pos, bool val) {
    buffer[block_index(pos)] |= bit_mask_if(pos, val);
    buffer[block_index(pos)] &= ~bit_mask_if(pos, !val);
  }
  static void flip_bit(block_type* buffer, bit_index_type pos) {
    buffer[block_index(pos)] ^= bit_mask(pos);
  }
  static bool test_bit(block_type const* buffer, bit_index_type pos) {
    return bool(buffer[block_index(pos)] & bit_mask(pos));
  }

  static void set_block(block_type* buffer, block_index_type pos, block_type val) {
    buffer[pos] = val;
  }
  static block_type get_block(block_type const* buffer, block_index_type pos) {
    return buffer[pos];
  }

  static block_index_type block_index(bit_index_type pos) {
    return pos / bits_per_block;

  }
  static block_width_type bit_index(bit_index_type pos) {
    return static_cast<block_width_type>(pos % bits_per_block);
  }
  static block_type bit_mask(bit_index_type pos) {
    return block_type(1) << bit_index(pos);
  }
  static block_type bit_mask_if(bit_index_type pos, bool val) {
    return block_type(val) << bit_index(pos);
  }

  static size_t num_blocks_needed_to_contain_num_bits(bit_index_type num_bits) {
    // Divide rounding up.
    return (num_bits + (bits_per_block - 1)) / bits_per_block;
  }
};

// Block must be an unsigned integral type
template<typename Block>
inline size_t num_blocks_needed_to_contain_num_bits(bit_index_type num_bits) {
  static const bit_index_type bits_per_block = std::numeric_limits<Block>::digits;
  // Divide rounding up.
  return (num_bits + (bits_per_block - 1)) / bits_per_block;
}

struct zeroable_bitset {
  typedef bitbuffer_ops<> bit_ops;
  bit_index_type num_bits;
  std::vector<bit_index_type> bits_to_clear;
  bit_ops::block_type* bits;

  zeroable_bitset(bit_index_type num_bits)
   : num_bits(num_bits),
     bits(bit_ops::new_zeroed_buffer_with_num_blocks(num_blocks_allocated()))
  {}
  ~zeroable_bitset() {
    bit_ops::delete_buffer(bits);
  }
  size_t num_blocks_allocated()const {
    return bit_ops::num_blocks_needed_to_contain_num_bits(num_bits);
  }
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
    #if 0
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
  typedef zeroable_bitset::bit_ops bit_ops;
public:
  explicit borrowed_bitset(bit_index_type num_bits_desired)
   : bs_(borrow_bitset(num_bits_desired)), num_bits_borrowed_(num_bits_desired) {}
  borrowed_bitset() : bs_(nullptr), num_bits_borrowed_(0) {} //default-constructing invalid bitsets is okay
  borrowed_bitset(borrowed_bitset&& other) {
    bs_ = other.bs_; other.bs_ = nullptr;
    num_bits_borrowed_ = other.num_bits_borrowed_;
  }
  bool test(bit_index_type which)const {
    caller_correct_if(which < size(), "borrowed_bitset bounds overflow");
    return bit_ops::test_bit(bs_->here.bits, which);
  }
  bool set(bit_index_type which) {
    bool was_already_set = test(which);
    if(!was_already_set && tracking_bits_individually_()) {
      bs_->here.bits_to_clear.push_back(which);
    }
    bit_ops::set_bit(bs_->here.bits, which);
    return was_already_set;
  }
  bit_index_type size()const {
    return num_bits_borrowed_;
  }
  ~borrowed_bitset() {
    if(!bs_) return;
    if(tracking_bits_individually_()) {
      for(bit_index_type which : bs_->here.bits_to_clear) {
        bit_ops::reset_bit(bs_->here.bits, which);
      }
    }
    else {
      bit_ops::reset_block_range(bs_->here.bits, 0, num_blocks_borrowed_());
    }
    bs_->here.bits_to_clear.clear();
    return_bitset(bs_);
  }
private:
  bool tracking_bits_individually_() const {
    return bs_->here.bits_to_clear.size()
         < (bs_->here.num_bits >> bit_exponent_below_which_its_worth_tracking_bits_individually);
  }
  size_t num_blocks_borrowed_()const {
    return bit_ops::num_blocks_needed_to_contain_num_bits(num_bits_borrowed_);
  }
  zeroable_bitset_node* bs_;
  bit_index_type num_bits_borrowed_;
};


// If you know ahead of time that you're going to set most of the bits,
// this will give a small constant-factor speed improvement over
// borrowed_bitset.
class borrowed_bitset_that_always_clears_using_memset : boost::noncopyable {
  typedef zeroable_bitset::bit_ops bit_ops;
public:
  explicit borrowed_bitset_that_always_clears_using_memset(bit_index_type num_bits_desired)
   : bs_(borrow_bitset(num_bits_desired)), num_bits_borrowed_(num_bits_desired) {}
  borrowed_bitset_that_always_clears_using_memset()
   : bs_(nullptr), num_bits_borrowed_(0) {} //default-constructing invalid bitsets is okay
  borrowed_bitset_that_always_clears_using_memset(borrowed_bitset_that_always_clears_using_memset&& other) {
    bs_ = other.bs_; other.bs_ = nullptr;
    num_bits_borrowed_ = other.num_bits_borrowed_;
  }
  bool test(bit_index_type bit_index)const {
    caller_correct_if(bit_index < size(), "borrowed_bitset bounds overflow");
    return bit_ops::test_bit(bs_->here.bits, bit_index);
  }
  bool set(bit_index_type bit_index) {
    bool was_already_set = test(bit_index);
    bit_ops::set_bit(bs_->here.bits, bit_index);
    return was_already_set;
  }

  // The lowest-order bit in the block is the first bit in the bit-ordering.
  // (i.e., conceptually, it's little-endian, though this doesn't require
  // processor endianness.)
  //
  // The index is specified by number of blocks from the beginning, not
  // number of bits from the beginning.
  //
  // set_block *overwrites* all bits in the block to either 0 or 1 per
  // the val argument.  To modify with e.g. |, get and then set.
  uint32_t get_block_32bit(size_t block_index)const {
    static_assert(std::numeric_limits<bit_ops::block_type>::digits == 32, "bug");
    caller_correct_if(block_index < num_blocks_borrowed_(), "borrowed_bitset bounds overflow");
    return bit_ops::get_block(bs_->here.bits, block_index);
  }
  void set_block_32bit(size_t block_index, uint32_t val) {
    static_assert(std::numeric_limits<bit_ops::block_type>::digits == 32, "bug");
    caller_correct_if(block_index < num_blocks_borrowed_(), "borrowed_bitset bounds overflow");
    bit_ops::set_block(bs_->here.bits, block_index, val);
  }

  bit_index_type size()const {
    return num_bits_borrowed_;
  }
  ~borrowed_bitset_that_always_clears_using_memset() {
    if(!bs_) return;
    bit_ops::reset_block_range(bs_->here.bits, 0, num_blocks_borrowed_());
    return_bitset(bs_);
  }
private:
  size_t num_blocks_borrowed_()const {
    return bit_ops::num_blocks_needed_to_contain_num_bits(num_bits_borrowed_);
  }
  zeroable_bitset_node* bs_;
  bit_index_type num_bits_borrowed_;
};


}

using borrowed_bitset_impl::borrowed_bitset;
using borrowed_bitset_impl::borrowed_bitset_that_always_clears_using_memset;

#endif

