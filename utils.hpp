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

#ifndef LASERCAKE_UTILS_HPP__
#define LASERCAKE_UTILS_HPP__

#include <array>
#include <cmath>
#include <inttypes.h>
#include <boost/functional/hash.hpp>
#include <boost/throw_exception.hpp>
#include <stdexcept>

#ifndef ATTRIBUTE_NORETURN
// from http://www.boost.org/doc/libs/1_48_0/boost/exception/detail/attribute_noreturn.hpp
#if defined(_MSC_VER)
#define ATTRIBUTE_NORETURN __declspec(noreturn)
#elif defined(__GNUC__)
#define ATTRIBUTE_NORETURN __attribute__((noreturn))
#else
#define ATTRIBUTE_NORETURN
#endif
#endif

// It's not polite for library functions to assert() because the library's users
// misused a correct library; use these for that case.
inline ATTRIBUTE_NORETURN void caller_error(std::string error) {
  // If exceptions prove worse for debugging than asserts/segfaults,
  // feel free to comment this out and use asserts/segfaults/breakpoints.
  boost::throw_exception(std::logic_error(error));
}
// You must provide an explanatory string so that the user of the library
// will know what *they* did wrong, and not have to interpret an assert() expression
// to find out.
inline void caller_error_if(bool cond, std::string error) {
  if(cond) {
    caller_error(error);
  }
}
inline void caller_correct_if(bool cond, std::string error) {
  if(!cond) {
    caller_error(error);
  }
}


template <typename SignedType>
inline SignedType sign(SignedType input) {
  return (input > 0) - (input < 0);
}

template<typename Map>
typename Map::mapped_type* find_as_pointer(Map& m, typename Map::key_type const& k) {
  auto i = m.find(k);
  if(i == m.end()) return nullptr;
  else return &(i->second);
}

template<typename Map>
typename Map::mapped_type const* find_as_pointer(Map const& m, typename Map::key_type const& k) {
  auto i = m.find(k);
  if(i == m.end()) return nullptr;
  else return &(i->second);
}

template<typename ScalarType> ScalarType divide_rounding_towards_zero(ScalarType dividend, ScalarType divisor)
{
  caller_correct_if(divisor != 0, "divisor must be nonzero");
  const ScalarType abs_result = std::abs(dividend) / std::abs(divisor);
  if ((dividend > 0) == (divisor > 0)) return abs_result;
  else return -abs_result;
}


inline uint32_t i64sqrt(uint64_t radicand)
{
  if(radicand == 0)return 0;
  
  int shift = radicand & (((1ULL << 32) - 1) << 32) ? 32 : 0;
  shift += radicand & ((((1ULL << 16) - 1) << 16) << shift) ? 16 : 0;
  shift += radicand & ((((1ULL << 8) - 1) << 8) << shift) ? 8 : 0;
  shift += radicand & ((((1ULL << 4) - 1) << 4) << shift) ? 4 : 0;
  shift += radicand & ((((1ULL << 2) - 1) << 2) << shift) ? 2 : 0;
  //I would just lose this piece of accuracy when I divide shift by 2 below:
  //shift += radicand & ((((uint64_t)1 << 1) - 1) << shift) ? 1 : 0;
  
  //shift should now be the log base 2 of radicand, rounded down.
  uint32_t lower_bound = 1 << (shift >> 1);
  
  //replace the lost accuracy:
  if(radicand & (((((uint64_t)1 << 1) - 1) << 1) << shift))lower_bound = (uint32_t)((lower_bound * 6074000999ULL) >> 32); //approximate the square root of 2
  
  uint64_t upper_bound = (uint64_t)lower_bound << 1;
  //lower_bound is guaranteed to be less than or equal to the answer
  //upper_bound is guaranteed to be greater than the answer
  
  while(lower_bound < upper_bound - 1)
  {
    const uint32_t mid = (uint32_t)((upper_bound + lower_bound) >> 1);
    if((uint64_t)mid * mid > radicand)upper_bound = mid;
    else lower_bound = mid;
  }
  
  return lower_bound;
}

template<typename ScalarType> class vector3 {
public:
  ScalarType x, y, z;
  vector3():x(0),y(0),z(0){}
  vector3(ScalarType x, ScalarType y, ScalarType z):x(x),y(y),z(z){}
  template<typename OtherType> explicit vector3(vector3<OtherType> const& other):
    x(other.x),y(other.y),z(other.z){}
  
  ScalarType& operator[](int index) {
    switch(index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
      default: caller_error("Trying to index a vector3 with an out-of-bounds index!");
    }
  }
  ScalarType operator[](int index)const {
    switch(index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
      default: caller_error("Trying to index a vector3 with an out-of-bounds index!");
    }
  }
  // Note: The operators are biased towards the type of the left operand (e.g. vector3<int> + vector3<int64_t> = vector3<int>)
  template<typename OtherType> vector3 operator+(vector3<OtherType> const& other)const {
    return vector3(x + other.x, y + other.y, z + other.z);
  }
  template<typename OtherType> vector3& operator+=(vector3<OtherType> const& other) {
    x += other.x; y += other.y; z += other.z; return *this;
  }
  template<typename OtherType> vector3 operator-(vector3<OtherType> const& other)const {
    return vector3(x - other.x, y - other.y, z - other.z);
  }
  template<typename OtherType> vector3& operator-=(vector3<OtherType> const& other) {
    x -= other.x; y -= other.y; z -= other.z; return *this;
  }
  vector3 operator*(ScalarType other)const {
    return vector3(x * other, y * other, z * other);
  }
  vector3& operator*=(ScalarType other) {
    x *= other; y *= other; z *= other; return *this;
  }
  vector3 operator/(ScalarType other)const {
    return vector3(divide_rounding_towards_zero(x, other), divide_rounding_towards_zero(y, other), divide_rounding_towards_zero(z, other));
  }
  vector3& operator/=(ScalarType other) {
    x = divide_rounding_towards_zero(x, other); y = divide_rounding_towards_zero(y, other); z = divide_rounding_towards_zero(z, other); return *this;
  }
  // Careful, shift operators on builtin types (ScalarType?) are only
  // defined for shift >= 0 && shift < bits_in_type
  vector3 operator<<(int shift)const {
    return vector3(x << shift, y << shift, z << shift);
  }
  vector3& operator<<=(int shift) {
    x <<= shift; y <<= shift; z <<= shift; return *this;
  }
  vector3 operator>>(int shift)const {
    return vector3(x >> shift, y >> shift, z >> shift);
  }
  vector3& operator>>=(int shift) {
    x >>= shift; y >>= shift; z >>= shift; return *this;
  }
  vector3 operator+()const { return *this; } // unary plus
  vector3 operator-()const { // unary minus
    return vector3(-x, -y, -z);
  }

  bool operator==(vector3 const& other)const {return x == other.x && y == other.y && z == other.z; }
  bool operator!=(vector3 const& other)const {return x != other.x || y != other.y || z != other.z; }
  
  // Do not try to use this if either vector has an unsigned ScalarType. It might work in some situations, but why would you ever do that anyway?
  // You are required to specify an output type, because of the risk of overflow. Make sure to choose one that can fit the squares of the numbers you're dealing with.
  template<typename OutputType, typename OtherType> OutputType dot(vector3<OtherType> const& other)const {
    return (OutputType)x * (OutputType)other.x +
           (OutputType)y * (OutputType)other.y +
           (OutputType)z * (OutputType)other.z;
  }
  
  ScalarType magnitude_within_32_bits()const { return (ScalarType)i64sqrt(dot<int64_t>(*this)); }
  
  // Choose these the way you'd choose dot's output type (see the comment above)
  // we had trouble making these templates, so now they just always use int64_t
  bool magnitude_within_32_bits_is_less_than(ScalarType amount)const {
    return dot<int64_t>(*this) < (int64_t)amount * (int64_t)amount;
  }
  bool magnitude_within_32_bits_is_greater_than(ScalarType amount)const {
    return dot<int64_t>(*this) > (int64_t)amount * (int64_t)amount;
  }
  bool operator<(vector3 const& other)const { return (x < other.x) || ((x == other.x) && ((y < other.y) || ((y == other.y) && (z < other.z)))); }
};
template<typename T> inline std::ostream& operator<<(std::ostream& os, vector3<T>const& v) {
  return os << '(' << v.x << ',' << v.y << ',' << v.z << ')';
}

namespace std {
  template<typename ScalarType> struct hash<vector3<ScalarType> > {
    inline size_t operator()(vector3<ScalarType> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.x);
      boost::hash_combine(seed, v.y);
      boost::hash_combine(seed, v.z);
      return seed;
    }
  };
}

typedef int8_t neighboring_tile_differential;
typedef int8_t cardinal_direction_index;
const cardinal_direction_index num_cardinal_directions = 6;
struct cardinal_direction {
  cardinal_direction(vector3<neighboring_tile_differential> v, cardinal_direction_index i):v(v),cardinal_direction_idx(i){}
  vector3<neighboring_tile_differential> v;
  cardinal_direction_index cardinal_direction_idx;
  cardinal_direction operator-()const;
  int which_dimension()const { return (int)(cardinal_direction_idx % 3); } // relies on the current order of the directions
};

template<typename ScalarType> inline vector3<ScalarType> project_onto_cardinal_direction(vector3<ScalarType> src, cardinal_direction dir) {
  return vector3<ScalarType>(src.x * std::abs((ScalarType)dir.v.x), src.y * std::abs((ScalarType)dir.v.y), src.z * std::abs((ScalarType)dir.v.z));
}

const vector3<neighboring_tile_differential> xunitv(1, 0, 0);
const vector3<neighboring_tile_differential> yunitv(0, 1, 0);
const vector3<neighboring_tile_differential> zunitv(0, 0, 1);
// the order of this must be in sync with the order of hacky_vector_indexing_internals::cardinal_direction_vector_to_index
const cardinal_direction cdir_xminus = cardinal_direction(-xunitv, 0);
const cardinal_direction cdir_yminus = cardinal_direction(-yunitv, 1);
const cardinal_direction cdir_zminus = cardinal_direction(-zunitv, 2);
const cardinal_direction cdir_xplus = cardinal_direction(xunitv, 3);
const cardinal_direction cdir_yplus = cardinal_direction(yunitv, 4);
const cardinal_direction cdir_zplus = cardinal_direction(zunitv, 5);
const cardinal_direction cardinal_directions[num_cardinal_directions] = { cdir_xminus, cdir_yminus, cdir_zminus, cdir_xplus, cdir_yplus, cdir_zplus };
#define EACH_CARDINAL_DIRECTION(varname) cardinal_direction varname : cardinal_directions
inline cardinal_direction cardinal_direction::operator-()const { return cardinal_directions[(cardinal_direction_idx + 3)%6]; }

template<typename ValueType> class value_for_each_cardinal_direction {
public:
  explicit value_for_each_cardinal_direction(ValueType const& iv/*initial_value*/) : data({{iv,iv,iv,iv,iv,iv}}) { static_assert(num_cardinal_directions == 6, "fix {{iv,iv,...}} to have the right number"); }
  ValueType      & operator[](cardinal_direction const& dir) { return data[dir.cardinal_direction_idx]; }
  ValueType const& operator[](cardinal_direction const& dir)const { return data[dir.cardinal_direction_idx]; }
private:
  typedef std::array<ValueType, num_cardinal_directions> internal_array;
public:
  typename internal_array::iterator begin() { return data.begin(); }
  typename internal_array::iterator end() { return data.end(); }
  typename internal_array::const_iterator cbegin()const { return data.cbegin(); }
  typename internal_array::const_iterator cend()const { return data.cend(); }
private:
  internal_array data;
};


class bounds_checked_int {
public:
  bounds_checked_int():value(0){}
  bounds_checked_int(int value) : value(value) {
    caller_correct_if(value != -(1LL << 31), "bounds_checked_int underflow in constructor");
  }
  bounds_checked_int &operator=(int other) { value = other; return *this; }
  bounds_checked_int operator+()const { return *this; }
  bounds_checked_int operator-()const { return bounds_checked_int(-value); }
  bounds_checked_int operator+(int other)const {
    caller_correct_if((int64_t)value + (int64_t)other < (1LL << 31), "bounds_checked_int overflow in +");
    caller_correct_if((int64_t)value + (int64_t)other > -(1LL << 31), "bounds_checked_int underflow in +");
    return bounds_checked_int(value + other);
  }
  bounds_checked_int& operator+=(int other) {
    return *this = *this + other;
  }
  bounds_checked_int& operator++() {
    *this += 1; return *this;
  }
  bounds_checked_int operator++(int) {
    bounds_checked_int result(value);
    *this += 1;
    return result;
  }
  bounds_checked_int& operator--() {
    *this -= 1; return *this;
  }
  bounds_checked_int operator--(int) {
    bounds_checked_int result(value);
    *this -= 1;
    return result;
  }
  bounds_checked_int operator-(int other)const {
    caller_correct_if((int64_t)value - (int64_t)other < (1LL << 31), "bounds_checked_int overflow in -");
    caller_correct_if((int64_t)value - (int64_t)other > -(1LL << 31), "bounds_checked_int underflow in -");
    return bounds_checked_int(value - other);
  }
  bounds_checked_int& operator-=(int other) {
    return *this = *this - other;
  }
  bounds_checked_int operator*(int other)const {
    caller_correct_if((int64_t)value * (int64_t)other < (1LL << 31), "bounds_checked_int overflow in *");
    caller_correct_if((int64_t)value * (int64_t)other > -(1LL << 31), "bounds_checked_int underflow in *");
    return bounds_checked_int(value * other);
  }
  bounds_checked_int& operator*=(int other) {
    return *this = *this * other;
  }
  bounds_checked_int operator/(int other)const {
    return bounds_checked_int(value / other);
  }
  bounds_checked_int& operator/=(int other) {
    return *this = *this / other;
  }
  operator int()const{ return value; }
private:
  int value;
};

#endif

