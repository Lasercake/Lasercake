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
#include <unordered_set>
#include <vector>
#include <cmath>
#include <inttypes.h>
#include <boost/functional/hash.hpp>
#include <boost/throw_exception.hpp>
#include <stdexcept>

// Some asserts take too much runtime to turn on by default.
// So write them "assert_if_ASSERT_EVERYTHING(x);"
// Build with -DASSERT_EVERYTHING to turn them on.
#ifdef ASSERT_EVERYTHING
  // By not using parentheses (e.g. not "#define assert_if_ASSERT_EVERYTHING(x) assert(x)"),
  // we prevent any additional unintended macro-expansion,
  // which in the case of assert would affect the string
  // that is printed when the assert fails.
  #define assert_if_ASSERT_EVERYTHING assert
  const bool assert_everything = true;
#else
  // Make sure the code takes no runtime (the compiler will optimize it out)
  // but that it still compiles to a boolean expression (so that turning on
  // ASSERT_EVERYTHING is sure to compile even if we didn't test with it on
  // recently).
  #define assert_if_ASSERT_EVERYTHING(x) ((true) ? (void)0 : ((x) ? (void)0 : (void)0))
  const bool assert_everything = false;
#endif

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

#if !LASERCAKE_NO_THREADS
const bool LASERCAKE_NO_THREADS = false;
#endif

// (not enabled unless you enable it) #define USE_BOUNDS_CHECKED_INTS 1


// It's not polite for library functions to assert() because the library's users
// misused a correct library; use these for that case.
inline ATTRIBUTE_NORETURN void caller_error(const char* error) {
  // If exceptions prove worse for debugging than asserts/segfaults,
  // feel free to comment this out and use asserts/segfaults/breakpoints.
  boost::throw_exception(std::logic_error(error));
}
// You must provide an explanatory string so that the user of the library
// will know what *they* did wrong, and not have to interpret an assert() expression
// to find out.
inline void caller_error_if(bool cond, const char* error) {
  if(cond) {
    caller_error(error);
  }
}
inline void caller_correct_if(bool cond, const char* error) {
  if(!cond) {
    caller_error(error);
  }
}


// returns the signum (-1, 0, or 1)
template <typename Number>
inline Number sign(Number n) {
  return (n > 0) - (n < 0);
}

template <typename Number>
inline bool is_negative(Number n) {
  // this is (very slightly) the fastest comparison-with-zero among < 0, <= 0, > 0, >= 0:
  // in signed two's complement, it's the value of the highest bit:
  // which leads to one less x86 instruction (small code = more fits in cache).
  // (Once inlined and optimized in context, the chosen comparison might make no difference.)
  return (n < 0);
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

#include "bounds_checked_int.hpp"

#if USE_BOUNDS_CHECKED_INTS
template<typename Int> struct lasercake_int { typedef bounds_checked_int<Int> type; };
#else
template<typename Int> struct lasercake_int { typedef Int type; };
#endif

template<typename ScalarType1, typename ScalarType2>
auto divide_rounding_towards_zero(ScalarType1 dividend, ScalarType2 divisor)
  -> decltype(dividend/divisor) /*C++11 syntax for return type that lets it refer to argument names*/
{
  caller_correct_if(divisor != 0, "divisor must be nonzero");
  using namespace std;
  const auto abs_result = abs(dividend) / abs(divisor);
  if (is_negative(dividend) == is_negative(divisor)) return abs_result;
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
  if(radicand & ((((1ULL << 1) - 1) << 1) << shift))lower_bound = (uint32_t)((lower_bound * 6074000999ULL) >> 32); //approximate the square root of 2
  
  uint64_t upper_bound = (lower_bound < (uint32_t(1) << 31)) ? (lower_bound << 1) : (1ULL << 32);
  //lower_bound is guaranteed to be less than or equal to the answer
  //upper_bound is guaranteed to be greater than the answer
  
  while(lower_bound < upper_bound - 1)
  {
    const uint32_t mid = uint32_t((upper_bound + lower_bound) >> 1);
    if(uint64_t(mid) * mid > radicand)upper_bound = mid;
    else lower_bound = mid;
  }
  
  return lower_bound;
}

// If you need to pass around a dimension for use as an index to
// a vector3, you can optionally use these names.
enum {
  X = 0, Y = 1, Z = 2
};

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

  typedef lasercake_int<int64_t>::type int64_type_to_use_with_dot;
  ScalarType magnitude_within_32_bits()const { return ScalarType(get_un_bounds_checked_int(i64sqrt(dot<int64_type_to_use_with_dot>(*this)))); }
  
  // Choose these the way you'd choose dot's output type (see the comment above)
  // we had trouble making these templates, so now they just always use int64_t
  bool magnitude_within_32_bits_is_less_than(ScalarType amount)const {
    return dot<int64_type_to_use_with_dot>(*this) < (int64_type_to_use_with_dot)amount * (int64_type_to_use_with_dot)amount;
  }
  bool magnitude_within_32_bits_is_greater_than(ScalarType amount)const {
    return dot<int64_type_to_use_with_dot>(*this) > (int64_type_to_use_with_dot)amount * (int64_type_to_use_with_dot)amount;
  }
  bool operator<(vector3 const& other)const { return (x < other.x) || ((x == other.x) && ((y < other.y) || ((y == other.y) && (z < other.z)))); }
};

// Make sure this is declared so that overload resolution on vector3 ostream <<
// will choose this for the component types (rather than choosing bool(!)).
template<typename Int, Int Min, Int Max>
std::ostream& operator<<(std::ostream& os, bounds_checked_int<Int,Min,Max> i);

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
enum {
  first_cardinal_direction = 0,
  xminus = 0,
  yminus,
  zminus,
  xplus,
  yplus,
  zplus,
  num_cardinal_directions
};
typedef int8_t cardinal_direction;


template <cardinal_direction Dir> struct cdir_info;
template<> struct cdir_info<xminus> {
  static const cardinal_direction opposite = xplus;
  static const int dimension = X;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(-1, 0, 0); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { --t.x; }
};
template<> struct cdir_info<yminus> {
  static const cardinal_direction opposite = yplus;
  static const int dimension = Y;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(0, -1, 0); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { --t.y; }
};
template<> struct cdir_info<zminus> {
  static const cardinal_direction opposite = zplus;
  static const int dimension = Z;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(0, 0, -1); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { --t.z; }
};
template<> struct cdir_info<xplus> {
  static const cardinal_direction opposite = xminus;
  static const int dimension = X;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(1, 0, 0); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { ++t.x; }
};
template<> struct cdir_info<yplus> {
  static const cardinal_direction opposite = yminus;
  static const int dimension = Y;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(0, 1, 0); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { ++t.y; }
};
template<> struct cdir_info<zplus> {
  static const cardinal_direction opposite = zminus;
  static const int dimension = Z;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(0, 0, 1); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { ++t.z; }
};

// This ordering must match the dir ordering above.
// Sadly C++ isn't supporting C99's = { [cdiridx_xminus] = cdir_xminus, [...] };.
const vector3<neighboring_tile_differential> cardinal_direction_vectors[num_cardinal_directions] = { cdir_info<xminus>::as_vector(), cdir_info<yminus>::as_vector(), cdir_info<zminus>::as_vector(), cdir_info<xplus>::as_vector(), cdir_info<yplus>::as_vector(), cdir_info<zplus>::as_vector() };

template<cardinal_direction Dir, typename ThingWithCoordinates>
ThingWithCoordinates next_in_direction(ThingWithCoordinates const& t) {
  ThingWithCoordinates result = t;
  cdir_info<Dir>::add_to(result);
  return result;
}

// These are macros (not inline functions) so they can be used in constant expressions:
// 'constexpr' support in compilers isn't quite functional yet.
#define which_dimension_is_cardinal_direction(dir) (dir % 3)
#define opposite_cardinal_direction(dir) (cardinal_direction((dir + 3) % 6))
#define is_a_positive_directional_cardinal_direction(dir) (dir >= 3)

template<typename ScalarType> inline vector3<ScalarType> project_onto_cardinal_direction(vector3<ScalarType> src, cardinal_direction dir) {
  vector3<ScalarType> result(0,0,0);
  result[which_dimension_is_cardinal_direction(dir)] = src[which_dimension_is_cardinal_direction(dir)];
  return result;
}

inline bool cardinal_directions_are_perpendicular(cardinal_direction d1, cardinal_direction d2) {
  return (which_dimension_is_cardinal_direction(d1) != which_dimension_is_cardinal_direction(d2));
}


template<typename ValueType> class value_for_each_cardinal_direction {
public:
  explicit value_for_each_cardinal_direction(ValueType const& iv/*initial_value*/) : data({{iv,iv,iv,iv,iv,iv}}) { static_assert(num_cardinal_directions == 6, "fix {{iv,iv,...}} to have the right number"); }
  template<cardinal_direction Dir> ValueType      & get()      { return data[Dir]; }
  template<cardinal_direction Dir> ValueType const& get()const { return data[Dir]; }
  ValueType      & operator[](cardinal_direction const& dir)      { return data[dir]; }
  ValueType const& operator[](cardinal_direction const& dir)const { return data[dir]; }
private:
  typedef std::array<ValueType, num_cardinal_directions> internal_array;
public:
  typedef ValueType value_type;
  typedef ValueType& reference;
  typedef ValueType const& const_reference;
  typedef ValueType* pointer;
  typedef ValueType const* const_pointer;
  typedef typename internal_array::iterator iterator;
  typedef typename internal_array::const_iterator const_iterator;
  typedef typename internal_array::size_type size_type;
  typedef typename internal_array::difference_type difference_type;
  iterator begin() { return data.begin(); }
  iterator end() { return data.end(); }
  const_iterator cbegin()const { return data.cbegin(); }
  const_iterator cend()const { return data.cend(); }
private:
  internal_array data;
};

template<typename IntType> struct non_normalized_rational {
  IntType numerator;
  IntType denominator;
  non_normalized_rational(IntType n, IntType d):numerator(n),denominator(d){ caller_error_if(denominator == 0, "Constructing a rational with denominator zero..."); }
  non_normalized_rational(IntType n):numerator(n),denominator(1){}
  non_normalized_rational():numerator(0),denominator(1){}
  bool operator< (non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator <  o.numerator*denominator;
    else
      return numerator*o.denominator >  o.numerator*denominator;
  }
  bool operator> (non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator >  o.numerator*denominator;
    else
      return numerator*o.denominator <  o.numerator*denominator;
  }
  bool operator<=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator <= o.numerator*denominator;
    else
      return numerator*o.denominator >= o.numerator*denominator;
  }
  bool operator>=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    if (is_negative(denominator) == is_negative(o.denominator))
      return numerator*o.denominator >= o.numerator*denominator;
    else
      return numerator*o.denominator <= o.numerator*denominator;
  }
  bool operator==(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    return numerator*o.denominator == o.numerator*denominator; }
  bool operator!=(non_normalized_rational const& o)const {
    assert(denominator != 0 && o.denominator != 0);
    return numerator*o.denominator != o.numerator*denominator; }
};
template<typename IntType> inline std::ostream& operator<<(std::ostream& os, non_normalized_rational<IntType>const& r) {
  return os << r.numerator << '/' << r.denominator;
}

template<typename Stuff> struct literally_random_access_removable_stuff {
public:
  void insert(Stuff const& stuff) {
    if ((stuffs_set.insert(stuff)).second) {
      stuffs_superset_vector.push_back(stuff);
    }
  }
  bool erase(Stuff const& which) {
    if (stuffs_set.erase(which)) {
      if (stuffs_set.size() * 2 <= stuffs_superset_vector.size()) {
        purge_nonexistent_stuffs();
      }
      return true;
    }
    return false;
  }
  Stuff const& get_random()const {
    caller_error_if(stuffs_set.empty(), "Trying to get a random element of an empty literally_random_access_removable_stuff");
    size_t idx;
    do {
      idx = (size_t)(rand()%(stuffs_superset_vector.size()));
    } while (stuffs_set.find(stuffs_superset_vector[idx]) == stuffs_set.end());
    return stuffs_superset_vector[idx];
  }
  bool empty()const { return stuffs_set.empty(); }
  std::unordered_set<Stuff> const& as_unordered_set()const { return stuffs_set; }
private:
  std::vector<Stuff> stuffs_superset_vector;
  std::unordered_set<Stuff> stuffs_set;
  void purge_nonexistent_stuffs() {
    size_t next_insert_idx = 0;
    for (Stuff const& st : stuffs_set) {
      stuffs_superset_vector[next_insert_idx] = st;
      ++next_insert_idx;
    }
    stuffs_superset_vector.erase(stuffs_superset_vector.begin() + next_insert_idx, stuffs_superset_vector.end());
  }
};

#endif

