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
#include <unordered_map>
#include <vector>
#include <memory>
#include <cmath>
#include <boost/functional/hash.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/taus88.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/random_number_generator.hpp>
#if defined(LASERCAKE_USE_GLIB)
#include <glib.h>
#endif

#include "config.hpp"
#include "data_structures/numbers.hpp"
#include "data_structures/bounds_checked_int.hpp"

template<typename Int> struct lasercake_int {
  typedef typename maybe_bounds_checked_int<Int>::type type;
};

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


// TODO think more about what RNGs go where, and so forth
// (e.g. for increased determinacy, maintained speed, and
// if networked then cryptographic-secureness could be nice too).
// The larger RNG here takes up a few kilobytes (RAM/cache) but produces nicer random numbers.
typedef boost::random::mt19937 large_fast_noncrypto_rng;
typedef boost::random::taus88 small_fast_noncrypto_rng;

template<typename RandomAccessRange, typename RNG> inline typename
RandomAccessRange::const_iterator random_element_of_sequence(RandomAccessRange const& c, RNG& rng) {
  caller_error_if(c.size() == 0, "random_element_of_sequence() on empty sequence");
  const boost::random::uniform_int_distribution<size_t> random_index(0, c.size()-1);
  return c.begin() + random_index(rng);
}
template<typename RandomAccessRange, typename RNG> inline typename
RandomAccessRange::iterator random_element_of_sequence(RandomAccessRange& c, RNG& rng) {
  caller_error_if(c.size() == 0, "random_element_of_sequence() on empty sequence");
  const boost::random::uniform_int_distribution<size_t> random_index(0, c.size()-1);
  return c.begin() + random_index(rng);
}

// If you need to pass around a dimension for use as an index to
// a vector3, you can optionally use these names.
enum {
  X = 0, Y = 1, Z = 2, num_dimensions = 3
};
typedef int which_dimension_type;

template<typename ScalarType> class vector3 {
public:
  ScalarType x, y, z;
  vector3():x(0),y(0),z(0){}
  // implicit conversion from literal 0, so you can write 0 for any zero vector.
  vector3(decltype(nullptr)):x(0),y(0),z(0){}
  vector3(ScalarType x, ScalarType y, ScalarType z):x(x),y(y),z(z){}
  template<typename OtherType> explicit vector3(vector3<OtherType> const& other):
    x(other.x),y(other.y),z(other.z){}

  // implicit conversions to/from array:
  BOOST_FORCEINLINE operator std::array<ScalarType, 3>()const {
    std::array<ScalarType, 3> result = {{ x, y, z }};
    return result;
  }
  BOOST_FORCEINLINE vector3(std::array<ScalarType, 3> const& arr)
    : x(arr[0]), y(arr[1]), z(arr[2]) {}
  
  BOOST_FORCEINLINE ScalarType& operator[](which_dimension_type index) {
    switch(index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
      default: caller_error("Trying to index a vector3 with an out-of-bounds index!");
    }
  }
  BOOST_FORCEINLINE ScalarType operator[](which_dimension_type index)const {
    switch(index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
      default: caller_error("Trying to index a vector3 with an out-of-bounds index!");
    }
  }
  BOOST_FORCEINLINE ScalarType operator()(which_dimension_type index)const { return (*this)[index]; }
  BOOST_FORCEINLINE void set(which_dimension_type index, ScalarType value) { (*this)[index] = value; }

  template<typename OtherType> auto operator+(vector3<OtherType> const& other)const
  -> vector3<decltype(x + other.x)> {
    return vector3<decltype(x + other.x)>(x + other.x, y + other.y, z + other.z);
  }
  template<typename OtherType> vector3& operator+=(vector3<OtherType> const& other) {
    x += other.x; y += other.y; z += other.z; return *this;
  }
  template<typename OtherType> auto operator-(vector3<OtherType> const& other)const
  -> vector3<decltype(x - other.x)> {
    return vector3<decltype(x - other.x)>(x - other.x, y - other.y, z - other.z);
  }
  template<typename OtherType> vector3& operator-=(vector3<OtherType> const& other) {
    x -= other.x; y -= other.y; z -= other.z; return *this;
  }
  template<typename OtherType> auto operator*(OtherType const& other)const
  -> vector3<decltype(x * other)> {
    return vector3<decltype(x * other)>(x * other, y * other, z * other);
  }
  vector3& operator*=(ScalarType other) {
    x *= other; y *= other; z *= other; return *this;
  }
  template<typename OtherType, typename RoundingStrategy>
  friend inline auto divide(vector3 const& v, OtherType const& other, RoundingStrategy strat)
  -> vector3<decltype(std::declval<ScalarType>() / other)>{
    return vector3<decltype(v.x / other)>(
      divide(v.x, other, strat),
      divide(v.y, other, strat),
      divide(v.z, other, strat));
  }
  // Default to rounding towards zero. (TODO: is it wise to have any default here?
  // It's not like we use division much.  But we don't want to use shifting
  // without considering that shifting rounds down towards negative infinity, too.)
  typedef rounding_strategy<round_down, negative_mirrors_positive> default_rounding_strategy;

  // In C++11 integer division rounds towards zero,
  // which is often what we want for vectors; IEEE754 floating point division,
  // by default, rounds to nearest and to even for ties.
  template<typename OtherType> auto operator/(OtherType const& other)const
  -> vector3<decltype(x / other)> {
    return vector3<decltype(x / other)>(x/other, y/other, z/other);
  }
  vector3& operator/=(ScalarType other) {
    x /= other; y /= other; z /= other;
    return *this;
  }
  // Multiplying two vectors is usually a type-error mistake, so
  // you have to say you're doing it in words:
  template<typename OtherType> auto multiply_piecewise_by(vector3<OtherType> const& other)const
  -> vector3<decltype(x * other.x)> {
    return vector3<decltype(x * other.x)>(x * other.x, y * other.y, z * other.z);
  }
  template<typename OtherType, typename RoundingStrategy>
  auto divide_piecewise_by(vector3<OtherType> const& other, RoundingStrategy strat)const
  -> vector3<decltype(x / other.x)> {
    return vector3<decltype(x / other.x)>(
      divide(x, other.x, strat),
      divide(y, other.y, strat),
      divide(z, other.z, strat));
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
  vector3 operator^(ScalarType other)const {
    return vector3(x ^ other, y ^ other, z ^ other);
  }
  vector3 operator|(ScalarType other)const {
    return vector3(x | other, y | other, z | other);
  }
  vector3 operator&(ScalarType other)const {
    return vector3(x & other, y & other, z & other);
  }
  vector3 operator~()const {
    return vector3(~x, ~y, ~z);
  }

  vector3 operator+()const { return *this; } // unary plus
  vector3 operator-()const { // unary minus
    return vector3(-x, -y, -z);
  }

  bool operator==(vector3 const& other)const {return x == other.x && y == other.y && z == other.z; }
  bool operator!=(vector3 const& other)const {return x != other.x || y != other.y || z != other.z; }
  
  // Do not try to use this if either vector has an unsigned ScalarType.
  // It might work in some situations, but why would you ever do that anyway?
  //
  // You are required to specify an output representation type, because of the
  // risk of overflow. Make sure to choose one that can fit the squares of the
  // numbers you're dealing with.
  template<typename OutputRepr, typename OtherType>
  auto dot(vector3<OtherType> const& other)const
  -> decltype(numeric_representation_cast<OutputRepr>(x) * numeric_representation_cast<OutputRepr>(other.x)) {
    return
      numeric_representation_cast<OutputRepr>(x) * numeric_representation_cast<OutputRepr>(other.x) +
      numeric_representation_cast<OutputRepr>(y) * numeric_representation_cast<OutputRepr>(other.y) +
      numeric_representation_cast<OutputRepr>(z) * numeric_representation_cast<OutputRepr>(other.z);
  }

  typedef lasercake_int<int64_t>::type int64_type_to_use_with_dot;
  ScalarType magnitude_within_32_bits()const {
    return ScalarType(i64sqrt(dot<int64_type_to_use_with_dot>(*this)));
  }
  
  // Choose these the way you'd choose dot's output type (see the comment above)
  // we had trouble making these templates, so now they just always use int64_t
  bool magnitude_within_32_bits_is_less_than(ScalarType amount)const {
    return dot<int64_type_to_use_with_dot>(*this) <
          numeric_representation_cast<int64_type_to_use_with_dot>(amount)
        * numeric_representation_cast<int64_type_to_use_with_dot>(amount);
  }
  bool magnitude_within_32_bits_is_greater_than(ScalarType amount)const {
    return dot<int64_type_to_use_with_dot>(*this) >
          numeric_representation_cast<int64_type_to_use_with_dot>(amount)
        * numeric_representation_cast<int64_type_to_use_with_dot>(amount);
  }
  bool operator<(vector3 const& other)const { return (x < other.x) || ((x == other.x) && ((y < other.y) || ((y == other.y) && (z < other.z)))); }

  friend inline std::ostream& operator<<(std::ostream& os, vector3 const& v) {
    return os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
  }
  friend inline size_t hash_value(vector3 const& v) {
      size_t seed = 0;
      boost::hash_combine(seed, v.x);
      boost::hash_combine(seed, v.y);
      boost::hash_combine(seed, v.z);
      return seed;
  }
};

namespace std {
  template<typename ScalarType> struct hash<vector3<ScalarType> > {
    inline size_t operator()(vector3<ScalarType> const& v) const {
      return hash_value(v);
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
  static const char* name() { return "-X"; }
  static const cardinal_direction opposite = xplus;
  static const int dimension = X;
  static const neighboring_tile_differential x_delta = -1;
  static const neighboring_tile_differential y_delta = 0;
  static const neighboring_tile_differential z_delta = 0;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(x_delta, y_delta, z_delta); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { --t.x; }
};
template<> struct cdir_info<yminus> {
  static const char* name() { return "-Y"; }
  static const cardinal_direction opposite = yplus;
  static const int dimension = Y;
  static const neighboring_tile_differential x_delta = 0;
  static const neighboring_tile_differential y_delta = -1;
  static const neighboring_tile_differential z_delta = 0;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(x_delta, y_delta, z_delta); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { --t.y; }
};
template<> struct cdir_info<zminus> {
  static const char* name() { return "-Z"; }
  static const cardinal_direction opposite = zplus;
  static const int dimension = Z;
  static const neighboring_tile_differential x_delta = 0;
  static const neighboring_tile_differential y_delta = 0;
  static const neighboring_tile_differential z_delta = -1;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(x_delta, y_delta, z_delta); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { --t.z; }
};
template<> struct cdir_info<xplus> {
  static const char* name() { return "+X"; }
  static const cardinal_direction opposite = xminus;
  static const int dimension = X;
  static const neighboring_tile_differential x_delta = 1;
  static const neighboring_tile_differential y_delta = 0;
  static const neighboring_tile_differential z_delta = 0;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(x_delta, y_delta, z_delta); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { ++t.x; }
};
template<> struct cdir_info<yplus> {
  static const char* name() { return "+Y"; }
  static const cardinal_direction opposite = yminus;
  static const int dimension = Y;
  static const neighboring_tile_differential x_delta = 0;
  static const neighboring_tile_differential y_delta = 1;
  static const neighboring_tile_differential z_delta = 0;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(x_delta, y_delta, z_delta); }
  template<class ThingWithCoordinates> static void add_to(ThingWithCoordinates& t) { ++t.y; }
};
template<> struct cdir_info<zplus> {
  static const char* name() { return "+Z"; }
  static const cardinal_direction opposite = zminus;
  static const int dimension = Z;
  static const neighboring_tile_differential x_delta = 0;
  static const neighboring_tile_differential y_delta = 0;
  static const neighboring_tile_differential z_delta = 1;
  static vector3<neighboring_tile_differential> as_vector() { return vector3<neighboring_tile_differential>(x_delta, y_delta, z_delta); }
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
#define which_dimension_is_cardinal_direction(dir) ((dir) % 3)
#define opposite_cardinal_direction(dir) (cardinal_direction(((dir) + 3) % 6))
#define is_a_positive_directional_cardinal_direction(dir) ((dir) >= 3)

inline bool cardinal_directions_are_perpendicular(cardinal_direction d1, cardinal_direction d2) {
  return (which_dimension_is_cardinal_direction(d1) != which_dimension_is_cardinal_direction(d2));
}

inline cardinal_direction cardinal_direction_of_dimension_and_positiveness(which_dimension_type dim, bool positive) {
  return constexpr_require_and_return(dim < 3, "can't pass a cardinal_direction as a dimension here",
                                      dim + positive*3);
}

template<typename ScalarType> inline vector3<ScalarType> project_onto_cardinal_direction(vector3<ScalarType> src, cardinal_direction dir) {
  vector3<ScalarType> result(0,0,0);
  result[which_dimension_is_cardinal_direction(dir)] = src[which_dimension_is_cardinal_direction(dir)];
  return result;
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
template<cardinal_direction Dir, typename T>
void ostream_direction_of_value_for_each_cardinal_direction(std::ostream& os,
      value_for_each_cardinal_direction<T> const& vs) {
  os << (Dir == 0 ? "" : ", ") << cdir_info<Dir>::name() << '=' << vs.template get<Dir>();
}
template<typename T>
inline std::ostream& operator<<(std::ostream& os, value_for_each_cardinal_direction<T> const& vs) {
  os << '[';
  ostream_direction_of_value_for_each_cardinal_direction<0>(vs);
  ostream_direction_of_value_for_each_cardinal_direction<1>(vs);
  ostream_direction_of_value_for_each_cardinal_direction<2>(vs);
  ostream_direction_of_value_for_each_cardinal_direction<3>(vs);
  ostream_direction_of_value_for_each_cardinal_direction<4>(vs);
  ostream_direction_of_value_for_each_cardinal_direction<5>(vs);
  return os << ']';
}

typedef int8_t octant_number;
#define LASERCAKE_MAKE_OCTANT(xpositive, ypositive, zpositive) \
  (octant_number((bool((xpositive))<<2) + (bool((ypositive))<<1) + (bool((zpositive)))))
#define LASERCAKE_OCTANT_X_POSITIVE(octant) (bool((octant) & (1<<2)))
#define LASERCAKE_OCTANT_Y_POSITIVE(octant) (bool((octant) & (1<<1)))
#define LASERCAKE_OCTANT_Z_POSITIVE(octant) (bool((octant) & (1<<0)))
// When a component of the vector is 0, vector_octant() returns an
// arbitrary result for that dimension.
template<typename ScalarType>
inline octant_number vector_octant(vector3<ScalarType> const& v) {
  return LASERCAKE_MAKE_OCTANT(v[X] >= 0, v[Y] >= 0, v[Z] >= 0);
}



// Use a define so that it can be a constant expression on all compilers.
#define LASERCAKE_MAKE_UINT64_MASK_FROM_UINT8(b) \
  ( (uint64_t(uint8_t((b)))<<(8*7)) | (uint64_t(uint8_t((b)))<<(8*6)) \
  | (uint64_t(uint8_t((b)))<<(8*5)) | (uint64_t(uint8_t((b)))<<(8*4)) \
  | (uint64_t(uint8_t((b)))<<(8*3)) | (uint64_t(uint8_t((b)))<<(8*2)) \
  | (uint64_t(uint8_t((b)))<<(8*1)) | (uint64_t(uint8_t((b))))        )
#define LASERCAKE_MAKE_UINT32_MASK_FROM_UINT8(b) \
  | (uint32_t(uint8_t((b)))<<(8*3)) | (uint32_t(uint8_t((b)))<<(8*2)) \
  | (uint32_t(uint8_t((b)))<<(8*1)) | (uint32_t(uint8_t((b))))        )


#if defined(LASERCAKE_USE_GLIB)
template<typename T>
class g_slice_allocator {
public:
  typedef T value_type;
  typedef value_type* pointer;
  typedef const value_type* const_pointer;
  typedef value_type& reference;
  typedef const value_type& const_reference;
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;

  template<typename U>
  struct rebind {
    typedef g_slice_allocator<U> other;
  };

  g_slice_allocator() {}
  //~g_slice_allocator() {}
  //explicit g_slice_allocator(g_slice_allocator const&) {}
  template<typename U>
  explicit g_slice_allocator(g_slice_allocator<U> const&) {}

  pointer address(reference ref) { return std::addressof(ref); }
  const_pointer address(const_reference ref) { return std::addressof(ref); }

  pointer allocate(size_type count, void* = 0) {
    return static_cast<pointer>(g_slice_alloc(count * sizeof(T)));
  }
  void deallocate(pointer ptr, size_type count) {
    g_slice_free1(count * sizeof(T), ptr);
  }

  size_type max_size() const {
    return std::numeric_limits<size_type>::max() / sizeof(T);
  }

  void construct(pointer p, const T& t) { new(p) T(t); }
  void destroy(pointer p) { p->~T(); }

  bool operator==(g_slice_allocator const&) { return true; }
  bool operator!=(g_slice_allocator const&) { return false; }
};
template<typename T>
struct lasercake_nice_allocator {
  typedef g_slice_allocator<T> type;
};
template<typename T, typename... Args>
inline T* lasercake_nice_new(Args... args) {
  T* ptr = static_cast<T*>(g_slice_alloc(sizeof(T)));
  new (ptr) T(args...);
  return ptr;
}
template<typename T>
struct lasercake_nice_deleter {
  void operator()(T* ptr)const { ptr->~T(); g_slice_free1(sizeof(T), ptr); }
};
#else
template<typename T>
struct lasercake_nice_allocator {
  typedef std::allocator<T> type;
};
template<typename T, typename... Args>
inline T* lasercake_nice_new(Args... args) {
  return new T(args...);
}
template<typename T>
struct lasercake_nice_deleter {
  void operator()(T* ptr)const { delete ptr; }
};
#endif

template<typename T>
struct lasercake_set {
  typedef std::unordered_set<T, std::hash<T>, std::equal_to<T>, typename lasercake_nice_allocator<T>::type > type;
};
template<typename K, typename V>
struct lasercake_map {
  typedef std::unordered_map<K, V, std::hash<K>, std::equal_to<K>, typename lasercake_nice_allocator< std::pair<const K, V> >::type > type;
};
template<typename T>
struct lasercake_vector {
  typedef std::vector<T, typename lasercake_nice_allocator<T>::type > type;
};

#endif

