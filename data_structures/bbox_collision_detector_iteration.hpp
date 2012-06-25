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

#ifndef LASERCAKE_BBOX_COLLISION_DETECTOR_ITERATION_HPP__
#define LASERCAKE_BBOX_COLLISION_DETECTOR_ITERATION_HPP__

// The documentation for these interfaces is in bbox_collision_detector.hpp.

#include <vector>
#include <queue>
#include <boost/variant.hpp>
#include <boost/compressed_pair.hpp>
#include <boost/range/iterator_range.hpp>

#include "bbox_collision_detector.hpp"
#include "borrowed_bitset.hpp"
#include "misc_structures.hpp"


namespace collision_detector {
namespace impl {

// bbox_collision_detector ztree structure we'll need:

template<num_bits_type CoordinateBits>
struct coordinate_bit_math {
  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;

  static Coordinate safe_left_shift_one(num_bits_type shift) {
    if (shift >= CoordinateBits) return 0;
    return Coordinate(1) << shift;
  }

  static Coordinate this_many_low_bits(num_bits_type num_bits) {
    return safe_left_shift_one(num_bits) - 1;
  }
};


template<typename Coordinate, size_t/*template argument deduction requires this type here*/ NumDimensions>
inline Coordinate max_in_array_of_unsigned(std::array<Coordinate, NumDimensions> const& arr) {
  if(NumDimensions == 0) {
    return 0;
  }
  else {
    Coordinate max_val = arr[0];
    for (size_t i = 1; i < NumDimensions; ++i) {
      if (arr[i] > max_val) max_val = arr[i];
    }
    return max_val;
  }
}

// bbox_collision_detector uses "z-ordering", named such because "z" is a visual for
// the zigzag the ordering creates when drawn.  See
//     https://en.wikipedia.org/wiki/Z-order_curve
//
// A zbox is:
//
// If you interleave the coordinates' bits in the z-ordering way, it is a contiguous
// range of interleaved bits from, for example, binary VVVVV000 through VVVVV111,
// for some VVVVV.  Consider the zbox to consist of VVVVV and the number of low bits
// that vary.
// (This example could be a two-dimensional bbox_collision_detector<foo, 4, 2>,
// if four-bit integers existed -- imagine lots more bits for a real zbox.)
//
// Equivalently, a zbox is a square/cube/etc. on a maximally-bit-aligned grid of
// square/cubes of its size, or a rectangle/etc. that consists of 2/etc. of these
// square/cubes adjacent (in the case that the number of low bits in the first
// explanation is not a multiple of NumDimensions).
template<num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class zbox {
private:
  typedef typename coordinate_type_from_bits<CoordinateBits>::type Coordinate;
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef coordinate_bit_math<CoordinateBits> math_;
  typedef std::array<Coordinate, NumDimensions> coordinate_array;
  // We ensure that every bit except the ones specifically supposed to be on is off.
  // (Specifically, the "low bits" are zero.)
  coordinate_array coords_;

  //small_num_bits_type can represent at least [0, CoordinateBits*NumDimensions] inclusive.
  //smaller_num_bits_type can represent at least [0, CoordinateBits] inclusive.
  typedef typename boost::uint_t<static_num_bits_in_integer_that_are_not_leading_zeroes<CoordinateBits*NumDimensions>::value>::least small_num_bits_type;
  typedef typename boost::uint_t<static_num_bits_in_integer_that_are_not_leading_zeroes<CoordinateBits>::value>::least smaller_num_bits_type;
  small_num_bits_type num_low_bits_;
  std::array<smaller_num_bits_type, NumDimensions> dim_num_low_bits_;

public:
  typedef std::ostream hack_to_make_bbox_collision_detector_zbox_ostreamable;

  zbox():num_low_bits_(CoordinateBits * NumDimensions){}

  // Named constructor idiom
  static zbox smallest_joint_parent(zbox zb1, zbox zb2) {
    zbox new_box;
    num_bits_type dim_low_bits_heuristic[NumDimensions];
    num_bits_type largest_dim_low_bits = 0;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      const Coordinate uncommon_bits =
          (zb1.coords_[i] | math_::this_many_low_bits(zb2.num_low_bits_by_dimension(i)))
        ^ (zb2.coords_[i] & ~math_::this_many_low_bits(zb1.num_low_bits_by_dimension(i)));
      const num_bits_type this_dimension_low_bits = num_bits_in_integer_that_are_not_leading_zeroes(uncommon_bits);
      dim_low_bits_heuristic[i] = this_dimension_low_bits;
      if(largest_dim_low_bits < this_dimension_low_bits) largest_dim_low_bits = this_dimension_low_bits;
    }
    const num_bits_type low_bits_minor = largest_dim_low_bits - 1;
    const num_bits_type low_bits_major = largest_dim_low_bits;
    num_bits_type lim_low_bits = low_bits_minor;
    num_bits_type dim_low_bits[NumDimensions];
    num_bits_type total_low_bits = 0;
    for (num_coordinates_type i = NumDimensions - 1; i >= 0; --i) {
      if(dim_low_bits_heuristic[i] ==/*a.k.a.>=*/ low_bits_major) lim_low_bits = low_bits_major;
      dim_low_bits[i] = lim_low_bits;
      new_box.dim_num_low_bits_[i] = lim_low_bits;
      total_low_bits += lim_low_bits;
    }
    new_box.num_low_bits_ = total_low_bits;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      assert_if_ASSERT_EVERYTHING(
            (zb1.coords_[i] & ~math_::this_many_low_bits(dim_low_bits[i]))
        == (zb2.coords_[i] & ~math_::this_many_low_bits(dim_low_bits[i]))
      );
      new_box.coords_[i] = zb1.coords_[i] & ~math_::this_many_low_bits(dim_low_bits[i]);
    }
    return new_box;
  }

  static zbox box_from_coords(coordinate_array const& coords, num_bits_type num_low_bits) {
    assert(num_low_bits >= 0 && num_low_bits <= CoordinateBits*NumDimensions);
    zbox result;
    result.num_low_bits_ = num_low_bits;
    const num_bits_type base_num_low_bits = num_low_bits / NumDimensions;
    const num_bits_type tweak_num_low_bits = num_low_bits % NumDimensions;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      result.dim_num_low_bits_[i] = base_num_low_bits + (i < tweak_num_low_bits);
      result.coords_[i] = coords[i] & ~math_::this_many_low_bits(result.num_low_bits_by_dimension(i));
    }
    return result;
  }

  bool subsumes(zbox const& other)const {
    if (other.num_low_bits_ > num_low_bits_) return false;
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      if (coords_[i] != (other.coords_[i] & ~math_::this_many_low_bits(num_low_bits_by_dimension(i)))) return false;
    }
    return true;
  }
  bool overlaps(zbox const& other)const {
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      if ( (coords_[i] & ~math_::this_many_low_bits(other.num_low_bits_by_dimension(i)))
        != (other.coords_[i] & ~math_::this_many_low_bits(num_low_bits_by_dimension(i)))) {
          return false;
      }
    }
    return true;
  }
  bool overlaps(bounding_box const& bbox)const {
    for (num_coordinates_type i = 0; i != NumDimensions; ++i) {
      const Coordinate this_size_minus_one_i = math_::this_many_low_bits(num_low_bits_by_dimension(i));
      if (bbox.size_minus_one(i) <  coords_[i] - bbox.min(i)
       && this_size_minus_one_i  < bbox.min(i) -  coords_[i]) return false;
    }
    return true;
  }
  bool get_bit(num_bits_type bit)const {
    return coords_[bit % NumDimensions] & math_::safe_left_shift_one(bit / NumDimensions);
  }
  num_bits_type num_low_bits_by_dimension(num_coordinates_type dim)const {
    return dim_num_low_bits_[dim];
  }
  bounding_box get_bbox()const {
    coordinate_array size_minus_one;
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      size_minus_one[i] = math_::this_many_low_bits(num_low_bits_by_dimension(i));
    }
    return bounding_box::min_and_size_minus_one(coords_, size_minus_one);
  }
  num_bits_type num_low_bits()const {
    return num_low_bits_;
  }
  bool operator==(zbox const& other)const {
    return num_low_bits_ == other.num_low_bits_ && coords_ == other.coords_;
  }
  bool operator!=(zbox const& other)const {
    return !(*this == other);
  }
  friend inline std::ostream& operator<<(std::ostream& os, zbox const& zb) {
    return os << "0x" << std::hex << zb.get_bbox() << std::dec;
  }
};

// The bbox_collision_detector has a "z-tree".  This
// is a binary tree.  Keys in the tree are zboxes (see above).
// Values are ObjectIdentifiers; each zbox may have any number
// of ObjectIdentifiers.  This code is happy for objects to overlap
// each other, and besides, even non-overlapping objects often
// have a minimal containing zbox in common.
//
// Because of the definition of zboxes, either they are
// A: the same, and thus the same ztree_node
// B: one is smaller and fully within the node, and it is a
//          descendant of the other
// C: they don't overlap at all, and in the ztree neither is a
//          descendant of the other.
// (In particular, zboxes can't partially overlap each other.)
//
// An object may need to be in up to (2**NumDimensions) zboxes
// so that the area covered by its zboxes is only a constant
// factor larger than the object's regular (non-z-order) bounding
// box.  Consider an object that's a box of width 2 or 3 with
// x min-coordinate 10111111 and max 11000001 (binary).  The minimal
// common prefix there is just a single bit; a single bit means
// a huge box.  Conceptually, dimensions' bits are interleaved
// before looking for a common prefix; any dimension has the
// potential to differ in a high bit and disrupt the common prefix.
// Splitting the object across two zboxes, for each dimension,
// is sufficient to avoid this explosion.
//
// Specifically, a ztree is a Patricia trie on z-ordered
// bits (zboxes), where the bits are seen as a sequence with the
// highest-order bits first and the sequence-length equal to the
// number of high bits specified by a given key/ztree_node/zbox.
// The ztree_node happens to contain (unlike typical tries) the
// entire key that it represents, because the key is small and
// constant-sized and it's generally easier to do so.
//
// What goes into child0 vs. child1?  If trying to insert, say,
// the zbox B = 10100??? (binary, 5 high bits, 3 low bits) into
// A = 10??????, B goes at or below A's child1 because B's next bit
// after A's bits is 1.  (Current: 10, next: 101).
//
// If there would be a node with zero ObjectIdentifiers and
// only one child, then that child node goes there directly
// instead of that trivial node.  If the tree, to be correct,
// needs nodes with two children and zero ObjectIdentifiers,
// then it will have them.
//} /* temporarily end anonymous namespace */
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
struct ztree_node {
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef impl::zbox<CoordinateBits, NumDimensions> zbox;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  typedef impl::object_metadata<CoordinateBits, NumDimensions> object_metadata;
  typedef std::pair<const ObjectIdentifier, object_metadata> id_and_bbox_type;
  typedef id_and_bbox_type* id_and_bbox_ptr;
  typedef std::unordered_set<id_and_bbox_ptr> objects_here_type;

  const zbox here;
  ztree_node_ptr child0;
  ztree_node_ptr child1;

  objects_here_type objects_here;

  ztree_node(zbox box):here(box),child0(nullptr),child1(nullptr){}
  ztree_node(ztree_node const& other) :
    here(other.here),
    child0(other.child0 ? new ztree_node(*other.child0) : nullptr),
    child1(other.child1 ? new ztree_node(*other.child1) : nullptr),
    objects_here(other.objects_here)
    {}
  // operator= could exist if we wanted to make zbox non-const.
  // ztree_node& operator=(ztree_node const& other) = delete;
};




// iterator:

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
struct ztree_get {
  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef impl::ztree_node<ObjectIdentifier, CoordinateBits, NumDimensions> ztree_node;
  // These can be defined out-of-line (at cost to inlining) in order
  // to not include ztree/zbox types in this header.
  static bounding_box get_node_bbox(ztree_node* node) { return node->here.get_bbox(); }
  static ztree_node* get_node_child0(ztree_node* node) { return node->child0.get(); }
  static ztree_node* get_node_child1(ztree_node* node) { return node->child1.get(); }
  static typename ztree_node::objects_here_type& get_node_objects_here(ztree_node* node) { return node->objects_here; }
};

template<typename OrderingFunctor, typename ComparedType>
struct reverse_first_ordering : private OrderingFunctor {
  typedef bool result_type; typedef ComparedType first_argument_type; typedef ComparedType second_argument_type;
  reverse_first_ordering() : OrderingFunctor() {}
  reverse_first_ordering(OrderingFunctor const& o) : OrderingFunctor(o) {}

  bool operator()(ComparedType const& a, ComparedType const& b) {
    return static_cast<OrderingFunctor&>(*this)(b.first(), a.first());
  }
};

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions,
          typename GetCost,
          typename CostOrdering = std::less<typename GetCost::cost_type> >
class iterator {
private:
  typedef typename GetCost::cost_type CostType;

  typedef collision_detector::bounding_box<CoordinateBits, NumDimensions> bounding_box;
  typedef impl::object_metadata<CoordinateBits, NumDimensions> object_metadata;
  typedef std::pair<const ObjectIdentifier, object_metadata> id_and_bbox_type;
  typedef id_and_bbox_type* id_and_bbox_ptr;
  typedef impl::ztree_node<ObjectIdentifier, CoordinateBits, NumDimensions> ztree_node;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  typedef collision_detector::bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions> bbox_collision_detector;

  typedef impl::ztree_get<ObjectIdentifier, CoordinateBits, NumDimensions> ztree_get;
  typedef boost::variant<ztree_node*, id_and_bbox_ptr> node_variant_type;
  typedef boost::compressed_pair<CostType, node_variant_type> queue_value_type_;
  // swap greater/less because pq sorts by greatest and we want least by default (as sorting normally is)
  typedef std::priority_queue<queue_value_type_, std::vector<queue_value_type_>, reverse_first_ordering<CostOrdering, queue_value_type_> > queue_type_;

  struct iteree {
    ObjectIdentifier const& object;
    bounding_box const& bbox;
    CostType cost;
    iteree(CostType const& cost, id_and_bbox_ptr v)
      : object(v->first), bbox(v->second.bbox), cost(cost) {}
  };
public:
  typedef iteree value_type;
  typedef value_type reference;
  typedef value_as_ptr<value_type, reference> pointer;
  typedef ptrdiff_t difference_type; //Ha. Ha.
  typedef std::input_iterator_tag iterator_category;

private:

  static value_type to_value_reference(id_and_bbox_ptr id_and_bbox) {
    return value_type(id_and_bbox->first, id_and_bbox->second.bbox);
  }


  struct contents_ : private GetCost {
    contents_(bbox_collision_detector const& detector, GetCost const& getcost, CostOrdering const& costordering)
    : GetCost(getcost), queue_(costordering), seen_(detector.size())
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
    , detector_(&detector)
    , revision_count_(detector.revision_count_)
#endif
    {}

    contents_(contents_&&) = default;
    queue_type_ queue_;
    borrowed_bitset seen_;
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
    bbox_collision_detector const* detector_;
    size_t revision_count_;
#endif

    GetCost& get_get_cost() { return *this; }

    template<typename VariantMember>
    inline void push_cost(CostType const& cost, VariantMember v) {
      queue_.push(queue_value_type_(cost, v));
    }
    template<typename IndirectCostType, typename VariantMember>
    inline typename boost::disable_if<boost::is_convertible<IndirectCostType, CostType> >::type
    push_cost(IndirectCostType const& maybe_cost, VariantMember v) {
      if(maybe_cost) {
        queue_.push(queue_value_type_(*maybe_cost, v));
      }
    }

    void add_child(ztree_node* child) {
      if(child) {
        push_cost(get_get_cost().min_cost(ztree_get::get_node_bbox(child)), child);
      }
    }
    void add_child(id_and_bbox_ptr obj) {
      if(!seen_.test(obj->second.numeric_id)) {
        seen_.set(obj->second.numeric_id);
        push_cost(get_get_cost().cost(obj->first, obj->second.bbox), obj);
      }
    }

    void add_children_of(ztree_node* node) {
      add_child(ztree_get::get_node_child0(node));
      add_child(ztree_get::get_node_child1(node));
      for(id_and_bbox_ptr obj : ztree_get::get_node_objects_here(node)) {
        add_child(obj);
      }
    }
  };

  //dynamic allocation, pooh.
  boost::shared_ptr<contents_> c_;

  void advance_to_a_returnable_() {
    if(c_) {
#ifdef BBOX_COLLISION_DETECTOR_DEBUG
      caller_correct_if(c_->revision_count_ == c_->detector_->revision_count_,
                        "Error: using a bbox_collision_detector iterator "
                        "after the container has changed!");
#endif
      while(true) {
        if(c_->queue_.empty()) {
          c_.reset();
          break;
        }
        if(ztree_node*const* node_top_ptr = boost::get<ztree_node*>(&c_->queue_.top().second())) {
          ztree_node* node_top = *node_top_ptr;
          c_->queue_.pop();
          c_->add_children_of(node_top);
        }
        else {
          break;
        }
      }
    }
  }

  struct unspecified_bool_{int member; private:unspecified_bool_();};
  typedef int unspecified_bool_::* unspecified_bool_type;

public:

  iterator() : c_() {}
  explicit iterator(bbox_collision_detector const& detector,
                    GetCost const& getcost = GetCost(),
                    CostOrdering const& costordering = CostOrdering()) : c_(new contents_(detector, getcost, costordering)) {}
  template<typename T>
  explicit iterator(bbox_collision_detector const& detector,
                    T const& initial,
                    GetCost const& getcost = GetCost(),
                    CostOrdering const& costordering = CostOrdering())
    : c_(new contents_(detector, getcost, costordering)) { push(initial); }

  template<typename T>
  void push(T const& v) { c_->add_child(v); advance_to_a_returnable_(); }
  template<typename InputIterator>
  void push(InputIterator begin, InputIterator end) {
    for( ; begin != end; ++begin) { c_->add_child(*begin); }
    advance_to_a_returnable_();
  }

  reference operator*() const {
    caller_error_if(c_->queue_.empty(), "can't dereference an empty iterator");
    queue_value_type_ const& top = c_->queue_.top();
    return iteree(top.first(), boost::get<id_and_bbox_ptr>(top.second()));
  }
  pointer operator->() const { return pointer(*(*this)); }
  pointer operator++(int) {
    pointer result(*(*this));
    ++*this;
    return result;
  }
  iterator& operator++() {
    c_->queue_.pop(); advance_to_a_returnable_(); return *this;
  }

  operator unspecified_bool_type() const { return c_ ? &unspecified_bool_::member : nullptr; }

  bool operator==(iterator const& other) const { return c_ == other.c_; }
  bool operator!=(iterator const& other) const { return !(*this == other); }
};

template<typename GetCost>
inline bool bool_with_error_if_implicit_conversions_to_bool_and_cost_type_are_ambiguous(bool arg) { return arg; }
template<typename GetCost>
inline bool bool_with_error_if_implicit_conversions_to_bool_and_cost_type_are_ambiguous(typename GetCost::cost_type const&) { return true; }

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions, typename GetCostBool>
inline void filter_impl(
      ztree_node<ObjectIdentifier, CoordinateBits, NumDimensions>* tree,
      std::vector<ObjectIdentifier>& results,
      borrowed_bitset& bitmap_indicating_found_results,
      GetCostBool& getcost) {
  typedef impl::ztree_get<ObjectIdentifier, CoordinateBits, NumDimensions> ztree_get;
  if (tree) {
    if(bool_with_error_if_implicit_conversions_to_bool_and_cost_type_are_ambiguous<GetCostBool>(
          getcost.min_cost(ztree_get::get_node_bbox(tree)))) {
      for(auto id_and_bbox : ztree_get::get_node_objects_here(tree)) {
        if (!bitmap_indicating_found_results.test(id_and_bbox->second.numeric_id)) {
          bitmap_indicating_found_results.set(id_and_bbox->second.numeric_id);
          if(bool_with_error_if_implicit_conversions_to_bool_and_cost_type_are_ambiguous<GetCostBool>(
                getcost.cost(id_and_bbox->first, id_and_bbox->second.bbox))) {
            results.push_back(id_and_bbox->first);
          }
        }
      }
      filter_impl(ztree_get::get_node_child0(tree), results, bitmap_indicating_found_results, getcost);
      filter_impl(ztree_get::get_node_child1(tree), results, bitmap_indicating_found_results, getcost);
    }
  }
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions, typename GetCost>
struct iteration_types {
  typedef impl::iterator<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost> iterator;
  typedef boost::iterator_range<iterator> iterator_range;
  typedef typename iterator::value_type value_type;
  typedef boost::optional<value_type> optional_value_type;
};

}


template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
template<typename GetCost>
inline typename impl::iteration_types<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost>::iterator_range
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::iterate(GetCost const& getcost) const {
  typedef typename impl::iterator<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost> iter;
  boost::iterator_range<iter> result(iter(*this, objects_tree_.get(), getcost), iter());
  return result;
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
template<typename GetCost>
inline typename impl::iteration_types<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost>::optional_value_type
bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::find_least(GetCost const& getcost) const {
  typedef typename impl::iterator<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost> iter;
  typedef typename impl::iteration_types<ObjectIdentifier, CoordinateBits, NumDimensions, GetCost>::optional_value_type result_type;
  iter i(*this, objects_tree_.get(), getcost);
  return i ? result_type(*i) : result_type();
}

template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
template<typename GetCostBool>
inline void bbox_collision_detector<ObjectIdentifier, CoordinateBits, NumDimensions>::
filter(std::vector<ObjectIdentifier>& results, GetCostBool getcost)const {
  borrowed_bitset bitmap_indicating_found_results(objects_sequence_.size());
  impl::filter_impl(objects_tree_.get(), results, bitmap_indicating_found_results, getcost);
}

#if 0
//make & provide rough uint128_t, and width_doubling_mul - overloads for 32x32->64, 64x64->128 (, 128x128->256?)

//template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
template<num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
struct bounding_box_min_manhattan_distance {
  typedef uint64_t distance_type; //er... overflow?
  distance_type min_distance(bounding_box<CoordinateBits, NumDimensions> const& bbox)const
  { return calculate_it(center, bbox); }
  template<typename ObjectIdentifier>
  distance_type distance(reference_pair<ObjectIdentifier, bounding_box<CoordinateBits, NumDimensions> > p)const
  { return calculate_it(center, p.second); }
  typedef long point;
  bounding_box_min_manhattan_distance(point center) : center(center) {}
  point center;
};
#endif

}


#endif
