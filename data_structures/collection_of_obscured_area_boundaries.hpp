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

#ifndef LASERCAKE_COLLECTION_OF_OBSCURED_AREA_BOUNDARIES_HPP__
#define LASERCAKE_COLLECTION_OF_OBSCURED_AREA_BOUNDARIES_HPP__

// The largest logical node in the collection is always the square [-1,1] \cross [-1,1].

#include <cstring> // for memset
#include "../utils.hpp"

namespace collection_of_obscured_area_boundaries_impl {

typedef lint64_t coord_int_type;
typedef non_normalized_rational<coord_int_type> coord_type;

struct bounds_2d {
  coord_type min_x;
  coord_type min_y;
  coord_type max_x;
  coord_type max_y;
  bounds_2d(coord_type min_x, coord_type min_y, coord_type max_x, coord_type max_y):min_x(min_x),min_y(min_y),max_x(max_x),max_y(max_y){}
  bool operator==(bounds_2d const& o)const {
    return (min_x == o.min_x) && (min_y == o.min_y) && (max_x == o.max_x) && (max_y == o.max_y);
  }
};

const bounds_2d top_node_bounds(coord_type(-1,1),coord_type(-1,1),coord_type(1,1),coord_type(1,1));



/* TODO: Conceptually, there are three types of information about a (axis-aligned or perspective) point: x, y, and slope from the origin. Any two of those determine the third, with the relationships:
 slope = y / x
 y = slope * x
 x = y / slope

And any line is simply *one* of those three things held constant, with the other two allowed to vary up to certain bounds.
To save on rational height (-> rounding error), those bounds can be specified by *either* other coordinate and the cross-computation is only to be done when necessary.
By this means, they can all be generalized to the same type,
much the way "axis_aligned_line" generalizes "vertical line" and "horizontal line".

A further note: Vertices made from points in integer three-space know their x, y, and slope with approximately equal rational height, and can compute the third from any two without a height cost (because they share numerators/denominators, and the non_normalized_rational code checks for that.) But once they're cropped in one dimension, the heights of their types-of-information become different.

How this interacts with lines-between-arbitrary-pairs-of-integer-points-in-three-space remains to be considered...

*/

struct axis_aligned_line {
  axis_aligned_line(coord_type l,coord_type b1,coord_type b2,bool b):l(l),b1(b1),b2(b2),is_obscured_beyond(b){}
  // The location of the line in one dimension, and its bounds in the other dimension.
  coord_type l;
  coord_type b1;
  coord_type b2;
  // "beyond" being upwards in the dimension that crosses the line
  bool is_obscured_beyond;
  // A... hack, perhaps? ...to allow templating between aalines and plines
  inline coord_type projective_angle()const { return l; }
};
// Assuming that they're perpendicular...
inline bool intersects(axis_aligned_line l1, axis_aligned_line l2) {
  return (l1.b1 <= l2.l) && (l2.l <= l1.b2)
      && (l2.b1 <= l1.l) && (l1.l <= l2.b2);
}
struct x_or_y_boundary {
  x_or_y_boundary(coord_type b, bool i):b(b),is_x_boundary(i){}
  //x_or_y_boundary(x_or_y_boundary const& o):b(o.b),is_x_boundary(o.is_x_boundary){}
  coord_type b;
  bool is_x_boundary;
};
struct perspective_line {
  perspective_line(coord_type m,x_or_y_boundary b1,x_or_y_boundary b2,bool obsright):slope(m),b1(b1),b2(b2),is_obscured_beyond(obsright),positive_slope(m > coord_type(0)){}
  coord_type slope;
  // A... hack, perhaps? ...to allow templating between aalines and plines
  inline coord_type projective_angle()const { return slope; }
  x_or_y_boundary b1;
  x_or_y_boundary b2;
  bool is_obscured_beyond; // in this case, meaning to the right
  bool positive_slope;
  bool is_obscured_to_the_right()const { return is_obscured_beyond; }
  bool is_obscured_above()const { return positive_slope !=  is_obscured_beyond; }
  coord_type y_intercept(coord_type x)const {
    // slope = y / x;
    // y = x * slope
    return x * slope;
  }
  coord_type x_intercept(coord_type y)const {
    // slope = y / x;
    // x = y / slope
    return y / slope;
  }
  coord_type min_x()const {
    return b1.is_x_boundary ? b1.b : x_intercept(b1.b);
  }
  coord_type max_x()const {
    return b2.is_x_boundary ? b2.b : x_intercept(b2.b);
  }
  coord_type start_y()const {
    return b1.is_x_boundary ? y_intercept(b1.b) : b1.b;
  }
  coord_type end_y()const {
    return b2.is_x_boundary ? y_intercept(b2.b) : b2.b;
  }
  coord_type min_y()const {
    return positive_slope ? start_y() : end_y();
  }
  coord_type max_y()const {
    return positive_slope ? end_y() : start_y();
  }
  bool point_is_within_end_boundaries(coord_type x, coord_type y)const {
    if (b1.is_x_boundary && (x < b1.b)) return false;
    if (b2.is_x_boundary && (x > b2.b)) return false;
    if (positive_slope) {
      if ((!b1.is_x_boundary) && (y < b1.b)) return false;
      if ((!b2.is_x_boundary) && (y > b2.b)) return false;
    }
    else {
      if ((!b1.is_x_boundary) && (y > b1.b)) return false;
      if ((!b2.is_x_boundary) && (y < b2.b)) return false;
    }
    return true;
  }
};
struct pline_tpt_loc {
  pline_tpt_loc(coord_type b, bool i, perspective_line const* pline):b(b,i),pline(pline){}
  x_or_y_boundary b;
  perspective_line const* pline;
  
  // A... hack, perhaps? ...to allow templating between aalines and plines
  operator x_or_y_boundary()const { return b; }
  
  bool operator<(pline_tpt_loc const& other)const {
    assert(pline == other.pline);
    if (b.is_x_boundary == other.b.is_x_boundary) {
      return (b.b < other.b.b) == (b.is_x_boundary || pline->positive_slope);
    }
    // How to compare an x boundary to a y boundary, with a given slope?
    // Let's say we sort by x - then "xbound < ybound" is
    // x < y / m
    else if (b.is_x_boundary) {
      return (b.b < pline->x_intercept(other.b.b));
    }
    else if (other.b.is_x_boundary) {
      return (b.b < pline->y_intercept(other.b.b));
    }
    else assert(false);
  }
};

// "transition point"
template<typename BoundaryLocType = coord_type> struct tpt {
  tpt(BoundaryLocType l, bool b):loc(l),is_obscured_beyond(b){}
  BoundaryLocType loc;
  bool is_obscured_beyond;
  bool operator<(tpt const& other)const{ return loc < other.loc; }
};

template<typename BoundaryLocType = coord_type> struct obscured_areas_tracker_1d {
  obscured_areas_tracker_1d():is_obscured_initially(false){}
  bool is_obscured_initially;
  std::multiset<tpt<BoundaryLocType>> tpts;
  
  void clear() {
    tpts.clear(); is_obscured_initially = false;
  }
  
  inline void insert(tpt<BoundaryLocType> t) {
    tpts.insert(t);
  }
  bool point_is_obscured(BoundaryLocType point)const {
    auto b = tpts.lower_bound(tpt<BoundaryLocType>(point, false));
    if (b == tpts.begin()) {
      return is_obscured_initially;
    }
    else if ((b != tpts.end()) && (b->loc == point)) {
      return true;
    }
    else {
      --b;
      return b->is_obscured_beyond;
    }
  }
  void combine(obscured_areas_tracker_1d const& other) {
    auto l1 = tpts.begin();
    auto l2 = other.tpts.begin();
    bool currently_obscured_1 = is_obscured_initially;
    bool currently_obscured_2 = other.is_obscured_initially;
    
    is_obscured_initially = is_obscured_initially || other.is_obscured_initially;
    
    while ((l1 != tpts.end()) && l2 != other.tpts.end()) {
      if ((l1->loc == l2->loc) && (currently_obscured_1 || currently_obscured_2) && (l1->is_obscured_beyond || l2->is_obscured_beyond)) {
        currently_obscured_1 = l1->is_obscured_beyond;
        currently_obscured_2 = l2->is_obscured_beyond;
        tpts.erase(l1++);
        ++l2;
      }
      else if (l1->loc <= l2->loc) {
        currently_obscured_1 = l1->is_obscured_beyond;
        if (currently_obscured_2) tpts.erase(l1++);
        else ++l1;
      }
      else if (l2->loc <= l1->loc) {
        currently_obscured_2 = l2->is_obscured_beyond;
        if (!currently_obscured_1) tpts.insert(*l2);
        ++l2;
      }
    }
  }
};

struct logical_node_lines_collection {
public:
  obscured_areas_tracker_1d<> left_side;
  
  std::vector<axis_aligned_line> hlines;
  std::vector<axis_aligned_line> vlines;
  std::vector<perspective_line> plines;
  size_t size()const { return hlines.size() + vlines.size() + plines.size(); }
  
  bounds_2d bounds;
  
  logical_node_lines_collection():bounds(top_node_bounds){}
  logical_node_lines_collection(bounds_2d bounds):bounds(bounds){}
  bool empty()const {
    return hlines.empty() && vlines.empty() && plines.empty() && !left_side.is_obscured_initially;
  }
  bool full()const {
    return hlines.empty() && vlines.empty() && plines.empty() && left_side.is_obscured_initially;
  }
  
  void clear() {
    hlines.clear(); vlines.clear(); plines.clear(); left_side.clear();
  }

private:
  void note_hline_in_left_side(axis_aligned_line const& hline);
  void note_pline_in_left_side(perspective_line const& pline);
  // Only regenerates the tpts; you have to keep track of is_obscured_initially on your own,
  // since it can't always be inferred
  void regenerate_left_side_tpts();

public:
  // The "insert" functions are expected to be used a bunch of times in sequence,
  // to insert a set of lines that form a closed loop,
  // which, in total, should create a valid structure.
  // In the middle of that process, the structure will be invalid.
  // left_side.is_obscured_initially isn't updated by this;
  // currently you have to set it yourself, elsewhere.
  // It could be inferred from the lines as long as there's at least one line...
  // but it obviously can't be inferred from the lines when there aren't any
  void insert_hline(axis_aligned_line const& hline);
  void insert_vline(axis_aligned_line const& vline);
  void insert_pline(perspective_line const& pline);
  
  // Can copy from any (direct or indirect) child.
  // Use this as many times as is necessary after
  // default-constructing a logical_node_lines_collection and setting its bounds to the proper values,
  // in order to collapse a child tree into a single node
  // (or perform the trivial collapse of a single lines-node to itself, i.e. copying).
  void copy_from_same_or_child(logical_node_lines_collection const& child_collection);
  
  // Copy another collection, but crop to a certain set of bounds.
  logical_node_lines_collection(logical_node_lines_collection const& original, bounds_2d bounds);
  
  bool point_is_obscured(coord_type x, coord_type y)const;

private:
  // Utility functions for logical_node_lines_collection::relate():
  bool handle_aaline_vs_aaline(axis_aligned_line l1, obscured_areas_tracker_1d<>& l1_info, axis_aligned_line l2, obscured_areas_tracker_1d<>& l2_info);

  bool handle_aaline_vs_pline(bool aaline_is_horizontal, axis_aligned_line aaline, obscured_areas_tracker_1d<>& aaline_info, perspective_line pline, obscured_areas_tracker_1d<pline_tpt_loc>& pline_info);
  
  void mark_obscured_aalines(logical_node_lines_collection const& other, bool aalines_are_horizontal, bool src_aalines_is_our_own_collection, std::vector<axis_aligned_line> const& src_aalines, std::vector<obscured_areas_tracker_1d<>> const& tpts, std::vector<size_t>& completely_obscured_or_completely_visible_list);
  
  void mark_obscured_plines(logical_node_lines_collection const& other, bool src_plines_is_our_own_collection, std::vector<perspective_line> const& src_plines, std::vector<obscured_areas_tracker_1d<pline_tpt_loc>> const& tpts, std::vector<size_t>& completely_obscured_or_completely_visible_list);

public:
  bool relate(logical_node_lines_collection const& other, bool insert);
  
  bool allows_to_be_visible(logical_node_lines_collection const& other) {
    return relate(other, false);
  }
  bool combine(logical_node_lines_collection const& other) {
    return relate(other, true);
  }
};



const uint8_t ALL_CLEAR = 0x0;
const uint8_t ALL_BLOCKED = 0x1;
const uint8_t MIXED_AS_LINES = 0x2;
const uint8_t MIXED_AS_CHILDREN = 0x3;

constexpr int total_entries_at_subimpnode_level(int level) {
  return (level == 0) ? 1 : (4 * total_entries_at_subimpnode_level(level - 1));
}

constexpr int total_entries_above_subimpnode_level(int level) {
  return (level == 0) ? 0 : (total_entries_at_subimpnode_level(level - 1) + total_entries_above_subimpnode_level(level - 1));
}

const uint32_t total_subimpnode_levels = 3;
const uint32_t max_subimpnode_level = total_subimpnode_levels - 1;

const uint32_t total_entries = total_entries_above_subimpnode_level(total_subimpnode_levels);
const uint32_t total_entry_bytes = total_entries / 4;
const uint32_t total_entries_at_bottom_subimpnode_level = total_entries_at_subimpnode_level(total_subimpnode_levels);

// An implementation_node contains information about total_subimpnode_levels levels of logical nodes.
struct implementation_node {
  /* Each entry contains two bits of information about each of four children.
     The top entry gives information about the four quadrants.
     The next four entries (in one byte) give information about the quadrants of the quadrants.
     The next sixteen... and so on.
     
     Child entries are not necessarily valid if their parents aren't marked as
     being defined by their children ("MIXED_AS_CHILDREN")
     
     If node A is entry N of level M, then A's children are entries N*4-N*4 + 3 of level M+1. */
  uint8_t top_entry;
  uint8_t entry_bytes[total_entry_bytes];
  
  /* Each pointer is one of the following:
     1) A pointer to another instance of this struct,
           if-and-only-if this pointer's entry at the bottom level,
           and all entries above that, are "MIXED_AS_CHILDREN", or
     2) A pointer to a collection of lines,
           if-and-only-if this is the *first* pointer in a block described by
           a "MIXED_AS_LINES" entry with "MIXED_AS_CHILDREN" entries above it, or
     3) NULL
           otherwise. */
  void *lines_or_further_children[total_entries_at_bottom_subimpnode_level];
  
  implementation_node() {
    memset(this, 0, sizeof(implementation_node));
  }
};

struct logical_node {
public:
  implementation_node* impnode;
  uint32_t which_entry_at_this_subimpnode_level;
  uint8_t subimpnode_level;
  
  uint8_t contents_type;
  uint8_t which_bits_in_byte;
  uint32_t which_byte_in_impnode;
  // For the bounds info, there's a special case to allow the [-1,1] \cross [-1,1] shape.
  // (The special case would be, effectively, numerators of -0.5 and a denominator of 0.5).
  // The upper bound numerators are always 1 + these
  coord_int_type x_lower_bound_numerator;
  coord_int_type y_lower_bound_numerator;
  coord_int_type bounds_denominator; // the same for both X and Y, and always a power of 2
  
  bounds_2d bounds()const;
  
  logical_node(implementation_node* i,uint8_t l,uint32_t w,coord_int_type nx,coord_int_type ny,coord_int_type d)
    :
    impnode(i),
    which_entry_at_this_subimpnode_level(w),
    subimpnode_level(l),
    x_lower_bound_numerator(nx),
    y_lower_bound_numerator(ny),
    bounds_denominator(d)
{
  assert(subimpnode_level <= max_subimpnode_level);
  if (subimpnode_level == 0) {
    contents_type = impnode->top_entry;
  }
  else {
    which_byte_in_impnode = (total_entries_above_subimpnode_level(subimpnode_level) / 4) + (which_entry_at_this_subimpnode_level/4);
    which_bits_in_byte = 2*(which_entry_at_this_subimpnode_level % 4);
    assert(which_byte_in_impnode < total_entry_bytes);
    contents_type = (impnode->entry_bytes[which_byte_in_impnode] >> which_bits_in_byte) & 0x3;
  }
}

private:
  void set_contents_type_bits(uint8_t type);
  void clear_children();
  void create_children(uint8_t children_contents_type);
  uint32_t first_entry_at_bottom_subimpnode_level()const {
    return which_entry_at_this_subimpnode_level << (2*(total_subimpnode_levels - subimpnode_level));
  }

public:
  logical_node_lines_collection*& lines_pointer_reference()const;
  
  logical_node child(int which_child)const;
  
  void split_if_necessary();
  void retrieve_all_lines(logical_node_lines_collection& collector);
  void set_contents_type(uint8_t new_type, bool keep_line_info = true);
  
  bool lines_collection_is_visible(logical_node_lines_collection const& lines);
  bool insert_from_lines_collection(logical_node_lines_collection const& lines);
};


class obscured_areas_tree {
public:
  bool insert_from_lines_collection(logical_node_lines_collection const& lines) {
    return top_logical_node().insert_from_lines_collection(lines);
  }
  logical_node top_logical_node() {
    return logical_node(&top_impnode, 0, 0, 0, 0, 0);
  }
private:
  implementation_node top_impnode;
};

} /* namespace collection_of_obscured_area_boundaries */

#endif

















