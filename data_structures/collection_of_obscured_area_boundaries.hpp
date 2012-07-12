
// The largest logical node in the collection is always the square [-1,1] \cross [-1,1].

#include "../utils.hpp"
typedef lasercake_int<int64_t>::type coord_int_type;
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
bounds_2d top_node_bounds(coord_type(-1,1),coord_type(-1,1),coord_type(1,1),coord_type(1,1));




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
  perspective_line(coord_type m,x_or_y_boundary b1,x_or_y_boundary b2,bool obsright):slope(m),b1(b1),b2(b2),is_obscured_beyond(obsright){}
  coord_type slope;
  // A... hack, perhaps? ...to allow templating between aalines and plines
  inline coord_type projective_angle()const { return slope; }
  x_or_y_boundary b1;
  x_or_y_boundary b2;
  bool is_obscured_beyond; // in this case, meaning to the right
  bool is_obscured_to_the_right()const { return is_obscured_beyond; }
  bool is_obscured_above()const { return (slope < coord_type(0)) == is_obscured_beyond; }
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
  bool point_is_within_end_boundaries(coord_type x, coord_type y)const {
    if (b1.is_x_boundary && (x < b1.b)) return false;
    if (b2.is_x_boundary && (x > b2.b)) return false;
    if (slope > coord_type(0)) {
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
  pline_tpt_loc(coord_type b, bool i, coord_type m):b(b,i),slope(m){}
  x_or_y_boundary b;
  coord_type slope;
  
  // A... hack, perhaps? ...to allow templating between aalines and plines
  operator x_or_y_boundary()const { return b; }
  
  bool operator<(pline_tpt_loc const& other)const {
    if (b.is_x_boundary == other.b.is_x_boundary) {
      return (b.b < other.b.b) == (b.is_x_boundary || (slope > coord_type(0)));
    }
    // How to compare an x boundary to a y boundary, with a given slope?
    // Let's say we sort by x - then "xbound < ybound" is
    // x < y / m
    else if (b.is_x_boundary) {
      return (b.b < other.b.b / slope);
    }
    else if (other.b.is_x_boundary) {
      return (b.b < other.b.b * slope);
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
  
  inline void insert(tpt<BoundaryLocType> t) {
    tpts.insert(t);
  }
  bool point_is_obscured(BoundaryLocType point)const {
    if (tpts.empty()) return is_obscured_initially;
    auto b = tpts.lower_bound(tpt<BoundaryLocType>(point, false));
    if (b->loc == point) {
      return true;
    }
    else if (b == tpts.begin()){
      return is_obscured_initially;
    }
    else {
      --b;
      return b->is_obscured_beyond;
    }
  }
  void combine(obscured_areas_tracker_1d const& other) {
    auto l1 = tpts.begin();
    auto l2 = other.tpts.begin();
    is_obscured_initially = is_obscured_initially || other.is_obscured_initially;
    
    bool currently_obscured_1 = is_obscured_initially;
    bool currently_obscured_2 = other.is_obscured_initially;
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

template<class LineType, typename TptLocType> void slice_marked_lines(bool src_lines_is_our_own_collection, std::vector<LineType> const& src_lines, std::vector<LineType>& dst_lines, std::vector<obscured_areas_tracker_1d<TptLocType>> const& tpts) {
  for (size_t i = 0; i < tpts.size(); ++i) {
    auto const& switch_points = tpts[i].tpts;
    auto t = switch_points.begin();
    if (t != switch_points.end()) {
      LineType const& line = src_lines[i];
      // If we're in the dst collection, replace ourselves with the first still-visible piece of ourselves.
      auto original_b2 = line.b2;
      if (t->is_obscured_beyond) {
        auto new_b1 = t->loc;
        if (src_lines_is_our_own_collection) dst_lines[i].b1 = new_b1;
        ++t;
        //TODO: This assertion might fail if rounding error stuff happens. What to do about that?
        assert(!t->is_obscured_beyond);
        if (t != switch_points.end()) {
          if (src_lines_is_our_own_collection) dst_lines[i].b2 = t->loc;
          else dst_lines.push_back(LineType(line.projective_angle(), new_b1, t->loc, line.is_obscured_beyond));
        }
        else {
          if(!src_lines_is_our_own_collection) dst_lines.push_back(LineType(line.projective_angle(), new_b1, original_b2, line.is_obscured_beyond));
        }
      }
      else {
        if (src_lines_is_our_own_collection) dst_lines[i].b2 = t->loc;
        else dst_lines.push_back(LineType(line.projective_angle(), line.b1, t->loc, line.is_obscured_beyond));
      }
      
      // Now insert the rest of the still-visible pieces of ourselves at the end.
      ++t;
      while(t != switch_points.end()) {
        //TODO: This assertion might fail if rounding error stuff happens. What to do about that?
        assert(!t->is_obscured_beyond);
        auto startb = t->loc;
        ++t;
        if (t == switch_points.end()) {
          dst_lines.push_back(LineType(line.projective_angle(), startb, original_b2, line.is_obscured_beyond));
        }
        else {
          dst_lines.push_back(LineType(line.projective_angle(), startb, t->loc, line.is_obscured_beyond));
          ++t;
        }
      }
    }
  }
}
template<class LineType> void purge_obscured_lines(std::vector<LineType>& lines, std::vector<size_t> const& obscured_list) {
  for (int i = obscured_list.size() - 1; i >= 0 ; --i) {
    lines[obscured_list[i]] = lines.back(); lines.pop_back();
  }
}
template<class LineType> void copy_visible_lines(std::vector<LineType> const& src_lines, std::vector<LineType> dst_lines, std::vector<size_t> const& visible_list) {
  for (size_t i : visible_list) {
    dst_lines.push_back(src_lines[i]);
  }
}

struct logical_node_lines_collection {
  obscured_areas_tracker_1d<> left_side;
  
  std::vector<axis_aligned_line> hlines;
  std::vector<axis_aligned_line> vlines;
  std::vector<perspective_line> plines;
  size_t size()const { return hlines.size() + vlines.size() + plines.size(); }
  
  bounds_2d bounds;
  
  logical_node_lines_collection():bounds(top_node_bounds){}
  bool empty()const {
    return hlines.empty() && vlines.empty() && plines.empty() && !left_side.is_obscured_initially;
  }
  bool full()const {
    return hlines.empty() && vlines.empty() && plines.empty() && left_side.is_obscured_initially;
  }
  
  // The "insert" functions are expected to be used a bunch of times in sequence,
  // which, in total, should create a valid structure.
  // In the middle of that process, the structure will be invalid.
  void insert_hline(axis_aligned_line const& hline) {
    assert(hline.b1 <= bounds.max_x);
    assert(hline.b2 >= bounds.min_x);
    assert(hline.l <= bounds.max_y);
    assert(hline.l >= bounds.min_y);
    hlines.push_back(hline);
    if ((hline.b1 <= bounds.min_x) && (hline.b2 >= bounds.min_x)) {
      left_side.tpts.insert(tpt<>(hline.l, hline.is_obscured_beyond));
    }
  }
  void insert_vline(axis_aligned_line const& vline) {
    assert(vline.b1 <= bounds.max_y);
    assert(vline.b2 >= bounds.min_y);
    assert(vline.l <= bounds.max_x);
    assert(vline.l >= bounds.min_x);
    vlines.push_back(vline);
  }
  void insert_pline(perspective_line const& pline) {
    const coord_type y = pline.y_intercept(bounds.min_x);
    if ((y >= bounds.min_y) && (y <= bounds.max_y)) {
      left_side.tpts.insert(tpt<>(y, pline.is_obscured_above()));
    }
  }
  
  // Can copy from any (direct or indirect) child.
  // Use this as many times as is necessary after
  // default-constructing a logical_node_lines_collection and setting its bounds to the proper values,
  // in order to collapse a child tree into a single node
  // (or perform the trivial collapse of a single lines-node to itself, i.e. copying).
  void copy_from_same_or_child(logical_node_lines_collection const& child_collection) {
    for (auto const& hline : child_collection.hlines) {
      insert_hline(hline);
    }
    for (auto const& vline : child_collection.vlines) {
      insert_vline(vline);
    }
    for (auto const& pline : child_collection.plines) {
      insert_pline(pline);
    }
    if (child_collection.bounds.min_x == bounds.min_x && child_collection.bounds.min_y == bounds.min_y) {
      left_side.is_obscured_initially = child_collection.left_side.is_obscured_initially;
    }
  }
  
  // Copy another collection, but crop to a certain set of bounds.
  logical_node_lines_collection(logical_node_lines_collection const& original, bounds_2d bounds):bounds(bounds) {
    /* We could copy their left-side info, but it's easy to just let it be generated by the insert functions.
    
    if (bounds.min_x == original.bounds.min_x) {
      left_side.tpts.insert(original.left_side.tpts.lower_bound(tpt<>(bounds.min_y, false)), original.left_side.tpts.upper_bound(tpt<>(max_y, false)));
      if (left_side.tpts.empty()) {
        left_side.is_obscured_initially = original.left_side.point_is_obscured(bounds.min_y);
      }
      else {
        left_side.is_obscured_initially = !left_side.begin().is_obscured_beyond;
      }
    }
    else {
    }*/
    
    left_side.is_obscured_initially = original.point_is_obscured(bounds.min_x, bounds.min_y);
    
    for (auto const& hline : original.hlines) {
      if ((hline.b1 <= bounds.max_x) && (hline.b2 >= bounds.min_x)
       && (hline.l  <= bounds.max_y) && (hline.l  >= bounds.min_y)) {
        insert_hline(hline);
      }
    }
    for (auto const& vline : original.vlines) {
      if ((vline.b1 <= bounds.max_y) && (vline.b2 >= bounds.min_y)
       && (vline.l  <= bounds.max_x) && (vline.l  >= bounds.min_x)) {
        insert_vline(vline);
      }
    }
    for (auto const& pline : original.plines) {
      // This is ugly, maybe there's a better way to check if a pline intersects a box?
      if ((  pline.b1.is_x_boundary  && pline.b1.b <= bounds.max_x)
       || ((!pline.b1.is_x_boundary) && (
                ((pline.slope > coord_type(0)) && (pline.b1.b <= bounds.max_y))
             || ((pline.slope < coord_type(0)) && (pline.b1.b >= bounds.min_y))
           ))) {
        if ((  pline.b2.is_x_boundary  && pline.b2.b >= bounds.min_x)
         || ((!pline.b2.is_x_boundary) && (
                  ((pline.slope > coord_type(0)) && (pline.b2.b >= bounds.min_y))
               || ((pline.slope < coord_type(0)) && (pline.b2.b <= bounds.max_y))
             ))) {
          const coord_type lyx = pline.x_intercept(bounds.min_y);
          const coord_type myx = pline.x_intercept(bounds.max_y);
          if (((lyx <= bounds.max_x) && (myx >= bounds.min_x))
           || ((lyx >= bounds.min_x) && (myx <= bounds.max_x))) {
            insert_pline(pline);
          }
        }
      }
    }
  }
  
  bool point_is_obscured(coord_type x, coord_type y)const {
    // Effectively: Draw a horizontal line back to the left side. If it intersects any lines,
    // then the closest intersection dictates whether you're obscured or not.
    // Otherwise, check the left_side structure to see whether you're obscured.
    bool found_any_lines;
    coord_type best_distance;
    bool best_obscuredness;
    for (auto const& vline : vlines) {
      if ((vline.l <= x) && (vline.l >= bounds.min_x) && (vline.b1 <= y) && (vline.b2 >= y)) {
        coord_type distance = x - vline.l;
        if ((!found_any_lines) || (distance < best_distance)) {
          found_any_lines = true;
          best_distance = distance;
          best_obscuredness = vline.is_obscured_beyond;
        }
      }
    }
    for (auto const& pline : plines) {
      coord_type pline_x = pline.x_intercept(y);
      if ((pline_x <= x) && (pline_x >= bounds.min_x) && pline.point_is_within_end_boundaries(pline_x, y)) {
        coord_type distance = x - pline_x;
        if ((!found_any_lines) || (distance < best_distance)) {
          found_any_lines = true;
          best_distance = distance;
          best_obscuredness = pline.is_obscured_to_the_right();
        }
      }
    }
    if (found_any_lines) {
      return best_obscuredness;
    }
    else {
      return left_side.point_is_obscured(y);
    }
  }

  bool handle_aaline_vs_aaline(axis_aligned_line l1, obscured_areas_tracker_1d<>& l1_info, axis_aligned_line l2, obscured_areas_tracker_1d<>& l2_info) {
    // Note: If the rules have been followed, they can never intersect outside the box,
    // so we don't need to check that the point of intersection is inside the box.
    if (intersects(l1, l2)) {
      l1_info.insert(tpt<>(l2.l, l2.is_obscured_beyond));
      l2_info.insert(tpt<>(l1.l, l1.is_obscured_beyond));
      return true;
    }
    else {
      return false;
    }
  }

  bool handle_aaline_vs_pline(bool aaline_is_horizontal, axis_aligned_line aaline, obscured_areas_tracker_1d<>& aaline_info, perspective_line pline, obscured_areas_tracker_1d<pline_tpt_loc>& pline_info) {
    coord_type b = (aaline_is_horizontal ? pline.x_intercept(aaline.l) : pline.y_intercept(aaline.l));
    // Make sure the point of intersection is inside the box...
    if (aaline_is_horizontal) {
      if ((b < bounds.min_x) || (b > bounds.max_x) || (aaline.l < bounds.min_y) || (aaline.l > bounds.max_y)) return false;
    }
    else {
      if ((b < bounds.min_y) || (b > bounds.max_y) || (aaline.l < bounds.min_x) || (aaline.l > bounds.max_x)) return false;
    }
    if ((aaline.b1 < b) && (b < aaline.b2) && pline.point_is_within_end_boundaries((aaline_is_horizontal ? b : aaline.l), (aaline_is_horizontal ? aaline.l : b))) {
      aaline_info.insert(tpt<>(b, (aaline_is_horizontal ? pline.is_obscured_to_the_right() : pline.is_obscured_above())));
      pline_info.insert(tpt<pline_tpt_loc>(
          pline_tpt_loc(aaline.l, !aaline_is_horizontal, pline.slope),
          aaline.is_obscured_beyond == ((pline.slope > coord_type(0)) || !aaline_is_horizontal)
        ));
      return true;
    }
    else {
      return false;
    }
  }
  
  void mark_obscured_aalines(logical_node_lines_collection const& other, bool aalines_are_horizontal, bool src_aalines_is_our_own_collection, std::vector<axis_aligned_line> const& src_aalines, std::vector<obscured_areas_tracker_1d<>> const& tpts, std::vector<size_t>& completely_obscured_or_completely_visible_list) {
    for (size_t i = 0; i < tpts.size(); ++i) {
      if (tpts[i].tpts.empty()) {
        axis_aligned_line const& aaline = src_aalines[i];
        if (other.point_is_obscured(aalines_are_horizontal ? aaline.b1 : aaline.l, aalines_are_horizontal ? aaline.l : aaline.b1) == src_aalines_is_our_own_collection) {
          completely_obscured_or_completely_visible_list.push_back(i);
        }
      }
    }
  }
  void mark_obscured_plines(logical_node_lines_collection const& other, bool src_plines_is_our_own_collection, std::vector<perspective_line> const& src_plines, std::vector<obscured_areas_tracker_1d<pline_tpt_loc>> const& tpts, std::vector<size_t>& completely_obscured_or_completely_visible_list) {
    for (size_t i = 0; i < tpts.size(); ++i) {
      if (tpts[i].tpts.empty()) {
        perspective_line const& pline = src_plines[i];
        if (other.point_is_obscured(
              pline.b1.is_x_boundary ? pline.b1.b : pline.x_intercept(pline.b1.b),
              pline.b1.is_x_boundary ? pline.y_intercept(pline.b1.b) : pline.b1.b
            ) == src_plines_is_our_own_collection) {
          completely_obscured_or_completely_visible_list.push_back(i);
        }
      }
    }
  }
  
  bool combine(logical_node_lines_collection const& other) {
    bool result = false;
    
    // ======== Combine the lines info. ==========
    // First, collect all overlaps.
    std::vector<obscured_areas_tracker_1d<>>               hlines_tpts(      hlines.size());
    std::vector<obscured_areas_tracker_1d<>>              ohlines_tpts(other.hlines.size());
    std::vector<obscured_areas_tracker_1d<>>               vlines_tpts(      vlines.size());
    std::vector<obscured_areas_tracker_1d<>>              ovlines_tpts(other.vlines.size());
    std::vector<obscured_areas_tracker_1d<pline_tpt_loc>>  plines_tpts(      plines.size());
    std::vector<obscured_areas_tracker_1d<pline_tpt_loc>> oplines_tpts(other.plines.size());
    std::vector<size_t> completely_obscured_hlines;
    std::vector<size_t> completely_obscured_vlines;
    std::vector<size_t> completely_obscured_plines;
    std::vector<size_t> completely_visible_ohlines;
    std::vector<size_t> completely_visible_ovlines;
    std::vector<size_t> completely_visible_oplines;
    for (size_t i = 0; i < hlines.size(); ++i) {
      for (size_t j = 0; j < other.vlines.size(); ++j) {
        if (handle_aaline_vs_aaline(hlines[i], hlines_tpts[i], other.vlines[j], ovlines_tpts[j])) result = true;
      }
      for (size_t j = 0; j < other.plines.size(); ++j) {
        if (handle_aaline_vs_pline(true, hlines[i], hlines_tpts[i], other.plines[j], oplines_tpts[j])) result = true;
      }
    }
    for (size_t i = 0; i < vlines.size(); ++i) {
      for (size_t j = 0; j < other.hlines.size(); ++j) {
        if (handle_aaline_vs_aaline(vlines[i], vlines_tpts[i], other.hlines[j], ohlines_tpts[j])) result = true;
      }
      for (size_t j = 0; j < other.plines.size(); ++j) {
        if (handle_aaline_vs_pline(false, vlines[i], vlines_tpts[i], other.plines[j], oplines_tpts[j])) result = true;
      }
    }
    for (size_t i = 0; i < plines.size(); ++i) {
      for (size_t j = 0; j < other.hlines.size(); ++j) {
        if (handle_aaline_vs_pline(true, other.hlines[j], ohlines_tpts[j], plines[i], plines_tpts[i])) result = true;
      }
      for (size_t j = 0; j < other.vlines.size(); ++j) {
        if (handle_aaline_vs_pline(false, other.vlines[j], ovlines_tpts[j], plines[i], plines_tpts[i])) result = true;
      }
    }
    
    // For lines that don't have overlaps, note whether or not they're completely obscured.
    // They can only be completely obscured *by the other collection*
    //  (since each line is completely necessary to its own collection)
    // and since we haven't modified either collection yet, we can just ask the other collection
    // whether a point on that line is visible.
    mark_obscured_aalines(other, true , true ,       hlines,  hlines_tpts, completely_obscured_hlines);
    mark_obscured_aalines(other, false, true ,       vlines,  vlines_tpts, completely_obscured_vlines);
    mark_obscured_plines (other,        true ,       plines,  plines_tpts, completely_obscured_plines);
    mark_obscured_aalines(other, true , false, other.hlines, ohlines_tpts, completely_visible_ohlines);
    mark_obscured_aalines(other, false, false, other.vlines, ovlines_tpts, completely_visible_ovlines);
    mark_obscured_plines (other,        false, other.plines, oplines_tpts, completely_visible_oplines);
    if ((!completely_visible_ohlines.empty()) || (!completely_visible_ovlines.empty()) || (!completely_visible_oplines.empty())) result = true;
    
    // Then slice up the overlapped lines based on the overlaps.
    slice_marked_lines(true ,       hlines, hlines,  hlines_tpts);
    slice_marked_lines(true ,       vlines, vlines,  vlines_tpts);
    slice_marked_lines(true ,       plines, plines,  plines_tpts);
    slice_marked_lines(false, other.hlines, hlines, ohlines_tpts);
    slice_marked_lines(false, other.vlines, vlines, ovlines_tpts);
    slice_marked_lines(false, other.plines, plines, oplines_tpts);
    
    // When you're done, purge the lines that were completely obscured.
    // And copy over the completely-visible lines from the other list.
    purge_obscured_lines(hlines, completely_obscured_hlines);
    purge_obscured_lines(vlines, completely_obscured_vlines);
    purge_obscured_lines(plines, completely_obscured_plines);
    copy_visible_lines(other.hlines, hlines, completely_visible_ohlines);
    copy_visible_lines(other.vlines, vlines, completely_visible_ovlines);
    copy_visible_lines(other.plines, plines, completely_visible_oplines);
    
    
    // ======== Combine the left-side info. ==========
    left_side.combine(other.left_side);
    
    return result;
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
  return (level == 0) ? 0 : (total_entries_at_subimpnode_level(level - 1) * total_entries_above_subimpnode_level(level - 1));
}

const int total_subimpnode_levels = 5;
const int max_subimpnode_level = total_subimpnode_levels - 1;

const int total_entries = total_entries_above_subimpnode_level(total_subimpnode_levels);
const int total_entry_bytes = total_entries / 4;
const int total_entries_at_bottom_subimpnode_level = total_entries_at_subimpnode_level(total_subimpnode_levels);

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
};

struct logical_node {
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
  
  bounds_2d bounds()const {
    if (bounds_denominator == 0) {
      return top_node_bounds;
    }
    else {
      return bounds_2d(
        coord_type(x_lower_bound_numerator  ,bounds_denominator),
        coord_type(y_lower_bound_numerator  ,bounds_denominator),
        coord_type(x_lower_bound_numerator+1,bounds_denominator),
        coord_type(y_lower_bound_numerator+1,bounds_denominator)
      );
    }
  }
  
  logical_node(implementation_node* i,uint8_t l,uint32_t w,coord_int_type nx,coord_int_type ny,coord_int_type d):impnode(i),subimpnode_level(l),which_entry_at_this_subimpnode_level(w),x_lower_bound_numerator(nx),y_lower_bound_numerator(ny),bounds_denominator(d)
  {
    assert(subimpnode_level <= max_subimpnode_level);
    assert(subimpnode_level >= 0);
    if (subimpnode_level == 0) {
      contents_type = impnode->top_entry;
    }
    else {
      which_byte_in_impnode = total_entries_above_subimpnode_level(subimpnode_level) + (which_entry_at_this_subimpnode_level/4);
      which_bits_in_byte = 2*(which_entry_at_this_subimpnode_level % 4);
      assert(which_byte_in_impnode >= 0);
      assert(which_byte_in_impnode < total_entry_bytes);
      contents_type = (impnode->entry_bytes[which_byte_in_impnode] >> which_bits_in_byte) & 0x3;
    }
  }
  
  void set_contents_type_bits(uint8_t type) {
    contents_type = type;
    impnode->entry_bytes[which_byte_in_impnode] = (impnode->entry_bytes[which_byte_in_impnode] & (~(0x3 << which_bits_in_byte))) | (type << which_bits_in_byte);
  }
  
  logical_node_lines_collection*& lines_pointer_reference()const {
    assert(contents_type == MIXED_AS_LINES);
    
    const uint32_t which_entry_at_bottom_subimpnode_level = which_entry_at_this_subimpnode_level << (total_subimpnode_levels - subimpnode_level);
    
    return reinterpret_cast<logical_node_lines_collection*&>(impnode->lines_or_further_children[which_entry_at_bottom_subimpnode_level]);
  }
  
  logical_node child(int which_child)const {
    assert(contents_type == MIXED_AS_CHILDREN);
    
    const uint32_t which_entry_at_next_subimpnode_level = which_entry_at_this_subimpnode_level * 4 + which_child;
    const coord_int_type new_denom = (bounds_denominator == 0) ? 1 : (bounds_denominator * 2);
    const coord_int_type new_xnum = ((bounds_denominator == 0) ? -1 : (x_lower_bound_numerator * 2)) + (which_child & 0x1);
    const coord_int_type new_ynum = ((bounds_denominator == 0) ? -1 : (y_lower_bound_numerator * 2)) + ((which_child & 0x2) >> 1);
    
    if (subimpnode_level == max_subimpnode_level) {
      assert(which_entry_at_next_subimpnode_level >= 0);
      assert(which_entry_at_next_subimpnode_level < total_entries_at_bottom_subimpnode_level);
      assert(impnode->lines_or_further_children[which_entry_at_next_subimpnode_level] != NULL);
      return logical_node(static_cast<implementation_node*>(impnode->lines_or_further_children[which_entry_at_next_subimpnode_level]), 0, 0, new_xnum, new_ynum, new_denom);
    }
    else {
      return logical_node(impnode, subimpnode_level + 1, which_entry_at_next_subimpnode_level, new_xnum, new_ynum, new_denom);
    }
  }
  
  void split_if_necessary();
  void retrieve_all_lines(logical_node_lines_collection& collector);
  void clear_children();
  void create_children(uint8_t children_contents_type);
  void set_contents_type(uint8_t new_type, bool keep_line_info = true);
  bool insert_from_lines_collection(logical_node_lines_collection const& lines);
};

class collection_of_obscured_area_boundaries {
public:
  //bool insert_collection(collection_of_obscured_area_boundaries const& other);
  bool insert_from_lines_collection(logical_node_lines_collection const& lines) {
    top_logical_node().insert_from_lines_collection(lines);
  }
private:
  implementation_node top_impnode;
  logical_node top_logical_node() {
    return logical_node(&top_impnode, 0, 0, 0, 0, 0);
  }
};

// If there get to be too many lines in this node [how many?], split into children.
/*
   How many is "too many"?
   Well, when is it a problem to have too many?
   The answer is that we iterate through the lines *exactly when we insert another polygon whose lines intersect this box*.
   For boxes that are sufficiently small compared to the length of the lines being inserted, the chance of a line hitting the box (if we make certain assumptions about how the line is "randomly" picked) is proportional to the "diameter" of the box.
   So the average-case complexity of "having N lines in this box" when inserting a polygon[1] is O(N * width of the box).
   So if we require N < c/(width of the box) (for some constant c), then the average-case complexity is O(c/(width of the box) * width of the box) = O(c) = O(1).
   
   We could restrict the number further, but having too many nested layers of children has a memory cost and is unproductive. Consider the case where there are M long lines "very very close" to each other; if the "too many" value doesn't exceed M soon enough, we will break down into many, many layers of children. But since the size of the box halves with each level, our rule allows twice as many lines at each level, so we only get down to level log2(M), so we only create about M boxes, no matter how big M is.
   
   [1] but it's not "per instance of inserting a polygon", because half the time when the (assumed to be very large compared with this box) polygon doesn't have an edge that hits this box, it instead *completely contains* this box and this box's info gets deleted. So the O(1) at the end of the paragraph ends up being a constant cost /for each time a box is created/ and (in the average case) is not proportional to the number of additional polygons that will be added.
   HOWEVER, TAKE NOTE: if you do a lot of lookups that *aren't* insertions, this logic doesn't apply. There are some worst-case O(number of lines inserted)-per-lookup cases that can't be fixed by adjusting the "too much" number.
*/
const coord_int_type lines_intolerance_constant = 4;
void logical_node::split_if_necessary() {
  assert(contents_type == MIXED_AS_LINES);
  
  if (lines_pointer_reference()->size() * lines_intolerance_constant > bounds_denominator) {
    set_contents_type(MIXED_AS_CHILDREN); // This automatically splits the lines we have up among our children.
    for (int which_child = 0; which_child < 4; ++which_child) {
      child(which_child).split_if_necessary();
    }
  }
}


/*bool collection_of_obscured_area_boundaries::insert_collection(collection_of_obscured_area_boundaries const& other) {
  return top_logical_node().insert_from_other_collection(other.top_logical_node())
}*/


// A utility function for set_contents_type.
void logical_node::retrieve_all_lines(logical_node_lines_collection& collector) {
  if (contents_type == MIXED_AS_LINES) {
    collector.copy_from_same_or_child(*lines_pointer_reference());
  }
  if (contents_type == MIXED_AS_CHILDREN) {
    for (int which_child = 0; which_child < 4; ++which_child) {
      child(which_child).retrieve_all_lines(collector);
    }
  }
}
// A utility function for set_contents_type.
void logical_node::clear_children() {
  // Clear the children, recursively. (The point of doing this is free anything behind
  //                        the bottom-level pointers.)
  for (int which_child = 0; which_child < 4; ++which_child) {
    // We need to overwrite them to ALL_CLEAR first even if we're going to delete the lower impnodes,
    // because the lower impnodes might have lower impnodes of their own.
    child(which_child).set_contents_type(ALL_CLEAR);
    
    if (subimpnode_level == max_subimpnode_level) {
      const uint32_t which_entry_at_bottom_subimpnode_level = which_entry_at_this_subimpnode_level * 4 + which_child;
      implementation_node*& next_impnode = reinterpret_cast<implementation_node*&>(impnode->lines_or_further_children[which_entry_at_bottom_subimpnode_level]);
      delete next_impnode;
      next_impnode = NULL;
    }
  }
}
// A utility function for set_contents_type.
void logical_node::create_children(uint8_t children_contents_type) {
  for (int which_child = 0; which_child < 4; ++which_child) {
    // We might have to create pointers for our new children before fetching them.
    if (subimpnode_level == max_subimpnode_level) {
      const uint32_t which_entry_at_bottom_subimpnode_level = which_entry_at_this_subimpnode_level * 4 + which_child;
      implementation_node*& next_impnode = reinterpret_cast<implementation_node*&>(impnode->lines_or_further_children[which_entry_at_bottom_subimpnode_level]);
      assert(next_impnode == NULL);
      next_impnode = new implementation_node;
    }
    
    child(which_child).set_contents_type(children_contents_type);
  }
}


void logical_node::set_contents_type(uint8_t new_type, bool keep_line_info) {
  // Note: Changing all-clear or all-blocked to MIXED_AS_CHILDREN or MIXED_AS_LINES
  // creates a situation where the type is marked as MIXED but the contents aren't actually
  // mixed; you need to immediately add lines / modify a child, or it'll mess up.
  
  if (new_type == contents_type) return;
  
  // The most complicated cases: Changing MIXED_AS_CHILDREN to MIXED_AS_LINES or vice versa.
  if (keep_line_info && (contents_type == MIXED_AS_CHILDREN) && (new_type == MIXED_AS_LINES)) {
    // We have to merge the lines containers,
    // and also the children, or children's lines containers, might be behind
    // the SAME POINTER that we're going to use for our own new lines container.
    logical_node_lines_collection* combined_lines = new logical_node_lines_collection;
    retrieve_all_lines(*combined_lines);
    clear_children();
    logical_node_lines_collection*& lpr = lines_pointer_reference();
    assert(lpr == NULL);
    lpr = combined_lines;
  }
  else if (keep_line_info && (contents_type == MIXED_AS_CHILDREN) && (new_type == MIXED_AS_LINES)) {
    logical_node_lines_collection*& lpr = lines_pointer_reference();
    logical_node_lines_collection* original_lines_ptr = lpr;
    lpr = NULL;
    create_children(ALL_CLEAR);
    for (int which_child = 0; which_child < 4; ++which_child) {
      logical_node c = child(which_child);
      
      logical_node_lines_collection*& clpr = c.lines_pointer_reference();
      assert(clpr == NULL);
      clpr = new logical_node_lines_collection(*original_lines_ptr, c.bounds());
    }
    delete original_lines_ptr;
  }
  else {
    // Either there's no line info to be kept, or we've been instructed not to keep it.
    if (contents_type == MIXED_AS_CHILDREN) {
      clear_children();
    }
    if (contents_type == MIXED_AS_LINES) {
      logical_node_lines_collection*& lpr = lines_pointer_reference();
      delete lpr;
      lpr = NULL;
    }
    
    if (new_type == MIXED_AS_LINES) {
      logical_node_lines_collection*& lpr = lines_pointer_reference();
      assert(lpr == NULL);
      lpr = new logical_node_lines_collection;
    }
    if (new_type == MIXED_AS_CHILDREN) {
      create_children((contents_type == MIXED_AS_LINES) ? ALL_CLEAR : contents_type);
    }
  }
  
  // Now set the bits.
  // If neither new_type nor contents_type was a MIXED type, we're just
  // changing from one ALL type to the other, which requires nothing else.
  
  set_contents_type_bits(new_type);
}
/*
void logical_node::overwrite_from_other_collection(logical_node const& other) {
  set_contents_type(other.contents_type, false);
  if (other.contents_type == MIXED_AS_CHILDREN) {
    for (int which_child = 0; which_child < 4; ++which_child) {
      child(which_child).overwrite_from_other_collection(other.child(which_child));
    }
  }
  else if (other.contents_type == MIXED_AS_LINES) {
    (*(lines_pointer_reference())) = (*(other.lines_pointer_reference()));
    Maybe something other than =?
  }
}

bool logical_node::insert_from_other_collection(logical_node const& other) {
  assert(x_lower_bound_numerator == other.x_lower_bound_numerator);
  assert(y_lower_bound_numerator == other.y_lower_bound_numerator);
  assert(bounds_denominator == other.bounds_denominator);
  
  if (other.contents_type == ALL_CLEAR) {
    return false;
  }
  // Now: other contains some blockage.
  else if (contents_type == ALL_BLOCKED) {
    return false;
  }
  // Now: other contains some blockage and we contain some clear area.
  else if (contents_type == ALL_CLEAR) {
    overwrite_from_other_collection(other);
    // If we were all clear, then wherever their blockage is, it was visible.
    return true;
  }
  // Now: We are mixed and the other contains some blockage.
  else if (other.contents_type == ALL_BLOCKED) {
    set_contents_type(ALL_BLOCKED);
    // If they were all blocked, then wherever our clear area was, you could see them through it.
    return true;
  }
  // Now: Both ourselves and the other are mixed.
  else if ((contents_type == MIXED_AS_CHILDREN) && (other.contents_type == MIXED_AS_CHILDREN)) {
    bool result = false;
    for (int which_child = 0; which_child < 4; ++which_child) { 
      result = child(which_child).insert_from_other_collection(other.child(which_child)) || result;
    }
    return result;
  }
  else if ((contents_type == MIXED_AS_LINES) && (other.contents_type == MIXED_AS_LINES)) {
    
  }
  // Now the awkward cases where one of us has children and the other has lines.
  else if ((contents_type == MIXED_AS_LINES) && (other.contents_type == MIXED_AS_CHILDREN)) {
  }
  else if ((contents_type == MIXED_AS_CHILDREN) && (other.contents_type == MIXED_AS_LINES)) {
  }
  else {
    assert(false);
  }
}
*/

bool logical_node::insert_from_lines_collection(logical_node_lines_collection const& lines) {
  assert(lines.bounds == bounds());
  
  if (lines.empty()) {
    // Nothing there at all, so nothing there is visible.
    return false;
  }
  // Now: "lines" contains some blockage.
  else if (contents_type == ALL_BLOCKED) {
    // We block everything, so nothing there is visible.
    return false;
  }
  // Now: "lines" contains some blockage and we contain some clear area.
  else if (lines.full()) {
    set_contents_type(ALL_BLOCKED);
    // If they were full, then wherever our clear area was, you could see them through it.
    return true;
  }
  // Now: "lines" is mixed and we contain some clear area.
  else if (contents_type == ALL_CLEAR) {
    set_contents_type(MIXED_AS_LINES);
    lines_pointer_reference()->copy_from_same_or_child(lines);
    split_if_necessary();
    // If we were all clear, then wherever their blockage is, it was visible.
    return true;
  }
  // Now: Both ourselves and the other are mixed.
  else if (contents_type == MIXED_AS_CHILDREN) {
    bool result = false;
    for (int which_child = 0; which_child < 4; ++which_child) {
      logical_node c = child(which_child);
      logical_node_lines_collection sub_lines(lines, c.bounds());
      if (c.insert_from_lines_collection(sub_lines)) result = true;
    }
    return result;
  }
  else if (contents_type == MIXED_AS_LINES) {
    const bool result = lines_pointer_reference()->combine(lines);
    split_if_necessary();
    return result;
  }
  else {
    assert(false);
  }
}



















