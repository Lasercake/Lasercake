
// The largest logical node in the collection is always the unit square [0,1] \cross [0,1].

typedef lasercake_int<int64_t> coord_int_type;
typedef non_normalized_rational<coord_int_type> coord_type;

class collection_of_obscured_area_boundaries {
public:
  bool insert_collection(collection_of_obscured_area_boundaries const& other);
private:
  implementation_node top_impnode;
  logical_node top_logical_node() {
    return logical_node(top_impnode, 0, 0, 0, 0, 1);
  }
};

const uint8_t ALL_CLEAR = 0x0;
const uint8_t ALL_BLOCKED = 0x1;
const uint8_t MIXED_AS_LINES = 0x2;
const uint8_t MIXED_AS_CHILDREN = 0x3;

int total_entries_at_subimpnode_level(int level) {
  if (level == 0) return 1;
  return 4 * total_entries_at_subimpnode_level(level - 1);
}

int total_entries_above_subimpnode_level(int level) {
  if (level == 0) return 0;
  return total_entries_at_subimpnode_level(level - 1) * total_entries_above_subimpnode_level(level - 1);
}

const int total_subimpnode_levels = 5;
const int max_subimpnode_level = total_levels - 1;

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
  void *lines_or_further_children[total_entries_at_bottom_level];
};

struct logical_node {
  implementation_node *impnode;
  uint32_t which_entry_at_this_subimpnode_level;
  uint8_t subimpnode_level;
  
  uint8_t contents_type;
  uint8_t which_bits_in_byte;
  uint32_t which_byte_in_impnode;
  // The upper bound numerators are always 1 + these
  coord_int_type x_lower_bound_numerator;
  coord_int_type y_lower_bound_numerator;
  coord_int_type bounds_denominator; // the same for both X and Y, and always a power of 2
  
  logical_node(i,l,w,nx,ny,d):impnode(i),subimpnode_level(l),which_entry_at_this_subimpnode_level(w),x_lower_bound_numerator(nx),y_lower_bound_numerator(ny),bounds_denominator(d){
    assert(subimpnode_level <= max_subimpnode_level);
    assert(subimpnode_level >= 0);
    if (subimpnode_level == 0) {
      contents_type = impnode->top_entry;
    }
    else {
      which_byte_in_impnode = total_entries_above_subimpnode_level(subimpnode_level) + (which_entry_at_this_subimpnode_level/4);
      which_bits_in_byte = 2*(which_entry_at_this_subimpnode_level % 4));
      assert(which_byte_in_impnode >= 0);
      assert(which_byte_in_impnode < total_entry_bytes);
      contents_type = (impnode->entries[which_byte_in_impnode] >> which_bits_in_byte) & 0x3;
    }
  }
  
  void set_contents_type_bits(uint8_t type) {
    contents_type = type;
    impnode->entries[which_byte_in_impnode] = (impnode->entries[which_byte_in_impnode] & (~(0x3 << which_bits_in_byte))) | (type << which_bits_in_byte);
  }
  
  COLLECTION OF LINES*& lines_pointer_reference()const {
    assert(contents_type == MIXED_AS_LINES);
    
    const uint32_t which_entry_at_bottom_level = which_entry_at_this_subimpnode_level << (total_subimpnode_levels - subimpnode_level);
    
    return static_cast<COLLECTION OF LINES*>(impnode->lines_or_further_children[which_entry_at_bottom_level]);
  }
  
  logical_node child(int which_child)const {
    assert(contents_type == MIXED_AS_CHILDREN);
    
    const uint32_t which_entry_at_next_subimpnode_level = which_entry_at_this_subimpnode_level * 4 + which_child;
    new_denom = bounds_denominator * 2;
    new_xnum = x_lower_bound_numerator * 2 + (which_child & 0x1);
    new_ynum = y_lower_bound_numerator * 2 + ((which_child & 0x2) >> 1);
    
    if (subimpnode_level == max_subimpnode_level) {
      assert(which_entry_at_next_subimpnode_level >= 0);
      assert(which_entry_at_next_subimpnode_level < total_entries_at_bottom_level);
      assert(impnode->lines_or_further_children[which_entry_at_next_subimpnode_level] != NULL);
      return logical_node(static_cast<implementation_node*>(impnode->lines_or_further_children[which_entry_at_next_subimpnode_level]), 0, 0, new_xnum, new_ynum, new_denom);
    }
    else {
      return logical_node(impnode, subimpnode_level + 1, which_entry_at_next_subimpnode_level, new_xnum, new_ynum, new_denom);
    }
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
void logical_node::split_if_necessary() {
  assert(contents_type == MIXED_AS_LINES);
  
  if ((number of lines in us) * (width of us) > c) {
    split into four children, then recursively call this on the children
  }
}


bool collection_of_obscured_area_boundaries::insert_collection(collection_of_obscured_area_boundaries const& other) {
  return top_logical_node().insert_from_other_collection(other.top_logical_node())
}


// A utility function for set_contents_type.
void logical_node::retrieve_all_lines(COLLECTION OF LINES& collector) {
  if (contents_type == MIXED_AS_LINES) {
    collector.merge_from(*lines_pointer_reference());
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
      const uint32_t which_entry_at_bottom_level = which_entry_at_this_subimpnode_level * 4 + which_child;
      implementation_node*& next_impnode = static_cast<implementation_node*&>(impnode->lines_or_further_children[which_entry_at_bottom_level]);
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
      const uint32_t which_entry_at_bottom_level = which_entry_at_this_subimpnode_level * 4 + which_child;
      implementation_node*& next_impnode = static_cast<implementation_node*&>(impnode->lines_or_further_children[which_entry_at_bottom_level]);
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
    COLLECTION OF LINES combined_lines;
    retrieve_all_lines(combined_lines);
    clear_children();
    COLLECTION OF LINES*& lpr = lines_pointer_reference();
    assert(lpr == NULL);
    lpr = new COLLECTION OF LINES(combined_lines);
    or maybe we should steal from combined_lines if we're using lists and that's faster.
  }
  else if (keep_line_info && (contents_type == MIXED_AS_CHILDREN) && (new_type == MIXED_AS_LINES))
    if (!preserve_line_info) {
      set_contents_type(ALL_CLEAR, true);
      set_contents_type(new_type, true);
    }
    COLLECTION OF LINES*& lpr = lines_pointer_reference();
    COLLECTION OF LINES original_lines(*lpr);
    or maybe we should steal from *lpr if we're using lists and that's faster.
    delete lpr;
    lpr = NULL;
    create_children(ALL_CLEAR);
    for (int which_child = 0; which_child < 4; ++which_child) {
      logical_node c = child(which_child);
      
      COLLECTION OF LINES*& clpr = c.lines_pointer_reference();
      assert(clpr == NULL);
      clpr = new COLLECTION OF LINES;
      clpr->copy_lines_intersecting_node(original_lines, c);
    }
  }
  else {
    // Either there's no line info to be kept, or we've been instructed not to keep it.
    if (contents_type == MIXED_AS_CHILDREN) {
      clear_children();
    }
    if (contents_type == MIXED_AS_LINES) {
      COLLECTION OF LINES*& lpr = lines_pointer_reference();
      delete lpr;
      lpr = NULL;
    }
    
    if (new_type == MIXED_AS_LINES) {
      COLLECTION OF LINES*& lpr = lines_pointer_reference();
      assert(lpr == NULL);
      lpr = new COLLECTION OF LINES;
    }
    if (new_type == MIXED_AS_CHILDREN) {
      create_children(contents_type);
    }
  }
  
  // Now set the bits.
  // If neither new_type nor contents_type was a MIXED type, we're just
  // changing from one ALL type to the other, which requires nothing else.
  
  set_contents_type_bits(new_type);
}

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

// assumes b is already "all clear" or "lines", inserts lines, returns whether the inserted lines overlapped any clear area
bool insert_lines ([collection of relevant lines] p, [treebox reference] b) {


  split_if_necessary(b);
}

bool insert_poly ([collection of possibly relevant lines] p, [treebox reference] b) {
  if (b is "all blocked") return false; // move this to the call for speed
  [collection of lines intersecting b] p2;
  for each line in p {
    if (that line intersects b) add that line to p2
  }
  if (p2 is empty) {
    // either we completely contain it or we're completely outside it
    if (p and b are separate) return false;
    else {
      // p2 completely contains b
      set b to "all blocked" (delete all data below b?)
      return true;
    }
  }
  if (b is "all clear") {
    set b to "lines" and insert all of p2 lines (cropped);
    return true;
  }
  if (b is "lines") {
    make a collector of crossing points for each b-line and each p2 line
    return (we crossed to the light side of any lines, or there were no crossings and we were on the light side the whole time);
  }
  if (b is "children") {
    bool result0 = insert_poly(p2, b.child0);
    bool result1 = insert_poly(p2, b.child1);
    bool result2 = insert_poly(p2, b.child2);
    bool result3 = insert_poly(p2, b.child3);
    return (result0 || result1 || result2 || result3);
  }
}



struct axis_aligned_line {
  axis_aligned_line(coord_type y,coord_type x1,coord_type x2,bool b):y(y),x1(x1),x2(x2),is_obscured_above(b){}
  // The location of the line in one dimension, and its bounds in the other dimension.
  coord_type l;
  coord_type b1;
  coord_type b2;
  // "beyond" being upwards in the dimension that crosses the line
  bool is_obscured_beyond;
}
// Assuming that they're perpendicular...
inline bool intersects(axis_aligned_line l1, axis_aligned_line l2) {
  return (l1.b1 <= l2.l) && (l2.l <= l1.b2)
      && (l2.b1 <= l1.l) && (l1.l <= l2.b2);
}
struct x_or_y_boundary {
  x_or_y_boundary(coord_type b, bool i):b(b),is_x_boundary(i){}
  coord_type b;
  bool is_x_boundary;
}
struct perspective_line {
  coord_type slope;
  x_or_y_boundary start;
  x_or_y_boundary end;
  bool is_obscured_to_the_right;
  bool is_obscured_above() { return (slope < 0) == is_obscured_to_the_right; }
  coord_type y_intercept(coord_type x) {
    // slope = y / x;
    // y = x * slope
    return x * slope;
  }
  coord_type x_intercept(coord_type y) {
    // slope = y / x;
    // x = y / slope
    return y / slope;
  }
  bool point_is_within_end_boundaries(coord_type x, coord_type y){
    if (start.is_x_boundary && (x < start.b)) return false;
    if (  end.is_x_boundary && (x >   end.b)) return false;
    if (slope > 0) {
      if ((!start.is_x_boundary) && (y < start.b)) return false;
      if ((!  end.is_x_boundary) && (y >   end.b)) return false;
    }
    else {
      if ((!start.is_x_boundary) && (y > start.b)) return false;
      if ((!  end.is_x_boundary) && (y <   end.b)) return false;
    }
    return true;
  }
}
struct pline_tpt_loc {
  pline_tpt_loc(coord_type b, bool i, coord_type m):b(b,i),slope(m){}
  x_or_y_boundary b;
  coord_type slope;
  bool operator<(pline_tpt_loc const& other)const {
    if (b.is_x_boundary == other.b.is_x_boundary) {
      return (b.b < other.b.b) == (b.is_x_boundary || (slope > 0));
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
}

// "transition point"
template<typename BoundaryLocType = coord_type> struct tpt {
  tpt(BoundaryLocType l, bool b):loc(l),is_obscured_beyond(b){}
  BoundaryLocType loc;
  bool is_obscured_beyond;
  bool operator<(tpt const& other)const{ return loc < other.loc; }
}

template<typename BoundaryLocType = coord_type> struct obscured_areas_tracker_1d {
  obscured_areas_tracker_1d():is_obscured_initially(false);
  bool is_obscured_initially;
  std::multiset<tpt<BoundaryLocType>> tpts;
  
  inline void insert(tpt t) {
    tpts.insert(t);
  }
  bool point_is_obscured(BoundaryLocType point) {
    if (tpts.empty()) return is_obscured_initially;
    auto b = tpts.lower_bound(point);
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
    is_obscured_at_bottom_of_left_side = is_obscured_at_bottom_of_left_side | other.is_obscured_at_bottom_of_left_side;
    
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
}

bool handle_aaline_vs_aaline(axis_aligned_line l1, obscured_areas_tracker_1d& l1_info, axis_aligned_line l2, obscured_areas_tracker_1d& l2_info) {
  if (intersects(l1, l2)) {
    l1_info.insert(tpt(l2.l, l2.is_obscured_beyond));
    l2_info.insert(tpt(l1.l, l1.is_obscured_beyond));
    return true;
  }
  else {
    return false;
  }
}

bool handle_aaline_vs_pline(bool aaline_is_horizontal, axis_aligned_line aaline, obscured_areas_tracker_1d& aaline_info, perspective_line pline, obscured_areas_tracker_1d<pline_tpt_loc>& pline_info) {
  coord_type b = (aaline_is_horizontal ? pline.x_intercept(aaline.l) : pline.y_intercept(aaline.l));
  if ((aaline.b1 < b) && (b < aaline.b2) && pline.point_is_within_end_boundaries((aaline_is_horizontal ? b : aaline.l), (aaline_is_horizontal ? aaline.l : b))) {
    aaline_info.insert(tpt(x, (aaline_is_horizontal ? pline.is_obscured_to_the_right : pline.is_obscured_above()));
    pline_info.insert(tpt<pline_tpt_loc>(
        pline_tpt_loc(aaline.l, !aaline_is_horizontal, pline.slope),
        aaline.is_obscured_beyond == ((opline.slope > 0) || !aaline_is_horizontal)
      ));
    return true;
  }
  else {
    return false;
  }
}

void slice_marked_aalines(bool aalines_are_horizontal, bool src_aalines_is_our_own_collection, std::vector<axis_aligned_line>& src_aalines, std::vector<axis_aligned_line>& dst_aalines, std::vector<obscured_areas_tracker_1d> const& tpts, std::vector<size_t> completely_obscured_or_completely_visible_list) {
  for (size_t i = 0; i < tpts.size(); ++i) {
    auto const& switch_points = tpts[i].tpts;
    axis_aligned_line& aaline = src_aalines[i];
    if (switch_points.empty()) {
      // Check if we're completely obscured
      if (other.point_is_obscured(aalines_are_horizontal ? aaline.b1 : aaline.l, aalines_are_horizontal ? aaline.l : aaline.b1) == src_aalines_is_our_own_collection) {
        completely_obscured_or_completely_visible_list.push_back(i);
      }
    }
    else {
      // Replace ourselves with the first still-visible piece of ourselves.
      auto t = switch_points.begin();
      assert(t != switch_points.end());
      coord_type original_end = aaline.b2;
      if (t->is_obscured_beyond) {
        aaline.b1 = t->loc;
        ++t;
        This assertion might fail if rounding error stuff happens. What to do about that?
        assert(!t->is_obscured_beyond);
        if (t != switch_points.end()) {
          aaline.b2 = t->loc;
        }
      }
      else {
        aaline.b2 = t->loc;
      }
      if (!src_aalines_is_our_own_collection) dst_aalines.push_back(aaline);
      
      // Now insert the rest of the still-visible pieces of ourselves at the end.
      ++t;
      while(t != switch_points.end()) {
        This assertion might fail if rounding error stuff happens. What to do about that?
        assert(!t->is_obscured_beyond);
        coord_type startb = t->loc;
        ++t;
        if (t == switch_points.end()) {
          dst_aalines.push_back(axis_aligned_line(aaline.l, startb, original_end, aaline.is_obscured_beyond));
        }
        else {
          dst_aalines.push_back(axis_aligned_line(aaline.l, startb, t->loc, aaline.is_obscured_beyond));
          ++t;
        }
      }
    }
  }
}

struct logical_node_lines_collection {
  obscured_areas_tracker_1d<coord_type> left_side;
  
  std::vector<axis_aligned_line> vlines;
  std::vector<axis_aligned_line> hlines;
  std::vector<perspective_line> plines;
  
  bool point_is_obscured(coord_type x, coord_type y) {
    // Effectively: Draw a horizontal line back to the left side. If it intersects any lines,
    // then the closest intersection dictates whether you're obscured or not.
    // Otherwise, check the left_side structure to see whether you're obscured.
    bool found_any_lines;
    coord_type best_distance;
    bool best_obscuredness;
    for (auto const& vline : vlines) {
      if (vline.l <= x && vline.b1 <= y && vline.b2 >= y) {
        coord_type distance = x - vline.l;
        if ((!found_any_lines) || (distance < best_distance)) {
          found_any_lines = true;
          best_distance = distance;
          best_obscuredness = vline.is_obscured_to_the_right;
        }
      }
    }
    for (auto const& pline : plines) {
      coord_type pline_x = pline.x_intercept(y);
      if ((pline_x <= x) && pline.point_is_within_end_boundaries(pline_x, y)) {
        coord_type distance = x - pline_x;
        if ((!found_any_lines) || (distance < best_distance)) {
          found_any_lines = true;
          best_distance = distance;
          best_obscuredness = pline.is_obscured_to_the_right;
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
  
  needs better name:
  bool combine(logical_node_lines_collection& other) {
    bool result = false;
    
    // ======== Combine the lines info. ==========
    // First, collect all overlaps.
    std::vector<obscured_areas_tracker_1d>                 hlines_tpts(      hlines.size());
    std::vector<obscured_areas_tracker_1d>                ohlines_tpts(other.hlines.size());
    std::vector<obscured_areas_tracker_1d>                 vlines_tpts(      vlines.size());
    std::vector<obscured_areas_tracker_1d>                ovlines_tpts(other.vlines.size());
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
    
    // Then slice the lines up based on the overlaps.
    // For lines that don't have overlaps, note whether or not they're completely obscured.
    // They can only be completely obscured *by the other collection*
    //  (since each line is completely necessary to its own collection)
    // and since we haven't modified either collection yet, we can just ask the other collection
    // whether a point on that line is visible.
    slice_marked_aalines(true , true ,       hlines, hlines,  hlines_tpts, completely_obscured_hlines);
    slice_marked_aalines(false, true ,       vlines, vlines,  vlines_tpts, completely_obscured_vlines);
    slice_marked_aalines(true , false, other.hlines, hlines, ohlines_tpts, completely_visible_ohlines);
    slice_marked_aalines(false, false, other.vlines, vlines, ovlines_tpts, completely_visible_ovlines);
    
    // When you're done, purge the lines that were completely obscured.
    for (int i = completely_obscured_hlines.size() - 1; i >= 0 ; --i) {
      hlines[completely_obscured_hlines[i]] = hlines.back(); hlines.pop_back();
    }
    for (int i = completely_obscured_vlines.size() - 1; i >= 0 ; --i) {
      vlines[completely_obscured_vlines[i]] = vlines.back(); vlines.pop_back();
    }
    for (int i = completely_obscured_plines.size() - 1; i >= 0 ; --i) {
      plines[completely_obscured_plines[i]] = plines.back(); plines.pop_back();
    }
    // And copy over the completely-visible lines from the other list.
    for (size_t i : completely_visible_ohlines) {
      hlines.push_back(other.hlines[i]);
    }
    for (size_t i : completely_visible_ovlines) {
      vlines.push_back(other.vlines[i]);
    }
    for (size_t i : completely_visible_oplines) {
      plines.push_back(other.plines[i]);
    }
    
    // ======== Combine the left-side info. ==========
    left_side.combine(other.left_side);
    
    return result;
  }
}



















