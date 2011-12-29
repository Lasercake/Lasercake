

#include "world.hpp"





/*location ztree_entry::loc()const { assert(worldblock_if_known); return location(locv, worldblock_if_known); }

void ztree_entry::set_bit(size_t idx) {
  assert(idx < bits_in_loc_coord * 3);
  interleaved_bits[idx / bits_in_loc_coord] |= (size_t(1) << (idx % bits_in_loc_coord));
}

void ztree_entry::set_bits() {
  interleaved_bits[0] = 0;
  interleaved_bits[1] = 0;
  interleaved_bits[2] = 0;
  for (size_t bit = 0; bit < bits_in_loc_coord; ++bit) {
    if (locv.x & (location_coordinate(1) << bit)) set_bit(3*bit + 0);
    if (locv.y & (location_coordinate(1) << bit)) set_bit(3*bit + 1);
    if (locv.z & (location_coordinate(1) << bit)) set_bit(3*bit + 2);
  }
}
ztree_entry::ztree_entry(location const& loc):
    locv(loc.coords()),
    worldblock_if_known(loc.wb),
    interleaved_bits() {
  set_bits();
}
ztree_entry::ztree_entry(vector3<location_coordinate> const& locv):
    locv(locv),
    worldblock_if_known(nullptr),
    interleaved_bits() {
  set_bits();
}
  
bool ztree_entry::operator==(ztree_entry const& other)const { return locv == other.locv; }
bool ztree_entry::operator<(ztree_entry const& other)const {
  if (interleaved_bits[2] < other.interleaved_bits[2]) return true;
  if (interleaved_bits[2] > other.interleaved_bits[2]) return false;
  if (interleaved_bits[1] < other.interleaved_bits[1]) return true;
  if (interleaved_bits[1] > other.interleaved_bits[1]) return false;
  return (interleaved_bits[0] < other.interleaved_bits[0]);
}*/

void world::collect_tiles_that_contain_anything_near(unordered_set<location> &results, axis_aligned_bounding_box bounds) {
  // TODO use something nicer than "int"
  /*const int total_width = std::max(std::max(bounds.size.x,bounds.size.y),bounds.size.z);*/
  ensure_space_exists(bounds);
  /*std::cerr << "Number of tiles that contain anything: " << tiles_that_contain_anything.size() << "\n";
  int exp = 0; while ((1 << exp) < total_width) ++exp;
  for (int x = 0; x < 2; ++x) { for (int y = 0; y < 2; ++y) { for (int z = 0; z < 2; ++z) {
    set<ztree_entry>::iterator lower_bound = tiles_that_contain_anything.lower_bound(
      ztree_entry(vector3<location_coordinate>(
        (bounds.min.x & ~((1 << exp) - 1)) + (x << exp),
        (bounds.min.y & ~((1 << exp) - 1)) + (y << exp),
        (bounds.min.z & ~((1 << exp) - 1)) + (z << exp)
      )
    ));
    set<ztree_entry>::iterator upper_bound = tiles_that_contain_anything.upper_bound(
      ztree_entry(vector3<location_coordinate>(
        (bounds.min.x | ((1 << exp) - 1)) + (x << exp),
        (bounds.min.y | ((1 << exp) - 1)) + (y << exp),
        (bounds.min.z | ((1 << exp) - 1)) + (z << exp)
      )
    ));
    for(set<ztree_entry>::iterator i = lower_bound; i != upper_bound; ++i) {
      const location loc = i->loc();
      if (bounds.contains(loc.coords()))
        results.insert(loc);
    }
  }}}*/
  space_with_fast_lookup_of_everything_overlapping_localized_area<location, 32, 3>::bounding_box b;
  b.min[0] = bounds.min.x;
  b.min[1] = bounds.min.y;
  b.min[2] = bounds.min.z;
  b.size[0] = bounds.size.x;
  b.size[1] = bounds.size.y;
  b.size[2] = bounds.size.z;
  tiles_that_contain_anything.get_objects_overlapping(results, b);
}

