
#include "world.hpp"

void ztree_entry::set_bit(size_t idx) {
  assert(idx < bits_in_loc_coord * 3);
  interleaved_bits[idx / bits_in_loc_coord] |= (size_t(1) << (idx % bits_in_loc_coord));
}

ztree_entry::ztree_entry(location const& loc_):loc_(loc_),interleaved_bits() {
  interleaved_bits[0] = 0;
  interleaved_bits[1] = 0;
  interleaved_bits[2] = 0;
  for (size_t bit = 0; bit < bits_in_loc_coord; ++bit) {
    if (loc_.coords().x & (location_coordinate(1) << bit)) set_bit(3*bit + 0);
    if (loc_.coords().y & (location_coordinate(1) << bit)) set_bit(3*bit + 1);
    if (loc_.coords().z & (location_coordinate(1) << bit)) set_bit(3*bit + 2);
  }
}
  
bool ztree_entry::operator==(ztree_entry const& other)const { return loc_.coords() == other.loc_.coords(); }
bool ztree_entry::operator<(ztree_entry const& other)const {
  if (interleaved_bits[2] < other.interleaved_bits[2]) return true;
  if (interleaved_bits[2] > other.interleaved_bits[2]) return false;
  if (interleaved_bits[1] < other.interleaved_bits[1]) return true;
  if (interleaved_bits[1] > other.interleaved_bits[1]) return false;
  return (interleaved_bits[0] < other.interleaved_bits[0]);
}

void world::collect_tiles_that_contain_anything_near(unordered_set<location> &results, location center, int radius) {
  // TODO use something nicer than "int"
  const int total_width = 2*radius + 1;
  ensure_space_exists(axis_aligned_bounding_box{center.coords() - vector3<location_coordinate>(radius,radius,radius), vector3<location_coordinate>(total_width,total_width,total_width) });
  std::cerr << "Number of tiles that contain anything: " << tiles_that_contain_anything.size() << "\n";
  int exp = 0; while ((1 << exp) < total_width) ++exp;
  const int x_shift = (center.coords().x & ((1 << exp) - 1)) < (1 << (exp - 1)) ? -1 : 0;
  const int y_shift = (center.coords().y & ((1 << exp) - 1)) < (1 << (exp - 1)) ? -1 : 0;
  const int z_shift = (center.coords().z & ((1 << exp) - 1)) < (1 << (exp - 1)) ? -1 : 0;
  for (int x = 0; x < 2; ++x) { for (int y = 0; y < 2; ++y) { for (int z = 0; z < 2; ++z) {
    set<ztree_entry>::iterator lower_bound = tiles_that_contain_anything.lower_bound(
      ztree_entry(make_location(vector3<location_coordinate>(
        (center.coords().x & ~((1 << exp) - 1)) + ((x+x_shift) * (1 << exp)),
        (center.coords().y & ~((1 << exp) - 1)) + ((y+y_shift) * (1 << exp)),
        (center.coords().z & ~((1 << exp) - 1)) + ((z+z_shift) * (1 << exp))
      ))
    ));
    set<ztree_entry>::iterator upper_bound = tiles_that_contain_anything.upper_bound(
      ztree_entry(make_location(vector3<location_coordinate>(
        (center.coords().x | ((1 << exp) - 1)) + ((x+x_shift) * (1 << exp)),
        (center.coords().y | ((1 << exp) - 1)) + ((y+y_shift) * (1 << exp)),
        (center.coords().z | ((1 << exp) - 1)) + ((z+z_shift) * (1 << exp))
      ))
    ));
    for(set<ztree_entry>::iterator i = lower_bound; i != upper_bound; ++i) {
      results.insert(i->loc());
    }
  }}}
}

