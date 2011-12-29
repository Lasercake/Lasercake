

#include "world.hpp"


#include <boost/integer.hpp>

typedef int num_bits_type;
typedef size_t num_coordinates_type;


// ObjectIdentifier needs hash and == and to be freely copiable. So, ints will do, pointers will do...
// I imagine that Coordinate can be uint32_t
template<typename ObjectIdentifier, num_bits_type coordinate_bits, num_coordinates_type num_dimensions>
class space_with_fast_lookup_of_everything_overlapping_localized_area {
  static_assert(num_dimensions >= 0);
  static_assert(coordinate_bits >= 0);
  
  typedef boost::uint_t<coordinate_bits>::fast Coordinate;
  struct bounding_box {
    std::array<Coordinate, num_dimensions> min, size;
    bool overlaps(bounding_box other)const;
  };
  
  ztree():objects_tree(nullptr){}
  
  class iterator : public boost::iterator_facade<
      iterator,
     /* TODO ITERATOR INFO TYPE */,
      boost::forward_traversal_tag,
      ObjectIdentifier> {
  public:
    bool equal(iterator other)const;
    void increment();
    ObjectIdentifier dereference()const;
    explicit iterator(/* TODO ITERATOR INFO TYPE */);
  private:
  };
  
private:
  static const num_bits_type total_bits = coordinate_bits * num_dimensions;
  
  unordered_multimap<ObjectIdentifier, bounding_box> bboxes_by_object;
  ztree_node* objects_tree;
  
  struct zbox {
    // We ensure that every bit except the ones specifically supposed to be on is off.
    std::array<Coordinate, num_dimensions> interleaved_bits;
    num_bits_type num_low_bits_ignored;
    
    zbox():num_low_bits_ignored(total_bits){ for (num_coordinates_type i = 0; i < num_dimensions; ++i) interleaved_bits[i] = 0; }
    
    bool subsumes(zbox const& other)const {
      if (other.num_low_bits_ignored > num_low_bits_ignored) return false;
      for (num_coordinates_type i = num_low_bits_ignored / coordinate_bits; i < num_dimensions; ++i) {
        Coordinate mask = ~0;
        if (i == num_low_bits_ignored / coordinate_bits) {
          mask &= ~((num_bits_type(1) << (num_low_bits_ignored % coordinate_bits)) - 1);
        }
        if ((interleaved_bits[i] & mask) != (other.interleaved_bits[i] & mask)) return false;
      }
      return true;
    }
    bool get_bit(num_bits_type bit) {
      return interleaved_bits[bit / coordinate_bits] & ~(Coordinate(1) << (bit % coordinate_bits));
    }
    /*zbox crop(new_num_low_bits_ignored) {
      assert(new_num_low_bits_ignored >= num_low_bits_ignored);
      zbox new_box;
      new_box.num_low_bits_ignored = new_num_low_bits_ignored;
      for (num_coordinates_type i = new_num_low_bits_ignored / coordinate_bits; i < num_dimensions; ++i) {
        Coordinate mask = ~0;
        if (i == new_num_low_bits_ignored / coordinate_bits) {
          mask &= ~((Coordinate(1) << (new_num_low_bits_ignored % coordinate_bits)) - 1);
        }
        new_box.interleaved_bits[i] = interleaved_bits[i] & mask;
      }
      return new_box;
    }*/
  };
  
  num_bits_type idx_of_highest_bit(Coordinate i) {
    int upper_bound = coordinate_bits;
    int lower_bound = -1;
    while(true) {
      int halfway_bit_idx = (upper_bound + lower_bound) / 2;
      if (halfway_bit_idx == lower_bound) return lower_bound;
      
      if (i & ~((Coordinate(1) << halfway_bit_idx) - 1)) lower_bound = halfway_bit_idx;
      else                                               upper_bound = halfway_bit_idx;
    }
  }
  
  struct ztree_node {
    zbox here;
    ztree_node *child0;
    ztree_node *child1;
    unordered_set<ObjectIdentifier> objects_here;
  };
  
  zbox smallest_joint_parent(zbox zb1, zbox zb2) {
    zbox new_box;
    for (num_coordinates_type i = num_dimensions; i >= 0; --i) {
      const num_bits_type highest_bit_idx = idx_of_highest_bit(zb1.interleaved_bits[i] ^ zb2.interleaved_bits[i]);
      assert((zb1.interleaved_bits[i] & ~((1 << (highest_bit_idx + 1)) - 1)) == (zb2.interleaved_bits[i] & ~((1 << (highest_bit_idx + 1)) - 1)))
      new_box.interleaved_bits[i] = zb1.interleaved_bits[i] & ~((1 << (highest_bit_idx + 1)) - 1);
      if (highest_bit_idx > 0) {
        new_box.num_low_bits_ignored = highest_bit_idx + i * coordinate_bits;
        return new_box;
      }
    }
    new_box.num_low_bits_ignored = 0;
    return new_box;
  }
  
  zbox box_from_coords(std::array<Coordinate, num_dimensions> const& coords, num_bits_type num_low_bits_ignored) {
    zbox result;
    result.num_low_bits_ignored = num_low_bits_ignored
      box.num_low_bits_ignored = exp * num_dimensions;
    for (num_bits_type bit_within_interleaved_bits = num_low_bits_ignored;
                       bit_within_interleaved_bits < total_bits;
                     ++bit_within_interleaved_bits) {
      const num_bits_type bit_idx_within_coordinates = bit_within_interleaved_bits / num_dimensions;
      const num_dimensions_type which_coordinate     = bit_within_interleaved_bits % num_dimensions;
      const num_bits_type interleaved_bit_array_idx  = bit_within_interleaved_bits / coordinate_bits;
      const num_bits_type interleaved_bit_local_idx  = bit_within_interleaved_bits % coordinate_bits;
      box.interleaved_bits[interleaved_bit_array_idx] |= ((coords[which_coordinate] >> bit_idx_within_coordinates) & 1) << interleaved_bit_local_idx;
    }
    return result;
  }
  
  void insert_box((ztree_node*)& tree, ObjectIdentifier obj, zbox box) {
    if (!tree) {
      tree = new ztree_node {
        box,
        nullptr,
        nullptr,
        unordered_set<ObjectIdentifier>()
      };
      tree->objects_here.insert(obj);
    }
    else {
      if (tree->here.subsumes(box)) {
        if (box.num_low_bits_ignored == tree->here.num_low_bits_ignored) {
          tree->objects_here.insert(obj);
        }
        else {
          if (box.get_bit(tree->here.num_low_bits_ignored - 1)) insert_box(tree->child1, obj, box);
          else                                                  insert_box(tree->child0, obj, box);
        }
      }
      else {
        const ztree_node *new_tree = new ztree_node {
          smallest_joint_parent(tree->here, box),
          nullptr,
          nullptr,
          unordered_set<ObjectIdentifier>()
        };
        if (tree->here.get_bit(box.num_low_bits_ignored - 1)) new_tree->child1 = tree;
        else                                                  new_tree->child0 = tree;
      
        tree = new_tree;
        insert_box(tree, obj, box);
      }
    }
  }
  
  void delete_object((ztree_node*)& tree, ObjectIdentifier obj, bounding_box bbox) {
    if (!tree) return;
    if (tree->here.intersects(bbox)) {
      tree->objects_here.erase(obj);
      delete_object(tree->child0, obj, bbox);
      delete_object(tree->child1, obj, bbox);
      
      if (tree->objects_here.empty()) {
        if (tree->child0) {
          if (!tree->child1) {
            tree = child0;
          }
        }
        else tree = child1; // which could be null
      }
    }
  }
public:

  void insert(ObjectIdentifier id, bounding_box bbox) {
    bboxes_by_object.insert(make_pair(id, bbox));
    Coordinate max_dim = bbox[0];
    for (num_coordinates_type i = 1; i < num_dimensions; ++i) {
      if (bbox[i] > max_dim) max_dim = bbox.size[i];
    }
    int exp = 0; while ((Coordinate(1) << exp) < total_width) ++exp;
    for (int i = 0; i < (1 << num_dimensions); ++i) {
      std::array<Coordinate, num_dimensions> coords == bbox[min];
      for (num_coordinates_type j = 0; j < num_dimensions; ++j) {
        if (i & (1<<j)) coords[j] += (Coordinate(1) << exp);
      }
      insert_box(objects_tree, id, box_from_coords(coords, exp * num_dimensions));
    }
  }
  void erase(ObjectIdentifier id) {
    bbox_iter = bboxes_by_object.find(id);
    assert(bbox_iter != bboxes_by_object.end());
    delete_object(objects_tree, id, (*bbox_iter));
    bboxes_by_object.erase(id);
  }
  
  // TODO document iterator validity lifetime
  boost::iterator_range<iterator> objects_overlapping(bounding_box bbox)const;
};



/*

If there's one zbox in the tree [call it Z]

tree = ztree_node {
  Z
  nullptr
  nullptr
}

Two zboxes that differ at bit B:
child0 of a node with B ignored bits is the child whose Bth bit is 0.
tree = ztree_node {
  Z1/Z2, with B ignored bits
  ptr to ztree_node {
    Z1
    nullptr
    nullptr
  }
  ptr to ztree_node {
    Z2
    nullptr
    nullptr
  }
}

*/






location ztree_entry::loc()const { assert(worldblock_if_known); return location(locv, worldblock_if_known); }

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
}

void world::collect_tiles_that_contain_anything_near(unordered_set<location> &results, axis_aligned_bounding_box bounds) {
  // TODO use something nicer than "int"
  const int total_width = std::max(std::max(bounds.size.x,bounds.size.y),bounds.size.z);
  ensure_space_exists(bounds);
  std::cerr << "Number of tiles that contain anything: " << tiles_that_contain_anything.size() << "\n";
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
  }}}
}

