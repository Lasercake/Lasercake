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

#ifndef LASERCAKE_BBOX_COLLISION_DETECTOR_HPP__
#define LASERCAKE_BBOX_COLLISION_DETECTOR_HPP__

#include <boost/integer.hpp>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <cassert>
#include <cstdlib>
#include <boost/scoped_ptr.hpp>

using std::unordered_map;
using std::unordered_set;

typedef size_t num_bits_type;
typedef size_t num_coordinates_type;

// ObjectIdentifier needs hash and == and to be freely copiable. So, ints will do, pointers will do...
// CoordinateBits should usually be 32 or 64. I don't know if it works for other values.
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class bbox_collision_detector {
  static_assert(NumDimensions >= 0, "You can't make a space with negative dimensions!");
  static_assert(CoordinateBits >= 0, "You can't have an int type with negative bits!");
  friend class zbox_tester;

public:
  typedef typename boost::uint_t<CoordinateBits>::fast Coordinate;
  struct bounding_box {
    std::array<Coordinate, NumDimensions> min, size;
    bool overlaps(bounding_box const& other)const {
      for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
        // this should correctly handle zboxes' "size=0" when all bits are ignored
        if (other.min[i] + (other.size[i] - 1) <       min[i]) return false;
        if (      min[i] + (      size[i] - 1) < other.min[i]) return false;
      }
      return true;
    }
  };
  
  bbox_collision_detector():objects_tree(nullptr){}
  bbox_collision_detector(bbox_collision_detector const& other) { *this = other; }
  bbox_collision_detector& operator=(bbox_collision_detector const& other) {
    bboxes_by_object = other.bboxes_by_object;
    if(other.objects_tree) objects_tree.reset(new ztree_node(*other.objects_tree));
    return *this;
  }
  
private:
  static const num_bits_type total_bits = CoordinateBits * NumDimensions;
  
  static Coordinate safe_left_shift_one(num_bits_type shift) {
    if (shift >= 8*sizeof(Coordinate)) return 0;
    return Coordinate(1) << shift; // TODO address the fact that this could put bits higher than the appropriate amount if CoordinateBits isn't the number of bits of the type
  }
  
  struct zbox {
    // We ensure that every bit except the ones specifically supposed to be on is off.
    std::array<Coordinate, NumDimensions> coords;
    std::array<Coordinate, NumDimensions> interleaved_bits;
    num_bits_type num_low_bits_ignored;
    
    zbox():num_low_bits_ignored(total_bits){ for (num_coordinates_type i = 0; i < NumDimensions; ++i) interleaved_bits[i] = 0; }
    
    bool subsumes(zbox const& other)const {
      if (other.num_low_bits_ignored > num_low_bits_ignored) return false;
      for (num_coordinates_type i = num_low_bits_ignored / CoordinateBits; i < NumDimensions; ++i) {
        Coordinate mask = ~Coordinate(0);
        if (i == num_low_bits_ignored / CoordinateBits) {
          mask &= ~(safe_left_shift_one(num_low_bits_ignored % CoordinateBits) - 1);
        }
        if ((interleaved_bits[i] & mask) != (other.interleaved_bits[i] & mask)) return false;
      }
      return true;
    }
    bool get_bit(num_bits_type bit)const {
      return interleaved_bits[bit / CoordinateBits] & safe_left_shift_one(bit % CoordinateBits);
    }
    num_bits_type num_bits_ignored_by_dimension(num_coordinates_type dim)const {
      return (num_low_bits_ignored + (NumDimensions - 1) - dim) / NumDimensions;
    }
    // note: gives "size=0" for max-sized things
    bounding_box get_bbox()const {
      bounding_box result;
      result.min = coords;
      for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
        result.size[i] = safe_left_shift_one(num_bits_ignored_by_dimension(i));
      }
      return result;
    }
  };
  
  static int idx_of_highest_bit(Coordinate i) {
    int upper_bound = CoordinateBits;
    int lower_bound = -1;
    while(true) {
      int halfway_bit_idx = (upper_bound + lower_bound) >> 1;
      if (halfway_bit_idx == lower_bound) return lower_bound;
      
      if (i & ~(safe_left_shift_one(halfway_bit_idx) - 1)) lower_bound = halfway_bit_idx;
      else                                                 upper_bound = halfway_bit_idx;
    }
  }
  
  struct ztree_node;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  struct ztree_node {
    zbox here;
    ztree_node_ptr child0;
    ztree_node_ptr child1;
    unordered_set<ObjectIdentifier> objects_here;
    
    ztree_node(zbox box):here(box),child0(nullptr),child1(nullptr){}
    ztree_node(ztree_node const& other) { *this = other; }
    ztree_node& operator=(ztree_node const& other) {
      here = other.here;
      if(other.child0) child0.reset(new ztree_node(*other.child0));
      if(other.child1) child1.reset(new ztree_node(*other.child1));
      return *this;
    }
  };
  
  static zbox smallest_joint_parent(zbox zb1, zbox zb2) {
    zbox new_box;
    const num_bits_type max_ignored = std::max(zb1.num_low_bits_ignored, zb2.num_low_bits_ignored);
    for (int /* hack... TODO, should possibly be num_coordinates_type, but signed? */ i = NumDimensions - 1; i >= 0; --i) {
      int highest_bit_idx = idx_of_highest_bit(zb1.interleaved_bits[i] ^ zb2.interleaved_bits[i]);
      if ((highest_bit_idx+1 + i * CoordinateBits) < max_ignored) highest_bit_idx = max_ignored - i * CoordinateBits - 1;
      assert((zb1.interleaved_bits[i] & ~((safe_left_shift_one(highest_bit_idx+1)) - 1)) == (zb2.interleaved_bits[i] & ~((safe_left_shift_one(highest_bit_idx+1)) - 1)));
      new_box.interleaved_bits[i] = (zb1.interleaved_bits[i]) & (~((safe_left_shift_one(highest_bit_idx + 1)) - 1));
      if (highest_bit_idx >= 0) {
        new_box.num_low_bits_ignored = highest_bit_idx+1 + i * CoordinateBits;
        for (num_coordinates_type j = 0; j < NumDimensions; ++j) {
          assert(             (zb1.coords[j] & ~(safe_left_shift_one(new_box.num_bits_ignored_by_dimension(j)) - 1))
                           == (zb2.coords[j] & ~(safe_left_shift_one(new_box.num_bits_ignored_by_dimension(j)) - 1)));
          new_box.coords[j] = zb1.coords[j] & ~(safe_left_shift_one(new_box.num_bits_ignored_by_dimension(j)) - 1);
        }
        return new_box;
      }
    }
    new_box.num_low_bits_ignored = max_ignored;
    assert(zb1.coords == zb2.coords);
    new_box.coords = zb1.coords;
    return new_box;
  }
  
  static zbox box_from_coords(std::array<Coordinate, NumDimensions> const& coords, num_bits_type num_low_bits_ignored) {
    zbox result;
    result.num_low_bits_ignored = num_low_bits_ignored;
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      result.coords[i] = coords[i] & (~(safe_left_shift_one(result.num_bits_ignored_by_dimension(i)) - 1));
    }
    for (num_bits_type bit_within_interleaved_bits = num_low_bits_ignored;
                       bit_within_interleaved_bits < total_bits;
                     ++bit_within_interleaved_bits) {
      const num_bits_type bit_idx_within_coordinates = bit_within_interleaved_bits / NumDimensions;
      const num_coordinates_type which_coordinate    = bit_within_interleaved_bits % NumDimensions;
      const num_bits_type interleaved_bit_array_idx  = bit_within_interleaved_bits / CoordinateBits;
      const num_bits_type interleaved_bit_local_idx  = bit_within_interleaved_bits % CoordinateBits;
      assert(bit_idx_within_coordinates >= result.num_bits_ignored_by_dimension(which_coordinate));
      result.interleaved_bits[interleaved_bit_array_idx] |= ((coords[which_coordinate] >> bit_idx_within_coordinates) & 1) << interleaved_bit_local_idx;
    }
    return result;
  }
  
  static void insert_box(ztree_node_ptr& tree, ObjectIdentifier obj, zbox box) {
    if (!tree) {
      tree.reset(new ztree_node(box));
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
        ztree_node_ptr new_tree(new ztree_node(smallest_joint_parent(tree->here, box)));

        assert(new_tree->here.num_low_bits_ignored > tree->here.num_low_bits_ignored);
        assert(new_tree->here.subsumes(tree->here));
        assert(new_tree->here.subsumes(box));
        assert(box.subsumes(tree->here) || (tree->here.get_bit(new_tree->here.num_low_bits_ignored - 1) != box.get_bit(new_tree->here.num_low_bits_ignored - 1)));

        if (tree->here.get_bit(new_tree->here.num_low_bits_ignored - 1)) tree.swap(new_tree->child1);
        else                                                             tree.swap(new_tree->child0);

        tree.swap(new_tree);
        insert_box(tree, obj, box);
      }
    }
  }
  
  static void delete_object(ztree_node_ptr& tree, ObjectIdentifier obj, bounding_box const& bbox) {
    if (!tree) return;
    if (tree->here.get_bbox().overlaps(bbox)) {
      tree->objects_here.erase(obj);
      delete_object(tree->child0, obj, bbox);
      delete_object(tree->child1, obj, bbox);
      
      if (tree->objects_here.empty()) {
        if (tree->child0) {
          if (!tree->child1) {
            ztree_node_ptr dead_tree;
            dead_tree.swap(tree);
            dead_tree->child0.swap(tree);
          }
        }
        else {
          // old 'child1' a.k.a. new 'tree' could be null
          ztree_node_ptr dead_tree;
          dead_tree.swap(tree);
          dead_tree->child1.swap(tree);
        }
      }
    }
  }
  
  void zget_objects_overlapping(ztree_node const* tree, unordered_set<ObjectIdentifier>& results, bounding_box const& bbox)const {
    if (tree && tree->here.get_bbox().overlaps(bbox)) {
      for (const ObjectIdentifier obj : tree->objects_here) {
        auto bbox_iter = bboxes_by_object.find(obj);
        assert(bbox_iter != bboxes_by_object.end());
        if (bbox_iter->second.overlaps(bbox)) results.insert(obj);
      }
      zget_objects_overlapping(tree->child0.get(), results, bbox);
      zget_objects_overlapping(tree->child1.get(), results, bbox);
    }
  }
  
  unordered_map<ObjectIdentifier, bounding_box> bboxes_by_object;
  ztree_node_ptr objects_tree;
  
public:

  void insert(ObjectIdentifier id, bounding_box const& bbox) {
    bboxes_by_object.insert(std::make_pair(id, bbox));
    Coordinate max_dim = bbox.size[0];
    for (num_coordinates_type i = 1; i < NumDimensions; ++i) {
      if (bbox.size[i] > max_dim) max_dim = bbox.size[i];
    }
    int exp = 0; while ((Coordinate(1) << exp) < max_dim) ++exp;
    int dimensions_we_can_single = 0;
    int dimensions_we_can_double = 0;
    const Coordinate base_box_size = safe_left_shift_one(exp);
    const Coordinate used_bits_mask = ~(base_box_size - 1);
    
    for (int i = NumDimensions - 1; i >= 0; --i) {
      if ((bbox.min[i] & used_bits_mask) + base_box_size >= bbox.min[i] + bbox.size[i]) ++dimensions_we_can_single;
      else break;
    }
    for (num_coordinates_type i = 0; i < NumDimensions - dimensions_we_can_single; ++i) {
      if (!(bbox.min[i] & base_box_size)) ++dimensions_we_can_double;
      else break;
    }
#ifdef ZTREE_TESTING
    std::cerr << dimensions_we_can_single << "... " << dimensions_we_can_double << "...\n";
#endif
    for (int i = 0; i < (1 << ((NumDimensions - dimensions_we_can_single) - dimensions_we_can_double)); ++i) {
      std::array<Coordinate, NumDimensions> coords = bbox.min;
      for (num_coordinates_type j = dimensions_we_can_double; j < NumDimensions - dimensions_we_can_single; ++j) {
        if ((i << dimensions_we_can_double) & (1<<j)) coords[j] += base_box_size;
      }
      zbox zb = box_from_coords(coords, exp * NumDimensions + dimensions_we_can_double);
      if (zb.get_bbox().overlaps(bbox))
        insert_box(objects_tree, id, zb);
    }
  }
  bool erase(ObjectIdentifier id) {
    auto bbox_iter = bboxes_by_object.find(id);
    if (bbox_iter == bboxes_by_object.end()) return false;
    delete_object(objects_tree, id, bbox_iter->second);
    bboxes_by_object.erase(bbox_iter);
    return true;
  }
  
  void get_objects_overlapping(unordered_set<ObjectIdentifier>& results, bounding_box const& bbox)const {
    zget_objects_overlapping(objects_tree.get(), results, bbox);
  }

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

#endif

