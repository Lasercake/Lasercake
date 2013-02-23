/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_TILE_ITERATION_HPP__
#define LASERCAKE_TILE_ITERATION_HPP__

#include <boost/logic/tribool.hpp>

#include "world.hpp"

using boost::tribool;
using boost::indeterminate;

struct boring_visitor {
  // return 'true' = definitely want to see everything within here,
  // 'false' = don't want to see anything within here,
  // 'indeterminate' = keep asking in more detail
  //    ('indeterminate' all the way down *will* call collidable_tile()).
  tribool look_here(power_of_two_bounding_cube<3, tile_coordinate> const&) {
    return true;
  }
  // return 'true' = keep going, 'false' = exit the search
  // giving tile_location an idx member should make just providing the tile_loc fine
  bool collidable_tile(tile_location const&) {
    return true;
  }
  // octant() is only called once, at the beginning of the search.
  octant_number octant()const { return 0; }
};

/*
slow_tile_location
tile_with_neighbors
tile_location could/should have an idx member.

ah the whiles could use idx and idx could be xored in the very inner place
to get the actual idx. i think that is most reasonable. and xyz will be pre-xored.
*/

// For your convenience:
inline bool overlaps(tile_bounding_box const& a, power_of_two_bounding_cube<3, tile_coordinate> const& b) {
  return
       a.min(X) <= b.max(X) && b.min(X) <= a.max(X)
    && a.min(Y) <= b.max(Y) && b.min(Y) <= a.max(Y)
    && a.min(Z) <= b.max(Z) && b.min(Z) <= a.max(Z);
}
inline bool subsumes(tile_bounding_box const& a, power_of_two_bounding_cube<3, tile_coordinate> const& b) {
  return
       a.min(X) <= b.min(X) && b.max(X) <= a.max(X)
    && a.min(Y) <= b.min(Y) && b.max(Y) <= a.max(Y)
    && a.min(Z) <= b.min(Z) && b.max(Z) <= a.max(Z);
}

inline tile_bounding_box cube_bbox_to_tile_bounding_box(power_of_two_bounding_cube<3, tile_coordinate> const& b) {
  return tile_bounding_box(b.min(), b.size());
}

template<typename Visitor>
void world::visit_collidable_tiles(Visitor&& visitor) {
  using namespace the_decomposition_of_the_world_into_blocks_impl;
  size_t COUNT = 0;

  const octant_number octant = visitor.octant();
  const int octant_xor = ~octant & 7;

  worldblock_trie* trie_node = &worldblock_trie_;
  int trie_sub_idx = 0; // use (trie_sub_idx ^ ~octant) to get the actual idx

  // worldblock-bits in the trie are right-shifted from tile_coordinates.
  // Luckily this means the top bit never varies, so we won't need to set
  // the nonexistent (1<<32) bit in this bitmap.
  tile_coordinate entirely_valid_as_of_worldblock_bit = 0;

  // array [0..8) of 3-bit quantities saying
  // which sibling is the next one.
  // "the end" is indicated by pointing to itself. or last_sub_node ?
  //int first_sub_node;
  //uint32_t trie_iteration_order = ;
  typedef power_of_two_bounding_cube<3, tile_coordinate> bounding_cube;
  //LOG << "BEGIN\n";
  while(true) {
    //LOG << std::hex << "NODE " << size_t(trie_node) << std::dec << std::endl;
    const int trie_bit = trie_node->bounding_box().size_exponent_in_each_dimension();
    assert(size_t(trie_bit) < sizeof(tile_coordinate)*8);
    bool trie_node_uninteresting_to_look_within = false;
    // Anything in the trie has something collidable inside it, by definition.
    if(entirely_valid_as_of_worldblock_bit == 0) {
      const bounding_cube::loc_type min = {{
        trie_node->bounding_box().min(X) << worldblock_dimension_exp,
        trie_node->bounding_box().min(Y) << worldblock_dimension_exp,
        trie_node->bounding_box().min(Z) << worldblock_dimension_exp
      }};
      const bounding_cube bbox(min, trie_bit + worldblock_dimension_exp);
      const tribool is_here_interesting = visitor.look_here(bbox);
      if(!is_here_interesting) { trie_node_uninteresting_to_look_within = true; }
      if(is_here_interesting) { entirely_valid_as_of_worldblock_bit |= (tile_coordinate(1)<<trie_bit); }
    }
    if(!trie_node_uninteresting_to_look_within) {
      if(worldblock* wb = trie_node->leaf().worldblock_) {
        if(wb->non_interior_bitmap_large_scale_) {
          ++COUNT;
          worldblock_dimension_type entirely_valid_as_of_tile_coordinate_bit = (!!entirely_valid_as_of_worldblock_bit) << worldblock_dimension_exp;
          #if 0
          //this is redundant with trie check
          if(entirely_valid_as_of_tile_coordinate_bit == 0) {
            const int bit = 4;
            const tribool is_here_interesting = visitor.look_here(bounding_cube(wb->global_position_, bit));
            if(!is_here_interesting) { continue; }
            if(is_here_interesting) { entirely_valid_as_of_tile_coordinate_bit |= (1<<bit); }
          }
          #endif
          //if(entirely_valid_as_of_bit) {results.reserve(results.size() + wb->count_of_non_interior_tiles_here_);}
          size_t idx = 0;
          size_t ll_scale_i = octant_xor << 3;
          const worldblock_dimension_type x_xor = (worldblock_dimension-1) * !LASERCAKE_OCTANT_X_POSITIVE(octant);
          const worldblock_dimension_type y_xor = (worldblock_dimension-1) * !LASERCAKE_OCTANT_Y_POSITIVE(octant);
          const worldblock_dimension_type z_xor = (worldblock_dimension-1) * !LASERCAKE_OCTANT_Z_POSITIVE(octant);
          const worldblock_dimension_type idx_xor = x_xor*worldblock_x_factor + y_xor*worldblock_y_factor + z_xor*worldblock_z_factor;
          vector3<tile_coordinate> global_loc(
            wb->global_position_.x ^ x_xor,
            wb->global_position_.y ^ y_xor,
            wb->global_position_.z ^ z_xor
          );
          do { do { do {
            if(wb->non_interior_bitmap_large_scale_ & (uint64_t(0xff) << ll_scale_i)) {
              if(entirely_valid_as_of_tile_coordinate_bit == 0) { // && more than one of those bits was set
                const int bit = 3;
                const tribool is_here_interesting = visitor.look_here(bounding_cube(global_loc & (~0 << bit), bit));
                if(!is_here_interesting) { goto continue_3; }
                if(is_here_interesting) { entirely_valid_as_of_tile_coordinate_bit |= (1<<bit); }
              }
              {
                size_t large_scale_i = ll_scale_i ^ octant_xor;
                do { do { do {
                  if(wb->non_interior_bitmap_small_scale_[large_scale_i]) {
                    if(entirely_valid_as_of_tile_coordinate_bit == 0) {
                      const int bit = 2;
                      const tribool is_here_interesting = visitor.look_here(bounding_cube(global_loc & (~0 << bit), bit));
                      if(!is_here_interesting) { goto continue_2; }
                      if(is_here_interesting) { entirely_valid_as_of_tile_coordinate_bit |= (1<<bit); }
                    }
                    {
                      size_t ss_scale_i = octant_xor << 3;
                      do { do { do {
                        if(wb->non_interior_bitmap_small_scale_[large_scale_i] & (uint64_t(0xff) << ss_scale_i)) {
                          if(entirely_valid_as_of_tile_coordinate_bit == 0) {
                            const int bit = 1;
                            const tribool is_here_interesting = visitor.look_here(bounding_cube(global_loc & (~0 << bit), bit));
                            if(!is_here_interesting) { goto continue_1; }
                            if(is_here_interesting) { entirely_valid_as_of_tile_coordinate_bit |= (1<<bit); }
                          }
                          {
                            size_t small_scale_i = ss_scale_i ^ octant_xor;
                            do { do { do {
                              if(wb->non_interior_bitmap_small_scale_[large_scale_i] & (uint64_t(1) << small_scale_i)) { // !t.is_interior()) {
                                if(entirely_valid_as_of_tile_coordinate_bit == 0) {
                                  const tribool is_here_interesting = visitor.look_here(bounding_cube(global_loc, 0));
                                  if(!is_here_interesting) { goto continue_0; }
                                }
                                {
                                  const tile_location tloc(global_loc, idx ^ idx_xor, wb);
                                  assert_if_ASSERT_EVERYTHING(!tloc.stuff_at().is_interior());
                                  if(!visitor.collidable_tile(tloc)) {
                                    //LOG << "ONE~" << COUNT << ":count.\n";
                                    return;
                                  }
                                }
                                continue_0:;
                              }
                            global_loc.z ^= (1<<0); idx ^= (1<<0); small_scale_i ^= (1<<0); } while(idx & (1<<0));
                            global_loc.y ^= (1<<0); idx ^= (1<<4); small_scale_i ^= (1<<1); } while(idx & (1<<4));
                            global_loc.x ^= (1<<0); idx ^= (1<<8); small_scale_i ^= (1<<2); } while(idx & (1<<8));
                            entirely_valid_as_of_tile_coordinate_bit &= ~(1<<1);
                          }
                          continue_1:;
                        }
                      global_loc.z ^= (1<<1); idx ^= (1<<1); ss_scale_i ^= (1<<3); } while(idx & (1<<1));
                      global_loc.y ^= (1<<1); idx ^= (1<<5); ss_scale_i ^= (1<<4); } while(idx & (1<<5));
                      global_loc.x ^= (1<<1); idx ^= (1<<9); ss_scale_i ^= (1<<5); } while(idx & (1<<9));
                      entirely_valid_as_of_tile_coordinate_bit &= ~(1<<2);
                    }
                    continue_2:;
                  }
                global_loc.z ^= (1<<2); idx ^= (1<<2);  large_scale_i ^= (1<<0); } while(idx & (1<<2));
                global_loc.y ^= (1<<2); idx ^= (1<<6);  large_scale_i ^= (1<<1); } while(idx & (1<<6));
                global_loc.x ^= (1<<2); idx ^= (1<<10); large_scale_i ^= (1<<2);} while(idx & (1<<10));
                entirely_valid_as_of_tile_coordinate_bit &= ~(1<<3);
              }
              continue_3:;
            }
          global_loc.z ^= (1<<3); idx ^= (1<<3);  ll_scale_i ^= (1<<3); } while(idx & (1<<3));
          global_loc.y ^= (1<<3); idx ^= (1<<7);  ll_scale_i ^= (1<<4); } while(idx & (1<<7));
          global_loc.x ^= (1<<3); idx ^= (1<<11); ll_scale_i ^= (1<<5); } while(idx & (1<<11));
          //entirely_valid_as_of_tile_coordinate_bit &= ~(1<<4);
        }
      }

      if(worldblock_trie::sub_nodes_type* sub_nodes = trie_node->sub_nodes()) {
        trie_sub_idx = 0;
        trie_node = &(*sub_nodes)[octant_xor];
        continue;
      }
    }

    //sibling_or_up:
    entirely_valid_as_of_worldblock_bit &= ~(tile_coordinate(1)<<trie_bit);
    //huh, using ++ seems to create a zyx iteration order here, which is
    //a bit undesirable because z is already the shortest dimension currently, TODO
    if(!trie_node->siblings()) {
      //LOG << "TWO~" << COUNT << ":count.\n";
      return;
    }
    ++trie_sub_idx;
    while(trie_sub_idx == 8) {
      trie_node = trie_node->parent();
      if(trie_node == nullptr || !trie_node->siblings()) {
        //LOG << "THREE~" << COUNT << ":count.\n";
        return;
      }
      const int new_trie_bit = trie_node->bounding_box().size_exponent_in_each_dimension();
      entirely_valid_as_of_worldblock_bit &= ~(tile_coordinate(1)<<new_trie_bit);
      trie_sub_idx = (trie_node - &(*trie_node->siblings())[0]) ^ octant_xor;
      assert(trie_sub_idx >= 0);
      assert(trie_sub_idx < 8);
      ++trie_sub_idx;
    }
    trie_node = &(*trie_node->siblings())[trie_sub_idx ^ octant_xor];
  }
}


#endif
