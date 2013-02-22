/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

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

// When you add a new tests file, define a new name here and with
// DECLARE_TESTS_FILE near the top of test_header.hpp, and put at
// the bottom of your tests file:
// REGISTER_TESTS // This must come last in the file.
#define TESTS_FILE patricia_trie_tests
#include "test_header.hpp"

#include "../data_structures/patricia_trie.hpp"

//namespace /*anonymous*/ {

struct trie_traits : default_pow2_radix_patricia_trie_traits {
  typedef noop_deleter leaf_deleter;
  typedef size_t monoid;
};
typedef lint32_t coord;
struct block {
  int contents;
};
// tile_coordinates here are right-shifted by worldblock_dimension_exp
typedef pow2_radix_patricia_trie_node<3, coord, block, trie_traits> trie_node;


BOOST_AUTO_TEST_CASE( patricia_trie_tests_ ) {
  trie_node root;
  BOOST_CHECK(root.is_root_node());
  BOOST_CHECK(!root.points_to_leaf());
  BOOST_CHECK(!root.points_to_sub_nodes());
  BOOST_CHECK(root.is_empty());
  BOOST_CHECK(!root.sub_nodes());
  BOOST_CHECK(!root.leaf());
  BOOST_CHECK(!root.parent());
  BOOST_CHECK(!root.siblings());
  const std::array<coord, 3> somewhere = {{ 7, 27, -3 }};
  //BOOST_CHECK(root.contains(somewhere));
  //BOOST_CHECK(!root.find_node(somewhere));
  BOOST_CHECK(!root.find_leaf_node(somewhere));
  BOOST_CHECK(!root.find_leaf(somewhere));
  BOOST_CHECK_EQUAL(&root, root.find_root());
  BOOST_CHECK_EQUAL(0u, root.monoid());
  //BOOST_CHECK_EQUAL(root.bounding_box().size_exponent_in_each_dimension(), 33);
  
}

//} /* end namespace tests */

REGISTER_TESTS // This must come last in the file.
