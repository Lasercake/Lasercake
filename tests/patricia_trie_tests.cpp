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

#include "patricia_trie_tests.hpp"

//namespace /*anonymous*/ {

using namespace patricia_trie_testing;

struct patricia_trie_tester {
  // This is a friend of patricia_trie
  static void test() {
    trie_node root;
    BOOST_CHECK(root.is_root_node());
    BOOST_CHECK(!root.points_to_leaf());
    BOOST_CHECK(!root.points_to_sub_nodes());
    BOOST_CHECK(root.is_empty());
    BOOST_CHECK(!root.sub_nodes());
    BOOST_CHECK(!root.leaf());
    BOOST_CHECK(!root.parent());
    BOOST_CHECK(!root.siblings());
    BOOST_CHECK_EQUAL(root.monoid(), 0u);
    BOOST_CHECK_THROW(root.update_monoid(5), std::logic_error);
    const std::array<coord, 3> somewhere = {{ 7, 27, -3 }};
    const std::array<coord, 3> zerozerozero = {{ 0, 0, 0 }};

    // This is undesirable but harmless:
    BOOST_CHECK(root.contains(zerozerozero));
    // or it could be desirable if the root contained everywhere
    BOOST_CHECK(!root.contains(somewhere));
    // related current fact:
    BOOST_CHECK_EQUAL(root.bounding_box().size_exponent_in_each_dimension(), 0);

    BOOST_CHECK(!root.find_leaf_node(somewhere));
    BOOST_CHECK(!root.find_leaf(somewhere));
    BOOST_CHECK_EQUAL(&root, &root.find_root());
    BOOST_CHECK_EQUAL(0u, root.monoid());

    BOOST_CHECK(!root.erase(somewhere));
    BOOST_CHECK(!root.erase(zerozerozero));
    BOOST_CHECK_NO_THROW(root.insert(somewhere, new block{86}, 2));
    BOOST_CHECK_THROW(root.insert(somewhere, new block{86}, 5), std::logic_error);
    BOOST_CHECK_EQUAL(root.monoid(), 2u);
    BOOST_CHECK_NO_THROW(root.update_monoid(5));
    BOOST_CHECK_EQUAL(root.monoid(), 5u);
    BOOST_CHECK(!root.sub_nodes());
    BOOST_CHECK(!root.is_empty());
    BOOST_CHECK(root.erase(somewhere));
    BOOST_CHECK_EQUAL(root.monoid(), 0u);
    BOOST_CHECK(!root.erase(somewhere));
    BOOST_CHECK(root.is_empty());

    const std::array<coord, 3> v1 = {{ 10, 100, 1000 }};
    const std::array<coord, 3> v2 = {{ 10, 100, 1001 }};
    const std::array<coord, 3> v3 = {{ 10, 100, 1010 }};
    const std::array<coord, 3> v4 = {{ 10, 200, 1005 }};
    BOOST_CHECK_NO_THROW(root.insert(v1, new block{87}, 2));
    BOOST_CHECK_NO_THROW(root.insert(v2, new block{88}, 3));
    BOOST_CHECK_NO_THROW(root.insert(v3, new block{89}, 4));
    BOOST_CHECK_NO_THROW(root.insert(v4, new block{90}, 5));
    BOOST_CHECK_EQUAL(root.monoid(), 14u);
    BOOST_CHECK_THROW(root.update_monoid(5), std::logic_error);
    BOOST_CHECK_EQUAL(root.monoid(), 14u);

    BOOST_CHECK_EQUAL(&root.find_node(v3).find_root(), &root);
    BOOST_CHECK(root.find_leaf(v3));
    BOOST_CHECK(root.find_leaf_node(v3));
    BOOST_CHECK_EQUAL(root.find_leaf(v3)->contents, 89);
    BOOST_CHECK_EQUAL(&root.find_leaf_node(v3)->find_root(), &root);
    BOOST_CHECK_EQUAL(root.find_leaf_node(v3)->bounding_box(),
      trie_node::power_of_two_bounding_cube_type(v3,0)
    );
    BOOST_CHECK(root.find_leaf_node(v3)->parent());
    BOOST_CHECK_EQUAL(root.find_leaf_node(v3)->parent()->bounding_box(),
      trie_node::power_of_two_bounding_cube_type(trie_node::loc_type{{0,32*3,32*31}},5)
    );

    BOOST_CHECK(root.erase(v3));
    BOOST_CHECK_EQUAL(root.monoid(), 10u);
    BOOST_CHECK(!root.find_leaf(v3));
    BOOST_CHECK(!root.find_leaf_node(v3));

    root.debug_check_recursive();
    //root.debug_print();
  }
};

BOOST_AUTO_TEST_CASE( patricia_trie_tests_ ) {
  patricia_trie_tester::test();
}

//} /* end namespace tests */

REGISTER_TESTS // This must come last in the file.
