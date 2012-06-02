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

#include "../bbox_collision_detector.hpp"

//#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_MODULE bbox_collision_detector test
#include <boost/test/unit_test.hpp>

typedef int32_t obj_t;
typedef bbox_collision_detector<obj_t, 32, 1> detector_1d;
typedef bbox_collision_detector<obj_t, 32, 2> detector_2d;
typedef bbox_collision_detector<obj_t, 64, 3> detector_3d;

BOOST_AUTO_TEST_CASE( bbox_test_bounding_box ) {
  detector_1d::bounding_box bb1;
  bb1.min[X] = 2;
  bb1.size[X] = 3;
  detector_1d::bounding_box bb2;
  bb2.min[X] = 5;
  bb2.size[X] = 1;
  BOOST_CHECK(!bb1.overlaps(bb2));
  BOOST_CHECK(!bb2.overlaps(bb1));
  detector_1d::bounding_box bb3;
  bb3.min[X] = 4;
  bb3.size[X] = 1;
  BOOST_CHECK(bb1.overlaps(bb3));
  BOOST_CHECK(bb3.overlaps(bb1));
  BOOST_CHECK(!bb3.overlaps(bb2));
  BOOST_CHECK(!bb2.overlaps(bb3));

  detector_2d::bounding_box bb4;
  bb4.min[X] = 2;
  bb4.min[Y] = 4;
  bb4.size[X] = 3;
  bb4.size[Y] = 7;
  detector_2d::bounding_box bb5;
  bb5.min[X] = 7;
  bb5.min[Y] = 4;
  bb5.size[X] = 3;
  bb5.size[Y] = 7;
  detector_2d::bounding_box bb6;
  bb6.min[0] = 3;
  bb6.min[1] = 5;
  bb6.size[0] = 1;
  bb6.size[1] = 1;
  BOOST_CHECK(!bb4.overlaps(bb5));
  BOOST_CHECK(!bb5.overlaps(bb4));
  BOOST_CHECK(!bb6.overlaps(bb5));
  BOOST_CHECK(!bb5.overlaps(bb6));
  BOOST_CHECK(bb4.overlaps(bb6));
  BOOST_CHECK(bb6.overlaps(bb4));
}

class bbox_collision_detector_tester {
  // (friend of bbox_collision_detector, so we can test e.g. zbox on its own)
public:
  static void test_zbox() {
    const std::array<detector_2d::coordinate_type, 2> boxcoords1 = {{ 0, 0 }};
    const std::array<detector_2d::coordinate_type, 2> boxcoords2 = {{ 0x80000000u, 0x80000000u }};
    const std::array<detector_2d::coordinate_type, 2> boxcoords3 = {{ 0x80000000u, 0xc0000013u }};
    const std::array<detector_2d::coordinate_type, 2> boxcoords4 = {{ 0xc0000000u, 0xc0000013u }};
    const detector_2d::zbox everywhere = detector_2d::zbox::box_from_coords(boxcoords1, 32*2); //32bits * 2dimensions
    const detector_2d::zbox one_by_one = detector_2d::zbox::box_from_coords(boxcoords1, 0);
    const detector_2d::zbox quartant = detector_2d::zbox::box_from_coords(boxcoords2, 31*2);
    const detector_2d::zbox rect_in_quartant = detector_2d::zbox::box_from_coords(boxcoords2, 30*2+1);
    const detector_2d::zbox one_by_one_2 = detector_2d::zbox::box_from_coords(boxcoords3, 0);
    const detector_2d::zbox one_by_one_3 = detector_2d::zbox::box_from_coords(boxcoords4, 0);

    // bits are ordered ZYXZYXZYX... (or in 2D, YXYXYX...)
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(63), true);
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(62), true);
    
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(61), true);
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(60), false);
    
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(59), false);
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(58), false);
    
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(57), false);
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(56), false);
    
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(1), true);
    BOOST_CHECK_EQUAL(one_by_one_2.get_bit(0), false);

    BOOST_CHECK_EQUAL(quartant.num_low_bits(), 62);
    BOOST_CHECK_EQUAL(quartant.num_low_bits_by_dimension(X), 31);
    BOOST_CHECK_EQUAL(quartant.num_low_bits_by_dimension(Y), 31);
    
    BOOST_CHECK_EQUAL(rect_in_quartant.num_low_bits(), 61);
    BOOST_CHECK_EQUAL(rect_in_quartant.num_low_bits_by_dimension(X), 31);
    BOOST_CHECK_EQUAL(rect_in_quartant.num_low_bits_by_dimension(Y), 30);
    
    // Both the size *and* location have to be the same for equality.
    BOOST_CHECK_EQUAL(one_by_one, one_by_one);
    BOOST_CHECK_NE(one_by_one, one_by_one_2);
    BOOST_CHECK_NE(one_by_one, everywhere);
    
    // subsumes is reflexive
    BOOST_CHECK(everywhere.subsumes(everywhere));
    BOOST_CHECK(one_by_one.subsumes(one_by_one));
    BOOST_CHECK(quartant.subsumes(quartant));
    BOOST_CHECK(rect_in_quartant.subsumes(rect_in_quartant));
    BOOST_CHECK(one_by_one_2.subsumes(one_by_one_2));
    
    // 'everywhere' subsumes everywhere
    BOOST_CHECK(everywhere.subsumes(everywhere));
    BOOST_CHECK(everywhere.subsumes(one_by_one));
    BOOST_CHECK(everywhere.subsumes(quartant));
    BOOST_CHECK(everywhere.subsumes(rect_in_quartant));
    BOOST_CHECK(everywhere.subsumes(one_by_one_2));

    // subsumes is antisymmetric (two things can only subsume each other if they are equal).
    BOOST_CHECK(!one_by_one.subsumes(everywhere));
    BOOST_CHECK(!quartant.subsumes(everywhere));
    BOOST_CHECK(!rect_in_quartant.subsumes(everywhere));
    BOOST_CHECK(!one_by_one_2.subsumes(everywhere));
    
    // because they're zboxes, either one subsumes the other or they don't overlap.
    // (not tested thoroughly.)

    // smallest_joint_parent can create non-square rectangles correctly:
    const detector_2d::zbox should_be_a_rect = detector_2d::zbox::smallest_joint_parent(one_by_one_2, one_by_one_3);
    BOOST_CHECK(should_be_a_rect.subsumes(one_by_one_2));
    BOOST_CHECK(should_be_a_rect.subsumes(one_by_one_3));
    BOOST_CHECK_EQUAL(should_be_a_rect.num_low_bits(), 61);
    BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(63), true);
    BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(62), true);
    BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(61), true);
    BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(60), false);
    BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(59), false);
    BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(58), false);
    
    BOOST_CHECK(!rect_in_quartant.subsumes(one_by_one_2));
    BOOST_CHECK(!one_by_one_2.subsumes(rect_in_quartant));
    const detector_2d::zbox joint_parent = detector_2d::zbox::smallest_joint_parent(rect_in_quartant, one_by_one_2);
    BOOST_CHECK_EQUAL(joint_parent, quartant);
    BOOST_CHECK_NE(joint_parent, rect_in_quartant);
    BOOST_CHECK_NE(joint_parent, one_by_one_2);
    BOOST_CHECK(joint_parent.subsumes(rect_in_quartant));
    BOOST_CHECK(joint_parent.subsumes(one_by_one_2));
    const detector_2d::zbox joint_parent_2 = detector_2d::zbox::smallest_joint_parent(rect_in_quartant, quartant);
    BOOST_CHECK_EQUAL(joint_parent_2, quartant);
    BOOST_CHECK(!rect_in_quartant.subsumes(quartant));
  }
};

BOOST_AUTO_TEST_CASE( bbox_test_zbox ) {
  bbox_collision_detector_tester::test_zbox();
}
