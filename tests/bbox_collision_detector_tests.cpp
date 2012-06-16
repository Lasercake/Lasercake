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

// hack to let us instantiate more bbox_collision_detector variations for the
// tests, as well as test the internals in various ways.
#define BBOX_COLLISION_DETECTOR_IMPL_TREAT_AS_HEADER 1
#include "../bbox_collision_detector.cpp"

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
  bb6.min[X] = 3;
  bb6.min[Y] = 5;
  bb6.size[X] = 1;
  bb6.size[Y] = 1;
  BOOST_CHECK(!bb4.overlaps(bb5));
  BOOST_CHECK(!bb5.overlaps(bb4));
  BOOST_CHECK(!bb6.overlaps(bb5));
  BOOST_CHECK(!bb5.overlaps(bb6));
  BOOST_CHECK(bb4.overlaps(bb6));
  BOOST_CHECK(bb6.overlaps(bb4));

  detector_2d::bounding_box bb7;
  bb7.min[X] = 4;
  bb7.min[Y] = 0;
  bb7.size[X] = 1;
  bb7.size[Y] = 0;//meaning maximum
  detector_2d::bounding_box bb8;
  bb8.min[X] = 0;
  bb8.min[Y] = 2;
  bb8.size[X] = 0;//meaning maximum
  bb8.size[Y] = 1;
  detector_2d::bounding_box bb9;
  bb9.min[X] = 0;
  bb9.min[Y] = 0;
  bb9.size[X] = 0;//meaning maximum
  bb9.size[Y] = 0;//meaning maximum
  detector_2d::bounding_box bb10;
  bb10.min[X] = 13;
  bb10.min[Y] = 13;
  bb10.size[X] = 0;//meaning maximum
  bb10.size[Y] = 0;//meaning maximum
  BOOST_CHECK(!bb8.overlaps(bb4));
  BOOST_CHECK(!bb8.overlaps(bb5));
  BOOST_CHECK(!bb8.overlaps(bb6));
  BOOST_CHECK(bb8.overlaps(bb7));
  BOOST_CHECK(bb8.overlaps(bb8));

  BOOST_CHECK(bb7.overlaps(bb4));
  BOOST_CHECK(!bb7.overlaps(bb5));
  BOOST_CHECK(!bb7.overlaps(bb6));
  BOOST_CHECK(bb7.overlaps(bb7));
  BOOST_CHECK(bb7.overlaps(bb8));

  BOOST_CHECK(bb9.overlaps(bb4));
  BOOST_CHECK(bb9.overlaps(bb5));
  BOOST_CHECK(bb9.overlaps(bb6));
  BOOST_CHECK(bb9.overlaps(bb7));
  BOOST_CHECK(bb9.overlaps(bb8));
  BOOST_CHECK(bb9.overlaps(bb9));

  BOOST_CHECK(bb10.overlaps(bb4));
  BOOST_CHECK(bb10.overlaps(bb5));
  BOOST_CHECK(bb10.overlaps(bb6));
  BOOST_CHECK(bb10.overlaps(bb7));
  BOOST_CHECK(bb10.overlaps(bb8));
  BOOST_CHECK(bb10.overlaps(bb9));
  BOOST_CHECK(bb10.overlaps(bb10));

  detector_2d dect;

  {
    std::vector<obj_t> results1;
    dect.get_objects_overlapping(results1, bb6);
    BOOST_CHECK_EQUAL(results1.size(), 0);
  }

  BOOST_CHECK_EQUAL(dect.exists(42), false);

  dect.insert(1337, bb4);

  {
    std::vector<obj_t> results2;
    dect.get_objects_overlapping(results2, bb6);
    BOOST_CHECK_EQUAL(results2.size(), 1);
    if(results2.size() >= 1) {
      BOOST_CHECK_EQUAL(results2[0], 1337);
    }
  }

  BOOST_CHECK_EQUAL(dect.exists(42), false);

  dect.insert(42, bb5);

  {
    std::vector<obj_t> results3;
    dect.get_objects_overlapping(results3, bb6);
    BOOST_CHECK_EQUAL(results3.size(), 1);
    if(results3.size() >= 1) {
      BOOST_CHECK_EQUAL(results3[0], 1337);
    }
  }

  BOOST_CHECK_EQUAL(dect.exists(42), true);

  dect.insert(31337, bb6);

  {
    std::vector<obj_t> results4;
    dect.get_objects_overlapping(results4, bb6);
    BOOST_CHECK_EQUAL(results4.size(), 2);
    if(results4.size() >= 2) {
      std::sort(results4.begin(), results4.end());
      BOOST_CHECK_EQUAL(results4[0], 1337);
      BOOST_CHECK_EQUAL(results4[1], 31337);
    }
  }

  BOOST_CHECK_EQUAL(dect.exists(42), true);
  BOOST_CHECK_THROW(dect.insert(42, bb5), std::logic_error);
  BOOST_CHECK_EQUAL(dect.erase(42), true);
  BOOST_CHECK_EQUAL(dect.erase(42), false);
  BOOST_CHECK_EQUAL(dect.exists(42), false);
  BOOST_CHECK_EQUAL(dect.erase(42), false);
  BOOST_CHECK_EQUAL(dect.exists(42), false);
  BOOST_CHECK(!dect.find_bounding_box(42));
  BOOST_CHECK_NO_THROW(dect.insert(42, bb5));
  BOOST_CHECK_EQUAL(dect.exists(42), true);
  BOOST_CHECK_EQUAL(*dect.find_bounding_box(42), bb5);

  {
    std::vector<obj_t> results5;
    dect.get_objects_overlapping(results5, bb8);
    BOOST_CHECK_EQUAL(results5.size(), 0);
  }

  dect.insert(189, bb9);
  dect.insert(187, bb7);
  dect.insert(188, bb8);

  {
    std::vector<obj_t> results6;
    dect.get_objects_overlapping(results6, bb8);
    BOOST_CHECK_EQUAL(results6.size(), 3);
  }

  {
    std::vector<obj_t> results7;
    dect.get_objects_overlapping(results7, bb7);
    BOOST_CHECK_EQUAL(results7.size(), 4);
  }

  dect.insert(190, bb10);

  {
    std::vector<obj_t> results8;
    dect.get_objects_overlapping(results8, bb10);
    BOOST_CHECK_EQUAL(results8.size(), 7);
  }
}

BOOST_AUTO_TEST_CASE( bbox_test_zbox ) {
  typedef collision_detector::impl::zbox<32, 2> zbox;
  typedef std::array<detector_2d::coordinate_type, 2> coords;

  static_assert(sizeof(collision_detector::impl::zbox<64,3>) <= 32, "are we still saving zbox space well");
  //static_assert(sizeof(collision_detector::impl::ztree_node<void*, 64, 3>) <= 64, "are we still saving ztree_node space well");

  const coords boxcoords1 = {{ 0, 0 }};
  const coords boxcoords2 = {{ 0x80000000u, 0x80000000u }};
  const coords boxcoords3 = {{ 0x80000000u, 0xc0000013u }};
  const coords boxcoords4 = {{ 0xc0000000u, 0xc0000013u }};
  const coords boxcoords5 = {{ 0xc0000000u, 0x80000000u }};
  const coords boxcoords6 = {{ 0xc0000000u, 0x11000000u }};
  const zbox everywhere = zbox::box_from_coords(boxcoords1, 32*2); //32bits * 2dimensions
  const zbox half_of_everywhere = zbox::box_from_coords(boxcoords1, 32*2-1);
  const zbox one_by_one_1 = zbox::box_from_coords(boxcoords1, 0);
  const zbox quartant = zbox::box_from_coords(boxcoords2, 31*2);
  const zbox rect_in_quartant = zbox::box_from_coords(boxcoords2, 30*2+1);
  //const zbox one_by_one_2 = zbox::box_from_coords(boxcoords2, 0);
  const zbox one_by_one_3 = zbox::box_from_coords(boxcoords3, 0);
  const zbox one_by_one_4 = zbox::box_from_coords(boxcoords4, 0);
  const zbox one_by_one_5 = zbox::box_from_coords(boxcoords5, 0);
  const zbox one_by_one_6 = zbox::box_from_coords(boxcoords6, 0);

  // bits are ordered ZYXZYXZYX... (or in 2D, YXYXYX...)
  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(63), true);
  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(62), true);

  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(61), true);
  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(60), false);

  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(59), false);
  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(58), false);

  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(57), false);
  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(56), false);

  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(1), true);
  BOOST_CHECK_EQUAL(one_by_one_3.get_bit(0), false);

  BOOST_CHECK_EQUAL(quartant.num_low_bits(), 62);
  BOOST_CHECK_EQUAL(quartant.num_low_bits_by_dimension(X), 31);
  BOOST_CHECK_EQUAL(quartant.num_low_bits_by_dimension(Y), 31);

  BOOST_CHECK_EQUAL(rect_in_quartant.num_low_bits(), 61);
  BOOST_CHECK_EQUAL(rect_in_quartant.num_low_bits_by_dimension(X), 31);
  BOOST_CHECK_EQUAL(rect_in_quartant.num_low_bits_by_dimension(Y), 30);

  // Both the size *and* location have to be the same for equality.
  BOOST_CHECK_EQUAL(one_by_one_1, one_by_one_1);
  BOOST_CHECK_NE(one_by_one_1, one_by_one_3);
  BOOST_CHECK_NE(one_by_one_1, everywhere);

  // subsumes is reflexive
  BOOST_CHECK(everywhere.subsumes(everywhere));
  BOOST_CHECK(half_of_everywhere.subsumes(half_of_everywhere));
  BOOST_CHECK(one_by_one_1.subsumes(one_by_one_1));
  BOOST_CHECK(quartant.subsumes(quartant));
  BOOST_CHECK(rect_in_quartant.subsumes(rect_in_quartant));
  BOOST_CHECK(one_by_one_3.subsumes(one_by_one_3));

  // 'everywhere' subsumes everywhere
  BOOST_CHECK(everywhere.subsumes(everywhere));
  BOOST_CHECK(everywhere.subsumes(half_of_everywhere));
  BOOST_CHECK(everywhere.subsumes(one_by_one_1));
  BOOST_CHECK(everywhere.subsumes(quartant));
  BOOST_CHECK(everywhere.subsumes(rect_in_quartant));
  BOOST_CHECK(everywhere.subsumes(one_by_one_3));
  BOOST_CHECK(everywhere.subsumes(one_by_one_6));

  // subsumes is antisymmetric (two things can only subsume each other if they are equal).
  BOOST_CHECK(!half_of_everywhere.subsumes(everywhere));
  BOOST_CHECK(!one_by_one_1.subsumes(everywhere));
  BOOST_CHECK(!quartant.subsumes(everywhere));
  BOOST_CHECK(!rect_in_quartant.subsumes(everywhere));
  BOOST_CHECK(!one_by_one_3.subsumes(everywhere));
  BOOST_CHECK(!one_by_one_6.subsumes(everywhere));

  // because they're zboxes, either one subsumes the other or they don't overlap.
  BOOST_CHECK(everywhere.overlaps(everywhere));
  BOOST_CHECK(one_by_one_1.overlaps(one_by_one_1));
  BOOST_CHECK(quartant.overlaps(quartant));
  BOOST_CHECK(rect_in_quartant.overlaps(rect_in_quartant));
  BOOST_CHECK(one_by_one_3.overlaps(one_by_one_3));
  BOOST_CHECK(everywhere.overlaps(everywhere));
  BOOST_CHECK(one_by_one_1.overlaps(everywhere));
  BOOST_CHECK(quartant.overlaps(everywhere));
  BOOST_CHECK(rect_in_quartant.overlaps(everywhere));
  BOOST_CHECK(one_by_one_3.overlaps(everywhere));
  BOOST_CHECK(everywhere.overlaps(everywhere));
  BOOST_CHECK(everywhere.overlaps(one_by_one_1));
  BOOST_CHECK(everywhere.overlaps(quartant));
  BOOST_CHECK(everywhere.overlaps(rect_in_quartant));
  BOOST_CHECK(everywhere.overlaps(one_by_one_3));
  BOOST_CHECK(half_of_everywhere.overlaps(one_by_one_6));
  BOOST_CHECK(half_of_everywhere.subsumes(one_by_one_6));
  BOOST_CHECK(!rect_in_quartant.overlaps(one_by_one_3));
  BOOST_CHECK(!rect_in_quartant.overlaps(one_by_one_1));
  BOOST_CHECK(!one_by_one_3.overlaps(rect_in_quartant));
  BOOST_CHECK(!one_by_one_1.overlaps(rect_in_quartant));
  BOOST_CHECK(!one_by_one_1.overlaps(one_by_one_3));
  BOOST_CHECK(!one_by_one_3.overlaps(one_by_one_1));

  // check that zbox overlaps is consistent with bbox overlaps
  BOOST_CHECK(everywhere.get_bbox().overlaps(everywhere.get_bbox()));
  BOOST_CHECK(one_by_one_1.get_bbox().overlaps(one_by_one_1.get_bbox()));
  BOOST_CHECK(quartant.get_bbox().overlaps(quartant.get_bbox()));
  BOOST_CHECK(rect_in_quartant.get_bbox().overlaps(rect_in_quartant.get_bbox()));
  BOOST_CHECK(one_by_one_3.get_bbox().overlaps(one_by_one_3.get_bbox()));
  BOOST_CHECK(everywhere.get_bbox().overlaps(everywhere.get_bbox()));
  BOOST_CHECK(one_by_one_1.get_bbox().overlaps(everywhere.get_bbox()));
  BOOST_CHECK(quartant.get_bbox().overlaps(everywhere.get_bbox()));
  BOOST_CHECK(rect_in_quartant.get_bbox().overlaps(everywhere.get_bbox()));
  BOOST_CHECK(one_by_one_3.get_bbox().overlaps(everywhere.get_bbox()));
  BOOST_CHECK(everywhere.get_bbox().overlaps(everywhere.get_bbox()));
  BOOST_CHECK(everywhere.get_bbox().overlaps(one_by_one_1.get_bbox()));
  BOOST_CHECK(everywhere.get_bbox().overlaps(quartant.get_bbox()));
  BOOST_CHECK(everywhere.get_bbox().overlaps(rect_in_quartant.get_bbox()));
  BOOST_CHECK(everywhere.get_bbox().overlaps(one_by_one_3.get_bbox()));
  BOOST_CHECK(half_of_everywhere.get_bbox().overlaps(half_of_everywhere.get_bbox()));
  BOOST_CHECK(half_of_everywhere.get_bbox().overlaps(one_by_one_6.get_bbox()));
  BOOST_CHECK(!half_of_everywhere.get_bbox().overlaps(one_by_one_3.get_bbox()));
  BOOST_CHECK(!rect_in_quartant.get_bbox().overlaps(one_by_one_3.get_bbox()));
  BOOST_CHECK(!rect_in_quartant.get_bbox().overlaps(one_by_one_1.get_bbox()));
  BOOST_CHECK(!one_by_one_3.get_bbox().overlaps(rect_in_quartant.get_bbox()));
  BOOST_CHECK(!one_by_one_1.get_bbox().overlaps(rect_in_quartant.get_bbox()));
  BOOST_CHECK(!one_by_one_1.get_bbox().overlaps(one_by_one_3.get_bbox()));
  BOOST_CHECK(!one_by_one_3.get_bbox().overlaps(one_by_one_1.get_bbox()));

  // smallest_joint_parent can create non-square rectangles correctly:
  const zbox should_be_a_rect = zbox::smallest_joint_parent(one_by_one_3, one_by_one_4);
  BOOST_CHECK(should_be_a_rect.subsumes(one_by_one_3));
  BOOST_CHECK(should_be_a_rect.subsumes(one_by_one_4));
  BOOST_CHECK_EQUAL(should_be_a_rect.num_low_bits(), 61);
  BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(63), true);
  BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(62), true);
  BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(61), true);
  BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(60), false);
  BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(59), false);
  BOOST_CHECK_EQUAL(should_be_a_rect.get_bit(58), false);

  // but rectangles shaped in the other direction are impossible:
  const zbox should_not_be_a_rect = zbox::smallest_joint_parent(one_by_one_4, one_by_one_5);
  BOOST_CHECK_EQUAL(should_not_be_a_rect, quartant);

  BOOST_CHECK(!rect_in_quartant.subsumes(one_by_one_3));
  BOOST_CHECK(!one_by_one_3.subsumes(rect_in_quartant));
  const zbox joint_parent = zbox::smallest_joint_parent(rect_in_quartant, one_by_one_3);
  BOOST_CHECK_EQUAL(joint_parent, quartant);
  BOOST_CHECK_NE(joint_parent, rect_in_quartant);
  BOOST_CHECK_NE(joint_parent, one_by_one_3);
  BOOST_CHECK(joint_parent.subsumes(rect_in_quartant));
  BOOST_CHECK(joint_parent.subsumes(one_by_one_3));
  const zbox joint_parent_2 = zbox::smallest_joint_parent(rect_in_quartant, quartant);
  BOOST_CHECK_EQUAL(joint_parent_2, quartant);
  BOOST_CHECK(!rect_in_quartant.subsumes(quartant));
}
