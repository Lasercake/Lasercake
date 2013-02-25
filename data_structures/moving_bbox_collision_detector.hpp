/*

    Copyright Eli Dupree and Isaac Dupree, 2013

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

#ifndef LASERCAKE_MOVING_BBOX_COLLISION_DETECTOR_HPP__
#define LASERCAKE_MOVING_BBOX_COLLISION_DETECTOR_HPP__

#include "geometry.hpp"


namespace moving_bbox_collision_detector_internals {

const size_t MIN_NICE_NODE_SIZE = 2;

typedef int object_id;
//typedef lint64_t time_int_type;
typedef lint32_t time_int_type;
typedef non_normalized_rational<time_int_type> time_type;
//typedef faux_optional<time_type> optional_time;

template <int NumDimensions>
struct bbox {
  array<int64_t, NumDimensions> min;
  array<int64_t, NumDimensions> max;

  bbox combined_with(bbox const& o)const {
    bbox result;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      result.max[dim] = std::max(max[dim], o.max[dim]);
      result.min[dim] = std::min(min[dim], o.min[dim]);
    }
    return result;
  }
  bbox plus_nontrivially_including(bbox const& o)const {
    bbox result;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      result.max[dim] = std::max(max[dim], o.max[dim] + 1);
      result.min[dim] = std::min(min[dim], o.min[dim] - 1);
    }
    return result;
  }
  bbox outset()const {
    bbox result;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      result.max[dim] = max[dim] + 1;
      result.min[dim] = min[dim] - 1;
    }
    return result;
  }
  int64_t size(int dim)const { return max[dim] - min[dim]; }
  int64_t maxdim()const {
    int64_t result = 0;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      if (size(dim) > result) result = size(dim);
    }
    return result;
  }

  bool fitsnicely(bbox const& o)const {
    /*for (int dim = 0; dim < NumDimensions; ++dim) {
      if ((size(dim) << 1) > o.size(dim)) {
        return false;
      }
    }
    return true;*/
    return (maxdim() << 1) <= o.maxdim();
  }

  bool contains(bbox const& o)const {
    for (int dim = 0; dim < NumDimensions; ++dim) {
      if ((min[dim] > o.min[dim]) || (max[dim] < o.max[dim])) return false;
    }
    return true;
  }
};

template <int NumDimensions>
struct moving_object {
  object_id id;
  bbox<NumDimensions> phys_bounds;
  array<int64_t, NumDimensions> vel;

  bbox<NumDimensions*2> bounds()const {
    bbox<NumDimensions*2> result;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      result.max[dim] = phys_bounds.max[dim];
      result.min[dim] = phys_bounds.min[dim];
      result.max[dim+NumDimensions] = vel[dim];
      result.min[dim+NumDimensions] = vel[dim];
    }
    return result;
  }

  void double_coords() {
    for (int dim = 0; dim < NumDimensions; ++dim) {
      phys_bounds.max[dim] <<= 1;
      phys_bounds.min[dim] <<= 1;
      vel[dim] <<= 1;
    }
  }
};

template <int NumDimensions>
struct tree_node {
  bbox<NumDimensions*2> bounds;
  std::vector<tree_node> children;
  std::vector<moving_object<NumDimensions>> stuff_here;

  int64_t strain(moving_object<NumDimensions> const& o)const {
    bbox<NumDimensions*2> combined = bounds.plus_nontrivially_including(o.bounds());
    int64_t result = 100 * (combined.maxdim() - bounds.maxdim());
    for (int dim = 0; dim < NumDimensions*2; ++dim) result += combined.size(dim) - bounds.size(dim);
    return result;
  }

  void insert(moving_object<NumDimensions> const& o) {
    //    LOG << "Insert attempt:\n";
    bounds = bounds.plus_nontrivially_including(o.bounds());
    if (!o.bounds().fitsnicely(bounds)) {
      stuff_here.push_back(o);
      //  LOG << "BZX\n";
      return;
    }
    int best_child = -1;
    int64_t least_strain = -1;
    for (size_t i = 0; i < children.size(); ++i) {
      int64_t s = children[i].strain(o);
      if (best_child == -1 || s < least_strain) {
        best_child = i;
        least_strain = s;
      }
    }
    if (best_child != -1) {
      //LOG << least_strain << "\n";
      tree_node& bc = children[best_child];
      if (bc.bounds.plus_nontrivially_including(o.bounds()).fitsnicely(bounds)) {
        //LOG << "foo\n";
        bc.insert(o);
        return;
      }
    }

    if (stuff_here.size() > MIN_NICE_NODE_SIZE) {
      std::vector<int> pulled_stuffs;
      tree_node child_attempt;
      child_attempt.stuff_here.push_back(o);
      child_attempt.bounds = o.bounds().outset();
      for (size_t i = 0; i < stuff_here.size(); ++i) {
        if (child_attempt.bounds.plus_nontrivially_including(stuff_here[i].bounds()).fitsnicely(bounds)) {
          child_attempt.bounds = child_attempt.bounds.plus_nontrivially_including(stuff_here[i].bounds());
          pulled_stuffs.push_back(i);
        }
      }
      if (pulled_stuffs.size() >= MIN_NICE_NODE_SIZE) {
        for (int i = pulled_stuffs.size() - 1; i >= 0; --i) {
          child_attempt.stuff_here.push_back(stuff_here[pulled_stuffs[i]]);
          stuff_here[pulled_stuffs[i]] = stuff_here.back(); stuff_here.pop_back();
        }
        //LOG << "PULL! Woot!\n";
        assert(bounds.contains(child_attempt.bounds));
        children.push_back(child_attempt);
        return;
      }
    }
    //LOG << "plunk\n";
    stuff_here.push_back(o);
  }

  void search_collect_stuff(std::vector<object_id>& results,
        moving_object<NumDimensions> const& o,
        time_type start_time,
        time_type end_time)const //__attribute__((__noinline__))
  {
    for (auto const& o2 : stuff_here) {
      if (o2.id != o.id) {
        time_type first_collision_moment = start_time;
        time_type  last_collision_moment =   end_time;
        for (int dim = 0; dim < NumDimensions; ++dim) {
          // putting the denom line above these two consistently made it a bit faster in 2D, but a bunch slower in 3D
          const int32_t max_num = (o2.phys_bounds.max[dim] - o.phys_bounds.min[dim]);
          const int32_t min_num = (o2.phys_bounds.min[dim] - o.phys_bounds.max[dim]);
          const int32_t denom = (o2.vel[dim] - o.vel[dim]);
          if (denom == 0) {
            if ((min_num > 0) || (max_num < 0)) {
              // Same speed, never overlap - never colliding
              goto breakcontinue;
            }
            // else { They are moving at the same speed so they overlap forever. Restrict nothing. }
          }
          else {
            const time_type min_time((denom > 0) ? min_num : max_num, denom);
            const time_type max_time((denom > 0) ? max_num : min_num, denom);
            if (max_time <  last_collision_moment)  last_collision_moment = max_time; // Duplicate code!!!
            if (min_time > first_collision_moment) first_collision_moment = min_time; // Duplicate code!!!
          }

          // This line should logically be inside the else{} block because we don't need to do this check otherwise,
          // but when I put it in there, it became slower!
          if (first_collision_moment > last_collision_moment) goto breakcontinue;
        }

        // Note - not returning the time of collision
        results.push_back(o2.id);
      }

      breakcontinue:;
    }
  }

  void search(std::vector<object_id>& results,
        moving_object<NumDimensions> const& o,
        time_type start_time,
        time_type end_time)const
  {
    time_type first_possible_overlap = start_time;
    time_type last_possible_overlap = end_time;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      // basically
      // relative velocity = relative position / time
      // time = relative position / relative velocity
      // time = (bounds.something[dim] - o.phys_bounds.something[dim]) / (bounds.something[dim + NUM_DIMENSIONS] - o.velocity[dim])
      // Three of those have multiple possibilities. We're looking for the min and max time.
      // If the denominator is always positive or always negative, it's easy to compute min/max times.
      // If it crosses zero, then there's no min or max.

#if 0
      const int32_t plah[4] = {
        (int32_t)(bounds.max[dim] - o.phys_bounds.min[dim]),
        (int32_t)(bounds.min[dim] - o.phys_bounds.max[dim]),
        (int32_t)(bounds.max[dim + NumDimensions] - o.vel[dim]),
        (int32_t)(bounds.min[dim + NumDimensions] - o.vel[dim])
      };
      int32_t const& max_num = plah[0];
      int32_t const& min_num = plah[1];
      int32_t const& max_denom = plah[2];
      int32_t const& min_denom = plah[3];

      if (denom crosses zero and num crosses zero) {
          // any time is possible
      }
      else {
        const bool max_first = min_denom > 0;
        const int32_t max_time_num = (((min_num > 0) ? min_denom : max_denom) > 0) ? max_num : min_num;
        const int32_t max_time_denom = (max_time_num > 0) ? min_denom : max_denom;
        const int32_t min_time_num = (((min_num > 0) ? max_denom : min_denom) > 0) ? min_num : max_num;
        const int32_t min_time_denom = (min_time_num > 0) ? max_denom : min_denom;
        if (max_time_num *  last_possible_overlap.denominator <  last_possible_overlap.numerator * max_time_denom) {
           last_possible_overlap.numerator   = max_time_num  ;
           last_possible_overlap.denominator = max_time_denom;
        }
        if (min_time_num * first_possible_overlap.denominator > first_possible_overlap.numerator * min_time_denom) {
          first_possible_overlap.numerator   = min_time_num  ;
          first_possible_overlap.denominator = min_time_denom;
        }
      }
#endif

      const int32_t max_num = (bounds.max[dim] - o.phys_bounds.min[dim]);
      const int32_t min_num = (bounds.min[dim] - o.phys_bounds.max[dim]);
      //const int32_t max_denom = (bounds.max[dim + NumDimensions] - o.vel[dim]);
      //const int32_t min_denom = (bounds.min[dim + NumDimensions] - o.vel[dim]);
      const int32_t maxmin_denom[2] = {(int32_t)(bounds.max[dim + NumDimensions] - o.vel[dim]), (int32_t)(bounds.min[dim + NumDimensions] - o.vel[dim])};
      int32_t const& max_denom = maxmin_denom[0];
      int32_t const& min_denom = maxmin_denom[1];
#if 0
      // These assertions should be successful. We omit them only because they cause significant slowdown.
      assert(max_num & 1);
      assert(min_num & 1);
      assert(max_denom & 1);
      assert(min_denom & 1);
      assert(max_num > min_num);
      assert(max_denom > min_denom);
#endif
      if ((min_denom > 0) || (max_denom < 0)) {
        const int32_t max_time_num = ((max_denom < 0) ? min_num : max_num);
        const time_type max_time(max_time_num, (max_time_num < 0) ? max_denom : min_denom);
        const int32_t min_time_num = ((max_denom < 0) ? max_num : min_num);
        const time_type min_time(min_time_num, (min_time_num > 0) ? max_denom : min_denom);
        if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
        if (first_possible_overlap > last_possible_overlap) return;
      }
#if 0
      // This block is equivalent to the above, and indistinguishable in speed.
      // It's worse because it has more duplicate code.
           if (min_denom > 0) {
        const time_type max_time(max_num, (max_num <  0) ? max_denom : min_denom);
        const time_type min_time(min_num, (min_num >  0) ? max_denom : min_denom);
        assert(min_time < max_time);
        if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
        if (first_possible_overlap > last_possible_overlap) return;
      }
      else if (max_denom < 0) {
        const time_type max_time(min_num, (min_num <= 0) ? max_denom : min_denom);
        const time_type min_time(max_num, (max_num >= 0) ? max_denom : min_denom);
        assert(min_time < max_time);
        if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
        if (first_possible_overlap > last_possible_overlap) return;
      }
#endif
#if 0
      // This block would handle cases when one of the denominator edges is zero.
      // In order to avoid having to make these checks, we require that they never be zero
      // by making all objects have even coordinates and all nodes have odd coordinates.
      else if (min_denom == 0) {
        if (max_denom == 0) {
          if ((min_num > 0) || (max_num < 0)) {
            // They are moving at the same speed and do not overlap.
            return;
          }
          // else { They are moving at the same speed so they overlap forever. Restrict nothing. }
        }
        else if (min_num > 0) {
          // Arbitrarily high, but has a min
          const time_type min_time(min_num, max_denom);
          if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
        }
        else if (max_num < 0) {
          // Arbitrarily low, but has a max
          const time_type max_time(max_num, max_denom);
          if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        }
      }
      else if (max_denom == 0) {
        assert(min_denom != 0);
        if (min_num > 0) {
          // Arbitrarily low, but has a max
          const time_type max_time(min_num, min_denom);
          if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        }
        else if (max_num < 0) {
          // Arbitrarily high, but has a min
          const time_type min_time(max_num, min_denom);
          if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
        }
      }
#endif
      else {
        // This is what we call a "split node"; it could be going in either direction,
        // and so its possible overlaps are not an interval, but the complement of an interval
        // (or, in the case where its x values also overlap, its possible overlaps are every time.)

        if (min_num > 0) {
          const time_type    exit_time(min_num, min_denom);
          const time_type reentry_time(min_num, max_denom);
          assert(exit_time < reentry_time);

          // Duplicate code!!!
               if (reentry_time >  last_possible_overlap) { if (   exit_time <  last_possible_overlap) {  last_possible_overlap =    exit_time; } }
          else if (   exit_time < first_possible_overlap) { if (reentry_time > first_possible_overlap) { first_possible_overlap = reentry_time; } }
          else {
            // In this situation, the proper resulting shape would be not one interval, but TWO intervals.
            // To keep it simple, we just leave it as a single contiguous interval.
          }
          if (first_possible_overlap > last_possible_overlap) return;
        }
        else if (max_num < 0) {
          const time_type    exit_time(max_num, max_denom);
          const time_type reentry_time(max_num, min_denom);
          assert(exit_time < reentry_time);

          // Duplicate code!!!
               if (reentry_time >  last_possible_overlap) { if (   exit_time <  last_possible_overlap) {  last_possible_overlap =    exit_time; } }
          else if (   exit_time < first_possible_overlap) { if (reentry_time > first_possible_overlap) { first_possible_overlap = reentry_time; } }
          else {
            // In this situation, the proper resulting shape would be not one interval, but TWO intervals.
            // To keep it simple, we just leave it as a single contiguous interval.
          }
          if (first_possible_overlap > last_possible_overlap) return;
        }
        // else { The boxes overlap and could be going at relative speed zero, so they could overlap at any time, so restrict nothing }
      }

    }

    // The object's trajectory overlaps some part of this box. Check our stuff-here and children.

    for (auto const& c : children) {
      c.search(results, o,
               // Odd behavior - passing the computed values improves 2D, but hurts 3D.
               // (Note that this check takes no time because it's templated.)
               (NumDimensions == 2) ? first_possible_overlap : start_time,
               (NumDimensions == 2) ? last_possible_overlap : end_time);
    }

    // Passing first_possible_overlap and last_possible_overlap instead of start_time and end_time would be equivalent,
    // but in practice I found it to be marginally slower in both 2D and 3D (why?)
    search_collect_stuff(results, o, start_time, end_time);
  }
};

} /* namespace moving_bbox_collision_detector_internals */


// MEGA HACK TODO FIX: all the stuff from the other functions is just being pooped into different-number-of-bits variables
// and not having its formalities respected.
class moving_bbox_collision_detector {
public:
  moving_bbox_collision_detector(){
    for (int dim = 0; dim < 6; ++dim) {
      root.bounds.min[dim] = -1;
      root.bounds.max[dim] = 1;
    }
  }
  void insert(moving_bbox_collision_detector_internals::object_id const& id, bounding_box const& bbox, vector3<distance> vel) {
    moving_bbox_collision_detector_internals::moving_object<3> o;
    o.id = id;
    for (int dim = 0; dim < 3; ++dim) {
      o.phys_bounds.min[dim] = bbox.min(dim) * 2;
      o.phys_bounds.max[dim] = bbox.max(dim) * 2;
      o.vel[dim] = vel(dim);
    }
    root.insert(o);
  }
  bool erase(moving_bbox_collision_detector_internals::object_id const& id) {
    LOG << "Aah! Erasing is unimplemented!\n";
  }
  void get_objects_overlapping(std::vector<moving_bbox_collision_detector_internals::object_id>& results, bounding_box const& bbox, vector3<distance> vel)const {
    o.id = id;
    for (int dim = 0; dim < 3; ++dim) {
      o.phys_bounds.min[dim] = bbox.min(dim) * 2;
      o.phys_bounds.max[dim] = bbox.max(dim) * 2;
      o.vel[dim] = vel(dim);
    }
    root.search(results, o, 0, 1);
  }
private:
  moving_bbox_collision_detector_internals::tree_node<3> root;
};

#endif

