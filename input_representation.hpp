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

#ifndef LASERCAKE_INPUT_REPRESENTATION_HPP__
#define LASERCAKE_INPUT_REPRESENTATION_HPP__

#include <algorithm>
#include <set>
#include <vector>
#include <ostream>
//TODO try a binary search for removing headers and see what works

namespace input_representation {

// We abstract from the GUI toolkit so that we can serialize the user-input
// during a frame,
// transfer it over the network consistently,
// change toolkit or support multiple at once if needed,
// and allow simulation code to have a predictable representation of
// input without including any GUI-toolkit headers
// (because the input we need is simple enough that it does not need
// the full force of a GUI toolkit).

// often just a single Unicode character
typedef std::string key_type;

#if 0
// these don't happen at present
namespace special_keys {
const key_type left_arrow = "←";
const key_type up_arrow = "↑";
const key_type right_arrow = "→";
const key_type down_arrow = "↓";
// Modifier keys? Naming them cross-platform?
// Is this meant for display to the user? storage in persistent save-files?
// which purposes?  (TODO figure these out if we need them)
}
#endif


enum pressed_or_released { PRESSED, RELEASED };
// Use set rather than unordered_set because
// (1) the total size is usually small, so it generally doesn't matter
// (2) users control their input very directly, and could intentionally trigger
//       unordered_set's worse-case O(n) time.
// (also, (3), if anything enumerates it, it's in a consistent order)
typedef std::set<key_type> keys_currently_pressed_t;
typedef std::pair<key_type, pressed_or_released> key_change_t;
typedef std::vector<key_change_t> key_activity_t;
typedef int64_t mouse_displacement_coord_t;

// X is rightwards, Y is upwards.
struct mouse_displacement_t {
  mouse_displacement_t() : x(0), y(0) {}
  mouse_displacement_t(mouse_displacement_coord_t x, mouse_displacement_coord_t y) : x(x), y(y) {}
  mouse_displacement_coord_t x;
  mouse_displacement_coord_t y;
  mouse_displacement_t& operator+=(mouse_displacement_t other) {
    x += other.x; y += other.y; return *this;
  }
};

inline bool is_currently_pressed(keys_currently_pressed_t const& keys, key_type const& k) {
  return (keys.find(k) != keys.end());
}
inline bool not_currently_pressed(keys_currently_pressed_t const& keys, key_type const& k) {
  return (keys.find(k) == keys.end());
}
inline int32_t num_times_pressed(key_activity_t const& activity, key_type const& k) {
  return std::count(activity.begin(), activity.end(), key_change_t(k, PRESSED));
}
inline int32_t num_times_released(key_activity_t const& activity, key_type const& k) {
  return std::count(activity.begin(), activity.end(), key_change_t(k, RELEASED));
}



class input_news_t {
public:
  // The default constructor is no keys pressed.
  input_news_t() {}

  input_news_t(
    keys_currently_pressed_t const& keys_currently_pressed,
    key_activity_t const& key_activity_since_last_frame,
    mouse_displacement_t mouse_displacement
  ) :
    keys_currently_pressed_(keys_currently_pressed),
    key_activity_since_last_frame_(key_activity_since_last_frame),
    mouse_displacement_(mouse_displacement) {}

  static input_news_t merge_immediate_sequence(input_news_t const& directly_prior, input_news_t const& directly_hence) {
    input_news_t result;
    result.keys_currently_pressed_ = directly_hence.keys_currently_pressed_;
    result.key_activity_since_last_frame_.insert(
        result.key_activity_since_last_frame_.end(),
        directly_prior.key_activity_since_last_frame_.begin(), directly_prior.key_activity_since_last_frame_.end());
    result.key_activity_since_last_frame_.insert(
        result.key_activity_since_last_frame_.end(),
        directly_hence.key_activity_since_last_frame_.begin(), directly_hence.key_activity_since_last_frame_.end());
    return result;
  }

  // These are generally useful for continuous actions (e.g. if you're
  // moving forwards while you hold down a key).
  //
  // Worst-case O(log(n)) time where "n" is the number of keys currently pressed.
  bool is_currently_pressed(key_type const& k)const {
    return input_representation::is_currently_pressed(keys_currently_pressed_, k);
  }
  bool not_currently_pressed(key_type const& k)const {
    return input_representation::not_currently_pressed(keys_currently_pressed_, k);
  }

  // These are generally useful for instantaneous actions (e.g. if you
  // launch an atomic missile each time you hit a certain key).  Note that a
  // key might have been pressed and released any number of times in-between frames
  // without being currently pressed at the moment that any frame happens.
  //
  // O(n) in number of keys pressed since last frame (TODO fix if there are ever
  // more than O(1) times that the game reads this per frame.).
  int32_t num_times_pressed(key_type const& k)const {
    return input_representation::num_times_pressed(key_activity_since_last_frame_, k);
  }
  int32_t num_times_released(key_type const& k)const {
    return input_representation::num_times_released(key_activity_since_last_frame_, k);
  }

  keys_currently_pressed_t const& keys_currently_pressed()const { return keys_currently_pressed_; }
  key_activity_t const& key_activity_since_last_frame()const { return key_activity_since_last_frame_; }

  // We could, instead, store a sequence of displacements, if some input news
  // user wants to be able to tell the difference between dragging the mouse
  // in a circle and not moving it at all.
  mouse_displacement_t mouse_displacement()const { return mouse_displacement_; }
  void add_mouse_displacement(mouse_displacement_t add) {
    mouse_displacement_ += add;
  }

private:
  keys_currently_pressed_t keys_currently_pressed_;
  key_activity_t key_activity_since_last_frame_;
  mouse_displacement_t mouse_displacement_;
};
inline std::ostream& operator<<(std::ostream& os, input_news_t const& news) {
  os << "{currently pressed:";
  //bool first = true;
  for(key_type key : news.keys_currently_pressed()) {
    os << ' ' << key;
    //if(!first) { os << ' '; }
    //first = false;
    //os << key;
  }
  os << "; activity:";
  for(key_change_t change : news.key_activity_since_last_frame()) {
    os << ' ' << change.first << ':' << (change.second == PRESSED ? "down" : "up");
  }
  return os << '}';
}

} //end namespace input_representation

#endif
