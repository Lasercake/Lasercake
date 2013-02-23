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

#ifndef LASERCAKE_WORLDGEN_HPP__
#define LASERCAKE_WORLDGEN_HPP__

#include <limits>
#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>

#include "world.hpp"

inline void check_tile_contents_as_valid_worldgenned_values(tile_contents c) {
  caller_correct_if(c == ROCK || c == AIR || c == GROUPABLE_WATER || c == RUBBLE,
    "Trying to place a type of tile other than AIR, ROCK, GROUPABLE_WATER, and RUBBLE");
}

struct worldgen_summary_of_area {
public:
  // You are permitted to be conservative and not specify things about
  // the area even if they're true.  If it'd be slower to be precise
  // than to let the simulation check smaller areas that it needs,
  // then just let it check the smaller areas.
  
  // Default constructor specifies nothing.
  worldgen_summary_of_area()
    : everything_here_is(UNSPECIFIED_TILE_CONTENTS),
      all_objects_whose_centres_are_here(boost::none) {}

  tile_contents everything_here_is; // or UNSPECIFIED_TILE_CONTENTS

  // worldgen'd objects must deterministically compute a numeric identifier
  // for each object, with values between half the max numeric object_identifier
  // and the max object_identifier.  TODO: how can we do better?  Ah we can
  //boost::optional<std::vector<std::pair<object_identifier, shared_ptr<object>>> all_objects_that_intersect_here;
  boost::optional<
    std::pair<
      std::vector<shared_ptr<object>>, // all_objects_whose_centres_are_here
      boost::optional<tile_bounding_box> // what region we need to examine to get all objects that *intersect* here
    >
  > all_objects_whose_centres_are_here;
  // ^ with this method, we need to record which regions have been examined
  // so we won't duplicate objects... hmm.

  static std::pair<
      std::vector<shared_ptr<object>>, // all_objects_whose_centres_are_here
      boost::optional<tile_bounding_box> // what region we need to examine to get all objects that *intersect* here
    > no_objects() { return std::make_pair(
        std::vector<shared_ptr<object>>(),
        boost::optional<tile_bounding_box>()); }
};

class block_initializer {
public:
  // If you like, you can check the region first.
  tile_bounding_box region()const {
    return bounds_;
  }
  // Then call one of the ways of initializing the region's contents.
  // Most of them involve a callback for you to implement and pass to them.

  void initialize_to_uniform_contents(tile_contents c) {
    memset(&wb_->tile_data_uint8_array, c,
      the_decomposition_of_the_world_into_blocks_impl::worldblock_volume);
  }
  template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
  void initialize_by_tile(Functor&& xyz_to_tile_contents) {
    size_t i = 0;
    for (tile_coordinate x = bounds_.min(X); x != bounds_.min(X) + bounds_.size(X); ++x) {
      for (tile_coordinate y = bounds_.min(Y); y != bounds_.min(Y) + bounds_.size(Y); ++y) {
        for (tile_coordinate z = bounds_.min(Z); z != bounds_.min(Z) + bounds_.size(Z); ++z, ++i) {
          const vector3<tile_coordinate> l(x, y, z);
          const tile_contents new_contents = xyz_to_tile_contents(l);
          check_tile_contents_as_valid_worldgenned_values(new_contents);
          wb_->tiles_[i] = tile::make_tile_with_contents_and_interiorness(new_contents, true);
        }
      }
    }
  }

  template<typename Functor /*void (worldblock*, tile_bounding_box*/>
  void initialize_by_legacy_worldgen_function(Functor&& f) {
    f(wb_, bounds_);
  }



  class column_callback {
  public:
    template<typename Functor>
    void operator()(Functor&& f) const {
      for(tile_coordinate z = min_z_; z != max_plus_one_z_; ++z, ++*i_) {
        const tile_contents c = f(z);
        check_tile_contents_as_valid_worldgenned_values(c);
        wb_->tiles_[*i_] = tile::make_tile_with_contents_and_interiorness(c, true);
      }
    }
  private:
    tile_coordinate x_;
    tile_coordinate y_;
    tile_coordinate min_z_;
    tile_coordinate max_plus_one_z_;
    size_t* i_;
    the_decomposition_of_the_world_into_blocks_impl::worldblock* wb_;
    friend class block_initializer;
  };
  template<typename Functor
  /* tile_contents (tile_coordinate x, tile_coordinate y, column_callback fz) */>
  void initialize_by_column(Functor&& x_y_cbz_to_tile_contents) {
    size_t i = 0;
    for (tile_coordinate x = bounds_.min(X); x != bounds_.min(X) + bounds_.size(X); ++x) {
      for (tile_coordinate y = bounds_.min(Y); y != bounds_.min(Y) + bounds_.size(Y); ++y) {
        column_callback cb; cb.x_ = x; cb.y_ = y;
        cb.min_z_ = bounds_.min(Z); cb.max_plus_one_z_ = bounds_.min(Z) + bounds_.size(Z);
        cb.i_ = &i; cb.wb_ = wb_;
        x_y_cbz_to_tile_contents(x, y, cb);
      }
    }
  }

  explicit block_initializer(
    the_decomposition_of_the_world_into_blocks_impl::worldblock* wb) : bounds_(wb->bounding_box()), wb_(wb) {}
  explicit block_initializer(
    the_decomposition_of_the_world_into_blocks_impl::worldblock* wb, tile_bounding_box bounds) : bounds_(bounds), wb_(wb) {}
private:
  tile_bounding_box bounds_;
  the_decomposition_of_the_world_into_blocks_impl::worldblock* wb_;
};
typedef block_initializer::column_callback column_callback;




class worldgen_type {
public:
  virtual worldgen_summary_of_area examine_region(tile_bounding_box /*region*/) {
    // You should override this!
    return worldgen_summary_of_area();
  }
  virtual void create_tiles(block_initializer b) = 0;
  virtual ~worldgen_type() {}
};

namespace std {
  template<> struct hash<pair<tile_coordinate, tile_coordinate> > {
    inline size_t operator()(pair<tile_coordinate, tile_coordinate> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.first);
      boost::hash_combine(seed, v.second);
      return seed;
    }
  };
}























///////////// DEPRECATED
// Don't construct worldgen_function_t:s yourself; use worldgen_from_tilespec().
template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
class worldgen_from_tilespec_t {
public:
  worldgen_from_tilespec_t(Functor const& xyz_to_tile_contents) : xyz_to_tile_contents_(xyz_to_tile_contents) {}
  void operator()(the_decomposition_of_the_world_into_blocks_impl::worldblock* wb, tile_bounding_box bounds)const {
    size_t i = 0;
    for (tile_coordinate x = bounds.min(X); x != bounds.min(X) + bounds.size(X); ++x) {
      for (tile_coordinate y = bounds.min(Y); y != bounds.min(Y) + bounds.size(Y); ++y) {
        for (tile_coordinate z = bounds.min(Z); z != bounds.min(Z) + bounds.size(Z); ++z, ++i) {
          const vector3<tile_coordinate> l(x, y, z);
          const tile_contents new_contents = xyz_to_tile_contents_(l);
          caller_correct_if(new_contents == ROCK || new_contents == AIR || new_contents == GROUPABLE_WATER || new_contents == RUBBLE,
                            "Trying to place a type of tile other than AIR, ROCK, GROUPABLE_WATER, and RUBBLE");
          wb->tiles_[i] = tile::make_tile_with_contents_and_interiorness(new_contents, true);
        }
      }
    }
  }
private:
  Functor xyz_to_tile_contents_;
};
template<typename Functor>
inline worldgen_from_tilespec_t<Functor> make_worldgen_from_tilespec_t(Functor const& f) {
  return worldgen_from_tilespec_t<Functor>(f);
}

class world_column_builder {
public:
  void specify_lowest(tile_contents contents);
  void specify(tile_coordinate min_z, tile_contents contents);
  // (TODO - worldblock_column_builder does not treat these correctly currently)
  void dont_specify_lowest();
  void dont_specify(tile_coordinate min_z);

  world_column_builder() : len(0) {}
  struct spec_from {
    tile_coordinate min;
    tile_contents contents;//or inited tile..
    bool operator<(spec_from other) { return min < other.min; }
  };
  size_t len;
  std::array<spec_from, 12> arr;
};
inline void world_column_builder::specify_lowest(tile_contents contents) {
  arr[len++] = spec_from{0, contents};
}
inline void world_column_builder::specify(tile_coordinate min_z, tile_contents contents) {
  arr[len++] = spec_from{min_z, contents};
}
inline void world_column_builder::dont_specify_lowest() {
  arr[len++] = spec_from{0, UNSPECIFIED_TILE_CONTENTS};
}
inline void world_column_builder::dont_specify(tile_coordinate min_z) {
  arr[len++] = spec_from{min_z, UNSPECIFIED_TILE_CONTENTS};
}

// Functor(world_column_builder& b, coord x, coord y, coord min_z_demanded, coord max_z_demanded)
namespace the_decomposition_of_the_world_into_blocks_impl {
template<typename Functor>
class worldblock_column_builder {
public:
  worldblock_column_builder(Functor const& column_spec) : column_spec_(column_spec) {}
  void operator()(worldblock* wb, tile_bounding_box bounds) {
    // for each x and y column
    // do we have all needed data for that column already?
    // otherwise ask for it, remember it.
    // use the data for the column to fill in the column.
    // (that's all for now.)
    // heyy.. first, just ask every wb and fill and see how it speeds ^_^
    const tile_coordinate min_z = bounds.min(Z);
    const tile_coordinate max_z = bounds.max(Z);
    std::array<world_column_builder, worldblock_dimension*worldblock_dimension>& worldblock_column =
      already_computed_columns_[make_pair(bounds.min(X), bounds.min(Y))];
    // Hack - if there is any data in the worldblock_column,
    // assume it has all the information we need. (TODO.)
    if(worldblock_column[0].len) {
      size_t column_base_idx = 0;
      for (tile_coordinate x = bounds.min(X); x != bounds.min(X) + bounds.size(X); ++x) {
        for (tile_coordinate y = bounds.min(Y); y != bounds.min(Y) + bounds.size(Y);
             ++y, column_base_idx += worldblock_y_factor) {
          world_column_builder& b = worldblock_column[column_base_idx / worldblock_y_factor];
          size_t arr_i = 0;
          while(b.arr[arr_i+1].min <= min_z) {++arr_i;}
          for (tile_coordinate z = bounds.min(Z); z != bounds.min(Z) + bounds.size(Z); ++z) {
            if(b.arr[arr_i+1].min == z) {++arr_i;}
            wb->tile_data_uint8_array[column_base_idx + get_primitive_int(z - wb->global_position_.z)]
              = (b.arr[arr_i].contents | tile::interior_bit_mask);
          }
        }
      }
    }
    else {
      size_t column_base_idx = 0;
      for (tile_coordinate x = bounds.min(X); x != bounds.min(X) + bounds.size(X); ++x) {
        for (tile_coordinate y = bounds.min(Y); y != bounds.min(Y) + bounds.size(Y);
             ++y, column_base_idx += worldblock_y_factor) {
          world_column_builder& b = worldblock_column[column_base_idx / worldblock_y_factor];
          column_spec_(b, x, y, min_z, max_z);
          // This is a bit of a hack:
          b.specify(std::numeric_limits<int32_t>::max(), UNSPECIFIED_TILE_CONTENTS);
          // no binary searches at the moment.
          //
          // These two impls seem to be similar fine speeds, but I haven't
          // looked at this closely -Isaac
          #if 1
            size_t arr_i = 0;
            while(b.arr[arr_i+1].min <= min_z) {++arr_i;}
            //worldblock_dimension_type z_idx_begin = 0;
            for (tile_coordinate z = bounds.min(Z); z != bounds.min(Z) + bounds.size(Z); ++z) {
              if(b.arr[arr_i+1].min == z) {++arr_i;}
              wb->tile_data_uint8_array[column_base_idx + get_primitive_int(z - wb->global_position_.z)]
                = (b.arr[arr_i].contents | tile::interior_bit_mask);
            }
          #else
            for (size_t i = 0; i != b.len - 1 /*due to the hack of +1 end thing*/; ++i) {
              world_column_builder::spec_from spec = b.arr[i];
              world_column_builder::spec_from next_spec = b.arr[i+1];
              if (spec.min <= max_z && next_spec.min > min_z) {
                worldblock_dimension_type z_idx_begin =
                  std::max(0, get_primitive_int(spec.min - wb->global_position_.z));
                worldblock_dimension_type z_idx_end =
                  std::min(worldblock_dimension, get_primitive_int(next_spec.min - wb->global_position_.z));
                memset(&wb->tiles_[column_base_idx + z_idx_begin],
                       spec.contents | tile::interior_bit_mask,
                       z_idx_end - z_idx_begin);
              }
            }
          #endif
        }
      }
    }
  }
private:
  Functor column_spec_;
  std::unordered_map<std::pair<tile_coordinate, tile_coordinate>,
                     std::array<world_column_builder, worldblock_dimension*worldblock_dimension>
  > already_computed_columns_;
};
template<typename Functor>
inline worldblock_column_builder<Functor> make_worldblock_column_builder(Functor const& f) {
  return worldblock_column_builder<Functor>(f);
}
}

template<typename LegacyWorldgenFunctor>
class legacy_worldgen : public worldgen_type {
public:
  legacy_worldgen(LegacyWorldgenFunctor f) : f_(f) {}
  void create_tiles(block_initializer b) override {
    b.initialize_by_legacy_worldgen_function(f_);
  }
private:
  LegacyWorldgenFunctor f_;
};
template<typename Functor>
inline legacy_worldgen<Functor> make_legacy_worldgen(Functor const& f) {
  return legacy_worldgen<Functor>(f);
}
template<typename Functor>
inline legacy_worldgen<Functor>* new_legacy_worldgen(Functor const& f) {
  return new legacy_worldgen<Functor>(f);
}

template<typename Functor>
shared_ptr<worldgen_type> worldgen_from_column_spec(Functor const& column_spec) {
  return shared_ptr<worldgen_type>(new_legacy_worldgen(
    the_decomposition_of_the_world_into_blocks_impl::make_worldblock_column_builder(column_spec)));
}


template<typename Functor /* tile_contents (vector3<tile_coordinate>) */>
shared_ptr<worldgen_type> worldgen_from_tilespec(Functor const& xyz_to_tile_contents) {
  return shared_ptr<worldgen_type>(new_legacy_worldgen(
    make_worldgen_from_tilespec_t(xyz_to_tile_contents)));
}

#endif
