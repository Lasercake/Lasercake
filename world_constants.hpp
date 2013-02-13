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

#ifndef LASERCAKE_WORLD_CONSTANTS_HPP__
#define LASERCAKE_WORLD_CONSTANTS_HPP__

#include "utils.hpp"
#include "units.hpp"

// fine_distance_units: Fine as opposed to coarse, that is.

// time_units: Choose a number that makes lots of frames-per-second values multiply in evenly.
// TODO if we use this more and want a different representation, that would be fine too.
// TODO where it doesn't already, code should refer to some kind of time unit more,
//        rather than implicitly relying on frames being a fixed duration.

typedef units<dim::ratio<10>, dim::meter<1>> tile_widths_t;
typedef units<dim::ratio< 2>, dim::meter<1>> tile_heights_t;
typedef units<dim::ratio< 1, 200>, dim::meter<1>> fine_distance_units_t;
typedef typename units_prod<fine_distance_units_t, dim::ratio<1, 30>>::type tile_physics_sub_tile_units_t;
typedef units<dim::ratio<1, (2*2*2*2 * 3*3*3 * 5*5 * 7 * 11)>, dim::second<1>> time_units_t;
typedef typename units_prod<fine_distance_units_t, dim::second<(-1)>>::type velocity_units_t;
typedef typename units_prod<fine_distance_units_t, dim::second<(-2)>>::type acceleration_units_t;
typedef typename units_prod<milli_t, grams_t, dim::meter<(-3)>>::type density_units_t;
typedef pascals_t pressure_units_t;

// This is a stupid unit but it exists currently:
typedef typename units_prod<seconds_t, dim::ratio<1, 30>>::type fixed_frame_lengths_t;



//typedef typename units_prod<kilograms_t, fine_distance_units_t::pow<(-3)>>::type density_units_t;


//constexpr auto velocity_units = fine_units / /* units_factor<30>() / ? */ seconds;
//constexpr auto tile_physics_sub_tile_units = fine_units / units_factor<30>();


constexpr auto tile_widths = tile_widths_t();
constexpr auto tile_heights = tile_heights_t();
constexpr auto fine_distance_units = fine_distance_units_t();
constexpr auto tile_physics_sub_tile_units = tile_physics_sub_tile_units_t();
constexpr auto time_units = time_units_t();
constexpr auto velocity_units = velocity_units_t();
constexpr auto acceleration_units = acceleration_units_t();
constexpr auto density_units = density_units_t();
constexpr auto pressure_units = pressure_units_t();
constexpr auto fixed_frame_lengths = fixed_frame_lengths_t();

// Rationale for the 64 bit types:
// A lot of computations in Lasercake require more than 32 bits.
//
// Rationale for the 32 bit types:
// We can fit it within 32 bits, so we might as well have faster math
// (on 32bit, and possibly 64bit platform multiply/divides) and smaller storage.

typedef physical_quantity<lint64_t, fine_distance_units_t> distance;
typedef physical_quantity<lint64_t, velocity_units_t> velocity;
typedef physical_quantity<lint64_t, acceleration_units_t> acceleration;
typedef physical_quantity<lint32_t, tile_physics_sub_tile_units_t> sub_tile_distance;
typedef physical_quantity<lint64_t, tile_physics_sub_tile_units_t> large_sub_tile_distance;
typedef physical_quantity<lint64_t, time_units_t> time_unit;


typedef distance fine_scalar; // TODO replace fine_scalar->distance.
typedef fine_distance_units_t fine_units_t; // TODO
constexpr auto fine_units = fine_units_t(); // TODO




//ok...
const distance tile_width = 1 * tile_widths * identity(fine_units / tile_widths);
const distance tile_height = 1 * tile_heights * identity(fine_units / tile_heights);
const vector3<distance> tile_size(tile_width, tile_width, tile_height);

// Standard (Earth-equivalent) gravity: precisely 9.80665 m/s2
const acceleration gravity_acceleration_magnitude = divide(9806650 * (micro*meters) / (seconds*seconds), identity((micro*meters) / fine_distance_units), rounding_strategy<rounding_strategies::round_to_nearest_with_ties_rounding_to_even>());
const vector3<acceleration> gravity_acceleration(0, 0, -gravity_acceleration_magnitude);

#if 0
const time_unit time_units_per_second = 2*2*2*2 * 3*3*3 * 5*5 * 7 * 11;
// delete these if we make frames variable length:
const time_unit fixed_frames_per_second = 30;
const time_unit time_units_per_fixed_frame = time_units_per_second / fixed_frames_per_second;

const fine_scalar tile_width = 2000;
const fine_scalar tile_height = 400;
const vector3<fine_scalar> tile_size(tile_width, tile_width, tile_height);

// Velocity is currently in units of (fine_unit / second).  We might change this.
const fine_scalar velocity_scale_factor = fixed_frames_per_second;

const sub_tile_distance min_convincing_speed           = velocity_scale_factor * tile_width / 50;
const sub_tile_distance gravity_acceleration_magnitude = velocity_scale_factor * tile_width / 200;
const vector3<sub_tile_distance> gravity_acceleration(0, 0, -gravity_acceleration_magnitude); // in mini-units per frame squared
const sub_tile_distance friction_amount                = velocity_scale_factor * tile_width / 1800;

// TODO: Get some of these constants out of the header that everyone includes
const sub_tile_distance pressure_per_depth_in_tile_heights = gravity_acceleration_magnitude * tile_height * velocity_scale_factor;

// I believe this value makes it so that the terminal velocity of falling fluid is "half a vertical tile per frame".
const sub_tile_distance air_resistance_constant = tile_height * tile_height * velocity_scale_factor * velocity_scale_factor / gravity_acceleration_magnitude / 2;
const sub_tile_distance idle_progress_reduction_rate = 20 * velocity_scale_factor;

const vector3<sub_tile_distance> inactive_fluid_velocity(0, 0, -min_convincing_speed);

const fine_scalar max_object_speed_through_water = tile_width * velocity_scale_factor / 16;
#endif

#endif
