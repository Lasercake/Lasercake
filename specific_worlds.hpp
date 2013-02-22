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


#ifndef LASERCAKE_SPECIFIC_WORLDS_HPP__
#define LASERCAKE_SPECIFIC_WORLDS_HPP__

#include "world.hpp"

// There's no particular reason that scenarios need to be identified by strings,
// that's just how we have it for now.
// If you pass a string that isn't a scenario, you'll get back
// nothing (if(result) will be false and calling it will be in error).
worldgen_function_t make_world_building_func(std::string scenario);

std::vector<std::string> scenario_names();

#endif

