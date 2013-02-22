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

#ifndef LASERCAKE_PATRICIA_TRIE_TESTS_HPP__
#define LASERCAKE_PATRICIA_TRIE_TESTS_HPP__

#include "../data_structures/patricia_trie.hpp"

namespace patricia_trie_testing {
struct trie_traits : default_pow2_radix_patricia_trie_traits {
  typedef luint64_t monoid;
};
typedef lint32_t coord;
struct block {
  int contents;
};
// tile_coordinates here are right-shifted by worldblock_dimension_exp
typedef pow2_radix_patricia_trie_node<3, coord, block, trie_traits> trie_node;
}

#endif

 