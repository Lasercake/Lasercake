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

#include "borrowed_bitset.hpp"

// This makes the process not safe to exit while the
// sim thread is running...
//#include <QtCore/QThreadStorage>

namespace borrowed_bitset_impl {

#ifdef NO_COMPILER_SUPPORTED_TLS
// TODO - QThreadStorage? pthreads? atomic operations on a shared global
// structure? drop support?
#warning "borrowed_bitset is not thread-safe with this compiler/environment!"
#else
thread_local
#endif
zeroable_bitset_array array_of_bitset_lists = nullptr;

//QThreadStorage<zeroable_bitset_array> thread_local_array_of_bitset_lists;

/*
void delete_array_of_bitset_lists() { delete[] array_of_bitset_lists; }
*/

zeroable_bitset_array get_this_thread_array_of_bitset_lists() {
  //zeroable_bitset_array& array_of_bitset_lists = thread_local_array_of_bitset_lists.localData();
  if(array_of_bitset_lists == nullptr) {
    array_of_bitset_lists = new zeroable_bitset_list[array_of_bitset_lists_len];
  }
  return array_of_bitset_lists;
};

} /* end namespace borrowed_bitset_impl */
