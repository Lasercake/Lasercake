#!/usr/bin/env python
print("""
Usage: ./generate-boostbcp /path/to/boost/1.50.0/
    [ in top-level Lasercake source dir ]
""")

import os, sys, glob, subprocess, shutil

if len(sys.argv) != 2: sys.exit(1)

sources = []
for source_dir in ['.', 'tests', 'data_structures']:
	sources += glob.glob(source_dir+'/*.[ch]pp')

boost_dir = sys.argv[1]
boostbcp_dir = 'bundled_libs/boostbcp'

try: os.mkdir(boostbcp_dir)
except OSError: pass

# `bcp` doesn't copy the Boost license file (the license is permissive
# and, as typical, requires the license be copied into source copies
# of the code), so we must copy it ourselves.
boost_license_file_name = 'LICENSE_1_0.txt'
shutil.copy(boost_dir+'/'+boost_license_file_name,
            boostbcp_dir+'/'+boost_license_file_name)

# Run `bcp` to copy the Boost sources we depend upon into the local
#   ./bundled_libs/boostbcp/
# This requires an installed Boost for the `bcp` binary; any version
#                                                        of Boost will do;
#   and requires a download of the correct version of Boost's source code.
bcp_cmdline = ['bcp', '--scan', '--boost='+boost_dir] + sources + [boostbcp_dir]
subprocess.check_call(bcp_cmdline)

# We define BOOST_SYSTEM_NO_DEPRECATED and BOOST_CHRONO_HEADER_ONLY which
# together make Boost Chrono header-only and not depend on the system .cpp:s.
try: shutil.rmtree(boostbcp_dir+'/libs/system')
except FileNotFoundError: pass

# (We are not using the Boost.Test implementation at all anymore.)
# # In boostbcp mode, we include a (documented) header-only version of test
# # in tests/test_main.cpp.  This avoids complications with the test lib
# # defining main() and related functions the right number of times
# # (0 for lasercake, exactly 1 for tests).
try: shutil.rmtree(boostbcp_dir+'/libs/test')
except FileNotFoundError: pass

# We don't use Boost.Thread currently:
# * Qt also has portable threading.
# * Building Boost.Thread is more complicated than building all its .cpps;
#     e.g. there is a win32 dir whose files don't build on Linux.
# * Linking to system Boost libraries can have difficulties[*].
# (C++11 std::thread would be fine too, but seemed to be not widely
#  enough implemented yet compared to the other C++11 features we use
#  -June 2012.)
try: shutil.rmtree(boostbcp_dir+'/libs/thread')
except FileNotFoundError: pass

# Work around https://svn.boost.org/trac/boost/ticket/7081
# (which is fixed in Boost 1.51) :
with open(boostbcp_dir+'/boost/detail/win/basic_types.hpp', 'r') as f:
	f_contents = f.read()
fixed_f_contents = f_contents.replace('<WinError.h>', '<winerror.h>')
with open(boostbcp_dir+'/boost/detail/win/basic_types.hpp', 'w') as f:
	f.write(fixed_f_contents)

# [*] Regarding system Boost:
#    - Is it the right Boost version? e.g. 1.49.0 has a Boost.Random bug
#      that affects us <https://svn.boost.org/trac/boost/ticket/6189>.
#      1.50.0 made lexical_cast compile in my mingw cross-compile environment
#      <https://svn.boost.org/trac/boost/ticket/6717>. There are probably
#      more Boost-version-related quirks.
#
#    - Is it built with the exact same C++ compiler?  Boost uses lots of
#      tricks - albeit standards-conforming ones - and is likely to be
#      sensitive to ABI.  I had an issue while building Lasercake on OSX
#      that was likely similar to http://stackoverflow.com/a/6083998
#
#    - On my (Isaac's) 64-bit Arch Linux, the lib32 Boost binary didn't
#      happen to work for me, so it was harder to test 32-bit performance.
#      Other lib32 libraries worked fine.
#
# (We really must support system Boost in some cases, at least for the
#  sake of Linux distros.  In any case, many distros won't have the ABI
#  issue so their job is easier.)
