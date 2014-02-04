#!/usr/bin/env python
#usage: ./gdb-lasercake [lasercake arguments...]
#or: ./gdb-lasercake path/to/lasercake/binary [lasercake arguments...]
#
#If the first argument exists as a file, it's treated as the Lasercake
#binary; otherwise "Lasercake" (in this script's directory) is used.

import sys, subprocess
from pipes import quote
from os.path import dirname, join, exists, normpath
scriptdir = normpath(dirname(join('.', __file__)))
for f in ['Lasercake.exe', 'Lasercake']:
	g = join(scriptdir, f)
	if exists(g):
		lasercake = g
args = sys.argv[1:]
if len(args) > 0 and exists(args[0]):
	lasercake = args[0]
	args = args[1:]
gdb_arg_file = join(scriptdir, 'build/gdbarg')
with open(gdb_arg_file, 'w') as f:
	f.write('set args %s\nrun\n' % ' '.join(map(quote, args)))
gdb_args = [lasercake, '-x', gdb_arg_file]
try: FileNotFoundError
except NameError: FileNotFoundError = OSError
try:
	exit(subprocess.call(['gdb'] + gdb_args))
except FileNotFoundError:
	exit(subprocess.call(['ggdb'] + gdb_args))

