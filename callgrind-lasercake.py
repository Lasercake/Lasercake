#!/usr/bin/env python
#usage: ./callgrind-lasercake [lasercake arguments...]
#or: ./callgrind-lasercake path/to/lasercake/binary [lasercake arguments...]
#
#If the first argument exists as a file, it's treated as the Lasercake
#binary; otherwise "Lasercake" (in this script's directory) is used.

import os, sys, subprocess
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
outdir = 'zzzuncommitted/callgrindz'
if not exists(outdir): os.makedirs(outdir)
exit(subprocess.call(['valgrind', '--tool=callgrind', '--dump-instr=yes',
	'--dump-line=yes', '--branch-sim=yes', '--cache-sim=yes',
	'--callgrind-out-file='+outdir+'/callgrind.out.%p',
	lasercake] + args))
