#!/usr/bin/env python

import os, sys, subprocess, re, shutil

MAKE_PARALLEL_JOBS = 7

make_flags = ['-j'+str(MAKE_PARALLEL_JOBS)]

ansi_grey = '\033[30m'
ansi_red = '\033[31m'
ansi_green = '\033[32m'
ansi_yellow = '\033[33m'
ansi_blue = '\033[34m'
ansi_magenta = '\033[35m'
ansi_cyan = '\033[36m'
ansi_white = '\033[37m'
ansi_end = '\033[0m'

def main():
	name_for_this_config = ','.join([re.sub(r'[^-+.a-zA-Z0-9]', r'_', arg) for arg in sys.argv[1:]]) or 'default'
	build_dir = 'build/'+name_for_this_config
	cmake_args = []
	make_args = []
	making_lasercake = True
	for arg in sys.argv[1:]:
		m_cxx = re.search(r'^(?:CC|CXX)=(.*)$', arg)
		m_cxxflags = re.search(r'^(?:C|CXX)FLAGS=(.*)$', arg)
		if arg in ['-?', '-h', '--help', '-help']:
			sys.stdout.write(
				"""Usage:\n./dev-build.py\n\t[CXX=g++-4.6|clang++|..]\n\t[CXXFLAGS='-Os -w']\n\t[args for cmake]\n\nCompiles in a build dir that depends on the flags you give,\nthen runs tests.\n"""
				)
			sys.stdout.flush()
			sys.exit()
		elif arg == 'test':
			make_args.append('test-lasercake')
			making_lasercake = False
		elif m_cxx:
			cmake_args.append('-DCMAKE_CXX_COMPILER='+m_cxx.group(1))
		elif m_cxxflags:
			cmake_args.append('-DCMAKE_CXX_FLAGS='+m_cxxflags.group(1))
		else:
			cmake_args.append(arg)
	try: os.remove('CMakeCache.txt')
	except OSError: pass
	try: os.makedirs(build_dir)
	except OSError: pass
	os.chdir(build_dir)
	subprocess.check_call(['cmake', '../../'] + cmake_args)
	make_status = subprocess.call(['time', '-f', (ansi_magenta+'`make` took %E'+ansi_end), 'make'] + make_flags + make_args)
	# How to print the compiler & flags here? Can we get it from cmake
	# somehow? Add to the cmakelists to write those?
	# but with the -l's and all... hmm.
	if make_status != 0:
		subprocess.call(['make'] + make_args) #to get just one file's error messages not mixed up with the others
		sys.stdout.write(ansi_red+'build failed'+ansi_end+'\n')
		sys.stdout.flush()
		exit(1)
	if making_lasercake:
		sys.stdout.write(ansi_green+'and you got:\n./'+build_dir+'/lasercake\n(etc.)'+ansi_end+'\n')
		sys.stdout.flush()
	sys.stdout.write(ansi_cyan+'Testing...'+ansi_end+'\n')
	sys.stdout.flush()
	test_status = subprocess.call(['./test-lasercake'])
	if test_status == 0 and making_lasercake:
		sys.stdout.write(ansi_green+'success; copying '+build_dir+'/lasercake to ./lasercake'+ansi_end+'\n')
		sys.stdout.flush()
		#TODO does this work if Windows .exe extension?
		#(or any of the rest of this script for that matter.)
		shutil.copy2('lasercake', '../../lasercake')
	exit()

if __name__ == '__main__':
	main()

