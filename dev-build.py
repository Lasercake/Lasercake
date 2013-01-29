#!/usr/bin/env python

import os, sys, subprocess, re, shutil, hashlib

MAKE_PARALLEL_JOBS = 3

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

os.environ['PATH'] = '/usr/lib/colorgcc/bin:'+os.environ['PATH']

def say(string):
	sys.stdout.write(string)
	sys.stdout.flush()

def escaped_control_characters(string):
	return re.sub(r"[\x01-\x1F\x7F]", r"\\", string)

def say_we_are_calling(string):
	say(ansi_cyan+'% '+escaped_control_characters(string)+ansi_end+'\n')

def hash_list(l):
	return hashlib.sha256(b''.join(hashlib.sha256(arg.encode('utf8')).digest() for arg in l)).hexdigest()

def main():
	try: subprocess.call(['cmake', '--version'])
	except OSError:
		say(ansi_red+"Error: 'cmake' not found; please install it."+ansi_end+'\n')
		exit(1)
	try: subprocess.call(['make', '--version'])
	except OSError:
		say(ansi_red+"Error: 'make' not found; please install it."+ansi_end+'\n')
		exit(1)
	try:
		status = subprocess.call(['time', '-f', '', 'true'])
		time_has_f = (status == 0)
	except OSError:
		say(ansi_red+"Error: GNU-compatible 'time' not found; please install it."+ansi_end+'\n')
		exit(1)
	cmake_args = []
	make_args = []
	making_lasercake = True
	running_tests = True
	for arg in sys.argv[1:]:
		m_cxx = re.search(r'^(?:CC|CXX)=(.*)$', arg)
		m_cxxflags = re.search(r'^(?:C|CXX)FLAGS=(.*)$', arg)
		if arg in ['-?', '-h', '--help', '-help']:
			say(
				"""Usage:\n./dev-build.py\n\t[CXX=g++-4.6|clang++|..]\n\t[CXXFLAGS='-Os -w']\n\t[args for cmake]\n\nCompiles in a build dir that depends on the flags you give,\nthen runs tests.\n"""
			   )
			sys.exit()
		elif arg == 'no-test':
			running_tests = False
		elif m_cxx:
			cmake_args.append('-DCMAKE_CXX_COMPILER='+m_cxx.group(1))
		elif m_cxxflags:
			cmake_args.append('-DCMAKE_CXX_FLAGS='+m_cxxflags.group(1))
		else:
			cmake_args.append(arg)
		if arg == '-DBUILD_SELF_TESTS=OFF':
			running_tests = False
	# Make it more likely the directories are named non-conflictingly
	# even though we delete some special characters from their paths.
	hash_for_this_config = hash_list(cmake_args)
	name_for_this_config = hash_for_this_config[:10] + '_' + (','.join([re.sub(r'[^-+.a-zA-Z0-9]', r'_', arg) for arg in cmake_args]) or 'default')
	build_dir = 'build/'+name_for_this_config
	try: os.remove('CMakeCache.txt')
	except OSError: pass
	try:
		os.makedirs(build_dir)
	except OSError:
		say(ansi_green+'This build dir already exists!'+ansi_end+'\n')
		cmake_args = []
	to_call_cmake = ['cmake', '../../'] + cmake_args
	say_we_are_calling('cd '+build_dir+'; '+'   '.join(to_call_cmake))
	say(ansi_cyan+'''  (^^ not escaped properly in these info messages - doin' it right in python)'''+ansi_end+'\n')
	os.chdir(build_dir)
	subprocess.check_call(to_call_cmake)
	if time_has_f:
		time_args = ['time', '-f', (ansi_magenta+'`make` took %E'+ansi_end)]
	else:
		time_args = ['time']
	to_call_make = time_args + ['make'] + make_flags + make_args
	say_we_are_calling('   '.join(to_call_make))
	make_status = subprocess.call(to_call_make)
	if not time_has_f:
		say(ansi_magenta+'  (time to run `make`).'+ansi_end+'\n')
	# How to print the compiler & flags here? Can we get it from cmake
	# somehow? Add to the cmakelists to write those?
	# but with the -l's and all... hmm.
	if make_status != 0:
		# skip make_flags (-jN) to get just one file's error messages
		# not mixed up with the others
		to_call_make_again = ['make'] + make_args
		say_we_are_calling('   '.join(to_call_make_again))
		subprocess.call(to_call_make_again)
		say(ansi_red+'build failed'+ansi_end+'\n')
		exit(1)
	is_windows_exe = False
	if os.access('lasercake', os.F_OK):
		exe_name = 'lasercake'
	if os.access('lasercake.exe', os.F_OK):
		exe_name = 'lasercake.exe'
		is_windows_exe = True
	if exe_name == None:
		say(ansi_red+"couldn't find lasercake binary?!"+ansi_end+'\n')
		exit(1)
	# poor estimate that assumes people only cross-compile
	# if it involves Windows:
	is_cross_compiling = (is_windows_exe != (os.name == 'nt'))
	if making_lasercake:
		say(ansi_green+'and you got:\n./'+build_dir+'/'+exe_name+'\n(etc.)'+ansi_end+'\n')
	if is_cross_compiling:
		say(ansi_yellow+'Cross-compiling, so not running tests or copying the binary.'+ansi_end+'\n')
	else:
		if running_tests:
			say(ansi_cyan+'Testing...\n')
			say_we_are_calling('./'+build_dir+'/lasercake --run-self-tests')
			test_status = subprocess.call(['./lasercake', '--run-self-tests'])
			if test_status != 0:
				exit(test_status)
			say(ansi_green+'success')
		else:
			say(ansi_yellow+'NOT RUNNING TESTS')
		if making_lasercake:
			say('; copying '+build_dir+'/lasercake to ./lasercake')
		say(ansi_end+'\n')
		if making_lasercake:
			shutil.copy2(exe_name, '../../'+exe_name)
	exit()

if __name__ == '__main__':
	main()

