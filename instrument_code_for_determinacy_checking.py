#!/usr/bin/env python

# Using regexps for parsing C++ is, of course, entirely a hack.

import re, sys, subprocess, glob

if len(sys.argv) < 1 or (sys.argv[1] != "for-real"):
	print("Don't do this in a modified repo: it's dangerous! Also, read the code.")
	sys.exit(1)

find_functions = re.compile(
	r"""\b(\w+)  #function name
	    \(       #begin parenthesis
	    ([^()]*) #arguments
	    \)       #end parenthesis
	    \s*(?:const\s*)?  #filler matter
	    (?::.*?[)\s])? # constructor filler matter, ending with ) or space
	                   # right before the function begin curly brace
	    {        #begin function body
	 """,
	re.VERBOSE | re.DOTALL)
	    # (?:(?:[^()]|\([^()]*\)))     #constructor filler matter

filenames = glob.glob('*.cpp')

subprocess.check_call(['git', 'checkout'] + filenames)

filecontents_initial = {}
for filename in filenames:
	with open(filename, 'r') as f:
		filecontents_initial[filename] = f.read()

argname_re = re.compile(r"""
		(.*?[^[<(])  #type
		(\b\w+)      #arg name
		(\s*=[^,]+)? #default argument value
		,            #comma between arguments (or for hack at end)
		""",
		re.VERBOSE | re.DOTALL)
excluded_re = re.compile(r'\b(?:world|frame_output_t|gl_all_data|gl_collection|gl_call_data|state_t|tile_physics_state_t|volume_calipers|active_fluids_t|water_groups_by_location_t|persistent_water_group_info|groupable_water_volume_calipers_t|persistent_water_groups_t|objects_map|object_shapes_t)\b|\bQ[A-Z]|\bLasercake[A-Z]|function|_map\b|_set\b|\bset\b|collision_detector|priority_queue')
def get_arg_names(argstr):
	#return re.findall(argname_re, argstr+',')
	result = []
	for m in re.finditer(argname_re, argstr+','):
		#print(m.group(1), m.group(2), re.search(excluded_re, m.group(1)))
		if not re.search(excluded_re, m.group(1)):
			result.append(m.group(2))
	return result

# Give up on parameter packs / vararg functions
# rather than try hard to implement sensible things for uncommon functions.
functions_to_give_up_on_re = re.compile(r"\.\.\.")
# Avoid these specific functions for speed reasons.
function_names_to_skip_re = re.compile(r"\b(in_old_box|compute_tile_color|collidable_tile|prepare_tile|cast_vector3_to_float|cast_vector3_to_double|look_here|tile_manhattan_distance_to_tile_bounding_box)\b")

# These deal strangely with newlines/tabs/etc currently:
escape_string_for_C_re = re.compile(r"""(["\\])""")
collapse_whitespace_re = re.compile(r"""\s+""")
def escape_string_for_C(string):
	return re.sub(escape_string_for_C_re, r'\\\1',
			re.sub(collapse_whitespace_re, r' ', string))
def make_string_for_C(string):
	return '"' + escape_string_for_C(string) + '"'
de_curly_re = re.compile(r'''\s+{$''')

# TODO find a way to print 'this', only for member functions?
def augment_functions(filename, m):
	if m.group(1) in set(['if', 'while', 'switch', 'for', 'do', 'catch',
	                      'BOOST_SCOPE_EXIT']):
		return m.group(0)
	if re.search(functions_to_give_up_on_re, m.group(0)):
		return m.group(0)
	if re.search(function_names_to_skip_re, m.group(1)):
		return m.group(0)
	# This file is mostly time-critical functions:
	if filename == 'the_decomposition_of_the_world_into_blocks.cpp' \
			and m.group(1) != 'ensure_realization_impl':
		return m.group(0)
	fnname = m.group(1)
	argnames = get_arg_names(m.group(2))
	result = m.group(0)
	result += (""" {debug_print_ostream() << "%s("; """ % (escape_string_for_C(fnname)))
	#result += """ {debug_print_ostream() << __func__ << '('; """
	#result += """ {debug_print_ostream() << __FILE__ << ':' << __LINE__ << ':' << __PRETTY_FUNCTION__ << '('; """
	#result += """ {debug_print_ostream() << __PRETTY_FUNCTION__ << "  ("; """
	first = True
	for argname in argnames:
		if first:
			first = False
		else:
			result += """debug_print_ostream() << ", "; """
		result += """debug_print_val_deterministically("""+argname+"); "
	fnfullish = re.sub(de_curly_re, '', m.group(0))
	#result += r"""debug_print_ostream() << ")\n";}"""
	#result += r"""debug_print_ostream() << "): " << __PRETTY_FUNCTION__ << '\n';}"""
	# There was a difference between 'long int' and 'long long int' meaning int64_t
	# on two different platforms, so avoid __PRETTY_FUNCTION__.
	# Hopefully __LINE__ is consistent; it'd be better to compute it here.
	# Stringize it at preprocessor-time, anyway, to make it faster at runtime if possible.
	result += r"""debug_print_ostream() << "): %s:" BOOST_PP_STRINGIZE(__LINE__) ": %s\n";}""" % \
			(escape_string_for_C(filename),
			escape_string_for_C(fnfullish))
	return result

filecontents_final = {}
for filename in filenames:
	filecontents_final[filename] = re.sub(
			find_functions,
			lambda m: augment_functions(filename, m),
			filecontents_initial[filename])

for filename in filenames:
	with open(filename, 'w') as f:
		f.write(filecontents_final[filename])

ch = 'config.hpp'
with open(ch, 'r') as f:
	config_contents = f.read()
with open(ch, 'w') as f:
	f.write(re.sub('#if DEBUG_PRINT_DETERMINISTICALLY', '#if 1', config_contents))

