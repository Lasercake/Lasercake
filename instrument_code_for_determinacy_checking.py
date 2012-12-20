#!/usr/bin/env python

# Using regexps for parsing C++ is, of course, entirely a hack.
# Configuration that you might want to change is in this file,
# below the help message and above the trickier code.

import re, os, sys, subprocess, glob

if len(sys.argv) < 2 or sys.argv[1] in set(['-h','-?','-help','--help']) \
		or sys.argv[1] not in set(['instrument', 'restore']):
	print("""
Usage: %s [instrument|restore]
  'instrument' adds/updates instrumentation; 'restore' deletes it.

Instrumentation is just additions to Lasercake code that send
debug message info to stdout upon entering most Lasercake functions.

It can be useful combined with -Q or such to help debug whether two
different compilations or runs of Lasercake that *should* be doing
the exact same thing in fact *are* doing the exact same thing.

If there's a problem with buggy optimizers, this might be unhelpful,
because the instrumentation will quite likely change what the compiler's
optimizer does.
""" % sys.argv[0])
	sys.exit(0)

do_restore = True
do_instrument = (sys.argv[1] == 'instrument')

# Configuration that you might want to change:

# Instrument functions in these files:
# (Note: files not currently instrumented might not easily work
# to instrument, because of regexp hacks doing the wrong thing
# or argument types that can't easily be serialized.  To fix the
# latter, add functions for your types similar to
#     std::ostream& operator<<(std::ostream&, type)
# , or put the troublesome argument's type name [in the form it's
# used textually] in excluded_re below, or add an overload in
# debug_print_deterministically.hpp.)
filenames = glob.glob('*.cpp')

# Any function argument type strings (as written) that contain
# anything matching this regexp are omitted (not attempted to
# be written to output).  This can be useful for large or
# impossible-to-output data (though various tricky things *can*
# be done for certain data; see debug_print_deterministically.hpp).
excluded_re = re.compile(r"""
  \b(?:
    world|frame_output_t|gl_all_data|gl_collection|gl_call_data
    |state_t|tile_physics_state_t|volume_calipers|active_fluids_t
      |water_groups_by_location_t|persistent_water_group_info
      |groupable_water_volume_calipers_t|persistent_water_groups_t
    |objects_map|object_shapes_t
  )\b
  |\bQ[A-Z]|\bLasercake[A-Z]
  |function|_map\b|_set\b|\bset\b|\bmap\b
  |collision_detector|priority_queue|borrowed_bitset
  """, re.VERBOSE)

# Avoid these specific functions for speed reasons.
# (Alternately, we could put e.g. /*noinstrument*/ immediately before
# the function's begin curly brace and that would also prevent this code
# from instrumenting that function.)
function_names_to_skip_re = re.compile(r"""
  \b(
    in_old_box|compute_tile_color|collidable_tile|prepare_tile
    |cast_vector3_to_float|cast_vector3_to_double|look_here
    |tile_manhattan_distance_to_tile_bounding_box
    |do_tile
  )\b
  """, re.VERBOSE)

# The code below is closer to black magic, though it's somewhat commented.
# If you can tweak the regexps or output, for your gain, without breaking
# anything that currently works (instrument and recover on all the files
# in the default value of 'filenames', and as much deterministicness of
# Lasercake output as we can get), then go ahead!

find_functions_re = re.compile(
	r"""\b(\w+)  #function name
	    \(       #begin parenthesis
	    ([^()]*) #arguments
	    \)       #end parenthesis
	    \s*(?:const\s*)?  #filler matter
	    (?::[^;]*?[)\s])? # constructor filler matter, ending with ) or
	                      # space right before the function begin curly
	                      # brace.
	                      # Semicolons are excluded as a hack to keep the
			      # ?: operator from occasionally looking like a
			      # constructor definition e.g. non-function
			      # result_type(*i) in:
			      #   i ? result_type(*i) : result_type();
	    {        #begin function body
	 """,
	re.VERBOSE | re.DOTALL)

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

# These deal strangely with newlines/tabs/etc currently:
escape_string_for_C_re = re.compile(r"""(["\\])""")
collapse_whitespace_re = re.compile(r"""\s+""")
def escape_string_for_C(string):
	return re.sub(escape_string_for_C_re, r'\\\1',
			re.sub(collapse_whitespace_re, r' ', string))
def make_string_for_C(string):
	return '"' + escape_string_for_C(string) + '"'
de_curly_re = re.compile(r'''\s+{$''')

# These are placed directly into a regex; luckily they
# don't contain any regex special characters:
begin_debug_instrument_str = " {DEBUG_INSTRUMENT_BEGIN;"
end_debug_instrument_str = "DEBUG_INSTRUMENT_END;}"
# The regex that includes those lucky strings above:
remove_instruments_re = re.compile(
	begin_debug_instrument_str+'.*?'+end_debug_instrument_str)

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
	result += begin_debug_instrument_str
	result += (""" debug_print_ostream() << "%s("; """ % (escape_string_for_C(fnname)))
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
	result += r"""debug_print_ostream() << "): %s:" BOOST_PP_STRINGIZE(__LINE__) ": %s\n";""" % \
			(escape_string_for_C(filename),
			escape_string_for_C(fnfullish))
	result += end_debug_instrument_str
	return result

filecontents_clean = {}
filecontents_instrumented = {}
filecontents_final = {}
for filename in filenames:
	cont = filecontents_initial[filename]
	if do_restore:
		cont = filecontents_clean[filename] = re.sub(remove_instruments_re, '', cont)
	if do_instrument:
		cont = filecontents_instrumented[filename] = re.sub(
			find_functions_re,
			lambda m: augment_functions(filename, m),
			cont)
	filecontents_final[filename] = cont

for filename in filenames:
	if filecontents_final[filename] != filecontents_initial[filename]:
		with open(filename, 'w') as f:
			f.write(filecontents_final[filename])

ch = 'config.hpp'
with open(ch, 'r') as f:
	config_contents_initial = f.read()

cont = config_contents_initial
if do_restore:
	cont = re.sub('#if 1\|\|DEBUG_PRINT_DETERMINISTICALLY', '#if DEBUG_PRINT_DETERMINISTICALLY', cont)
if do_instrument:
	cont = re.sub('#if DEBUG_PRINT_DETERMINISTICALLY', '#if 1||DEBUG_PRINT_DETERMINISTICALLY', cont)
config_contents_final = cont

if config_contents_final != config_contents_initial:
	with open(ch, 'w') as f:
		f.write(config_contents_final)

