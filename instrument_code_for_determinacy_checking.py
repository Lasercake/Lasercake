#!/usr/bin/env python

# Using regexps for parsing C++ is, of course, entirely a hack.

import re, sys, subprocess

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

filenames = ['tiles.hpp', 'main.cpp']

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
excluded_re = re.compile(r'\b(?:world|frame_output_t|gl_all_data)\b|\bQ[A-Z]|\bLasercake[A-Z]|function')
def get_arg_names(argstr):
	#return re.findall(argname_re, argstr+',')
	result = []
	for m in re.finditer(argname_re, argstr+','):
		#print(m.group(1), m.group(2), re.search(excluded_re, m.group(1)))
		if not re.search(excluded_re, m.group(1)):
			result.append(m.group(2))
	return result

def augment_functions(m):
	if m.group(1) in {'if', 'while', 'switch', 'for', 'do'}:
		return m.group(0)
	fnname = m.group(1)
	argnames = get_arg_names(m.group(2))
	result = m.group(0)
	result += """ {debug_print_ostream() << __func__ << '('; """
	#result += """ {debug_print_ostream() << __FILE__ << ':' << __LINE__ << ':' << __PRETTY_FUNCTION__ << '('; """
	#result += """ {debug_print_ostream() << __PRETTY_FUNCTION__ << "  ("; """
	first = True
	for argname in argnames:
		if first:
			first = False
		else:
			result += """debug_print_ostream() << ", "; """
		result += """debug_print_val_deterministically("""+argname+"); "
	#result += r"""debug_print_ostream() << ")\n";}"""
	result += r"""debug_print_ostream() << "): " << __PRETTY_FUNCTION__ << '\n';}"""
	return result

filecontents_final = {}
for filename in filenames:
	filecontents_final[filename] = re.sub(find_functions,
			augment_functions, filecontents_initial[filename])

for filename in filenames:
	with open(filename, 'w') as f:
		f.write(filecontents_final[filename])

ch = 'config.hpp'
with open(ch, 'r') as f:
	config_contents = f.read()
with open(ch, 'w') as f:
	f.write(re.sub('#if DEBUG_PRINT_DETERMINISTICALLY', '#if 1', config_contents))

