

Mighty principles
-----------------

Things the player doesn't start shouldn't take any processing time.
Everything should move continuously unless it has a reason to be tile-aligned.


Petty principles
----------------

The axis-aligned terrain (water mechanics...) simulates without regarding
mobile objects. Mobile objects move on their own and mess with the terrain
or actively get messed with by it.

`int` denotes a Very Small Number (e.g. coordinate numbers 0-2). Anything
of real magnitude should be typedef'd, and the base type should have its
bit width named (e.g. `int32_t`, or `lint32_t` if you want it to be
overflow-checkable, which you probably do).

Anything that blocks you after it hits you must collision-detect with your
'personal space shape'. Anything that doesn't block you after it hits you
(either it ceases to exist or just passes through you with an effect, or...)
is free to use your detailed shape. ("You" and "your" refer to any mobile
object). This is because it's a terrible idea to let complicated shapes get
tangled up with each other. In general, you don't compute collisions between
detail shapes and personal space shapes.

For proper data hiding, any large function of class world that uses private
members of world should be implemented as a call to a non-member function
that takes references only to the private members that it actually changes.


The-assigning-of-code-to-files
------------------------------

* .cpp files should divide *systems*
* Things directly defining the tile-based physics system (fluid simulation, etc)
    go in tile_physics.cpp
* Things directly defining the objects-moving system go in object_motion.cpp
* Everything that makes reference to the worldblock system goes in
    the_decomposition_of_the_world_into_blocks.cpp (It's nice that the
    worldblock system can be hidden behind a relatively simple set of
    abstractions).
* data_structures/ are data structures and algorithms that are are not
    specific to the specific world that Lasercake envisions.
* specific_object_types.cpp/hpp is currently a hodgepodge of implementations
    of leaf object types; I have no attachment to that fact.
* disorganized_stuff.cpp houses the rest of the stuff that I haven't
    come up with a good way to categorize.


C++ conventions
---------------

Anything declared+defined in a single .cpp file and not used elsewhere should be enclosed
in an anonymous namespace, to indicate this to the reader and the compiler.
The begin/end comments saying "anonymous" are encouraged but not required.

    namespace /* anonymous */ {
    code (not indented) {
    }
    more code;
    } /* end anonymous namespace */

If a header must refer to something that logically goes only in a .cpp file
(e.g. so a class can have a private member that's a class defined/used in only
one implementation file, or to allow something to be inlined)
then that file's code, if the file is that_file.cpp, should go in
`namespace that_file_impl {}` and the _impl suffix will indicate that no one
else should try to use that code.  "that_file.cpp" may
`using namespace that_file_impl;` if it wishes. In this case, if it wishes,
"that_file.cpp" may use namespace that_file_impl in place of anonymous
namespaces for its convenience.

Also, as a debugging hack, gl_data_preparation.cpp may use things in _impl
namespaces in its drawing code in order to draw implementation-detail
states-of-being.


Conditional compilation (e.g. #ifdef) should be avoided where practical,
because it means the less commonly tested possibility might not even
*compile*.  If a compile-time conditional is necessary at all, use
if(BOOLEAN_CONSTANT) where practical; the compiler will optimize this out.


Formatting conventions
----------------------

    curly braces {
      two space indentation;
      with creative indentation for very extremely long
        and very uniform
        and more uniform
        and sooo uniform
      lines, so that
            things
            like
            x y and z
            line up better;
      there is no concerted attempt to limit the length of lines to anything in particular;
      // But comments shall be wrapped at less than eighty characters, yes
      // they will!
      and code lines that are more than around a hundred characters are likely
          to get wrapped to eighty characters by anyone; also, only leave them
          at greater than 80 if it is more readable to have it be a single
          line than to split it onto two.
    };


Various names look like
-----------------------

    local_variable
    private_member_variable_, private_member_function_
    public_member
    function_name, struct_name, constant_quantity
    IDENTIFIER_CONSTANT (e.g. enum values. Not an amount of something,
                          just a number that represents something)
    TemplateArgument
    typedefed_thing or typedefed_thing_t (the latter if it'd be nice to have
                                           variables named "typedefed_thing")
    ClassDerivedFromQt (but our made-up method names are still method_name)

    GLOBAL_var_name: but global variables are forbidden.  "Global" means in
        any namespace, or function-static or class-static. "Variable" means
        it can vary; constants and functions can't change so they are fine.
        Furthermore, any part of the code except the UI is also forbidden from
        using global state via libraries that have global state, and from
        doing any I/O except writing debug messages to std::cerr (TODO: do
        something with the debug messages too).


Pointer and reference types put the * or & next to the type, not the name
(after all a `foo*` is a pointer, not a foo):

    foo* a; bar& b; baz const& c;

Variables are never declared together (`foo* a, *b;`) to avoid the way that
C's syntax makes the above convention strange.  Don't do it anywhere,
not even for non-pointer non-reference types.

Variables, even local variables, should be marked "const" wherever possible.
(We fail at this sometimes. Is there a good way to get the compiler to suggest
'const's?)

