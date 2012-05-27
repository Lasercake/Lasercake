# The top of this file is a bit interesting.

GCC=g++
CLANG=clang++
GCC45=g++-4.5
GCC46=g++-4.6

CC=$(GCC)

OPTFLAGS=-O3
UNOPTFLAGS=-O0
GENERAL_FLAGS=-Wall -Wextra -fmax-errors=15 -fstack-protector --param=ssp-buffer-size=4 -D_FORTIFY_SOURCE=2 $(CFLAGS) $(CXXFLAGS)
COMPILE_FLAGS=-std=gnu++0x $(shell sdl-config --cflags) $(CPPFLAGS) $(GENERAL_FLAGS)
LINK_FLAGS_NO_THREADS=$(shell sdl-config --libs) -lGL -lGLU -lrt $(GENERAL_FLAGS) $(LDFLAGS)
LINK_FLAGS=-lboost_thread $(LINK_FLAGS_NO_THREADS)


# The rest of this file is pretty boring.

ODIR=output
ODIR_DEPS=output/deps
ODIR_TESTS=output/tests
ODIR_OPT=output/optimized
ODIR_UNOPT=output/unoptimized
ODIR_ASREV=output/assert-everything
ODIR_CLANG=output/clang
ODIR_CLLIB=output/clang-libcxx
ODIR_GCC45=output/gcc45
ODIR_GCC46=output/gcc46
ODIR_NOTHR=output/no-threads

SOURCES = $(wildcard *.cpp)
TEST_SOURCES = $(wildcard tests/*.cpp)
DEPS      = $(patsubst %,$(ODIR_DEPS)/%,$(SOURCES:.cpp=.makedeps))
OBJ_OPT   = $(patsubst %,$(ODIR_OPT)/%,$(SOURCES:.cpp=.o))
OBJ_ASREV = $(patsubst %,$(ODIR_ASREV)/%,$(SOURCES:.cpp=.o))
OBJ_UNOPT = $(patsubst %,$(ODIR_UNOPT)/%,$(SOURCES:.cpp=.o))
OBJ_CLANG = $(patsubst %,$(ODIR_CLANG)/%,$(SOURCES:.cpp=.o))
OBJ_CLLIB = $(patsubst %,$(ODIR_CLLIB)/%,$(SOURCES:.cpp=.o))
OBJ_GCC45 = $(patsubst %,$(ODIR_GCC45)/%,$(SOURCES:.cpp=.o))
OBJ_GCC46 = $(patsubst %,$(ODIR_GCC46)/%,$(SOURCES:.cpp=.o))
OBJ_NOTHR = $(patsubst %,$(ODIR_NOTHR)/%,$(SOURCES:.cpp=.o))
OBJ_TESTS = $(patsubst tests/%,$(ODIR_TESTS)/%,$(TEST_SOURCES:.cpp=.o))

$(ODIR_DEPS)/%.makedeps: %.cpp
	@set -e; mkdir -p $(ODIR_DEPS); rm -f $@; \
	$(CC) -MM $(COMPILE_FLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,$(ODIR_OPT)/\1.o $(ODIR_UNOPT)/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

# The top target in the file is the default one invoked by 'make'.

# When you add a target, remember to add it to the 'clean' target below
# and to the '.gitignore' file.

lasercake: $(OBJ_OPT)
	$(CC) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

lasercake-assert-everything: $(OBJ_ASREV)
	$(CC) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

lasercake-debug: $(OBJ_UNOPT)
	$(CC) -o $@ $^ $(UNOPTFLAGS) $(LINK_FLAGS)

lasercake-clang: $(OBJ_CLANG)
	$(CLANG) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

lasercake-clang-libcxx: $(OBJ_CLLIB)
	$(CLANG) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS) -stdlib=libc++ -Wl,-lstdc++ \
		#that -Wl was necessary for me but is it right?

lasercake-gcc45: $(OBJ_GCC45)
	$(GCC45) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

lasercake-gcc46: $(OBJ_GCC46)
	$(GCC46) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

lasercake-no-threads: $(OBJ_NOTHR)
	$(CC) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS_NO_THREADS)

lasercake-test-misc: $(ODIR_TESTS)/misc_utils_tests.o $(ODIR_TESTS)/test_main.o
	$(CC) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS) -lboost_unit_test_framework

lasercake-test-concurrent: $(ODIR_TESTS)/concurrency_utils_tests.o $(ODIR_TESTS)/test_main.o
	$(CC) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS) -lboost_unit_test_framework

lasercake-test-bounds-checked-int: $(ODIR_TESTS)/bounds_checked_int_tests.o $(ODIR_TESTS)/test_main.o
	$(CC) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS) -lboost_unit_test_framework

compile: $(OBJ_OPT) $(OBJ_TESTS)

include $(DEPS)

$(ODIR_TESTS)/misc_utils_tests.o: utils.hpp
$(ODIR_TESTS)/concurrency_utils_tests.o: concurrency_utils.hpp utils.hpp
$(ODIR_TESTS)/bounds_checked_int_tests.o: bounds_checked_int.hpp utils.hpp
$(ODIR_TESTS)/%.o: tests/%.cpp
	@mkdir -p $(ODIR_TESTS)
	$(CC) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_OPT)/%.o: %.cpp
	@mkdir -p $(ODIR_OPT)
	$(CC) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_ASREV)/%.o: %.cpp
	@mkdir -p $(ODIR_ASREV)
	$(CC) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS) -DASSERT_EVERYTHING

$(ODIR_UNOPT)/%.o: %.cpp
	@mkdir -p $(ODIR_UNOPT)
	$(CC) -c -o $@ $< $(UNOPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_CLANG)/%.o: %.cpp
	@mkdir -p $(ODIR_CLANG)
	$(CLANG) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_CLLIB)/%.o: %.cpp
	@mkdir -p $(ODIR_CLLIB)
	$(CLANG) -stdlib=libc++ -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_GCC45)/%.o: %.cpp
	@mkdir -p $(ODIR_GCC45)
	$(GCC45) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_GCC46)/%.o: %.cpp
	@mkdir -p $(ODIR_GCC46)
	$(GCC46) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_NOTHR)/%.o: %.cpp
	@mkdir -p $(ODIR_NOTHR)
	$(CC) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS) -DLASERCAKE_NO_THREADS=1

.PHONY :
	clean compile

clean:
	rm -rf output lasercake lasercake-debug lasercake-optimized \
		lasercake-clang lasercake-clang-libcxx lasercake-gcc45 \
		lasercake-gcc46 lasercake-assert-everything \
		lasercake-no-threads lasercake-test-concurrent \
		lasercake-test-bounds-checked-int lasercake-test-misc

