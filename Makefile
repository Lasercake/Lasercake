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
LINK_FLAGS=$(shell sdl-config --libs) -lGL -lGLU -lrt $(GENERAL_FLAGS) $(LDFLAGS)


# The rest of this file is pretty boring.

ODIR=output
ODIR_DEPS=output/deps
ODIR_OPT=output/optimized
ODIR_UNOPT=output/unoptimized
ODIR_ASREV=output/assert-everything
ODIR_CLANG=output/clang
ODIR_GCC45=output/gcc45
ODIR_GCC46=output/gcc46

SOURCES = $(wildcard *.cpp)
DEPS      = $(patsubst %,$(ODIR_DEPS)/%,$(SOURCES:.cpp=.makedeps))
OBJ_OPT   = $(patsubst %,$(ODIR_OPT)/%,$(SOURCES:.cpp=.o))
OBJ_ASREV = $(patsubst %,$(ODIR_ASREV)/%,$(SOURCES:.cpp=.o))
OBJ_UNOPT = $(patsubst %,$(ODIR_UNOPT)/%,$(SOURCES:.cpp=.o))
OBJ_CLANG = $(patsubst %,$(ODIR_CLANG)/%,$(SOURCES:.cpp=.o))
OBJ_GCC45 = $(patsubst %,$(ODIR_GCC45)/%,$(SOURCES:.cpp=.o))
OBJ_GCC46 = $(patsubst %,$(ODIR_GCC46)/%,$(SOURCES:.cpp=.o))

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

lasercake-gcc45: $(OBJ_GCC45)
	$(GCC45) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

lasercake-gcc46: $(OBJ_GCC46)
	$(GCC46) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

include $(DEPS)


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

$(ODIR_GCC45)/%.o: %.cpp
	@mkdir -p $(ODIR_GCC45)
	$(GCC45) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_GCC46)/%.o: %.cpp
	@mkdir -p $(ODIR_GCC46)
	$(GCC46) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

.PHONY :
	clean

clean:
	rm -rf output lasercake lasercake-debug lasercake-optimized \
		lasercake-clang lasercake-gcc45 lasercake-assert-everything

