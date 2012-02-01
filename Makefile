# The top of this file is a bit interesting.

GCC=g++
CLANG=clang++

CC=$(GCC)

OPTFLAGS=-O3
UNOPTFLAGS=-O0
GENERAL_FLAGS=-Wall -Wextra -fmax-errors=15 $(CFLAGS) $(CXXFLAGS)
COMPILE_FLAGS=-std=gnu++0x -I/usr/include/SDL/ $(CPPFLAGS) $(GENERAL_FLAGS)
LINK_FLAGS=-lSDL -lGL -lGLU -lrt $(GENERAL_FLAGS) $(LDFLAGS)


# The rest of this file is pretty boring.

ODIR=output
ODIR_DEPS=output/deps
ODIR_OPT=output/optimized
ODIR_UNOPT=output/unoptimized
ODIR_CLANG=output/clang

SOURCES = $(wildcard *.cpp)
DEPS      = $(patsubst %,$(ODIR_DEPS)/%,$(SOURCES:.cpp=.makedeps))
OBJ_OPT   = $(patsubst %,$(ODIR_OPT)/%,$(SOURCES:.cpp=.o))
OBJ_UNOPT = $(patsubst %,$(ODIR_UNOPT)/%,$(SOURCES:.cpp=.o))
OBJ_CLANG = $(patsubst %,$(ODIR_CLANG)/%,$(SOURCES:.cpp=.o))

$(ODIR_DEPS)/%.makedeps: %.cpp
	@set -e; mkdir -p $(ODIR_DEPS); rm -f $@; \
	$(CC) -MM $(COMPILE_FLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,$(ODIR_OPT)/\1.o $(ODIR_UNOPT)/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

lasercake: $(OBJ_OPT)
	$(CC) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

lasercake-debug: $(OBJ_UNOPT)
	$(CC) -o $@ $^ $(UNOPTFLAGS) $(LINK_FLAGS)

lasercake-clang: $(OBJ_CLANG)
	$(CLANG) -o $@ $^ $(OPTFLAGS) $(LINK_FLAGS)

include $(DEPS)


$(ODIR_OPT)/%.o: %.cpp
	@mkdir -p $(ODIR_OPT)
	$(CC) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_UNOPT)/%.o: %.cpp
	@mkdir -p $(ODIR_UNOPT)
	$(CC) -c -o $@ $< $(UNOPTFLAGS) $(COMPILE_FLAGS)

$(ODIR_CLANG)/%.o: %.cpp
	@mkdir -p $(ODIR_CLANG)
	$(CLANG) -c -o $@ $< $(OPTFLAGS) $(COMPILE_FLAGS)

.PHONY :
	clean

clean:
	rm -rf output lasercake lasercake-debug lasercake-optimized

