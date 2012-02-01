CC=g++
OPTFLAGS=-O3
UNOPTFLAGS=-O0
CXXFLAGS=-std=gnu++0x -ggdb -Wall -Wextra -fmax-errors=15 -I/usr/include/SDL/ -lSDL -lGL -lGLU -lrt

ODIR=output
ODIR_DEPS=output/deps
ODIR_OPT=output/optimized
ODIR_UNOPT=output/unoptimized

SOURCES = $(wildcard *.cpp)
DEPS      = $(patsubst %,$(ODIR_DEPS)/%,$(SOURCES:.cpp=.makedeps))
OBJ_OPT   = $(patsubst %,$(ODIR_OPT)/%,$(SOURCES:.cpp=.o))
OBJ_UNOPT = $(patsubst %,$(ODIR_UNOPT)/%,$(SOURCES:.cpp=.o))

$(ODIR_DEPS)/%.makedeps: %.cpp
	@set -e; mkdir -p $(ODIR_DEPS); rm -f $@; \
	$(CC) -MM $(CPPFLAGS) $(CXXFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,$(ODIR_OPT)/\1.o $(ODIR_UNOPT)/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

lasercake: $(OBJ_OPT)
	$(CC) -o $@ $^ $(OPTFLAGS) $(CXXFLAGS)

lasercake-debug: $(OBJ_UNOPT)
	$(CC) -o $@ $^ $(UNOPTFLAGS) $(CXXFLAGS)

include $(DEPS)


$(ODIR_OPT)/%.o: %.cpp
	@mkdir -p $(ODIR_OPT)
	$(CC) -c -o $@ $< $(OPTFLAGS) $(CXXFLAGS)

$(ODIR_UNOPT)/%.o: %.cpp
	@mkdir -p $(ODIR_UNOPT)
	$(CC) -c -o $@ $< $(UNOPTFLAGS) $(CXXFLAGS)

.PHONY :
	clean

clean:
	rm -rf output lasercake lasercake-debug lasercake-optimized

