CC=g++
OPTFLAGS=-O3
CXXFLAGS=-std=gnu++0x -ggdb -Wall -Wextra -O0 -fmax-errors=15 -I/usr/include/SDL/ -lSDL -lGL -lGLU

ODIR=output
ODIR_OPTIMIZED=output/optimized

SOURCES = $(wildcard *.cpp)
OBJ = $(patsubst %,$(ODIR)/%,$(SOURCES:.cpp=.o))
OBJ_OPTIMIZED = $(patsubst %,$(ODIR_OPTIMIZED)/%,$(SOURCES:.cpp=.o))

$(ODIR)/%.d: %.cpp
	@set -e; mkdir -p $(ODIR); rm -f $@; \
	$(CC) -MM $(CPPFLAGS) $(CXXFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,$(ODIR)/\1.o $(ODIR_OPTIMIZED)/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

lasercake: $(OBJ)
	$(CC) -o $@ $^ $(CXXFLAGS)

lasercake-optimized: $(OBJ_OPTIMIZED)
	$(CC) -o $@ $^ $(CXXFLAGS) $(OPTFLAGS)

include $(OBJ:.o=.d)


$(ODIR)/%.o: %.cpp
	@mkdir -p $(ODIR)
	$(CC) -c -o $@ $< $(CXXFLAGS)

$(ODIR_OPTIMIZED)/%.o: %.cpp
	@mkdir -p $(ODIR_OPTIMIZED)
	$(CC) -c -o $@ $< $(CXXFLAGS) $(OPTFLAGS)

.PHONY :
	clean

clean:
	rm -rf output lasercake lasercake-optimized

